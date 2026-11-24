// SPDX-License-Identifier: BSD-2-Clause
/*
 * Renesas RZ/A2 GPIO ports: pin function controller and GPIO.
 */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/gpio.h>
#include <sys/kmem.h>
#include <sys/mutex.h>
#include <sys/systm.h>

#include <dev/fdt/fdtvar.h>

#include <arm/rza2/rza2_var.h>

#define	RZA2_NPORTS		22
#define	RZA2_PINS_PER_PORT	8
#define	RZA2_PORTM		21

#define	PORT_PDR(m)		(0x0000 + 2 * (m))
#define	 PDR_HIZ		0
#define	 PDR_INPUT		2
#define	 PDR_OUTPUT		3
#define	 PDR_MASK		3
#define	PORT_PODR(m)		(0x0040 + (m))
#define	PORT_PIDR(m)		(0x0060 + (m))
#define	PORT_PMR(m)		(0x0080 + (m))
#define	PORT_PFS(m, n)		(0x0200 + 8 * (m) + (n))
#define	 PFS_ISEL		0x40
#define	 PFS_PSEL		0x07
#define	PORT_PWPR		0x02ff
#define	 PWPR_B0WI		0x80
#define	 PWPR_PFSWE		0x40
#define	PORT_PFENET		0x0820
#define	 PFENET_PHYMODE(ch)	__BIT(ch)

#define	PINMUX_PIN(x)		((x) & 0xffff)
#define	PINMUX_FUNC(x)		((x) >> 16)

static const uint8_t rza2_port_pins[RZA2_NPORTS] = {
	0x7f, 0x1f, 0x0f, 0x3f, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0x3f, 0xff, 0xff, 0x7f, 0xff,
	0xff, 0x7f, 0xff, 0x3f, 0x1f, 0x03,
};

#define	RZA2_PORT_INPUT_ONLY(m)	((m) == 5 || (m) == 20)

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "renesas,r7s9210-pinctrl" },
	DEVICE_COMPAT_EOL
};

struct rza2_pinctrl_softc;

struct rza2_gpio_pin {
	struct rza2_pinctrl_softc *pin_sc;
	u_int pin_port;
	u_int pin_num;
	bool pin_actlo;
};

struct rza2_pinctrl_softc {
	device_t sc_dev;
	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;
	kmutex_t sc_lock;
};

#define	RD1(sc, reg)		bus_space_read_1((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define	WR1(sc, reg, val)	bus_space_write_1((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))
#define	RD2(sc, reg)		bus_space_read_2((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define	WR2(sc, reg, val)	bus_space_write_2((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))

static int	rza2_pinctrl_match(device_t, cfdata_t, void *);
static void	rza2_pinctrl_attach(device_t, device_t, void *);

static struct rza2_pinctrl_softc *rza2_pinctrl_sc;

CFATTACH_DECL_NEW(rza2_pinctrl, sizeof(struct rza2_pinctrl_softc),
    rza2_pinctrl_match, rza2_pinctrl_attach, NULL, NULL);

static bool
rza2_pin_valid(u_int port, u_int pin)
{

	return port < RZA2_NPORTS && pin < RZA2_PINS_PER_PORT &&
	    (rza2_port_pins[port] & __BIT(pin)) != 0;
}

static void
rza2_set_pdr(struct rza2_pinctrl_softc *sc, u_int port, u_int pin, u_int dir)
{
	uint16_t val;

	val = RD2(sc, PORT_PDR(port));
	val &= ~(PDR_MASK << (2 * pin));
	val |= dir << (2 * pin);
	WR2(sc, PORT_PDR(port), val);
}

static void
rza2_set_pmr(struct rza2_pinctrl_softc *sc, u_int port, u_int pin, bool on)
{
	uint8_t val;

	val = RD1(sc, PORT_PMR(port));
	if (on)
		val |= __BIT(pin);
	else
		val &= ~__BIT(pin);
	WR1(sc, PORT_PMR(port), val);
}

static int
rza2_pinctrl_set_pin(struct rza2_pinctrl_softc *sc, u_int pinmux, int flags)
{
	const u_int port = PINMUX_PIN(pinmux) / RZA2_PINS_PER_PORT;
	const u_int pin = PINMUX_PIN(pinmux) % RZA2_PINS_PER_PORT;
	const u_int func = PINMUX_FUNC(pinmux);
	const u_int dir = (flags != -1 && (flags & GPIO_PIN_INPUT) != 0) ?
	    PDR_INPUT : PDR_HIZ;

	if (!rza2_pin_valid(port, pin) || func < 1 || func > PFS_PSEL)
		return EINVAL;

	if ((RD1(sc, PORT_PMR(port)) & __BIT(pin)) != 0 &&
	    (RD1(sc, PORT_PFS(port, pin)) & PFS_PSEL) == func) {
		rza2_set_pdr(sc, port, pin, dir);
		return 0;
	}

	rza2_set_pmr(sc, port, pin, false);
	rza2_set_pdr(sc, port, pin, PDR_HIZ);
	WR1(sc, PORT_PWPR, 0);
	WR1(sc, PORT_PWPR, PWPR_PFSWE);
	WR1(sc, PORT_PFS(port, pin), func);
	WR1(sc, PORT_PWPR, 0);
	WR1(sc, PORT_PWPR, PWPR_B0WI);
	rza2_set_pmr(sc, port, pin, true);
	if (dir != PDR_HIZ)
		rza2_set_pdr(sc, port, pin, dir);

	return 0;
}

static int
rza2_pinctrl_set_config(device_t dev, const void *data, size_t len)
{
	struct rza2_pinctrl_softc * const sc = device_private(dev);
	const u_int *pinmux;
	int phandle, plen, flags, outval, error;

	if (len != 4)
		return EINVAL;

	phandle = fdtbus_get_phandle_from_native(be32dec(data));
	pinmux = fdtbus_pinctrl_parse_pinmux(phandle, &plen);
	if (pinmux == NULL)
		return ENXIO;

	flags = fdtbus_pinctrl_parse_input_output(phandle, &outval);

	mutex_enter(&sc->sc_lock);
	for (; plen >= 4; plen -= 4, pinmux++) {
		error = rza2_pinctrl_set_pin(sc, be32toh(*pinmux), flags);
		if (error != 0) {
			mutex_exit(&sc->sc_lock);
			aprint_error_dev(dev, "bad pinmux %#x\n",
			    be32toh(*pinmux));
			return error;
		}
	}
	mutex_exit(&sc->sc_lock);

	return 0;
}

static const struct fdtbus_pinctrl_controller_func rza2_pinctrl_funcs = {
	.set_config = rza2_pinctrl_set_config,
};

int
rza2_pinctrl_set_ether_mii(u_int ch, bool mii)
{
	struct rza2_pinctrl_softc * const sc = rza2_pinctrl_sc;
	uint8_t val, rval;

	if (sc == NULL)
		return ENXIO;
	if (ch > 1)
		return EINVAL;

	mutex_enter(&sc->sc_lock);
	val = RD1(sc, PORT_PFENET);
	if (mii)
		val |= PFENET_PHYMODE(ch);
	else
		val &= ~PFENET_PHYMODE(ch);
	WR1(sc, PORT_PWPR, 0);
	WR1(sc, PORT_PWPR, PWPR_PFSWE);
	WR1(sc, PORT_PFENET, val);
	WR1(sc, PORT_PWPR, 0);
	WR1(sc, PORT_PWPR, PWPR_B0WI);
	rval = RD1(sc, PORT_PFENET);
	mutex_exit(&sc->sc_lock);

	return rval == val ? 0 : EIO;
}

static void *
rza2_gpio_acquire(device_t dev, const void *data, size_t len, int flags)
{
	struct rza2_pinctrl_softc * const sc = device_private(dev);
	const u_int *cells = data;
	struct rza2_gpio_pin *pin;
	u_int idx, port, num;

	if (len != 12)
		return NULL;
	idx = be32toh(cells[1]);
	port = idx / RZA2_PINS_PER_PORT;
	num = idx % RZA2_PINS_PER_PORT;
	if (!rza2_pin_valid(port, num) || port == RZA2_PORTM)
		return NULL;
	if ((flags & GPIO_PIN_OUTPUT) != 0 && RZA2_PORT_INPUT_ONLY(port))
		return NULL;

	pin = kmem_zalloc(sizeof(*pin), KM_SLEEP);
	pin->pin_sc = sc;
	pin->pin_port = port;
	pin->pin_num = num;
	pin->pin_actlo = (be32toh(cells[2]) & 1) != 0;

	mutex_enter(&sc->sc_lock);
	rza2_set_pmr(sc, port, num, false);
	rza2_set_pdr(sc, port, num,
	    (flags & GPIO_PIN_OUTPUT) != 0 ? PDR_OUTPUT : PDR_INPUT);
	mutex_exit(&sc->sc_lock);

	return pin;
}

static void
rza2_gpio_release(device_t dev, void *priv)
{
	struct rza2_gpio_pin * const pin = priv;
	struct rza2_pinctrl_softc * const sc = pin->pin_sc;

	mutex_enter(&sc->sc_lock);
	rza2_set_pdr(sc, pin->pin_port, pin->pin_num, PDR_HIZ);
	mutex_exit(&sc->sc_lock);

	kmem_free(pin, sizeof(*pin));
}

static int
rza2_gpio_read(device_t dev, void *priv, bool raw)
{
	struct rza2_gpio_pin * const pin = priv;
	struct rza2_pinctrl_softc * const sc = pin->pin_sc;
	int val;

	val = (RD1(sc, PORT_PIDR(pin->pin_port)) >> pin->pin_num) & 1;
	if (!raw && pin->pin_actlo)
		val = !val;

	return val;
}

static void
rza2_gpio_write(device_t dev, void *priv, int val, bool raw)
{
	struct rza2_gpio_pin * const pin = priv;
	struct rza2_pinctrl_softc * const sc = pin->pin_sc;
	uint8_t podr;

	if (!raw && pin->pin_actlo)
		val = !val;

	mutex_enter(&sc->sc_lock);
	podr = RD1(sc, PORT_PODR(pin->pin_port));
	if (val)
		podr |= __BIT(pin->pin_num);
	else
		podr &= ~__BIT(pin->pin_num);
	WR1(sc, PORT_PODR(pin->pin_port), podr);
	mutex_exit(&sc->sc_lock);
}

static const struct fdtbus_gpio_controller_func rza2_gpio_funcs = {
	.acquire = rza2_gpio_acquire,
	.release = rza2_gpio_release,
	.read = rza2_gpio_read,
	.write = rza2_gpio_write,
};

static char
rza2_port_char(u_int port)
{
	if (port < 10)
		return '0' + port;
	if (port < 18)
		return 'A' + port - 10;
	return 'J' + port - 18;
}

static void
rza2_gpio_hogs(struct rza2_pinctrl_softc *sc, int phandle)
{
	const u_int *gpios;
	const char *name;
	int child, len;
	uint8_t podr;

	for (child = OF_child(phandle); child; child = OF_peer(child)) {
		if (!of_hasprop(child, "gpio-hog"))
			continue;
		name = fdtbus_get_string(child, "line-name");
		if (name == NULL)
			name = fdtbus_get_string(child, "name");
		const bool input = of_hasprop(child, "input");
		const bool high = of_hasprop(child, "output-high");
		const bool low = of_hasprop(child, "output-low");
		if (input + high + low != 1) {
			aprint_error_dev(sc->sc_dev, "%s: no single direction\n",
			    name);
			continue;
		}

		gpios = fdtbus_get_prop(child, "gpios", &len);
		for (; gpios != NULL && len >= 8; len -= 8, gpios += 2) {
			const u_int idx = be32toh(gpios[0]);
			const bool actlo = (be32toh(gpios[1]) & 1) != 0;
			const u_int port = idx / RZA2_PINS_PER_PORT;
			const u_int num = idx % RZA2_PINS_PER_PORT;
			const bool val = high != actlo;

			if (!rza2_pin_valid(port, num) || port == RZA2_PORTM ||
			    (!input && RZA2_PORT_INPUT_ONLY(port))) {
				aprint_error_dev(sc->sc_dev,
				    "%s: bad pin %u\n", name, idx);
				continue;
			}

			mutex_enter(&sc->sc_lock);
			rza2_set_pmr(sc, port, num, false);
			if (input) {
				rza2_set_pdr(sc, port, num, PDR_INPUT);
			} else {
				podr = RD1(sc, PORT_PODR(port));
				if (val)
					podr |= __BIT(num);
				else
					podr &= ~__BIT(num);
				WR1(sc, PORT_PODR(port), podr);
				rza2_set_pdr(sc, port, num, PDR_OUTPUT);
			}
			mutex_exit(&sc->sc_lock);

			aprint_normal_dev(sc->sc_dev, "%s: P%c_%u %s\n", name,
			    rza2_port_char(port), num,
			    input ? "input" : val ? "high" : "low");
		}
	}
}

static int
rza2_pinctrl_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
rza2_pinctrl_attach(device_t parent, device_t self, void *aux)
{
	struct rza2_pinctrl_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;
	u_int ncfg = 0;
	int child, sub;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_VM);
	if (rza2_pinctrl_sc == NULL)
		rza2_pinctrl_sc = sc;

	aprint_naive("\n");
	aprint_normal(": RZ/A2 GPIO ports\n");

	for (child = OF_child(phandle); child; child = OF_peer(child)) {
		if (of_hasprop(child, "pinmux")) {
			fdtbus_register_pinctrl_config(self, child,
			    &rza2_pinctrl_funcs);
			ncfg++;
		}
		for (sub = OF_child(child); sub; sub = OF_peer(sub)) {
			if (of_hasprop(sub, "pinmux")) {
				fdtbus_register_pinctrl_config(self, sub,
				    &rza2_pinctrl_funcs);
				ncfg++;
			}
		}
	}

	if (of_hasprop(phandle, "gpio-controller")) {
		fdtbus_register_gpio_controller(self, phandle,
		    &rza2_gpio_funcs);
		rza2_gpio_hogs(sc, phandle);
	}

	aprint_verbose_dev(self, "%u pin configurations\n", ncfg);
}
