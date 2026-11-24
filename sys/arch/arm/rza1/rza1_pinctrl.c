// SPDX-License-Identifier: BSD-2-Clause
/*
 * Renesas RZ/A1 I/O ports: pin function controller and GPIO.
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

#define	RZA1_NPORTS		12
#define	RZA1_PINS_PER_PORT	16

#define	PORT_P(n)		(0x0000 + 4 * (n))
#define	PORT_PSR(n)		(0x0100 + 4 * (n))
#define	PORT_PPR(n)		(0x0200 + 4 * (n))
#define	PORT_PM(n)		(0x0300 + 4 * (n))
#define	PORT_PMC(n)		(0x0400 + 4 * (n))
#define	PORT_PFC(n)		(0x0500 + 4 * (n))
#define	PORT_PFCE(n)		(0x0600 + 4 * (n))
#define	PORT_PNOT(n)		(0x0700 + 4 * (n))
#define	PORT_PMSR(n)		(0x0800 + 4 * (n))
#define	PORT_PMCSR(n)		(0x0900 + 4 * (n))
#define	PORT_PFCAE(n)		(0x0a00 + 4 * (n))
#define	PORT_PIBC(n)		(0x4000 + 4 * (n))
#define	PORT_PBDC(n)		(0x4100 + 4 * (n))
#define	PORT_PIPC(n)		(0x4200 + 4 * (n))

#define	SETCLR(pin, val)	(__BIT((pin) + 16) | ((val) ? __BIT(pin) : 0))

#define	PINMUX_PIN(x)		((x) & 0xffff)
#define	PINMUX_FUNC(x)		(((x) >> 16) & 0xf)

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "renesas,r7s72100-ports" },
	DEVICE_COMPAT_EOL
};

struct rza1_pinctrl_softc;

struct rza1_gpio_port {
	struct rza1_pinctrl_softc *gp_sc;
	u_int gp_port;
};

struct rza1_gpio_pin {
	struct rza1_gpio_port *pin_port;
	u_int pin_num;
	bool pin_actlo;
};

struct rza1_pinctrl_softc {
	device_t sc_dev;
	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;
	kmutex_t sc_lock;
};

#define	RD2(sc, reg)		bus_space_read_2((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define	WR2(sc, reg, val)	bus_space_write_2((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))
#define	WR4(sc, reg, val)	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))

static int	rza1_pinctrl_match(device_t, cfdata_t, void *);
static void	rza1_pinctrl_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(rza1_pinctrl, sizeof(struct rza1_pinctrl_softc),
    rza1_pinctrl_match, rza1_pinctrl_attach, NULL, NULL);

static void
rza1_setbit(struct rza1_pinctrl_softc *sc, bus_size_t reg, u_int pin, bool on)
{
	uint16_t val;

	val = RD2(sc, reg);
	if (on)
		val |= __BIT(pin);
	else
		val &= ~__BIT(pin);
	WR2(sc, reg, val);
}

static void
rza1_pin_reset(struct rza1_pinctrl_softc *sc, u_int port, u_int pin)
{
	rza1_setbit(sc, PORT_PIBC(port), pin, false);
	rza1_setbit(sc, PORT_PBDC(port), pin, false);
	WR4(sc, PORT_PMSR(port), SETCLR(pin, 1));
	WR4(sc, PORT_PMCSR(port), SETCLR(pin, 0));
	rza1_setbit(sc, PORT_PIPC(port), pin, false);
}

static int
rza1_pinctrl_set_pin(struct rza1_pinctrl_softc *sc, u_int pinmux, int flags,
    bool bidir)
{
	const u_int port = PINMUX_PIN(pinmux) / RZA1_PINS_PER_PORT;
	const u_int pin = PINMUX_PIN(pinmux) % RZA1_PINS_PER_PORT;
	const u_int func = PINMUX_FUNC(pinmux);

	if (port >= RZA1_NPORTS || func < 1 || func > 8)
		return EINVAL;

	const u_int f = func - 1;

	rza1_pin_reset(sc, port, pin);
	rza1_setbit(sc, PORT_PBDC(port), pin, bidir);
	rza1_setbit(sc, PORT_PFC(port), pin, (f & 1) != 0);
	rza1_setbit(sc, PORT_PFCE(port), pin, (f & 2) != 0);
	rza1_setbit(sc, PORT_PFCAE(port), pin, (f & 4) != 0);

	if (flags == -1)
		rza1_setbit(sc, PORT_PIPC(port), pin, true);
	WR4(sc, PORT_PMCSR(port), SETCLR(pin, 1));
	if (flags != -1)
		WR4(sc, PORT_PMSR(port),
		    SETCLR(pin, (flags & GPIO_PIN_INPUT) != 0));

	return 0;
}

static int
rza1_pinctrl_set_config(device_t dev, const void *data, size_t len)
{
	struct rza1_pinctrl_softc * const sc = device_private(dev);
	const u_int *pinmux;
	int phandle, plen, flags, outval, error;
	bool bidir;

	if (len != 4)
		return EINVAL;

	phandle = fdtbus_get_phandle_from_native(be32dec(data));
	pinmux = fdtbus_pinctrl_parse_pinmux(phandle, &plen);
	if (pinmux == NULL)
		return ENXIO;

	flags = fdtbus_pinctrl_parse_input_output(phandle, &outval);
	bidir = of_hasprop(phandle, "bi-directional");

	mutex_enter(&sc->sc_lock);
	for (; plen >= 4; plen -= 4, pinmux++) {
		error = rza1_pinctrl_set_pin(sc, be32toh(*pinmux), flags,
		    bidir);
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

static const struct fdtbus_pinctrl_controller_func rza1_pinctrl_funcs = {
	.set_config = rza1_pinctrl_set_config,
};

static void *
rza1_gpio_port_acquire(struct rza1_gpio_port *gp, const void *data,
    size_t len, int flags)
{
	struct rza1_pinctrl_softc * const sc = gp->gp_sc;
	const u_int *cells = data;
	struct rza1_gpio_pin *pin;
	u_int num, port;

	if (len != 12)
		return NULL;
	num = be32toh(cells[1]);
	if (num >= RZA1_PINS_PER_PORT)
		return NULL;
	port = gp->gp_port;

	pin = kmem_zalloc(sizeof(*pin), KM_SLEEP);
	pin->pin_port = gp;
	pin->pin_num = num;
	pin->pin_actlo = (be32toh(cells[2]) & 1) != 0;

	mutex_enter(&sc->sc_lock);

	WR4(sc, PORT_PMCSR(port), SETCLR(num, 0));
	rza1_setbit(sc, PORT_PIPC(port), num, false);
	rza1_setbit(sc, PORT_PBDC(port), num, false);
	if (flags & GPIO_PIN_OUTPUT) {
		rza1_setbit(sc, PORT_PIBC(port), num, false);
		WR4(sc, PORT_PMSR(port), SETCLR(num, 0));
	} else {
		WR4(sc, PORT_PMSR(port), SETCLR(num, 1));
		rza1_setbit(sc, PORT_PIBC(port), num, true);
	}
	mutex_exit(&sc->sc_lock);

	return pin;
}

static void
rza1_gpio_release(device_t dev, void *priv)
{
	struct rza1_gpio_pin * const pin = priv;
	struct rza1_pinctrl_softc * const sc = pin->pin_port->gp_sc;

	mutex_enter(&sc->sc_lock);
	rza1_pin_reset(sc, pin->pin_port->gp_port, pin->pin_num);
	mutex_exit(&sc->sc_lock);

	kmem_free(pin, sizeof(*pin));
}

static int
rza1_gpio_read(device_t dev, void *priv, bool raw)
{
	struct rza1_gpio_pin * const pin = priv;
	struct rza1_pinctrl_softc * const sc = pin->pin_port->gp_sc;
	int val;

	val = (RD2(sc, PORT_PPR(pin->pin_port->gp_port)) >> pin->pin_num) & 1;
	if (!raw && pin->pin_actlo)
		val = !val;

	return val;
}

static void
rza1_gpio_write(device_t dev, void *priv, int val, bool raw)
{
	struct rza1_gpio_pin * const pin = priv;
	struct rza1_pinctrl_softc * const sc = pin->pin_port->gp_sc;

	if (!raw && pin->pin_actlo)
		val = !val;

	WR4(sc, PORT_PSR(pin->pin_port->gp_port), SETCLR(pin->pin_num, val));
}

static struct rza1_gpio_port *rza1_gpio_ports[RZA1_NPORTS];
static int rza1_gpio_port_phandles[RZA1_NPORTS];

static void *
rza1_gpio_acquire(device_t dev, const void *data, size_t len, int flags)
{
	const u_int *cells = data;
	int xref;

	if (len < 4)
		return NULL;
	xref = fdtbus_get_phandle_from_native(be32toh(cells[0]));

	for (u_int i = 0; i < RZA1_NPORTS; i++) {
		if (rza1_gpio_ports[i] != NULL &&
		    rza1_gpio_port_phandles[i] == xref)
			return rza1_gpio_port_acquire(rza1_gpio_ports[i],
			    data, len, flags);
	}
	return NULL;
}

static const struct fdtbus_gpio_controller_func rza1_gpio_funcs = {
	.acquire = rza1_gpio_acquire,
	.release = rza1_gpio_release,
	.read = rza1_gpio_read,
	.write = rza1_gpio_write,
};

static int
rza1_pinctrl_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
rza1_pinctrl_attach(device_t parent, device_t self, void *aux)
{
	struct rza1_pinctrl_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;
	u_int ngpio = 0, ncfg = 0;
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

	aprint_naive("\n");
	aprint_normal(": RZ/A1 I/O ports\n");

	for (child = OF_child(phandle); child; child = OF_peer(child)) {
		if (of_hasprop(child, "gpio-controller")) {
			uint32_t ranges[4];
			struct rza1_gpio_port *gp;
			u_int port;

			if (OF_getproplen(child, "gpio-ranges") !=
			    sizeof(ranges) ||
			    of_getprop_uint32_array(child, "gpio-ranges",
			    ranges, 4) != 0)
				continue;
			port = ranges[2] / RZA1_PINS_PER_PORT;
			if (port >= RZA1_NPORTS)
				continue;
			gp = kmem_zalloc(sizeof(*gp), KM_SLEEP);
			gp->gp_sc = sc;
			gp->gp_port = port;
			rza1_gpio_ports[port] = gp;
			rza1_gpio_port_phandles[port] = child;
			fdtbus_register_gpio_controller(self, child,
			    &rza1_gpio_funcs);
			ngpio++;
			continue;
		}

		if (of_hasprop(child, "pinmux")) {
			fdtbus_register_pinctrl_config(self, child,
			    &rza1_pinctrl_funcs);
			ncfg++;
		}
		for (sub = OF_child(child); sub; sub = OF_peer(sub)) {
			if (of_hasprop(sub, "pinmux")) {
				fdtbus_register_pinctrl_config(self, sub,
				    &rza1_pinctrl_funcs);
				ncfg++;
			}
		}
	}

	aprint_verbose_dev(self, "%u GPIO ports, %u pin configurations\n",
	    ngpio, ncfg);
}
