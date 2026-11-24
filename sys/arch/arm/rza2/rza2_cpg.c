// SPDX-License-Identifier: BSD-2-Clause
/*
 * Renesas RZ/A2 clock pulse generator (CPG) and module stop bits
 * ("renesas,r7s9210-cpg-mssr").
 */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/mutex.h>
#include <sys/systm.h>

#include <dev/clk/clk_backend.h>
#include <dev/fdt/fdtvar.h>

#include <arm/rza2/rza2_reg.h>

#define	CPG_CORE	0
#define	CPG_MOD		1

#define	CLK_I		0	/* CPU */
#define	CLK_G		1	/* image processing, always 2 x B */
#define	CLK_B		2	/* internal bus */
#define	CLK_P1		3	/* peripheral 1 */
#define	CLK_P1C		4	/* P1 without spread spectrum modulation */
#define	CLK_P0		5	/* peripheral 0, PLL / 32 */
#define	CLK_NCORE	6
#define	CLK_NONE	0xff	/* clocked through SCLKSEL; rate unknown */

static const char * const rza2_cpg_core_names[CLK_NCORE] = {
	[CLK_I] = "i", [CLK_G] = "g", [CLK_B] = "b",
	[CLK_P1] = "p1", [CLK_P1C] = "p1c", [CLK_P0] = "p0",
};

struct rza2_cpg_mod {
	const char	*name;
	uint8_t		id;
	uint8_t		parent;
	uint8_t		also;
	uint8_t		stbreq;
	uint8_t		stbreq_bits;
	bool		keep;
};

#define	MOD(n, i, p)		{ .name = (n), .id = (i), .parent = (p) }
#define	MOD_REQ(n, i, p, r, b)	{ .name = (n), .id = (i), .parent = (p), \
				  .stbreq = (r), .stbreq_bits = (b) }
#define	MOD_REQ2(n, i, p, a, r, b) \
				{ .name = (n), .id = (i), .parent = (p), \
				  .also = (a), .stbreq = (r), .stbreq_bits = (b) }

static const struct rza2_cpg_mod rza2_cpg_mods[] = {
	MOD("ostm0", 36, CLK_P1C),
	{ .name = "ostm1", .id = 35, .parent = CLK_P1C, .keep = true },
	MOD("ostm2", 34, CLK_P1C),
	MOD("mtu3", 33, CLK_P1C),
	MOD("canfd", 32, CLK_P1C),
	MOD("gpt", 30, CLK_P1C),

	MOD("scif0", 47, CLK_P1C),
	MOD("scif1", 46, CLK_P1C),
	MOD("scif2", 45, CLK_P1C),
	MOD("scif3", 44, CLK_P1C),
	MOD("scif4", 43, CLK_P1C),
	MOD("sci0", 42, CLK_P1C),
	MOD("sci1", 41, CLK_P1C),
	MOD("irda", 40, CLK_P1C),

	MOD("adc", 57, CLK_P1),
	MOD_REQ("ceu", 56, CLK_B, 1, __BIT(0)),
	MOD("rtc0", 53, CLK_P0),
	MOD("rtc1", 52, CLK_P0),
	MOD_REQ("jcu", 51, CLK_B, 1, __BIT(3)),

	MOD_REQ("vin", 66, CLK_G, 2, __BIT(7)),
	MOD_REQ2("ether0", 65, CLK_B, 62, 2, __BIT(6)),
	MOD_REQ2("ether1", 64, CLK_B, 62, 2, __BIT(6)),
	MOD_REQ2("eptpc", 63, CLK_B, 62, 2, __BIT(6)),
	MOD_REQ("ether", 62, CLK_B, 2, __BIT(6)),
	MOD_REQ("usb0", 61, CLK_B, 3, __BIT(1) | __BIT(0)),
	MOD_REQ("usb1", 60, CLK_B, 3, __BIT(3) | __BIT(2)),

	MOD_REQ("imr", 77, CLK_G, 2, __BIT(3)),
	MOD_REQ("drw2d", 76, CLK_B, 2, __BIT(1) | __BIT(0)),
	MOD("csi2", 75, CLK_G),
	MOD("ssif0", 73, CLK_P1),
	MOD("ssif1", 72, CLK_P1),
	MOD("ssif2", 71, CLK_P1),
	MOD("ssif3", 70, CLK_P1),

	MOD("riic0", 87, CLK_P1),
	MOD("riic1", 86, CLK_P1),
	MOD("riic2", 85, CLK_P1),
	MOD("riic3", 84, CLK_P1),
	MOD("spibsc", 83, CLK_NONE),
	MOD_REQ("vdc6", 81, CLK_B, 2, __BIT(5)),

	MOD("rspi0", 97, CLK_P1),
	MOD("rspi1", 96, CLK_P1),
	MOD("rspi2", 95, CLK_P1),
	MOD("hyperbus", 93, CLK_NONE),
	{ .name = "octa", .id = 92, .parent = CLK_NONE, .keep = true },
	MOD("spdif", 91, CLK_P1),
	MOD_REQ("drp", 90, CLK_B, 2, __BIT(4)),

	MOD("tsip", 107, CLK_B),
	MOD_REQ("nand", 104, CLK_P1, 2, __BIT(2)),
	MOD_REQ2("sdhi00", 103, CLK_B, 102, 1, __BIT(2)),
	MOD_REQ2("sdhi01", 102, CLK_B, 103, 1, __BIT(2)),
	MOD_REQ2("sdhi10", 101, CLK_B, 100, 1, __BIT(1)),
	MOD_REQ2("sdhi11", 100, CLK_B, 101, 1, __BIT(1)),
};

#define	RZA2_CPG_NMODS	__arraycount(rza2_cpg_mods)

struct rza2_cpg_clk {
	struct clk base;
	const struct rza2_cpg_mod *mod;
	u_int core;
	u_int refcnt;
};

struct rza2_cpg_softc {
	device_t sc_dev;
	int sc_phandle;
	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;
	kmutex_t sc_lock;

	struct clk_domain sc_clkdom;
	struct rza2_cpg_clk sc_core[CLK_NCORE];
	struct rza2_cpg_clk sc_mods[RZA2_CPG_NMODS];
};

#define	RD1(sc, r) \
	bus_space_read_1((sc)->sc_bst, (sc)->sc_bsh, (r) - CPG_FRQCR)
#define	WR1(sc, r, v) \
	bus_space_write_1((sc)->sc_bst, (sc)->sc_bsh, (r) - CPG_FRQCR, (v))
#define	RD2(sc, r) \
	bus_space_read_2((sc)->sc_bst, (sc)->sc_bsh, (r) - CPG_FRQCR)

static int	rza2_cpg_match(device_t, cfdata_t, void *);
static void	rza2_cpg_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(rza2_cpg, sizeof(struct rza2_cpg_softc),
    rza2_cpg_match, rza2_cpg_attach, NULL, NULL);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "renesas,r7s9210-cpg-mssr" },
	DEVICE_COMPAT_EOL
};

static struct rza2_cpg_clk *
rza2_cpg_find_mod(struct rza2_cpg_softc *sc, u_int id)
{
	for (u_int i = 0; i < RZA2_CPG_NMODS; i++) {
		if (sc->sc_mods[i].mod->id == id)
			return &sc->sc_mods[i];
	}
	return NULL;
}

static struct clk *
rza2_cpg_decode(device_t dev, int cc_phandle, const void *data, size_t len)
{
	struct rza2_cpg_softc * const sc = device_private(dev);
	struct rza2_cpg_clk *clk;

	if (len != 8)
		return NULL;

	const u_int type = be32dec(data);
	const u_int index = be32dec((const uint8_t *)data + 4);
	switch (type) {
	case CPG_CORE:
		return index < CLK_NCORE ? &sc->sc_core[index].base : NULL;
	case CPG_MOD:
		clk = rza2_cpg_find_mod(sc, index);
		return clk != NULL ? &clk->base : NULL;
	default:
		return NULL;
	}
}

static const struct fdtbus_clock_controller_func rza2_cpg_fdt_funcs = {
	.decode = rza2_cpg_decode,
};

static struct clk *
rza2_cpg_clk_get(void *priv, const char *name)
{
	struct rza2_cpg_softc * const sc = priv;

	for (u_int i = 0; i < CLK_NCORE; i++) {
		if (strcmp(sc->sc_core[i].base.name, name) == 0)
			return &sc->sc_core[i].base;
	}
	for (u_int i = 0; i < RZA2_CPG_NMODS; i++) {
		if (strcmp(sc->sc_mods[i].base.name, name) == 0)
			return &sc->sc_mods[i].base;
	}
	return NULL;
}

static void
rza2_cpg_clk_put(void *priv, struct clk *clk)
{
}

static uint64_t
rza2_cpg_extal_rate(struct rza2_cpg_softc *sc)
{
	struct clk *extal;
	uint32_t freq;
	int phandle;

	extal = fdtbus_clock_get_index(sc->sc_phandle, 0);
	if (extal != NULL)
		return clk_get_rate(extal);

	phandle = fdtbus_get_phandle(sc->sc_phandle, "clocks");
	if (phandle > 0 &&
	    of_getprop_uint32(phandle, "clock-frequency", &freq) == 0)
		return freq;
	return 0;
}

static u_int
rza2_cpg_core_rate(struct rza2_cpg_softc *sc, u_int id)
{
	uint64_t in, pll;
	u_int div;

	in = rza2_cpg_extal_rate(sc);
	pll = (in > 16000000 ? in / 2 : in) * 88;

	const uint16_t frqcr = RD2(sc, CPG_FRQCR);
	switch (id) {
	case CLK_I:
		return pll / (2 << __SHIFTOUT(frqcr, FRQCR_IFC));
	case CLK_G:
	case CLK_B:
		div = __SHIFTOUT(frqcr, FRQCR_BFC);
		if (div == 0)
			return 0;
		return pll / (4 << div) * (id == CLK_G ? 2 : 1);
	case CLK_P1:
	case CLK_P1C:
		div = __SHIFTOUT(frqcr, FRQCR_PFC);
		if (div < 2)
			return 0;
		return pll / (4 << div);
	case CLK_P0:
		return pll / 32;
	default:
		return 0;
	}
}

static u_int
rza2_cpg_clk_get_rate(void *priv, struct clk *clkp)
{
	struct rza2_cpg_softc * const sc = priv;
	struct rza2_cpg_clk * const clk = (struct rza2_cpg_clk *)clkp;

	if (clk->mod == NULL)
		return rza2_cpg_core_rate(sc, clk->core);
	if (clk->mod->parent == CLK_NONE)
		return 0;
	return rza2_cpg_core_rate(sc, clk->mod->parent);
}

static int
rza2_cpg_clk_enable(void *priv, struct clk *clkp)
{
	struct rza2_cpg_softc * const sc = priv;
	struct rza2_cpg_clk * const clk = (struct rza2_cpg_clk *)clkp;
	const struct rza2_cpg_mod * const mod = clk->mod;
	uint8_t mask, val;
	int timo, error = 0;

	if (mod == NULL)
		return 0;

	const bus_size_t stbcr = CPG_STBCR(mod->id / 10);
	mask = __BIT(mod->id % 10);
	if (mod->also != 0)
		mask |= __BIT(mod->also % 10);

	mutex_enter(&sc->sc_lock);

	val = RD1(sc, stbcr);
	if ((val & mask) != 0) {
		WR1(sc, stbcr, val & ~mask);
		(void)RD1(sc, stbcr);
	}

	if (mod->stbreq != 0) {
		const bus_size_t req = CPG_STBREQ(mod->stbreq);
		const bus_size_t ack = CPG_STBACK(mod->stbreq);

		val = RD1(sc, req);
		if ((val & mod->stbreq_bits) != 0) {
			WR1(sc, req, val & ~mod->stbreq_bits);
			(void)RD1(sc, req);
			for (timo = 1000; timo > 0; timo--) {
				if ((RD1(sc, ack) & mod->stbreq_bits) == 0)
					break;
				delay(1);
			}
			if (timo == 0)
				error = ETIMEDOUT;
		}
	}

	if (error == 0)
		clk->refcnt++;
	mutex_exit(&sc->sc_lock);

	if (error != 0)
		device_printf(sc->sc_dev, "%s: standby request stuck\n",
		    mod->name);
	return error;
}

static int
rza2_cpg_clk_disable(void *priv, struct clk *clkp)
{
	struct rza2_cpg_softc * const sc = priv;
	struct rza2_cpg_clk * const clk = (struct rza2_cpg_clk *)clkp;
	const struct rza2_cpg_mod * const mod = clk->mod;
	uint8_t val;

	if (mod == NULL)
		return 0;

	mutex_enter(&sc->sc_lock);
	if (clk->refcnt > 0)
		clk->refcnt--;
	if (clk->refcnt == 0 && mod->stbreq == 0 && !mod->keep) {
		const bus_size_t stbcr = CPG_STBCR(mod->id / 10);

		val = RD1(sc, stbcr);
		WR1(sc, stbcr, val | __BIT(mod->id % 10));
		(void)RD1(sc, stbcr);
	}
	mutex_exit(&sc->sc_lock);

	return 0;
}

static struct clk *
rza2_cpg_clk_get_parent(void *priv, struct clk *clkp)
{
	struct rza2_cpg_softc * const sc = priv;
	struct rza2_cpg_clk * const clk = (struct rza2_cpg_clk *)clkp;

	if (clk->mod == NULL)
		return fdtbus_clock_get_index(sc->sc_phandle, 0);
	if (clk->mod->parent == CLK_NONE)
		return NULL;
	return &sc->sc_core[clk->mod->parent].base;
}

static const struct clk_funcs rza2_cpg_clk_funcs = {
	.get = rza2_cpg_clk_get,
	.put = rza2_cpg_clk_put,
	.get_rate = rza2_cpg_clk_get_rate,
	.enable = rza2_cpg_clk_enable,
	.disable = rza2_cpg_clk_disable,
	.get_parent = rza2_cpg_clk_get_parent,
};

static int
rza2_cpg_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
rza2_cpg_attach(device_t parent, device_t self, void *aux)
{
	struct rza2_cpg_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	bus_size_t size;
	u_int i;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	if (addr != RZA2_CPG_PBASE + CPG_FRQCR ||
	    size <= CPG_STBCR(10) - CPG_FRQCR) {
		aprint_error(": unexpected registers %#" PRIxBUSADDR "\n",
		    addr);
		return;
	}

	sc->sc_dev = self;
	sc->sc_phandle = phandle;
	sc->sc_bst = faa->faa_bst;
	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_NONE);

	sc->sc_clkdom.name = device_xname(self);
	sc->sc_clkdom.funcs = &rza2_cpg_clk_funcs;
	sc->sc_clkdom.priv = sc;

	for (i = 0; i < CLK_NCORE; i++) {
		struct rza2_cpg_clk * const clk = &sc->sc_core[i];

		clk->base.domain = &sc->sc_clkdom;
		clk->base.name = rza2_cpg_core_names[i];
		clk->core = i;
		clk_attach(&clk->base);
	}
	for (i = 0; i < RZA2_CPG_NMODS; i++) {
		struct rza2_cpg_clk * const clk = &sc->sc_mods[i];

		clk->base.domain = &sc->sc_clkdom;
		clk->base.name = rza2_cpg_mods[i].name;
		clk->mod = &rza2_cpg_mods[i];
		clk_attach(&clk->base);
	}

	aprint_naive("\n");
	aprint_normal(": RZ/A2 CPG, I %u MHz, B %u MHz, P1 %u MHz, "
	    "P0 %u MHz\n",
	    rza2_cpg_core_rate(sc, CLK_I) / 1000000,
	    rza2_cpg_core_rate(sc, CLK_B) / 1000000,
	    rza2_cpg_core_rate(sc, CLK_P1) / 1000000,
	    rza2_cpg_core_rate(sc, CLK_P0) / 1000000);

	fdtbus_register_clock_controller(self, phandle, &rza2_cpg_fdt_funcs);
}
