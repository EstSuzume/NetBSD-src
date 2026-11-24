// SPDX-License-Identifier: BSD-2-Clause
/*
 * Renesas RZ/A1 clock pulse generator (CPG).
 *
 * "renesas,r7s72100-cpg-clocks": PLL, CPU (I), image processing (G) clocks
 * "renesas,r7s72100-mstp-clocks": module stop bits in an 8-bit STBCRn register
 */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/kmem.h>
#include <sys/systm.h>

#include <dev/clk/clk_backend.h>
#include <dev/fdt/fdtvar.h>

#include <arm/rza1/rza1_reg.h>

#define	CPG_CLK_PLL	0
#define	CPG_CLK_I	1
#define	CPG_CLK_G	2
#define	CPG_NCLKS	3

#define	FRQCR_IFC	__BITS(9,8)
#define	FRQCR2_GFC	__BITS(1,0)

#define	MSTP_MAXCLKS	8

enum rza1_cpg_type {
	RZA1_CPG_CLOCKS = 1,
	RZA1_CPG_MSTP,
};

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "renesas,r7s72100-cpg-clocks", .value = RZA1_CPG_CLOCKS },
	{ .compat = "renesas,r7s72100-mstp-clocks", .value = RZA1_CPG_MSTP },
	DEVICE_COMPAT_EOL
};

struct rza1_cpg_clk {
	struct clk base;
	u_int index;		/* CPG clock id or MSTP bit */
	u_int parent;		/* MSTP: index into the "clocks" property */
};

struct rza1_cpg_softc {
	device_t sc_dev;
	int sc_phandle;
	enum rza1_cpg_type sc_type;
	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;

	struct clk_domain sc_clkdom;
	struct rza1_cpg_clk *sc_clks;
	u_int sc_nclks;
};

#define	CPG_READ_1(sc, reg) \
	bus_space_read_1((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define	CPG_WRITE_1(sc, reg, val) \
	bus_space_write_1((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))
#define	CPG_READ_2(sc, reg) \
	bus_space_read_2((sc)->sc_bst, (sc)->sc_bsh, (reg))

static int	rza1_cpg_match(device_t, cfdata_t, void *);
static void	rza1_cpg_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(rza1_cpg, sizeof(struct rza1_cpg_softc),
    rza1_cpg_match, rza1_cpg_attach, NULL, NULL);

static struct rza1_cpg_clk *
rza1_cpg_find(struct rza1_cpg_softc *sc, u_int index)
{
	for (u_int i = 0; i < sc->sc_nclks; i++) {
		if (sc->sc_clks[i].index == index)
			return &sc->sc_clks[i];
	}
	return NULL;
}

static struct clk *
rza1_cpg_decode(device_t dev, int cc_phandle, const void *data, size_t len)
{
	struct rza1_cpg_softc * const sc = device_private(dev);
	struct rza1_cpg_clk *clk;

	if (len != 4)
		return NULL;

	clk = rza1_cpg_find(sc, be32dec(data));
	return clk != NULL ? &clk->base : NULL;
}

static const struct fdtbus_clock_controller_func rza1_cpg_fdt_funcs = {
	.decode = rza1_cpg_decode,
};

static struct clk *
rza1_cpg_clk_get(void *priv, const char *name)
{
	struct rza1_cpg_softc * const sc = priv;

	for (u_int i = 0; i < sc->sc_nclks; i++) {
		if (strcmp(sc->sc_clks[i].base.name, name) == 0)
			return &sc->sc_clks[i].base;
	}
	return NULL;
}

static void
rza1_cpg_clk_put(void *priv, struct clk *clk)
{
}

static u_int
rza1_cpg_clk_get_rate(void *priv, struct clk *clkp)
{
	struct rza1_cpg_softc * const sc = priv;
	struct rza1_cpg_clk * const clk = (struct rza1_cpg_clk *)clkp;
	struct clk *parent;
	uint64_t pll;
	u_int div;

	if (sc->sc_type == RZA1_CPG_MSTP) {
		parent = fdtbus_clock_get_index(sc->sc_phandle, clk->parent);
		return parent != NULL ? clk_get_rate(parent) : 0;
	}

	/* clock mode 0 only: PLL = EXTAL x 30 */
	parent = fdtbus_clock_get_index(sc->sc_phandle, 0);
	if (parent == NULL)
		return 0;
	pll = (uint64_t)clk_get_rate(parent) * 30;

	switch (clk->index) {
	case CPG_CLK_PLL:
		return pll;
	case CPG_CLK_I:
		div = __SHIFTOUT(CPG_READ_2(sc, CPG_FRQCR), FRQCR_IFC);
		break;
	case CPG_CLK_G:
		div = __SHIFTOUT(CPG_READ_2(sc, CPG_FRQCR2), FRQCR2_GFC);
		break;
	default:
		return 0;
	}

	switch (div) {
	case 0:
		return pll;
	case 1:
		return pll * 2 / 3;
	case 3:
		return pll / 3;
	default:
		return 0;
	}
}

static int
rza1_cpg_clk_enable(void *priv, struct clk *clkp)
{
	struct rza1_cpg_softc * const sc = priv;
	struct rza1_cpg_clk * const clk = (struct rza1_cpg_clk *)clkp;
	uint8_t val;

	if (sc->sc_type != RZA1_CPG_MSTP)
		return 0;

	val = CPG_READ_1(sc, 0);
	CPG_WRITE_1(sc, 0, val & ~__BIT(clk->index));
	/* dummy read is required */
	(void)CPG_READ_1(sc, 0);

	return 0;
}

static int
rza1_cpg_clk_disable(void *priv, struct clk *clkp)
{
	struct rza1_cpg_softc * const sc = priv;
	struct rza1_cpg_clk * const clk = (struct rza1_cpg_clk *)clkp;
	uint8_t val;

	if (sc->sc_type != RZA1_CPG_MSTP)
		return 0;

	val = CPG_READ_1(sc, 0);
	CPG_WRITE_1(sc, 0, val | __BIT(clk->index));
	(void)CPG_READ_1(sc, 0);

	return 0;
}

static struct clk *
rza1_cpg_clk_get_parent(void *priv, struct clk *clkp)
{
	struct rza1_cpg_softc * const sc = priv;
	struct rza1_cpg_clk * const clk = (struct rza1_cpg_clk *)clkp;

	if (sc->sc_type == RZA1_CPG_MSTP)
		return fdtbus_clock_get_index(sc->sc_phandle, clk->parent);
	if (clk->index == CPG_CLK_PLL)
		return fdtbus_clock_get_index(sc->sc_phandle, 0);
	return &rza1_cpg_find(sc, CPG_CLK_PLL)->base;
}

static const struct clk_funcs rza1_cpg_clk_funcs = {
	.get = rza1_cpg_clk_get,
	.put = rza1_cpg_clk_put,
	.get_rate = rza1_cpg_clk_get_rate,
	.enable = rza1_cpg_clk_enable,
	.disable = rza1_cpg_clk_disable,
	.get_parent = rza1_cpg_clk_get_parent,
};

static int
rza1_cpg_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
rza1_cpg_attach(device_t parent, device_t self, void *aux)
{
	struct rza1_cpg_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	uint32_t indices[MSTP_MAXCLKS];
	bus_addr_t addr;
	bus_size_t size;
	const char *name;
	int len;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	sc->sc_dev = self;
	sc->sc_phandle = phandle;
	sc->sc_type = of_compatible_lookup(phandle, compat_data)->value;
	sc->sc_bst = faa->faa_bst;
	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	if (sc->sc_type == RZA1_CPG_CLOCKS) {
		sc->sc_nclks = CPG_NCLKS;
		for (u_int i = 0; i < CPG_NCLKS; i++)
			indices[i] = i;
	} else {
		len = OF_getproplen(phandle, "clock-indices");
		if (len <= 0 || len > (int)sizeof(indices) || len % 4 != 0) {
			aprint_error(": invalid clock-indices\n");
			return;
		}
		sc->sc_nclks = len / 4;
		of_getprop_uint32_array(phandle, "clock-indices", indices,
		    sc->sc_nclks);
	}

	sc->sc_clkdom.name = device_xname(self);
	sc->sc_clkdom.funcs = &rza1_cpg_clk_funcs;
	sc->sc_clkdom.priv = sc;

	sc->sc_clks = kmem_zalloc(sizeof(*sc->sc_clks) * sc->sc_nclks,
	    KM_SLEEP);
	for (u_int i = 0; i < sc->sc_nclks; i++) {
		struct rza1_cpg_clk * const clk = &sc->sc_clks[i];

		name = fdtbus_get_string_index(phandle, "clock-output-names",
		    i);
		clk->base.domain = &sc->sc_clkdom;
		clk->base.name = name != NULL ? name : device_xname(self);
		clk->index = indices[i];
		clk->parent = i;
		clk_attach(&clk->base);
	}

	aprint_naive("\n");
	if (sc->sc_type == RZA1_CPG_CLOCKS)
		aprint_normal(": RZ/A1 CPG\n");
	else
		aprint_normal(": RZ/A1 module stop clocks (%u)\n",
		    sc->sc_nclks);

	fdtbus_register_clock_controller(self, phandle, &rza1_cpg_fdt_funcs);
}
