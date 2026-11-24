// SPDX-License-Identifier: BSD-2-Clause
/*
 * Renesas RZ/A2 OS timer (OSTM), derived from the RZ/A1 driver.
 * XXX TODO: Unify the RZ/A1 driver with this.
 */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/intr.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/timetc.h>

#include <dev/fdt/fdtvar.h>

#include <arm/fdt/arm_fdtvar.h>

#include <arm/rza2/rza2_reg.h>
#include <arm/rza2/rza2_var.h>

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "renesas,r7s9210-ostm" },
	{ .compat = "renesas,ostm" },
	DEVICE_COMPAT_EOL
};

struct rza2_ostm_softc {
	device_t sc_dev;
	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;
	int sc_phandle;
	u_int sc_freq;
	struct timecounter sc_tc;
};

#define	OSTM_READ_1(sc, reg) \
	bus_space_read_1((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define	OSTM_WRITE_1(sc, reg, val) \
	bus_space_write_1((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))
#define	OSTM_READ_4(sc, reg) \
	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define	OSTM_WRITE_4(sc, reg, val) \
	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))

static struct rza2_ostm_softc *rza2_ostm_clock_sc;

static int	rza2_ostm_match(device_t, cfdata_t, void *);
static void	rza2_ostm_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(rza2_ostm, sizeof(struct rza2_ostm_softc),
    rza2_ostm_match, rza2_ostm_attach, NULL, NULL);

static int
rza2_ostm_intr(void *arg)
{
	struct clockframe * const frame = arg;

	hardclock(frame);

	return 1;
}

static void
rza2_ostm_cpu_initclocks(void)
{
	struct rza2_ostm_softc * const sc = rza2_ostm_clock_sc;
	char intrstr[128];
	void *ih;

	KASSERT(sc != NULL);

	if (!fdtbus_intr_str(sc->sc_phandle, 0, intrstr, sizeof(intrstr)))
		panic("%s: failed to decode interrupt", __func__);

	ih = fdtbus_intr_establish_xname(sc->sc_phandle, 0, IPL_CLOCK,
	    FDT_INTR_MPSAFE, rza2_ostm_intr, NULL, device_xname(sc->sc_dev));
	if (ih == NULL)
		panic("%s: failed to establish timer interrupt", __func__);

	aprint_normal_dev(sc->sc_dev, "interrupting on %s\n", intrstr);

	OSTM_WRITE_1(sc, OSTM_TT, 1);
	OSTM_WRITE_1(sc, OSTM_CTL, 0);
	OSTM_WRITE_4(sc, OSTM_CMP, sc->sc_freq / hz - 1);
	OSTM_WRITE_1(sc, OSTM_TS, 1);
}

static u_int
rza2_ostm_get_timecount(struct timecounter *tc)
{
	struct rza2_ostm_softc * const sc = tc->tc_priv;

	return OSTM_READ_4(sc, OSTM_CNT);
}

static int
rza2_ostm_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
rza2_ostm_attach(device_t parent, device_t self, void *aux)
{
	struct rza2_ostm_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	struct clk *clk;
	bus_addr_t addr;
	bus_size_t size;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	sc->sc_dev = self;
	sc->sc_phandle = phandle;
	sc->sc_bst = faa->faa_bst;
	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	clk = fdtbus_clock_get_index(phandle, 0);
	if (clk != NULL) {
		if (clk_enable(clk) != 0) {
			aprint_error(": couldn't enable clock\n");
			return;
		}
		sc->sc_freq = clk_get_rate(clk);
	}
	if (sc->sc_freq == 0)
		sc->sc_freq = rza2_get_p1_freq();

	aprint_naive("\n");
	aprint_normal(": OS timer, %u Hz\n", sc->sc_freq);

	if (addr == RZA2_OSTM_PBASE(RZA2_DELAY_OSTM)) {
		struct timecounter * const tc = &sc->sc_tc;

		tc->tc_get_timecount = rza2_ostm_get_timecount;
		tc->tc_counter_mask = ~0u;
		tc->tc_frequency = sc->sc_freq;
		tc->tc_name = device_xname(self);
		tc->tc_quality = 200;
		tc->tc_priv = sc;
		tc_init(tc);
	} else if (rza2_ostm_clock_sc == NULL) {
		OSTM_WRITE_1(sc, OSTM_TT, 1);
		rza2_ostm_clock_sc = sc;
		arm_fdt_timer_register(rza2_ostm_cpu_initclocks);
	}
}
