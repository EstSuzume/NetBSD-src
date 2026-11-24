// SPDX-License-Identifier: BSD-2-Clause
/*
 * Platform support for Renesas RZ/A1 (R7S72100).
 *
 * The RZ/A1 has a single Cortex-A9 core without the MPCore private timers,
 * so delay() and the system clock are provided by the OS timer (OSTM).
 * Most other peripherals are derived from SuperH designs.
 */

#include "opt_console.h"
#include "arml2cc.h"

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/device.h>

#include <dev/fdt/fdtvar.h>
#include <dev/fdt/fdt_platform.h>

#include <arm/fdt/arm_fdtvar.h>
#include <arm/locore.h>

#include <uvm/uvm_extern.h>

#include <arm/cortex/pl310_var.h>

#include <arm/rza1/rza1_reg.h>
#include <arm/rza1/rza1_scifreg.h>
#include <arm/rza1/rza1_var.h>

#include <libfdt.h>

extern struct bus_space armv7_generic_bs_tag;
extern struct arm32_bus_dma_tag arm_generic_dma_tag;

void rza1_platform_early_putchar(char);

static u_int rza1_p0_freq = RZA1_P0_FREQ_DEFAULT; /* P0 (OSTM, RTC) */
static u_int rza1_p1_freq = RZA1_P1_FREQ_DEFAULT; /* P1 (SCIF, RSPI, ETHER) */

static bool rza1_delay_running;

static const struct pmap_devmap *
rza1_platform_devmap(void)
{
	static const struct pmap_devmap devmap[] = {
		DEVMAP_ENTRY(RZA1_PERIPH0_VBASE,
			     RZA1_PERIPH0_PBASE,
			     RZA1_PERIPH0_SIZE),
		DEVMAP_ENTRY(RZA1_PERIPH1_VBASE,
			     RZA1_PERIPH1_PBASE,
			     RZA1_PERIPH1_SIZE),
		DEVMAP_ENTRY(RZA1_PERIPH2_VBASE,
			     RZA1_PERIPH2_PBASE,
			     RZA1_PERIPH2_SIZE),
		DEVMAP_ENTRY_END
	};

	return devmap;
}

static void
rza1_platform_init_attach_args(struct fdt_attach_args *faa)
{
	faa->faa_bst = &armv7_generic_bs_tag;
	faa->faa_dmat = &arm_generic_dma_tag;
}

void __noasan
rza1_platform_early_putchar(char c)
{
#ifdef CONSADDR
	const vaddr_t base = cpu_earlydevice_va_p() ?
	    RZA1_P2V(CONSADDR, 0) : CONSADDR;
	volatile uint16_t *fsr = (volatile uint16_t *)(base + SCIF_SCFSR);
	volatile uint8_t *ftdr = (volatile uint8_t *)(base + SCIF_SCFTDR);

	while ((*fsr & SCFSR_TDFE) == 0)
		continue;
	*ftdr = c;
	*fsr = (uint16_t)~(SCFSR_TDFE | SCFSR_TEND);
#endif
}

static void
rza1_platform_init_clocks(void)
{
	const void *fdt = fdtbus_get_data();
	const fdt32_t *prop;
	int off, len;

	off = fdt_path_offset(fdt, "/extal");
	if (off < 0)
		return;
	prop = fdt_getprop(fdt, off, "clock-frequency", &len);
	if (prop == NULL || len != sizeof(*prop) || fdt32_to_cpu(*prop) == 0)
		return;

	const uint64_t pll = (uint64_t)fdt32_to_cpu(*prop) * 30;
	rza1_p1_freq = pll / 6;
	rza1_p0_freq = pll / 12;
}

static void
rza1_delay_init(void)
{
	bus_space_tag_t bst = &armv7_generic_bs_tag;
	const bus_space_handle_t cpg = RZA1_P2V(RZA1_CPG_PBASE, 1);
	const bus_space_handle_t ostm =
	    RZA1_P2V(RZA1_OSTM_PBASE(RZA1_DELAY_OSTM), 1);
	uint8_t stbcr;

	stbcr = bus_space_read_1(bst, cpg, CPG_STBCR(5));
	bus_space_write_1(bst, cpg, CPG_STBCR(5), stbcr & ~__BITS(1,0));
	(void)bus_space_read_1(bst, cpg, CPG_STBCR(5));

	if ((bus_space_read_1(bst, ostm, OSTM_TE) & 1) == 0 ||
	    (bus_space_read_1(bst, ostm, OSTM_CTL) & OSTM_CTL_FREERUN) == 0) {
		bus_space_write_1(bst, ostm, OSTM_TT, 1);
		bus_space_write_1(bst, ostm, OSTM_CTL, OSTM_CTL_FREERUN);
		bus_space_write_4(bst, ostm, OSTM_CMP, 0xffffffff);
		bus_space_write_1(bst, ostm, OSTM_TS, 1);
	}
	rza1_delay_running = true;
}

static void
rza1_platform_delay(u_int us)
{
	bus_space_tag_t bst = &armv7_generic_bs_tag;
	const bus_space_handle_t ostm =
	    RZA1_P2V(RZA1_OSTM_PBASE(RZA1_DELAY_OSTM), 1);

	if (!rza1_delay_running) {
		for (volatile u_int n = us * 100; n > 0; n--)
			continue;
		return;
	}

	const uint32_t ticks_per_us = howmany(rza1_p0_freq, 1000000);
	uint64_t remaining = (uint64_t)us * ticks_per_us;
	uint32_t last = bus_space_read_4(bst, ostm, OSTM_CNT);

	while (remaining > 0) {
		const uint32_t now = bus_space_read_4(bst, ostm, OSTM_CNT);
		const uint32_t delta = now - last;

		last = now;
		if (delta >= remaining)
			break;
		remaining -= delta;
	}
}

u_int
rza1_get_p0_freq(void)
{
	return rza1_p0_freq;
}

u_int
rza1_get_p1_freq(void)
{
	return rza1_p1_freq;
}

uint32_t
rza1_get_entropy32(void)
{
	bus_space_tag_t bst = &armv7_generic_bs_tag;
	const bus_space_handle_t ostm =
	    RZA1_P2V(RZA1_OSTM_PBASE(RZA1_DELAY_OSTM), 1);

	if (!rza1_delay_running)
		return 0;
	return bus_space_read_4(bst, ostm, OSTM_CNT);
}

/*
 * The first 128KiB of the on-chip RAM is the data-retention RAM,
 * which is write protected after reset. Enable writes here.
 */
static void
rza1_enable_retention_ram(void)
{
	bus_space_tag_t bst = &armv7_generic_bs_tag;
	const bus_space_handle_t cpg = RZA1_P2V(RZA1_CPG_PBASE, 1);

	bus_space_write_1(bst, cpg, CPG_SYSCR3, SYSCR3_RRAMWE_ALL);
	(void)bus_space_read_1(bst, cpg, CPG_SYSCR3);
}

static void
rza1_platform_bootstrap(void)
{
	rza1_enable_retention_ram();
	rza1_platform_init_clocks();
	rza1_delay_init();

#if NARML2CC > 0
	const bus_space_handle_t pl310_bh = RZA1_P2V(RZA1_L2C_PBASE, 2);
	arml2cc_get_cacheinfo(&armv7_generic_bs_tag, pl310_bh, 0);
#endif

	arm_fdt_cpu_bootstrap();
}

static void
rza1_platform_device_register(device_t self, void *aux)
{
}

static void
rza1_platform_reset(void)
{
	bus_space_tag_t bst = &armv7_generic_bs_tag;
	const bus_space_handle_t wdt = RZA1_P2V(RZA1_WDT_PBASE, 1);

	bus_space_write_2(bst, wdt, WDT_WTCSR, WTCSR_KEY);
	(void)bus_space_read_1(bst, wdt, WDT_WRCSR);
	bus_space_write_2(bst, wdt, WDT_WRCSR, WRCSR_CLEAR_WOVF);
	bus_space_write_2(bst, wdt, WDT_WRCSR, WRCSR_KEY | WRCSR_RSTE);
	bus_space_write_2(bst, wdt, WDT_WTCNT, WTCNT_KEY | 0xff);
	bus_space_write_2(bst, wdt, WDT_WTCSR,
	    WTCSR_KEY | WTCSR_WT | WTCSR_TME);

	for (;;)
		continue;
}

static u_int
rza1_platform_uart_freq(void)
{
	return rza1_p1_freq;
}

static const struct fdt_platform rza1_platform = {
	.fp_devmap = rza1_platform_devmap,
	.fp_bootstrap = rza1_platform_bootstrap,
	.fp_init_attach_args = rza1_platform_init_attach_args,
	.fp_device_register = rza1_platform_device_register,
	.fp_reset = rza1_platform_reset,
	.fp_delay = rza1_platform_delay,
	.fp_uart_freq = rza1_platform_uart_freq,
};

FDT_PLATFORM(rza1, "renesas,r7s72100", &rza1_platform);
