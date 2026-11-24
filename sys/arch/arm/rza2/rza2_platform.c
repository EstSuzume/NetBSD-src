// SPDX-License-Identifier: BSD-2-Clause
/*
 * Platform support for Renesas RZ/A2 (R7S9210), e.g. GR-MANGO.
 *
 * Like the RZ/A1, the RZ/A2 has a single Cortex-A9 core without the MPCore
 * private timers, so delay() and the system clock are provided by the OS
 * timer (OSTM). The SCIFA has the RZ/G2L register layout.
 *
 * Memory: 4 MiB of on-chip RAM at 0x80000000 and 8 MiB of OctaRAM at
 * 0x60000000. The kernel has to be loaded at the lowest physical address
 * (the OctaRAM), because the generic start code maps its load address to
 * KERNEL_BASE and the direct map goes up from there.
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
#include <dev/fdt/rz_scifreg.h>

#include <arm/fdt/arm_fdtvar.h>
#include <arm/locore.h>

#include <uvm/uvm_extern.h>

#include <machine/bootconfig.h>

#include <arm/cortex/pl310_var.h>

#include <arm/rza2/rza2_reg.h>
#include <arm/rza2/rza2_var.h>

#include <libfdt.h>

extern struct bus_space armv7_generic_bs_tag;
extern struct arm32_bus_dma_tag arm_generic_dma_tag;

void rza2_platform_early_putchar(char);

static u_int rza2_p1_freq = RZA2_P1_FREQ_DEFAULT;

static bool rza2_delay_running;

static const struct pmap_devmap *
rza2_platform_devmap(void)
{
	static const struct pmap_devmap devmap[] = {
		DEVMAP_ENTRY(RZA2_PERIPH0_VBASE,
			     RZA2_PERIPH0_PBASE,
			     RZA2_PERIPH0_SIZE),
		DEVMAP_ENTRY(RZA2_PERIPH1_VBASE,
			     RZA2_PERIPH1_PBASE,
			     RZA2_PERIPH1_SIZE),
		DEVMAP_ENTRY(RZA2_PERIPH2_VBASE,
			     RZA2_PERIPH2_PBASE,
			     RZA2_PERIPH2_SIZE),
		DEVMAP_ENTRY_END
	};

	return devmap;
}

static void
rza2_platform_init_attach_args(struct fdt_attach_args *faa)
{
	faa->faa_bst = &armv7_generic_bs_tag;
	faa->faa_dmat = &arm_generic_dma_tag;
}

void __noasan
rza2_platform_early_putchar(char c)
{
#ifdef CONSADDR
	const vaddr_t base = cpu_earlydevice_va_p() ?
	    RZA2_P2V(CONSADDR, 0) : CONSADDR;
	volatile uint16_t *fdr = (volatile uint16_t *)(base + SCIF_SCFDR);
	volatile uint8_t *ftdr = (volatile uint8_t *)(base + SCIF_SCFTDR);

	while (__SHIFTOUT(*fdr, SCFDR_TXCNT) >= SCIF_FIFO_LEN)
		continue;
	*ftdr = c;
#endif
}

static void
rza2_platform_init_clocks(void)
{
	bus_space_tag_t bst = &armv7_generic_bs_tag;
	const bus_space_handle_t cpg = RZA2_P2V(RZA2_CPG_PBASE, 1);
	const void *fdt = fdtbus_get_data();
	const fdt32_t *prop;
	int off, len;

	off = fdt_path_offset(fdt, "/extal");
	if (off < 0)
		return;
	prop = fdt_getprop(fdt, off, "clock-frequency", &len);
	if (prop == NULL || len != sizeof(*prop) || fdt32_to_cpu(*prop) == 0)
		return;

	const uint64_t extal = fdt32_to_cpu(*prop);
	const uint64_t pll = extal * (extal > 12000000 ? 44 : 88);
	const uint16_t frqcr = bus_space_read_2(bst, cpg, CPG_FRQCR);
	rza2_p1_freq = pll / (4 << __SHIFTOUT(frqcr, FRQCR_PFC));
}

static void
rza2_delay_init(void)
{
	bus_space_tag_t bst = &armv7_generic_bs_tag;
	const bus_space_handle_t cpg = RZA2_P2V(RZA2_CPG_PBASE, 1);
	const bus_space_handle_t ostm =
	    RZA2_P2V(RZA2_OSTM_PBASE(RZA2_DELAY_OSTM), 0);
	uint8_t stbcr;

	stbcr = bus_space_read_1(bst, cpg, CPG_STBCR(3));
	bus_space_write_1(bst, cpg, CPG_STBCR(3),
	    stbcr & ~(STBCR3_MSTP36 | STBCR3_MSTP35));
	(void)bus_space_read_1(bst, cpg, CPG_STBCR(3));

	if ((bus_space_read_1(bst, ostm, OSTM_TE) & 1) == 0 ||
	    (bus_space_read_1(bst, ostm, OSTM_CTL) & OSTM_CTL_FREERUN) == 0) {
		bus_space_write_1(bst, ostm, OSTM_TT, 1);
		bus_space_write_1(bst, ostm, OSTM_CTL, OSTM_CTL_FREERUN);
		bus_space_write_4(bst, ostm, OSTM_CMP, 0xffffffff);
		bus_space_write_1(bst, ostm, OSTM_TS, 1);
	}
	rza2_delay_running = true;
}

static void
rza2_platform_delay(u_int us)
{
	bus_space_tag_t bst = &armv7_generic_bs_tag;
	const bus_space_handle_t ostm =
	    RZA2_P2V(RZA2_OSTM_PBASE(RZA2_DELAY_OSTM), 0);

	if (!rza2_delay_running) {
		for (volatile u_int n = us * 100; n > 0; n--)
			continue;
		return;
	}

	const uint32_t ticks_per_us = howmany(rza2_p1_freq, 1000000);
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
rza2_get_p1_freq(void)
{
	return rza2_p1_freq;
}

uint32_t
rza2_get_entropy32(void)
{
	bus_space_tag_t bst = &armv7_generic_bs_tag;
	const bus_space_handle_t ostm =
	    RZA2_P2V(RZA2_OSTM_PBASE(RZA2_DELAY_OSTM), 0);

	if (!rza2_delay_running)
		return 0;
	return bus_space_read_4(bst, ostm, OSTM_CNT);
}

static void
rza2_enable_retention_ram(void)
{
	bus_space_tag_t bst = &armv7_generic_bs_tag;
	const bus_space_handle_t cpg = RZA2_P2V(RZA2_CPG_PBASE, 1);

	bus_space_write_1(bst, cpg, CPG_SYSCR3, SYSCR3_RRAMWE_ALL);
	(void)bus_space_read_1(bst, cpg, CPG_SYSCR3);
}

static void
rza2_platform_bypass_eptpc(void)
{
	bus_space_tag_t bst = &armv7_generic_bs_tag;
	const bus_space_handle_t cpg = RZA2_P2V(RZA2_CPG_PBASE, 1);
	const bus_space_handle_t eptpc = RZA2_P2V(RZA2_EPTPC_PBASE, 0);
	const uint32_t bypass = BYPASS_BYPASS1 | BYPASS_BYPASS0;
	const uint8_t stbcr6 = bus_space_read_1(bst, cpg, CPG_STBCR(6));

	if ((stbcr6 & STBCR6_MSTP63) == 0)
		return;
	if ((stbcr6 & STBCR6_MSTP62) == 0 &&
	    bus_space_read_4(bst, eptpc, EPTPC_BYPASS) == bypass)
		return;

	bus_space_write_1(bst, cpg, CPG_STBCR(6),
	    stbcr6 & ~(STBCR6_MSTP65 | STBCR6_MSTP64 | STBCR6_MSTP62));
	(void)bus_space_read_1(bst, cpg, CPG_STBCR(6));
	rza2_platform_delay(3);
	bus_space_write_4(bst, eptpc, EPTPC_BYPASS, bypass);
	(void)bus_space_read_4(bst, eptpc, EPTPC_BYPASS);
	rza2_platform_delay(3);
}

static void
rza2_platform_bootstrap(void)
{
	rza2_enable_retention_ram();
	rza2_platform_init_clocks();
	rza2_delay_init();
	rza2_platform_bypass_eptpc();

#if NARML2CC > 0
	const bus_space_handle_t pl310_bh = RZA2_P2V(RZA2_L2C_PBASE, 2);
	arml2cc_get_cacheinfo(&armv7_generic_bs_tag, pl310_bh, 0);
#endif

	arm_fdt_cpu_bootstrap();
}

static void
rza2_platform_device_register(device_t self, void *aux)
{
}

static void
rza2_platform_reset(void)
{
	bus_space_tag_t bst = &armv7_generic_bs_tag;
	const bus_space_handle_t wdt = RZA2_P2V(RZA2_WDT_PBASE, 1);

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

static int
rza2_platform_mpstart(void)
{
	psize_t pages = 0;

	for (u_int i = 0; i < bootconfig.dramblocks; i++)
		pages += bootconfig.dram[i].pages;
	if (pages != 0)
		physmem = pages;

	return 0;
}

static u_int
rza2_platform_uart_freq(void)
{
	return rza2_p1_freq;
}

static const struct fdt_platform rza2_platform = {
	.fp_devmap = rza2_platform_devmap,
	.fp_bootstrap = rza2_platform_bootstrap,
	.fp_init_attach_args = rza2_platform_init_attach_args,
	.fp_device_register = rza2_platform_device_register,
	.fp_reset = rza2_platform_reset,
	.fp_delay = rza2_platform_delay,
	.fp_mpstart = rza2_platform_mpstart,
	.fp_uart_freq = rza2_platform_uart_freq,
};

FDT_PLATFORM(rza2, "renesas,r7s9210", &rza2_platform);
