// SPDX-License-Identifier: BSD-2-Clause
/*
 * Renesas RZ/A1 (R7S72100) physical memory map and fixed mappings.
 */

#ifndef _ARM_RZA1_RZA1_REG_H_
#define _ARM_RZA1_RZA1_REG_H_

#include <machine/vmparam.h>
#include <arch/evbarm/fdt/platform.h>

#define	RZA1_PERIPH0_PBASE	0xe8000000	/* SCIF, RSPI, GIC, ETHER .. */
#define	RZA1_PERIPH0_SIZE	0x00300000
#define	RZA1_PERIPH0_VBASE	(KERNEL_IO_VBASE + 0x00000000)

#define	RZA1_PERIPH1_PBASE	0xfcf00000	/* CPG, WDT, ports, OSTM, RTC */
#define	RZA1_PERIPH1_SIZE	0x00100000
#define	RZA1_PERIPH1_VBASE	(KERNEL_IO_VBASE + 0x00300000)

#define	RZA1_PERIPH2_PBASE	0x3fe00000	/* SPIBSC, L2C */
#define	RZA1_PERIPH2_SIZE	0x00200000
#define	RZA1_PERIPH2_VBASE	(KERNEL_IO_VBASE + 0x00400000)

#define	RZA1_P2V(pa, n)		\
	((pa) - RZA1_PERIPH ## n ## _PBASE + RZA1_PERIPH ## n ## _VBASE)

#define	RZA1_SCIF_PBASE(n)	(0xe8007000 + 0x800 * (n))
#define	RZA1_GIC_DIST_PBASE	0xe8201000
#define	RZA1_GIC_CPU_PBASE	0xe8202000
#define	RZA1_WDT_PBASE		0xfcfe0000
#define	RZA1_CPG_PBASE		0xfcfe0000
#define	RZA1_OSTM_PBASE(n)	(0xfcfec000 + 0x400 * (n))
#define	RZA1_L2C_PBASE		0x3ffff000
#define	RZA1_L2C_SIZE		0x1000

#define	CPG_FRQCR		0x10
#define	CPG_FRQCR2		0x14
#define	CPG_STBCR(n)		((n) < 3 ? 0x20 + 4 * ((n) - 1) : \
				 0x420 + 4 * ((n) - 3))
#define	CPG_SYSCR3		0x408
#define	 SYSCR3_RRAMWE_ALL	0x0f

#define	WDT_WTCSR		0x00
#define	 WTCSR_KEY		0xa500
#define	 WTCSR_WT		__BIT(6)
#define	 WTCSR_TME		__BIT(5)
#define	 WTCSR_CKS		__BITS(2,0)
#define	WDT_WTCNT		0x02
#define	 WTCNT_KEY		0x5a00
#define	WDT_WRCSR		0x04
#define	 WRCSR_KEY		0x5a00
#define	 WRCSR_RSTE		__BIT(6)
#define	 WRCSR_CLEAR_WOVF	0xa500

#define	OSTM_CMP		0x00
#define	OSTM_CNT		0x04
#define	OSTM_TE			0x10
#define	OSTM_TS			0x14
#define	OSTM_TT			0x18
#define	OSTM_CTL		0x20
#define	 OSTM_CTL_FREERUN	__BIT(1)
#define	 OSTM_CTL_STARTINT	__BIT(0)

#define	RZA1_DELAY_OSTM		1

#endif /* _ARM_RZA1_RZA1_REG_H_ */
