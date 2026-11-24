// SPDX-License-Identifier: BSD-2-Clause
/*
 * Renesas RZ/A2 (R7S9210) register addresses.
 */

#ifndef _ARM_RZA2_RZA2_REG_H_
#define _ARM_RZA2_RZA2_REG_H_

#include <arch/evbarm/fdt/platform.h>

/* Statically mapped I/O regions (devmap) */
#define	RZA2_PERIPH0_PBASE	0xe8000000	/* SCIF, RSPI, OSTM, ETHER, GIC, SDHI */
#define	RZA2_PERIPH0_SIZE	0x00300000
#define	RZA2_PERIPH0_VBASE	(KERNEL_IO_VBASE + 0x00000000)

#define	RZA2_PERIPH1_PBASE	0xfcf00000	/* CPG, WDT, IRQC, ports */
#define	RZA2_PERIPH1_SIZE	0x00100000
#define	RZA2_PERIPH1_VBASE	(KERNEL_IO_VBASE + 0x00300000)

#define	RZA2_PERIPH2_PBASE	0x1f000000	/* L2C-310 */
#define	RZA2_PERIPH2_SIZE	0x00100000
#define	RZA2_PERIPH2_VBASE	(KERNEL_IO_VBASE + 0x00400000)

#define	RZA2_P2V(pa, n)	\
	((pa) - RZA2_PERIPH ## n ## _PBASE + RZA2_PERIPH ## n ## _VBASE)

#define	RZA2_L2C_PBASE		0x1f003000
#define	RZA2_CPG_PBASE		0xfcfe0000
#define	RZA2_WDT_PBASE		0xfcfe7000
#define	RZA2_OSTM_PBASE(n)	(0xe803b000 + 0x1000 * (n))
#define	RZA2_EPTPC_PBASE	0xe8204000

#define	CPG_FRQCR		0x010
#define	 FRQCR_IFC		__BITS(9,8)
#define	 FRQCR_BFC		__BITS(5,4)
#define	 FRQCR_PFC		__BITS(1,0)
#define	CPG_STBCR(n)		((n) < 3 ? 0x020 + 4 * ((n) - 1) : \
				 0x420 + 4 * ((n) - 3))
#define	CPG_STBREQ(n)		(0x030 + 4 * ((n) - 1))
#define	CPG_STBACK(n)		(0x040 + 4 * ((n) - 1))
#define	 STBCR3_MSTP36		__BIT(6)	/* OSTM0 */
#define	 STBCR3_MSTP35		__BIT(5)	/* OSTM1 */
#define	 STBCR3_MSTP34		__BIT(4)	/* OSTM2 */
#define	 STBCR6_MSTP65		__BIT(5)	/* ETHERC0, EDMAC0 */
#define	 STBCR6_MSTP64		__BIT(4)	/* ETHERC1, EDMAC1 */
#define	 STBCR6_MSTP63		__BIT(3)	/* EPTPC, PTPEDMAC */
#define	 STBCR6_MSTP62		__BIT(2)	/* Ethernet common circuits */
#define	CPG_SYSCR3		0x408
#define	 SYSCR3_RRAMWE_ALL	0x0f

#define	EPTPC_BYPASS		0x508
#define	 BYPASS_BYPASS1		__BIT(16)
#define	 BYPASS_BYPASS0		__BIT(0)

#define	WDT_WTCSR		0x00
#define	 WTCSR_KEY		0xa500
#define	 WTCSR_WT		__BIT(6)
#define	 WTCSR_TME		__BIT(5)
#define	WDT_WTCNT		0x02
#define	 WTCNT_KEY		0x5a00
#define	WDT_WRCSR		0x04
#define	 WRCSR_KEY		0x5a00
#define	 WRCSR_RSTE		__BIT(6)
#define	 WRCSR_CLEAR_WOVF	0xa500

/* OSTM (same layout as on the RZ/A1) */
#define	OSTM_CMP		0x00
#define	OSTM_CNT		0x04
#define	OSTM_TE			0x10
#define	OSTM_TS			0x14
#define	OSTM_TT			0x18
#define	OSTM_CTL		0x20
#define	 OSTM_CTL_FREERUN	__BIT(1)
#define	 OSTM_CTL_STARTINT	__BIT(0)

#define	RZA2_DELAY_OSTM		1

#endif /* _ARM_RZA2_RZA2_REG_H_ */
