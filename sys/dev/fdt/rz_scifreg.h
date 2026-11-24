// SPDX-License-Identifier: BSD-2-Clause
/*
 * Renesas RZ/G2L family SCIF (serial communication interface with FIFO),
 * as found in the RZ/Five (R9A07G043F) and RZ/G2UL.
 */

#ifndef _DEV_FDT_RZ_SCIFREG_H_
#define _DEV_FDT_RZ_SCIFREG_H_

#define	SCIF_SCSMR		0x00	/* 16: serial mode */
#define	 SCSMR_CHR		0x0040	/* 7-bit characters */
#define	 SCSMR_PE		0x0020	/* parity enable */
#define	 SCSMR_OE		0x0010	/* odd parity */
#define	 SCSMR_STOP		0x0008	/* 2 stop bits */
#define	 SCSMR_CKS		0x0003	/* clock select */
#define	SCIF_SCBRR		0x02	/* 8: bit rate */
#define	SCIF_SCSCR		0x04	/* 16: serial control */
#define	 SCSCR_TIE		0x0080	/* transmit interrupt enable */
#define	 SCSCR_RIE		0x0040	/* receive interrupt enable */
#define	 SCSCR_TE		0x0020	/* transmit enable */
#define	 SCSCR_RE		0x0010	/* receive enable */
#define	 SCSCR_REIE		0x0008	/* receive error interrupt enable */
#define	 SCSCR_CKE		0x0003	/* clock enable */
#define	SCIF_SCFTDR		0x06	/* 8: transmit FIFO data */
#define	SCIF_SCFSR		0x08	/* 16: serial status */
#define	 SCFSR_ER		0x0080	/* receive error */
#define	 SCFSR_TEND		0x0040	/* transmit end */
#define	 SCFSR_TDFE		0x0020	/* transmit FIFO data empty */
#define	 SCFSR_BRK		0x0010	/* break detect */
#define	 SCFSR_FER		0x0008	/* framing error */
#define	 SCFSR_PER		0x0004	/* parity error */
#define	 SCFSR_RDF		0x0002	/* receive FIFO data full */
#define	 SCFSR_DR		0x0001	/* receive data ready */
#define	SCIF_SCFRDR		0x0a	/* 8: receive FIFO data */
#define	SCIF_SCFCR		0x0c	/* 16: FIFO control */
#define	 SCFCR_RTRG		0x00c0	/* receive trigger */
#define	  SCFCR_RTRG_1		0x0000
#define	  SCFCR_RTRG_4		0x0040
#define	  SCFCR_RTRG_8		0x0080
#define	  SCFCR_RTRG_14		0x00c0
#define	 SCFCR_TTRG		0x0030	/* transmit trigger */
#define	  SCFCR_TTRG_8		0x0000
#define	  SCFCR_TTRG_4		0x0010
#define	  SCFCR_TTRG_2		0x0020
#define	  SCFCR_TTRG_0		0x0030
#define	 SCFCR_MCE		0x0008	/* modem control enable */
#define	 SCFCR_TFRST		0x0004	/* transmit FIFO reset */
#define	 SCFCR_RFRST		0x0002	/* receive FIFO reset */
#define	 SCFCR_LOOP		0x0001	/* loopback */
#define	SCIF_SCFDR		0x0e	/* 16: FIFO data count */
#define	 SCFDR_TXCNT		0x1f00
#define	 SCFDR_RXCNT		0x001f
#define	SCIF_SCSPTR		0x10	/* 16: serial port */
#define	SCIF_SCLSR		0x12	/* 16: line status */
#define	 SCLSR_ORER		0x0001	/* overrun error */
#define	SCIF_SEMR		0x14	/* 8: serial extended mode */
#define	 SEMR_MDDRS		0x10	/* BRR address selects MDDR */
#define	SCIF_FTCR		0x16	/* 16: FIFO trigger control */
#define	SCIF_REGSIZE		0x18

#define	SCIF_FIFO_LEN		16

#endif /* _DEV_FDT_RZ_SCIFREG_H_ */
