// SPDX-License-Identifier: BSD-2-Clause
/*
 * RZ/A1 Serial Communication Interface with FIFO (SCIF)
 *
 * The register layout and bit assignments are those of the SH-4 SCIF
 * (sys/arch/sh3/include/scifreg.h), only the access method differs.
 */

/*-
 * Copyright (C) 1999 SAITOH Masanobu.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _ARM_RZA1_RZA1_SCIFREG_H_
#define _ARM_RZA1_RZA1_SCIFREG_H_

#define	SCIF_SCSMR		0x00	/* 16: serial mode */
#define	 SCSMR_CHR		0x0040	/* 7-bit characters */
#define	 SCSMR_PE		0x0020	/* parity enable */
#define	 SCSMR_OE		0x0010	/* odd parity */
#define	 SCSMR_STOP		0x0008	/* 2 stop bits */
#define	 SCSMR_CKS		0x0003	/* clock select */
#define	SCIF_SCBRR		0x04	/* 8: bit rate */
#define	SCIF_SCSCR		0x08	/* 16: serial control */
#define	 SCSCR_TIE		0x0080	/* transmit interrupt enable */
#define	 SCSCR_RIE		0x0040	/* receive interrupt enable */
#define	 SCSCR_TE		0x0020	/* transmit enable */
#define	 SCSCR_RE		0x0010	/* receive enable */
#define	 SCSCR_REIE		0x0008	/* receive error interrupt enable */
#define	 SCSCR_CKE		0x0003	/* clock enable */
#define	SCIF_SCFTDR		0x0c	/* 8: transmit FIFO data */
#define	SCIF_SCFSR		0x10	/* 16: serial status */
#define	 SCFSR_ER		0x0080	/* receive error */
#define	 SCFSR_TEND		0x0040	/* transmit end */
#define	 SCFSR_TDFE		0x0020	/* transmit FIFO data empty */
#define	 SCFSR_BRK		0x0010	/* break detect */
#define	 SCFSR_FER		0x0008	/* framing error */
#define	 SCFSR_PER		0x0004	/* parity error */
#define	 SCFSR_RDF		0x0002	/* receive FIFO data full */
#define	 SCFSR_DR		0x0001	/* receive data ready */
#define	SCIF_SCFRDR		0x14	/* 8: receive FIFO data */
#define	SCIF_SCFCR		0x18	/* 16: FIFO control */
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
#define	SCIF_SCFDR		0x1c	/* 16: FIFO data count */
#define	 SCFDR_TXCNT		0x1f00
#define	 SCFDR_RXCNT		0x001f
#define	SCIF_SCSPTR		0x20	/* 16: serial port */
#define	SCIF_SCLSR		0x24	/* 16: line status */
#define	 SCLSR_ORER		0x0001	/* overrun error */
#define	SCIF_SCEMR		0x28	/* 16: serial extended mode */

#define	SCIF_FIFO_LEN		16

#endif /* _ARM_RZA1_RZA1_SCIFREG_H_ */
