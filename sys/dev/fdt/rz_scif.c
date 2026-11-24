// SPDX-License-Identifier: BSD-2-Clause
/*
 * Renesas RZ/A1 SCIF driver.
 *
 * Renesas RZ/G2L family SCIF driver (RZ/Five R9A07G043F, RZ/G2UL), also
 * used for the SCIFA of the RZ/A2 (R7S9210), which has the same register
 * layout.
 *
 * Derived from the SuperH SCIF driver (sys/arch/sh3/dev/scif.c, rev 1.70),
 * itself derived from z8530tty.c and com.c. The tty and soft interrupt
 * logic is kept as is; the SH-specific fixed register accessors and the
 * INTC hookup are replaced by bus_space and FDT interrupts, and multiple
 * channels are supported.
 */

/*-
 * Copyright (C) 1999 T.Horiuchi and SAITOH Masanobu.  All rights reserved.
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

/*-
 * Copyright (c) 1998, 1999 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Charles M. Hannum.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Copyright (c) 1991 The Regents of the University of California.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)com.c	7.5 (Berkeley) 5/16/91
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/tty.h>
#include <sys/proc.h>
#include <sys/conf.h>
#include <sys/file.h>
#include <sys/syslog.h>
#include <sys/kernel.h>
#include <sys/device.h>
#include <sys/kmem.h>
#include <sys/kauth.h>
#include <sys/intr.h>
#include <sys/bus.h>

#include <dev/cons.h>

#include <dev/fdt/fdtvar.h>
#include <dev/fdt/fdt_console.h>

#include <dev/fdt/rz_scifreg.h>

struct rz_scif_softc {
	device_t sc_dev;
	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;
	u_int sc_frequency;

	struct tty *sc_tty;
	void *sc_si;

	callout_t sc_diag_ch;

	u_int sc_overflows,
	      sc_floods,
	      sc_errors;		/* number of retries so far */

	int sc_hwflags;
	int sc_swflags;
	u_int sc_fifolen;

	u_int sc_r_hiwat,
	      sc_r_lowat;
	u_char *volatile sc_rbget,
	       *volatile sc_rbput;
	volatile u_int sc_rbavail;
	u_char *sc_rbuf,
	       *sc_ebuf;

	u_char *sc_tba;			/* transmit buffer address */
	u_int sc_tbc,			/* transmit byte count */
	      sc_heldtbc;

	volatile u_char sc_rx_flags,
#define	RX_TTY_BLOCKED		0x01
#define	RX_TTY_OVERFLOWED	0x02
#define	RX_IBUF_BLOCKED		0x04
#define	RX_IBUF_OVERFLOWED	0x08
#define	RX_ANY_BLOCK		0x0f
			sc_tx_busy,	/* working on an output chunk */
			sc_tx_done,	/* done with one output chunk */
			sc_tx_stopped,	/* H/W level stop (lost CTS) */
			sc_rx_ready;

	volatile u_char sc_heldchange;
};

/* Console state, set up by rz_scif_cnattach() */
static struct {
	bus_space_tag_t bst;
	bus_space_handle_t bsh;
	bus_addr_t addr;
	u_int frequency;
	int speed;
	tcflag_t cflag;
} scif_cons;

#define	divrnd(n, q)	(((n)*2/(q)+1)/2)	/* divide and round off */

static int rz_scif_match(device_t, cfdata_t, void *);
static void rz_scif_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(rz_scif, sizeof(struct rz_scif_softc),
    rz_scif_match, rz_scif_attach, NULL, NULL);

extern struct cfdriver scif_cd;

#define	SCIFUNIT(x)	TTUNIT(x)
#define	SCIFDIALOUT(x)	TTDIALOUT(x)

dev_type_open(scifopen);
dev_type_close(scifclose);
dev_type_read(scifread);
dev_type_write(scifwrite);
dev_type_ioctl(scifioctl);
dev_type_stop(scifstop);
dev_type_tty(sciftty);
dev_type_poll(scifpoll);

const struct cdevsw scif_cdevsw = {
	.d_open = scifopen,
	.d_close = scifclose,
	.d_read = scifread,
	.d_write = scifwrite,
	.d_ioctl = scifioctl,
	.d_stop = scifstop,
	.d_tty = sciftty,
	.d_poll = scifpoll,
	.d_mmap = nommap,
	.d_kqfilter = ttykqfilter,
	.d_discard = nodiscard,
	.d_flag = D_TTY
};

static int scifcngetc(dev_t);
static void scifcnputc(dev_t, int);
static void scifcnpollc(dev_t, int);

static struct consdev scif_consdev = {
	.cn_getc = scifcngetc,
	.cn_putc = scifcnputc,
	.cn_pollc = scifcnpollc,
	.cn_dev = NODEV,
	.cn_pri = CN_NORMAL,
};

static void scifstart(struct tty *);
static int scifparam(struct tty *, struct termios *);

static int scifintr(void *);
static void scifsoft(void *);
static void scif_rxsoft(struct rz_scif_softc *, struct tty *);
static void scif_txsoft(struct rz_scif_softc *, struct tty *);
static void scif_schedrx(struct rz_scif_softc *);
static void scifdiag(void *);
static void scif_break(struct rz_scif_softc *, int);
static void scif_iflush(struct rz_scif_softc *);

/* Hardware flag masks (sc_hwflags) */
#define	SCIF_HW_DEV_OK	0x20
#define	SCIF_HW_CONSOLE	0x40

/* Buffer size for character buffer */
#define	SCIF_RING_SIZE	2048
static u_int scif_rbuf_size = SCIF_RING_SIZE;

/* Stop input when 3/4 of the ring is full; restart when only 1/4 is full. */
static u_int scif_rbuf_hiwat = (SCIF_RING_SIZE * 1) / 4;
static u_int scif_rbuf_lowat = (SCIF_RING_SIZE * 3) / 4;

#define	CONMODE ((TTYDEF_CFLAG & ~(CSIZE | CSTOPB | PARENB)) | CS8) /* 8N1 */

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "renesas,scif-r9a07g044" },	/* RZ/G2L family layout */
	{ .compat = "renesas,scif-r7s9210" },	/* RZ/A2 SCIFA, same layout */
	DEVICE_COMPAT_EOL
};

/*
 * Register access. SCBRR, SCFTDR and SCFRDR are 8 bits wide, the rest
 * are 16 bits wide.
 */
#define	SCIF_READ_1(bst, bsh, r)	bus_space_read_1((bst), (bsh), (r))
#define	SCIF_WRITE_1(bst, bsh, r, v)	bus_space_write_1((bst), (bsh), (r), (v))
#define	SCIF_READ_2(bst, bsh, r)	bus_space_read_2((bst), (bsh), (r))
#define	SCIF_WRITE_2(bst, bsh, r, v)	bus_space_write_2((bst), (bsh), (r), (v))

#define	sc_read_1(sc, r)	SCIF_READ_1((sc)->sc_bst, (sc)->sc_bsh, (r))
#define	sc_write_1(sc, r, v)	SCIF_WRITE_1((sc)->sc_bst, (sc)->sc_bsh, (r), (v))
#define	sc_read_2(sc, r)	SCIF_READ_2((sc)->sc_bst, (sc)->sc_bsh, (r))
#define	sc_write_2(sc, r, v)	SCIF_WRITE_2((sc)->sc_bst, (sc)->sc_bsh, (r), (v))

static u_int
scif_brr(u_int frequency, int speed)
{
	/* Asynchronous mode, CKS = 0 (P1 clock), 16x oversampling */
	return divrnd(frequency, 32 * speed) - 1;
}

static void
scif_init_regs(bus_space_tag_t bst, bus_space_handle_t bsh, u_int frequency,
    int speed)
{
	SCIF_WRITE_2(bst, bsh, SCIF_SCSCR, 0);
	SCIF_WRITE_2(bst, bsh, SCIF_SCFCR, SCFCR_TFRST | SCFCR_RFRST);
	(void)SCIF_READ_2(bst, bsh, SCIF_SCFSR);
	SCIF_WRITE_2(bst, bsh, SCIF_SCFSR, 0);
	SCIF_WRITE_2(bst, bsh, SCIF_SCLSR, 0);
	SCIF_WRITE_1(bst, bsh, SCIF_SEMR,
	    SCIF_READ_1(bst, bsh, SCIF_SEMR) & ~SEMR_MDDRS);
	if (frequency != 0) {
		SCIF_WRITE_2(bst, bsh, SCIF_SCSMR, 0);
		SCIF_WRITE_1(bst, bsh, SCIF_SCBRR, scif_brr(frequency, speed));
	} else {
		SCIF_WRITE_2(bst, bsh, SCIF_SCSMR,
		    SCIF_READ_2(bst, bsh, SCIF_SCSMR) & SCSMR_CKS);
	}

	delay(2000);

	SCIF_WRITE_2(bst, bsh, SCIF_SCFCR, SCFCR_RTRG_14 | SCFCR_TTRG_0);
	SCIF_WRITE_2(bst, bsh, SCIF_SCSCR, SCSCR_TE | SCSCR_RE);
}

static void
scif_putc_raw(bus_space_tag_t bst, bus_space_handle_t bsh, u_char c)
{
	/* wait for room in the FIFO */
	while (__SHIFTOUT(SCIF_READ_2(bst, bsh, SCIF_SCFDR), SCFDR_TXCNT) >=
	    SCIF_FIFO_LEN)
		continue;

	SCIF_WRITE_1(bst, bsh, SCIF_SCFTDR, c);

	/* clear ready flag */
	SCIF_WRITE_2(bst, bsh, SCIF_SCFSR,
	    SCIF_READ_2(bst, bsh, SCIF_SCFSR) & ~(SCFSR_TDFE | SCFSR_TEND));
}

static int
scif_getc_raw(bus_space_tag_t bst, bus_space_handle_t bsh, bool wait)
{
	uint16_t fsr, lsr;
	u_char c;

	for (;;) {
		if ((SCIF_READ_2(bst, bsh, SCIF_SCFDR) & SCFDR_RXCNT) == 0) {
			if (!wait)
				return -1;
			continue;
		}

		c = SCIF_READ_1(bst, bsh, SCIF_SCFRDR);
		fsr = SCIF_READ_2(bst, bsh, SCIF_SCFSR);
		SCIF_WRITE_2(bst, bsh, SCIF_SCFSR,
		    fsr & ~(SCFSR_ER | SCFSR_BRK | SCFSR_RDF | SCFSR_DR));
		lsr = SCIF_READ_2(bst, bsh, SCIF_SCLSR);
		SCIF_WRITE_2(bst, bsh, SCIF_SCLSR, lsr & ~SCLSR_ORER);

		if ((fsr & (SCFSR_ER | SCFSR_BRK | SCFSR_FER | SCFSR_PER)) == 0 &&
		    (lsr & SCLSR_ORER) == 0)
			return c;
	}
}

static void
scif_putc(struct rz_scif_softc *sc, u_char c)
{
	scif_putc_raw(sc->sc_bst, sc->sc_bsh, c);
}

static int
rz_scif_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
rz_scif_attach(device_t parent, device_t self, void *aux)
{
	struct rz_scif_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	char intrstr[128];
	struct clk *clk;
	bus_addr_t addr;
	bus_size_t size;
	struct tty *tp;
	int i, nintr;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	sc->sc_hwflags = 0;
	sc->sc_swflags = 0;
	sc->sc_fifolen = SCIF_FIFO_LEN;

	if (scif_cons.bst != NULL && scif_cons.addr == addr) {
		sc->sc_bsh = scif_cons.bsh;
		sc->sc_frequency = scif_cons.frequency;
		SET(sc->sc_hwflags, SCIF_HW_CONSOLE);
		SET(sc->sc_swflags, TIOCFLAG_SOFTCAR);
	} else {
		if (bus_space_map(sc->sc_bst, addr, size, 0,
		    &sc->sc_bsh) != 0) {
			aprint_error(": couldn't map registers\n");
			return;
		}
		clk = fdtbus_clock_get(phandle, "fck");
		if (clk == NULL)
			clk = fdtbus_clock_get_index(phandle, 0);
		if (clk != NULL && clk_enable(clk) != 0) {
			aprint_error(": couldn't enable clock\n");
			return;
		}
		sc->sc_frequency = clk != NULL ? clk_get_rate(clk) : 0;
		scif_init_regs(sc->sc_bst, sc->sc_bsh, sc->sc_frequency,
		    TTYDEF_SPEED);
	}

	aprint_naive("\n");
	aprint_normal(": RZ/G2L SCIF\n");
	if (ISSET(sc->sc_hwflags, SCIF_HW_CONSOLE)) {
		int maj = cdevsw_lookup_major(&scif_cdevsw);

		cn_tab->cn_dev = makedev(maj, device_unit(self));
		aprint_normal_dev(self, "console\n");
	}

	callout_init(&sc->sc_diag_ch, 0);

	/* ERI, RXI, TXI and BRI all go to the same handler. */
	nintr = 0;
	for (i = 0; i < 6; i++) {
		if (!fdtbus_intr_str(phandle, i, intrstr, sizeof(intrstr)))
			break;
		bool dup = false;
		for (int j = 0; j < i; j++) {
			char prev[128];
			if (fdtbus_intr_str(phandle, j, prev, sizeof(prev)) &&
			    strcmp(prev, intrstr) == 0)
				dup = true;
		}
		if (dup)
			continue;
		if (fdtbus_intr_establish_xname(phandle, i, IPL_SERIAL, 0,
		    scifintr, sc, device_xname(self)) == NULL) {
			aprint_error_dev(self,
			    "failed to establish interrupt on %s\n", intrstr);
			return;
		}
		if (nintr++ == 0)
			aprint_normal_dev(self, "interrupting on %s", intrstr);
		else
			aprint_normal(", %s", intrstr);
	}
	if (nintr > 0)
		aprint_normal("\n");

	sc->sc_si = softint_establish(SOFTINT_SERIAL, scifsoft, sc);
	SET(sc->sc_hwflags, SCIF_HW_DEV_OK);

	tp = tty_alloc();
	tp->t_oproc = scifstart;
	tp->t_param = scifparam;
	tp->t_hwiflow = NULL;

	sc->sc_tty = tp;
	sc->sc_rbuf = kmem_alloc(scif_rbuf_size << 1, KM_SLEEP);
	sc->sc_ebuf = sc->sc_rbuf + (scif_rbuf_size << 1);

	tty_attach(tp);

	if (!pmf_device_register(self, NULL, NULL))
		aprint_error_dev(self, "unable to establish power handler\n");
}

/*
 * Start or restart transmission.
 */
static void
scifstart(struct tty *tp)
{
	struct rz_scif_softc *sc;
	int s;

	sc = device_lookup_private(&scif_cd, SCIFUNIT(tp->t_dev));

	s = spltty();
	if (ISSET(tp->t_state, TS_BUSY | TS_TIMEOUT | TS_TTSTOP))
		goto out;
	if (sc->sc_tx_stopped)
		goto out;
	if (!ttypull(tp))
		goto out;

	/* Grab the first contiguous region of buffer space. */
	{
		u_char *tba;
		int tbc;

		tba = tp->t_outq.c_cf;
		tbc = ndqb(&tp->t_outq, 0);

		(void)splserial();

		sc->sc_tba = tba;
		sc->sc_tbc = tbc;
	}

	SET(tp->t_state, TS_BUSY);
	sc->sc_tx_busy = 1;

	/* Enable transmit completion interrupts if necessary. */
	sc_write_2(sc, SCIF_SCSCR,
	    sc_read_2(sc, SCIF_SCSCR) | SCSCR_TIE | SCSCR_RIE);

	/* Output the first chunk of the contiguous buffer. */
	{
		int n;
		int maxchars;
		int i;

		n = sc->sc_tbc;
		maxchars = sc->sc_fifolen
			- __SHIFTOUT(sc_read_2(sc, SCIF_SCFDR), SCFDR_TXCNT);
		if (n > maxchars)
			n = maxchars;

		for (i = 0; i < n; i++) {
			scif_putc(sc, *(sc->sc_tba));
			sc->sc_tba++;
		}
		sc->sc_tbc -= n;
	}
out:
	splx(s);
	return;
}

/*
 * Set SCIF tty parameters from termios.
 */
static int
scifparam(struct tty *tp, struct termios *t)
{
	struct rz_scif_softc *sc;
	int ospeed = t->c_ospeed;
	uint16_t smr;
	int s;

	sc = device_lookup_private(&scif_cd, SCIFUNIT(tp->t_dev));
	if (!device_is_active(sc->sc_dev))
		return (EIO);

	/* Check requested parameters. */
	if (ospeed <= 0)
		return (EINVAL);
	if (t->c_ispeed && t->c_ispeed != t->c_ospeed)
		return (EINVAL);
	if (scif_brr(sc->sc_frequency, ospeed) > 0xff)
		return (EINVAL);

	/*
	 * For the console, always force CLOCAL and !HUPCL, so that the port
	 * is always active.
	 */
	if (ISSET(sc->sc_swflags, TIOCFLAG_SOFTCAR) ||
	    ISSET(sc->sc_hwflags, SCIF_HW_CONSOLE)) {
		SET(t->c_cflag, CLOCAL);
		CLR(t->c_cflag, HUPCL);
	}

	/*
	 * If there were no changes, don't do anything.  This avoids dropping
	 * input and improves performance when all we did was frob things like
	 * VMIN and VTIME.
	 */
	if (tp->t_ospeed == t->c_ospeed &&
	    tp->t_cflag == t->c_cflag)
		return (0);

	s = splserial();

	/*
	 * Set the flow control pins depending on the current flow control
	 * mode.
	 */
	if (ISSET(t->c_cflag, CRTSCTS)) {
		sc_write_2(sc, SCIF_SCFCR,
		    sc_read_2(sc, SCIF_SCFCR) | SCFCR_MCE);
	} else {
		sc_write_2(sc, SCIF_SCFCR,
		    sc_read_2(sc, SCIF_SCFCR) & ~SCFCR_MCE);
	}

	/* Character format */
	smr = sc_read_2(sc, SCIF_SCSMR) &
	    ~(SCSMR_CHR | SCSMR_PE | SCSMR_OE | SCSMR_STOP);
	if ((t->c_cflag & CSIZE) == CS7)
		smr |= SCSMR_CHR;
	if (ISSET(t->c_cflag, PARENB)) {
		smr |= SCSMR_PE;
		if (ISSET(t->c_cflag, PARODD))
			smr |= SCSMR_OE;
	}
	if (ISSET(t->c_cflag, CSTOPB))
		smr |= SCSMR_STOP;
	sc_write_2(sc, SCIF_SCSMR, smr);

	sc_write_1(sc, SCIF_SCBRR, scif_brr(sc->sc_frequency, ospeed));

	/* And copy to tty. */
	tp->t_ispeed = 0;
	tp->t_ospeed = t->c_ospeed;
	tp->t_cflag = t->c_cflag;

	if (!sc->sc_heldchange) {
		if (sc->sc_tx_busy) {
			sc->sc_heldtbc = sc->sc_tbc;
			sc->sc_tbc = 0;
			sc->sc_heldchange = 1;
		}
	}

	if (!ISSET(t->c_cflag, CHWFLOW)) {
		/* Disable the high water mark. */
		sc->sc_r_hiwat = 0;
		sc->sc_r_lowat = 0;
		if (ISSET(sc->sc_rx_flags, RX_TTY_OVERFLOWED)) {
			CLR(sc->sc_rx_flags, RX_TTY_OVERFLOWED);
			scif_schedrx(sc);
		}
	} else {
		sc->sc_r_hiwat = scif_rbuf_hiwat;
		sc->sc_r_lowat = scif_rbuf_lowat;
	}

	splx(s);

	if (!ISSET(t->c_cflag, CHWFLOW)) {
		if (sc->sc_tx_stopped) {
			sc->sc_tx_stopped = 0;
			scifstart(tp);
		}
	}

	return (0);
}

static void
scif_iflush(struct rz_scif_softc *sc)
{
	int i;

	i = sc_read_2(sc, SCIF_SCFDR) & SCFDR_RXCNT;

	while (i > 0) {
		(void)sc_read_1(sc, SCIF_SCFRDR);
		sc_write_2(sc, SCIF_SCFSR,
		    sc_read_2(sc, SCIF_SCFSR) & ~(SCFSR_RDF | SCFSR_DR));
		i--;
	}
}

int
scifopen(dev_t dev, int flag, int mode, struct lwp *l)
{
	struct rz_scif_softc *sc;
	struct tty *tp;
	int s, s2;
	int error;

	sc = device_lookup_private(&scif_cd, SCIFUNIT(dev));
	if (sc == NULL || !ISSET(sc->sc_hwflags, SCIF_HW_DEV_OK) ||
	    sc->sc_rbuf == NULL)
		return (ENXIO);

	if (!device_is_active(sc->sc_dev))
		return (ENXIO);

	tp = sc->sc_tty;

	if (kauth_authorize_device_tty(l->l_cred, KAUTH_DEVICE_TTY_OPEN, tp))
		return (EBUSY);

	s = spltty();

	/*
	 * Do the following iff this is a first open.
	 */
	if (!ISSET(tp->t_state, TS_ISOPEN) && tp->t_wopen == 0) {
		struct termios t;

		tp->t_dev = dev;

		s2 = splserial();

		/* Turn on interrupts. */
		sc_write_2(sc, SCIF_SCSCR,
		    sc_read_2(sc, SCIF_SCSCR) | SCSCR_TIE | SCSCR_RIE);

		splx(s2);

		/*
		 * Initialize the termios status to the defaults.  Add in the
		 * sticky bits from TIOCSFLAGS.
		 */
		t.c_ispeed = 0;
		if (ISSET(sc->sc_hwflags, SCIF_HW_CONSOLE)) {
			t.c_ospeed = scif_cons.speed;
			t.c_cflag = scif_cons.cflag;
		} else {
			t.c_ospeed = TTYDEF_SPEED;
			t.c_cflag = TTYDEF_CFLAG;
		}
		if (ISSET(sc->sc_swflags, TIOCFLAG_CLOCAL))
			SET(t.c_cflag, CLOCAL);
		if (ISSET(sc->sc_swflags, TIOCFLAG_CRTSCTS))
			SET(t.c_cflag, CRTSCTS);
		if (ISSET(sc->sc_swflags, TIOCFLAG_MDMBUF))
			SET(t.c_cflag, MDMBUF);
		/* Make sure scifparam() will do something. */
		tp->t_ospeed = 0;
		(void) scifparam(tp, &t);
		tp->t_iflag = TTYDEF_IFLAG;
		tp->t_oflag = TTYDEF_OFLAG;
		tp->t_lflag = TTYDEF_LFLAG;
		ttychars(tp);
		ttsetwater(tp);

		s2 = splserial();

		/* Clear the input ring, and unblock. */
		sc->sc_rbput = sc->sc_rbget = sc->sc_rbuf;
		sc->sc_rbavail = scif_rbuf_size;
		scif_iflush(sc);
		CLR(sc->sc_rx_flags, RX_ANY_BLOCK);

		splx(s2);
	}

	splx(s);

	error = ttyopen(tp, SCIFDIALOUT(dev), ISSET(flag, O_NONBLOCK));
	if (error)
		goto bad;

	error = (*tp->t_linesw->l_open)(dev, tp);
	if (error)
		goto bad;

	return (0);

bad:

	return (error);
}

int
scifclose(dev_t dev, int flag, int mode, struct lwp *l)
{
	struct rz_scif_softc *sc;
	struct tty *tp;

	sc = device_lookup_private(&scif_cd, SCIFUNIT(dev));
	tp = sc->sc_tty;

	/* XXX This is for cons.c. */
	if (!ISSET(tp->t_state, TS_ISOPEN))
		return (0);

	(*tp->t_linesw->l_close)(tp, flag);
	ttyclose(tp);

	return (0);
}

int
scifread(dev_t dev, struct uio *uio, int flag)
{
	struct rz_scif_softc *sc;
	struct tty *tp;

	sc = device_lookup_private(&scif_cd, SCIFUNIT(dev));
	tp = sc->sc_tty;

	return ((*tp->t_linesw->l_read)(tp, uio, flag));
}

int
scifwrite(dev_t dev, struct uio *uio, int flag)
{
	struct rz_scif_softc *sc;
	struct tty *tp;

	sc = device_lookup_private(&scif_cd, SCIFUNIT(dev));
	tp = sc->sc_tty;

	return ((*tp->t_linesw->l_write)(tp, uio, flag));
}

int
scifpoll(dev_t dev, int events, struct lwp *l)
{
	struct rz_scif_softc *sc;
	struct tty *tp;

	sc = device_lookup_private(&scif_cd, SCIFUNIT(dev));
	tp = sc->sc_tty;

	return ((*tp->t_linesw->l_poll)(tp, events, l));
}

struct tty *
sciftty(dev_t dev)
{
	struct rz_scif_softc *sc;

	sc = device_lookup_private(&scif_cd, SCIFUNIT(dev));

	return (sc->sc_tty);
}

int
scifioctl(dev_t dev, u_long cmd, void *data, int flag, struct lwp *l)
{
	struct rz_scif_softc *sc;
	struct tty *tp;
	int error;
	int s;

	sc = device_lookup_private(&scif_cd, SCIFUNIT(dev));
	if (!device_is_active(sc->sc_dev))
		return (EIO);

	tp = sc->sc_tty;
	error = (*tp->t_linesw->l_ioctl)(tp, cmd, data, flag, l);
	if (error != EPASSTHROUGH)
		return (error);

	error = ttioctl(tp, cmd, data, flag, l);
	if (error != EPASSTHROUGH)
		return (error);

	error = 0;

	s = splserial();

	switch (cmd) {
	case TIOCSBRK:
		scif_break(sc, 1);
		break;

	case TIOCCBRK:
		scif_break(sc, 0);
		break;

	case TIOCGFLAGS:
		*(int *)data = sc->sc_swflags;
		break;

	case TIOCSFLAGS:
		error = kauth_authorize_device_tty(l->l_cred,
		    KAUTH_DEVICE_TTY_PRIVSET, tp);
		if (error)
			break;
		sc->sc_swflags = *(int *)data;
		break;

	default:
		error = EPASSTHROUGH;
		break;
	}

	splx(s);

	return (error);
}

static void
scif_schedrx(struct rz_scif_softc *sc)
{

	sc->sc_rx_ready = 1;

	softint_schedule(sc->sc_si);
}

static void
scif_break(struct rz_scif_softc *sc, int onoff)
{

	if (onoff)
		sc_write_2(sc, SCIF_SCFSR,
		    sc_read_2(sc, SCIF_SCFSR) & ~SCFSR_TDFE);
	else
		sc_write_2(sc, SCIF_SCFSR,
		    sc_read_2(sc, SCIF_SCFSR) | SCFSR_TDFE);
}

/*
 * Stop output, e.g., for ^S or output flush.
 */
void
scifstop(struct tty *tp, int flag)
{
	struct rz_scif_softc *sc;
	int s;

	sc = device_lookup_private(&scif_cd, SCIFUNIT(tp->t_dev));

	s = splserial();
	if (ISSET(tp->t_state, TS_BUSY)) {
		/* Stop transmitting at the next chunk. */
		sc->sc_tbc = 0;
		sc->sc_heldtbc = 0;
		if (!ISSET(tp->t_state, TS_TTSTOP))
			SET(tp->t_state, TS_FLUSH);
	}
	splx(s);
}

static void
scifdiag(void *arg)
{
	struct rz_scif_softc *sc = arg;
	int overflows, floods;
	int s;

	s = splserial();
	overflows = sc->sc_overflows;
	sc->sc_overflows = 0;
	floods = sc->sc_floods;
	sc->sc_floods = 0;
	sc->sc_errors = 0;
	splx(s);

	log(LOG_WARNING, "%s: %d silo overflow%s, %d ibuf flood%s\n",
	    device_xname(sc->sc_dev),
	    overflows, overflows == 1 ? "" : "s",
	    floods, floods == 1 ? "" : "s");
}

static void
scif_rxsoft(struct rz_scif_softc *sc, struct tty *tp)
{
	int (*rint)(int, struct tty *) = tp->t_linesw->l_rint;
	u_char *get, *end;
	u_int cc, scc;
	u_char fsr;
	int code;
	int s;

	end = sc->sc_ebuf;
	get = sc->sc_rbget;
	scc = cc = scif_rbuf_size - sc->sc_rbavail;

	if (cc == scif_rbuf_size) {
		sc->sc_floods++;
		if (sc->sc_errors++ == 0)
			callout_reset(&sc->sc_diag_ch, 60 * hz, scifdiag, sc);
	}

	while (cc) {
		code = get[0];
		fsr = get[1];
		if (ISSET(fsr, SCFSR_BRK | SCFSR_FER | SCFSR_PER)) {
			if (ISSET(fsr, SCFSR_BRK | SCFSR_FER))
				SET(code, TTY_FE);
			if (ISSET(fsr, SCFSR_PER))
				SET(code, TTY_PE);
		}
		if ((*rint)(code, tp) == -1) {
			/*
			 * The line discipline's buffer is out of space.
			 */
			if (!ISSET(sc->sc_rx_flags, RX_TTY_BLOCKED)) {
				/*
				 * We're either not using flow control, or the
				 * line discipline didn't tell us to block for
				 * some reason.  Either way, we have no way to
				 * know when there's more space available, so
				 * just drop the rest of the data.
				 */
				get += cc << 1;
				if (get >= end)
					get -= scif_rbuf_size << 1;
				cc = 0;
			} else {
				/*
				 * Don't schedule any more receive processing
				 * until the line discipline tells us there's
				 * space available.
				 * Leave the rest of the data in the input
				 * buffer.
				 */
				SET(sc->sc_rx_flags, RX_TTY_OVERFLOWED);
			}
			break;
		}
		get += 2;
		if (get >= end)
			get = sc->sc_rbuf;
		cc--;
	}

	if (cc != scc) {
		sc->sc_rbget = get;
		s = splserial();
		cc = sc->sc_rbavail += scc - cc;
		/* Buffers should be ok again, release possible block. */
		if (cc >= sc->sc_r_lowat) {
			if (ISSET(sc->sc_rx_flags, RX_IBUF_OVERFLOWED)) {
				CLR(sc->sc_rx_flags, RX_IBUF_OVERFLOWED);
				sc_write_2(sc, SCIF_SCSCR,
				    sc_read_2(sc, SCIF_SCSCR) | SCSCR_RIE);
			}
		}
		splx(s);
	}
}

static void
scif_txsoft(struct rz_scif_softc *sc, struct tty *tp)
{

	CLR(tp->t_state, TS_BUSY);
	if (ISSET(tp->t_state, TS_FLUSH))
		CLR(tp->t_state, TS_FLUSH);
	else
		ndflush(&tp->t_outq, (int)(sc->sc_tba - tp->t_outq.c_cf));
	(*tp->t_linesw->l_start)(tp);
}

static void
scifsoft(void *arg)
{
	struct rz_scif_softc *sc = arg;
	struct tty *tp;

	if (!device_is_active(sc->sc_dev))
		return;

	tp = sc->sc_tty;

	if (sc->sc_rx_ready) {
		sc->sc_rx_ready = 0;
		scif_rxsoft(sc, tp);
	}

	if (sc->sc_tx_done) {
		sc->sc_tx_done = 0;
		scif_txsoft(sc, tp);
	}
}

static int
scifintr(void *arg)
{
	struct rz_scif_softc *sc = arg;
	u_char *put, *end;
	u_int cc;
	uint16_t fsr;
	int count;

	if (!device_is_active(sc->sc_dev))
		return (0);

	end = sc->sc_ebuf;
	put = sc->sc_rbput;
	cc = sc->sc_rbavail;

	do {
		fsr = sc_read_2(sc, SCIF_SCFSR);
		if (ISSET(fsr, SCFSR_BRK)) {
			sc_write_2(sc, SCIF_SCFSR, sc_read_2(sc, SCIF_SCFSR)
			    & ~(SCFSR_ER | SCFSR_BRK | SCFSR_DR));
#ifdef DDB
			if (ISSET(sc->sc_hwflags, SCIF_HW_CONSOLE)) {
				console_debugger();
			}
#endif /* DDB */
		}
		count = sc_read_2(sc, SCIF_SCFDR) & SCFDR_RXCNT;
		if (count != 0) {
			for (;;) {
				u_char c = sc_read_1(sc, SCIF_SCFRDR);
				u_char err =
				    (u_char)(sc_read_2(sc, SCIF_SCFSR) & 0x00ff);

				sc_write_2(sc, SCIF_SCFSR,
				    sc_read_2(sc, SCIF_SCFSR)
				    & ~(SCFSR_ER | SCFSR_RDF | SCFSR_DR));
				if (sc_read_2(sc, SCIF_SCLSR) & SCLSR_ORER) {
					sc->sc_overflows++;
					sc_write_2(sc, SCIF_SCLSR, 0);
				}
				if ((cc > 0) && (count > 0)) {
					put[0] = c;
					put[1] = err;
					put += 2;
					if (put >= end)
						put = sc->sc_rbuf;
					cc--;
					count--;
				} else
					break;
			}

			/*
			 * Current string of incoming characters ended because
			 * no more data was available or we ran out of space.
			 * Schedule a receive event if any data was received.
			 * If we're out of space, turn off receive interrupts.
			 */
			sc->sc_rbput = put;
			sc->sc_rbavail = cc;
			if (!ISSET(sc->sc_rx_flags, RX_TTY_OVERFLOWED))
				sc->sc_rx_ready = 1;

			/*
			 * See if we are in danger of overflowing a buffer.
			 */
			if (!ISSET(sc->sc_rx_flags, RX_IBUF_BLOCKED) &&
			    cc < sc->sc_r_hiwat) {
				SET(sc->sc_rx_flags, RX_IBUF_BLOCKED);
			}

			/*
			 * If we're out of space, disable receive interrupts
			 * until the queue has drained a bit.
			 */
			if (!cc) {
				SET(sc->sc_rx_flags, RX_IBUF_OVERFLOWED);
				sc_write_2(sc, SCIF_SCSCR,
				    sc_read_2(sc, SCIF_SCSCR) & ~SCSCR_RIE);
				break;
			}
		} else {
			if (sc_read_2(sc, SCIF_SCFSR) & (SCFSR_RDF | SCFSR_DR)) {
				/* Stale flags with an empty FIFO; clear them. */
				sc_write_2(sc, SCIF_SCFSR,
				    sc_read_2(sc, SCIF_SCFSR)
				    & ~(SCFSR_ER | SCFSR_RDF | SCFSR_DR));
				continue;
			}
		}
	} while (sc_read_2(sc, SCIF_SCFSR) & (SCFSR_RDF | SCFSR_DR));

	/*
	 * Done handling any receive interrupts. See if data can be
	 * transmitted as well. Schedule tx done event if no data left
	 * and tty was marked busy.
	 */
	if (__SHIFTOUT(sc_read_2(sc, SCIF_SCFDR), SCFDR_TXCNT) <
	    sc->sc_fifolen) {
		/*
		 * If we've delayed a parameter change, do it now, and restart
		 * output.
		 */
		if (sc->sc_heldchange) {
			sc->sc_heldchange = 0;
			sc->sc_tbc = sc->sc_heldtbc;
			sc->sc_heldtbc = 0;
		}

		/* Output the next chunk of the contiguous buffer, if any. */
		if (sc->sc_tbc > 0) {
			int n;
			int maxchars;
			int i;

			n = sc->sc_tbc;
			maxchars = sc->sc_fifolen -
			    __SHIFTOUT(sc_read_2(sc, SCIF_SCFDR), SCFDR_TXCNT);
			if (n > maxchars)
				n = maxchars;

			for (i = 0; i < n; i++) {
				scif_putc(sc, *(sc->sc_tba));
				sc->sc_tba++;
			}
			sc->sc_tbc -= n;
		} else {
			/* Disable transmit completion interrupts. */
			sc_write_2(sc, SCIF_SCSCR,
			    sc_read_2(sc, SCIF_SCSCR) & ~SCSCR_TIE);

			if (sc->sc_tx_busy) {
				sc->sc_tx_busy = 0;
				sc->sc_tx_done = 1;
			}
		}
	}

	/* Wake up the poller. */
	softint_schedule(sc->sc_si);

	return (1);
}

/*
 * Console support
 */

static int
scifcngetc(dev_t dev)
{
	int c;
	int s;

	s = splserial();
	c = scif_getc_raw(scif_cons.bst, scif_cons.bsh, true);
	splx(s);

	return (c);
}

static void
scifcnputc(dev_t dev, int c)
{
	int s;

	s = splserial();
	scif_putc_raw(scif_cons.bst, scif_cons.bsh, (u_char)c);
	splx(s);
}

static void
scifcnpollc(dev_t dev, int on)
{
}

static int
rz_scif_cnattach(bus_space_tag_t bst, bus_addr_t addr, u_int frequency,
    int speed, tcflag_t cflag)
{
	bus_space_handle_t bsh;
	int error;

	error = bus_space_map(bst, addr, SCIF_REGSIZE, 0, &bsh);
	if (error)
		return error;

	scif_cons.bst = bst;
	scif_cons.bsh = bsh;
	scif_cons.addr = addr;
	scif_cons.frequency = frequency;
	scif_cons.speed = speed;
	scif_cons.cflag = cflag;

	/* Let the boot loader's output drain before reprogramming. */
	for (u_int i = 0; i < 100000; i++) {
		if (SCIF_READ_2(bst, bsh, SCIF_SCFSR) & SCFSR_TEND)
			break;
	}
	scif_init_regs(bst, bsh, frequency, speed);

	cn_tab = &scif_consdev;

	return (0);
}

static int
rz_scif_console_match(int phandle)
{
	return of_compatible_match(phandle, compat_data);
}

static void
rz_scif_console_consinit(struct fdt_attach_args *faa, u_int uart_freq)
{
	const int phandle = faa->faa_phandle;
	bus_addr_t addr;
	tcflag_t flags;
	int speed;

	if (fdtbus_get_reg(phandle, 0, &addr, NULL) != 0)
		panic("rz_scif: couldn't get console registers");
	speed = fdtbus_get_stdout_speed();
	if (speed < 0)
		speed = 115200;	/* default */
	flags = fdtbus_get_stdout_flags();
	if (flags == 0)
		flags = CONMODE;

	if (rz_scif_cnattach(faa->faa_bst, addr, uart_freq, speed, flags))
		panic("rz_scif: cannot initialize console");
}

static const struct fdt_console rz_scif_console = {
	.match = rz_scif_console_match,
	.consinit = rz_scif_console_consinit,
};

FDT_CONSOLE(rz_scif, &rz_scif_console);
