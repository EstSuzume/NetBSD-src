// SPDX-License-Identifier: BSD-2-Clause
/*
 * Renesas RZ/A1 Ethernet controller (ETHERC + E-DMAC, "SH-Ether").
 *
 * Notice: keep memory use and complexity low on a 10MiB system
 */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/device.h>
#include <sys/gpio.h>
#include <sys/intr.h>
#include <sys/kernel.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/systm.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_ether.h>
#include <net/if_media.h>

#include <dev/mii/mii.h>
#include <dev/mii/mii_bitbang.h>
#include <dev/mii/miivar.h>

#include <dev/fdt/fdtvar.h>

#include <arm/rza1/rza1_var.h>

/* E-MAC */
#define	ETH_ECMR	0x500
#define	 ECMR_PRM	__BIT(0)
#define	 ECMR_DM	__BIT(1)
#define	 ECMR_TE	__BIT(5)
#define	 ECMR_RE	__BIT(6)
#define	 ECMR_MCT	__BIT(13)
#define	ETH_RFLR	0x508
#define	ETH_ECSR	0x510
#define	ETH_ECSIPR	0x518
#define	ETH_PIR		0x520
#define	 PIR_MDC	__BIT(0)
#define	 PIR_MMD	__BIT(1)
#define	 PIR_MDO	__BIT(2)
#define	 PIR_MDI	__BIT(3)
#define	ETH_APR		0x554
#define	ETH_MPR		0x558
#define	ETH_TPAUSER	0x564
#define	ETH_MAHR	0x5c0
#define	ETH_MALR	0x5c8
#define	ETH_ARSTR	0x1800
#define	 ARSTR_ARST	__BIT(0)
#define	ETH_TSU_TEN	0x1864

/* E-DMAC */
#define	ETH_EDSR	0x000
#define	 EDSR_ENR	__BIT(0)
#define	 EDSR_ENT	__BIT(1)
#define	ETH_TDLAR	0x010
#define	ETH_TDFAR	0x014
#define	ETH_TDFXR	0x018
#define	ETH_TDFFR	0x01c
#define	ETH_RDLAR	0x030
#define	ETH_RDFAR	0x034
#define	ETH_RDFXR	0x038
#define	ETH_RDFFR	0x03c
#define	ETH_EDMR	0x400
#define	 EDMR_SWRR	__BIT(0)
#define	 EDMR_SWRT	__BIT(1)
#define	 EDMR_DL_16	__SHIFTIN(0, __BITS(5,4))
#define	 EDMR_DE	__BIT(6)
#define	ETH_EDTRR	0x408
#define	 EDTRR_TR	__BITS(1,0)
#define	ETH_EDRRR	0x410
#define	 EDRRR_RR	__BIT(0)
#define	ETH_EESR	0x428
#define	 EESR_CERF	__BIT(0)
#define	 EESR_PRE	__BIT(1)
#define	 EESR_RTSF	__BIT(2)
#define	 EESR_RTLF	__BIT(3)
#define	 EESR_RRF	__BIT(4)
#define	 EESR_RFOF	__BIT(16)
#define	 EESR_RDE	__BIT(17)
#define	 EESR_FR	__BIT(18)
#define	 EESR_TFUF	__BIT(19)
#define	 EESR_TDE	__BIT(20)
#define	 EESR_TC0	__BIT(21)
#define	 EESR_ECI	__BIT(22)
#define	 EESR_TABT	__BIT(26)
#define	 EESR_TC1	__BIT(29)
#define	ETH_EESIPR	0x430
#define	ETH_TRSCER	0x438
#define	ETH_RMFCR	0x440
#define	ETH_TFTR	0x448
#define	ETH_FDR		0x450
#define	ETH_RMCR	0x458
#define	 RMCR_RNC	__BIT(0)
#define	ETH_RPADIR	0x460
#define	ETH_FCFTR	0x468
#define	ETH_CSMR	0x4e4

struct rza1eth_desc {
	volatile uint32_t d0;
	volatile uint32_t d1;
	volatile uint32_t d2;
	volatile uint32_t pad;
};

#define	TD0_TACT	__BIT(31)
#define	TD0_TDLE	__BIT(30)
#define	TD0_TFP_ONE	__BITS(29,28)
#define	TD0_TFE		__BIT(27)
#define	TD1_TDL(n)	((uint32_t)(n) << 16)

#define	RD0_RACT	__BIT(31)
#define	RD0_RDLE	__BIT(30)
#define	RD0_RFP		__BITS(29,28)
#define	 RD0_RFP_ONE	3
#define	RD0_RFE		__BIT(27)
#define	RD0_RFS		__BITS(25,16)
#define	 RD0_RFS_RMAF	__BIT(23)
#define	RD1_RBL(n)	((uint32_t)(n) << 16)
#define	RD1_RDL		__BITS(15,0)

#define	RZA1ETH_NTXDESC	4
#define	RZA1ETH_NRXDESC	8
#define	RZA1ETH_BUFSIZE	1536	/* multiple of 32, >= 1518 */

struct rza1eth_dma {
	struct rza1eth_desc txd[RZA1ETH_NTXDESC];
	struct rza1eth_desc rxd[RZA1ETH_NRXDESC];
	uint8_t txbuf[RZA1ETH_NTXDESC][RZA1ETH_BUFSIZE];
	uint8_t rxbuf[RZA1ETH_NRXDESC][RZA1ETH_BUFSIZE];
};

#define	DMA_OFF(m)	offsetof(struct rza1eth_dma, m)

struct rza1eth_softc {
	device_t sc_dev;
	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;
	bus_dma_tag_t sc_dmat;
	int sc_phandle;

	bus_dmamap_t sc_dmamap;
	bus_dma_segment_t sc_dmaseg;
	struct rza1eth_dma *sc_dma;
	bus_addr_t sc_dmapa;

	struct ethercom sc_ec;
	struct mii_data sc_mii;
	callout_t sc_tick_ch;
	struct fdtbus_gpio_pin *sc_phy_reset;
	uint8_t sc_enaddr[ETHER_ADDR_LEN];
	bool sc_fdx;

	u_int sc_txnext;
	u_int sc_txdirty;
	u_int sc_txfree;
	u_int sc_rxnext;
};

#define	RD4(sc, reg)		bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define	WR4(sc, reg, val)	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "renesas,ether-r7s72100" },
	DEVICE_COMPAT_EOL
};

static int	rza1eth_match(device_t, cfdata_t, void *);
static void	rza1eth_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(rza1_ether, sizeof(struct rza1eth_softc),
    rza1eth_match, rza1eth_attach, NULL, NULL);

static int	rza1eth_init(struct ifnet *);
static void	rza1eth_stop(struct ifnet *, int);
static void	rza1eth_start(struct ifnet *);
static int	rza1eth_ioctl(struct ifnet *, u_long, void *);
static void	rza1eth_watchdog(struct ifnet *);

static uint32_t
rza1eth_mii_bb_read(device_t self)
{
	struct rza1eth_softc * const sc = device_private(self);

	return RD4(sc, ETH_PIR);
}

static void
rza1eth_mii_bb_write(device_t self, uint32_t val)
{
	struct rza1eth_softc * const sc = device_private(self);

	WR4(sc, ETH_PIR, val);
}

static const struct mii_bitbang_ops rza1eth_mii_bb_ops = {
	rza1eth_mii_bb_read,
	rza1eth_mii_bb_write,
	{
		PIR_MDO,
		PIR_MDI,
		PIR_MDC,
		PIR_MMD,
		0,
	}
};

static int
rza1eth_mii_readreg(device_t self, int phy, int reg, uint16_t *val)
{
	return mii_bitbang_readreg(self, &rza1eth_mii_bb_ops, phy, reg, val);
}

static int
rza1eth_mii_writereg(device_t self, int phy, int reg, uint16_t val)
{
	return mii_bitbang_writereg(self, &rza1eth_mii_bb_ops, phy, reg, val);
}

static void
rza1eth_mii_statchg(struct ifnet *ifp)
{
	struct rza1eth_softc * const sc = ifp->if_softc;
	const bool fdx =
	    (IFM_OPTIONS(sc->sc_mii.mii_media_active) & IFM_FDX) != 0;
	uint32_t ecmr;

	if (fdx == sc->sc_fdx)
		return;
	sc->sc_fdx = fdx;

	/* ECMR must not be changed while transmission and reception are enabled */
	ecmr = RD4(sc, ETH_ECMR);
	WR4(sc, ETH_ECMR, ecmr & ~(ECMR_TE | ECMR_RE));
	if (fdx)
		ecmr |= ECMR_DM;
	else
		ecmr &= ~ECMR_DM;
	WR4(sc, ETH_ECMR, ecmr & ~(ECMR_TE | ECMR_RE));
	WR4(sc, ETH_ECMR, ecmr);
}

static void
rza1eth_tick(void *arg)
{
	struct rza1eth_softc * const sc = arg;
	int s;

	s = splnet();
	mii_tick(&sc->sc_mii);
	splx(s);

	callout_schedule(&sc->sc_tick_ch, hz);
}

static void
rza1eth_sync(struct rza1eth_softc *sc, bus_size_t off, bus_size_t len,
    int ops)
{
	bus_dmamap_sync(sc->sc_dmat, sc->sc_dmamap, off, len, ops);
}

static void
rza1eth_init_rings(struct rza1eth_softc *sc)
{
	struct rza1eth_dma * const dma = sc->sc_dma;
	u_int i;

	for (i = 0; i < RZA1ETH_NTXDESC; i++) {
		dma->txd[i].d0 = TD0_TFP_ONE |
		    (i == RZA1ETH_NTXDESC - 1 ? TD0_TDLE : 0);
		dma->txd[i].d1 = 0;
		dma->txd[i].d2 = sc->sc_dmapa + DMA_OFF(txbuf[i]);
	}
	for (i = 0; i < RZA1ETH_NRXDESC; i++) {
		dma->rxd[i].d1 = RD1_RBL(RZA1ETH_BUFSIZE);
		dma->rxd[i].d2 = sc->sc_dmapa + DMA_OFF(rxbuf[i]);
		dma->rxd[i].d0 = RD0_RACT |
		    (i == RZA1ETH_NRXDESC - 1 ? RD0_RDLE : 0);
	}
	rza1eth_sync(sc, 0, sizeof(*dma),
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	sc->sc_txnext = sc->sc_txdirty = 0;
	sc->sc_txfree = RZA1ETH_NTXDESC;
	sc->sc_rxnext = 0;
}

static void
rza1eth_reset(struct rza1eth_softc *sc)
{
	WR4(sc, ETH_ARSTR, ARSTR_ARST);
	/* The reset lasts 256 B-clock cycles; registers are inaccessible. */
	delay(10);

	WR4(sc, ETH_EDSR, EDSR_ENT | EDSR_ENR);
	WR4(sc, ETH_EDMR, EDMR_SWRR | EDMR_SWRT);
	for (int timo = 1000; timo > 0; timo--) {
		if ((RD4(sc, ETH_EDMR) & (EDMR_SWRR | EDMR_SWRT)) == 0)
			return;
		delay(10);
	}
	aprint_error_dev(sc->sc_dev, "E-DMAC reset timeout\n");
}

static int
rza1eth_init(struct ifnet *ifp)
{
	struct rza1eth_softc * const sc = ifp->if_softc;
	const uint8_t *ea = CLLADDR(ifp->if_sadl);
	const bus_addr_t txd = sc->sc_dmapa + DMA_OFF(txd);
	const bus_addr_t rxd = sc->sc_dmapa + DMA_OFF(rxd);
	uint32_t ecmr;
	int s;

	s = splnet();

	rza1eth_stop(ifp, 0);
	rza1eth_reset(sc);
	rza1eth_init_rings(sc);

	WR4(sc, ETH_TDLAR, txd);
	WR4(sc, ETH_TDFAR, txd);
	WR4(sc, ETH_TDFXR, txd + sizeof(struct rza1eth_desc) *
	    (RZA1ETH_NTXDESC - 1));
	WR4(sc, ETH_TDFFR, 1);
	WR4(sc, ETH_RDLAR, rxd);
	WR4(sc, ETH_RDFAR, rxd);
	WR4(sc, ETH_RDFXR, rxd + sizeof(struct rza1eth_desc) *
	    (RZA1ETH_NRXDESC - 1));
	WR4(sc, ETH_RDFFR, 1);

	WR4(sc, ETH_EDMR, EDMR_DE | EDMR_DL_16);
	WR4(sc, ETH_TRSCER, 0);
	WR4(sc, ETH_TFTR, 0);
	WR4(sc, ETH_FDR, 0x00000707);
	WR4(sc, ETH_RMCR, RMCR_RNC);
	WR4(sc, ETH_RPADIR, 0);
	WR4(sc, ETH_FCFTR, 0x00070007);
	WR4(sc, ETH_CSMR, 0);

	WR4(sc, ETH_MAHR, ((uint32_t)ea[0] << 24) | ((uint32_t)ea[1] << 16) |
	    ((uint32_t)ea[2] << 8) | ea[3]);
	WR4(sc, ETH_MALR, ((uint32_t)ea[4] << 8) | ea[5]);
	WR4(sc, ETH_RFLR, 0);
	WR4(sc, ETH_APR, 0);
	WR4(sc, ETH_MPR, 0);
	WR4(sc, ETH_TPAUSER, 0);
	WR4(sc, ETH_TSU_TEN, 0);
	WR4(sc, ETH_ECSIPR, 0);
	WR4(sc, ETH_ECSR, RD4(sc, ETH_ECSR));

	WR4(sc, ETH_EESR, RD4(sc, ETH_EESR));
	WR4(sc, ETH_EESIPR, EESR_FR | EESR_RDE | EESR_RFOF | EESR_TC0 |
	    EESR_TC1 | EESR_TABT | EESR_TFUF);

	WR4(sc, ETH_EDRRR, EDRRR_RR);
	ecmr = ECMR_TE | ECMR_RE;
	if (sc->sc_fdx)
		ecmr |= ECMR_DM;
	if (ifp->if_flags & IFF_PROMISC)
		ecmr |= ECMR_PRM;
	WR4(sc, ETH_ECMR, ecmr);

	ifp->if_flags |= IFF_RUNNING;
	ifp->if_timer = 0;

	mii_mediachg(&sc->sc_mii);
	callout_schedule(&sc->sc_tick_ch, hz);

	splx(s);

	rza1eth_start(ifp);
	return 0;
}

static void
rza1eth_stop(struct ifnet *ifp, int disable)
{
	struct rza1eth_softc * const sc = ifp->if_softc;

	callout_stop(&sc->sc_tick_ch);

	WR4(sc, ETH_EESIPR, 0);
	WR4(sc, ETH_ECMR, RD4(sc, ETH_ECMR) & ~(ECMR_TE | ECMR_RE));
	WR4(sc, ETH_EDRRR, 0);
	WR4(sc, ETH_EDTRR, 0);

	if (disable)
		mii_down(&sc->sc_mii);

	ifp->if_flags &= ~IFF_RUNNING;
	ifp->if_timer = 0;
}

static void
rza1eth_start(struct ifnet *ifp)
{
	struct rza1eth_softc * const sc = ifp->if_softc;
	struct rza1eth_dma * const dma = sc->sc_dma;
	struct mbuf *m;
	bool queued = false;
	u_int i, len;

	if ((ifp->if_flags & IFF_RUNNING) == 0)
		return;

	while (sc->sc_txfree > 0) {
		IFQ_DEQUEUE(&ifp->if_snd, m);
		if (m == NULL)
			break;

		i = sc->sc_txnext;
		len = m->m_pkthdr.len;
		if (len > RZA1ETH_BUFSIZE) {
			if_statinc(ifp, if_oerrors);
			m_freem(m);
			continue;
		}
		m_copydata(m, 0, len, dma->txbuf[i]);
		if (len < ETHER_MIN_LEN - ETHER_CRC_LEN) {
			memset(dma->txbuf[i] + len, 0,
			    ETHER_MIN_LEN - ETHER_CRC_LEN - len);
			len = ETHER_MIN_LEN - ETHER_CRC_LEN;
		}
		bpf_mtap(ifp, m, BPF_D_OUT);
		m_freem(m);

		rza1eth_sync(sc, DMA_OFF(txbuf[i]), len, BUS_DMASYNC_PREWRITE);
		dma->txd[i].d1 = TD1_TDL(len);
		dma->txd[i].d0 = TD0_TACT | TD0_TFP_ONE |
		    (i == RZA1ETH_NTXDESC - 1 ? TD0_TDLE : 0);
		rza1eth_sync(sc, DMA_OFF(txd[i]), sizeof(dma->txd[i]),
		    BUS_DMASYNC_PREWRITE);

		sc->sc_txnext = (i + 1) % RZA1ETH_NTXDESC;
		sc->sc_txfree--;
		queued = true;
	}

	if (queued) {
		if ((RD4(sc, ETH_EDTRR) & EDTRR_TR) != EDTRR_TR)
			WR4(sc, ETH_EDTRR, EDTRR_TR);
		ifp->if_timer = 5;
	}
}

static void
rza1eth_txeof(struct rza1eth_softc *sc)
{
	struct ifnet * const ifp = &sc->sc_ec.ec_if;
	struct rza1eth_dma * const dma = sc->sc_dma;
	u_int i;

	while (sc->sc_txfree < RZA1ETH_NTXDESC) {
		i = sc->sc_txdirty;
		rza1eth_sync(sc, DMA_OFF(txd[i]), sizeof(dma->txd[i]),
		    BUS_DMASYNC_POSTREAD);
		const uint32_t d0 = dma->txd[i].d0;
		if (d0 & TD0_TACT)
			break;
		if (d0 & TD0_TFE)
			if_statinc(ifp, if_oerrors);
		else
			if_statinc(ifp, if_opackets);
		sc->sc_txdirty = (i + 1) % RZA1ETH_NTXDESC;
		sc->sc_txfree++;
	}
	if (sc->sc_txfree == RZA1ETH_NTXDESC)
		ifp->if_timer = 0;
}

static void
rza1eth_rxeof(struct rza1eth_softc *sc)
{
	struct ifnet * const ifp = &sc->sc_ec.ec_if;
	struct rza1eth_dma * const dma = sc->sc_dma;
	struct mbuf *m;
	u_int i, len;

	for (;;) {
		i = sc->sc_rxnext;
		rza1eth_sync(sc, DMA_OFF(rxd[i]), sizeof(dma->rxd[i]),
		    BUS_DMASYNC_POSTREAD);
		const uint32_t d0 = dma->rxd[i].d0;
		if (d0 & RD0_RACT)
			break;

		len = __SHIFTOUT(dma->rxd[i].d1, RD1_RDL);
		if (__SHIFTOUT(d0, RD0_RFP) != RD0_RFP_ONE ||
		    ((d0 & RD0_RFE) && (d0 & RD0_RFS & ~RD0_RFS_RMAF)) ||
		    len < ETHER_HDR_LEN || len > RZA1ETH_BUFSIZE) {
			if_statinc(ifp, if_ierrors);
			goto next;
		}

		MGETHDR(m, M_DONTWAIT, MT_DATA);
		if (m != NULL && len + ETHER_ALIGN > MHLEN) {
			MCLGET(m, M_DONTWAIT);
			if ((m->m_flags & M_EXT) == 0) {
				m_freem(m);
				m = NULL;
			}
		}
		if (m == NULL) {
			if_statinc(ifp, if_ierrors);
			goto next;
		}
		m->m_data += ETHER_ALIGN;
		rza1eth_sync(sc, DMA_OFF(rxbuf[i]), len, BUS_DMASYNC_POSTREAD);
		memcpy(mtod(m, void *), dma->rxbuf[i], len);
		m->m_len = m->m_pkthdr.len = len;
		m_set_rcvif(m, ifp);
		if_percpuq_enqueue(ifp->if_percpuq, m);

next:
		rza1eth_sync(sc, DMA_OFF(rxbuf[i]), RZA1ETH_BUFSIZE,
		    BUS_DMASYNC_PREREAD);
		dma->rxd[i].d1 = RD1_RBL(RZA1ETH_BUFSIZE);
		dma->rxd[i].d0 = RD0_RACT |
		    (i == RZA1ETH_NRXDESC - 1 ? RD0_RDLE : 0);
		rza1eth_sync(sc, DMA_OFF(rxd[i]), sizeof(dma->rxd[i]),
		    BUS_DMASYNC_PREWRITE);
		sc->sc_rxnext = (i + 1) % RZA1ETH_NRXDESC;
	}

	if ((RD4(sc, ETH_EDRRR) & EDRRR_RR) == 0)
		WR4(sc, ETH_EDRRR, EDRRR_RR);
}

static int
rza1eth_intr(void *arg)
{
	struct rza1eth_softc * const sc = arg;
	struct ifnet * const ifp = &sc->sc_ec.ec_if;
	uint32_t status;

	status = RD4(sc, ETH_EESR) & RD4(sc, ETH_EESIPR);
	if (status == 0)
		return 0;
	WR4(sc, ETH_EESR, status & ~EESR_ECI);

	if (status & (EESR_FR | EESR_RDE | EESR_RFOF))
		rza1eth_rxeof(sc);
	if (status & EESR_RFOF)
		if_statinc(ifp, if_iqdrops);
	if (status & (EESR_TC0 | EESR_TC1 | EESR_TABT | EESR_TFUF)) {
		rza1eth_txeof(sc);
		if_schedule_deferred_start(ifp);
	}

	return 1;
}

static void
rza1eth_watchdog(struct ifnet *ifp)
{
	struct rza1eth_softc * const sc = ifp->if_softc;

	device_printf(sc->sc_dev, "transmit timeout\n");
	if_statinc(ifp, if_oerrors);
	rza1eth_init(ifp);
}

static int
rza1eth_ioctl(struct ifnet *ifp, u_long cmd, void *data)
{
	int s, error;

	s = splnet();
	error = ether_ioctl(ifp, cmd, data);
	if (error == ENETRESET) {
		error = 0;
		/* all multicast is accepted. */
		if (cmd != SIOCADDMULTI && cmd != SIOCDELMULTI &&
		    (ifp->if_flags & IFF_RUNNING))
			error = rza1eth_init(ifp);
	}
	splx(s);

	return error;
}

static void
rza1eth_phy_reset(struct rza1eth_softc *sc)
{
	struct fdtbus_gpio_pin *pin;
	const int phy = fdtbus_get_phandle(sc->sc_phandle, "phy-handle");
	u_int us;

	if (phy <= 0)
		return;
	pin = fdtbus_gpio_acquire(phy, "reset-gpios", GPIO_PIN_OUTPUT);
	if (pin == NULL)
		return;
	if (of_getprop_uint32(phy, "reset-delay-us", &us) != 0 || us < 25000)
		us = 25000;
	fdtbus_gpio_write(pin, 1);
	delay(us);
	fdtbus_gpio_write(pin, 0);
	delay(10000);
	sc->sc_phy_reset = pin;
}

static void
rza1eth_get_enaddr(struct rza1eth_softc *sc)
{
	const uint8_t *prop;
	int len;

	prop = fdtbus_get_prop(sc->sc_phandle, "local-mac-address", &len);
	if (prop == NULL || len != ETHER_ADDR_LEN)
		prop = fdtbus_get_prop(sc->sc_phandle, "mac-address", &len);
	if (prop != NULL && len == ETHER_ADDR_LEN &&
	    memcmp(prop, "\0\0\0\0\0\0", ETHER_ADDR_LEN) != 0) {
		memcpy(sc->sc_enaddr, prop, ETHER_ADDR_LEN);
		return;
	}

	const uint32_t r = rza1_get_entropy32();
	sc->sc_enaddr[0] = 0x02;
	sc->sc_enaddr[1] = 0x00;
	sc->sc_enaddr[2] = r >> 24;
	sc->sc_enaddr[3] = r >> 16;
	sc->sc_enaddr[4] = r >> 8;
	sc->sc_enaddr[5] = r;
	aprint_normal_dev(sc->sc_dev, "no MAC address in the device tree, "
	    "using a generated one (set local-mac-address to fix it)\n");
}

static int
rza1eth_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
rza1eth_attach(device_t parent, device_t self, void *aux)
{
	struct rza1eth_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	struct ifnet * const ifp = &sc->sc_ec.ec_if;
	struct mii_data * const mii = &sc->sc_mii;
	char intrstr[128];
	struct clk *clk;
	bus_addr_t addr;
	bus_size_t size;
	void *kva;
	int rseg, error;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}
	if (!fdtbus_intr_str(phandle, 0, intrstr, sizeof(intrstr))) {
		aprint_error(": couldn't decode interrupt\n");
		return;
	}

	sc->sc_dev = self;
	sc->sc_phandle = phandle;
	sc->sc_bst = faa->faa_bst;
	sc->sc_dmat = faa->faa_dmat;
	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	clk = fdtbus_clock_get_index(phandle, 0);
	if (clk != NULL && clk_enable(clk) != 0) {
		aprint_error(": couldn't enable clock\n");
		return;
	}
	fdtbus_pinctrl_set_config(phandle, "default");

	aprint_naive("\n");
	aprint_normal(": RZ/A1 Ethernet\n");

	error = bus_dmamem_alloc(sc->sc_dmat, sizeof(struct rza1eth_dma), 32,
	    0, &sc->sc_dmaseg, 1, &rseg, BUS_DMA_WAITOK);
	if (error == 0)
		error = bus_dmamem_map(sc->sc_dmat, &sc->sc_dmaseg, rseg,
		    sizeof(struct rza1eth_dma), &kva,
		    BUS_DMA_WAITOK | BUS_DMA_COHERENT);
	if (error == 0)
		error = bus_dmamap_create(sc->sc_dmat,
		    sizeof(struct rza1eth_dma), 1, sizeof(struct rza1eth_dma),
		    0, BUS_DMA_WAITOK, &sc->sc_dmamap);
	if (error == 0)
		error = bus_dmamap_load(sc->sc_dmat, sc->sc_dmamap, kva,
		    sizeof(struct rza1eth_dma), NULL, BUS_DMA_WAITOK);
	if (error != 0) {
		aprint_error_dev(self, "couldn't set up DMA memory (%d)\n",
		    error);
		return;
	}
	sc->sc_dma = kva;
	sc->sc_dmapa = sc->sc_dmamap->dm_segs[0].ds_addr;

	rza1eth_get_enaddr(sc);
	aprint_normal_dev(self, "Ethernet address %s\n",
	    ether_sprintf(sc->sc_enaddr));

	rza1eth_phy_reset(sc);
	rza1eth_reset(sc);

	if (fdtbus_intr_establish_xname(phandle, 0, IPL_NET, 0,
	    rza1eth_intr, sc, device_xname(self)) == NULL) {
		aprint_error_dev(self, "couldn't establish interrupt on %s\n",
		    intrstr);
		return;
	}
	aprint_normal_dev(self, "interrupting on %s\n", intrstr);

	callout_init(&sc->sc_tick_ch, 0);
	callout_setfunc(&sc->sc_tick_ch, rza1eth_tick, sc);

	strlcpy(ifp->if_xname, device_xname(self), IFNAMSIZ);
	ifp->if_softc = sc;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_start = rza1eth_start;
	ifp->if_ioctl = rza1eth_ioctl;
	ifp->if_init = rza1eth_init;
	ifp->if_stop = rza1eth_stop;
	ifp->if_watchdog = rza1eth_watchdog;
	IFQ_SET_MAXLEN(&ifp->if_snd, IFQ_MAXLEN);
	IFQ_SET_READY(&ifp->if_snd);

	sc->sc_ec.ec_mii = mii;
	mii->mii_ifp = ifp;
	mii->mii_readreg = rza1eth_mii_readreg;
	mii->mii_writereg = rza1eth_mii_writereg;
	mii->mii_statchg = rza1eth_mii_statchg;
	ifmedia_init(&mii->mii_media, 0, ether_mediachange,
	    ether_mediastatus);
	mii_attach(self, mii, 0xffffffff, MII_PHY_ANY, MII_OFFSET_ANY, 0);
	if (LIST_EMPTY(&mii->mii_phys)) {
		aprint_error_dev(self, "no PHY found\n");
		ifmedia_add(&mii->mii_media, IFM_ETHER | IFM_MANUAL, 0, NULL);
		ifmedia_set(&mii->mii_media, IFM_ETHER | IFM_MANUAL);
	} else {
		ifmedia_set(&mii->mii_media, IFM_ETHER | IFM_AUTO);
	}

	if_attach(ifp);
	if_deferred_start_init(ifp, NULL);
	ether_ifattach(ifp, sc->sc_enaddr);
}
