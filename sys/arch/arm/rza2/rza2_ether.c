// SPDX-License-Identifier: BSD-2-Clause
/*
 * Renesas RZ/A2 Ethernet controller: ETHERC + EDMACa, one instance per
 * channel ("renesas,ether-r7s9210").
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

#include <arm/rza2/rza2_var.h>

#define	ETH_EDMR	0x000
#define	 EDMR_SWR	__BIT(0)
#define	 EDMR_DL_16	__SHIFTIN(0, __BITS(5,4))
#define	 EDMR_DE	__BIT(6)
#define	ETH_EDTRR	0x008
#define	 EDTRR_TR	__BIT(0)
#define	ETH_EDRRR	0x010
#define	 EDRRR_RR	__BIT(0)
#define	ETH_TDLAR	0x018
#define	ETH_RDLAR	0x020
#define	ETH_EESR	0x028
#define	 EESR_RMAF	__BIT(7)
#define	 EESR_RFOF	__BIT(16)
#define	 EESR_RDE	__BIT(17)
#define	 EESR_FR	__BIT(18)
#define	 EESR_TFUF	__BIT(19)
#define	 EESR_TDE	__BIT(20)
#define	 EESR_TC	__BIT(21)
#define	 EESR_ECI	__BIT(22)
#define	 EESR_RFCOF	__BIT(24)
#define	 EESR_RABT	__BIT(25)
#define	 EESR_TABT	__BIT(26)
#define	 EESR_TWB	__BIT(30)
#define	ETH_EESIPR	0x030
#define	ETH_TRSCER	0x038
#define	 TRSCER_RRFCE	__BIT(4)
#define	 TRSCER_RMAFCE	__BIT(7)
#define	ETH_RMFCR	0x040
#define	ETH_TFTR	0x048
#define	ETH_FDR		0x050
#define	 FDR_VALUE	0x0000070f
#define	ETH_RMCR	0x058
#define	 RMCR_RNR	__BIT(0)
#define	ETH_RPADIR	0x078
#define	ETH_TRIMD	0x07c
#define	 TRIMD_TIS	__BIT(0)

#define	ETH_ECMR	0x100
#define	 ECMR_PRM	__BIT(0)
#define	 ECMR_DM	__BIT(1)
#define	 ECMR_RTM	__BIT(2)
#define	 ECMR_TE	__BIT(5)
#define	 ECMR_RE	__BIT(6)
#define	ETH_RFLR	0x108
#define	ETH_ECSR	0x110
#define	ETH_ECSIPR	0x118
#define	ETH_PIR		0x120
#define	 PIR_MDC	__BIT(0)
#define	 PIR_MMD	__BIT(1)
#define	 PIR_MDO	__BIT(2)
#define	 PIR_MDI	__BIT(3)
#define	ETH_MAHR	0x1c0
#define	ETH_MALR	0x1c8

#define	RZA2ETH_CH_BASE		0xe8204000
#define	RZA2ETH_CH_STRIDE	0x200

struct rza2eth_desc {
	volatile uint32_t d0;
	volatile uint32_t d1;
	volatile uint32_t d2;
	volatile uint32_t pad;
};

#define	TD0_TACT	__BIT(31)
#define	TD0_TDLE	__BIT(30)
#define	TD0_TFP_ONE	__BITS(29,28)
#define	TD0_TFE		__BIT(27)
#define	TD1_TBL(n)	((uint32_t)(n) << 16)

#define	RD0_RACT	__BIT(31)
#define	RD0_RDLE	__BIT(30)
#define	RD0_RFP		__BITS(29,28)
#define	 RD0_RFP_ONE	3
#define	RD0_RFE		__BIT(27)
#define	RD1_RBL(n)	((uint32_t)(n) << 16)
#define	RD1_RFL		__BITS(15,0)

#define	RZA2ETH_NTXDESC	4
#define	RZA2ETH_NRXDESC	8
#define	RZA2ETH_BUFSIZE	1536

struct rza2eth_ring {
	struct rza2eth_desc txd[RZA2ETH_NTXDESC];
	struct rza2eth_desc rxd[RZA2ETH_NRXDESC];
};

struct rza2eth_bufs {
	uint8_t txbuf[RZA2ETH_NTXDESC][RZA2ETH_BUFSIZE];
	uint8_t rxbuf[RZA2ETH_NRXDESC][RZA2ETH_BUFSIZE];
};

#define	RING_OFF(m)	offsetof(struct rza2eth_ring, m)
#define	BUF_OFF(m)	offsetof(struct rza2eth_bufs, m)

struct rza2eth_dmamem {
	bus_dmamap_t dm_map;
	bus_dma_segment_t dm_seg;
	void *dm_kva;
	bus_addr_t dm_pa;
};

struct rza2eth_softc {
	device_t sc_dev;
	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;
	bus_dma_tag_t sc_dmat;
	int sc_phandle;
	bool sc_rmii;

	struct rza2eth_dmamem sc_ringmem;
	struct rza2eth_dmamem sc_bufmem;
	struct rza2eth_ring *sc_ring;
	struct rza2eth_bufs *sc_bufs;

	struct ethercom sc_ec;
	struct mii_data sc_mii;
	callout_t sc_tick_ch;
	struct fdtbus_gpio_pin *sc_phy_reset;
	uint8_t sc_enaddr[ETHER_ADDR_LEN];
	uint32_t sc_ecmr_mode;

	u_int sc_txnext;
	u_int sc_txdirty;
	u_int sc_txfree;
	u_int sc_rxnext;
};

#define	RD4(sc, reg)		bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define	WR4(sc, reg, val)	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "renesas,ether-r7s9210" },
	DEVICE_COMPAT_EOL
};

static int	rza2eth_match(device_t, cfdata_t, void *);
static void	rza2eth_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(rza2_ether, sizeof(struct rza2eth_softc),
    rza2eth_match, rza2eth_attach, NULL, NULL);

static int	rza2eth_init(struct ifnet *);
static void	rza2eth_init_hw(struct rza2eth_softc *);
static void	rza2eth_stop(struct ifnet *, int);
static void	rza2eth_start(struct ifnet *);
static int	rza2eth_ioctl(struct ifnet *, u_long, void *);
static void	rza2eth_watchdog(struct ifnet *);

static uint32_t
rza2eth_mii_bb_read(device_t self)
{
	struct rza2eth_softc * const sc = device_private(self);

	return RD4(sc, ETH_PIR);
}

static void
rza2eth_mii_bb_write(device_t self, uint32_t val)
{
	struct rza2eth_softc * const sc = device_private(self);

	WR4(sc, ETH_PIR, val);
}

static const struct mii_bitbang_ops rza2eth_mii_bb_ops = {
	rza2eth_mii_bb_read,
	rza2eth_mii_bb_write,
	{
		PIR_MDO,
		PIR_MDI,
		PIR_MDC,
		PIR_MMD,
		0,
	}
};

static int
rza2eth_mii_readreg(device_t self, int phy, int reg, uint16_t *val)
{
	return mii_bitbang_readreg(self, &rza2eth_mii_bb_ops, phy, reg, val);
}

static int
rza2eth_mii_writereg(device_t self, int phy, int reg, uint16_t val)
{
	return mii_bitbang_writereg(self, &rza2eth_mii_bb_ops, phy, reg, val);
}

static void
rza2eth_mii_statchg(struct ifnet *ifp)
{
	struct rza2eth_softc * const sc = ifp->if_softc;
	const u_int media = sc->sc_mii.mii_media_active;
	uint32_t mode = 0;

	if ((IFM_OPTIONS(media) & IFM_FDX) != 0)
		mode |= ECMR_DM;
	if (sc->sc_rmii && IFM_SUBTYPE(media) == IFM_100_TX)
		mode |= ECMR_RTM;
	if (mode == sc->sc_ecmr_mode)
		return;
	sc->sc_ecmr_mode = mode;

	if (ifp->if_flags & IFF_RUNNING)
		rza2eth_init_hw(sc);
}

static void
rza2eth_tick(void *arg)
{
	struct rza2eth_softc * const sc = arg;
	int s;

	s = splnet();
	mii_tick(&sc->sc_mii);
	splx(s);

	callout_schedule(&sc->sc_tick_ch, hz);
}

static void
rza2eth_ring_sync(struct rza2eth_softc *sc, bus_size_t off, bus_size_t len,
    int ops)
{
	bus_dmamap_sync(sc->sc_dmat, sc->sc_ringmem.dm_map, off, len, ops);
}

static void
rza2eth_buf_sync(struct rza2eth_softc *sc, bus_size_t off, bus_size_t len,
    int ops)
{
	bus_dmamap_sync(sc->sc_dmat, sc->sc_bufmem.dm_map, off, len, ops);
}

static void
rza2eth_init_rings(struct rza2eth_softc *sc)
{
	struct rza2eth_ring * const ring = sc->sc_ring;
	const bus_addr_t bufpa = sc->sc_bufmem.dm_pa;
	u_int i;

	rza2eth_buf_sync(sc, BUF_OFF(rxbuf), sizeof(sc->sc_bufs->rxbuf),
	    BUS_DMASYNC_PREREAD);

	for (i = 0; i < RZA2ETH_NTXDESC; i++) {
		ring->txd[i].d0 = TD0_TFP_ONE |
		    (i == RZA2ETH_NTXDESC - 1 ? TD0_TDLE : 0);
		ring->txd[i].d1 = 0;
		ring->txd[i].d2 = bufpa + BUF_OFF(txbuf[i]);
	}
	for (i = 0; i < RZA2ETH_NRXDESC; i++) {
		ring->rxd[i].d1 = RD1_RBL(RZA2ETH_BUFSIZE);
		ring->rxd[i].d2 = bufpa + BUF_OFF(rxbuf[i]);
		ring->rxd[i].d0 = RD0_RACT |
		    (i == RZA2ETH_NRXDESC - 1 ? RD0_RDLE : 0);
	}
	rza2eth_ring_sync(sc, 0, sizeof(*ring),
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	sc->sc_txnext = sc->sc_txdirty = 0;
	sc->sc_txfree = RZA2ETH_NTXDESC;
	sc->sc_rxnext = 0;
}

static void
rza2eth_reset(struct rza2eth_softc *sc)
{
	int timo;

	WR4(sc, ETH_EESIPR, 0);
	WR4(sc, ETH_ECMR, RD4(sc, ETH_ECMR) & ~(ECMR_TE | ECMR_RE));
	for (timo = 200; timo > 0; timo--) {
		if ((RD4(sc, ETH_EDTRR) & EDTRR_TR) == 0)
			break;
		delay(10);
	}
	WR4(sc, ETH_EDRRR, 0);

	WR4(sc, ETH_EDMR, EDMR_SWR);

	delay(10);
}

static void
rza2eth_init_hw(struct rza2eth_softc *sc)
{
	struct ifnet * const ifp = &sc->sc_ec.ec_if;
	const uint8_t *ea = CLLADDR(ifp->if_sadl);
	uint32_t ecmr;

	rza2eth_reset(sc);
	rza2eth_init_rings(sc);

	WR4(sc, ETH_EDMR, EDMR_DE | EDMR_DL_16);
	WR4(sc, ETH_TDLAR, sc->sc_ringmem.dm_pa + RING_OFF(txd));
	WR4(sc, ETH_RDLAR, sc->sc_ringmem.dm_pa + RING_OFF(rxd));

	WR4(sc, ETH_TRSCER, TRSCER_RMAFCE);
	WR4(sc, ETH_TFTR, 0);
	WR4(sc, ETH_FDR, FDR_VALUE);
	WR4(sc, ETH_RMCR, RMCR_RNR);
	WR4(sc, ETH_RPADIR, 0);
	WR4(sc, ETH_TRIMD, TRIMD_TIS);
	WR4(sc, ETH_RMFCR, 0);

	ecmr = sc->sc_ecmr_mode;
	if (ifp->if_flags & IFF_PROMISC)
		ecmr |= ECMR_PRM;
	WR4(sc, ETH_ECMR, ecmr);
	WR4(sc, ETH_RFLR, ETHER_MAX_LEN);
	WR4(sc, ETH_MAHR, ((uint32_t)ea[0] << 24) | ((uint32_t)ea[1] << 16) |
	    ((uint32_t)ea[2] << 8) | ea[3]);
	WR4(sc, ETH_MALR, ((uint32_t)ea[4] << 8) | ea[5]);
	WR4(sc, ETH_ECSIPR, 0);
	WR4(sc, ETH_ECSR, RD4(sc, ETH_ECSR));

	WR4(sc, ETH_EESR, RD4(sc, ETH_EESR) & ~EESR_ECI);
	WR4(sc, ETH_EESIPR, EESR_FR | EESR_RDE | EESR_RFOF | EESR_RFCOF |
	    EESR_TC | EESR_TWB | EESR_TABT | EESR_TFUF);

	WR4(sc, ETH_EDRRR, EDRRR_RR);
	WR4(sc, ETH_ECMR, ecmr | ECMR_TE | ECMR_RE);

	ifp->if_timer = 0;
}

static int
rza2eth_init(struct ifnet *ifp)
{
	struct rza2eth_softc * const sc = ifp->if_softc;
	struct ethercom * const ec = &sc->sc_ec;
	int s;

	s = splnet();

	rza2eth_stop(ifp, 0);
	rza2eth_init_hw(sc);
	ifp->if_flags |= IFF_RUNNING;

	ETHER_LOCK(ec);
	ec->ec_flags |= ETHER_F_ALLMULTI;
	ETHER_UNLOCK(ec);

	mii_mediachg(&sc->sc_mii);
	callout_schedule(&sc->sc_tick_ch, hz);

	splx(s);

	rza2eth_start(ifp);
	return 0;
}

static void
rza2eth_stop(struct ifnet *ifp, int disable)
{
	struct rza2eth_softc * const sc = ifp->if_softc;

	callout_stop(&sc->sc_tick_ch);
	rza2eth_reset(sc);

	if (disable)
		mii_down(&sc->sc_mii);

	ifp->if_flags &= ~IFF_RUNNING;
	ifp->if_timer = 0;
}

static void
rza2eth_start(struct ifnet *ifp)
{
	struct rza2eth_softc * const sc = ifp->if_softc;
	struct rza2eth_ring * const ring = sc->sc_ring;
	struct rza2eth_bufs * const bufs = sc->sc_bufs;
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
		if (len > RZA2ETH_BUFSIZE) {
			if_statinc(ifp, if_oerrors);
			m_freem(m);
			continue;
		}
		m_copydata(m, 0, len, bufs->txbuf[i]);
		if (len < ETHER_MIN_LEN - ETHER_CRC_LEN) {
			memset(bufs->txbuf[i] + len, 0,
			    ETHER_MIN_LEN - ETHER_CRC_LEN - len);
			len = ETHER_MIN_LEN - ETHER_CRC_LEN;
		}
		bpf_mtap(ifp, m, BPF_D_OUT);
		m_freem(m);

		rza2eth_buf_sync(sc, BUF_OFF(txbuf[i]), len,
		    BUS_DMASYNC_PREWRITE);
		ring->txd[i].d1 = TD1_TBL(len);
		ring->txd[i].d0 = TD0_TACT | TD0_TFP_ONE |
		    (i == RZA2ETH_NTXDESC - 1 ? TD0_TDLE : 0);
		rza2eth_ring_sync(sc, RING_OFF(txd[i]), sizeof(ring->txd[i]),
		    BUS_DMASYNC_PREWRITE);

		sc->sc_txnext = (i + 1) % RZA2ETH_NTXDESC;
		sc->sc_txfree--;
		queued = true;
	}

	if (queued) {
		if ((RD4(sc, ETH_EDTRR) & EDTRR_TR) == 0)
			WR4(sc, ETH_EDTRR, EDTRR_TR);
		ifp->if_timer = 5;
	}
}

static void
rza2eth_txeof(struct rza2eth_softc *sc)
{
	struct ifnet * const ifp = &sc->sc_ec.ec_if;
	struct rza2eth_ring * const ring = sc->sc_ring;
	u_int i;

	while (sc->sc_txfree < RZA2ETH_NTXDESC) {
		i = sc->sc_txdirty;
		rza2eth_ring_sync(sc, RING_OFF(txd[i]), sizeof(ring->txd[i]),
		    BUS_DMASYNC_POSTREAD);
		const uint32_t d0 = ring->txd[i].d0;
		if (d0 & TD0_TACT)
			break;
		rza2eth_buf_sync(sc, BUF_OFF(txbuf[i]), RZA2ETH_BUFSIZE,
		    BUS_DMASYNC_POSTWRITE);
		if (d0 & TD0_TFE)
			if_statinc(ifp, if_oerrors);
		else
			if_statinc(ifp, if_opackets);
		sc->sc_txdirty = (i + 1) % RZA2ETH_NTXDESC;
		sc->sc_txfree++;
	}
	if (sc->sc_txfree == RZA2ETH_NTXDESC)
		ifp->if_timer = 0;
}

static void
rza2eth_rxeof(struct rza2eth_softc *sc)
{
	struct ifnet * const ifp = &sc->sc_ec.ec_if;
	struct rza2eth_ring * const ring = sc->sc_ring;
	struct rza2eth_bufs * const bufs = sc->sc_bufs;
	struct mbuf *m;
	u_int i, len;

	for (;;) {
		i = sc->sc_rxnext;
		rza2eth_ring_sync(sc, RING_OFF(rxd[i]), sizeof(ring->rxd[i]),
		    BUS_DMASYNC_POSTREAD);
		const uint32_t d0 = ring->rxd[i].d0;
		if (d0 & RD0_RACT)
			break;

		len = __SHIFTOUT(ring->rxd[i].d1, RD1_RFL);
		if (__SHIFTOUT(d0, RD0_RFP) != RD0_RFP_ONE ||
		    (d0 & RD0_RFE) != 0 ||
		    len < ETHER_HDR_LEN || len > RZA2ETH_BUFSIZE) {
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
		rza2eth_buf_sync(sc, BUF_OFF(rxbuf[i]), len,
		    BUS_DMASYNC_POSTREAD);
		memcpy(mtod(m, void *), bufs->rxbuf[i], len);
		m->m_len = m->m_pkthdr.len = len;
		m_set_rcvif(m, ifp);
		if_percpuq_enqueue(ifp->if_percpuq, m);

next:
		rza2eth_buf_sync(sc, BUF_OFF(rxbuf[i]), RZA2ETH_BUFSIZE,
		    BUS_DMASYNC_PREREAD);
		ring->rxd[i].d1 = RD1_RBL(RZA2ETH_BUFSIZE);
		ring->rxd[i].d0 = RD0_RACT |
		    (i == RZA2ETH_NRXDESC - 1 ? RD0_RDLE : 0);
		rza2eth_ring_sync(sc, RING_OFF(rxd[i]), sizeof(ring->rxd[i]),
		    BUS_DMASYNC_PREWRITE);
		sc->sc_rxnext = (i + 1) % RZA2ETH_NRXDESC;
	}

	if ((RD4(sc, ETH_EDRRR) & EDRRR_RR) == 0)
		WR4(sc, ETH_EDRRR, EDRRR_RR);
}

static int
rza2eth_intr(void *arg)
{
	struct rza2eth_softc * const sc = arg;
	struct ifnet * const ifp = &sc->sc_ec.ec_if;
	uint32_t status;

	status = RD4(sc, ETH_EESR) & RD4(sc, ETH_EESIPR);
	if (status == 0)
		return 0;
	WR4(sc, ETH_EESR, status & ~EESR_ECI);

	if (status & (EESR_FR | EESR_RDE | EESR_RFOF | EESR_RFCOF))
		rza2eth_rxeof(sc);
	if (status & (EESR_RFOF | EESR_RFCOF))
		if_statinc(ifp, if_iqdrops);
	if (status & (EESR_TC | EESR_TWB | EESR_TABT | EESR_TFUF)) {
		rza2eth_txeof(sc);
		if_schedule_deferred_start(ifp);
	}

	return 1;
}

static void
rza2eth_watchdog(struct ifnet *ifp)
{
	struct rza2eth_softc * const sc = ifp->if_softc;

	device_printf(sc->sc_dev, "transmit timeout\n");
	if_statinc(ifp, if_oerrors);
	rza2eth_init(ifp);
}

static int
rza2eth_ioctl(struct ifnet *ifp, u_long cmd, void *data)
{
	int s, error;

	s = splnet();
	error = ether_ioctl(ifp, cmd, data);
	if (error == ENETRESET) {
		error = 0;
		if (cmd != SIOCADDMULTI && cmd != SIOCDELMULTI &&
		    (ifp->if_flags & IFF_RUNNING))
			error = rza2eth_init(ifp);
	}
	splx(s);

	return error;
}

static void
rza2eth_phy_reset(struct rza2eth_softc *sc)
{
	struct fdtbus_gpio_pin *pin;
	const int phy = fdtbus_get_phandle(sc->sc_phandle, "phy-handle");
	u_int assert_us, deassert_us;

	if (phy <= 0)
		return;
	pin = fdtbus_gpio_acquire(phy, "reset-gpios", GPIO_PIN_OUTPUT);
	if (pin == NULL)
		return;
	if (of_getprop_uint32(phy, "reset-assert-us", &assert_us) != 0)
		assert_us = 25000;
	if (of_getprop_uint32(phy, "reset-deassert-us", &deassert_us) != 0)
		deassert_us = 10000;
	fdtbus_gpio_write(pin, 1);
	delay(assert_us);
	fdtbus_gpio_write(pin, 0);
	delay(deassert_us);
	sc->sc_phy_reset = pin;
}

static void
rza2eth_get_enaddr(struct rza2eth_softc *sc)
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

	const uint32_t r = rza2_get_entropy32();
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
rza2eth_dmamem_alloc(struct rza2eth_softc *sc, struct rza2eth_dmamem *dm,
    bus_size_t size, int mapflags)
{
	int rseg, error;

	error = bus_dmamem_alloc(sc->sc_dmat, size, 32, 0, &dm->dm_seg, 1,
	    &rseg, BUS_DMA_WAITOK);
	if (error != 0)
		return error;
	error = bus_dmamem_map(sc->sc_dmat, &dm->dm_seg, rseg, size,
	    &dm->dm_kva, BUS_DMA_WAITOK | mapflags);
	if (error != 0)
		goto free;
	error = bus_dmamap_create(sc->sc_dmat, size, 1, size, 0,
	    BUS_DMA_WAITOK, &dm->dm_map);
	if (error != 0)
		goto unmap;
	error = bus_dmamap_load(sc->sc_dmat, dm->dm_map, dm->dm_kva, size,
	    NULL, BUS_DMA_WAITOK);
	if (error != 0)
		goto destroy;
	dm->dm_pa = dm->dm_map->dm_segs[0].ds_addr;
	return 0;

destroy:
	bus_dmamap_destroy(sc->sc_dmat, dm->dm_map);
unmap:
	bus_dmamem_unmap(sc->sc_dmat, dm->dm_kva, size);
free:
	bus_dmamem_free(sc->sc_dmat, &dm->dm_seg, rseg);
	return error;
}

static int
rza2eth_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
rza2eth_attach(device_t parent, device_t self, void *aux)
{
	struct rza2eth_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	struct ifnet * const ifp = &sc->sc_ec.ec_if;
	struct mii_data * const mii = &sc->sc_mii;
	char intrstr[128];
	const char *phy_mode;
	struct clk *clk;
	bus_addr_t addr;
	bus_size_t size;
	u_int ch;
	int i, error;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}
	if (addr < RZA2ETH_CH_BASE ||
	    (ch = (addr - RZA2ETH_CH_BASE) / RZA2ETH_CH_STRIDE) > 1 ||
	    addr != RZA2ETH_CH_BASE + ch * RZA2ETH_CH_STRIDE ||
	    size < RZA2ETH_CH_STRIDE) {
		aprint_error(": unexpected registers %#" PRIxBUSADDR "\n",
		    addr);
		return;
	}
	if (!fdtbus_intr_str(phandle, 0, intrstr, sizeof(intrstr))) {
		aprint_error(": couldn't decode interrupt\n");
		return;
	}

	phy_mode = fdtbus_get_string(phandle, "phy-mode");
	if (phy_mode == NULL || strcmp(phy_mode, "rmii") == 0)
		sc->sc_rmii = true;
	else if (strcmp(phy_mode, "mii") == 0)
		sc->sc_rmii = false;
	else {
		aprint_error(": unsupported phy-mode \"%s\"\n", phy_mode);
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

	for (i = 0; (clk = fdtbus_clock_get_index(phandle, i)) != NULL; i++) {
		if (clk_enable(clk) != 0) {
			aprint_error(": couldn't enable clock #%d\n", i);
			return;
		}
	}

	aprint_naive("\n");
	aprint_normal(": RZ/A2 Ethernet, channel %u, %s\n", ch,
	    sc->sc_rmii ? "RMII" : "MII");

	error = rza2_pinctrl_set_ether_mii(ch, !sc->sc_rmii);
	if (error != 0) {
		aprint_error_dev(self, "couldn't select %s (%d)\n",
		    sc->sc_rmii ? "RMII" : "MII", error);
		return;
	}

	error = rza2eth_dmamem_alloc(sc, &sc->sc_ringmem,
	    sizeof(struct rza2eth_ring), BUS_DMA_COHERENT);
	if (error == 0)
		error = rza2eth_dmamem_alloc(sc, &sc->sc_bufmem,
		    sizeof(struct rza2eth_bufs), 0);
	if (error != 0) {
		aprint_error_dev(self, "couldn't set up DMA memory (%d)\n",
		    error);
		return;
	}
	sc->sc_ring = sc->sc_ringmem.dm_kva;
	sc->sc_bufs = sc->sc_bufmem.dm_kva;

	rza2eth_get_enaddr(sc);
	aprint_normal_dev(self, "Ethernet address %s\n",
	    ether_sprintf(sc->sc_enaddr));

	rza2eth_phy_reset(sc);
	rza2eth_reset(sc);

	if (fdtbus_intr_establish_xname(phandle, 0, IPL_NET, 0,
	    rza2eth_intr, sc, device_xname(self)) == NULL) {
		aprint_error_dev(self, "couldn't establish interrupt on %s\n",
		    intrstr);
		return;
	}
	aprint_normal_dev(self, "interrupting on %s\n", intrstr);

	callout_init(&sc->sc_tick_ch, 0);
	callout_setfunc(&sc->sc_tick_ch, rza2eth_tick, sc);

	strlcpy(ifp->if_xname, device_xname(self), IFNAMSIZ);
	ifp->if_softc = sc;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_start = rza2eth_start;
	ifp->if_ioctl = rza2eth_ioctl;
	ifp->if_init = rza2eth_init;
	ifp->if_stop = rza2eth_stop;
	ifp->if_watchdog = rza2eth_watchdog;
	IFQ_SET_MAXLEN(&ifp->if_snd, IFQ_MAXLEN);
	IFQ_SET_READY(&ifp->if_snd);

	sc->sc_ec.ec_mii = mii;
	mii->mii_ifp = ifp;
	mii->mii_readreg = rza2eth_mii_readreg;
	mii->mii_writereg = rza2eth_mii_writereg;
	mii->mii_statchg = rza2eth_mii_statchg;
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
