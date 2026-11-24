// SPDX-License-Identifier: BSD-2-Clause
/*
 * Renesas RZ/A1 RSPI (serial peripheral interface).
 */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/gpio.h>
#include <sys/systm.h>

#include <dev/fdt/fdtvar.h>
#include <dev/sdmmc/sdspivar.h>

#include <arm/rza1/rza1_var.h>

#define	RSPI_SPCR	0x00
#define	 SPCR_MSTR	__BIT(3)
#define	 SPCR_SPE	__BIT(6)
#define	RSPI_SSLP	0x01
#define	RSPI_SPPCR	0x02
#define	 SPPCR_MOIFV	__BIT(4)
#define	 SPPCR_MOIFE	__BIT(5)
#define	RSPI_SPSR	0x03
#define	 SPSR_OVRF	__BIT(0)
#define	 SPSR_SPTEF	__BIT(5)
#define	 SPSR_TEND	__BIT(6)
#define	 SPSR_SPRF	__BIT(7)
#define	RSPI_SPDR	0x04
#define	RSPI_SPSCR	0x08
#define	RSPI_SPBR	0x0a
#define	RSPI_SPDCR	0x0b
#define	 SPDCR_SPLW_BYTE	__SHIFTIN(1, __BITS(6,5))
#define	RSPI_SPCKD	0x0c
#define	RSPI_SSLND	0x0d
#define	RSPI_SPND	0x0e
#define	RSPI_SPCMD0	0x10
#define	 SPCMD_CPHA	__BIT(0)
#define	 SPCMD_CPOL	__BIT(1)
#define	 SPCMD_BRDV	__BITS(3,2)
#define	 SPCMD_SPB_8	__SHIFTIN(7, __BITS(11,8))
#define	RSPI_SPBFCR	0x20
#define	 SPBFCR_TXRST	__BIT(7)
#define	 SPBFCR_RXRST	__BIT(6)
#define	 SPBFCR_TXTRG	__BITS(5,4)

#define	RSPI_TIMEOUT	100000
#define	RSPI_FIFO_LEN	8

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "renesas,rspi-r7s72100" },
	DEVICE_COMPAT_EOL
};

static const struct device_compatible_entry slot_compat[] = {
	{ .compat = "mmc-spi-slot" },
	DEVICE_COMPAT_EOL
};

struct rza1_rspi_softc {
	device_t sc_dev;
	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;
	u_int sc_pclk;
	uint16_t sc_spcmd;

	struct fdtbus_gpio_pin *sc_cs;
	struct fdtbus_gpio_pin *sc_cd;

	struct sdspi_softc sc_sdspi;
};

#define	RD1(sc, reg)		bus_space_read_1((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define	WR1(sc, reg, val)	bus_space_write_1((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))
#define	WR2(sc, reg, val)	bus_space_write_2((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))

static int	rza1_rspi_match(device_t, cfdata_t, void *);
static void	rza1_rspi_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(rza1_rspi, sizeof(struct rza1_rspi_softc),
    rza1_rspi_match, rza1_rspi_attach, NULL, NULL);

static void
rza1_rspi_hw_init(struct rza1_rspi_softc *sc, uint8_t spbr)
{
	WR1(sc, RSPI_SPCR, 0);
	WR1(sc, RSPI_SPSCR, 0);
	WR1(sc, RSPI_SSLP, 0);
	WR1(sc, RSPI_SPDCR, SPDCR_SPLW_BYTE);
	WR1(sc, RSPI_SPCKD, 0);
	WR1(sc, RSPI_SSLND, 0);
	WR1(sc, RSPI_SPND, 0);

	WR1(sc, RSPI_SPPCR, SPPCR_MOIFE | SPPCR_MOIFV);
	WR1(sc, RSPI_SPBR, spbr);
	WR2(sc, RSPI_SPCMD0, sc->sc_spcmd);
	WR1(sc, RSPI_SPBFCR, SPBFCR_TXRST | SPBFCR_RXRST |
	    __SHIFTIN(3, SPBFCR_TXTRG));
	WR1(sc, RSPI_SPBFCR, __SHIFTIN(3, SPBFCR_TXTRG));
	WR1(sc, RSPI_SPCR, SPCR_MSTR | SPCR_SPE);
}

static int
rza1_rspi_wait(struct rza1_rspi_softc *sc, uint8_t bit)
{
	for (int timo = RSPI_TIMEOUT; timo > 0; timo--) {
		if (RD1(sc, RSPI_SPSR) & bit)
			return 0;
	}
	return ETIMEDOUT;
}

static void
rza1_rspi_select(void *priv, bool on)
{
	struct rza1_rspi_softc * const sc = priv;

	if (sc->sc_cs != NULL)
		fdtbus_gpio_write(sc->sc_cs, on ? 1 : 0);
}

static int
rza1_rspi_xfer(void *priv, const uint8_t *tx, uint8_t *rx, size_t len)
{
	struct rza1_rspi_softc * const sc = priv;
	uint8_t val;
	int error;

	while (len > 0) {
		const size_t n = MIN(len, RSPI_FIFO_LEN);
		size_t i;

		error = rza1_rspi_wait(sc, SPSR_SPTEF);
		if (error)
			return error;
		for (i = 0; i < n; i++)
			WR1(sc, RSPI_SPDR, tx != NULL ? *tx++ : 0xff);
		for (i = 0; i < n; i++) {
			error = rza1_rspi_wait(sc, SPSR_SPRF);
			if (error)
				return error;
			val = RD1(sc, RSPI_SPDR);
			if (rx != NULL)
				*rx++ = val;
		}
		len -= n;
	}
	return rza1_rspi_wait(sc, SPSR_TEND);
}

static int
rza1_rspi_set_clock(void *priv, u_int khz)
{
	struct rza1_rspi_softc * const sc = priv;
	const uint64_t rate = (uint64_t)khz * 1000;
	u_int brdv, div;

	if (khz == 0)
		return 0;

	div = 256;
	for (brdv = 0; brdv < 4; brdv++) {
		div = howmany(sc->sc_pclk, 2 * rate << brdv);
		if (div <= 256)
			break;
	}
	if (brdv == 4) {
		brdv = 3;
		div = 256;
	}
	if (div == 0)
		div = 1;

	sc->sc_spcmd = (sc->sc_spcmd & ~SPCMD_BRDV) |
	    __SHIFTIN(brdv, SPCMD_BRDV);
	rza1_rspi_hw_init(sc, div - 1);

	aprint_debug_dev(sc->sc_dev, "clock %u kHz -> %u Hz\n", khz,
	    sc->sc_pclk / (2 * div << brdv));
	return 0;
}

static int
rza1_rspi_card_detect(void *priv)
{
	struct rza1_rspi_softc * const sc = priv;

	return fdtbus_gpio_read(sc->sc_cd);
}

static const struct sdspi_ops rza1_rspi_sdspi_ops = {
	.select = rza1_rspi_select,
	.xfer = rza1_rspi_xfer,
	.set_clock = rza1_rspi_set_clock,
	.card_detect = rza1_rspi_card_detect,
};

static const struct sdspi_ops rza1_rspi_sdspi_ops_nocd = {
	.select = rza1_rspi_select,
	.xfer = rza1_rspi_xfer,
	.set_clock = rza1_rspi_set_clock,
};

static int
rza1_rspi_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
rza1_rspi_attach(device_t parent, device_t self, void *aux)
{
	struct rza1_rspi_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	struct sdspi_softc * const sds = &sc->sc_sdspi;
	struct clk *clk;
	bus_addr_t addr;
	bus_size_t size;
	uint32_t maxfreq;
	int child, slot = 0;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	clk = fdtbus_clock_get_index(phandle, 0);
	if (clk != NULL && clk_enable(clk) != 0) {
		aprint_error(": couldn't enable clock\n");
		return;
	}
	sc->sc_pclk = clk != NULL ? clk_get_rate(clk) : 0;
	if (sc->sc_pclk == 0)
		sc->sc_pclk = rza1_get_p1_freq();

	fdtbus_pinctrl_set_config(phandle, "default");

	sc->sc_cs = fdtbus_gpio_acquire(phandle, "cs-gpios", GPIO_PIN_OUTPUT);
	if (sc->sc_cs != NULL)
		fdtbus_gpio_write(sc->sc_cs, 0);

	aprint_naive("\n");
	aprint_normal(": RZ/A1 RSPI\n");

	sc->sc_spcmd = SPCMD_SPB_8;
	rza1_rspi_set_clock(sc, 400);

	for (child = OF_child(phandle); child; child = OF_peer(child)) {
		if (fdtbus_status_okay(child) &&
		    of_compatible_match(child, slot_compat)) {
			slot = child;
			break;
		}
	}
	if (slot == 0) {
		aprint_normal_dev(self, "no supported device\n");
		return;
	}
	if (sc->sc_cs == NULL) {
		aprint_error_dev(self, "mmc-spi-slot needs cs-gpios\n");
		return;
	}

	if (of_getprop_uint32(slot, "spi-max-frequency", &maxfreq) != 0)
		maxfreq = 400000;
	sc->sc_cd = fdtbus_gpio_acquire(slot, "cd-gpios", GPIO_PIN_INPUT);

	sds->sc_dev = self;
	sds->sc_ops = sc->sc_cd != NULL ?
	    &rza1_rspi_sdspi_ops : &rza1_rspi_sdspi_ops_nocd;
	sds->sc_cookie = sc;
	sds->sc_clkmin = 100;
	sds->sc_clkmax = maxfreq / 1000;
	sdspi_attach(sds);
}
