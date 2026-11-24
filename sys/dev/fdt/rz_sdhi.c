// SPDX-License-Identifier: BSD-2-Clause
/*
 * Renesas SD host interface (SDHI) of the RZ/G2L family (RZ/Five
 * R9A07G043F, RZ/G2UL) and of the RZ/A2 (R7S9210), programmed I/O only.
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/kernel.h>
#include <sys/systm.h>

#include <dev/fdt/fdtvar.h>

#include <dev/sdmmc/sdmmcchip.h>
#include <dev/sdmmc/sdmmcvar.h>
#include <dev/sdmmc/sdmmc_ioreg.h>

/* Registers (byte offsets) */
#define	SD_CMD		0x000
#define	 CMD_APP		0x0040	/* ACMD (previous command was CMD55) */
#define	 CMD_RESP_NONE		0x0300
#define	 CMD_RESP_R1		0x0400	/* R1, R5, R6, R7 */
#define	 CMD_RESP_R1B		0x0500
#define	 CMD_RESP_R2		0x0600
#define	 CMD_RESP_R3		0x0700	/* R3, R4: no CRC */
#define	 CMD_DATA		0x0800
#define	 CMD_READ		0x1000
#define	 CMD_MULTI		0x2000
#define	SD_PORTSEL	0x008
#define	SD_ARG0		0x010
#define	SD_ARG1		0x018
#define	SD_STOP		0x020
#define	 STOP_SEC		0x0100	/* sector count enable: auto CMD12 */
#define	SD_SECCNT	0x028
#define	SD_RSP(n)	(0x030 + 8 * (n))	/* RSP00 .. RSP07 */
#define	SD_INFO1	0x070
#define	 INFO1_RESP_END		0x0001
#define	 INFO1_ACCESS_END	0x0004
#define	 INFO1_CD_REMOVE	0x0008
#define	 INFO1_CD_INSERT	0x0010
#define	 INFO1_CD		0x0020	/* card present */
#define	 INFO1_WP		0x0080
#define	SD_INFO2	0x078
#define	 INFO2_ERR_CMD		0x0001
#define	 INFO2_ERR_CRC		0x0002
#define	 INFO2_ERR_END		0x0004
#define	 INFO2_ERR_DTO		0x0008	/* data timeout */
#define	 INFO2_ERR_ILW		0x0010
#define	 INFO2_ERR_ILR		0x0020
#define	 INFO2_ERR_RTO		0x0040	/* response timeout */
#define	 INFO2_BRE		0x0100	/* read buffer ready */
#define	 INFO2_BWE		0x0200	/* write buffer ready */
#define	 INFO2_SCLKDIVEN	0x2000	/* clock may be changed */
#define	 INFO2_CBSY		0x4000	/* command / busy */
#define	 INFO2_ILA		0x8000
#define	 INFO2_ERRORS		0x807f
#define	SD_INFO1_MASK	0x080
#define	SD_INFO2_MASK	0x088
#define	SD_CLK_CTRL	0x090
#define	 CLK_SCLKEN		0x0100
#define	 CLK_DIV_MASK		0x00ff	/* 0x00: /2, 0x01: /4 ... 0x80: /512 */
#define	SD_SIZE		0x098
#define	SD_OPTION	0x0a0
#define	 OPTION_WIDTH_MASK	0xe000
#define	 OPTION_WIDTH_1		0x8000
#define	 OPTION_WIDTH_4		0x0000
#define	 OPTION_WIDTH_8		0x2000
#define	SD_ERR_STS1	0x0b0
#define	SD_ERR_STS2	0x0b8
#define	SD_BUF0		0x0c0	/* 64-bit data port */
#define	SOFT_RST	0x380
#define	 SOFT_RST_RUN		0x0001
#define	SD_VERSION	0x388
#define	HOST_MODE	0x390
#define	 HOST_MODE_WMODE	0x0001	/* 1: 16/32-bit SD_BUF0 access */
#define	 HOST_MODE_BUSWIDTH	0x0100	/* with WMODE: 1 = 32 bits */

struct rz_sdhi_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	int			sc_phandle;
	u_int			sc_clkbase;	/* core clock, Hz */
	bool			sc_cd_fixed;	/* broken-cd / non-removable */
	bool			sc_cd_state;	/* last card detect state */
	u_int			sc_cd_resettime; /* ticks at the last reset */
	bool			sc_app;		/* next command is an ACMD */
	device_t		sc_sdmmc;
};

/*
 * SD_INFO1.CD follows the card detect pin once the pin has been stable
 * for the period set in SD_OPTION (2^24 cycles at the reset value, about
 * 0.13 s at 132 MHz).  Whether a soft reset restarts that is not
 * documented, so for this long after a reset the last state is reported.
 */
#define	RZ_SDHI_CD_SETTLE_MS	250

#define	RD2(sc, r)	bus_space_read_2((sc)->sc_bst, (sc)->sc_bsh, (r))
#define	WR2(sc, r, v)	bus_space_write_2((sc)->sc_bst, (sc)->sc_bsh, (r), (v))

static int	rz_sdhi_match(device_t, cfdata_t, void *);
static void	rz_sdhi_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(rz_sdhi, sizeof(struct rz_sdhi_softc),
    rz_sdhi_match, rz_sdhi_attach, NULL, NULL);

static const struct device_compatible_entry compat_data[] = {
	/* value: core clock if the device tree gives none, Hz */
	{ .compat = "renesas,sdhi-r9a07g043", .value = 133333333 },
	{ .compat = "renesas,sdhi-r7s9210", .value = 132000000 },	/* B */
	DEVICE_COMPAT_EOL
};

static int	rz_sdhi_host_reset(sdmmc_chipset_handle_t);
static uint32_t	rz_sdhi_host_ocr(sdmmc_chipset_handle_t);
static int	rz_sdhi_host_maxblklen(sdmmc_chipset_handle_t);
static int	rz_sdhi_card_detect(sdmmc_chipset_handle_t);
static int	rz_sdhi_write_protect(sdmmc_chipset_handle_t);
static int	rz_sdhi_bus_power(sdmmc_chipset_handle_t, uint32_t);
static int	rz_sdhi_bus_clock(sdmmc_chipset_handle_t, int);
static int	rz_sdhi_bus_width(sdmmc_chipset_handle_t, int);
static int	rz_sdhi_bus_rod(sdmmc_chipset_handle_t, int);
static void	rz_sdhi_exec_command(sdmmc_chipset_handle_t,
		    struct sdmmc_command *);
static void	rz_sdhi_card_enable_intr(sdmmc_chipset_handle_t, int);
static void	rz_sdhi_card_intr_ack(sdmmc_chipset_handle_t);

static struct sdmmc_chip_functions rz_sdhi_chip_functions = {
	.host_reset = rz_sdhi_host_reset,
	.host_ocr = rz_sdhi_host_ocr,
	.host_maxblklen = rz_sdhi_host_maxblklen,
	.card_detect = rz_sdhi_card_detect,
	.write_protect = rz_sdhi_write_protect,
	.bus_power = rz_sdhi_bus_power,
	.bus_clock = rz_sdhi_bus_clock,
	.bus_width = rz_sdhi_bus_width,
	.bus_rod = rz_sdhi_bus_rod,
	.exec_command = rz_sdhi_exec_command,
	.card_enable_intr = rz_sdhi_card_enable_intr,
	.card_intr_ack = rz_sdhi_card_intr_ack,
};

/* Busy-wait in small steps, sleeping once the system is running. */
static int
rz_sdhi_wait(struct rz_sdhi_softc *sc, bus_size_t reg, uint16_t bits,
    bool set, u_int timeout_us)
{
	for (u_int t = 0; t < timeout_us; t += 10) {
		const uint16_t v = RD2(sc, reg);
		if (((v & bits) != 0) == set)
			return 0;
		if (reg != SD_INFO2 && (RD2(sc, SD_INFO2) & INFO2_ERRORS) != 0)
			return EIO;
		delay(10);
	}
	return ETIMEDOUT;
}

/* Width of the SD_BUF0 accesses, see rz_sdhi_pio() */
static void
rz_sdhi_host_mode(struct rz_sdhi_softc *sc)
{
#ifndef _LP64
	WR2(sc, HOST_MODE, HOST_MODE_WMODE | HOST_MODE_BUSWIDTH);
#endif
}

static void
rz_sdhi_soft_reset(struct rz_sdhi_softc *sc)
{
	const uint16_t clk = RD2(sc, SD_CLK_CTRL);
	const uint16_t opt = RD2(sc, SD_OPTION);

	WR2(sc, SOFT_RST, 0);
	WR2(sc, SOFT_RST, SOFT_RST_RUN);
	sc->sc_cd_resettime = getticks();
	rz_sdhi_host_mode(sc);
	WR2(sc, SD_CLK_CTRL, clk);
	WR2(sc, SD_OPTION, opt);
	WR2(sc, SD_INFO1_MASK, 0xffff);
	WR2(sc, SD_INFO2_MASK, 0xffff);
	sc->sc_app = false;
}

static int
rz_sdhi_host_reset(sdmmc_chipset_handle_t sch)
{
	struct rz_sdhi_softc * const sc = sch;

	WR2(sc, SOFT_RST, 0);
	WR2(sc, SOFT_RST, SOFT_RST_RUN);
	sc->sc_cd_resettime = getticks();
	rz_sdhi_host_mode(sc);
	WR2(sc, SD_PORTSEL, 0x0100);
	WR2(sc, SD_INFO1_MASK, 0xffff);		/* polled */
	WR2(sc, SD_INFO2_MASK, 0xffff);
	WR2(sc, SD_INFO1, 0);
	WR2(sc, SD_INFO2, 0);
	sc->sc_app = false;
	return 0;
}

static uint32_t
rz_sdhi_host_ocr(sdmmc_chipset_handle_t sch)
{

	return MMC_OCR_3_2V_3_3V | MMC_OCR_3_3V_3_4V;
}

static int
rz_sdhi_host_maxblklen(sdmmc_chipset_handle_t sch)
{

	return 512;
}

static int
rz_sdhi_card_detect(sdmmc_chipset_handle_t sch)
{
	struct rz_sdhi_softc * const sc = sch;

	if (sc->sc_cd_fixed)
		return 1;
	if ((u_int)getticks() - sc->sc_cd_resettime >=
	    (u_int)mstohz(RZ_SDHI_CD_SETTLE_MS))
		sc->sc_cd_state = (RD2(sc, SD_INFO1) & INFO1_CD) != 0;
	return sc->sc_cd_state;
}

static int
rz_sdhi_write_protect(sdmmc_chipset_handle_t sch)
{

	return 0;	/* microSD: no write protect switch */
}

static int
rz_sdhi_bus_power(sdmmc_chipset_handle_t sch, uint32_t ocr)
{

	/* Card power (P0_3) and 3.3V I/O (P6_1) are set up by U-Boot. */
	return 0;
}

static int
rz_sdhi_bus_clock(sdmmc_chipset_handle_t sch, int freq)
{
	struct rz_sdhi_softc * const sc = sch;
	u_int div, code;

	if (rz_sdhi_wait(sc, SD_INFO2, INFO2_CBSY, false, 100000) != 0)
		return EBUSY;
	WR2(sc, SD_CLK_CTRL, RD2(sc, SD_CLK_CTRL) & ~CLK_SCLKEN);
	if (freq == 0)
		return 0;

	/* Smallest power of two divider 2..512 with base / div <= freq. */
	for (div = 2, code = 0; div < 512; div <<= 1) {
		if (sc->sc_clkbase / div <= (u_int)freq * 1000)
			break;
		code = code == 0 ? 1 : code << 1;
	}
	if (rz_sdhi_wait(sc, SD_INFO2, INFO2_SCLKDIVEN, true, 100000) != 0)
		return EBUSY;
	WR2(sc, SD_CLK_CTRL, code & CLK_DIV_MASK);
	WR2(sc, SD_CLK_CTRL, (code & CLK_DIV_MASK) | CLK_SCLKEN);
	return 0;
}

static int
rz_sdhi_bus_width(sdmmc_chipset_handle_t sch, int width)
{
	struct rz_sdhi_softc * const sc = sch;
	uint16_t opt = RD2(sc, SD_OPTION) & ~OPTION_WIDTH_MASK;

	switch (width) {
	case 1:
		opt |= OPTION_WIDTH_1;
		break;
	case 4:
		opt |= OPTION_WIDTH_4;
		break;
	case 8:
		opt |= OPTION_WIDTH_8;
		break;
	default:
		return EINVAL;
	}
	WR2(sc, SD_OPTION, opt);
	return 0;
}

static int
rz_sdhi_bus_rod(sdmmc_chipset_handle_t sch, int on)
{

	return on ? ENOTSUP : 0;
}

static int
rz_sdhi_pio(struct rz_sdhi_softc *sc, struct sdmmc_command *cmd)
{
	const bool rd = ISSET(cmd->c_flags, SCF_CMD_READ);
	const uint16_t ready = rd ? INFO2_BRE : INFO2_BWE;
	uint8_t *p = cmd->c_data;
	int resid = cmd->c_datalen;
	int error;

	while (resid > 0) {
		const int n = MIN(resid, cmd->c_blklen);

		error = rz_sdhi_wait(sc, SD_INFO2, ready | INFO2_ERRORS, true,
		    1000000);
		if (error)
			return error;
		if (RD2(sc, SD_INFO2) & INFO2_ERRORS)
			return EIO;
		WR2(sc, SD_INFO2, (uint16_t)~ready);

#ifdef _LP64
		for (int i = 0; i < n; i += 8) {
			uint64_t w;

			if (rd) {
				w = bus_space_read_8(sc->sc_bst, sc->sc_bsh,
				    SD_BUF0);
				memcpy(p + i, &w, MIN(8, n - i));
			} else {
				w = 0;
				memcpy(&w, p + i, MIN(8, n - i));
				bus_space_write_8(sc->sc_bst, sc->sc_bsh,
				    SD_BUF0, w);
			}
		}
#else
		/* HOST_MODE selects 32-bit accesses (rz_sdhi_host_mode) */
		for (int i = 0; i < n; i += 4) {
			uint32_t w;

			if (rd) {
				w = bus_space_read_4(sc->sc_bst, sc->sc_bsh,
				    SD_BUF0);
				memcpy(p + i, &w, MIN(4, n - i));
			} else {
				w = 0;
				memcpy(&w, p + i, MIN(4, n - i));
				bus_space_write_4(sc->sc_bst, sc->sc_bsh,
				    SD_BUF0, w);
			}
		}
#endif
		p += n;
		resid -= n;
	}
	return 0;
}

static void
rz_sdhi_exec_command(sdmmc_chipset_handle_t sch, struct sdmmc_command *cmd)
{
	struct rz_sdhi_softc * const sc = sch;
	const bool app = sc->sc_app;
	uint16_t c, info2;
	int error;

	sc->sc_app = false;

	if (rz_sdhi_wait(sc, SD_INFO2, INFO2_CBSY, false, 1000000) != 0) {
		cmd->c_error = EBUSY;
		goto done;
	}
	WR2(sc, SD_INFO1, 0);
	WR2(sc, SD_INFO2, 0);

	c = cmd->c_opcode & 0x3f;
	if (app)
		c |= CMD_APP;
	if (!ISSET(cmd->c_flags, SCF_RSP_PRESENT))
		c |= CMD_RESP_NONE;
	else if (ISSET(cmd->c_flags, SCF_RSP_136))
		c |= CMD_RESP_R2;
	else if (ISSET(cmd->c_flags, SCF_RSP_BSY))
		c |= CMD_RESP_R1B;
	else if (!ISSET(cmd->c_flags, SCF_RSP_CRC))
		c |= CMD_RESP_R3;
	else
		c |= CMD_RESP_R1;

	const bool data = cmd->c_datalen > 0;
	const int nblks = data ? howmany(cmd->c_datalen, cmd->c_blklen) : 0;
	if (data) {
		c |= CMD_DATA;
		if (ISSET(cmd->c_flags, SCF_CMD_READ))
			c |= CMD_READ;
		WR2(sc, SD_SIZE, cmd->c_blklen);
		if (nblks > 1) {
			c |= CMD_MULTI;
			WR2(sc, SD_STOP,
			    ISSET(cmd->c_flags, SCF_NO_STOP) ? 0 : STOP_SEC);
			WR2(sc, SD_SECCNT, nblks);
		} else
			WR2(sc, SD_STOP, 0);
	}

	WR2(sc, SD_ARG0, cmd->c_arg & 0xffff);
	WR2(sc, SD_ARG1, cmd->c_arg >> 16);
	WR2(sc, SD_CMD, c);

	error = rz_sdhi_wait(sc, SD_INFO1, INFO1_RESP_END, true, 1000000);
	info2 = RD2(sc, SD_INFO2);
	if (error != 0 || (info2 & INFO2_ERRORS) != 0) {
		cmd->c_error = (info2 & INFO2_ERR_RTO) ? ETIMEDOUT :
		    error ? error : EIO;
		if (!ISSET(cmd->c_flags, SCF_TOUT_OK) ||
		    cmd->c_error != ETIMEDOUT)
			aprint_debug_dev(sc->sc_dev,
			    "CMD%u failed: info2 %#x sts1 %#x sts2 %#x\n",
			    cmd->c_opcode, info2, RD2(sc, SD_ERR_STS1),
			    RD2(sc, SD_ERR_STS2));
		rz_sdhi_soft_reset(sc);
		goto done;
	}
	WR2(sc, SD_INFO1, (uint16_t)~INFO1_RESP_END);

	if (ISSET(cmd->c_flags, SCF_RSP_PRESENT)) {
		/*
		 * RSP00..RSP07 hold response bits 127:8 (the CRC is not
		 * kept), lowest first: the layout sdmmc(4) expects.
		 */
		const int nw = ISSET(cmd->c_flags, SCF_RSP_136) ? 4 : 1;
		for (int i = 0; i < nw; i++)
			cmd->c_resp[i] = RD2(sc, SD_RSP(2 * i)) |
			    ((uint32_t)RD2(sc, SD_RSP(2 * i + 1)) << 16);
	}

	if (data) {
		error = rz_sdhi_pio(sc, cmd);
		if (error == 0)
			error = rz_sdhi_wait(sc, SD_INFO1, INFO1_ACCESS_END,
			    true, 1000000);
		if (error != 0 || (RD2(sc, SD_INFO2) & INFO2_ERRORS) != 0) {
			cmd->c_error = error ? error : EIO;
			aprint_debug_dev(sc->sc_dev,
			    "CMD%u data failed: info2 %#x\n", cmd->c_opcode,
			    RD2(sc, SD_INFO2));
			rz_sdhi_soft_reset(sc);
			goto done;
		}
	} else if (ISSET(cmd->c_flags, SCF_RSP_BSY)) {
		(void)rz_sdhi_wait(sc, SD_INFO2, INFO2_CBSY, false, 1000000);
	}

	WR2(sc, SD_INFO1, 0);
	WR2(sc, SD_INFO2, 0);
	cmd->c_error = 0;
	if (cmd->c_opcode == MMC_APP_CMD)
		sc->sc_app = true;
done:
	SET(cmd->c_flags, SCF_ITSDONE);
}

static void
rz_sdhi_card_enable_intr(sdmmc_chipset_handle_t sch, int enable)
{
}

static void
rz_sdhi_card_intr_ack(sdmmc_chipset_handle_t sch)
{
}

static int
rz_sdhi_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
rz_sdhi_attach(device_t parent, device_t self, void *aux)
{
	struct rz_sdhi_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	struct sdmmcbus_attach_args saa;
	struct fdtbus_reset *rst;
	struct clk *clk;
	bus_addr_t addr;
	bus_size_t size;
	uint32_t maxfreq;
	u_int i, clkmax;

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	sc->sc_phandle = phandle;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0 ||
	    bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}

	/* core, clkh, cd, aclk */
	for (i = 0; (clk = fdtbus_clock_get_index(phandle, i)) != NULL; i++) {
		if (clk_enable(clk) != 0) {
			aprint_error(": couldn't enable clock %u\n", i);
			return;
		}
	}
	clk = fdtbus_clock_get(phandle, "core");
	sc->sc_clkbase = clk != NULL ? clk_get_rate(clk) : 0;
	if (sc->sc_clkbase == 0)
		sc->sc_clkbase =
		    of_compatible_lookup(phandle, compat_data)->value;
	sc->sc_cd_fixed = of_hasprop(phandle, "broken-cd") ||
	    of_hasprop(phandle, "non-removable");
	rst = fdtbus_reset_get_index(phandle, 0);
	if (rst != NULL)
		fdtbus_reset_deassert(rst);

	aprint_naive("\n");
	aprint_normal(": SDHI (version %#x, %u Hz, PIO)\n",
	    RD2(sc, SD_VERSION), sc->sc_clkbase);

	rz_sdhi_host_reset(sc);
	WR2(sc, SD_OPTION, (RD2(sc, SD_OPTION) & ~OPTION_WIDTH_MASK) |
	    OPTION_WIDTH_1);

	/*
	 * sdmmc(4) looks for a card once before the root file system is
	 * chosen: give a card present at boot time to be detected.
	 */
	if (!sc->sc_cd_fixed) {
		for (i = 0; i < RZ_SDHI_CD_SETTLE_MS; i++) {
			if ((RD2(sc, SD_INFO1) & INFO1_CD) != 0)
				break;
			delay(1000);
		}
		sc->sc_cd_state = (RD2(sc, SD_INFO1) & INFO1_CD) != 0;
		sc->sc_cd_resettime = (u_int)getticks() -
		    (u_int)mstohz(RZ_SDHI_CD_SETTLE_MS);
	}

	memset(&saa, 0, sizeof(saa));
	saa.saa_busname = "sdmmc";
	saa.saa_sct = &rz_sdhi_chip_functions;
	saa.saa_sch = sc;
	saa.saa_dmat = faa->faa_dmat;
	saa.saa_caps = SMC_CAPS_4BIT_MODE | SMC_CAPS_AUTO_STOP |
	    SMC_CAPS_POLLING | SMC_CAPS_POLL_CARD_DET;

	/*
	 * Default speed (25 MHz), or high speed (50 MHz, still 3.3 V
	 * signalling) when the device tree says "cap-sd-highspeed".  SD_CLK
	 * is the core clock divided by a power of two, so the bus may run
	 * slower than that: high speed on RZ/A2 is 132 MHz / 4 = 33 MHz,
	 * default speed 132 MHz / 8 = 16.5 MHz.
	 */
	clkmax = 25000;
	if (of_hasprop(phandle, "cap-sd-highspeed")) {
		saa.saa_caps |= SMC_CAPS_SD_HIGHSPEED;
		clkmax = 50000;
	}
	if (of_getprop_uint32(phandle, "max-frequency", &maxfreq) == 0 &&
	    maxfreq / 1000 < clkmax)
		clkmax = maxfreq / 1000;
	saa.saa_clkmin = sc->sc_clkbase / 512 / 1000;
	saa.saa_clkmax = MIN(sc->sc_clkbase / 2 / 1000, clkmax);
	sc->sc_sdmmc = config_found(self, &saa, NULL, CFARGS_NONE);
}
