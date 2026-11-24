// SPDX-License-Identifier: BSD-2-Clause
/*
 * SD/MMC host in SPI mode.
 *
 * Derived from the SuperH SCI MMC driver (sys/arch/evbsh3/t_sh7706lan/scimci.c):
 * the command/response/data token handling follows that driver,
 * while the register-level SCI access is replaced by the controller callbacks of
 * struct sdspi_ops. Data responses of writes are checked, CRC7 is generated for
 * every command, and the card stays selected for the whole command.
 */

/*-
 * Copyright (C) 2009 NONAKA Kimihiro <nonaka@netbsd.org>
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
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/device.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>

#include <dev/sdmmc/sdmmcvar.h>
#include <dev/sdmmc/sdmmcchip.h>
#include <dev/sdmmc/sdmmcreg.h>
#include <dev/sdmmc/sdspivar.h>

#ifdef SDSPI_DEBUG
int sdspi_debug = 1;
#define DPRINTF(n,s)	do { if ((n) <= sdspi_debug) printf s; } while (0)
#else
#define DPRINTF(n,s)	do {} while (0)
#endif

#define	SDSPI_NCR		16	/* bytes before a command response */
#define	SDSPI_TOKEN_TRIES	200000	/* data token, ~1us per try */
#define	SDSPI_BUSY_TRIES	100000	/* busy wait, 10us per try */

#define	SDSPI_TOKEN_START	0xfe	/* start block (single block) */
#define	SDSPI_DATA_RESP_MASK	0x1f
#define	SDSPI_DATA_RESP_OK	0x05

static int	sdspi_host_reset(sdmmc_chipset_handle_t);
static uint32_t	sdspi_host_ocr(sdmmc_chipset_handle_t);
static int	sdspi_host_maxblklen(sdmmc_chipset_handle_t);
static int	sdspi_card_detect(sdmmc_chipset_handle_t);
static int	sdspi_write_protect(sdmmc_chipset_handle_t);
static int	sdspi_bus_power(sdmmc_chipset_handle_t, uint32_t);
static int	sdspi_bus_clock(sdmmc_chipset_handle_t, int);
static int	sdspi_bus_width(sdmmc_chipset_handle_t, int);
static void	sdspi_exec_command(sdmmc_chipset_handle_t,
		    struct sdmmc_command *);
static void	sdspi_spi_initialize(sdmmc_chipset_handle_t);

static struct sdmmc_chip_functions sdspi_chip_functions = {
	.host_reset		= sdspi_host_reset,
	.host_ocr		= sdspi_host_ocr,
	.host_maxblklen		= sdspi_host_maxblklen,
	.card_detect		= sdspi_card_detect,
	.write_protect		= sdspi_write_protect,
	.bus_power		= sdspi_bus_power,
	.bus_clock		= sdspi_bus_clock,
	.bus_width		= sdspi_bus_width,
	.exec_command		= sdspi_exec_command,
	.card_enable_intr	= NULL,
	.card_intr_ack		= NULL,
};

static struct sdmmc_spi_chip_functions sdspi_spi_chip_functions = {
	.initialize		= sdspi_spi_initialize,
};

static inline void
sdspi_select(struct sdspi_softc *sc, bool on)
{
	sc->sc_ops->select(sc->sc_cookie, on);
}

static inline int
sdspi_xfer(struct sdspi_softc *sc, const uint8_t *tx, uint8_t *rx, size_t len)
{
	return sc->sc_ops->xfer(sc->sc_cookie, tx, rx, len);
}

static int
sdspi_getc(struct sdspi_softc *sc)
{
	uint8_t c;

	if (sdspi_xfer(sc, NULL, &c, 1) != 0)
		return -1;
	return c;
}

static int
sdspi_putc(struct sdspi_softc *sc, uint8_t c)
{
	return sdspi_xfer(sc, &c, NULL, 1);
}

/* CRC7 (x^7 + x^3 + 1) */
static uint8_t
sdspi_crc7(const uint8_t *buf, size_t len)
{
	uint8_t crc = 0;

	for (size_t i = 0; i < len; i++) {
		uint8_t c = buf[i];
		for (int bit = 0; bit < 8; bit++) {
			crc <<= 1;
			if (((c ^ crc) & 0x80) != 0)
				crc ^= 0x09;
			c <<= 1;
		}
	}
	return (crc << 1) | 1;
}

void
sdspi_attach(struct sdspi_softc *sc)
{
	struct sdmmcbus_attach_args saa;

	if (sc->sc_clkmin == 0)
		sc->sc_clkmin = 100;
	if (sc->sc_clkmax == 0)
		sc->sc_clkmax = 400;

	sdspi_select(sc, false);
	sc->sc_ops->set_clock(sc->sc_cookie, 400);

	memset(&saa, 0, sizeof(saa));
	saa.saa_busname = "sdmmc";
	saa.saa_sct = &sdspi_chip_functions;
	saa.saa_spi_sct = &sdspi_spi_chip_functions;
	saa.saa_sch = sc;
	saa.saa_clkmin = sc->sc_clkmin;
	saa.saa_clkmax = sc->sc_clkmax;
	saa.saa_caps = SMC_CAPS_SPI_MODE
		     | SMC_CAPS_SINGLE_ONLY
		     | SMC_CAPS_POLL_CARD_DET;

	sc->sc_sdmmc = config_found(sc->sc_dev, &saa, NULL, CFARGS_NONE);
	if (sc->sc_sdmmc == NULL)
		aprint_error_dev(sc->sc_dev, "couldn't attach sdmmc bus\n");
}

static int
sdspi_host_reset(sdmmc_chipset_handle_t sch)
{
	return 0;
}

static uint32_t
sdspi_host_ocr(sdmmc_chipset_handle_t sch)
{
	return MMC_OCR_3_2V_3_3V | MMC_OCR_3_3V_3_4V;
}

static int
sdspi_host_maxblklen(sdmmc_chipset_handle_t sch)
{
	return 512;
}

static int
sdspi_card_detect(sdmmc_chipset_handle_t sch)
{
	struct sdspi_softc * const sc = sch;

	if (sc->sc_ops->card_detect == NULL)
		return 1;
	return sc->sc_ops->card_detect(sc->sc_cookie);
}

static int
sdspi_write_protect(sdmmc_chipset_handle_t sch)
{
	return 0;
}

static int
sdspi_bus_power(sdmmc_chipset_handle_t sch, uint32_t ocr)
{
	if ((ocr & (MMC_OCR_3_2V_3_3V | MMC_OCR_3_3V_3_4V)) == 0)
		return 1;
	return 0;
}

static int
sdspi_bus_clock(sdmmc_chipset_handle_t sch, int freq)
{
	struct sdspi_softc * const sc = sch;

	return sc->sc_ops->set_clock(sc->sc_cookie, freq);
}

static int
sdspi_bus_width(sdmmc_chipset_handle_t sch, int width)
{
	return width == 1 ? 0 : 1;
}

static void
sdspi_spi_initialize(sdmmc_chipset_handle_t sch)
{
	struct sdspi_softc * const sc = sch;
	uint8_t buf[10];

	memset(buf, 0xff, sizeof(buf));
	sdspi_select(sc, false);
	sdspi_xfer(sc, buf, NULL, sizeof(buf));
}

static int
sdspi_wait_busy(struct sdspi_softc *sc)
{
	int c;

	for (int timo = SDSPI_BUSY_TRIES; timo > 0; timo--) {
		c = sdspi_getc(sc);
		if (c < 0)
			return EIO;
		if (c == 0xff)
			return 0;
		delay(10);
	}
	return ETIMEDOUT;
}

static int
sdspi_wait_token(struct sdspi_softc *sc, int opcode)
{
	int c;

	for (int timo = SDSPI_TOKEN_TRIES; timo > 0; timo--) {
		c = sdspi_getc(sc);
		if (c < 0)
			return EIO;
		if (c == 0xff) {
			delay(1);
			continue;
		}
		if (c == SDSPI_TOKEN_START)
			return 0;
		if (c == 0x00 && timo == SDSPI_TOKEN_TRIES)
			continue;
		device_printf(sc->sc_dev, "CMD%d: data error token %#x\n",
		    opcode, c);
		return EIO;
	}
	device_printf(sc->sc_dev, "CMD%d: data token timeout\n", opcode);
	return ETIMEDOUT;
}

static int
sdspi_read_block(struct sdspi_softc *sc, int opcode, uint8_t *data,
    size_t len)
{
	uint8_t crc[2];
	int error;

	error = sdspi_wait_token(sc, opcode);
	if (error)
		return error;
	error = sdspi_xfer(sc, NULL, data, len);
	if (error)
		return error;
	return sdspi_xfer(sc, NULL, crc, sizeof(crc));
}

static int
sdspi_write_block(struct sdspi_softc *sc, int opcode, const uint8_t *data,
    size_t len)
{
	static const uint8_t start[2] = { 0xff, SDSPI_TOKEN_START };
	static const uint8_t crc[2] = { 0xff, 0xff };
	int c, error;

	error = sdspi_xfer(sc, start, NULL, sizeof(start));
	if (error)
		return error;
	error = sdspi_xfer(sc, data, NULL, len);
	if (error)
		return error;
	error = sdspi_xfer(sc, crc, NULL, sizeof(crc));
	if (error)
		return error;

	for (int i = 0; i < SDSPI_NCR; i++) {
		c = sdspi_getc(sc);
		if (c < 0)
			return EIO;
		if (c == 0xff)
			continue;
		if ((c & SDSPI_DATA_RESP_MASK) != SDSPI_DATA_RESP_OK) {
			device_printf(sc->sc_dev,
			    "CMD%d: write rejected (data response %#x)\n",
			    opcode, c);
			(void)sdspi_wait_busy(sc);
			return EIO;
		}
		return sdspi_wait_busy(sc);
	}
	device_printf(sc->sc_dev, "CMD%d: no data response\n", opcode);
	return ETIMEDOUT;
}

static void
sdspi_cxd_to_response(uint8_t *data)
{
	uint8_t shifted[16];
	uint32_t res[4];

	shifted[0] = 0;
	memcpy(&shifted[1], data, 15);
	for (int i = 0; i < 4; i++)
		res[i] = be32dec(&shifted[12 - 4 * i]);
	memcpy(data, res, sizeof(res));
}

static void
sdspi_exec_command(sdmmc_chipset_handle_t sch, struct sdmmc_command *cmd)
{
	struct sdspi_softc * const sc = sch;
	uint8_t frame[7];
	uint32_t resp;
	int c, error = 0;
	int i;

	DPRINTF(1,("%s: start cmd %d arg=%#x data=%p dlen=%d flags=%#x\n",
	    device_xname(sc->sc_dev), cmd->c_opcode, cmd->c_arg,
	    cmd->c_data, cmd->c_datalen, cmd->c_flags));

	sdspi_select(sc, true);

	frame[0] = 0xff;
	frame[1] = 0x40 | (cmd->c_opcode & 0x3f);
	be32enc(&frame[2], cmd->c_arg);
	frame[6] = sdspi_crc7(&frame[1], 5);
	error = sdspi_xfer(sc, frame, NULL, sizeof(frame));
	if (error)
		goto out;

	for (i = 0, c = 0xff; i < SDSPI_NCR; i++) {
		c = sdspi_getc(sc);
		if (c < 0 || (c & 0x80) == 0)
			break;
	}
	if (c < 0) {
		error = EIO;
		goto out;
	}
	if (c & 0x80) {
		DPRINTF(1, ("%s: response timeout\n",
		    device_xname(sc->sc_dev)));
		error = ETIMEDOUT;
		goto out;
	}
	resp = c;

	if (ISSET(cmd->c_flags, SCF_RSP_SPI_S2)) {
		c = sdspi_getc(sc);
		if (c < 0) {
			error = EIO;
			goto out;
		}
		resp |= (uint32_t)c << 8;
	} else if (ISSET(cmd->c_flags, SCF_RSP_SPI_B4)) {
		uint8_t b4[4];

		error = sdspi_xfer(sc, NULL, b4, sizeof(b4));
		if (error)
			goto out;
		cmd->c_resp[1] = be32dec(b4);
		DPRINTF(1, ("R3/R7 resp: %#x\n", cmd->c_resp[1]));
	}
	cmd->c_resp[0] = resp;

	if ((resp & 0xff & ~R1_SPI_IDLE) != 0) {
		DPRINTF(1, ("%s: response error: %#x\n",
		    device_xname(sc->sc_dev), resp));
		error = EIO;
		goto out;
	}

	if (ISSET(cmd->c_flags, SCF_RSP_SPI_BSY)) {
		error = sdspi_wait_busy(sc);
		if (error)
			goto out;
	}

	if (cmd->c_datalen > 0) {
		uint8_t *data = cmd->c_data;
		const int blklen = cmd->c_blklen > 0 ?
		    MIN(cmd->c_blklen, cmd->c_datalen) : cmd->c_datalen;

		for (i = 0; i < cmd->c_datalen && error == 0; i += blklen) {
			if (ISSET(cmd->c_flags, SCF_CMD_READ))
				error = sdspi_read_block(sc, cmd->c_opcode,
				    data + i, blklen);
			else
				error = sdspi_write_block(sc, cmd->c_opcode,
				    data + i, blklen);
		}
		if (error == 0 && ISSET(cmd->c_flags, SCF_CMD_READ) &&
		    (cmd->c_opcode == MMC_SEND_CID ||
		     cmd->c_opcode == MMC_SEND_CSD) && cmd->c_datalen == 16)
			sdspi_cxd_to_response(cmd->c_data);
	}

out:
	sdspi_select(sc, false);
	(void)sdspi_putc(sc, 0xff);

	cmd->c_error = error;
	SET(cmd->c_flags, SCF_ITSDONE);

	DPRINTF(1,("%s: cmd %d done (flags=%#x error=%d)\n",
	    device_xname(sc->sc_dev), cmd->c_opcode, cmd->c_flags,
	    cmd->c_error));
}
