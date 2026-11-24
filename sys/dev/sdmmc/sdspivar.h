// SPDX-License-Identifier: BSD-2-Clause
/*
 * SD/MMC host in SPI mode on top of a byte-oriented SPI controller.
 */

#ifndef _DEV_SDMMC_SDSPIVAR_H_
#define _DEV_SDMMC_SDSPIVAR_H_

#include <sys/device.h>

struct sdspi_ops {
	void	(*select)(void *, bool);
	int	(*xfer)(void *, const uint8_t *, uint8_t *, size_t);
	int	(*set_clock)(void *, u_int);
	int	(*card_detect)(void *);
};

struct sdspi_softc {
	device_t		sc_dev;
	const struct sdspi_ops	*sc_ops;
	void			*sc_cookie;
	u_int			sc_clkmin;
	u_int			sc_clkmax;

	device_t		sc_sdmmc;
};

void	sdspi_attach(struct sdspi_softc *);

#endif /* _DEV_SDMMC_SDSPIVAR_H_ */
