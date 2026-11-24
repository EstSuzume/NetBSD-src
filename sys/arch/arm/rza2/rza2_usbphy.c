// SPDX-License-Identifier: BSD-2-Clause
/*
 * Renesas RZ/A2 USB 2.0 host: the registers next to the OHCI and EHCI ones
 * ("renesas,usb2-phy-r7s9210", offset 200h-8FFh of a channel), set up as a
 * PHY for the generic ehci(4)/ohci(4) FDT attachments, host mode only.
 */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/mutex.h>
#include <sys/systm.h>

#include <dev/fdt/fdtvar.h>

#define	USBPHY_INT_ENABLE	0x000
#define	 INT_ENABLE_USBH_INTBEN	__BIT(2)	/* EHCI */
#define	 INT_ENABLE_USBH_INTAEN	__BIT(1)	/* OHCI */
#define	USBPHY_USBCTR		0x00c
#define	 USBCTR_DIRPD		__BIT(2)	/* PHY standby */
#define	 USBCTR_PLL_RST		__BIT(1)
#define	USBPHY_REGEN_CG_CTRL	0x104
#define	 REGEN_CG_CTRL_RPB_WEN	__BIT(24)	/* U2HC_EXT2 writable */
#define	USBPHY_U2HC_EXT2	0x160
#define	 U2HC_EXT2_DUR_CTRL	__BIT(9)
#define	USBPHY_COMMCTRL		0x600
#define	 COMMCTRL_OTG_PERI	__BIT(31)
#define	USBPHY_PHYCLK_CTRL	0x644
#define	 PHYCLK_CTRL_UCLKSEL	__BIT(0)
#define	USBPHY_SIZE		0x648

struct rza2_usbphy_softc {
	device_t sc_dev;
	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;
	kmutex_t sc_lock;
	bool sc_usb_x1;
	u_int sc_users;
};

#define	RD4(sc, r) \
	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (r))
#define	WR4(sc, r, v) \
	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (r), (v))

static int	rza2_usbphy_match(device_t, cfdata_t, void *);
static void	rza2_usbphy_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(rza2_usbphy, sizeof(struct rza2_usbphy_softc),
    rza2_usbphy_match, rza2_usbphy_attach, NULL, NULL);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "renesas,usb2-phy-r7s9210" },
	DEVICE_COMPAT_EOL
};

static void
rza2_usbphy_init(struct rza2_usbphy_softc *sc)
{

	WR4(sc, USBPHY_USBCTR, USBCTR_PLL_RST);
	delay(1);
	WR4(sc, USBPHY_PHYCLK_CTRL, sc->sc_usb_x1 ? PHYCLK_CTRL_UCLKSEL : 0);
	WR4(sc, USBPHY_COMMCTRL, 0);
	WR4(sc, USBPHY_USBCTR, 0);
	delay(100);

	WR4(sc, USBPHY_REGEN_CG_CTRL, REGEN_CG_CTRL_RPB_WEN);
	WR4(sc, USBPHY_U2HC_EXT2,
	    RD4(sc, USBPHY_U2HC_EXT2) | U2HC_EXT2_DUR_CTRL);
	WR4(sc, USBPHY_REGEN_CG_CTRL, 0);

	WR4(sc, USBPHY_INT_ENABLE,
	    INT_ENABLE_USBH_INTAEN | INT_ENABLE_USBH_INTBEN);
}

static void *
rza2_usbphy_acquire(device_t dev, const void *data, size_t len)
{

	if (len != 0)
		return NULL;
	return device_private(dev);
}

static void
rza2_usbphy_release(device_t dev, void *priv)
{
}

static int
rza2_usbphy_enable(device_t dev, void *priv, bool enable)
{
	struct rza2_usbphy_softc * const sc = priv;

	mutex_enter(&sc->sc_lock);
	if (enable) {
		if (sc->sc_users++ == 0)
			rza2_usbphy_init(sc);
	} else if (sc->sc_users > 0)
		sc->sc_users--;
	mutex_exit(&sc->sc_lock);

	return 0;
}

static const struct fdtbus_phy_controller_func rza2_usbphy_funcs = {
	.acquire = rza2_usbphy_acquire,
	.release = rza2_usbphy_release,
	.enable = rza2_usbphy_enable,
};

static int
rza2_usbphy_match(device_t parent, cfdata_t cf, void *aux)
{
	struct fdt_attach_args * const faa = aux;

	return of_compatible_match(faa->faa_phandle, compat_data);
}

static void
rza2_usbphy_attach(device_t parent, device_t self, void *aux)
{
	struct rza2_usbphy_softc * const sc = device_private(self);
	struct fdt_attach_args * const faa = aux;
	const int phandle = faa->faa_phandle;
	struct clk *clk;
	bus_addr_t addr;
	bus_size_t size;

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0 ||
	    size < USBPHY_SIZE) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	sc->sc_dev = self;
	sc->sc_bst = faa->faa_bst;
	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_NONE);

	clk = fdtbus_clock_get(phandle, "fck");
	if (clk == NULL || clk_enable(clk) != 0) {
		aprint_error(": couldn't enable the module clock\n");
		return;
	}

	clk = fdtbus_clock_get(phandle, "usb_x1");
	sc->sc_usb_x1 = clk != NULL && clk_get_rate(clk) != 0;

	aprint_naive("\n");
	aprint_normal(": RZ/A2 USB 2.0 PHY, %s clock\n",
	    sc->sc_usb_x1 ? "USB_X1" : "EXTAL");

	fdtbus_register_phy_controller(self, phandle, &rza2_usbphy_funcs);
}
