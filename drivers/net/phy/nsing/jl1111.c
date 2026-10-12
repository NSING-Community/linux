// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the JLSemi JL1111 Fast Ethernet PHY.
 *
 * The JL1111 is a 10/100 Mbps PHY that also drives the RMII REF_CLK in the
 * usual "PHY is the clock source" board wiring found on the N32H7xx HMI
 * boards, where the MAC's ETH_REFCLK pin is fed from the PHY TXC output.
 *
 * A page-7 register (RMSR) controls the RMII mode and the direction of the
 * 50 MHz RMII reference clock.  Its reset default assumes the PHY receives
 * the clock from the MAC, which leaves a MAC that expects to consume
 * REF_CLK with no clock at all: MDIO still works but RX/TX stay dead.  We
 * flip the direction bit here so Linux does not depend on a bootloader or a
 * debugger poking the register by hand after every reset.
 */

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/phy.h>

#define JL1111_PHY_ID			0x937c4025
#define JL1111_PHY_ID_MASK		0xffffffff

#define JL1111_PAGE_SELECT		0x1f
#define JL1111_PAGE_RMII		0x07

/* RMII status/setup register, page 7 register 16. */
#define JL1111_RMSR			0x10

/*
 * RMII clock direction, clear to have the PHY output the 50 MHz REF_CLK.
 * The reset default of 1 makes the PHY expect the MAC to source it.
 */
#define JL1111_RMSR_RMII_CLK_DIR	BIT(12)

/* MII/RMII mode select, set for RMII. */
#define JL1111_RMSR_RMII_MODE		BIT(3)

/*
 * Register 0x1f selects the register page.  These two callbacks are what
 * lets the generic phy_modify_paged()/phy_read_paged() helpers reach the
 * vendor pages; without them those helpers bail out with -EOPNOTSUPP.
 */
static int jl1111_read_page(struct phy_device *phydev)
{
	return __phy_read(phydev, JL1111_PAGE_SELECT);
}

static int jl1111_write_page(struct phy_device *phydev, int page)
{
	return __phy_write(phydev, JL1111_PAGE_SELECT, page);
}

static int jl1111_config_init(struct phy_device *phydev)
{
	if (phydev->interface != PHY_INTERFACE_MODE_RMII)
		return 0;

	/* Select RMII and drive REF_CLK from the PHY, preserving the skews. */
	return phy_modify_paged(phydev, JL1111_PAGE_RMII, JL1111_RMSR,
				JL1111_RMSR_RMII_CLK_DIR |
				JL1111_RMSR_RMII_MODE,
				JL1111_RMSR_RMII_MODE);
}

static struct phy_driver jl1111_driver[] = {
	{
		PHY_ID_MATCH_EXACT(JL1111_PHY_ID),
		.name		= "JLSemi JL1111 Fast Ethernet",
		.soft_reset	= genphy_soft_reset,
		.read_page	= jl1111_read_page,
		.write_page	= jl1111_write_page,
		.config_init	= jl1111_config_init,
		.config_aneg	= genphy_config_aneg,
		.read_status	= genphy_read_status,
		.suspend	= genphy_suspend,
		.resume		= genphy_resume,
	},
};

module_phy_driver(jl1111_driver);

MODULE_DESCRIPTION("JLSemi JL1111 Fast Ethernet PHY driver");
MODULE_LICENSE("GPL");

static const struct mdio_device_id __maybe_unused jl1111_tbl[] = {
	{ JL1111_PHY_ID, JL1111_PHY_ID_MASK },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(mdio, jl1111_tbl);
