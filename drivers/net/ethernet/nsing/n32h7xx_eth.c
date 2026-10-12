// SPDX-License-Identifier: GPL-2.0-only
/*
 * NSING N32H7xx 10/100 Ethernet MAC driver.
 *
 * This is a self-contained driver for the NSING N32H7xx on-chip Ethernet
 * MAC.  It programs the MAC, the MTL and the channel-0 DMA engine directly
 * and reaches the on-board PHY through the MAC's own MDIO controller.  The
 * SoC-specific clock gates, reset, interface select and pin mux live in
 * arch/arm/mach-nsing/n32h7xx-eth.c so that this file only owns the network
 * device itself.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irqreturn.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_reserved_mem.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/soc/nsing/n32h7xx-eth.h>
#include <linux/types.h>

/* Register offsets inside the 0x2000 window of the MAC. */
#define N32H7XX_ETH_MACCFG		0x0000
#define N32H7XX_ETH_MACEXTCFG		0x0004
#define N32H7XX_ETH_MACPFLT		0x0008
#define N32H7XX_ETH_MACTXFLWCTRL	0x0070
#define N32H7XX_ETH_MACRXFLWCTRL	0x0090
#define N32H7XX_ETH_MAC1USTICCNT	0x00dc
#define N32H7XX_ETH_MACMDIOADDR		0x0200
#define N32H7XX_ETH_MACMDIODATA		0x0204
#define N32H7XX_ETH_MACADDR0H		0x0300
#define N32H7XX_ETH_MACADDR0L		0x0304
#define N32H7XX_ETH_MTLTXQOPMOD		0x0d00
#define N32H7XX_ETH_MTLRXQOPMOD		0x0d30
#define N32H7XX_ETH_DMAMODE		0x1000
#define N32H7XX_ETH_DMASBMODE		0x1004
#define N32H7XX_ETH_DMACH0CTRL		0x1100
#define N32H7XX_ETH_DMACH0TXCTRL	0x1104
#define N32H7XX_ETH_DMACH0RXCTRL	0x1108
#define N32H7XX_ETH_DMACH0TXDLA		0x1114
#define N32H7XX_ETH_DMACH0RXDLA		0x111c
#define N32H7XX_ETH_DMACH0TXDTP		0x1120
#define N32H7XX_ETH_DMACH0RXDTP		0x1128
#define N32H7XX_ETH_DMACH0TXDRLEN	0x112c
#define N32H7XX_ETH_DMACH0RXDRLEN	0x1130
#define N32H7XX_ETH_DMACH0INTEN		0x1134
#define N32H7XX_ETH_DMACH0STS		0x1160

/* MAC configuration. */
#define MACCFG_RE			BIT(0)
#define MACCFG_TE			BIT(1)
#define MACCFG_DM			BIT(13)
#define MACCFG_FES			BIT(14)
#define MACCFG_PS			BIT(15)
#define MACCFG_ACS			BIT(20)
#define MACCFG_CST			BIT(21)

/* MAC packet filter. */
#define MACPFLT_PM			BIT(0)
#define MACPFLT_PAM			BIT(4)
#define MACPFLT_RA			BIT(31)

/* MAC MDIO address register. */
#define MDIO_GB				BIT(0)
#define MDIO_GOC			GENMASK(3, 2)
#define MDIO_GOC_WRITE			BIT(2)
#define MDIO_GOC_READ			(BIT(3) | BIT(2))
#define MDIO_CR				GENMASK(11, 8)
#define MDIO_RDA			GENMASK(20, 16)
#define MDIO_PA				GENMASK(25, 21)
/*
 * CSR clock range for a 300..500 MHz AHB2 clock, which divides MDC down to
 * roughly 1.5 MHz.
 */
#define MDIO_TIMEOUT_US			1000000

/* MTL queue operation mode. */
#define MTLTXQOM_TSF			BIT(1)
#define MTLRXQOM_RSF			BIT(5)

/* DMA mode. */
#define DMAMODE_SWR			BIT(0)
#define DMAMODE_SWR_TIMEOUT_US		1000000

/* DMA system bus mode. */
#define DMASBMODE_FB			BIT(0)
#define DMASBMODE_AAL			BIT(12)

/* DMA channel 0 transmit control. */
#define DMACH0TXCTRL_ST			BIT(0)
#define DMACH0TXCTRL_OSF		BIT(4)
#define DMACH0TXCTRL_PBL		GENMASK(21, 16)
#define DMACH0TX_PBL			32

/* DMA channel 0 receive control. */
#define DMACH0RXCTRL_SR			BIT(0)
#define DMACH0RXCTRL_RBSZ		GENMASK(14, 1)
#define DMACH0RXCTRL_PBL		GENMASK(21, 16)
#define DMACH0RX_PBL			32

/* DMA channel 0 interrupt enable and status. */
#define DMA_INT_TIE			BIT(0)
#define DMA_INT_TXSE			BIT(1)
#define DMA_INT_TBUE			BIT(2)
#define DMA_INT_RIE			BIT(6)
#define DMA_INT_RBUE			BIT(7)
#define DMA_INT_RSE			BIT(8)
#define DMA_INT_FBEE			BIT(12)
#define DMA_INT_AIE			BIT(14)
#define DMA_INT_NIE			BIT(15)

#define DMA_INT_ENABLE		(DMA_INT_TIE | DMA_INT_TBUE | DMA_INT_RIE | \
				 DMA_INT_RBUE | DMA_INT_RSE | DMA_INT_FBEE | \
				 DMA_INT_AIE | DMA_INT_NIE)
#define DMA_INT_RX		(DMA_INT_RIE)
#define DMA_INT_TX		(DMA_INT_TIE)
#define DMA_INT_RX_OR_TX	(DMA_INT_RX | DMA_INT_TX)

/*
 * Transmit descriptor, read format.  Each descriptor is exactly four words
 * long because the channel is left with a descriptor skip length of zero, so
 * the hardware walks the ring with a 16 byte stride.
 */
#define TDES2_B1L			GENMASK(13, 0)
#define TDES2_IOC			BIT(31)
#define TDES3_FL			GENMASK(14, 0)
#define TDES3_LD			BIT(28)
#define TDES3_FD			BIT(29)
#define TDES3_OWN			BIT(31)

/* Transmit descriptor, write-back format. */
#define TDES3_WB_ES			BIT(15)

/* Receive descriptor, read format. */
#define RDES3_BUF1V			BIT(24)
#define RDES3_IOC			BIT(30)
#define RDES3_OWN			BIT(31)

/* Receive descriptor, write-back format. */
#define RDES3_WB_PL			GENMASK(14, 0)
#define RDES3_WB_ES			BIT(15)
#define RDES3_WB_CE			BIT(24)
#define RDES3_WB_LD			BIT(28)
#define RDES3_WB_FD			BIT(29)
#define RDES3_WB_OWN			BIT(31)

#define N32H7XX_TX_DESC_NUM		64
#define N32H7XX_RX_DESC_NUM		64
#define N32H7XX_RX_BUF_SIZE		1536

struct n32h7xx_eth_desc {
	u32 des0;
	u32 des1;
	u32 des2;
	u32 des3;
};

struct n32h7xx_eth {
	struct device *dev;
	struct net_device *ndev;
	struct clk *clk;
	void __iomem *base;
	int irq;
	unsigned long csr_clk_hz;

	struct napi_struct napi;
	spinlock_t tx_lock;

	struct mii_bus *mdio_bus;
	struct phy_device *phydev;

	struct n32h7xx_eth_desc *tx_ring;
	dma_addr_t tx_ring_dma;
	struct sk_buff *tx_skb[N32H7XX_TX_DESC_NUM];
	dma_addr_t tx_dma[N32H7XX_TX_DESC_NUM];
	unsigned int tx_len[N32H7XX_TX_DESC_NUM];
	unsigned int tx_head;
	unsigned int tx_tail;

	struct n32h7xx_eth_desc *rx_ring;
	dma_addr_t rx_ring_dma;
	void *rx_buf[N32H7XX_RX_DESC_NUM];
	dma_addr_t rx_buf_dma[N32H7XX_RX_DESC_NUM];
	unsigned int rx_cur;
};

static void n32h7xx_set_rx_mode(struct net_device *ndev);

static dma_addr_t n32h7xx_rx_tail_addr(struct n32h7xx_eth *priv,
				       unsigned int idx)
{
	return priv->rx_ring_dma + idx * sizeof(struct n32h7xx_eth_desc);
}

static void n32h7xx_write_mac_addr(struct n32h7xx_eth *priv, const u8 *addr)
{
	writel((addr[5] << 8) | addr[4], priv->base + N32H7XX_ETH_MACADDR0H);
	writel((addr[3] << 24) | (addr[2] << 16) | (addr[1] << 8) | addr[0],
	       priv->base + N32H7XX_ETH_MACADDR0L);
}

static void n32h7xx_set_speed_duplex(struct n32h7xx_eth *priv, int speed,
				     int duplex)
{
	u32 val = readl(priv->base + N32H7XX_ETH_MACCFG);

	val &= ~(MACCFG_DM | MACCFG_FES | MACCFG_PS);
	val |= MACCFG_PS;
	if (speed == SPEED_100)
		val |= MACCFG_FES;
	if (duplex == DUPLEX_FULL)
		val |= MACCFG_DM;

	writel(val, priv->base + N32H7XX_ETH_MACCFG);
}

/*
 * The MDIO clock range field is a coarse divider table rather than a plain
 * divisor.  Pick the entry the MAC documents for the CSR (AHB2) clock so
 * that MDC lands inside the 1.0..2.5 MHz window the specification asks for.
 */
static u32 n32h7xx_mdio_cr(unsigned long csr_clk_hz)
{
	unsigned int mhz = DIV_ROUND_UP(csr_clk_hz, 1000000);

	if (mhz < 35)
		return 2;	/* /16  */
	if (mhz < 60)
		return 3;	/* /26  */
	if (mhz < 100)
		return 0;	/* /42  */
	if (mhz < 150)
		return 1;	/* /62  */
	if (mhz <= 250)
		return 4;	/* /102 */
	if (mhz < 300)
		return 5;	/* /124 */
	if (mhz < 500)
		return 6;	/* /204 */

	return 7;		/* /324 */
}

static int n32h7xx_reset_dma(struct n32h7xx_eth *priv)
{
	u32 val;
	int ret;

	writel(DMAMODE_SWR, priv->base + N32H7XX_ETH_DMAMODE);
	ret = readl_poll_timeout(priv->base + N32H7XX_ETH_DMAMODE, val,
				 !(val & DMAMODE_SWR), 10,
				 DMAMODE_SWR_TIMEOUT_US);
	if (ret)
		dev_err(priv->dev, "DMA software reset did not complete\n");

	return ret;
}

static int n32h7xx_mdio_wait(struct n32h7xx_eth *priv)
{
	u32 val;

	return readl_poll_timeout_atomic(priv->base + N32H7XX_ETH_MACMDIOADDR,
					 val, !(val & MDIO_GB), 10,
					 MDIO_TIMEOUT_US);
}

static int n32h7xx_mdio_read(struct mii_bus *bus, int phy_addr, int reg)
{
	struct n32h7xx_eth *priv = bus->priv;
	u32 addr;
	int ret;

	addr = readl(priv->base + N32H7XX_ETH_MACMDIOADDR);
	addr &= ~(MDIO_PA | MDIO_RDA | MDIO_GOC | MDIO_GB);
	addr |= FIELD_PREP(MDIO_PA, phy_addr) | FIELD_PREP(MDIO_RDA, reg) |
		MDIO_GOC_READ | MDIO_GB;
	writel(addr, priv->base + N32H7XX_ETH_MACMDIOADDR);

	ret = n32h7xx_mdio_wait(priv);
	if (ret)
		return ret;

	return readl(priv->base + N32H7XX_ETH_MACMDIODATA) & GENMASK(15, 0);
}

static int n32h7xx_mdio_write(struct mii_bus *bus, int phy_addr, int reg,
			      u16 value)
{
	struct n32h7xx_eth *priv = bus->priv;
	u32 addr;
	int ret;

	writel(value, priv->base + N32H7XX_ETH_MACMDIODATA);

	addr = readl(priv->base + N32H7XX_ETH_MACMDIOADDR);
	addr &= ~(MDIO_PA | MDIO_RDA | MDIO_GOC | MDIO_GB);
	addr |= FIELD_PREP(MDIO_PA, phy_addr) | FIELD_PREP(MDIO_RDA, reg) |
		MDIO_GOC_WRITE | MDIO_GB;
	writel(addr, priv->base + N32H7XX_ETH_MACMDIOADDR);

	ret = n32h7xx_mdio_wait(priv);
	return ret;
}

static void n32h7xx_tx_reclaim(struct n32h7xx_eth *priv)
{
	struct net_device *ndev = priv->ndev;
	unsigned int tail;
	bool freed = false;

	/* TX state is shared with NAPI and the watchdog, never the hard IRQ. */
	spin_lock_bh(&priv->tx_lock);

	tail = priv->tx_tail;
	while (tail != priv->tx_head) {
		struct n32h7xx_eth_desc *desc = &priv->tx_ring[tail];
		u32 des3 = READ_ONCE(desc->des3);

		if (des3 & TDES3_OWN)
			break;

		if (priv->tx_skb[tail]) {
			dma_unmap_single(priv->dev, priv->tx_dma[tail],
					 priv->tx_len[tail], DMA_TO_DEVICE);
			ndev->stats.tx_bytes += priv->tx_len[tail];
			ndev->stats.tx_packets++;
			if (unlikely(des3 & TDES3_WB_ES))
				ndev->stats.tx_errors++;
			dev_kfree_skb_any(priv->tx_skb[tail]);
			priv->tx_skb[tail] = NULL;
		}

		tail = (tail + 1) % N32H7XX_TX_DESC_NUM;
		freed = true;
	}

	priv->tx_tail = tail;

	spin_unlock_bh(&priv->tx_lock);

	if (freed && netif_queue_stopped(ndev) && netif_running(ndev))
		netif_wake_queue(ndev);
}

static void n32h7xx_rx_rearm(struct n32h7xx_eth *priv, unsigned int idx)
{
	struct n32h7xx_eth_desc *desc = &priv->rx_ring[idx];

	desc->des0 = (u32)priv->rx_buf_dma[idx];
	desc->des1 = 0;
	desc->des2 = 0;
	dma_wmb();
	WRITE_ONCE(desc->des3, RDES3_BUF1V | RDES3_IOC | RDES3_OWN);
}

static int n32h7xx_poll(struct napi_struct *napi, int budget)
{
	struct n32h7xx_eth *priv = container_of(napi, struct n32h7xx_eth, napi);
	struct net_device *ndev = priv->ndev;
	unsigned int done = 0;

	while (done < budget) {
		unsigned int idx = priv->rx_cur;
		struct n32h7xx_eth_desc *desc = &priv->rx_ring[idx];
		u32 des3 = READ_ONCE(desc->des3);
		unsigned int len;

		if (des3 & RDES3_WB_OWN)
			break;

		dma_rmb();
		len = FIELD_GET(RDES3_WB_PL, des3);

		if (unlikely(des3 & RDES3_WB_ES)) {
			ndev->stats.rx_errors++;
			if (des3 & RDES3_WB_CE)
				ndev->stats.rx_crc_errors++;
		} else if (unlikely(len > ndev->mtu + ETH_HLEN + VLAN_HLEN) ||
			   unlikely(len < ETH_ZLEN - ETH_FCS_LEN)) {
			ndev->stats.rx_length_errors++;
		} else {
			struct sk_buff *skb = napi_alloc_skb(napi, len);

			if (likely(skb)) {
				memcpy(skb->data, priv->rx_buf[idx], len);
				skb_put(skb, len);
				skb->protocol = eth_type_trans(skb, ndev);
				ndev->stats.rx_packets++;
				ndev->stats.rx_bytes += len;
				napi_gro_receive(napi, skb);
			} else {
				ndev->stats.rx_dropped++;
			}
		}

		n32h7xx_rx_rearm(priv, idx);
		priv->rx_cur = (idx + 1) % N32H7XX_RX_DESC_NUM;
		done++;
	}

	n32h7xx_tx_reclaim(priv);

	/*
	 * Every descriptor is owned by the DMA again, so point the tail at the
	 * last one of the ring.  The write doubles as the resume poke when the
	 * receive engine suspended itself because it caught us mid-refill, and
	 * the RBUE/RSE latches have to be cleared for it to try again.
	 */
	writel((u32)n32h7xx_rx_tail_addr(priv, N32H7XX_RX_DESC_NUM - 1),
	       priv->base + N32H7XX_ETH_DMACH0RXDTP);
	writel(DMA_INT_RBUE | DMA_INT_RSE, priv->base + N32H7XX_ETH_DMACH0STS);

	if (done < budget) {
		napi_complete_done(napi, done);
		writel(readl(priv->base + N32H7XX_ETH_DMACH0INTEN) |
		       DMA_INT_RX_OR_TX,
		       priv->base + N32H7XX_ETH_DMACH0INTEN);
	}

	return done;
}

static irqreturn_t n32h7xx_irq(int irq, void *data)
{
	struct net_device *ndev = data;
	struct n32h7xx_eth *priv = netdev_priv(ndev);
	u32 status;

	status = readl(priv->base + N32H7XX_ETH_DMACH0STS);
	if (!status)
		return IRQ_NONE;

	/* Status and summary latches are cleared by writing one. */
	writel(status, priv->base + N32H7XX_ETH_DMACH0STS);

	/* TBU is normal when the DMA reaches a CPU-owned, empty TX slot.
	 * Reclaim completed packets below; start_xmit advances the tail to
	 * resume DMA when another packet is submitted.
	 */
	if (status & (DMA_INT_TXSE | DMA_INT_FBEE)) {
		dev_warn_ratelimited(priv->dev,
				     "DMA error, status %#010x\n", status);
	}

	if (status & (DMA_INT_RIE | DMA_INT_RBUE | DMA_INT_RSE |
		      DMA_INT_TIE | DMA_INT_TBUE)) {
		writel(readl(priv->base + N32H7XX_ETH_DMACH0INTEN) &
		       ~DMA_INT_RX_OR_TX, priv->base + N32H7XX_ETH_DMACH0INTEN);
		if (napi_schedule_prep(&priv->napi))
			__napi_schedule(&priv->napi);
	}

	return IRQ_HANDLED;
}

static void n32h7xx_phy_adjust_link(struct net_device *ndev)
{
	struct n32h7xx_eth *priv = netdev_priv(ndev);
	struct phy_device *phydev = priv->phydev;

	if (!phydev->link) {
		if (netif_carrier_ok(ndev)) {
			netif_carrier_off(ndev);
			phy_print_status(phydev);
		}
		return;
	}

	n32h7xx_set_speed_duplex(priv, phydev->speed, phydev->duplex);
	if (!netif_carrier_ok(ndev)) {
		netif_carrier_on(ndev);
		phy_print_status(phydev);
	}
}

static void n32h7xx_init_tx_ring(struct n32h7xx_eth *priv)
{
	unsigned int i;

	for (i = 0; i < N32H7XX_TX_DESC_NUM; i++) {
		struct n32h7xx_eth_desc *desc = &priv->tx_ring[i];

		WRITE_ONCE(desc->des0, 0);
		WRITE_ONCE(desc->des1, 0);
		WRITE_ONCE(desc->des2, 0);
		WRITE_ONCE(desc->des3, 0);
		priv->tx_skb[i] = NULL;
	}

	priv->tx_head = 0;
	priv->tx_tail = 0;
}

static void n32h7xx_init_rx_ring(struct n32h7xx_eth *priv)
{
	unsigned int i;

	for (i = 0; i < N32H7XX_RX_DESC_NUM; i++)
		n32h7xx_rx_rearm(priv, i);

	priv->rx_cur = 0;
}

static void n32h7xx_hw_init(struct n32h7xx_eth *priv)
{
	struct net_device *ndev = priv->ndev;
	u32 val;

	/* Start from a clean slate. */
	writel(0, priv->base + N32H7XX_ETH_MACCFG);
	writel(0, priv->base + N32H7XX_ETH_MACEXTCFG);
	writel(0, priv->base + N32H7XX_ETH_MACPFLT);
	writel(0, priv->base + N32H7XX_ETH_MACTXFLWCTRL);
	writel(0, priv->base + N32H7XX_ETH_MACRXFLWCTRL);

	n32h7xx_reset_dma(priv);

	/* MDIO clock range; the rest of the field is set per transaction. */
	val = readl(priv->base + N32H7XX_ETH_MACMDIOADDR);
	val &= ~MDIO_CR;
	writel(val | FIELD_PREP(MDIO_CR, n32h7xx_mdio_cr(priv->csr_clk_hz)),
	       priv->base + N32H7XX_ETH_MACMDIOADDR);

	/*
	 * Strip the pad/FCS of received frames so that the DMA packet length
	 * is the length of the Ethernet frame the stack expects.  The port
	 * select bit keeps the MAC in its 10/100 Mbps mode.
	 */
	val = readl(priv->base + N32H7XX_ETH_MACCFG);
	writel(val | MACCFG_PS | MACCFG_ACS | MACCFG_CST,
	       priv->base + N32H7XX_ETH_MACCFG);

	n32h7xx_write_mac_addr(priv, ndev->dev_addr);
	n32h7xx_set_rx_mode(ndev);

	/* Store and forward both queues. */
	writel(MTLTXQOM_TSF, priv->base + N32H7XX_ETH_MTLTXQOPMOD);
	writel(MTLRXQOM_RSF, priv->base + N32H7XX_ETH_MTLRXQOPMOD);

	/* 1 us tick used by the MAC counters. */
	writel(priv->csr_clk_hz / 1000000 - 1,
	       priv->base + N32H7XX_ETH_MAC1USTICCNT);

	/* Fixed burst, address-aligned beats, 16 byte descriptor stride. */
	writel(DMASBMODE_FB | DMASBMODE_AAL,
	       priv->base + N32H7XX_ETH_DMASBMODE);
	writel(0, priv->base + N32H7XX_ETH_DMACH0CTRL);

	writel(FIELD_PREP(DMACH0TXCTRL_PBL, DMACH0TX_PBL) | DMACH0TXCTRL_OSF,
	       priv->base + N32H7XX_ETH_DMACH0TXCTRL);
	writel(FIELD_PREP(DMACH0RXCTRL_PBL, DMACH0RX_PBL) |
	       FIELD_PREP(DMACH0RXCTRL_RBSZ, N32H7XX_RX_BUF_SIZE),
	       priv->base + N32H7XX_ETH_DMACH0RXCTRL);

	writel(N32H7XX_TX_DESC_NUM - 1,
	       priv->base + N32H7XX_ETH_DMACH0TXDRLEN);
	writel(N32H7XX_RX_DESC_NUM - 1,
	       priv->base + N32H7XX_ETH_DMACH0RXDRLEN);

	writel((u32)priv->tx_ring_dma, priv->base + N32H7XX_ETH_DMACH0TXDLA);
	writel((u32)priv->rx_ring_dma, priv->base + N32H7XX_ETH_DMACH0RXDLA);

	n32h7xx_init_tx_ring(priv);
	n32h7xx_init_rx_ring(priv);

	writel((u32)priv->tx_ring_dma, priv->base + N32H7XX_ETH_DMACH0TXDTP);
	writel((u32)(priv->rx_ring_dma +
		     (N32H7XX_RX_DESC_NUM - 1) * sizeof(struct n32h7xx_eth_desc)),
	       priv->base + N32H7XX_ETH_DMACH0RXDTP);

	/* Mask everything until the ring and the PHY are ready. */
	writel(0, priv->base + N32H7XX_ETH_DMACH0INTEN);
	readl(priv->base + N32H7XX_ETH_DMACH0INTEN);
}

static void n32h7xx_start(struct n32h7xx_eth *priv)
{
	u32 val;

	writel(readl(priv->base + N32H7XX_ETH_DMACH0RXCTRL) |
	       DMACH0RXCTRL_SR, priv->base + N32H7XX_ETH_DMACH0RXCTRL);
	writel(readl(priv->base + N32H7XX_ETH_DMACH0TXCTRL) |
	       DMACH0TXCTRL_ST, priv->base + N32H7XX_ETH_DMACH0TXCTRL);

	val = readl(priv->base + N32H7XX_ETH_MACCFG);
	writel(val | MACCFG_RE | MACCFG_TE, priv->base + N32H7XX_ETH_MACCFG);

	writel(DMA_INT_ENABLE, priv->base + N32H7XX_ETH_DMACH0INTEN);
}

static void n32h7xx_hw_stop(struct n32h7xx_eth *priv)
{
	writel(0, priv->base + N32H7XX_ETH_DMACH0INTEN);
	writel(GENMASK(15, 0), priv->base + N32H7XX_ETH_DMACH0STS);

	writel(readl(priv->base + N32H7XX_ETH_MACCFG) &
	       ~(MACCFG_RE | MACCFG_TE), priv->base + N32H7XX_ETH_MACCFG);

	writel(readl(priv->base + N32H7XX_ETH_DMACH0RXCTRL) &
	       ~DMACH0RXCTRL_SR, priv->base + N32H7XX_ETH_DMACH0RXCTRL);
	writel(readl(priv->base + N32H7XX_ETH_DMACH0TXCTRL) &
	       ~DMACH0TXCTRL_ST, priv->base + N32H7XX_ETH_DMACH0TXCTRL);

	usleep_range(10, 100);
	n32h7xx_reset_dma(priv);
}

static void n32h7xx_free_tx_skbs(struct n32h7xx_eth *priv)
{
	unsigned int i;

	for (i = 0; i < N32H7XX_TX_DESC_NUM; i++) {
		if (!priv->tx_skb[i])
			continue;
		dma_unmap_single(priv->dev, priv->tx_dma[i], priv->tx_len[i],
				 DMA_TO_DEVICE);
		dev_kfree_skb_any(priv->tx_skb[i]);
		priv->tx_skb[i] = NULL;
	}
}

static void n32h7xx_free_rings(struct n32h7xx_eth *priv)
{
	n32h7xx_free_tx_skbs(priv);

	if (priv->rx_buf[0]) {
		dma_free_coherent(priv->dev,
				  N32H7XX_RX_BUF_SIZE * N32H7XX_RX_DESC_NUM,
				  priv->rx_buf[0], priv->rx_buf_dma[0]);
	}
	memset(priv->rx_buf, 0, sizeof(priv->rx_buf));

	if (priv->rx_ring) {
		dma_free_coherent(priv->dev,
				  N32H7XX_RX_DESC_NUM *
				  sizeof(struct n32h7xx_eth_desc),
				  priv->rx_ring, priv->rx_ring_dma);
		priv->rx_ring = NULL;
	}

	if (priv->tx_ring) {
		dma_free_coherent(priv->dev,
				  N32H7XX_TX_DESC_NUM *
				  sizeof(struct n32h7xx_eth_desc),
				  priv->tx_ring, priv->tx_ring_dma);
		priv->tx_ring = NULL;
	}
}

static int n32h7xx_alloc_rings(struct n32h7xx_eth *priv)
{
	dma_addr_t dma;
	unsigned int i;
	u8 *bufs;

	priv->tx_ring = dma_alloc_coherent(priv->dev,
				N32H7XX_TX_DESC_NUM *
				sizeof(struct n32h7xx_eth_desc),
				&priv->tx_ring_dma, GFP_KERNEL);
	if (!priv->tx_ring)
		return -ENOMEM;

	priv->rx_ring = dma_alloc_coherent(priv->dev,
				N32H7XX_RX_DESC_NUM *
				sizeof(struct n32h7xx_eth_desc),
				&priv->rx_ring_dma, GFP_KERNEL);
	if (!priv->rx_ring)
		goto free_tx;

	bufs = dma_alloc_coherent(priv->dev,
				  N32H7XX_RX_BUF_SIZE * N32H7XX_RX_DESC_NUM,
				  &dma, GFP_KERNEL);
	if (!bufs)
		goto free_rx;

	for (i = 0; i < N32H7XX_RX_DESC_NUM; i++) {
		priv->rx_buf[i] = bufs + i * N32H7XX_RX_BUF_SIZE;
		priv->rx_buf_dma[i] = dma + i * N32H7XX_RX_BUF_SIZE;
	}

	return 0;

free_rx:
	dma_free_coherent(priv->dev,
			  N32H7XX_RX_DESC_NUM * sizeof(struct n32h7xx_eth_desc),
			  priv->rx_ring, priv->rx_ring_dma);
	priv->rx_ring = NULL;
free_tx:
	dma_free_coherent(priv->dev,
			  N32H7XX_TX_DESC_NUM * sizeof(struct n32h7xx_eth_desc),
			  priv->tx_ring, priv->tx_ring_dma);
	priv->tx_ring = NULL;
	return -ENOMEM;
}

static int n32h7xx_open(struct net_device *ndev)
{
	struct n32h7xx_eth *priv = netdev_priv(ndev);
	int ret;

	ret = n32h7xx_alloc_rings(priv);
	if (ret)
		return ret;

	n32h7xx_hw_init(priv);

	ret = request_irq(priv->irq, n32h7xx_irq, 0, ndev->name, ndev);
	if (ret)
		goto free_rings;

	napi_enable(&priv->napi);
	n32h7xx_start(priv);

	phy_start(priv->phydev);
	netif_carrier_off(ndev);
	netif_start_queue(ndev);

	return 0;

free_rings:
	n32h7xx_free_rings(priv);
	return ret;
}

static int n32h7xx_stop(struct net_device *ndev)
{
	struct n32h7xx_eth *priv = netdev_priv(ndev);

	netif_stop_queue(ndev);
	phy_stop(priv->phydev);
	napi_disable(&priv->napi);

	n32h7xx_hw_stop(priv);
	free_irq(priv->irq, ndev);
	n32h7xx_free_rings(priv);

	return 0;
}

static netdev_tx_t n32h7xx_start_xmit(struct sk_buff *skb,
				      struct net_device *ndev)
{
	struct n32h7xx_eth *priv = netdev_priv(ndev);
	unsigned int idx = priv->tx_head;
	struct n32h7xx_eth_desc *desc = &priv->tx_ring[idx];
	dma_addr_t dma;
	unsigned int len = skb->len;

	spin_lock_bh(&priv->tx_lock);

	if (unlikely(READ_ONCE(desc->des3) & TDES3_OWN)) {
		netif_stop_queue(ndev);
		spin_unlock_bh(&priv->tx_lock);
		return NETDEV_TX_BUSY;
	}

	dma = dma_map_single(priv->dev, skb->data, len, DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(priv->dev, dma))) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		spin_unlock_bh(&priv->tx_lock);
		return NETDEV_TX_OK;
	}

	priv->tx_skb[idx] = skb;
	priv->tx_dma[idx] = dma;
	priv->tx_len[idx] = len;

	WRITE_ONCE(desc->des0, (u32)dma);
	WRITE_ONCE(desc->des1, 0);
	WRITE_ONCE(desc->des2, FIELD_PREP(TDES2_B1L, len) | TDES2_IOC);
	dma_wmb();
	WRITE_ONCE(desc->des3, FIELD_PREP(TDES3_FL, len) |
				   TDES3_FD | TDES3_LD | TDES3_OWN);

	priv->tx_head = (idx + 1) % N32H7XX_TX_DESC_NUM;

	/* Wake the transmit engine up to and including the new descriptor. */
	writel((u32)(priv->tx_ring_dma +
		     priv->tx_head * sizeof(struct n32h7xx_eth_desc)),
	       priv->base + N32H7XX_ETH_DMACH0TXDTP);

	if (READ_ONCE(priv->tx_ring[priv->tx_head].des3) & TDES3_OWN)
		netif_stop_queue(ndev);

	spin_unlock_bh(&priv->tx_lock);

	return NETDEV_TX_OK;
}

static void n32h7xx_tx_timeout(struct net_device *ndev, unsigned int txqueue)
{
	struct n32h7xx_eth *priv = netdev_priv(ndev);

	netdev_warn(ndev, "transmit timed out, DMA status %#010x\n",
		    readl(priv->base + N32H7XX_ETH_DMACH0STS));

	n32h7xx_tx_reclaim(priv);

	/* Any write to the tail pointer also resumes a suspended TX engine. */
	spin_lock_bh(&priv->tx_lock);
	writel((u32)(priv->tx_ring_dma +
		     priv->tx_head * sizeof(struct n32h7xx_eth_desc)),
	       priv->base + N32H7XX_ETH_DMACH0TXDTP);
	spin_unlock_bh(&priv->tx_lock);
}

static void n32h7xx_set_rx_mode(struct net_device *ndev)
{
	struct n32h7xx_eth *priv = netdev_priv(ndev);
	u32 val;

	val = readl(priv->base + N32H7XX_ETH_MACPFLT);
	val &= ~(MACPFLT_PM | MACPFLT_PAM | MACPFLT_RA);

	/*
	 * The multicast hash table is left empty, so the only way to not drop
	 * IPv6 neighbour discovery and friends on the floor is to let every
	 * multicast frame through and filter in software.  Promiscuous mode
	 * additionally opens the unicast filter.
	 */
	val |= MACPFLT_PAM;
	if (ndev->flags & IFF_PROMISC)
		val |= MACPFLT_PM | MACPFLT_RA;

	writel(val, priv->base + N32H7XX_ETH_MACPFLT);
}

static int n32h7xx_set_mac_address(struct net_device *ndev, void *addr)
{
	struct n32h7xx_eth *priv = netdev_priv(ndev);
	struct sockaddr *sa = addr;
	int ret;

	ret = eth_prepare_mac_addr_change(ndev, addr);
	if (ret)
		return ret;

	eth_hw_addr_set(ndev, sa->sa_data);
	n32h7xx_write_mac_addr(priv, ndev->dev_addr);
	eth_commit_mac_addr_change(ndev, addr);

	return 0;
}

static const struct net_device_ops n32h7xx_netdev_ops = {
	.ndo_open		= n32h7xx_open,
	.ndo_stop		= n32h7xx_stop,
	.ndo_start_xmit		= n32h7xx_start_xmit,
	.ndo_tx_timeout		= n32h7xx_tx_timeout,
	.ndo_set_rx_mode	= n32h7xx_set_rx_mode,
	.ndo_set_mac_address	= n32h7xx_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
};

static void n32h7xx_get_drvinfo(struct net_device *ndev,
				struct ethtool_drvinfo *info)
{
	strscpy(info->driver, "n32h7xx_eth", sizeof(info->driver));
}

static const struct ethtool_ops n32h7xx_ethtool_ops = {
	.get_drvinfo		= n32h7xx_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
	.set_link_ksettings	= phy_ethtool_set_link_ksettings,
	.nway_reset		= phy_ethtool_nway_reset,
};

static int n32h7xx_mdio_setup(struct n32h7xx_eth *priv)
{
	struct device *dev = priv->dev;
	struct device_node *mdio_np;
	struct mii_bus *bus;
	int ret;

	mdio_np = of_get_child_by_name(dev->of_node, "mdio");
	if (!mdio_np)
		return dev_err_probe(dev, -ENODEV, "missing mdio node\n");

	bus = devm_mdiobus_alloc(dev);
	if (!bus) {
		ret = -ENOMEM;
		goto err_put;
	}

	bus->name = "n32h7xx-mdio";
	bus->read = n32h7xx_mdio_read;
	bus->write = n32h7xx_mdio_write;
	bus->parent = dev;
	bus->priv = priv;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s", dev_name(dev));

	/* The bus is unregistered for us at driver detach. */
	ret = devm_of_mdiobus_register(dev, bus, mdio_np);
	if (ret)
		goto err_put;

	priv->mdio_bus = bus;
	of_node_put(mdio_np);
	return 0;

err_put:
	of_node_put(mdio_np);
	return dev_err_probe(dev, ret, "failed to register MDIO bus\n");
}

static void n32h7xx_release_dma_pool(void *data)
{
	of_reserved_mem_device_release(data);
}

static int n32h7xx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *ndev;
	struct n32h7xx_eth *priv;
	int ret;

	ndev = alloc_etherdev(sizeof(*priv));
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, dev);
	platform_set_drvdata(pdev, ndev);

	priv = netdev_priv(ndev);
	priv->dev = dev;
	priv->ndev = ndev;
	spin_lock_init(&priv->tx_lock);

	priv->clk = devm_clk_get(dev, "csr");
	if (IS_ERR(priv->clk)) {
		ret = PTR_ERR(priv->clk);
		goto free_netdev;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		goto free_netdev;

	priv->csr_clk_hz = clk_get_rate(priv->clk);
	if (!priv->csr_clk_hz) {
		ret = -EINVAL;
		dev_err_probe(dev, ret, "CSR clock has no rate\n");
		goto disable_clk;
	}

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base)) {
		ret = PTR_ERR(priv->base);
		goto disable_clk;
	}

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0) {
		ret = priv->irq;
		goto disable_clk;
	}

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		goto disable_clk;

	if (of_find_property(dev->of_node, "memory-region", NULL)) {
		ret = of_reserved_mem_device_init(dev);
		if (ret)
			goto disable_clk;
		ret = devm_add_action_or_reset(dev, n32h7xx_release_dma_pool, dev);
		if (ret)
			goto disable_clk;
	}

	ret = n32h7xx_eth_soc_init(dev);
	if (ret)
		goto disable_clk;

	/* The MDIO block needs the CSR clock range before the PHY is probed. */
	n32h7xx_reset_dma(priv);
	writel(FIELD_PREP(MDIO_CR, n32h7xx_mdio_cr(priv->csr_clk_hz)),
	       priv->base + N32H7XX_ETH_MACMDIOADDR);

	ret = n32h7xx_mdio_setup(priv);
	if (ret)
		goto disable_clk;

	/*
	 * Prefer a MAC address supplied by the firmware/bootloader through the
	 * device tree.  The chip itself has none, so fall back to a stable
	 * address derived from the SoC unique ID rather than a random one that
	 * changes every boot (and every DHCP lease).
	 */
	if (of_get_ethdev_address(dev->of_node, ndev)) {
		u8 addr[ETH_ALEN];

		if (n32h7xx_eth_soc_mac_addr(addr)) {
			eth_hw_addr_random(ndev);
			dev_warn(dev, "no DT MAC address and no chip ID, using random %pM\n",
				 ndev->dev_addr);
		} else {
			eth_hw_addr_set(ndev, addr);
			dev_info(dev, "no DT MAC address, using SoC-derived %pM\n",
				 ndev->dev_addr);
		}
	}

	ndev->netdev_ops = &n32h7xx_netdev_ops;
	ndev->ethtool_ops = &n32h7xx_ethtool_ops;
	ndev->watchdog_timeo = 2 * HZ;
	ndev->max_mtu = N32H7XX_RX_BUF_SIZE - ETH_HLEN - VLAN_HLEN;

	/*
	 * One linear skb per descriptor: no scatter-gather, segmentation or
	 * checksum offload is implemented, so nothing of that may be offered
	 * to the stack.
	 */
	ndev->features &= ~(NETIF_F_SG | NETIF_F_GSO_SOFTWARE |
			    NETIF_F_CSUM_MASK | NETIF_F_RXCSUM);
	ndev->hw_features = ndev->features;
	ndev->vlan_features = ndev->features;

	netif_napi_add(ndev, &priv->napi, n32h7xx_poll);
	/* NVIC IRQs share a priority, so polling at IRQ exit prevents the
	 * USART from draining its small RX FIFO. Run NAPI in task context.
	 */
	netif_threaded_enable(ndev);

	priv->phydev = of_phy_get_and_connect(ndev, dev->of_node,
					      n32h7xx_phy_adjust_link);
	if (!priv->phydev) {
		ret = -ENODEV;
		dev_err_probe(dev, ret, "failed to connect PHY\n");
		goto del_napi;
	}

	ret = register_netdev(ndev);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register netdev\n");
		goto disconnect_phy;
	}

	netif_carrier_off(ndev);

	dev_info(dev, "%s: NSING N32H7xx Ethernet, irq %d\n", ndev->name,
		 priv->irq);

	return 0;

disconnect_phy:
	phy_disconnect(priv->phydev);
del_napi:
	netif_napi_del(&priv->napi);
disable_clk:
	clk_disable_unprepare(priv->clk);
free_netdev:
	free_netdev(ndev);
	return ret;
}

static void n32h7xx_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct n32h7xx_eth *priv = netdev_priv(ndev);

	unregister_netdev(ndev);
	phy_disconnect(priv->phydev);
	netif_napi_del(&priv->napi);
	clk_disable_unprepare(priv->clk);
	free_netdev(ndev);
}

static const struct of_device_id n32h7xx_eth_match[] = {
	{ .compatible = "nsing,n32h7xx-eth" },
	{ }
};
MODULE_DEVICE_TABLE(of, n32h7xx_eth_match);

static struct platform_driver n32h7xx_eth_driver = {
	.probe	= n32h7xx_probe,
	.remove	= n32h7xx_remove,
	.driver	= {
		.name		= "n32h7xx-eth",
		.of_match_table	= n32h7xx_eth_match,
	},
};
module_platform_driver(n32h7xx_eth_driver);

MODULE_AUTHOR("NSING");
MODULE_DESCRIPTION("NSING N32H7xx Ethernet MAC driver");
MODULE_LICENSE("GPL");
