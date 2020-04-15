/*
 * (C) Copyright 2017 Linaro
 * Jorge Ramirez-Ortiz <jorge.ramirez-ortiz@linaro.org>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <console.h>
#include <linux/bug.h>
#include <linux/mii.h>

#include "eth_cfg.h"
#include "higmac.h"
#include "util.h"
#include "ctrl.h"
#include "mdio.h"

#define GSF_IOSIZE	0x1000

typedef enum {
	GSF0_PORT0,
	GSF0_PORT1,
	MAX_PORT_NUM,
} port_id_t;

static int mac_iobase[MAX_PORT_NUM] = {
	HIGMAC0_IOBASE,
	HIGMAC0_IOBASE + GSF_IOSIZE,
};

#define flush_desc(d) flush_cache((unsigned long) (d), sizeof(*(d)))
#define invalidate_desc(d)                                                     \
	invalidate_dcache_range((unsigned long)(d),                            \
				(unsigned long)(d) + sizeof(*(d)))
#define invalidate_cache(p, l) invalidate_dcache_range((p), (p) + (l))

/*
 * This will narrow the pointer and relies on u-boot running below the
 * 32-bit boundary.
 */
#define ptr_to_dma32(p) ((uint32_t)(uint64_t)(p))
#define dma32_to_ptr(da) ((void *)(uint64_t)(da))

/* suppose higmac_board_info[i] was initialed! */
#define for_each_gmac_netdev_local_priv(ld, i)  \
	for (i = 0; i < CONFIG_GMAC_NUMS &&     \
		({ld = &higmac_board_info[i].higmac_ld; 1;}); \
		i++)

struct higmac_board_info higmac_board_info[CONFIG_GMAC_NUMS] = {
	{
		{
			.index		= GSF0_PORT0,
			.iobase		= HIGMAC0_IOBASE,
			.iobase_phys	= HIGMAC0_IOBASE,
		},
		.phy_intf		= interface_mode_mii,
		.phy_addr		= CONFIG_HIGMAC_PHY1_ADDR,
	},
	{
		{
			.index		= GSF0_PORT1,
			.iobase		= HIGMAC0_IOBASE + GSF_IOSIZE,
			.iobase_phys	= HIGMAC0_IOBASE + GSF_IOSIZE,
		},
		.phy_intf		= interface_mode_mii,
		.phy_addr		= CONFIG_HIGMAC_PHY2_ADDR,
	},
};

static int phy_link_times;
static int gmac_debug;

static char *phy_intf_str[interface_mode_butt] = {
	"mii",
	"rmii",
	"rgmii",
};

static char *mdio_bus[CONFIG_GMAC_NUMS] = {
	"mdio0",
	"mdio1",
};

static int g_speed_portmode_table[speed_mode_butt][interface_mode_butt] = {
	{port_mode_10_mii,      port_mode_10_rmii,      port_mode_10_rgmii},
	{port_mode_100_mii,     port_mode_100_rmii,     port_mode_100_rgmii},
	{port_mode_butt,        port_mode_butt,         port_mode_1000_rgmii}
};

static int calculate_port_mode(enum speed_mode speed, enum if_mode if_mode,
		int is_duplex_half)
{
	int ret = port_mode_butt ;

	if (speed < speed_mode_butt && if_mode < interface_mode_butt) {
		ret = g_speed_portmode_table[speed][if_mode];
		if (is_duplex_half)
			ret &= ~BIT(4); /* see mac_if reg def. */
		return ret;
	}

	printf("Invalid speed(%d) & interface(%d) mode.\n", speed, if_mode);
	printf("Please assign which mode our eth will correctly work at.\n"
			"It may support 10M/100M MII, 10M/100M RMII, "
			"10M/100M/1000M RGMII interface.\n"
			"eg. if your board have two NICs, connecting each phy using "
			"mii and rgmii interface, you can use the module param "
			"'port_mode=mii,rgmii' to tell the driver.\n");
	BUG();

	/* can't reach here */
	return -1;
}

static void parse_module_parameters(void)
{
	eth_cfg_init();

	int tmp, gmac;

	//gmac_debug = 1;
	phy_link_times = DEFAULT_PHY_LINK_TIMES;


	BUILD_BUG_ON((int)interface_mode_mii != (int)ETH_PHY_MII);
	BUILD_BUG_ON((int)interface_mode_rmii != (int)ETH_PHY_RMII);
	BUILD_BUG_ON((int)interface_mode_rgmii != (int)ETH_PHY_RGMII);

	higmac_board_info[GSF0_PORT0].phy_intf = get_eth_phyintf(0, ETH_PHY_MII);
	higmac_board_info[GSF0_PORT1].phy_intf = get_eth_phyintf(1, ETH_PHY_RGMII);

	tmp = get_eth_phymdio(0, 0);
	higmac_board_info[GSF0_PORT0].mii_name = mdio_bus[tmp];

	tmp = get_eth_phymdio(1, 1);
	higmac_board_info[GSF0_PORT1].mii_name = mdio_bus[tmp];

	higmac_board_info[GSF0_PORT0].phy_addr = get_eth_phyaddr(0, INVALID_PHY_ADDR);
	higmac_board_info[GSF0_PORT1].phy_addr = get_eth_phyaddr(1, INVALID_PHY_ADDR);

	get_eth_phygpio(0, &higmac_board_info[GSF0_PORT0].gpio_base,
			&higmac_board_info[GSF0_PORT0].gpio_bit);
	get_eth_phygpio(1, &higmac_board_info[GSF0_PORT1].gpio_base,
			&higmac_board_info[GSF0_PORT1].gpio_bit);

	if (gmac_debug) {
		printf("phy_link_times=%d\n", phy_link_times);
		for (gmac = 0; gmac < CONFIG_GMAC_NUMS; gmac++) {
			printf("ETH%d: phyintf=%s, phymdio=%s, phyaddr=%d, "
				"gpio_base=0x%x, gpio_bit=0x%x\n",
				gmac, phy_intf_str[higmac_board_info[gmac].phy_intf],
				higmac_board_info[gmac].mii_name,
				higmac_board_info[gmac].phy_addr,
				higmac_board_info[gmac].gpio_base,
				higmac_board_info[gmac].gpio_bit);
		}
		printf("\n");
	}



}

static int higmac_net_set_mac_address(struct eth_device *dev)
{
	struct higmac_netdev_local *ld = (struct higmac_netdev_local *)dev->priv;

	higmac_hw_set_macaddress(ld, dev->enetaddr);
	//print_mac(dev->enetaddr);

	return 0;
}

static int higmac_init_hw_desc_queue(struct higmac_netdev_local *ld)
{
	higmac_desc *queue_phy_addr = NULL;
	int queue_len_in_bytes = 0;

	/* init rx fq */
	queue_len_in_bytes = HIGMAC_HWQ_RX_FQ_DEPTH * sizeof(higmac_desc);
	queue_phy_addr = memalign(ARCH_DMA_MINALIGN, queue_len_in_bytes);
	if (queue_phy_addr == NULL)
	{
		printf("alloc rx fq error!\n");
		goto _error_alloc_rx_fq;
	}

	memset(queue_phy_addr, 0, queue_len_in_bytes);
	flush_cache((unsigned long) queue_phy_addr, queue_len_in_bytes);
	ld->rx_fq_addr = queue_phy_addr;
	higmac_set_rx_fq_hwq_addr(ld, (unsigned long)queue_phy_addr);

	/* init rx bq */
	queue_len_in_bytes = HIGMAC_HWQ_RX_BQ_DEPTH * sizeof(higmac_desc);
	queue_phy_addr = memalign(ARCH_DMA_MINALIGN, queue_len_in_bytes);
	if (queue_phy_addr == NULL)
	{
		printf("alloc rx bq error!\n");
		goto _error_alloc_rx_bq;
	}

	memset(queue_phy_addr, 0, queue_len_in_bytes);
	flush_cache((unsigned long) queue_phy_addr, queue_len_in_bytes);
	ld->rx_bq_addr = queue_phy_addr;
	higmac_set_rx_bq_hwq_addr(ld, (unsigned long)queue_phy_addr);

	/* init tx bq */
	queue_len_in_bytes = HIGMAC_HWQ_TX_BQ_DEPTH * sizeof(higmac_desc);
	queue_phy_addr = (higmac_desc *)malloc(queue_len_in_bytes);
	if (queue_phy_addr == NULL)
	{
		printf("alloc tx bq error!\n");
		goto _error_alloc_tx_bq;
	}

	memset(queue_phy_addr, 0, queue_len_in_bytes);
	flush_cache((unsigned long) queue_phy_addr, queue_len_in_bytes);
	ld->tx_bq_addr = queue_phy_addr;
	higmac_set_tx_bq_hwq_addr(ld, (unsigned long)queue_phy_addr);

	/* init tx rq */
	queue_len_in_bytes = HIGMAC_HWQ_TX_RQ_DEPTH * sizeof(higmac_desc);
	queue_phy_addr = (higmac_desc *)malloc(queue_len_in_bytes);
	if (queue_phy_addr == NULL)
	{
		printf("alloc tx rq error!\n");
		goto _error_alloc_tx_rq;
	}

	memset(queue_phy_addr, 0, queue_len_in_bytes);
	flush_cache((unsigned long) queue_phy_addr, queue_len_in_bytes);
	ld->tx_rq_addr = queue_phy_addr;
	higmac_set_tx_rq_hwq_addr(ld, (unsigned long)queue_phy_addr);

	return 0;

_error_alloc_tx_rq:
	free(ld->tx_bq_addr);

_error_alloc_tx_bq:
	free(ld->rx_bq_addr);

_error_alloc_rx_bq:
	free(ld->rx_fq_addr);

_error_alloc_rx_fq:
	return -1;
}


#define PHY_ID_KSZ8051		0x00221550
#define PHY_ID_KSZ8081		0x00221560
#define PHY_ID_KSZ9031		0x00221620
#define PHY_ID_MASK		0xFFFFFFF0
#define ATH8035_PHY_ID		0x004dd072
#define ATH_PHY_ID_MASK		0xffffffef
#define RTL8211F_PHY_ID		0x001cc916
#define RTL_PHY_ID_MASK		0xffffffff
#define RTL8211F_PAGE_SEL	0x1f
#define RTL8211F_PAGE0		0xa42

/* MMD: MDIO Manageable Device */
#define MACR			0x0D
#define MAADR			0x0E
static void phy_mmd_read(char *devname, u8 phyaddr,
			u16 mmd_device, u8 regnum, u16 *val)
{
	miiphy_write(devname, phyaddr, MACR, mmd_device);
	miiphy_write(devname, phyaddr, MAADR, regnum);
	miiphy_write(devname, phyaddr, MACR, 0x4000 | mmd_device);

	miiphy_read(devname, phyaddr, MAADR, val);
}

static void phy_mmd_write(char *devname, u8 phyaddr,
			u16 mmd_device, u8 regnum, u16 val)
{
	miiphy_write(devname, phyaddr, MACR, mmd_device);
	miiphy_write(devname, phyaddr, MAADR, regnum);
	miiphy_write(devname, phyaddr, MACR, 0x4000 | mmd_device);

	miiphy_write(devname, phyaddr, MAADR, val);
}

#define EXP_ADDR	0x1d
#define EXP_DATA	0x1e
static void phy_expand_read(char *devname, u8 phyaddr, u8 regnum, u16 *val)
{
	miiphy_write(devname, phyaddr, EXP_ADDR, regnum);
	miiphy_read(devname, phyaddr, EXP_DATA, val);
}

static void phy_expand_write(char *devname, u8 phyaddr, u8 regnum, u16 val)
{
	miiphy_write(devname, phyaddr, EXP_ADDR, regnum);
	miiphy_write(devname, phyaddr, EXP_DATA, val);
}

static int phy_fixup(char *devname, unsigned char phyaddr)
{
	u32 phy_id;
	u16 id1, id2;

	if (miiphy_read(devname, phyaddr,MII_PHYSID1 , &id1) != 0) {
		printf("PHY IDR1 read failed\n");
		return -1;
	};
	if (miiphy_read(devname, phyaddr, MII_PHYSID2, &id2) != 0) {
		printf("PHY IDR2 read failed\n");
		return -1;
	};

	phy_id = (id1 & 0xffff) << 16;
	phy_id |= (id2 & 0xffff);

	/* If the phy_id is all Fs, there is no device there */
	if (0xffffffff == phy_id || 0 == phy_id
			|| phy_id == 0xFFFF || phy_id == 0xFFFF0000) {
		return -1;
	}

	/* PHY-KSZ8051 */
	if ((phy_id & PHY_ID_MASK) == PHY_ID_KSZ8051) {
		u16 val;

		if (miiphy_read(devname, phyaddr, 0x1F, &val) != 0) {
			printf("PHY reg read failed\n");
			return -1;
		};
		val |= BIT(7);       /* set phy RMII 50MHz clk; */
		if (miiphy_write(devname, phyaddr, 0x1F, val) != 0)
			return -1;

		if (miiphy_read(devname, phyaddr, 0x16, &val) != 0) {
			printf("PHY reg read failed\n");
			return -1;
		};
		val |= BIT(1);       /* set phy RMII override; */
		if (miiphy_write(devname, phyaddr, 0x16, val) != 0)
			return -1;
	}

	/* PHY-KSZ8081 */
	if ((phy_id & PHY_ID_MASK) == PHY_ID_KSZ8081) {
		u16 val;

		if (miiphy_read(devname, phyaddr, 0x1F, &val) != 0) {
			printf("PHY IDR1 read failed\n");
			return -1;
		};
		val |= BIT(7);       /* set phy RMII 50MHz clk; */
		if (miiphy_write(devname, phyaddr, 0x1F, val) != 0)
			return -1;
	}

	/* PHY-KSZ9031 */
	if ((phy_id & PHY_ID_MASK) == PHY_ID_KSZ9031) {
		u16 val;

		/* RX_CLK Pad Skew: 1_1101(+0.84) */
		phy_mmd_read(devname, phyaddr, 0x2, 0x8, &val);
		val = (val & ~0x1F) | 0x1D;
		phy_mmd_write(devname, phyaddr, 0x2, 0x8, val);
	}

	/* PHY-AR8035 */
	if ((phy_id & ATH_PHY_ID_MASK) == (ATH8035_PHY_ID & ATH_PHY_ID_MASK)) {
		u16 tx_delay;

		/* enable rgmii tx clock delay */
		phy_expand_read(devname, phyaddr, 0x05, &tx_delay);
		tx_delay |= BIT(8);
		phy_expand_write(devname, phyaddr, 0x05, tx_delay);
	}

	/* PHY-RTL8211F */
	if ((phy_id & RTL_PHY_ID_MASK) == (RTL8211F_PHY_ID & RTL_PHY_ID_MASK)) {
		u16 tx_delay;

		/* enable rgmii tx clock delay */
		miiphy_write(devname, phyaddr, RTL8211F_PAGE_SEL, 0xd08);
		miiphy_read(devname, phyaddr, 0x11, &tx_delay);
		tx_delay |= BIT(8);
		miiphy_write(devname, phyaddr, 0x11, tx_delay);
		miiphy_write(devname, phyaddr, RTL8211F_PAGE_SEL, RTL8211F_PAGE0);
	}

	return 0;
}

static int higmac_net_adjust_link(struct higmac_netdev_local *ld)
{
	char *mii_name = higmac_board_info[ld->index].mii_name;
	int phy_addr = higmac_board_info[ld->index].phy_addr;
	int stat = 0, speed = 0, is_duplex_half = 1, port_mode, phy_duplex;
	enum speed_mode speed_mode;

	if (phy_addr == INVALID_PHY_ADDR)
		return stat;

	phy_fixup(mii_name, phy_addr);

	stat |= miiphy_link(mii_name, phy_addr) ? HIGMAC_LINKED : 0;

	phy_duplex = miiphy_duplex(mii_name, phy_addr);
	if (phy_duplex == FULL) {
		stat |= HIGMAC_DUP_FULL;
		is_duplex_half = 0;
	}

	speed = miiphy_speed(mii_name, phy_addr);
	switch (speed) {
	case _10BASET:
		stat |= HIGMAC_SPD_10M;
		speed_mode = speed_mode_10M;
		break;
	default:
		printf("wired, phy speed!\n");
	case _100BASET:
		stat |= HIGMAC_SPD_100M;
		speed_mode = speed_mode_100M;
		break;
	case _1000BASET:
		stat |= HIGMAC_SPD_1000M;
		speed_mode = speed_mode_1000M;
		break;
	}

	if (ld->link_stat != stat) {
		if (stat & HIGMAC_LINKED) {
			port_mode = calculate_port_mode(speed_mode,
				higmac_board_info[ld->index].phy_intf,
				is_duplex_half);

			higmac_set_macif(ld, port_mode, speed_mode);

			if (is_duplex_half)
				higmac_writel(ld, 0x0, MAC_DUPLEX_HALF_CTRL);
			else
				higmac_writel(ld, 0x1, MAC_DUPLEX_HALF_CTRL);
		}
		ld->link_stat = stat;
	}

	return stat;
}

static int select_current_linked_phy(struct higmac_netdev_local *ld)
{
	int count = phy_link_times ? : DEFAULT_PHY_LINK_TIMES;
	int status;

	for (; count > 0; count--) {
		if (ctrlc()) {
			puts ("\nAbort\n");
			goto link_failed;
		}

		status = higmac_net_adjust_link(ld);
		if (status & HIGMAC_LINKED)
			goto link_on;
		mdelay(100);
	}

link_failed:
	printf("ETH%d: PHY(%s, phyaddr=%d, %s) not link!\n",
			ld->index, higmac_board_info[ld->index].mii_name,
			higmac_board_info[ld->index].phy_addr,
			phy_intf_str[higmac_board_info[ld->index].phy_intf]);

	return -1;

link_on:
	printf("ETH%d: PHY(phyaddr=%d, %s) link UP: DUPLEX=%s : SPEED=%s\n",
			ld->index, higmac_board_info[ld->index].phy_addr,
			phy_intf_str[higmac_board_info[ld->index].phy_intf],
			(status & HIGMAC_DUP_FULL) ? "FULL" : "HALF",
			(status & HIGMAC_SPD_100M) ? "100M" :
			((status & HIGMAC_SPD_10M) ? "10M" : "1000M"));

	miiphy_set_current_dev(higmac_board_info[ld->index].mii_name);

	return 0;
}

static int higmac_recv(struct eth_device *dev)
{
	struct higmac_netdev_local *ld = (struct higmac_netdev_local *)dev->priv;
	int timeout_us = 100000;
	int rx_fq_wr_offset = 0;
	int rx_fq_rd_offset = 0;
	int rx_bq_wr_offset = 0;
	int rx_bq_rd_offset = 0;
	int len = 0;
	int wr_rd_dist;
	int i;

	higmac_desc *rx_fq_desc= ld->rx_fq_addr;
	higmac_desc *rx_bq_desc= ld->rx_bq_addr;

	rx_fq_wr_offset = higmac_readl_bits(ld, RX_FQ_WR_ADDR, BITS_RX_FQ_WR_ADDR);
	rx_fq_rd_offset = higmac_readl_bits(ld, RX_FQ_RD_ADDR, BITS_RX_FQ_RD_ADDR);

	if (rx_fq_wr_offset >= rx_fq_rd_offset)
		wr_rd_dist = (HIGMAC_HWQ_RX_FQ_DEPTH << DESC_BYTE_SHIFT)
			- (rx_fq_wr_offset - rx_fq_rd_offset);
	else
		wr_rd_dist = rx_fq_rd_offset - rx_fq_wr_offset;

	wr_rd_dist >>= DESC_BYTE_SHIFT;/* offset was counted on bytes, desc size = 2^5 */

	/*
	 * wr_rd_dist - 1 for logic reason.
	 * Logic think the desc pool is full filled, ...?
	 */
	for (i = 0; i < wr_rd_dist - 1; i++) {
		void *buf = memalign(ARCH_DMA_MINALIGN, HIETH_MAX_FRAME_SIZE);
		if (!buf)
			break;

		rx_fq_desc = ld->rx_fq_addr + (rx_fq_wr_offset >> DESC_BYTE_SHIFT);
		rx_fq_desc->data_buff_addr = ptr_to_dma32(buf);
		invalidate_cache(rx_fq_desc->data_buff_addr, HIETH_MAX_FRAME_SIZE);

		rx_fq_desc->descvid = DESC_VLD_FREE;
		rx_fq_desc->buffer_len = (HIETH_MAX_FRAME_SIZE - 1);
		rx_fq_desc->data_len = 8;
		flush_desc(rx_fq_desc);

		rx_fq_wr_offset += DESC_SIZE;
		if (rx_fq_wr_offset >= (HIGMAC_HWQ_RX_FQ_DEPTH << DESC_BYTE_SHIFT))
			rx_fq_wr_offset = 0;

		higmac_writel_bits(ld, rx_fq_wr_offset, RX_FQ_WR_ADDR, BITS_RX_FQ_WR_ADDR);
	}

	/* enable Rx */
	//higmac_desc_enable(ld, RX_OUTCFF_WR_DESC_ENA|RX_CFF_RD_DESC_ENA);
	//higmac_writel_bits(ld, 1, PORT_EN, BITS_RX_EN);

	rx_bq_wr_offset = higmac_readl_bits(ld, RX_BQ_WR_ADDR, BITS_RX_BQ_WR_ADDR);
	rx_bq_rd_offset = higmac_readl_bits(ld, RX_BQ_RD_ADDR, BITS_RX_BQ_RD_ADDR);
	rx_bq_desc += (rx_bq_rd_offset >> DESC_BYTE_SHIFT);
	invalidate_desc(rx_bq_desc); /* the BQ is only ever written by the GMAC */

	while(--timeout_us && (rx_bq_wr_offset == rx_bq_rd_offset))
	{
		rx_bq_wr_offset = higmac_readl_bits(ld, RX_BQ_WR_ADDR, BITS_RX_BQ_WR_ADDR);
		udelay(1);
	}

	if (!timeout_us) {
		return -1;
	}

	rx_bq_rd_offset += DESC_SIZE;
	if (rx_bq_rd_offset >= (HIGMAC_HWQ_RX_BQ_DEPTH << DESC_BYTE_SHIFT))
		rx_bq_rd_offset = 0;

	len = rx_bq_desc->data_len;
	if (HIGMAC_INVALID_RXPKG_LEN(len)) {
		higmac_writel_bits(ld, rx_bq_rd_offset, RX_BQ_RD_ADDR, BITS_RX_BQ_RD_ADDR);
		if (rx_bq_desc->data_buff_addr)
			free(dma32_to_ptr(rx_bq_desc->data_buff_addr));

		if (gmac_debug)
			printf("invalid rx len %d\n", len);

		return -1;
	}
	if (gmac_debug) {
		printf("got packet!\n");
	}

	/*
	 * no cache management here... the CPU should not have touched
	 * this buffer since we added it to the FQ.
	 */
	invalidate_cache(rx_bq_desc->data_buff_addr, len);
	memcpy((void *)net_rx_packets[0],
	       dma32_to_ptr(rx_bq_desc->data_buff_addr), len);
	free(dma32_to_ptr(rx_bq_desc->data_buff_addr));

	higmac_writel_bits(ld, rx_bq_rd_offset, RX_BQ_RD_ADDR, BITS_RX_BQ_RD_ADDR);

	/*NetRecive*/
	net_process_received_packet(net_rx_packets[0], len);

	/* disable Rx */
	//higmac_writel_bits(ld, 0, PORT_EN, BITS_RX_EN);
	//higmac_desc_disable(ld, RX_OUTCFF_WR_DESC_ENA|RX_CFF_RD_DESC_ENA);

	return 0;
}

static int higmac_send(struct eth_device *dev, void *packet, int length)
{
	struct higmac_netdev_local *ld = (struct higmac_netdev_local *)dev->priv;
	int timeout_us = 1000;
	int tx_bq_wr_offset = 0;
	int tx_bq_rd_offset = 0;
	int tx_rq_wr_offset = 0;
	int tx_rq_rd_offset = 0;
	unsigned int tso_ver = 0;

	higmac_desc *tx_bq_desc= ld->tx_bq_addr;

	tx_bq_wr_offset = higmac_readl_bits(ld, TX_BQ_WR_ADDR, BITS_TX_BQ_WR_ADDR);
	tx_bq_rd_offset = higmac_readl_bits(ld, TX_BQ_RD_ADDR, BITS_TX_BQ_RD_ADDR);

	if (tx_bq_rd_offset != tx_bq_wr_offset)
	{
		higmac_writel_bits(ld, tx_bq_rd_offset, TX_BQ_WR_ADDR, BITS_TX_BQ_WR_ADDR);
		printf("tx bq is error!\n");
		return -1;
	}

	flush_cache((unsigned long) packet, length);

	tso_ver = higmac_readl_bits(ld, CRF_MIN_PACKET, BIT_TSO_VERSION);

	tx_bq_desc += (tx_bq_wr_offset >> DESC_BYTE_SHIFT);
	tx_bq_desc->data_buff_addr = ptr_to_dma32(packet);
	tx_bq_desc->descvid = DESC_VLD_BUSY;

	if (tso_ver) {
		tx_bq_desc->fl = 0;
	} else
		tx_bq_desc->fl = DESC_FL_FULL;
	tx_bq_desc->reserve1 = 0;

	tx_bq_desc->data_len = length;
	/* TODO: buffer_len is oversize (packet does not point to such a large buffer */
	tx_bq_desc->buffer_len =  (HIETH_MAX_FRAME_SIZE - 1);;
	flush_desc(tx_bq_desc);

	tx_bq_wr_offset += DESC_SIZE;
	if (tx_bq_wr_offset >= (HIGMAC_HWQ_TX_BQ_DEPTH << DESC_BYTE_SHIFT))
		tx_bq_wr_offset = 0;
	higmac_writel_bits(ld, tx_bq_wr_offset, TX_BQ_WR_ADDR, BITS_TX_BQ_WR_ADDR);

	/* enable tx */
	//higmac_writel_bits(ld, 1, PORT_EN, BITS_TX_EN);
	//higmac_desc_enable(ld, TX_OUTCFF_WR_DESC_ENA|TX_CFF_RD_DESC_ENA);

	tx_rq_wr_offset = higmac_readl_bits(ld, TX_RQ_WR_ADDR, BITS_TX_RQ_WR_ADDR);
	tx_rq_rd_offset = higmac_readl_bits(ld, TX_RQ_RD_ADDR, BITS_TX_RQ_RD_ADDR);
	tx_rq_rd_offset += DESC_SIZE;
	if (tx_rq_rd_offset >= (HIGMAC_HWQ_TX_RQ_DEPTH << DESC_BYTE_SHIFT))
		tx_rq_rd_offset = 0;

	while(--timeout_us && (tx_rq_rd_offset != tx_rq_wr_offset))
	{
		tx_rq_wr_offset = higmac_readl_bits(ld, TX_RQ_WR_ADDR, BITS_TX_RQ_WR_ADDR);
		udelay(1);
	}

	if (!timeout_us)
	{
		printf("send time out!\n");
		return -1;
	}

	/* TODO: We are ignoring an xmit errors (maybe wrong return code) */

	higmac_writel_bits(ld, tx_rq_rd_offset, TX_RQ_RD_ADDR, BITS_TX_RQ_RD_ADDR);
	if (gmac_debug)
		printf("send packet!\n");
	/* disable tx */

	//higmac_desc_disable(ld, TX_OUTCFF_WR_DESC_ENA|TX_CFF_RD_DESC_ENA);
	//higmac_writel_bits(ld, 0, PORT_EN, BITS_TX_EN);

	return 0;
}

static void higmac_phy_gpio_reset(void)
{
	void *gpio_base;
	u32 gpio_bit, v;
	int i;
	const int reset_data = 1;

	for (i = 0; i < CONFIG_GMAC_NUMS; i++) {
		gpio_base = (void *) (uint64_t) higmac_board_info[i].gpio_base;
		gpio_bit = higmac_board_info[i].gpio_bit;

		if (!gpio_base)
			continue;

		/* config gpio[x] dir to output */
		v = readb(gpio_base + 0x400);
		v |= (1 << gpio_bit);
		writeb(v, gpio_base + 0x400);

		/* output 1--0--1 */
		writeb(reset_data << gpio_bit,
				gpio_base + (4 << gpio_bit));
		udelay(20000);
		writeb((!reset_data) << gpio_bit,
				gpio_base + (4 << gpio_bit));
		udelay(20000);
		writeb(reset_data << gpio_bit,
				gpio_base + (4 << gpio_bit));
	}
}

static void higmac_miiphy_reset_all_port(void)
{
	char *mii_devname = NULL;
	int phy_addr = 0;
	enum if_mode phy_intf = interface_mode_butt;
	int i;

	for (i = 0; i < CONFIG_GMAC_NUMS; i++) {
		mii_devname = higmac_board_info[i].mii_name;
		phy_addr = higmac_board_info[i].phy_addr;
		phy_intf = higmac_board_info[i].phy_intf;

		miiphy_reset(mii_devname, phy_addr);
		if (phy_intf != interface_mode_rgmii) {
			unsigned short val = 0;
			if (!miiphy_read(mii_devname, phy_addr, MII_CTRL1000, &val)) {
				val &= ~(PHY_1000BTCR_1000FD | PHY_1000BTCR_1000HD);
				miiphy_write(mii_devname, phy_addr, MII_CTRL1000, val);
			}
		}
	}
}

static int gmac_hw_inited;
static void higmac_hw_init(void)
{
	/* init once to save time */
	if (!gmac_hw_inited) {
		higmac_phy_gpio_reset();
		higmac_sys_init();
		higmac_miiphy_reset_all_port();

		gmac_hw_inited = 1;
	}
}
static int higmac_init(struct eth_device *dev, bd_t *bd)
{
	struct higmac_netdev_local *ld = (struct higmac_netdev_local *)dev->priv;
	int ret = 0;

	/* init once to save time */
	if (!ld->initalized) {


		higmac_glb_preinit_dummy(ld);

		ret = higmac_set_hwq_depth(ld);
		if (ret) {
			printf("init eth%d hw desc queue depth fail!\n", ld->index);
			return ret;
		}

		ret = higmac_init_hw_desc_queue(ld);
		if (ret) {
			printf("init eth%d hw desc queue fail!\n", ld->index);
			return ret;
		}

		ld->initalized = 1;
	}

	ret = select_current_linked_phy(ld);
	if (ret)
		return ret;

	higmac_desc_enable(ld, RX_OUTCFF_WR_DESC_ENA|RX_CFF_RD_DESC_ENA);
	higmac_writel_bits(ld, 1, PORT_EN, BITS_RX_EN);

	higmac_writel_bits(ld, 1, PORT_EN, BITS_TX_EN);
	higmac_desc_enable(ld, TX_OUTCFF_WR_DESC_ENA|TX_CFF_RD_DESC_ENA);

	return 0;
}

static void higmac_halt(struct eth_device *dev)
{
	struct higmac_netdev_local *ld = (struct higmac_netdev_local *)dev->priv;

	higmac_writel_bits(ld, 0, PORT_EN, BITS_RX_EN);
	higmac_desc_disable(ld, RX_OUTCFF_WR_DESC_ENA|RX_CFF_RD_DESC_ENA);

	higmac_desc_disable(ld, TX_OUTCFF_WR_DESC_ENA|TX_CFF_RD_DESC_ENA);
	higmac_writel_bits(ld, 0, PORT_EN, BITS_TX_EN);

}

static int higmac_register_dev(port_id_t port_id)
{
	struct eth_device *dev;

	dev = malloc(sizeof(*dev));
	if (dev == NULL)
		return -1;
	memset(dev, 0, sizeof(*dev));

	dev->iobase = mac_iobase[port_id];
	dev->init = higmac_init;
	dev->halt = higmac_halt;
	dev->send = higmac_send;
	dev->recv = higmac_recv;
	dev->write_hwaddr = higmac_net_set_mac_address;
	dev->priv = &higmac_board_info[port_id].higmac_ld;
	snprintf(dev->name, sizeof(dev->name) - 1, "gmac%d", port_id);

	/* we must init the hardware before u-boot supplies our MAC address */
	higmac_hw_init();

	eth_register(dev);

#if defined(CONFIG_MII) || defined(CONFIG_CMD_MII)

	int retval;
	struct mii_dev *mdiodev = mdio_alloc();
	if (!mdiodev)
		return -ENOMEM;

	strncpy(mdiodev->name, mdio_bus[port_id], MDIO_NAME_LEN);
	mdiodev->read = higmac_mdiobus_read;
	mdiodev->write = higmac_mdiobus_write;

	retval = mdio_register(mdiodev);
	if (retval < 0)
		return retval;
#endif
	return 0;
}

int higmac_initialize(bd_t *bis)
{
	int ret;

	parse_module_parameters();

#if 0
	ret = higmac_register_dev(GSF0_PORT0);
	if (ret)
		return ret;
#endif

	ret = higmac_register_dev(GSF0_PORT1);
	if (ret)
		return ret;

	return 0;
}
