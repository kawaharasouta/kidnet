#include<linux/module.h>
#include<linux/types.h>
#include<linux/kernel.h>
#include<linux/pci.h>
#include<linux/netdevice.h>
#include<linux/ethtool.h>
#include<linux/dma-mapping.h>
#include<linux/timer.h>
#include<linux/workqueue.h>
#include<linux/rtnetlink.h>

#include"include/kidnet.h"

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Kawaharasouta <kawahara6514@gmail.com>");
MODULE_DESCRIPTION("network module");

static char *kidnet_msg = "module [kidnet]:";
static char kidnet_driver_name[] = "kidnet";

static struct pci_device_id kidnet_pci_tbl[] = {
	INTEL_KIDNET_ETHERNET_DEVICE(0x1528), /* X540-t1, X540-t2 */
	/* required last entry */
	{0,}
};

enum mac {
	mac_82557_D100_A  = 0,
	mac_82557_D100_B  = 1,
	mac_82557_D100_C  = 2,
	mac_82558_D101_A4 = 4,
	mac_82558_D101_B0 = 5,
	mac_82559_D101M   = 8,
	mac_82559_D101S   = 9,
	mac_82550_D102    = 12,
	mac_82550_D102_C  = 13,
	mac_82551_E       = 14,
	mac_82551_F       = 15,
	mac_82551_10      = 16,
	mac_unknown       = 0xFF,
};



enum eeprom_id {
	eeprom_id_wol = 0x0020,
};


//*******************************//
struct nic {
	/* Begin: frequently used values: keep adjacent for cache effect */
	u32 msg_enable				____cacheline_aligned;
	struct net_device *netdev;
	struct pci_dev *pdev;
	u16 (*mdio_ctrl)(struct nic *nic, u32 addr, u32 dir, u32 reg, u16 data);

	struct rx *rxs				____cacheline_aligned;
	struct rx *rx_to_use;
	struct rx *rx_to_clean;
	struct rfd blank_rfd;
	enum ru_state ru_running;

	spinlock_t cb_lock			____cacheline_aligned;
	spinlock_t cmd_lock;
	struct csr __iomem *csr;
	enum scb_cmd_lo cuc_cmd;
	unsigned int cbs_avail;
	//struct napi_struct napi;
	struct cb *cbs;
	struct cb *cb_to_use;
	struct cb *cb_to_send;
	struct cb *cb_to_clean;
	__le16 tx_command;
	/* End: frequently used values: keep adjacent for cache effect */

	enum {
		ich                = (1 << 0),
		promiscuous        = (1 << 1),
		multicast_all      = (1 << 2),
		wol_magic          = (1 << 3),
		ich_10h_workaround = (1 << 4),
	} flags					____cacheline_aligned;

	enum mac mac;
	enum phy phy;
	struct params params;
	struct timer_list watchdog;
	struct mii_if_info mii;
	struct work_struct tx_timeout_task;
	enum loopback loopback;

	struct mem *mem;
	dma_addr_t dma_addr;

	struct pci_pool *cbs_pool;
	dma_addr_t cbs_dma_addr;
	u8 adaptive_ifs;
	u8 tx_threshold;
	u32 tx_frames;
	u32 tx_collisions;
	u32 tx_deferred;
	u32 tx_single_collisions;
	u32 tx_multiple_collisions;
	u32 tx_fc_pause;
	u32 tx_tco_frames;

	u32 rx_fc_pause;
	u32 rx_fc_unsupported;
	u32 rx_tco_frames;
	u32 rx_short_frame_errors;
	u32 rx_over_length_errors;

	u16 eeprom_wc;
	__le16 eeprom[256];
	spinlock_t mdio_lock;
	const struct firmware *fw;
};

static inline void 
kidnet_write_flush(struct nic *nic) {
	/* Flush previous PCI writes through intermediate bridges
	 * by doing a benign read */
	(void)ioread8(&nic->csr->scb.status);
}

static void 
kidnet_enable_irq(struct nic *nic) {
	unsigned long flags;

	spin_lock_irqsave(&nic->cmd_lock, flags);
	iowrite8(irq_mask_none, &nic->csr->scb.cmd_hi);
	kidnet_write_flush(nic);
	spin_unlock_irqrestore(&nic->cmd_lock, flags);
}

static void 
kidnet_disable_irq(struct nic *nic) {
	unsigned long flags;

	spin_lock_irqsave(&nic->cmd_lock, flags);
	iowrite8(irq_mask_all, &nic->csr->scb.cmd_hi);
	kidnet_write_flush(nic);
	spin_unlock_irqrestore(&nic->cmd_lock, flags);
}

static void 
kidnet_hw_reset(struct nic *nic) {
	/* Put CU and RU into idle with a selective reset to get
	 * device off of PCI bus */
	iowrite32(selective_reset, &nic->csr->port);
	kidnet_write_flush(nic); udelay(20);

	/* Now fully reset device */
	iowrite32(software_reset, &nic->csr->port);
	kidnet_write_flush(nic); udelay(20);

	/* Mask off our interrupt line - it's unmasked after reset */
	kidnet_disable_irq(nic);
}

static int 
kidnet_alloc(struct nic *nic) {
	nic->mem = pci_alloc_consistent(nic->pdev, sizeof(struct mem), &nic->dma_addr);
	return nic->mem ? 0 : -ENOMEM;
}
static void 
kidnet_free(struct nic *nic) {
	if (nic->mem) {
		pci_free_consistent(nic->pdev, sizeof(struct mem),
			nic->mem, nic->dma_addr);
		nic->mem = NULL;
	}
}
//**********************************************//

static int 
mdio_read(struct net_device *netdev, int addr, int reg) {
	struct nic *nic = netdev_priv(netdev);
	return nic->mdio_ctrl(nic, addr, mdi_read, reg, 0);
}
static void 
mdio_write(struct net_device *netdev, int addr, int reg, int data) {
	struct nic *nic = netdev_priv(netdev);

	nic->mdio_ctrl(nic, addr, mdi_write, reg, data);
}

/* the standard mdio_ctrl() function for usual MII-compliant hardware */
static u16 
mdio_ctrl_hw(struct nic *nic, u32 addr, u32 dir, u32 reg, u16 data) {
	u32 data_out = 0;
	unsigned int i;
	unsigned long flags;

	/*
	 * Stratus87247: we shouldn't be writing the MDI control
	 * register until the Ready bit shows True.  Also, since
	 * manipulation of the MDI control registers is a multi-step
	 * procedure it should be done under lock.
	 */
	spin_lock_irqsave(&nic->mdio_lock, flags);
	for (i = 100; i; --i) {
		if (ioread32(&nic->csr->mdi_ctrl) & mdi_ready)
			break;
		udelay(20);
	}
	if (unlikely(!i)) {
		netdev_err(nic->netdev, "e100.mdio_ctrl won't go Ready\n");
		spin_unlock_irqrestore(&nic->mdio_lock, flags);
		return 0;		/* No way to indicate timeout error */
	}
	iowrite32((reg << 16) | (addr << 21) | dir | data, &nic->csr->mdi_ctrl);

	for (i = 0; i < 100; i++) {
		udelay(20);
		if ((data_out = ioread32(&nic->csr->mdi_ctrl)) & mdi_ready)
			break;
	}
	spin_unlock_irqrestore(&nic->mdio_lock, flags);
	netif_printk(nic, hw, KERN_DEBUG, nic->netdev,
		     "%s:addr=%d, reg=%d, data_in=0x%04X, data_out=0x%04X\n",
		     dir == mdi_read ? "READ" : "WRITE",
		     addr, reg, data, data_out);
	return (u16)data_out;
}

static int kidnet_phy_init(struct nic *nic)
{
	struct net_device *netdev = nic->netdev;
	u32 addr;
	u16 bmcr, stat, id_lo, id_hi, cong;

	/* Discover phy addr by searching addrs in order {1,0,2,..., 31} */
	for (addr = 0; addr < 32; addr++) {
		nic->mii.phy_id = (addr == 0) ? 1 : (addr == 1) ? 0 : addr;
		bmcr = mdio_read(netdev, nic->mii.phy_id, MII_BMCR);
		stat = mdio_read(netdev, nic->mii.phy_id, MII_BMSR);
		stat = mdio_read(netdev, nic->mii.phy_id, MII_BMSR);
		if (!((bmcr == 0xFFFF) || ((stat == 0) && (bmcr == 0))))
			break;
	}
	if (addr == 32) {
		/* uhoh, no PHY detected: check whether we seem to be some
		 * weird, rare variant which is *known* to not have any MII.
		 * But do this AFTER MII checking only, since this does
		 * lookup of EEPROM values which may easily be unreliable. */
		if (e100_phy_check_without_mii(nic))
			return 0; /* simply return and hope for the best */
		else {
			/* for unknown cases log a fatal error */
			netif_err(nic, hw, nic->netdev,
				  "Failed to locate any known PHY, aborting\n");
			return -EAGAIN;
		}
	} else
		netif_printk(nic, hw, KERN_DEBUG, nic->netdev,
			     "phy_addr = %d\n", nic->mii.phy_id);

	/* Get phy ID */
	id_lo = mdio_read(netdev, nic->mii.phy_id, MII_PHYSID1);
	id_hi = mdio_read(netdev, nic->mii.phy_id, MII_PHYSID2);
	nic->phy = (u32)id_hi << 16 | (u32)id_lo;
	netif_printk(nic, hw, KERN_DEBUG, nic->netdev,
		     "phy ID = 0x%08X\n", nic->phy);

	/* Select the phy and isolate the rest */
	for (addr = 0; addr < 32; addr++) {
		if (addr != nic->mii.phy_id) {
			mdio_write(netdev, addr, MII_BMCR, BMCR_ISOLATE);
		} else if (nic->phy != phy_82552_v) {
			bmcr = mdio_read(netdev, addr, MII_BMCR);
			mdio_write(netdev, addr, MII_BMCR,
				bmcr & ~BMCR_ISOLATE);
		}
	}
	/*
	 * Workaround for 82552:
	 * Clear the ISOLATE bit on selected phy_id last (mirrored on all
	 * other phy_id's) using bmcr value from addr discovery loop above.
	 */
	if (nic->phy == phy_82552_v)
		mdio_write(netdev, nic->mii.phy_id, MII_BMCR,
			bmcr & ~BMCR_ISOLATE);

	/* Handle National tx phys */
#define NCS_PHY_MODEL_MASK	0xFFF0FFFF
	if ((nic->phy & NCS_PHY_MODEL_MASK) == phy_nsc_tx) {
		/* Disable congestion control */
		cong = mdio_read(netdev, nic->mii.phy_id, MII_NSC_CONG);
		cong |= NSC_CONG_TXREADY;
		cong &= ~NSC_CONG_ENABLE;
		mdio_write(netdev, nic->mii.phy_id, MII_NSC_CONG, cong);
	}

	if (nic->phy == phy_82552_v) {
		u16 advert = mdio_read(netdev, nic->mii.phy_id, MII_ADVERTISE);

		/* assign special tweaked mdio_ctrl() function */
		nic->mdio_ctrl = mdio_ctrl_phy_82552_v;

		/* Workaround Si not advertising flow-control during autoneg */
		advert |= ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM;
		mdio_write(netdev, nic->mii.phy_id, MII_ADVERTISE, advert);

		/* Reset for the above changes to take effect */
		bmcr = mdio_read(netdev, nic->mii.phy_id, MII_BMCR);
		bmcr |= BMCR_RESET;
		mdio_write(netdev, nic->mii.phy_id, MII_BMCR, bmcr);
	} else if ((nic->mac >= mac_82550_D102) || ((nic->flags & ich) &&
	   (mdio_read(netdev, nic->mii.phy_id, MII_TPISTATUS) & 0x8000) &&
		(nic->eeprom[eeprom_cnfg_mdix] & eeprom_mdix_enabled))) {
		/* enable/disable MDI/MDI-X auto-switching. */
		mdio_write(netdev, nic->mii.phy_id, MII_NCONFIG,
				nic->mii.force_media ? 0 : NCONFIG_AUTO_SWITCH);
	}

	return 0;
}




static void 
kidnet_eeprom_write(struct nic *nic, u16 addr_len, u16 addr, __le16 data) {
	u32 cmd_addr_data[3];
	u8 ctrl;
	int i, j;

	/* Three cmds: write/erase enable, write data, write/erase disable */
	cmd_addr_data[0] = op_ewen << (addr_len - 2);
	cmd_addr_data[1] = (((op_write << addr_len) | addr) << 16) |
		le16_to_cpu(data);
	cmd_addr_data[2] = op_ewds << (addr_len - 2);

	/* Bit-bang cmds to write word to eeprom */
	for (j = 0; j < 3; j++) {

		/* Chip select */
		iowrite8(eecs | eesk, &nic->csr->eeprom_ctrl_lo);
		kidnet_write_flush(nic); udelay(4);

		for (i = 31; i >= 0; i--) {
			ctrl = (cmd_addr_data[j] & (1 << i)) ?
				eecs | eedi : eecs;
			iowrite8(ctrl, &nic->csr->eeprom_ctrl_lo);
			kidnet_write_flush(nic); udelay(4);

			iowrite8(ctrl | eesk, &nic->csr->eeprom_ctrl_lo);
			kidnet_write_flush(nic); udelay(4);
		}
		/* Wait 10 msec for cmd to complete */
		msleep(10);

		/* Chip deselect */
		iowrite8(0, &nic->csr->eeprom_ctrl_lo);
		kidnet_write_flush(nic); udelay(4);
	}
};

/* General technique stolen from the eepro100 driver - very clever */
static __le16 
kidnet_eeprom_read(struct nic *nic, u16 *addr_len, u16 addr) {
	u32 cmd_addr_data;
	u16 data = 0;
	u8 ctrl;
	int i;

	cmd_addr_data = ((op_read << *addr_len) | addr) << 16;

	/* Chip select */
	iowrite8(eecs | eesk, &nic->csr->eeprom_ctrl_lo);
	kidnet_write_flush(nic); udelay(4);

	/* Bit-bang to read word from eeprom */
	for (i = 31; i >= 0; i--) {
		ctrl = (cmd_addr_data & (1 << i)) ? eecs | eedi : eecs;
		iowrite8(ctrl, &nic->csr->eeprom_ctrl_lo);
		kidnet_write_flush(nic); udelay(4);

		iowrite8(ctrl | eesk, &nic->csr->eeprom_ctrl_lo);
		kidnet_write_flush(nic); udelay(4);

		/* Eeprom drives a dummy zero to EEDO after receiving
		 * complete address.  Use this to adjust addr_len. */
		ctrl = ioread8(&nic->csr->eeprom_ctrl_lo);
		if (!(ctrl & eedo) && i > 16) {
			*addr_len -= (i - 16);
			i = 17;
		}

		data = (data << 1) | (ctrl & eedo ? 1 : 0);
	}

	/* Chip deselect */
	iowrite8(0, &nic->csr->eeprom_ctrl_lo);
	kidnet_write_flush(nic); udelay(4);

	return cpu_to_le16(data);
};

/* Load entire EEPROM image into driver cache and validate checksum */
static int 
kidnet_eeprom_load(struct nic *nic) {
	u16 addr, addr_len = 8, checksum = 0;

	/* Try reading with an 8-bit addr len to discover actual addr len */
	kidnet_eeprom_read(nic, &addr_len, 0);
	nic->eeprom_wc = 1 << addr_len;

	for (addr = 0; addr < nic->eeprom_wc; addr++) {
		nic->eeprom[addr] = kidnet_eeprom_read(nic, &addr_len, addr);
		if (addr < nic->eeprom_wc - 1)
			checksum += le16_to_cpu(nic->eeprom[addr]);
	}

	/* The checksum, stored in the last word, is calculated such that
	 * the sum of words should be 0xBABA */
	if (cpu_to_le16(0xBABA - checksum) != nic->eeprom[nic->eeprom_wc - 1]) {
		netif_err(nic, probe, nic->netdev, "EEPROM corrupted\n");
		if (!eeprom_bad_csum_allow)
			return -EAGAIN;
	}

	return 0;
}

static void 
kidnet_get_defaults(struct nic *nic) {
	struct param_range rfds = { .min = 16, .max = 256, .count = 256 };
	struct param_range cbs  = { .min = 64, .max = 256, .count = 128 };

	/* MAC type is encoded as rev ID; exception: ICH is treated as 82559 */
	nic->mac = (nic->flags & ich) ? mac_82559_D101M : nic->pdev->revision;
	if (nic->mac == mac_unknown)
		nic->mac = mac_82557_D100_A;

	nic->params.rfds = rfds;
	nic->params.cbs = cbs;

	/* Quadwords to DMA into FIFO before starting frame transmit */
	nic->tx_threshold = 0xE0;

	/* no interrupt for every tx completion, delay = 256us if not 557 */
	nic->tx_command = cpu_to_le16(cb_tx | cb_tx_sf |
		((nic->mac >= mac_82558_D101_A4) ? cb_cid : cb_i));

	/* Template for a freshly allocated RFD */
	nic->blank_rfd.command = 0;
	nic->blank_rfd.rbd = cpu_to_le32(0xFFFFFFFF);
	nic->blank_rfd.size = cpu_to_le16(VLAN_ETH_FRAME_LEN + ETH_FCS_LEN);

	/* MII setup */
	nic->mii.phy_id_mask = 0x1F;
	nic->mii.reg_num_mask = 0x1F;
	nic->mii.dev = nic->netdev;
	nic->mii.mdio_read = mdio_read;
	nic->mii.mdio_write = mdio_write;
}

static void 
kidnet_tx_timeout_task(struct work_struct *work) {
	struct nic *nic = container_of(work, struct nic, tx_timeout_task);
	struct net_device *netdev = nic->netdev;

	netif_printk(nic, tx_err, KERN_DEBUG, nic->netdev,
		     "scb.status=0x%02X\n", ioread8(&nic->csr->scb.status));

	rtnl_lock();
	if (netif_running(netdev)) {
		e100_down(netdev_priv(netdev));
		e100_up(netdev_priv(netdev));
	}
	rtnl_unlock();
}



static int 
kidnet_init_one(struct pci_dev *pdev, const struct pci_device_id *ent) {
	struct net_device *netdev;
	struct nic *nic;
	int err;

	netdev = alloc_etherdev(sizeof(struct ***));
	if (!netdev)
		return -ENOMEM;

	netdev->hw_features |= NETIF_F_RXFCS;
	netdev->priv_flags |= IFF_SUPP_NOFCS;
	netdev->hw_features |= NETIF_F_RXALL;

	netdev->netdev_ops = &kidnet_ops;
	netdev->ethtool_ops = &kidnet_ethtool_ops;
	netdev->watchdog_timeo = KIDNET_WATCHDOG_PERIOD;
	strncpy(netdev->name, pci_name(), sizeof(netdev->name) -1);
	
	nic = netdev_priv(netdev);
	//netif_napi_add(***);
	nic->netdev = netdev;
	nic->pdev = pdev;
	nic->msg_enable = (1 << debug) -1;
	nic->mdio_ctrl = mdio_ctrl_hw; 
	pci_set_drvdata(pdev, netdev);

	if ((err = pci_enable_device(pdev))) {
		netif_err(nic, probe, nic->netdev, "%s cannot enable pci device, abording...\n", kidnet_msg);
		goto err_out_free_dev;
	}
	if (!(pci_resource_flags(pdev, 0) & IORESOURCE_MEM)) {
		netif_err(nic, probe, nic->netdev, "%s cannot find proper pci device base address, abording...\n", kidnet_msg);
		goto err_out_disable_pdev;
	}
	if ((err = pci_request_regions(pdev, kidnet_driver_name))) {
		netif_err(nic, probe, nic->netdev, "%s cannot obtain pci resources, abording...\n", kidnet_msg);
		goto err_out_disable_pdev;
	}
	if ((err = pci_set_dma_mask(pdev, DMA_BIT_MASK(32)))) {
		netif_err(nic, probe, nic->netdev, "%s no usable DMA configuration, abording...\n", kidnet_msg);
		goto err_out_free_res;
	}

	SET_NETDEV_DEV(netdev, &pdev->dev);

#if 0
	if (use_io)
		netif_info(nic, probe, nic->netdev, "using i/o access mode\n");

	nic->csr = pci_iomap(pdev, (use_io ? 1 : 0), sizeof(struct csr));
	if (!nic->csr) {
		netif_err(nic, probe, nic->netdev, "Cannot map device registers, aborting\n");
		err = -ENOMEM;
		goto err_out_free_res;
	}
#endif

	if (ent->driver_data)
		nic->flags |= ich;
	else
		nic->flags &= ~ich;

	kidnet_get_defaults(nic);

	/* D100 MAC doesn't allow rx of vlan packets with normal MTU */
	if (nic->mac < mac_82558_D101_A4)
		netdev->features |= NETIF_F_VLAN_CHALLENGED;

	/* locks must be initialized before calling hw_reset */
	spin_lock_init(&nic->cb_lock);
	spin_lock_init(&nic->cmd_lock);
	spin_lock_init(&nic->mdio_lock);

	/* Reset the device before pci_set_master() in case device is in some
	 * funky state and has an interrupt pending - hint: we don't have the
	 * interrupt handler registered yet. */
	kidnet_hw_reset(nic);

	pci_set_master(pdev);

	timer_setup(&nic->watchdog, kidnet_watchdog, 0);

	INIT_WORK(&nic->tx_timeout_task, e100_tx_timeout_task);

	if ((err = kidnet_alloc(nic))) {
		netif_err(nic, probe, nic->netdev, "Cannot alloc driver memory, aborting\n");
		goto err_out_iounmap;
	}

	if ((err = kidnet_eeprom_load(nic)))
		goto err_out_free;

	kidnet_phy_init(nic);

	memcpy(netdev->dev_addr, nic->eeprom, ETH_ALEN);
	if (!is_valid_ether_addr(netdev->dev_addr)) {
		netif_err(nic, probe, nic->netdev, "%s Invalid MAC address from EEPROM, you MUST configure one.\n", kidnet_msg);
	}
	
	/* Wol magic packet can be enabled from eeprom */
	if ((nic->mac >= mac_82558_D101_A4) &&
	   (nic->eeprom[eeprom_id] & eeprom_id_wol)) {
		nic->flags |= wol_magic;
		device_set_wakeup_enable(&pdev->dev, true);
	}

	/* ack any pending wake events, disable PME */
	pci_pme_active(pdev, false);

	strcpy(netdev->name, "eth%d");
	if ((err = register_netdev(netdev))) {
		netif_err(nic, probe, nic->netdev, "Cannot register net device, aborting\n");
		goto err_out_free;
	}

	nic->cbs_pool = pci_pool_create(netdev->name,
			   nic->pdev,
			   nic->params.cbs.max * sizeof(struct cb),
			   sizeof(u32),
			   0);

	if (!nic->cbs_pool) {
		netif_err(nic, probe, nic->netdev, "Cannot create DMA pool, aborting\n");
		err = -ENOMEM;
		goto err_out_pool;
	}

	netif_info(nic, probe, nic->netdev,
		   "addr 0x%llx, irq %d, MAC addr %pM\n",
		   (unsigned long long)pci_resource_start(pdev, use_io ? 1 : 0),
		   pdev->irq, netdev->dev_addr);


err_out_pool:
	unregister_netdev(netdev);
err_out_free:
	kidnet_free(nic);
err_out_iounmap:
	pci_iounmap(pdev, nic->csr);
err_out_free_res:
	pci_release_regions(pdev);
err_out_disable_pdev:
	pci_disable_pdev(pdev);
err_out_free_dev:
	free_netdev(netdev);
	return err;
}
static void 
kidnet_remove_one(struct pci_dev *pdev) {
	struct net_device *dev = pci_get_drvdata(pdev);

	flush_sheduled_work();

	unregister_netdev(dev);


	pci_disable_device(pdev);
}


#if 1
//struct pci_device_id pci_device_id;
struct pci_driver kidnet_driver = {
	.name = kidnet_driver_name,
	.id_table = kidnet_pci_tbl, /* The device list this driver support. */
	.probe = kidnet_init_one, /* It is called at device detection if it is a prescribed driver, or at (possibly) modprobe otherwise. */
	.remove =  kidnet_remove_one, /* It is called at device unload. */
#if 0
#ifdef CONFIG_PM /*When the power control is ON. */
	.suspend =
	.resume = 
#endif /* CONFIG_PM */
#endif
};
#endif


#if 0
static const struct net_device_ops kidnet_ops = {
	.ndo_open =
	.ndo_stop = 
	.ndo_start_xmit = 
	.ndo_get_stats =
	.ndo_set_rx_mode = 
//	.ndo_set_mac_address = 
	.ndo_tx_timeout = 
//	.ndo_change_mtu = 
	.ndo_do_ioctl = 
	.ndo_validate_addr = 
//	.ndo_vlan_rx_add_vid =
//	.ndo_vlan_rx_kill_vid =
	.ndo_poll_controller =
	.ndo_fix_features =
	.ndo_set_features =
}:
#endif

#if 0
static const struct ethtool_ops kidnet_ethtool_ops = {

};
#endif


static int 
kidnet_init(void) {
	printk(KERN_ALERT "%s loading kidnet.\n", kidnet_msg);
	return 0;

	int ret;
	ret = pci_register_driver(&kidnet_driver);

	return ret;
}
static void 
kidnet_exit(void) {
	printk(KERN_ALERT "%s kidnet bye.\n", kidnet_msg);

	pci_unregister_driver(&kidnet_driver);

	return;
}

module_init(kidnet_init);
module_exit(kidnet_exit);
