#include<linux/module.h>
#include<linux/types.h>
#include<linux/kernel.h>
#include<linux/pci.h>
#include<linux/netdevice.h>
#include<linux/ethtool.h>
#include<linux/dma-mapping.h>

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



//*******************************//
struct nic {
	/* Begin: frequently used values: keep adjacent for cache effect */
	u32 msg_enable				____cacheline_aligned;
	struct net_device *netdev;
	struct pci_dev *pdev;
	//u16 (*mdio_ctrl)(struct nic *nic, u32 addr, u32 dir, u32 reg, u16 data);

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
//**********************************************//



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
	//nic->mdio_ctrl = mdio_ctrl_hw; ??
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
