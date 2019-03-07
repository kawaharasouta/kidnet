#include<linux/module.h>
#include<asm/types.h>
#include<linux/kernel.h>
#include<linux/pci.h>

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


static int 
kidnet_init_one(struct pci_dev *pdev, const struct pci_device_id *ent) {
	struct net_device *netdev;
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
