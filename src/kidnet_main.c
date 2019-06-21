#include<linux/module.h>
#include<linux/types.h>
#include<linux/kernel.h>
#include<linux/pci.h>
#include<linux/etherdevice.h>
#include<linux/netdevice.h>
#include<linux/ethtool.h>
#include<linux/dma-mapping.h>
#include<linux/timer.h>
#include<linux/workqueue.h>
#include<linux/rtnetlink.h>

#include"include/kidnet.h"
#include"include/debug_util.h"

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Kawaharasouta <kawahara6514@gmail.com>");
MODULE_DESCRIPTION("network module");

static char kidnet_driver_name[] = "kidnet";

static struct pci_device_id kidnet_pci_tbl[] = {
	INTEL_KIDNET_ETHERNET_DEVICE(0x10d3), /* 82574L */
	INTEL_KIDNET_ETHERNET_DEVICE(0x1528), /* X540-t1, X540-t2 */
	/* required last entry */
	{0,}
};

irqreturn_t 
kidnet_intr(int irq, void *dev_id) {
	//struct kidnet_adapter *adapter = netdev_priv((struct net_device *)dev_id);
	
	//spin_lock(&adapter->lock);



}



int 
kidnet_open(struct net_device *netdev) {
	printk(KERN_INFO "%s kidnet_open.\n", kidnet_msg);

	//netif_carrier_off(netdev);
	//netif_start_queue(netdev);	
	netif_wake_queue(netdev);

	kidnet_dump_reg(netdev);
	
	uint8_t ims = kidnet_readb(netdev, 0x00D0);
	printk(KERN_INFO "%s ims: %02x.\n", kidnet_msg, ims);
	ims |= 0x40;
	//kidnet_writeb(netdev, 0x00D0, ims);
	ims = kidnet_readb(netdev, 0x00D0);
	printk(KERN_INFO "%s ims: %02x.\n", kidnet_msg, ims);
}
int 
kidnet_close(struct net_device *netdev) {
	netif_stop_queue(netdev);
}


int 
kidnet_hw_legacy_tx(struct net_device *netdev, void *data, unsigned int len) {
	struct kidnet_adapter *adapter = netdev_priv(netdev);

	//struct kidnet_regacy_tx_desc *tail = kidnet_readl(netdev, 0x3818) | (kidnet_readl(netdev, 0x381c) << 4);
	struct kidnet_regacy_tx_desc *tail = kidnet_readl(netdev, 0x3818);
	printk(KERN_INFO "%s kidnet hw transmit packet.\n", kidnet_msg);
	printk(KERN_INFO "%s tx_ring tail: %p.\n", kidnet_msg, tail);

	tail->buffer_addr = (uint64_t)data;
	tail->length = len;

	tail += 4; 
	kidnet_writel(netdev, 0x3818, tail);

	return 0;
}

	
static netdev_tx_t 
kidnet_xmit_frame(struct sk_buff *skb, struct net_device *netdev) {
	struct kidnet_adapter *adapter = netdev_priv(netdev);
	int ret;
	printk(KERN_INFO "%s kidnet transmit packet.\n", kidnet_msg);

	if (skb->len < ETH_ZLEN) {
		if (skb_pad(skb, ETH_ZLEN - skb->len)) {
			return NETDEV_TX_OK;
		}
	}

	//netdev->trans_start = jiffies;

	ret = kidnet_hw_legacy_tx(netdev, skb->data, skb->len);
	if (ret < 0) {
		adapter->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	adapter->stats.tx_packets++;
	adapter->stats.tx_bytes = skb->len;
	
	return NETDEV_TX_OK;
}

struct net_device_stats *
kidnet_get_stats(struct net_device *dev) {
	
}
static const struct net_device_ops kidnet_ops = {
	.ndo_open = kidnet_open,
	.ndo_stop = kidnet_close,
	.ndo_start_xmit = kidnet_xmit_frame,
	.ndo_get_stats = kidnet_get_stats
};


void 
kidnet_set_macaddr(struct net_device *netdev) {
	uint32_t rah, ral;
	struct kidnet_adapter *adapter;
	
	adapter = netdev_priv(netdev);
	
	rah = kidnet_readl(netdev, 0x05400);
	ral = kidnet_readl(netdev, 0x05404);

	netdev->dev_addr[0] = rah & 0xff;
	netdev->dev_addr[1] = (rah >> 8) & 0xff;
	netdev->dev_addr[2] = (rah >> 16) & 0xff;
	netdev->dev_addr[3] = (rah >> 24) & 0xff;

	netdev->dev_addr[4] = ral & 0xff;
	netdev->dev_addr[5] = (ral >> 8) & 0xff;

	
	printk(KERN_INFO "%s mac addr: %02x:%02x:%02x:%02x:%02x:%02x\n", kidnet_msg, netdev->dev_addr[0], netdev->dev_addr[1], netdev->dev_addr[2], netdev->dev_addr[3], netdev->dev_addr[4], netdev->dev_addr[5]);
}


static int 
kidnet_probe(struct pci_dev *pdev, const struct pci_device_id *ent) {
	int ret;
	struct net_device *netdev;
	struct kidnet_adapter *adapter;
	resource_size_t mmio_start, mmio_len;
	int bars;

#if 0
	//!initialize device memory.
	printk(KERN_INFO "%s pci_enable_device.\n", kidnet_msg);
	ret = pci_enable_device_mem(pdev);
	if (ret)
		return ret;
	//!dma
	printk(KERN_INFO "%s dma_set_mask_and_coherent.\n", kidnet_msg);
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (!ret) {
		goto err_dma;
	}

	//!setting base address register????????????????????????????????????
	printk(KERN_INFO "%s pci_select_bars.\n", kidnet_msg);
	bars = pci_select_bars(pdev, IORESOURCE_MEM);
	printk(KERN_INFO "%s pci_request_selected_regions_exclusive.\n", kidnet_msg);
	ret = pci_request_selected_regions_exclusive(pdev, bars, kidnet_driver_name);
	if (ret)
		goto err_pci_reg;

	//!Enables bus-mastering on the device and calls pcibios_set_master to do the needed arch specific settings.
	printk(KERN_INFO "%s pci_set_master.\n", kidnet_msg);
	pci_set_master(pdev);

	//!save the PCI configuration space of a device before suspending
	printk(KERN_INFO "%s pci_save_state.\n", kidnet_msg);
	ret = pci_save_state(pdev);
	if (ret)
		goto err_alloc_etherdev;
#else
//	//!request mem region (prederres to be excuted before ioremap)
//	ret = pci_request_regions(pdev, kidnet_driver_name);
//	if (ret)
//		return ret;

	//!add this call to get the correct BAR mask
	bars = pci_select_bars(pdev, 0);

	ret = pci_enable_device(pdev);
	if (ret)
		return ret;

	ret = pci_request_region(pdev, bars, kidnet_driver_name);
	if (ret)
		return ret;

#endif

	printk(KERN_INFO "%s alloc_etherdev.\n", kidnet_msg);
	ret = -ENOMEM;
	netdev = alloc_etherdev(sizeof(struct kidnet_adapter));
	if (!netdev)
		goto err_alloc_etherdev;

	printk(KERN_INFO "%s set netdev adapter.\n", kidnet_msg);
	SET_NETDEV_DEV(netdev, &pdev->dev);
	netdev->irq = pdev->irq;
	printk(KERN_INFO "%s pdev->irq:%x\n", kidnet_msg, pdev->irq);

	pci_set_drvdata(pdev, netdev);
	adapter = netdev_priv(netdev);
	adapter->netdev = netdev;
	adapter->pdev = pdev;
	spin_lock_init(&adapter->lock);

	//!mmio setting
	printk(KERN_INFO "%s pci_resource_start.\n", kidnet_msg);
	mmio_start = pci_resource_start(pdev, bars);
	mmio_len	= pci_resource_len(pdev, bars);
	ret = EIO;
	printk(KERN_INFO "%s ioremap.\n", kidnet_msg);
	adapter->mmio_addr = ioremap(mmio_start, mmio_len);
	if (!adapter->mmio_addr)
		goto err_ioremap;

	netdev->netdev_ops = &kidnet_ops;

	//!set mac addr
	kidnet_set_macaddr(netdev);

	//!set irq. for instance, flags is SHARED.
	ret = request_irq(pdev->irq, kidnet_intr, IRQF_SHARED, kidnet_driver_name, netdev);
	if (ret)
		goto err_irq;

	printk(KERN_INFO "%s register_netdev.\n", kidnet_msg);
	ret = register_netdev(netdev);	
	if (ret)
		goto err_register;
	
	kidnet_dump_reg(netdev);

	return 0;

err_register:
	free_irq(pdev->irq, netdev);
err_irq:
	iounmap(adapter->mmio_addr);
err_ioremap:
	free_netdev(netdev);
err_alloc_etherdev:
	pci_release_mem_regions(pdev);
err_pci_reg:
err_dma:
	pci_disable_device(pdev);

	return ret;
}

static void 
kidnet_remove(struct pci_dev *pdev) {
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct kidnet_adapter *adapter = netdev_priv(netdev);

	unregister_netdev(netdev);
	free_irq(pdev->irq, netdev);
	iounmap(adapter->mmio_addr);
	free_netdev(netdev);
	pci_release_mem_regions(pdev);
	pci_disable_device(pdev);
}





struct pci_driver kidnet_driver = {
	.name = kidnet_driver_name,
	.id_table = kidnet_pci_tbl, /* The device list this driver support. */
	.probe = kidnet_probe, /* It is called at device detection if it is a prescribed driver, or at (possibly) modprobe otherwise. */
	.remove =  kidnet_remove, /* It is called at device unload. */
};



static const struct ethtool_ops kidnet_ethtool_ops = {

};


static int 
kidnet_init(void) {
	printk(KERN_ALERT "%s loading kidnet.\n", kidnet_msg);
	return pci_register_driver(&kidnet_driver);
}
static void 
kidnet_exit(void) {
	printk(KERN_ALERT "%s kidnet bye.\n", kidnet_msg);
	return pci_unregister_driver(&kidnet_driver);
}
module_init(kidnet_init);
module_exit(kidnet_exit);
