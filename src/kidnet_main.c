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
#include<linux/pm_runtime.h>

#include"include/kidnet.h"
#include"include/debug_util.h"
#include"include/reg.h"

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Kawaharasouta <kawahara6514@gmail.com>");
MODULE_DESCRIPTION("network module");

static char kidnet_driver_name[] = "kidnet";

static struct pci_device_id kidnet_pci_tbl[] = {
  INTEL_KIDNET_ETHERNET_DEVICE(0x100E),
  INTEL_KIDNET_ETHERNET_DEVICE(0x2E6E), /* e1000 kvm */

	INTEL_KIDNET_ETHERNET_DEVICE(0x10d3), /* 82574L */
	INTEL_KIDNET_ETHERNET_DEVICE(0x1528), /* X540-t1, X540-t2 */
	/* required last entry */
	{0,}
};


irqreturn_t 
kidnet_intr(int irq, void *dev_id) {
	//struct kidnet_adapter *adapter = netdev_priv((struct net_device *)dev_id);
	
	//spin_lock(&adapter->lock);
	printk(KERN_INFO "%s reception.\n", kidnet_msg);


	return IRQ_HANDLED;
}

static inline void 
kidnet_global_reset(struct net_device *netdev) {
	//printk(KERN_INFO "%s global_reset.\n", kidnet_msg);
	uint32_t ctrl;
	ctrl = kidnet_readl(netdev, 0x0000);

	//!set the RST bit.
	ctrl |= 0x04000000;

	kidnet_writel(netdev, 0x0000, ctrl);
	//kidnet_dump_reg(netdev);
}

static void
kidnet_initialize_phy_setup_link(struct net_device *netdev) {
	//!MAC setting automatically based on duplex and speed resolved by phy.
	uint32_t ctrl;
	ctrl = kidnet_readl(netdev, 0x0000);
	
	//!CTRL.SLU
	ctrl |= 0x00000040;
	kidnet_writel(netdev, 0x0000, ctrl);
}

static inline void 
kidnet_enable_irq(struct net_device *netdev) {
	uint32_t ims = kidnet_readl(netdev, 0x00D0);

	ims |= IMS_ENABLE_MASK;
	kidnet_writel(netdev, 0x00D0, ims);
}

static inline void 
kidnet_disable_irq(struct net_device *netdev) {
	kidnet_writel(netdev, 0x00D8, ~0);
}


static int kidnet_alloc_ringdesc_dma(struct kidnet_adapter *adapter, struct kidnet_ring *ring) {
	struct pci_dev *pdev = adapter->pdev;

	ring->desc = dma_alloc_coherent(&pdev->dev, ring->size, &ring->dma, GFP_KERNEL);

	if (!ring->desc)
		return -ENOMEM;
	
	return 0;
}

static void 
kidnet_tx_configure(struct kidnet_adapter *adapter) {
	//!e1000e_configure_tx 
	//!init desc
	struct kidnet_ring *tx_ring = adapter->tx_ring;
	struct net_device *netdev = adapter->netdev;
	uint64_t tdba;
	uint32_t tdlen;

	//! config tx descripter.
	tdba = tx_ring->dma;
	tdlen = tx_ring->count * sizeof(struct kidnet_regacy_tx_desc);

	kidnet_writel(netdev, 0x3800, tdba & DMA_BIT_MASK(32)); //TDBAL
	kidnet_writel(netdev, 0x3804, tdba >> 32);					//TDBAH
	kidnet_writel(netdev, 0x3808, tdlen);								//TDLEN
	kidnet_writel(netdev, 0x3810, 0);								//TDH
	kidnet_writel(netdev, 0x3818, 0);								//TDT
	
	tx_ring->head = adapter->mmio_addr + 0x3810;
	tx_ring->tail = adapter->mmio_addr + 0x3818;

	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;

	return;
}

static int 
kidnet_tx_setup(struct net_device *netdev) {
	//!tx_reg_setting
	
	uint32_t txdctl, tctl, tipg;
	//txdctl = kidnet_readl(netdev, 0x3828); 
	//tctl = kidnet_readl(netdev, 0x0400); 
	//tipg = kidnet_readl(netdev, 0x0410); 

	//!TXDCTL GRAN = 1b, WTHRESH = 1b, all other fiekds 0b.
	txdctl = 0x02010000;

	//!TCTL CT = 0x0f, COLD : HDX = 0x1ff, FDC = 0x3f
	//!			PSP = 1b, EN = 1b, all other fields = 0b.
	tctl = 0x0003f0fa;

	//!TIPG IPGT = 8, IPGR1 = 2, IPGR2 = 10
	tipg = 0x10100808;

	//!write val
	kidnet_writel(netdev, 0x3828, txdctl);
	kidnet_writel(netdev, 0x0400, tctl);
	kidnet_writel(netdev, 0x0410, tipg);


	//!e1000e_setup_tx_resources
	//!init buffer
	struct kidnet_adapter *adapter = (struct kidnet_adapter *)(netdev_priv(netdev));
	struct kidnet_ring *tx_ring = adapter->tx_ring;
	int ret, size;

	ret = -ENOMEM;
	size = sizeof(struct kidnet_buffer) * tx_ring->count;
	tx_ring->buffer_info = vzalloc(size);
	if (tx_ring->buffer_info)
		goto err;
	
	tx_ring->size = tx_ring->count * sizeof(struct kidnet_regacy_tx_desc);
	tx_ring->size = ALIGN(tx_ring->size, 4096);

	ret = kidnet_alloc_ringdesc_dma(adapter, tx_ring);
	if (ret)
		goto err;

	kidnet_tx_configure(adapter);

	return ret;
	
err:
	vfree(tx_ring->buffer_info);
	return ret;
}

static void  
kidnet_rx_configure(struct kidnet_adapter *adapter) {
	uint64_t rdba;
	uint32_t rdlen, rctl;
	struct kidnet_ring *rx_ring = adapter->rx_ring;
	struct net_device *netdev = adapter->netdev;

	rdba = rx_ring->dma;
	rdlen = rx_ring->count * sizeof(struct kidnet_regacy_rx_desc);


	//!write desc
	kidnet_writel(netdev, 0x2800, rdba & DMA_BIT_MASK(32)); //RDBAL
	kidnet_writel(netdev, 0x2804, rdba >> 32);							//RDBAH
	kidnet_writel(netdev, 0x2808, rdlen);										//RDLEN
	kidnet_writel(netdev, 0x2810, 0);												//TDH
	kidnet_writel(netdev, 0x2818, 0);												//TDT

	rx_ring->head = adapter->mmio_addr + 0x2810;
	rx_ring->tail = adapter->mmio_addr + 0x2818;


	rctl = kidnet_readl(netdev, 0x0100); 

	//!EN = 1b, VFE = 0b
	rctl |= 0x00000002;
	rctl &= 0xfffbffff;

	kidnet_writel(netdev, 0x0100, rctl);

	return;
}

static int 
kidnet_rx_setup(struct net_device *netdev) {
	struct kidnet_adapter *adapter = (struct kidnet_adapter *)(netdev_priv(netdev));
	struct kidnet_ring *rx_ring = adapter->rx_ring;
	int ret, size;

	ret = -ENOMEM;
	size = sizeof(struct kidnet_buffer) * rx_ring->count;
	rx_ring->buffer_info = vzalloc(size);
	if (rx_ring->buffer_info)
		goto err;

	rx_ring->size = rx_ring->count * sizeof(struct kidnet_regacy_rx_desc);
	rx_ring->size = ALIGN(rx_ring->size, 4096);

	ret = kidnet_alloc_ringdesc_dma(adapter, rx_ring);
	if (ret)
		goto err;

	kidnet_rx_configure(adapter);


	int ics;

	ics = kidnet_readl(netdev, 0x00c8); 
	//!
	ics |= 0x000000d4;
	
	kidnet_rx_configure(adapter);

	return ret;

err:
	vfree(rx_ring->buffer_info);
	return ret;
}

static int
kidnet_alloc_ring(struct kidnet_adapter *adapter) {
	int size;
	size = sizeof(struct kidnet_ring);

	adapter->tx_ring = kzalloc(size, GFP_KERNEL);
	if (!adapter->tx_ring)
		goto tx_err;
	adapter->tx_ring->count = adapter->tx_ring_count;
	adapter->tx_ring->adapter = adapter;


	adapter->rx_ring = kzalloc(size, GFP_KERNEL);
	if (!adapter->rx_ring)
		goto rx_err;
	adapter->rx_ring->count = adapter->rx_ring_count;
	adapter->rx_ring->adapter = adapter;

	return 0;

rx_err:
	kfree(adapter->rx_ring);
tx_err:
	kfree(adapter->tx_ring);
	return -ENOMEM;
}

static void 
kidnet_adapter_init(struct kidnet_adapter *adapter) {
	struct net_device *netdev = adapter->netdev;
	int ret;
	ret = 0;
	//! e1000_sw_init
	//adapter->rx_buffer_len = 
	//adapter->rx_ps_bsize0 = 128;
	adapter->max_frame_size = netdev->mtu + ETH_FCS_LEN;
	adapter->min_frame_size = ETH_ZLEN + ETH_FCS_LEN;
	adapter->tx_ring_count = KIDNET_DEFAULT_TXD;
	adapter->rx_ring_count = KIDNET_DEFAULT_RXD;

	spin_lock_init(&adapter->lock);


	ret = kidnet_alloc_ring(adapter);
//	if (ret)
//		return ret;
	return ret;

}

//void 
//read_phyaddr(struct net_device *netdev) {
//	uint32_t mdic = kidnet_readl(netdev, 0x00020);
//printk(KERN_INFO "%s mdic: %08x.\n", kidnet_msg, mdic);
//
//	uint8_t phyaddr = (uint8_t)mdic >> 21 & 0x1f;
//	printk(KERN_INFO "%s phyaddr: %02x.\n", kidnet_msg, phyaddr);
//
//}


int 
kidnet_open(struct net_device *netdev) {
	struct pci_dev *pdev = ((struct kidnet_adapter *)(netdev_priv(netdev)))->pdev;
	struct netdev_queue *queue;
	//printk(KERN_INFO "%s kidnet_open.\n", kidnet_msg);

	netif_stop_queue(netdev);

	pm_runtime_get_sync(&pdev->dev);

	kidnet_disable_irq(netdev);
	kidnet_global_reset(netdev);
	kidnet_disable_irq(netdev);

	kidnet_initialize_phy_setup_link(netdev);


	//!tx_config
	kidnet_tx_setup(netdev);

	//!rx_config
	kidnet_rx_setup(netdev);


	//netif_carrier_on(netdev);
	kidnet_enable_irq(netdev);
	//////////////!!!!!!!!!!!!!!!!!!!!error!!!!!!!!!!!!!!!!
	//queue = netdev_get_tx_queue(netdev, 0);
	netif_start_queue(netdev);
	
	//pm_runtime_put(&pdev->dev);


	//!ims up RxQ0, TxQ0
	uint32_t ims;
	ims = kidnet_readl(netdev, 0x00d0);
	ims |= 0x00500000;
	kidnet_writel(netdev, 0x00d0, ims);

	//!link up (ICR.LSC)
	uint32_t icr;
	icr = kidnet_readl(netdev, 0x00c0);
	//printk(KERN_INFO "%s icr: %08x.\n", kidnet_msg, icr);

	//kidnet_dump_reg(netdev);
	return 0;
}
int 
kidnet_close(struct net_device *netdev) {
	netif_stop_queue(netdev);
}


int 
kidnet_hw_legacy_tx(struct sk_buff *skb, struct net_device *netdev) {
	struct kidnet_adapter *adapter = netdev_priv(netdev);
	struct pci_dev *pdev = adapter->pdev;
	struct kidnet_ring *tx_ring = adapter->tx_ring;
	struct kidnet_regacy_tx_desc *tx_desc;
	struct kidnet_buffer *buffer_info;
	int index = tx_ring->next_to_use;

		
	//!map
	buffer_info = &tx_ring->buffer_info[index];
	buffer_info->dma = dma_map_single(&pdev->dev, skb->data, skb->len, DMA_TO_DEVICE);
	if (dma_mapping_error(&pdev->dev, buffer_info->dma))
			goto dma_err;


	tx_desc = (struct kidnet_regacy_tx_desc *)&tx_ring->desc[index];
	tx_desc->buffer_addr = cpu_to_le64(buffer_info->dma);
	tx_desc->length = cpu_to_le16(skb->len);

	kidnet_writel(netdev, index, tx_ring->tail);

	index++;
	if (unlikely(index == tx_ring->count))
		index = 0;
	tx_ring->next_to_use = index;
	
	return 1;

dma_err:
	buffer_info->dma = 0;
	return -1;

}

	
static netdev_tx_t 
kidnet_xmit_frame(struct sk_buff *skb, struct net_device *netdev) {
	struct kidnet_adapter *adapter = netdev_priv(netdev);
	int ret;
	printk(KERN_INFO "%s kidnet transmit packet.\n", kidnet_msg);


	if (eth_skb_pad(skb)) 
		return NETDEV_TX_OK;
		
	

	//netdev->trans_start = jiffies;

	//netdev_sent_queue(netdev, skb->len);
	//skb_tx_timestamp(skb);
//	ret = kidnet_hw_legacy_tx(skb, netdev);
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

	//printk(KERN_INFO "%s mac addr: %02x:%02x:%02x:%02x:%02x:%02x\n", kidnet_msg, netdev->dev_addr[0], netdev->dev_addr[1], netdev->dev_addr[2], netdev->dev_addr[3], netdev->dev_addr[4], netdev->dev_addr[5]);
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


	ret = pci_enable_device(pdev);
	if (ret)
		return ret;
	
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		goto err_dma;
	}

	//!add this call to get the correct BAR mask
	bars = pci_select_bars(pdev, 0);
	ret = pci_request_region(pdev, bars, kidnet_driver_name);
	if (ret)
		goto err_pci_reg;

#endif

	//printk(KERN_INFO "%s alloc_etherdev.\n", kidnet_msg);
	ret = -ENOMEM;
	netdev = alloc_etherdev(sizeof(struct kidnet_adapter));
	if (!netdev)
		goto err_alloc_etherdev;

	//printk(KERN_INFO "%s set netdev adapter.\n", kidnet_msg);
	SET_NETDEV_DEV(netdev, &pdev->dev);
	netdev->irq = pdev->irq;
	//printk(KERN_INFO "%s pdev->irq:%x\n", kidnet_msg, pdev->irq);

	pci_set_drvdata(pdev, netdev);
	adapter = netdev_priv(netdev);
	adapter->netdev = netdev;
	adapter->pdev = pdev;

	//! setup adapter struct 
	kidnet_adapter_init(adapter);
	//kidnet_disable_irq(netdev);

	//!mmio setting
	//printk(KERN_INFO "%s pci_resource_start.\n", kidnet_msg);
	mmio_start = pci_resource_start(pdev, bars);
	mmio_len	= pci_resource_len(pdev, bars);
	ret = EIO;
	//printk(KERN_INFO "%s ioremap.\n", kidnet_msg);
	adapter->mmio_addr = ioremap(mmio_start, mmio_len);
	if (!adapter->mmio_addr)
		goto err_ioremap;

	netdev->netdev_ops = &kidnet_ops;

	//! set netdev mmio.
	netdev->mem_start = mmio_start;
	netdev->mem_end = mmio_start + mmio_len;

	//!set mac addr
	kidnet_set_macaddr(netdev);

	//!set irq. use regacy irq.
	ret = request_irq(pdev->irq, kidnet_intr, IRQF_SHARED, kidnet_driver_name, netdev);
	if (ret)
		goto err_irq;

	//printk(KERN_INFO "%s register_netdev.\n", kidnet_msg);
	ret = register_netdev(netdev);	
	if (ret)
		goto err_register;
	
	//kidnet_dump_reg(netdev);

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
