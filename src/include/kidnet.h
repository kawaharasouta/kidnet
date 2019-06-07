#ifndef _KIDNET_H_
#define _KIDNET_H_


#define INTEL_KIDNET_ETHERNET_DEVICE(device_id) {\
				PCI_DEVICE(PCI_VENDOR_ID_INTEL, device_id)}


#define kidnet_write(func, pdev, reg, val) \
	func(val, (uint8_t *)(((struct kidnet_adapter *)netdev_priv(pdev)))->mmio_addr + reg)
#define kidnet_writeb(pdev, reg, val) kidnet_write(writeb, pdev, reg, val)
#define kidnet_writew(pdev, reg, val) kidnet_write(writew, pdev, reg, val)
#define kidnet_writel(pdev, reg, val) kidnet_write(writel, pdev, reg, val)

#define kidnet_read(func, pdev, reg) \
	func((uint8_t *)(((struct kidnet_adapter *)netdev_priv(pdev)))->mmio_addr + reg)
#define kidnet_readb(pdev, reg) kidnet_read(readb, pdev, reg)
#define kidnet_readw(pdev, reg) kidnet_read(readw, pdev, reg)
#define kidnet_readl(pdev, reg) kidnet_read(readl, pdev, reg)



struct kidnet_adapter {
	struct net_device *netdev;
	struct pci_dev *pdev;
	struct net_device_stats stats;

	uint32_t *mem_start;
	int mem_len;
	void *mmio_addr;
};


#endif /* _KIDNET_H_ */
