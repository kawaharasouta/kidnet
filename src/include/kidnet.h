#ifndef _KIDNET_H_
#define _KIDNET_H_


#define INTEL_KIDNET_ETHERNET_DEVICE(device_id) {\
				PCI_DEVICE(PCI_VENDOR_ID_INTEL, device_id)}


#define kidnet_write(func, dev, reg, val) \
	func(val, (uint8_t *)(((struct kidnet_adapter *)netdev_priv(dev)))->mmio_addr + reg)
#define kidnet_writeb(dev, reg, val) kidnet_write(writeb, dev, reg, val)
#define kidnet_writew(dev, reg, val) kidnet_write(writew, dev, reg, val)
#define kidnet_writel(dev, reg, val) kidnet_write(writel, dev, reg, val)

#define kidnet_read(func, dev, reg) \
	func((uint8_t *)(((struct kidnet_adapter *)netdev_priv(dev)))->mmio_addr + reg)
#define kidnet_readb(dev, reg) kidnet_read(readb, dev, reg)
#define kidnet_readw(dev, reg) kidnet_read(readw, dev, reg)
#define kidnet_readl(dev, reg) kidnet_read(readl, dev, reg)



struct kidnet_adapter {
	struct net_device *netdev;
	struct pci_dev *pdev;
	struct net_device_stats stats;

	spinlock_t lock;

	uint32_t *mem_start;
	int mem_len;
	void *mmio_addr;
};


#endif /* _KIDNET_H_ */
