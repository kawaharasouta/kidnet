#ifndef _DEBUG_UTIL_H_
#define _DEBUG_UTIL_H_

#include"kidnet.h"


inline void 
kidnet_dump_ctrl(struct net_device *netdev) {
	uint32_t ctrl = kidnet_readl(netdev, 0x0000);
	printk(KERN_INFO "%s ctrl: %08x.\n", kidnet_msg, ctrl);
}

inline void 
kidnet_dump_status(struct net_device *netdev) {
	uint32_t status = kidnet_readl(netdev, 0x0008);
	printk(KERN_INFO "%s status: %08x.\n", kidnet_msg, status);
}

inline void 
kidnet_dump_reg(struct net_device *netdev) {
	kidnet_dump_ctrl(netdev);
	kidnet_dump_status(netdev);
}
 

#endif /*_DEBUG_UTIL_H_*/
