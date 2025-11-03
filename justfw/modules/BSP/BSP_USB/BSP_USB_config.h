#ifndef __BSP_USB_config_H__
#define __BSP_USB_config_H__

#include "interface.h"

#define BSP_USB_RX_PACKAGE_DLAY 5            // 等待分包时长
#define BSP_USB_RX_PACKAGE_BUFFER_SIZE 1024  // 分包缓存区大小
#define BSP_USB_RX_PACKAGE_MAX_SIZE 64       // 分包最大大小

#define BSP_USB_TX_BUFFER_SIZE 1024  // 发送缓存区大小

//@item:发送FIFO缓存区大小
//@type:int
//@default:16
//@range:1~1024
#define BSP_USB_TX_FIFO_SIZE 16

//@item:接收FIFO缓存区大小
//@type:int
//@default:16
//@range:1~1024
#define BSP_USB_RX_FIFO_SIZE 16

//@item:USB接收话题名称
//@type:string
//@default:"USB_RX"
#define BSP_USB_RX_TOPIC_NAME "USB_RX"

//@item:USB发送话题名称
//@type:string
//@default:"USB_TX"
#define BSP_USB_TX_TOPIC_NAME "USB_TX"

#endif