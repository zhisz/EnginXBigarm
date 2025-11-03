//
// Created by Ukua on 2023/11/2.
//

#ifndef JUSTFW_BSP_UART_H
#define JUSTFW_BSP_UART_H

#include <stdint.h>
#include "stdlib.h"
#include "interface.h"
#include "tinybus.h"
#include "main.h"
#include "fifo.h"

#define BSP_UART_TX_FIFO_SIZE 16
#define DEVICE_UART_CNT 4     // C板至多分配3个串口
#define UART_RXBUFF_LIMIT 256 // 如果协议需要更大的buff,请修改这里


typedef struct UART_InstanceConfig{
    UART_HandleTypeDef *UART_handle;
    uint16_t recv_buff_size;
    char *tx_topic_name;
    char *rx_topic_name;
}UART_InstanceConfigTypeDef;

// 串口实例结构体,每个module都要包含一个实例.
// 由于串口是独占的点对点通信,所以不需要考虑多个module同时使用一个串口的情况,因此不用加入id;当然也可以选择加入,这样在bsp层可以访问到module的其他信息
typedef struct UART_Instance
{
    uint8_t recv_buff[UART_RXBUFF_LIMIT]; // 预先定义的最大buff大小,如果太小请修改UART_RXBUFF_LIMIT
    uint8_t recv_buff_size;                // 模块接收一包数据的大小
    UART_HandleTypeDef *uart_handle;      // 实例对应的uart_handle
    Bus_SubscriberTypeDef *tx_topic;
    Bus_TopicHandleTypeDef *rx_topic;
    fifo_t *tx_fifo;
} UART_InstanceTypeDef;

//消息缓冲
typedef struct UART_Buffer
{
    UART_InstanceTypeDef *uart_instance;
    INTF_UART_MessageTypeDef *message;
} UART_BufferTypeDef;

/**
 * @brief 判断串口是否准备好,用于连续或异步的IT/DMA发送
 *
 * @param _instance 要判断的串口实例
 * @return uint8_t ready 1, busy 0
 */
uint8_t UARTIsReady(UART_InstanceTypeDef *_instance);


void BSP_UART_Init();

#endif //JUSTFW_BSP_UART_H
