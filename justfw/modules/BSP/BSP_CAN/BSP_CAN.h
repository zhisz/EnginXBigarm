//
// Created by Ukua on 2023/8/12.
//

#ifndef JUSTFW_CAN_H
#define JUSTFW_CAN_H



#include "main.h"

#include "interface.h"
#include "cmsis_os.h"
#include "fifo.h"
#include "tinybus.h"

#include "BSP_CAN_config.h"


/**
 * @brief CAN发送函数
 * @param *data:要发送的数据帧(8byte)
 * @param idType:数据帧的ID类型
 * @param id:数据帧的ID
 * @param port:要发送到的can端口
 */
void BSP_CAN_Transmit(uint8_t *data, INTF_CAN_ID_Type idType,INTF_CAN_RTR_Type rtrType, uint32_t id, CAN_HandleTypeDef *port);

/**
 * @brief CAN标准发送话题回调函数
 * @param *message:消息
 * @param *topic:来源话题
 * @note 该函数会将消息储存在fifo中
 */
void BSP_CAN_TX_CallBack(void *message, Bus_TopicHandleTypeDef *topic);

void BSP_CAN_MainLoop();

void BSP_CAN_Init();

#endif //JUSTFW_CAN_H
