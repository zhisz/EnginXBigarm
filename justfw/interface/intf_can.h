//
// Created by Ukua on 2023/8/12.
//

#ifndef JUSTFW_INTF_CAN_H
#define JUSTFW_INTF_CAN_H

#include <stdint.h>

//标识符类型枚举
typedef enum INTF_CAN_ID_Type{
    BSP_CAN_ID_STD=0,         //标准帧（11位ID）
    BSP_CAN_ID_EXT,           //扩展帧（29位ID）
}INTF_CAN_ID_Type;

//帧类型枚举
typedef enum INTF_CAN_RTR_Type{
    BSP_CAN_RTR_DATA=0,       //数据帧（携带数据）       
    BSP_CAN_RTR_REMOTE,       //远程帧（请求数据）
}INTF_CAN_RTR_Type;


typedef struct INTF_CAN_Message{
    INTF_CAN_ID_Type id_type;//ID类型，默认为标准帧
    INTF_CAN_RTR_Type rtr_type;//RTR类型，默认为数据帧
    uint32_t can_id;           //CAN标识符（11位或29位）
    uint8_t data[8];           //数据负载（最多8字节）
}INTF_CAN_MessageTypeDef;


#endif //JUSTFW_INTF_CAN_H
