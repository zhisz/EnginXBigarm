//
// Created by Ukua on 2024/1/21.
//

#ifndef JUSTFW_GMMOTORS_H
#define JUSTFW_GMMOTORS_H

#include "tinybus.h"
#include "GM6020.h"
#include "C620.h"
#include "C610.h"
#include "GM6020.h"

#define GM_BUFFER_NUM 2

typedef struct GM_Buffer {
    Bus_TopicHandleTypeDef *can_tx_topic;
    Bus_TopicHandleTypeDef *can_rx_topic;
    uint8_t buffer_0x200[8];
    uint8_t buffer_0x1FF[8];
    uint8_t buffer_0x2FF[8];
    uint8_t buffer_0x200_not_null;//buffer非空标志位，用于防止发送空包
    uint8_t buffer_0x1FF_not_null;//buffer非空标志位，用于防止发送空包
    uint8_t buffer_0x2FF_not_null;//buffer非空标志位，用于防止发送空包
} GM_BufferTypeDef;

void GM_Init();

#endif //JUSTFW_GMMOTORS_H
