//
// Created by Ukua on 2023/9/20.
//

#ifndef JUSTFW_INTF_UART_H
#define JUSTFW_INTF_UART_H

#include <stdint.h>

typedef struct INTF_UART_Message {
    uint16_t len;
    uint8_t *data;
} INTF_UART_MessageTypeDef;


#endif //JUSTFW_INTF_UART_H
