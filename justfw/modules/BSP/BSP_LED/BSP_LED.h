#ifndef __BSP_LED_H
#define __BSP_LED_H
#include "interface.h"

#define LED_PORT GPIOC
#define LED_PIN GPIO_PIN_13

void BSP_LED_Init(void);

#endif