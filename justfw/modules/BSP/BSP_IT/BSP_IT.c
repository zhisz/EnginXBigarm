#include "BSP_IT.h"

static Bus_TopicHandleTypeDef *_gpio_exit_callback_tp;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    Bus_Publish(_gpio_exit_callback_tp, &GPIO_Pin);
}

void BSP_IT_init() {
    _gpio_exit_callback_tp = Bus_TopicRegister("/GPIO/EXIT");
}