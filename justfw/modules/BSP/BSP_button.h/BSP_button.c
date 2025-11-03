#include "BSP_button.h"

#include "BSP_button_conf.h"

static osThreadId bsp_button_task_handle;

static Bus_TopicHandleTypeDef *bsp_button_down_topic;
static Bus_TopicHandleTypeDef *bsp_button_up_topic;

static const char *TAG = "BUTTON";

static uint8_t status = 1;

static void BSP_button_task_mainloop() {
    while (1) {
        uint8_t current_status = HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_GPIO_PIN);

        if (current_status != status) {
            if (current_status == 0) {
                Bus_Publish(bsp_button_down_topic, NULL);
                elog_i(TAG, "Button pressed");
            } else {
                Bus_Publish(bsp_button_up_topic, NULL);
                elog_i(TAG, "Button released");
            }

            status = current_status;
        }

        osDelay(10);
    }
}

void BSP_button_init(void) {
    bsp_button_down_topic = Bus_TopicRegister(BUTTON_DOWN_TOPIC);
    bsp_button_up_topic = Bus_TopicRegister(BUTTON_UP_TOPIC);

    osThreadDef(BSP_button_task, BSP_button_task_mainloop, osPriorityNormal, 0, 512);
    bsp_button_task_handle = osThreadCreate(osThread(BSP_button_task), NULL);
}