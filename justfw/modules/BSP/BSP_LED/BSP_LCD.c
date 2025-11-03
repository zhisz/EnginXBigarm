#include "BSP_LED.h"
#include "BSP_LED_conf.h"

static osThreadId LED_TaskHandle;
static Bus_SubscriberTypeDef* sub_led_countup;
static Bus_SubscriberTypeDef* sub_led_countdown;

static uint16_t period;
static uint16_t duty;
static uint8_t count;
static uint8_t max_count;

static void BSP_LED_mainloop() {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

    while (1) {
        for (uint8_t i = 0; i < count * 2; i++) {
            HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
            osDelay(duty / 2);
        }

        osDelay(period - duty * count);
    }
}

static void led_countup_callback(void*, Bus_TopicHandleTypeDef*) {
    if (count < max_count) {
        count++;
    }
}

static void led_countdown_callback(void*, Bus_TopicHandleTypeDef*) {
    if (count > 0) {
        count--;
    }
}

void BSP_LED_Init(void) {
    period = BSP_LED_DEFAULT_PERIOD;
    duty = BSP_LED_DEFAULT_DUTY;
    count = BSP_LED_DEFAULT_COUNT;

    max_count = period / duty;

    sub_led_countup = Bus_SubscribeFromName(BSP_LED_COUNTUP_TOPIC, led_countup_callback);
    sub_led_countdown = Bus_SubscribeFromName(BSP_LED_COUNTDOWN_TOPIC, led_countdown_callback);

    osThreadDef(LED_Task, BSP_LED_mainloop, osPriorityNormal, 0, 128);
    LED_TaskHandle = osThreadCreate(osThread(LED_Task), NULL);
}