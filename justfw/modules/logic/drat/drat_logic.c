//
// Created by LinorInk on 2024/9/12.
//

#include "drat_logic.h"

// #include "vofa.h"

#define DRAT_RT_SENSITIVITY 200          //定义摇杆灵敏度阈值sensitivity

osThreadId Drat_Logic_TaskHandle;        //定义任务句柄
static RC_ctrl_t *rc_ctrl;               //遥控器控制结构体指针
static INTF_Motor_HandleTypeDef *motor_test;   //测试电机句柄
static INTF_Motor_HandleTypeDef *motor_HL;     //左后电机句柄
static INTF_Motor_HandleTypeDef *motor_HR;
static INTF_Motor_HandleTypeDef *motor_TL;
static INTF_Motor_HandleTypeDef *motor_TR;
static Bus_TopicHandleTypeDef *usb_tx_topic, *usb_rx_topic, *can_tx_topic;  //定义通信主题句柄

static int8_t drat_flag = 0;       //标志位，用于判断是否进行角度调整

static int8_t Sensitivity_Judge(int16_t value);   //声明灵敏度判断函数

static float uart_data;         // 自定义数据，用于接收串口数据
static bool isReceive = false;  // 如果收到自定义数据, 则忽略遥控器

static void Drat_Logic_MainLoop() {
    float target_angle = 0;
    float target_speed;
    float feeder_angle = 0;
    uint16_t servo_angle = 160;     //伺服角度

    //can结构体，用于发送步进电机运动方向数据
    static INTF_CAN_MessageTypeDef drat_msg = {// 给步进电机发送运动方向的数据
                                               .can_id = 0x105,
                                               .data = {0},   //数据缓冲区
                                               .id_type = BSP_CAN_ID_STD,  //标准帧
                                               .rtr_type = BSP_CAN_RTR_DATA};  //数据帧类型

    while (1) {
        switch (rc_ctrl[0].rc.switch_left) {
            case RC_SW_DOWN:
                target_speed = 0; //目标速度
                 //HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
                break;
            case RC_SW_MID:
                target_speed = 300;
                 //HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
                break;
            case RC_SW_UP:
                target_speed = 600;
                 //HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
                break;
            default:
                target_speed = 0;
                 //HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        }

        //如果收到串口数据，则使用串口数据设置目标速度
        if (isReceive) {
            target_speed = uart_data;
        }

        //设置四个电机的速度
        motor_HL->set_speed(motor_HL, target_speed);
        motor_HR->set_speed(motor_HR, target_speed);
        motor_TL->set_speed(motor_TL, target_speed);
        motor_TR->set_speed(motor_TR, target_speed);

        //根据遥控器右开关位置和摇杆信号调整目标角度
        switch (rc_ctrl->rc.switch_right) {
            case RC_SW_UP:

                //如果摇杆向右偏转且未进行角度调整
                if ((Sensitivity_Judge(rc_ctrl[0].rc.rocker_r_) == 1) && !drat_flag) {
                    target_angle += 90 * PI / 180.0f;
                    //                    Emm_V5_Pos_Control(1, 0, 50, 0, 800, 0, 0);
                    drat_flag = 1;
                } 
                //向左
                else if ((Sensitivity_Judge(rc_ctrl[0].rc.rocker_r_) == -1) && !drat_flag) {
                    target_angle -= 90 * PI / 180.0f;
                    //                    Emm_V5_Pos_Control(1, 1, 50, 0, 800, 0, 0);
                    drat_flag = 1;
                }
                break;
            case RC_SW_DOWN:
                //向右
                if ((Sensitivity_Judge(rc_ctrl[0].rc.rocker_r_) == 1) && !drat_flag) {
                    target_angle += 5 * PI / 180.0f;
                    //                    Emm_V5_Pos_Control(1, 0, 50, 0, 50, 0, 0);

                    drat_flag = 1;
                } 
                //向左
                else if ((Sensitivity_Judge(rc_ctrl[0].rc.rocker_r_) == -1) && !drat_flag) {
                    target_angle -= 5 * PI / 180.0f;
                    //                    Emm_V5_Pos_Control(1, 1, 50, 0, 50, 0, 0);
                    drat_flag = 1;
                }
                break;

            case RC_SW_MID:  //根据摇杆信号调整伺服角度
                if ((Sensitivity_Judge(rc_ctrl[0].rc.rocker_r1) == 1)) {
                    servo_angle -= 1;    //减少伺服角度
                } else if (Sensitivity_Judge(rc_ctrl[0].rc.rocker_r1) == -1) {
                    servo_angle += 1;    //增加伺服角度
                }
                break;
        }

        // if (servo_angle < 0) servo_angle = 0;     //限制伺服角度最小值
        if (servo_angle > 250) servo_angle = 250;    //限制伺服角度最大值

        // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 500 + (uint32_t)(servo_angle * 2000 / 270));
        motor_test->set_angle(motor_test, target_angle);   //设置目标角度
        
        //如果摇杆信号在阈值范围内，重叠标志位
        if (!Sensitivity_Judge(rc_ctrl[0].rc.rocker_r_)) drat_flag = 0;

        //根据摇杆信号设置CAN消息数据
        if (rc_ctrl[0].rc.rocker_l1 > DRAT_RT_SENSITIVITY) {// 摇杆左1大于阈值
            drat_msg.data[0] = 0x11;
        } else if (rc_ctrl[0].rc.rocker_l1 < -DRAT_RT_SENSITIVITY) {// 摇杆左1小于负阈值
            drat_msg.data[0] = 0x22;
        } else if (rc_ctrl[0].rc.rocker_l_ < -DRAT_RT_SENSITIVITY) {// 摇杆左小于负阈值
            drat_msg.data[0] = 0x33;
        } else if (rc_ctrl[0].rc.rocker_l_ > DRAT_RT_SENSITIVITY) {// 摇杆左大于阈值
            drat_msg.data[0] = 0x44;
        } else {
            drat_msg.data[0] = 0x55;   //摇杆信号在阈值范围内
        }

        Bus_Publish(can_tx_topic, &drat_msg);//发布CAN消息

        //        vofa_send_data(0, motor_HL->real_speed);
        //        vofa_send_data(1, motor_HR->real_speed);
        //        vofa_send_data(2, motor_TL->real_speed);
        //        vofa_send_data(3, motor_TR->real_speed);
        //        vofa_sendframetail();

        //        vofa_send_data(0, motor_test->target_angle);
        //        vofa_send_data(1, motor_test->real_angle);

        // vofa_send_data(0, uart_data);
        //
        // vofa_sendframetail();

        osDelay(10);
    }
}

//灵敏度判断
static int8_t Sensitivity_Judge(int16_t value) {
    if (value < -DRAT_RT_SENSITIVITY) return -1;   //小于负阈值
    if (value > DRAT_RT_SENSITIVITY) return 1;     //大于阈值

    return 0;
}

static uint8_t count;

//串口接收回调函数
static void on_uart_rx(void *message, Bus_TopicHandleTypeDef *topic) {    //消息指针     ， 主题句柄
    INTF_UART_MessageTypeDef *msg = (INTF_UART_MessageTypeDef *)message; //转换消息类型
    memcpy(&uart_data, msg->data, sizeof(uart_data));  //复制数据到uart_data
    isReceive = true;   //设置收到数据标志位
}

//DRAT逻辑初始化函数
void Drat_Logic_Init() {

    rc_ctrl = Bus_SharePtr("DR16", sizeof(RC_ctrl_t) * 2);    //获取遥控器控制结构体共享内存
    motor_test = Bus_SharePtr("/motor/shooter_feeder", sizeof(INTF_Motor_HandleTypeDef));  //获取测试电机共享内存
    //    Bus_SubscribeFromName("/CAN1/RX", On_Can_RX);
    Bus_SubscribeFromName("/UART6/RX", on_uart_rx);   //订阅串口接收主题

    //获取四个电机共享内存
    motor_HL = Bus_SharePtr("/motor/shooter_HL", sizeof(INTF_Motor_HandleTypeDef));
    motor_HR = Bus_SharePtr("/motor/shooter_HR", sizeof(INTF_Motor_HandleTypeDef));
    motor_TL = Bus_SharePtr("/motor/shooter_TL", sizeof(INTF_Motor_HandleTypeDef));
    motor_TR = Bus_SharePtr("/motor/shooter_TR", sizeof(INTF_Motor_HandleTypeDef));
    //    Bus_SubscribeFromName("USB_RX", On_USB_RX);

    //注册USB发送主题和CAN发送主题
    usb_tx_topic = Bus_TopicRegister("USB_TX");
    can_tx_topic = Bus_TopicRegister("/CAN2/TX");

    // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_of(INTF_Motor_HandleTypeDef));
    // __HAL_TIM_SET_PRESCALER(&htim4, 84 - 1);
    // __HAL_TIM_SET_AUTORELOAD(&htim4, 20000 - 1);

    //创建任务
    osThreadDef(Drat_Logic_MainLoopTask, Drat_Logic_MainLoop, osPriorityLow, 0, 512);
    Drat_Logic_TaskHandle = osThreadCreate(osThread(Drat_Logic_MainLoopTask), NULL);
}
