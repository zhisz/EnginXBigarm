
//被弃用









////
//// Created by Ukua on 2024/2/23.
////
//
//#include "launcher.h"
//
//osThreadId Launcher_MainLoopTaskHandle;
//#include "main.h"
//#include "tim.h"
//#include "DR16.h"
//extern RC_ctrl_t rc_ctrl[2];
//
//float speed = 60.0f;
//
//int last_sw_r = 0;
//
////(500-2000)
//float counter = 1720;
//
//INTF_Motor_HandleTypeDef *motor_left_f;
//INTF_Motor_HandleTypeDef *motor_left_b;
//INTF_Motor_HandleTypeDef *motor_right_f;
//INTF_Motor_HandleTypeDef *motor_right_b;
//
//void Launcher_MainLoop() {
//    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
//    while (1){
//        if(rc_ctrl[0].rc.rocker_l_>0){
//            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET);
//            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
//            osDelay(2);
//            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
//        } else if(rc_ctrl[0].rc.rocker_l_<0){
//            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
//            osDelay(2);
//            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
//        }
//        if(rc_ctrl[0].rc.rocker_l1>0){
//            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
//            HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_SET);
//            osDelay(2);
//            HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_RESET);
//        } else if(rc_ctrl[0].rc.rocker_l1<0){
//            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_SET);
//            osDelay(2);
//            HAL_GPIO_WritePin(GPIOI,GPIO_PIN_6,GPIO_PIN_RESET);
//        }
//
//
//        if(rc_ctrl[0].rc.switch_left==1) {
//
//            motor_left_f->set_speed(motor_left_f, -2.0f*M_PI*speed);
//            motor_right_f->set_speed(motor_right_f, 2.0f*M_PI*speed);
//            motor_left_b->set_speed(motor_left_b, -2.0f*M_PI*speed);
//            motor_right_b->set_speed(motor_right_b, 2.0f*M_PI*speed);
//        }else{
//            motor_left_f->set_speed(motor_left_f, 0);
//            motor_right_f->set_speed(motor_right_f, 0);
//            motor_left_b->set_speed(motor_left_b, 0);
//            motor_right_b->set_speed(motor_right_b, 0);
//        }
//
//        if(rc_ctrl[0].rc.rocker_r1>0){
//            counter -= 2;
//        }else if(rc_ctrl[0].rc.rocker_r1<0){
//            counter += 2;
//        }
//
//        if(counter>1720){
//            counter = 1720;
//        }else if(counter<1000){
//            counter = 1000;
//        }
//
//        if(last_sw_r == 3 && rc_ctrl[0].rc.switch_right == 1){
//            speed+=1;
//        }
//        if(last_sw_r == 3 && rc_ctrl[0].rc.switch_right == 2){
//            speed-=1;
//        }
//
//        last_sw_r = rc_ctrl[0].rc.switch_right;
//        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,counter);
//        osDelay(2);
//    }
//}
//
//void Launcher_Init() {
//    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
//
//    motor_left_f = Bus_SharePtr("/motor/shooter_left_f", sizeof(INTF_Motor_HandleTypeDef));
//    motor_left_b = Bus_SharePtr("/motor/shooter_left_b", sizeof(INTF_Motor_HandleTypeDef));
//    motor_right_f = Bus_SharePtr("/motor/shooter_right_f", sizeof(INTF_Motor_HandleTypeDef));
//    motor_right_b = Bus_SharePtr("/motor/shooter_right_b", sizeof(INTF_Motor_HandleTypeDef));
//
//    osThreadDef(Launcher_MainLoopTask, Launcher_MainLoop, osPriorityLow, 0, 256);
//    Launcher_MainLoopTaskHandle = osThreadCreate(osThread(Launcher_MainLoopTask), NULL);
//}