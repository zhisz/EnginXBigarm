//
// Created by Ukua on 2023/9/20.
//

#include "PID_classic.h"

/**
  * @brief  初始化一个pid结构体
  * @param  *p 要进行初始化的结构体
  * @param  kp 要设置的比例系数
  * @param  ki 要设置的积分系数
  * @param  kd 要设置的微分系数
  * @param  max_i 要设置的最大积分值（设置为0以禁用)
  * @param  target 要设置的系统目标值
  */
void PID_Init(PID_HandleTypeDef *pid,float kp,float ki,float kd,float max_i,float target){
    pid->kp=kp;
    pid->ki=ki;
    pid->kd=kd;
    pid->max_i=max_i;
    pid->target=target;

    pid->_xi=0;
    pid->_last_error=0;
}


/**
  * @brief  设置pid的系统目标值
  * @param  *p 要设置的结构体
  * @param  target 要设置的系统目标值
  */
void PID_Set_Target(PID_HandleTypeDef *pid,float target){
    pid->target=target;
}

/**
  * @brief  使用目标值和当前值更新pid
  * @param  *p 计算使用的pid结构体
  * @param  current 系统当前值
  * @retval pid计算结果
  */
float PID_Update(PID_HandleTypeDef *pid,float current,float time){
    float error = pid->target - current;

    return PID_Update_Error(pid,error,time);
}


/**
  * @brief  使用指定的偏差值更新pid
  * @param  *p 计算使用的pid结构体
  * @param  error 系统当前误差
  * @retval pid计算结果
  */
float PID_Update_Error(PID_HandleTypeDef *pid,float error,float time){
    pid->_xi += error*pid->ki*time;

    if(pid->max_i != 0){
        if(pid->_xi > pid->max_i){
            pid->_xi = pid->max_i;
        }else if(pid->_xi < -pid->max_i){
            pid->_xi = -pid->max_i;
        }
    }

    float ans = pid->kp*error + pid->_xi+ (error-pid->_last_error)*pid->kd/time;

    pid->_last_error = error;
    return ans;
}