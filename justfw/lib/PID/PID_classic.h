//
// Created by Ukua on 2023/9/20.
//

#ifndef JUSTFW_PID_CLASSIC_H
#define JUSTFW_PID_CLASSIC_H

typedef struct PID_classic{
    float kp , ki, kd;             //pid系数
    float max_i ;                  //积分最大值,设置为0以关闭
    float target;                  //pid目标值
    float _last_error;             //上次误差，用于计算微分，不应该被手动设置
    float _xi;                     //积分累计值,用于计算积分，不应该被手动设置
} PID_HandleTypeDef;



/**
* @brief  初始化一个pid结构体
* @param  *pid 要设置的pid结构体
*/
void PID_Init(PID_HandleTypeDef *pid,float kp,float ki,float kd,float max_i,float target);


/**
* @brief  设置pid的系统目标值
* @param  *pid 要设置的pid结构体
* @param  target 要设置的系统目标值
*/
void PID_Set_Target(PID_HandleTypeDef *pid,float target);


/**
* @brief  使用目标值和当前值更新pid
* @param  *pid 计算使用的pid结构体
* @param  current 系统当前值
* @param  time 从上次更新至此次更新的间隔时间
* @return pid计算结果
*/
float PID_Update(PID_HandleTypeDef *pid,float current,float time);


/**
  * @brief  使用指定的偏差值更新pid
  * @param  *pid 计算使用的pid结构体
  * @param  error 系统当前误差
  * @param  current 从上次更新至此次更新的间隔时间
  * @return pid计算结果
  */
float PID_Update_Error(PID_HandleTypeDef *pid,float error,float time);



#endif //JUSTFW_PID_CLASSIC_H
