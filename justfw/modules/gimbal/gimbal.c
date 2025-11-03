//
// Created by Ukua on 2024/1/22.
//

#include "gimbal.h"

osThreadId Gimbal_MainLoopTaskHandle;
INTF_Gimbal_HandleTypeDef *g_gimbal;

PIDInstance gimbal_yaw_gyro_pid;
PIDInstance gimbal_pitch_gyro_pid;

void Gimbal_SetYaw(struct INTF_Gimbal_Handle *self, float target_yaw) {
    //限制yaw轴角度
    if (self->pitch_limit_max==self->pitch_limit_min) {//未设置限制
        self->target_yaw = target_yaw;
    }else{
        if (target_yaw>self->yaw_limit_max){
            self->target_yaw = self->yaw_limit_max;
        }else if (target_yaw<self->yaw_limit_min){
            self->target_yaw = self->yaw_limit_min;
        }else{
            self->target_yaw = target_yaw;
        }
    }
}

void Gimbal_SetPitch(struct INTF_Gimbal_Handle *self, float target_pitch) {
    //限制pitch轴角度
    if (self->pitch_limit_max==self->pitch_limit_min) {//未设置限制
        self->target_pitch = target_pitch;
    }else{
        if (target_pitch>self->pitch_limit_max){
            self->target_pitch = self->pitch_limit_max;
        }else if (target_pitch<self->pitch_limit_min){
            self->target_pitch = self->pitch_limit_min;
        }else{
            self->target_pitch = target_pitch;
        }
    }
}

void Gimbal_SetMode(struct INTF_Gimbal_Handle *self,Gimbal_ModeTypeDef mode){
    extern float INS_angle[3];
    switch (mode) {
        //这边做一点处理来防止模式切换时甩头
        case GIMBAL_MODE_NORMAL:
            self->motor_yaw->set_mode(g_gimbal->motor_yaw,MOTOR_MODE_ANGLE);
            self->motor_yaw->set_mode(g_gimbal->motor_yaw,MOTOR_MODE_ANGLE);

            self->set_yaw(self,self->real_yaw);
            self->set_pitch(self,self->real_pitch);
            break;
        case GIMBAL_MODE_FOLLOW_GYRO:
            self->motor_yaw->set_mode(g_gimbal->motor_yaw,MOTOR_MODE_SPEED);
            self->motor_pitch->set_mode(g_gimbal->motor_pitch,MOTOR_MODE_SPEED);

            self->set_yaw(self,INS_angle[2]);
            self->set_pitch(self,INS_angle[0]);
            break;
    }
    self->mode = mode;
}

float test;

void Gimbal_MainLoop() {
    osDelay(5000); //等待电机,陀螺仪初始化


    //shit:
    g_gimbal->set_mode(g_gimbal,GIMBAL_MODE_FOLLOW_GYRO);
//    g_gimbal->target_yaw=0;
//    g_gimbal->target_pitch=0;

    while (1) {
        extern float INS_angle[3];
        switch (g_gimbal->mode) {
            case GIMBAL_MODE_NORMAL:
                g_gimbal->motor_yaw->set_angle(g_gimbal->motor_yaw,g_gimbal->target_yaw);
                break;
            case GIMBAL_MODE_FOLLOW_GYRO:
                g_gimbal->motor_yaw->set_speed(g_gimbal->motor_yaw, PIDCalculate(&gimbal_yaw_gyro_pid,-loop_float_constrain(g_gimbal->target_yaw-INS_angle[2],-PI,PI),0));
                g_gimbal->motor_pitch->set_speed(g_gimbal->motor_pitch, PIDCalculate(&gimbal_pitch_gyro_pid,-loop_float_constrain(g_gimbal->target_pitch-INS_angle[2],-PI,PI),0));
                break;
        }

        char a[50];
        sprintf(a,"%f,%f\n",g_gimbal->real_yaw,g_gimbal->target_yaw);
        LOG(a)
        g_gimbal->real_pitch = g_gimbal->motor_pitch->real_angle;
        g_gimbal->real_yaw = g_gimbal->motor_yaw->real_angle;

        osDelay(2);
    }
}



void Gimbal_Init() {
    g_gimbal = Bus_SharePtr("gimbal", sizeof(INTF_Gimbal_HandleTypeDef ));

    g_gimbal->mode=GIMBAL_MODE_NORMAL;

    g_gimbal->motor_yaw = Bus_SharePtr("/motor/gimbal_yaw", sizeof(INTF_Motor_HandleTypeDef));
    g_gimbal->motor_pitch = Bus_SharePtr("/motor/gimbal_pitch", sizeof(INTF_Motor_HandleTypeDef));

    g_gimbal->set_pitch = Gimbal_SetPitch;
    g_gimbal->set_yaw = Gimbal_SetYaw;
    g_gimbal->set_mode = Gimbal_SetMode;

    g_gimbal->target_yaw=0;
    g_gimbal->target_pitch=0;

    g_gimbal->pitch_limit_max = 0;
    g_gimbal->pitch_limit_min = 0;
    g_gimbal->yaw_limit_max = 0;
    g_gimbal->yaw_limit_min = 0;

    PID_Init_Config_s gimbal_yaw_angle_config = {
            .Kp=3.7f,
            .Ki=0.00005f,
            .Kd=0.0008f,
            .IntegralLimit=0.09f,
            .MaxOut=10.0f * RPM2RPS,
            .DeadBand = 0.0f,
            .Improve=PID_Integral_Limit,
    };
    PIDInit(&gimbal_yaw_gyro_pid,&gimbal_yaw_angle_config);

//    PID_Init_Config_s gimbal_yaw_speed_config = {
//            .Kp=60.0f,
//            .Ki=0.0f,
//            .Kd=12.0f,
//            .MaxOut=10.0f * RPM2RPS,
//            .DeadBand = 0.0f,
//            .Improve=PID_OutputFilter|PID_Integral_Limit,//|PID_Derivative_On_Measurement,
//            .IntegralLimit=6.0f,
//    };
//    PIDInit(&gimbal_yaw_gyro_pid,&gimbal_yaw_speed_config);

    PID_Init_Config_s gimbal_pitch_config = {
            .Kp=12.0f,
            .Ki=0.0f,
            .Kd=1.0f,
            .MaxOut=5.0f * RPM2RPS,
            .DeadBand = 0.0f,
            .Improve=PID_Integral_Limit,//|PID_Derivative_On_Measurement,
            .IntegralLimit=1.0f,
    };
    PIDInit(&gimbal_pitch_gyro_pid,&gimbal_pitch_config);

    osThreadDef(Gimbal_MainLoopTask, Gimbal_MainLoop, osPriorityLow, 0, 256);
    Gimbal_MainLoopTaskHandle = osThreadCreate(osThread(Gimbal_MainLoopTask), NULL);
}
