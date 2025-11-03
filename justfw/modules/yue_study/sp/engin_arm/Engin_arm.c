
#include "Engin_arm.h"
#include "math.h"
#include "BSP_CAN.h"
#include "NRF24L01/NRF24L01.h"
#include "can.h"
#include "usart.h"
#include "BSP_UART.h"
#include "C610.h"


#define SP_RT_SENSITIVITY       200
#define MAX_SPEED               160.0f            //按扭矩给
#define MAX_angle               800.0f
#define MIN_angle               0.0f
#define ROCKER_RANGE            660.0f
#define MAX_FACTOR              300.0f           //任意，无具体含义
#define DAEDZONE                200
#define DAEDZONE0               150
#define offset_angle            (49.0f/180.0f) * PI
#define left                  65535
#define right                 65534


uint8_t a=0;
uint16_t ADCRX_Data[6];
float diff [1]={0};

osThreadId Engin_Arm_TaskHandle;

static RC_ctrl_t *rc_ctrl;
static INTF_Motor_HandleTypeDef *motor_2006_1;
static INTF_Motor_HandleTypeDef *motor_2006_2;
static INTF_Motor_HandleTypeDef *motor_6220;
INTF_Motor_HandleTypeDef *arm_l_86_motor;
INTF_Motor_HandleTypeDef *arm_r_86_motor;
static INTF_Motor_HandleTypeDef *arm_yaw_3508_motor;
// static INTF_Motor_HandleTypeDef *arm_up_3508_motor;


static void Control_Last_Movement();
static void Control_8006_l(float a);
static void Control_8006_r(float a);
static void Control_3508_yaw();
static void Control_6220(float a);
static float DeadZone_Process(int16_t input);   // 摇杆死区处理函数   ai生成的，逻辑不用管直接用就行。
static float DeadZone_Process_0(int16_t input);  //死区处理函数（第二个）

//解包，解出含六个adc数值的数组
void unpackADC6(uint8_t *buf, uint16_t adc_vals[6])
{
    for (int i = 0; i < 6; i++) {
        int a = ((uint16_t)buf[2*i] << 8) | buf[2*i + 1];
        if (a >= 8500) return ;
        adc_vals[i] = ((uint16_t)buf[2*i] << 8) | buf[2*i + 1];
    }
}



///adc数值转化为角度值
float ADC_To_Angle(uint16_t adc_value,
                   uint16_t adc_min, uint16_t adc_max,
                   float angle_min, float angle_max)
{
    // 防止除零
    if (adc_max == adc_min) {
        return angle_min;
    }

    // 限制输入到范围内
    // if (adc_value < adc_min) adc_value = adc_min;
    // if (adc_value > adc_max) adc_value = adc_max;

    // 比例映射
    float ratio = (float)(adc_value - adc_min) / (float)(adc_max - adc_min);
    return angle_min + ratio * (angle_max - angle_min);
}




static void Engin_Arm_MainLoop(){    //大臂用的逻辑基本都是角度累加
     //osDelay(1000);
     //arm_r_86_motor->target_angle = arm_r_86_motor->real_angle;

    uint8_t current1 = C610_Torque2Current(motor_2006_1->real_torque);

    // arm_r_86_motor->target_angle = 0.762046;
    while (1)
    {

        NRF24L01_Receive();
        unpackADC6(NRF24L01_RxPacket,ADCRX_Data);

        // arm_yaw_3508_motor->target_angle = a*1.0f/4096.0f*6.4f;
        // USB_printf("%d,%d,%d,%d,%d,%d\n",ADCRX_Data[0],ADCRX_Data[1],ADCRX_Data[2],ADCRX_Data[3],ADCRX_Data[4],ADCRX_Data[5]);
        USB_printf("3508:%f\n",arm_yaw_3508_motor->real_angle);
        // // osDelay(1);
        //
        //
        USB_printf("lmotor:%f\n",arm_l_86_motor->real_angle);
        USB_printf("rmotor:%f\n",arm_r_86_motor->real_angle);

        USB_printf("6220motor:%f\n",motor_6220->real_angle);

        // USB_printf("2006current:%f\n",current1);
        // USB_printf("2006torque:%f\n",motor_2006_1->real_torque);






        arm_yaw_3508_motor -> target_angle = ADC_To_Angle(ADCRX_Data[0],2815,1650,-1.28182,1.28182);
        // USB_printf("3508:%f\n",arm_yaw_3508_motor->target_angle);
        // USB_printf("%d,%d,%d,%d,%d,%d\n",ADCRX_Data[0],ADCRX_Data[1],ADCRX_Data[2],ADCRX_Data[3],ADCRX_Data[4],ADCRX_Data[5]);
        // arm_yaw_3508_motor->set_angle(arm_yaw_3508_motor,ADC_To_Angle(ADCRX_Data[0],2815,1650,0.616008,3.323191));



        arm_l_86_motor->target_angle = ADC_To_Angle(ADCRX_Data[1],3452,1052,0.747932,-2.040273);
        // USB_printf("l-86:%f\n",arm_l_86_motor->target_angle);
        // arm_l_86_motor->set_angle(arm_l_86_motor,arm_l_86_motor->real_angle);

        arm_r_86_motor->target_angle = ADC_To_Angle(ADCRX_Data[2],7300,2387,-1.273125,-7.161963);
        // USB_printf("r-86:%f\n",arm_r_86_motor->target_angle);
        // arm_r_86_motor->set_angle(arm_r_86_motor,arm_r_86_motor->real_angle);

        motor_6220->set_angle(motor_6220,ADC_To_Angle(ADCRX_Data[3],0,4090,-2,2))  ;
        // motor_6220->set_angle(motor_6220,motor_6220->real_angle);

        motor_2006_1->target_angle =  ( ADC_To_Angle(ADCRX_Data[4],1680,4096,82.0,-88.0) + ADC_To_Angle(ADCRX_Data[5],0,4095,-160.0,160.0) );

        motor_2006_2->target_angle = ( ADC_To_Angle(ADCRX_Data[4],1680,4096,-82.0,88.0) - ADC_To_Angle(ADCRX_Data[5],0,4095,160.0,-160.0) );


        // arm_yaw_3508_motor->set_speed(arm_yaw_3508_motor,10.0f);
        // Control_8006_r(left);
        //
        // HAL_UARTEx_ReceiveToIdle_DMA(&huart5,ADCreceiveData,sizeof(ADCreceiveData));
        //
        // __HAL_DMA_DISABLE_IT(&hdma_usart5_rx, DMA_IT_HT);
        // uint16_t ADvalue=unpack_Date(ADCreceiveData);

        // arm_yaw_3508_motor->set_angle(arm_yaw_3508_motor,ADvalue*1.0f/4096.0f*6.4f);
        // arm_yaw_3508_motor->real_angle=0.0f;

        // USB_pridntf("ata:%.2f,%.2f,%.2f,%.2f\n",arm_yaw_3508_motor->target_angle,arm_yaw_3508_motor->real_angle,arm_up_3508_motor->target_angle,arm_up_3508_motor->real_angle);

        
        //左摇杆
        // if (rc_ctrl->rc.switch_left==RC_SW_UP)   //左手控小头//右手x控6220row转手，y肘8006pitch抬升。
        // {
        //       Control_Last_Movement();
        //
        // }
        // else if(rc_ctrl->rc.switch_left==RC_SW_MID)   //左手x控6220row，y肘8006pitch //右手x控3508，y臂8006抬升
        // {
        //     Control_6220(left);
        //     Control_8006_r(left);
        //
        //
        //
        // }
        //
        // //右摇杆
        // if (rc_ctrl->rc.switch_right==RC_SW_UP){
        //
        //      Control_6220(right);
        //      Control_8006_r(right);
        // }
        // else if (rc_ctrl->rc.switch_right==RC_SW_MID){
        //     Control_3508_yaw();
        //
        //      Control_8006_l(right);
        // }
        // // USB_printf("data:%.2f,%.2f,%.2f,%.2f,%.2f\n",arm_r_86_motor->real_angle,arm_r_86_motor->angle_offset,arm_r_86_motor->target_angle,arm_r_86_motor->target_torque,arm_r_86_motor->real_torque);
        // // USB_printf("data:%.2f,%.2f,%.2f,%.2f,%.2f\n",arm_l_86_motor->real_angle,arm_l_86_motor->angle_offset,arm_l_86_motor->target_angle,arm_l_86_motor->target_torque,arm_l_86_motor->real_torque);
        // // extern UART_HandleTypeDef huart5;
        // // HAL_UART_AbortReceive(&huart5);
        // // uint8_t state = HAL_UART_Receive(&huart5, adcrx, 10, 0xff);
        // // if (state)
        // // {
        // //     osDelay(10);
        // // }
        //
        // osDelay(1);
    }
}

void Engin_Arm_Init(){

    rc_ctrl = Bus_SharePtr("DR16", sizeof(RC_ctrl_t));
    NRF24L01_Init(); // 初始化NRF24L01

    motor_2006_1 = Bus_SharePtr("/motor/big_arm_1", sizeof(INTF_Motor_HandleTypeDef));
    motor_2006_2 = Bus_SharePtr("/motor/big_arm_2", sizeof(INTF_Motor_HandleTypeDef));
    motor_6220 = Bus_SharePtr("/motor/arm_6220", sizeof(INTF_Motor_HandleTypeDef));
     arm_l_86_motor = Bus_SharePtr("/motor/arm_l_86", sizeof(INTF_Motor_HandleTypeDef));
    arm_r_86_motor = Bus_SharePtr("/motor/arm_r_86", sizeof(INTF_Motor_HandleTypeDef));
    arm_yaw_3508_motor = Bus_SharePtr("/motor/big_arm_yaw", sizeof(INTF_Motor_HandleTypeDef));


    // arm_up_3508_motor = Bus_SharePtr("/motor/big_arm_up",sizeof(INTF_Motor_HandleTypeDef)) ;

    osThreadDef(Engin_Arm_MainLoopTask, Engin_Arm_MainLoop, osPriorityLow, 0, 512);
    Engin_Arm_TaskHandle = osThreadCreate(osThread(Engin_Arm_MainLoopTask), NULL);
}


//大臂电机俯瞰图
// // 2006      2006                        1     2
// //
// //      6220                             3(row)
// //
// //
// //
// // 8006      8006                    4（pitch）5(pitch)
// //
// // 
// //   3508                                6（yaw）
// //
// //
// //
// //
// //       3508                            7（抬升）如果有的话


//2025.9.2   yue_0017
//大臂的布线还是挺好的，我为此改过好几根线的长度。线也都是新的，没怎么用工程的老线
//最前端的row左转右转会有点问题，一边顺一边不顺，可能是机械上的问题
//2006上俩电调，有一个电调的3相线的接口脱落了，我重新焊过但是不是很牢稳，前端那个扎带就是绑他的。
//6220有时候会断信号，不知道是CAN线的问题还是接口的问题还是板子的问题，我换成另一个接口了，现在还好，如果再出现信号不良建议换板子。
//两个8006大概工作10分钟后就休息，后面可能会发热严重，不建议长时间不停的使用
//3508算是比较放心的，但是似乎有点超调的意思
//8006的劲其实是比较大的，我不小心被他伤到过好多次，调的时候当心，感觉不对劲立马关电源