//现在可以上车的二代代码  在这个框架算是最原始的代码

// #include "sp_logic.h"
// #include "special_foot.h"
// #include "math.h"

// #define SP_RT_SENSITIVITY       200
// #define MAX_SPEED               5.0f
// #define MAX_angle               800.0f
// #define MIN_angle               0.0f
// #define ROCKER_RANGE            660.0f
// #define MAX_FACTOR              3.0f
// #define DAEDZONE                100

//单位转换

// //运动模式枚举
// typedef enum{
//     MODE_COMBAT,      //对抗模式
//     MODE_RACE,        //竞速模式
//     MODE_REVERSE,   //反向
// } OperationMode;

// //运动方向枚举
// typedef enum{
//     DIR_FORWARD,
//     DIR_BACKWARD,
//     DIR_LEFT,
//     DIR_RIGHT,
// } MoveDirection;

// //电机控制参数结构体
// typedef struct{
//     float left_front;
//     float left_back;

// } LegLeft;
// typedef struct{
   
//     float right_front;
//     float right_back;
// } LegRight;


// uint32_t last_tick = 0;

// osThreadId SP_Logic_TaskHandle;

// static RC_ctrl_t *rc_ctrl;
// static INTF_Motor_HandleTypeDef *leglf_motor;
// static INTF_Motor_HandleTypeDef *leglb_motor;
// static INTF_Motor_HandleTypeDef *legrb_motor;
// static INTF_Motor_HandleTypeDef *legrf_motor;
// static INTF_Motor_HandleTypeDef *spade_motor;
// static OperationMode current_mode = MODE_COMBAT;

// static int8_t SP_flag = 0; 
// static void Control_LegMovement();
// static void Control_Spade();
// // static LegLeft CalcLeftSpeeds(float speed);
// // static LegRight CalcRightSpeeds(float speed);
// // static int8_t Sensitivity_Judge(int16_t value); // 摇杆灵敏度判断函数
// static float DeadZone_Process(int16_t input);   // 摇杆死区处理函数

// static void SP_Logic_MainLoop(){
//     while (1)
//     {
//         // if(rc_ctrl == NULL || leglf_motor == NULL || leglb_motor == NULL || legrb_motor == NULL || legrf_motor == NULL || spade_motor == NULL){
//         //          osDelay(100);
//         //     continue;
//         // }

//         if (rc_ctrl->rc.switch_left == RC_SW_UP) {
//             current_mode = MODE_RACE;
//         } 
//         else if (rc_ctrl->rc.switch_left == RC_SW_DOWN) {
//             current_mode = MODE_COMBAT;
//         } 
//         else if (rc_ctrl->rc.switch_left == RC_SW_MID){
//             current_mode = MODE_REVERSE;
//         } 

//         Control_LegMovement();
//         Control_Spade();

//         osDelay(5);
//     }
// }

// static void Control_LegMovement(){
    
    
//     float rc_ctrl_ly = DeadZone_Process(rc_ctrl->rc.rocker_l1);
//     float rc_ctrl_lx = DeadZone_Process(rc_ctrl->rc.rocker_l_);
//     // float rc_ctrl_ly = rc_ctrl->rc.rocker_l1;
//     // float rc_ctrl_lx = rc_ctrl->rc.rocker_l_;
 
 
//     //float max_speed = Get_MaxSpeed();
//     float base_speed = (rc_ctrl_ly/ROCKER_RANGE) * MAX_SPEED;
//     float turn_factor = (rc_ctrl_lx/ROCKER_RANGE) * MAX_FACTOR;
    
//     float L = base_speed + turn_factor;
//     float R = base_speed - turn_factor;

//     LegLeft spleft = {L,L};
//     LegRight spright = {R,R};

//     //float speed = speed + turn_factor;
//     //运动方向判定
//     // MoveDirection dir = DIR_FORWARD;
//     // const int8_t dir_h = Sensitivity_Judge(rc_ctrl_lx);
//     // if(dir_h == 0) dir = DIR_FORWARD;
//     // else if (dir_h == 1)       dir = DIR_RIGHT;
//     // else if (dir_h == -1) dir = DIR_LEFT;
//     // else if (rc_ctrl_ly < 0) dir = DIR_BACKWARD;
   
    

//     //生成速度配置
//     // LegLeft target_speed = CalcLeftSpeeds(base_speed);
//     // LegRight target_speed = CalcRightSpeeds(base_speed);
//     // LegSpeeds target_speed = base_speed * 2;

//     //设置电机速度
// //    leglf_motor->set_speed(leglf_motor,target_speed.left_front);
//     leglf_motor->set_speed(leglf_motor,spleft.left_front);
//     leglb_motor->set_speed(leglb_motor,spleft.left_back);
//     legrf_motor->set_speed(legrf_motor,spright.right_front);
//     legrb_motor->set_speed(legrb_motor,spright.right_back);

    
// }

// static void Control_Spade(){
//     //float dt = DWT_GetDeltaT(&last_tick);
//     //float rc_ctrl_ry = rc_ctrl->rc.switch_right;
//     //float target_angle = (rc_ctrl_ry / ROCKER_RANGE) * MAX_angle;
//     // target_angle = fmaxf(MIN_angle, fminf(target_angle, MAX_angle));
    
//     //spade_motor->set_angle(spade_motor, target_angle);
//     // spade_motor->set_angle(spade_motor,20.0f*PI);
//     float get_angle = MAX_angle * M_PI / 180.0f;
//     //spade_motor->set_angle(spade_motor,get_angle);
//     if(rc_ctrl->rc.switch_right == RC_SW_UP){
//         spade_motor->set_angle(spade_motor,get_angle);
//     }
//     if(rc_ctrl->rc.switch_right == RC_SW_MID){
//         spade_motor->set_angle(spade_motor,MIN_angle);
//     }
    
// }

// // static LegLeft CalcLeftSpeeds(float speed){
// //     LegLeft sp = {0};
// //     float turn_factor = 0.6f;//转向系数（0.6表示转向时内外轮速差40%）

// //     if(current_mode == MODE_COMBAT){
// //             switch (dir){
// //                 case DIR_FORWARD:
// //                     sp = (LegSpeeds){speed,speed,speed,speed};
// //                     break;
// //                 case DIR_BACKWARD: // 后退
// //                     sp = (LegSpeeds){-speed,-speed,-speed,-speed};
// //                     break;
// //                 case DIR_LEFT:
// //                     sp = (LegSpeeds){-speed,-speed,speed,speed};
// //                     break;
// //                 case DIR_RIGHT:
// //                     sp = (LegSpeeds){speed,speed,-speed,-speed};
// //             }
// //         }
// //         if(current_mode == MODE_RACE){
// //             switch(dir) {
// //                 case DIR_FORWARD:  // 直行
// //                     sp = (LegSpeeds){speed,speed,speed,speed};
// //                     break;
// //                 case DIR_BACKWARD: // 后退
// //                     sp = (LegSpeeds){-speed,-speed,-speed,-speed};
// //                     break;
// //                 case DIR_LEFT:     // 左转
// //                     sp.left_front  = speed * (1 - turn_factor);
// //                     sp.left_back   = speed * (1 - turn_factor);
// //                     sp.right_front = speed * (1 + turn_factor); 
// //                     sp.right_back  = speed * (1 + turn_factor);
// //                     break; 
// //                 case DIR_RIGHT:    // 右转
// //                     sp.left_front  = speed * (1 + turn_factor);
// //                      sp.left_back   = speed * (1 + turn_factor);
// //                      sp.right_front = speed * (1 - turn_factor);
// //                      sp.right_back  = speed * (1 - turn_factor);
// //                     break;  
// //             }
// //         }
// //         if (current_mode == MODE_REVERSE) {
// //             switch(dir) {
// //                 case DIR_FORWARD:  // 直行
// //                     sp = (LegSpeeds){-speed,-speed,-speed,-speed};
// //                     break;
// //                 case DIR_BACKWARD: // 后退
// //                     sp = (LegSpeeds){speed,speed,speed,speed};
// //                     break;
// //                 case DIR_LEFT:     // 左转
// //                     sp.left_front  = -speed * (1 - turn_factor);
// //                     sp.left_back   = -speed * (1 - turn_factor);
// //                     sp.right_front = -speed * (1 + turn_factor);
// //                     sp.right_back  = -speed * (1 + turn_factor);
// //                     break; 
// //                 case DIR_RIGHT:    // 右转
// //                     sp.left_front  = -speed * (1 + turn_factor);
// //                      sp.left_back   = -speed * (1 + turn_factor);
// //                      sp.right_front = -speed * (1 - turn_factor);
// //                      sp.right_back  = -speed * (1 - turn_factor);
// //                     break;
// //             }
// //         }            
// //     return sp;
// // } 

// static float DeadZone_Process(int16_t input){
//     if (abs(input)< DAEDZONE) return 0.0f;
//     return (input > 0) ? (input - DAEDZONE) : (input + DAEDZONE);
// }

// // static int8_t Sensitivity_Judge(int16_t value){
// //     if (value > SP_RT_SENSITIVITY) return 1;     //大于阈值
// //     if (value < -SP_RT_SENSITIVITY) return -1;  //小于负阈值

// //     return 0;
// // }


// void SP_logic_Init(){
//     rc_ctrl = Bus_SharePtr("DR16", sizeof(RC_ctrl_t));
//     leglf_motor = Bus_SharePtr("/motor/leg-l-f", sizeof(INTF_Motor_HandleTypeDef));
//     leglb_motor = Bus_SharePtr("/motor/leg-l-b", sizeof(INTF_Motor_HandleTypeDef));
//     legrf_motor = Bus_SharePtr("/motor/leg-r-f", sizeof(INTF_Motor_HandleTypeDef));
//     legrb_motor = Bus_SharePtr("/motor/leg-r-b", sizeof(INTF_Motor_HandleTypeDef));
//     spade_motor = Bus_SharePtr("/motor/spade", sizeof(INTF_Motor_HandleTypeDef));

//     osThreadDef(SP_Logic_MainLoopTask, SP_Logic_MainLoop, osPriorityLow, 0, 512);
//     SP_Logic_TaskHandle = osThreadCreate(osThread(SP_Logic_MainLoopTask), NULL);
// }
