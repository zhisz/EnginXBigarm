// //人为优化版 /结果到最后只是写了这么一点点东西啊……   
// //速度形态

// #include "sp_logic.h"
// #include "special_foot.h"
// #include "math.h"

// #define SP_RT_SENSITIVITY       200
// #define MAX_SPEED_0             19.0f
// #define MAX_angle               800.0f
// #define MIN_angle               0.0f
// #define ROCKER_RANGE            660.0f
// #define MAX_FACTOR_O            8.0f
// #define DAEDZONE                90
// #define DAEDZONEONE             200


// #define SPADE_POS_LIMIT_MAX  60.0f  // 铲子最大允许位置（根据实际校准）
// #define SPADE_POS_LIMIT_MIN  -60.0f  // 铲子最小允许位置

// float MAX_FACTOR;
// float MAX_SPEED;

// //电机控制参数结构体
// typedef struct{
//     float left_front;
//     float left_back;
// } LegLeft;
// typedef struct{
//     float right_front;
//     float right_back;
// } LegRight;

// osThreadId SP_Logic_TaskHandle;

// static RC_ctrl_t *rc_ctrl;
// static INTF_Motor_HandleTypeDef *leglf_motor;
// static INTF_Motor_HandleTypeDef *leglb_motor;
// static INTF_Motor_HandleTypeDef *legrb_motor;
// static INTF_Motor_HandleTypeDef *legrf_motor;
// static INTF_Motor_HandleTypeDef *spade_motor;
// static void Control_LegMovement();
// static void Control_Spade();
// static float DeadZone_Process(int16_t input);   // 摇杆死区处理函数
// static float DeadZone_Process_1(int16_t input); //低敏的

// static void SP_Logic_MainLoop(){
//     while (1)
//     {
//         // if(rc_ctrl == NULL || leglf_motor == NULL || leglb_motor == NULL || legrb_motor == NULL || legrf_motor == NULL || spade_motor == NULL){
//         //          osDelay(100);
//         //     continue;
//         // }

//         if (rc_ctrl->rc.switch_left == RC_SW_UP) {          //竞速（上）
//             MAX_FACTOR = MAX_FACTOR_O * 1.5;
//             MAX_SPEED = MAX_SPEED_0;
//         } 
//         else if (rc_ctrl->rc.switch_left == RC_SW_DOWN) {   //对抗（下）
//             MAX_FACTOR = MAX_FACTOR_O;
//             MAX_SPEED = MAX_SPEED_0;
//         } 
//         else if (rc_ctrl->rc.switch_left == RC_SW_MID){     //反向（中）
//             MAX_FACTOR = -MAX_FACTOR_O;
//             MAX_SPEED = -MAX_SPEED_0;
//         }


//         Control_LegMovement();
//         Control_Spade();

//         osDelay(5);
//     }
// }

// static void Control_LegMovement(){
    
//     float rc_ctrl_ly = DeadZone_Process_1(rc_ctrl->rc.rocker_l1);      //左，前后             //高敏
//     float rc_ctrl_lx = DeadZone_Process(rc_ctrl->rc.rocker_l_);      //左，左右（优先级低）  //低敏
//     float rc_ctrl_rx = DeadZone_Process(rc_ctrl->rc.rocker_r_);      //右，左右             //低敏

//     float base_speed = (rc_ctrl_ly/ROCKER_RANGE) * MAX_SPEED;
//     float turn_factor;
//         if(rc_ctrl->rc.rocker_r_ == 0){
//             turn_factor = (rc_ctrl_lx/ROCKER_RANGE) * MAX_FACTOR;          //转弯右手优先级高;
//         }
//         else{
//             turn_factor = (rc_ctrl_rx/ROCKER_RANGE) * MAX_FACTOR;
//         }
    
//     float L;
//         if (base_speed - turn_factor > MAX_SPEED) L = MAX_SPEED;
//         else if(base_speed - turn_factor < -MAX_SPEED) L = -MAX_SPEED;
//         else L =base_speed - turn_factor;
//     float R;
//     if (base_speed + turn_factor > MAX_SPEED) R = MAX_SPEED;
//         else if(base_speed + turn_factor < -MAX_SPEED) R = -MAX_SPEED;
//         else R =base_speed + turn_factor;

//     LegLeft spleft = {L,L};
//     LegRight spright = {R,R};

//     USB_printf("%.2f\n",leglf_motor->real_angle);
//     USB_printf("%.2f\n",leglb_motor->real_angle);
//     USB_printf("%.2f\n",legrf_motor->real_angle);
//     USB_printf("%.2f\n",legrb_motor->real_angle);

//     //设置电机速度
//     leglf_motor->set_speed(leglf_motor,spleft.left_front);
//     leglb_motor->set_speed(leglb_motor,spleft.left_back);
//     legrf_motor->set_speed(legrf_motor,spright.right_front);
//     legrb_motor->set_speed(legrb_motor,spright.right_back);  
// }

// static void Control_Spade(){
//     float rc_ctrl_ry = DeadZone_Process(rc_ctrl->rc.rocker_r1);
//     //float angle =spade_motor->real_angle+ (rc_ctrl_ry/ROCKER_RANGE) * 10.0f;
//     float speed = (rc_ctrl_ry/ROCKER_RANGE) * 19.0f;
//     //spade_motor->position += spade_motor->real_speed * 0.05;
//     //float current_pos = spade_motor->real_torque;
//     // if (rc_ctrl->rc.switch_right == RC_SW_DOWN){
//     //     spade_motor->position == 0.0f;
//     // }

//     // // 新增：动态限位逻辑
//     // if (current_pos >= SPADE_POS_LIMIT_MAX) {
//     //     current_pos = SPADE_POS_LIMIT_MAX;
//     //     // // 到达上限位时，仅允许负向速度（向下运动）
//     //     // if (speed > 0) {  // 试图继续向上运动
//     //     //     speed = 0;    // 强制归零
//     //     //     // 可选：触发警报/日志
//     //     // }
//     // } 
//     // if (current_pos <= SPADE_POS_LIMIT_MIN) {
//     //     current_pos = SPADE_POS_LIMIT_MIN;
//     //     // // 到达下限位时，仅允许正向速度（向上运动）
//     //     // if (speed < 0) {  // 试图继续向下运动
//     //     //     speed = 0;
//     //     //     // 可选：触发警报/日志
//     //     // }
//     // }

//     spade_motor -> set_speed(spade_motor,speed);
//     //USB_printf("%.2f\n",spade_motor->real_torque);
    
// }

// static float DeadZone_Process(int16_t input){               //高敏
//     if (abs(input)< DAEDZONE) return 0.0f;
//     return (input > 0) ? (input - DAEDZONE) : (input + DAEDZONE);
// }
// static float DeadZone_Process_1(int16_t input){             //低敏
//     if (abs(input)< DAEDZONEONE) return 0.0f;
//     return (input > 0) ? (input - DAEDZONEONE) : (input + DAEDZONEONE);
// }

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
