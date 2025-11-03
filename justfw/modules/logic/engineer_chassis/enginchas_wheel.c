#include "enginchas_logic.h"

#define Chassis_Width 0.49
#define Chassis_Length 0.35

/* engineer */
static INTF_Motor_HandleTypeDef *m_s_l;
static INTF_Motor_HandleTypeDef *m_s_r;

void enginchs_set_wheel(float speed_x, float speed_y, float speed_w) {
    /* engineer */

    m_s_l->set_speed(m_s_l, -((speed_y - speed_w) * (Chassis_Width + Chassis_Length) * 0.5f / 0.028f));
    m_s_r->set_speed(m_s_r, ((speed_y + speed_w) * (Chassis_Width + Chassis_Length) * 0.5f / 0.028f));
}

void enginchas_wheel_init() {
    /*engineer */
    m_s_l = Bus_SharePtr("/motor/EN_S_L", sizeof(INTF_Motor_HandleTypeDef));
    m_s_r = Bus_SharePtr("/motor/EN_S_R", sizeof(INTF_Motor_HandleTypeDef));
}