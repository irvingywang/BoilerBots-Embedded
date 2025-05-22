#include "chassis_task.h"

#include "robot.h"
#include "remote.h"
#include "dm_motor.h"
extern Robot_State_t g_robot_state;
extern Remote_t g_remote;
DM_Motor_Handle_t *g_chassis_motor[4] = {NULL};

void Chassis_Task_Init()
{
    DM_Motor_Config_t motor_config = {
        .can_bus = 1,
        .control_mode = DM_MOTOR_MIT,
        .disable_behavior = DM_MOTOR_ZERO_CURRENT,
        .pos_offset = 0.0f,
        .kp = 10.0f,
        .kd = .5f,
    };
    for (int i = 0; i < 4; i++) {
        motor_config.tx_id = 0x00 + i;
        motor_config.rx_id = 0x10 + i;
        g_chassis_motor[i] = DM_Motor_Init(&motor_config);
    }
}

void Chassis_Ctrl_Loop()
{
    if (g_robot_state.state == ENABLED)
    {
        DM_Motor_Enable_Motor(g_chassis_motor[0]);
        DM_Motor_Enable_Motor(g_chassis_motor[1]);
        DM_Motor_Enable_Motor(g_chassis_motor[2]);
        DM_Motor_Enable_Motor(g_chassis_motor[3]);
    }
    

    DM_Motor_Ctrl_MIT(g_chassis_motor[0], g_remote.controller.left_stick.y/220.0f, g_remote.controller.right_stick.x, 0.0f);
    DM_Motor_Ctrl_MIT(g_chassis_motor[1], g_remote.controller.left_stick.y/220.0f, g_remote.controller.right_stick.x, 0.0f);
    DM_Motor_Ctrl_MIT(g_chassis_motor[2], g_remote.controller.left_stick.y/220.0f, g_remote.controller.right_stick.x, 0.0f);
    DM_Motor_Ctrl_MIT(g_chassis_motor[3], g_remote.controller.left_stick.y/220.0f, g_remote.controller.right_stick.x, 0.0f);
}