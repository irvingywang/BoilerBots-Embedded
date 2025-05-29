#include "chassis_task.h"

#include "robot.h"
#include "remote.h"
#include "dm_motor.h"
#include "two_bar_leg.h"
#include "pid.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;
DM_Motor_Handle_t *g_chassis_motor[4] = {NULL};
Two_Bar_Kinematics_t g_right_leg_kinematics = {0};
PID_t g_chassis_right_leg_length_pid = {0};
PID_t g_chassis_right_leg_angle_pid = {0};
Two_Bar_Virtual_Force virtual_force = {
    .supportive_force = 0,
    .torque = 0.0f
};
Two_Bar_Motor_Torque motor_torque = {
    .torque1 = 0.0f,
    .torque2 = 0.0f
};
struct {
    float target_leg_length;
    float target_leg_angle;
} g_chassis_target;
void Chassis_Task_Init()
{
    DM_Motor_Config_t motor_config = {
        .can_bus = 1,
        .control_mode = DM_MOTOR_MIT,
        .disable_behavior = DM_MOTOR_HARDWARE_DISABLE,//DM_MOTOR_HARDWARE_DISABLE
        .pos_offset = 0.0f,
        .kp = 10.0f,
        .kd = .5f,
    };
    for (int i = 0; i < 4; i++) {
        motor_config.tx_id = 0x00 + i;
        motor_config.rx_id = 0x10 + i;
        g_chassis_motor[i] = DM_Motor_Init(&motor_config);
    }

    PID_Init(&g_chassis_right_leg_length_pid, 200.0f, 0.0f, 400.0f, 10.0f, 0.0f, 0.0f);
    // PID_Init(&g_chassis_right_leg_angle_pid, 1.5f, 0.0f, 400.0f, 10.0f, 0.0f, 0.0f);
    
}

void Chassis_Kinematics_Update()
{
    Two_Bar_Forward_Kinematics(&g_right_leg_kinematics, g_chassis_motor[2]->stats->pos, g_chassis_motor[3]->stats->pos);

}

void Chassis_Ctrl_Loop()
{
    Chassis_Kinematics_Update();
    g_chassis_target.target_leg_length = 0.25f + g_remote.controller.left_stick.y / 2000.0f;
    g_chassis_target.target_leg_angle = g_remote.controller.right_stick.x / 660.0f + 1.57f;
    float virtual_supportive_force = PID(&g_chassis_right_leg_length_pid, g_chassis_target.target_leg_length - g_right_leg_kinematics.leg_length);
    float virtual_torque = PID(&g_chassis_right_leg_angle_pid, g_chassis_target.target_leg_angle - g_right_leg_kinematics.theta);
    virtual_force.supportive_force = virtual_supportive_force;
    virtual_force.torque = virtual_torque;

    Two_Bar_Get_Motor_Torque_From_Virtual_Force(&g_right_leg_kinematics, &virtual_force, &motor_torque);

    if (g_robot_state.state == ENABLED)
    {
        DM_Motor_Enable_Motor(g_chassis_motor[0]);
        DM_Motor_Enable_Motor(g_chassis_motor[1]);
        DM_Motor_Enable_Motor(g_chassis_motor[2]);
        DM_Motor_Enable_Motor(g_chassis_motor[3]);
    }
    
    if (g_remote.controller.right_switch == MID)
    {
        DM_Motor_Ctrl_MIT_PD(g_chassis_motor[0], 0.0f, 0.0f, g_remote.controller.left_stick.x/220.0f, 0.0f, 0.0f);
        DM_Motor_Ctrl_MIT_PD(g_chassis_motor[1], 0.0f, 0.0f, g_remote.controller.left_stick.y/220.0f, 0.0f, 0.0f);
        DM_Motor_Ctrl_MIT_PD(g_chassis_motor[2], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        DM_Motor_Ctrl_MIT_PD(g_chassis_motor[3], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    else if (g_remote.controller.right_switch == UP)
    {
        
        
        DM_Motor_Ctrl_MIT_PD(g_chassis_motor[2], 0.0f, 0.0f, motor_torque.torque1, 0.0f, 0.0f);
        DM_Motor_Ctrl_MIT_PD(g_chassis_motor[3], 0.0f, 0.0f, motor_torque.torque2, 0.0f, 0.0f);
    }
}