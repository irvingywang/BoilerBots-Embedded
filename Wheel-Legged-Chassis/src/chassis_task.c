#include "chassis_task.h"

#include "robot.h"
#include "remote.h"
#include "dm_motor.h"
#include "two_bar_leg.h"
#include "pid.h"
#include "user_math.h"
#include "wheel_legged_3d_lqr.h"
#include "dji_motor.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;
DM_Motor_Handle_t *g_chassis_joint_motor[4] = {NULL};
DJI_Motor_Handle_t *g_chassis_drive_motor[2] = {NULL};
Two_Bar_Kinematics_t g_right_leg_kinematics = {0};
Two_Bar_Kinematics_t g_left_leg_kinematics = {0};
PID_t g_chassis_right_leg_length_pid = {0};
PID_t g_chassis_left_leg_length_pid = {0};
PID_t g_chassis_right_leg_angle_pid = {0};
PID_t g_chassis_left_leg_angle_pid = {0};
Two_Bar_Virtual_Force right_virtual_force = {
    .supportive_force = 0,
    .torque = 0.0f};
Two_Bar_Motor_Torque right_motor_torque = {
    .torque1 = 0.0f,
    .torque2 = 0.0f};

Two_Bar_Virtual_Force left_virtual_force = {
    .supportive_force = 0,
    .torque = 0.0f};

Two_Bar_Motor_Torque left_motor_torque = {
    .torque1 = 0.0f,
    .torque2 = 0.0f};
struct
{
    float target_left_leg_length;
    float target_left_leg_angle;
    float target_right_leg_length;
    float target_right_leg_angle;
} g_chassis_target;

float g_chassis_right_angle_error;
float g_chassis_right_angle_error_original;

WheelLeggedState g_wheel_legged_state = {0};
WheelLeggedInput g_wheel_legged_input = {0};

void Chassis_Task_Init()
{
    DM_Motor_Config_t motor_config = {
        .can_bus = 1,
        .control_mode = DM_MOTOR_MIT,
        .disable_behavior = DM_MOTOR_HARDWARE_DISABLE, // DM_MOTOR_HARDWARE_DISABLE
        .pos_offset = 0.0f,
        .kp = 10.0f,
        .kd = .5f,
    };
    float pos_offset[4] = {-.3736f, -3.0679f, -2.607f, -2.6766f};
    for (int i = 0; i < 4; i++)
    {
        motor_config.tx_id = 0x00 + i;
        motor_config.rx_id = 0x10 + i;
        motor_config.pos_offset = pos_offset[i];
        g_chassis_joint_motor[i] = DM_Motor_Init(&motor_config);
    }
    Motor_Config_t drive_motor_config = {
        .can_bus = 1,
        .speed_controller_id = 0x01,
        .motor_reversal = MOTOR_REVERSAL_REVERSED,
        .control_mode = TORQUE_CONTROL,
    };
    g_chassis_drive_motor[0] = DJI_Motor_Init(&drive_motor_config, M3508_PLANETARY);
    drive_motor_config.speed_controller_id = 0x02;
    g_chassis_drive_motor[1] = DJI_Motor_Init(&drive_motor_config, M3508_PLANETARY);
    
    PID_Init(&g_chassis_right_leg_length_pid, 100.0f, 0.0f, 1000.0f, 50.0f, 0.0f, 0.0f);
    PID_Init(&g_chassis_left_leg_length_pid, 100.0f, 0.0f, 1000.0f, 50.0f, 0.0f, 0.0f);
    PID_Init(&g_chassis_right_leg_angle_pid, 3.0f, 0.0f, 100.0f, 20.0f, 0.0f, 0.0f);
    PID_Init(&g_chassis_left_leg_angle_pid, 3.0f, 0.0f, 100.0f, 20.0f, 0.0f, 0.0f);


}

void Chassis_Kinematics_Update()
{
    Two_Bar_Forward_Kinematics(&g_right_leg_kinematics, g_chassis_joint_motor[2]->stats->pos, g_chassis_joint_motor[3]->stats->pos);
    Two_Bar_Forward_Kinematics(&g_left_leg_kinematics, g_chassis_joint_motor[0]->stats->pos, g_chassis_joint_motor[1]->stats->pos);
    // Construct Wheel Legged State

}

void Chassis_Apply_Controller()
{
    // Set Goal
        g_chassis_target.target_left_leg_length = 0.25f + g_remote.controller.left_stick.y / 3000.0f;
        g_chassis_target.target_left_leg_angle = g_remote.controller.left_stick.x / 110.0f - 1.57f;
        g_chassis_target.target_right_leg_length = 0.25f + g_remote.controller.right_stick.y / 3000.0f;
        g_chassis_target.target_right_leg_angle = g_remote.controller.right_stick.x / 110.0f - 1.57f;

        // Right Leg
        float virtual_supportive_force = PID(&g_chassis_right_leg_length_pid, g_chassis_target.target_left_leg_length - g_right_leg_kinematics.leg_length);
        float angle_error = g_chassis_target.target_left_leg_angle - g_right_leg_kinematics.theta;
        __MAP_ANGLE_TO_UNIT_CIRCLE(angle_error);
        right_virtual_force.supportive_force = virtual_supportive_force;
        right_virtual_force.torque = PID(&g_chassis_right_leg_angle_pid, angle_error);

        // Left Leg
        virtual_supportive_force = PID(&g_chassis_left_leg_length_pid, g_chassis_target.target_right_leg_length - g_left_leg_kinematics.leg_length);
        angle_error = g_chassis_target.target_right_leg_angle - g_left_leg_kinematics.theta;
        __MAP_ANGLE_TO_UNIT_CIRCLE(angle_error);
        left_virtual_force.supportive_force = virtual_supportive_force;
        left_virtual_force.torque = PID(&g_chassis_left_leg_angle_pid, angle_error);

        Two_Bar_Get_Motor_Torque_From_Virtual_Force(&g_right_leg_kinematics, &right_virtual_force, &right_motor_torque);
        Two_Bar_Get_Motor_Torque_From_Virtual_Force(&g_left_leg_kinematics, &left_virtual_force, &left_motor_torque);
}

void Chassis_Ctrl_Loop()
{
    Chassis_Kinematics_Update();

    if (g_robot_state.state == ENABLED)
    {
        DM_Motor_Enable_Motor(g_chassis_joint_motor[0]);
        DM_Motor_Enable_Motor(g_chassis_joint_motor[1]);
        DM_Motor_Enable_Motor(g_chassis_joint_motor[2]);
        DM_Motor_Enable_Motor(g_chassis_joint_motor[3]);
    }

    if (g_remote.controller.right_switch == MID) // Open Loop VMC (Debug Only)
    {
        // left_virtual_force.supportive_force = g_remote.controller.left_stick.y / 33.0f;
        // left_virtual_force.torque = g_remote.controller.left_stick.x / 66.0f;
        // Two_Bar_Get_Motor_Torque_From_Virtual_Force(&g_right_leg_kinematics, &left_virtual_force, &left_motor_torque);
        // DM_Motor_Ctrl_MIT_PD(g_chassis_joint_motor[0], 0.0f, 0.0f, left_motor_torque.torque1, 0.0f, 0.0f);
        // DM_Motor_Ctrl_MIT_PD(g_chassis_joint_motor[1], 0.0f, 0.0f, left_motor_torque.torque2, 0.0f, 0.0f);

        // right_virtual_force.supportive_force = g_remote.controller.right_stick.y / 33.0f;
        // right_virtual_force.torque = g_remote.controller.right_stick.x / 66.0f;
        // DM_Motor_Ctrl_MIT_PD(g_chassis_joint_motor[2], 0.0f, 0.0f, right_motor_torque.torque1, 0.0f, 0.0f);
        // DM_Motor_Ctrl_MIT_PD(g_chassis_joint_motor[3], 0.0f, 0.0f, right_motor_torque.torque2, 0.0f, 0.0f);
        DJI_Motor_Set_Torque(g_chassis_drive_motor[0], g_remote.controller.left_stick.y / 660.0f * 3.6f);
        DJI_Motor_Set_Torque(g_chassis_drive_motor[1], g_remote.controller.right_stick.y / 660.0f * 3.6f);
    }
    else if (g_remote.controller.right_switch == UP) // Close Loop
    {
        Chassis_Apply_Controller();
        DM_Motor_Ctrl_MIT_PD(g_chassis_joint_motor[0], 0.0f, 0.0f, left_motor_torque.torque1, 0.0f, 0.0f);
        DM_Motor_Ctrl_MIT_PD(g_chassis_joint_motor[1], 0.0f, 0.0f, left_motor_torque.torque2, 0.0f, 0.0f);
        DM_Motor_Ctrl_MIT_PD(g_chassis_joint_motor[2], 0.0f, 0.0f, right_motor_torque.torque1, 0.0f, 0.0f);
        DM_Motor_Ctrl_MIT_PD(g_chassis_joint_motor[3], 0.0f, 0.0f, right_motor_torque.torque2, 0.0f, 0.0f);
    }
}