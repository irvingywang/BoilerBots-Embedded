#include "gimbal_task.h"

#include "robot.h"
#include "remote.h"
#include "user_math.h"
#include "dji_motor.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;
DJI_Motor_Handle_t *g_feed_motor = NULL;
DJI_Motor_Handle_t *g_launch_motor_left, *g_launch_motor_right = NULL;

void Gimbal_Task_Init()
{
    Motor_Config_t feed_motor_config = {
        .can_bus = 1,
        .speed_controller_id = 1,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid = {
            .kp = 100.0f,
            .ki = 0.0f,
            .kd = 1000.0f,
            .integral_limit = 0.0f,
            .output_limit = M2006_MAX_CURRENT_INT,
        },
    };
    g_feed_motor = DJI_Motor_Init(&feed_motor_config, M2006);

    Motor_Config_t launch_motor_config = {
        .can_bus = 1,
        .speed_controller_id = 2,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid = {
            .kp = 100.0f,
            .ki = 0.0f,
            .kd = 1000.0f,
            .integral_limit = 0.0f,
            .output_limit = M2006_MAX_CURRENT_INT,
        },
    };
    g_launch_motor_left = DJI_Motor_Init(&launch_motor_config, M3508);
    launch_motor_config.speed_controller_id = 3;
    launch_motor_config.motor_reversal = MOTOR_REVERSAL_REVERSED;
    g_launch_motor_right = DJI_Motor_Init(&launch_motor_config, M3508);
}

void Gimbal_Ctrl_Loop()
{
    DJI_Motor_Set_Velocity(g_feed_motor, g_remote.controller.wheel);
    
    if (g_remote.controller.left_switch == UP)
    {
        g_robot_state.launch.IS_FLYWHEEL_ENABLED = 1;
        DJI_Motor_Set_Velocity(g_launch_motor_left, 50.0f);
        DJI_Motor_Set_Velocity(g_launch_motor_right, 50.0f);
    }
    else
    {
        g_robot_state.launch.IS_FLYWHEEL_ENABLED = 0;
        DJI_Motor_Set_Velocity(g_launch_motor_left, 0.0f);
        DJI_Motor_Set_Velocity(g_launch_motor_right, 0.0f);
    }
}
