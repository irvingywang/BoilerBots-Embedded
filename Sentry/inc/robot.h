#ifndef ROBOT_H
#define ROBOT_H

#include "user_math.h"
#include <stdint.h>

typedef enum Robot_State_e {
    // Primary Enable Modes
    STARTING_UP,
    DISABLED,
    ENABLED
} Robot_State_e;

typedef struct {
    // chassis motion
    float x_speed;
    float y_speed;
    float omega;
    uint8_t IS_SPINTOP_ENABLED;

    // power management
    uint16_t power_index;
    uint16_t power_count;
    float avg_power;
    float total_power;
    float power_increment_ratio;
} Chassis_State_t;

typedef struct {
    float pitch_angle;
    float yaw_angle;
} Gimbal_State_t;

typedef enum Fire_Mode_e {
    NO_FIRE,
    SINGLE_FIRE,
    BURST_FIRE,
    FULL_AUTO,
    REJIGGLE, // primarily for busy mode
    IDLE      // primarily for busy mode
} Fire_Mode_e;

typedef struct Shooter_State_t {
    float prev_time;
    float prev_vel;
    float accum_angle;
} Shooter_State_t;

typedef struct {
    uint8_t IS_FIRING_ENABLED;
    uint8_t IS_AUTO_AIMING_ENABLED;
    uint8_t IS_FLYWHEEL_ENABLED;
    uint8_t IS_BUSY;

    Fire_Mode_e fire_mode;         // requested fire mode
    Fire_Mode_e busy_mode;         // current fire mode (in progress)
    Shooter_State_t shooter_state; // used for position integration
} Launch_State_t;

typedef struct {
    // controller input
    float vx;
    float vy;
    float vomega; // TODO Remove

    // mouse input
    float vx_keyboard;
    float vy_keyboard;

    // previous switch states
    uint8_t prev_left_switch;
    uint8_t prev_right_switch;

    // previous key states
    uint8_t prev_B;
    uint8_t prev_G;
    uint8_t prev_V;
    uint8_t prev_Z;
    uint8_t prev_Shift;
} Input_State_t;

typedef struct {
    Robot_State_e state;
    Chassis_State_t chassis;
    Gimbal_State_t gimbal;
    Launch_State_t launch;
    Input_State_t input;

    uint8_t IS_SUPER_CAPACITOR_ENABLED;
    uint8_t UI_ENABLED;
    uint8_t IS_SAFELY_STARTED;
} Robot_State_t;

void Robot_Init(void);
void Robot_Command_Loop(void);
void Handle_Starting_Up_State(void);
void Handle_Enabled_State(void);
void Handle_Disabled_State(void);
void Process_Remote_Input(void);
void Process_Chassis_Control(void);
void Process_Gimbal_Control(void);
void Process_Launch_Control(void);

extern Robot_State_t g_robot_state;

#endif // ROBOT_H
