#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

// TODO: Find correct values

// Note: these values are in degrees so that they are human-readable

#define PITCH_LOWER_LIMIT (-0.8f)
#define PITCH_UPPER_LIMIT (0.7f)

#define PITCH_OFFSET (0.06f)
#define YAW_OFFSET (0.0f)

#define PITCH_CONTROLLER_VELOCITY_COEF (4e-5f)
#define YAW_CONTORLLER_VELOCITY_COEF (5e-5f)

#define PITCH_MOUSE_VELOCITY_COEF (5e-5f)
#define YAW_MOUSE_VELOCITY_COEF (1e-5f)

// TODO: Find correct values

// Degrees per second

#define PITCH_RATE_LIMIT (16.0f)
#define YAW_RATE_LIMIT (1.0f)

typedef struct
{
    float pitch_velocity;
    float pitch_angle;
    float yaw_velocity;
    float yaw_angle;
} Gimbal_Target_t;


// Function prototypes
void Gimbal_Task_Init(void);
void Gimbal_Ctrl_Loop(void);

#endif // GIMBAL_TASK_H