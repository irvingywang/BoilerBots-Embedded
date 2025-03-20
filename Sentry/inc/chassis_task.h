#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#define SPIN_TOP_OMEGA 2 // rad / s
#define MAX_ANGLUAR_SPEED 3.14 // rad / s

// Function prototypes
void Chassis_Task_Init(void);
void Chassis_Ctrl_Loop(void);

#endif // CHASSIS_TASK_H
