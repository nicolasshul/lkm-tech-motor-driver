#include "chassis_task.h"

#include "robot.h"
#include "remote.h"
#include "lkm_motor.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

float chassis_rad;

LKM_Motor_Handle_t *g_test;

void Chassis_Task_Init()
{
    Motor_Config_t test_motor_config = {
        .can_bus = 1,
        .speed_controller_id = 2,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .control_mode = VELOCITY_CONTROL,
        .velocity_pid =
            {
                .kp = 10.0f,
                .kd = 1.0f
            },
        
    };

    g_test = LKM_Motor_Init(&test_motor_config, MG8016);
}

void Chassis_Ctrl_Loop()
{
    LKM_Motor_Set_Velocity(g_test, 2.0f);
    
    // Control loop for the chassis
}