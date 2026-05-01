#include "lkm_motor.h"

#include <stdint.h>
#include <stdlib.h>
#include "bsp_can.h"
#include "user_math.h"
#include "motor.h"
#include <math.h>

LKM_Motor_Handle_t * g_lkm_motors[MAX_LKM_MOTORS] = {NULL};
uint8_t g_lkm_motor_count = 0;

void LKM_Motor_Decode(CAN_Instance_t *can_instance);

LKM_Motor_Handle_t * LKM_Motor_Init(Motor_Config_t *config, LKM_Motor_Type_e type) {
    LKM_Motor_Handle_t * motor_handle = (LKM_Motor_Handle_t *) calloc(1, sizeof(LKM_Motor_Handle_t));
    // Initializing motor handle
    motor_handle->motor_type = type;
    motor_handle->can_bus = config->can_bus;
    motor_handle->speed_controller_id = config->speed_controller_id;
    motor_handle->command = LKM_NO_COMMAND;
    motor_handle->reversal = config->motor_reversal;
    motor_handle->enabled = 0;
    motor_handle->encoder_offset = config->offset;
    motor_handle->max_speed_limit = 0.0f;
    motor_handle->speed_limit_enabled = 0;
    motor_handle->control_mode = config->control_mode;
    
    // Initializing stats
    LKM_Motor_Stats_t * motor_stats = (LKM_Motor_Stats_t *) calloc(1, sizeof(LKM_Motor_Stats_t));
    motor_stats->command_byte = LKM_NO_COMMAND_SET;
    motor_stats->temp = 0.0f;
    motor_stats->torque_current = 0.0f;
    motor_stats->motor_speed = 0.0f;
    motor_stats->encoder_pos = 0.0f;

    // Initializing PID controllers
    if ((motor_handle->control_mode & VELOCITY_CONTROL) == VELOCITY_CONTROL)
    {
        motor_handle->velocity_pid = malloc(sizeof(PID_t));
        memcpy(motor_handle->velocity_pid, &config->velocity_pid, sizeof(PID_t));
    }
    if ((motor_handle->control_mode & POSITION_CONTROL) == POSITION_CONTROL)
    {
        motor_handle->angle_pid = malloc(sizeof(PID_t));
        memcpy(motor_handle->angle_pid, &config->angle_pid, sizeof(PID_t));
    }
    if ((motor_handle->control_mode & TORQUE_CONTROL) == TORQUE_CONTROL)
    {
        motor_handle->torque_pid = malloc(sizeof(PID_t));
        memcpy(motor_handle->torque_pid, &config->torque_pid, sizeof(PID_t));
    }

    // Initializing CAN instance
    CAN_Instance_t * receiver_can_instance = NULL;
    receiver_can_instance = CAN_Device_Register(motor_handle->can_bus, motor_handle->speed_controller_id + 0x140, 
                                                motor_handle->speed_controller_id + 0x140, LKM_Motor_Decode);
    
    receiver_can_instance->binding_motor_stats = motor_stats;

    if (motor_handle->motor_type == MG8016) {
        motor_stats->reduction_ratio = MG8016_REDUCTION_RATIO;
    }

    g_lkm_motors[g_lkm_motor_count++] = motor_handle;
    return motor_handle;
}
