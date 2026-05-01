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

uint8_t message_updates_stats(uint8_t command_byte) {
    uint8_t cb = command_byte;
    return cb == LKM_CMD_SET_TORQUE                ||
           cb == LKM_CMD_SET_SPEED                 ||
           cb == LKM_CMD_MULTI_ANGLE               ||
           cb == LKM_CMD_MULTI_ANGLE_SPEED_LIM     ||
           cb == LKM_CMD_SINGLE_ANGLE              ||
           cb == LKM_CMD_SINGLE_ANGLE_SPEED_LIM    ||
           cb == LKM_CMD_INCREMENT_ANGLE           ||
           cb == LKM_CMD_INCREMENT_ANGLE_SPEED_LIM ||
           cb == LKM_CMD_READ_STATE;
}

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

    // Initializing CAN instance
    CAN_Instance_t * receiver_can_instance = NULL;
    receiver_can_instance = CAN_Device_Register(motor_handle->can_bus, motor_handle->speed_controller_id + 0x140, 
                                                motor_handle->speed_controller_id + 0x140, LKM_Motor_Decode);
    
    receiver_can_instance->binding_motor_stats = motor_stats;

    motor_handle->can_instance = receiver_can_instance; // remember to set pid here

    if (motor_handle->motor_type == MG8016) {
        motor_stats->reduction_ratio = MG8016_REDUCTION_RATIO;
    }

    // writing PID information to RAM

    uint8_t * data = motor_handle->can_instance->tx_buffer;
    memset(data, 0x0, 8);
    data[0] = LKM_CMD_WRITE_PID_RAM;

    if ((motor_handle->control_mode & VELOCITY_CONTROL) == VELOCITY_CONTROL)
    {
        uint8_t kp = 0;
        if (motor_handle->velocity_pid->kp < 0) {
            kp = 0;
        }
        else if (motor_handle->velocity_pid->kp > 255) {
            kp = 255;
        }
        else {
            kp = (uint8_t) motor_handle->velocity_pid->kp;
        }
        data[4] = kp;
        uint8_t ki = 0;
        if (motor_handle->velocity_pid->ki < 0) {
            ki = 0;
        }
        else if (motor_handle->velocity_pid->ki > 255) {
            ki = 255;
        }
        else {
            ki = (uint8_t) motor_handle->velocity_pid->ki;
        }
        data[5] = ki;
    }
    if ((motor_handle->control_mode & POSITION_CONTROL) == POSITION_CONTROL)
    {
        uint8_t kp = 0;
        if (motor_handle->velocity_pid->kp < 0) {
            kp = 0;
        }
        else if (motor_handle->velocity_pid->kp > 255) {
            kp = 255;
        }
        else {
            kp = (uint8_t) motor_handle->velocity_pid->kp;
        }
        data[1] = kp;
        uint8_t ki = 0;
        if (motor_handle->velocity_pid->ki < 0) {
            ki = 0;
        }
        else if (motor_handle->velocity_pid->ki > 255) {
            ki = 255;
        }
        else {
            ki = (uint8_t) motor_handle->velocity_pid->ki;
        }
        data[2] = ki;
    }
    if ((motor_handle->control_mode & TORQUE_CONTROL) == TORQUE_CONTROL)
    {
        uint8_t kp = 0;
        if (motor_handle->velocity_pid->kp < 0) {
            kp = 0;
        }
        else if (motor_handle->velocity_pid->kp > 255) {
            kp = 255;
        }
        else {
            kp = (uint8_t) motor_handle->velocity_pid->kp;
        }
        data[6] = kp;
        uint8_t ki = 0;
        if (motor_handle->velocity_pid->ki < 0) {
            ki = 0;
        }
        else if (motor_handle->velocity_pid->ki > 255) {
            ki = 255;
        }
        else {
            ki = (uint8_t) motor_handle->velocity_pid->ki;
        }
        data[7] = ki;
    }
    CAN_Transmit(motor_handle->can_instance);
    g_lkm_motors[g_lkm_motor_count++] = motor_handle;
    return motor_handle;
}

void LKM_Motor_Send(void) {
    for (int i = 0; i < g_lkm_motor_count; i++) {
        LKM_Motor_Handle_t * motor = g_lkm_motors[i];
        if (motor->command == LKM_HAS_COMMAND) {
            CAN_Transmit(motor->can_instance);
            motor->command = LKM_NO_COMMAND;
        }
        else {
            uint8_t * data = motor->can_instance->tx_buffer;
            memset(data, 0x0, 8);
            data[0] = LKM_CMD_READ_STATE;
            CAN_Transmit(motor->can_instance);
        }
    }
}

void LKM_Motor_Decode(CAN_Instance_t *can_instance) {
    uint8_t * data = can_instance->rx_buffer;
    LKM_Motor_Stats_t *data_frame = (LKM_Motor_Stats_t *) can_instance->binding_motor_stats;

    uint8_t cb = data[0]; // control byte
    data_frame->command_byte = cb;

    if (message_updates_stats(cb)) {
            
        uint8_t temp = data[1];
        data_frame->temp = temp;
        
        uint16_t torque_current = (uint16_t) ((data[3] << 8) | data[2]);
        data_frame->torque_current = (float) torque_current * (MG8016_MAX_AMPS / IQ_CONTROL_MAX);

        int16_t motor_speed = (uint16_t) ((data[5] << 8) | data[4]);
        data_frame->motor_speed = motor_speed * DEG_TO_RAD;
        
        uint16_t encoder_pos = (uint16_t) ((data[6] << 8) | data[7]);
        data_frame->encoder_pos = (encoder_pos / FOURTEEN_BIT_ENCODER_RANGE) * 360.0f * DEG_TO_RAD;
    }
}

void LKM_Motor_Enable(LKM_Motor_Handle_t * motor) {
    motor->command = LKM_HAS_COMMAND;
    motor->enabled = 1;
    uint8_t * data = motor->can_instance->tx_buffer;
    memset(data, 0x0, 8);
    data[0] = LKM_CMD_MOTOR_ON;
    return;
}

void LKM_Motor_Disable(LKM_Motor_Handle_t * motor) {
    motor->command = LKM_HAS_COMMAND;
    motor->enabled = 0;
    uint8_t * data = motor->can_instance->tx_buffer;
    memset(data, 0x0, 8);
    data[0] = LKM_CMD_MOTOR_OFF;
    return;
}

void LKM_Motor_Stop(LKM_Motor_Handle_t * motor) {
    motor->command = LKM_HAS_COMMAND;
    uint8_t * data = motor->can_instance->tx_buffer;
    memset(data, 0x0, 8);
    data[0] = LKM_CMD_MOTOR_STOP;
    return;
}

void LKM_Motor_Set_Torque_Current(LKM_Motor_Handle_t * motor, float torque_current) {
    motor->command = LKM_HAS_COMMAND;
    uint8_t * data = motor->can_instance->tx_buffer;
    memset(data, 0x0, 8);
    data[0] = LKM_CMD_SET_TORQUE;

    int16_t iqControl = (int16_t) ((torque_current / MG8016_MAX_AMPS) * IQ_CONTROL_MAX);

    if (iqControl > IQ_CONTROL_MAX) {
        iqControl = IQ_CONTROL_MAX;
    }
    if (iqControl < -IQ_CONTROL_MAX) {
        iqControl = -IQ_CONTROL_MAX; 
    }

    data[4] = *(uint8_t *) (&iqControl);
    data[5] = *((uint8_t *) (&iqControl) + 1);
    return;    
}

void LKM_Motor_Set_Velocity(LKM_Motor_Handle_t * motor, float velocity) {
    motor->command = LKM_HAS_COMMAND;
    uint8_t * data = motor->can_instance->tx_buffer;
    memset(data, 0x0, 8);
    data[0] = LKM_CMD_SET_SPEED;

    int32_t desired_velo = (int32_t) (velocity * (1 / DEG_TO_RAD) * 100.0f * motor->reversal);

    data[4] = *(uint8_t *) &desired_velo;           
    data[5] = *((uint8_t *) (&desired_velo) + 1);     
    data[6] = *((uint8_t *) (&desired_velo) + 2);     
    data[7] = *((uint8_t *) (&desired_velo) + 3);     
}

void LKM_Motor_Set_Absolute_Angle(LKM_Motor_Handle_t * motor, float abs_angle) {
    motor->command = LKM_HAS_COMMAND;
    uint8_t * data = motor->can_instance->tx_buffer;
    memset(data, 0x0, 8);
    data[0] = LKM_CMD_MULTI_ANGLE;

    int32_t desiredAngle = (int32_t) (abs_angle * (1 / DEG_TO_RAD) * 100.0f);

    data[4] = *(uint8_t *) (&desiredAngle);
    data[5] = *((uint8_t *) (&desiredAngle) + 1);
    data[6] = *((uint8_t *) (&desiredAngle) + 2);
    data[7] = *((uint8_t *) (&desiredAngle) + 3); 
}

void LKM_Motor_Set_Absolute_Angle_Limit_Speed(LKM_Motor_Handle_t * motor, float abs_angle, float speed_limit) {
    motor->command = LKM_HAS_COMMAND;
    uint8_t * data = motor->can_instance->tx_buffer;
    memset(data, 0x0, 8);
    data[0] = LKM_CMD_MULTI_ANGLE_SPEED_LIM;

    int32_t desiredAngle = (int32_t) (abs_angle * (1 / DEG_TO_RAD) * 100.0f);

    int16_t speedLimit = (int16_t) (speed_limit * (1 / DEG_TO_RAD));

    data[2] = *(uint8_t *) (&speedLimit);
    data[3] = *((uint8_t *) (&speedLimit) + 1);

    data[4] = *(uint8_t *) (&desiredAngle);
    data[5] = *((uint8_t *) (&desiredAngle) + 1);
    data[6] = *((uint8_t *) (&desiredAngle) + 2);
    data[7] = *((uint8_t *) (&desiredAngle) + 3); 
}

void LKM_Motor_Set_Angle(LKM_Motor_Handle_t * motor, float angle) {
    motor->command = LKM_HAS_COMMAND;
    uint8_t * data = motor->can_instance->tx_buffer;
    memset(data, 0x0, 8);
    data[0] = LKM_CMD_SINGLE_ANGLE;
    if (motor->reversal == 1) {
        data[1] = 0x0; // clockwise
    }
    else {
        data[1] = 0x1; // counterclockwise
    }

    int32_t desiredAngle = (int32_t) (angle * (1 / DEG_TO_RAD) * 100.0f);
    data[4] = *(uint8_t *) (&desiredAngle);
    data[5] = *((uint8_t *) (&desiredAngle) + 1);
    data[6] = *((uint8_t *) (&desiredAngle) + 2);
    data[7] = *((uint8_t *) (&desiredAngle) + 3);
}

void LKM_Motor_Set_Angle_Limit_Speed(LKM_Motor_Handle_t * motor, float angle, float speed_limit) {
    motor->command = LKM_HAS_COMMAND;
    uint8_t * data = motor->can_instance->tx_buffer;
    memset(data, 0x0, 8);
    data[0] = LKM_CMD_SINGLE_ANGLE_SPEED_LIM;
    if (motor->reversal == 1) {
        data[1] = 0x0; // clockwise
    }
    else {
        data[1] = 0x1; // counterclockwise
    }

    int32_t desiredAngle = (int32_t) (angle * (1 / DEG_TO_RAD) * 100.0f);
    data[4] = *(uint8_t *) (&desiredAngle);
    data[5] = *((uint8_t *) (&desiredAngle) + 1);
    data[6] = *((uint8_t *) (&desiredAngle) + 2);
    data[7] = *((uint8_t *) (&desiredAngle) + 3);

    int16_t speedLimit = (int16_t) (speed_limit * (1 / DEG_TO_RAD));

    data[2] = *(uint8_t *) (&speedLimit);
    data[3] = *((uint8_t *) (&speedLimit) + 1);
}

void LKM_Motor_Increment_Angle(LKM_Motor_Handle_t * motor, float angle) {
    motor->command = LKM_HAS_COMMAND;
    uint8_t * data = motor->can_instance->tx_buffer;
    memset(data, 0x0, 8);
    data[0] = LKM_CMD_INCREMENT_ANGLE;

    int32_t desiredAngle = (int32_t) (angle * (1 / DEG_TO_RAD) * 100.0f);

    data[4] = *(uint8_t *) (&desiredAngle);
    data[5] = *((uint8_t *) (&desiredAngle) + 1);
    data[6] = *((uint8_t *) (&desiredAngle) + 2);
    data[7] = *((uint8_t *) (&desiredAngle) + 3); 
}

void LKM_Motor_Increment_Angle_Limit_Speed(LKM_Motor_Handle_t * motor, float angle, float speed_limit) {
    motor->command = LKM_HAS_COMMAND;
    uint8_t * data = motor->can_instance->tx_buffer;
    memset(data, 0x0, 8);
    data[0] = LKM_CMD_INCREMENT_ANGLE_SPEED_LIM;

    int32_t desiredAngle = (int32_t) (angle * (1 / DEG_TO_RAD) * 100.0f);

    data[4] = *(uint8_t *) (&desiredAngle);
    data[5] = *((uint8_t *) (&desiredAngle) + 1);
    data[6] = *((uint8_t *) (&desiredAngle) + 2);
    data[7] = *((uint8_t *) (&desiredAngle) + 3); 

    int16_t speedLimit = (int16_t) (speed_limit * (1 / DEG_TO_RAD));

    data[2] = *(uint8_t *) (&speedLimit);
    data[3] = *((uint8_t *) (&speedLimit) + 1);

}

void LKM_Motor_Enable_All() {
    for (int i = 0; i < g_lkm_motor_count; i++) {
        LKM_Motor_Enable(g_lkm_motors[i]);
    }
}

void LKM_Motor_Disable_All() {
    for (int i = 0; i < g_lkm_motor_count; i++) {
        LKM_Motor_Disable(g_lkm_motors[i]);
    }
}
