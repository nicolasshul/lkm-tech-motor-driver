#ifndef LKM_MOTOR_H
#define LKM_MOTOR_H

#include <stdint.h>
#include "bsp_can.h"
#include "motor.h"
#include "pid.h"

#define MAX_LKM_MOTORS (32)

#define MG8016_REDUCTION_RATIO (6.0f)
#define MG8016_MAX_AMPS 33.0f
#define IQ_CONTROL_MAX 2048.0f
#define FOURTEEN_BIT_ENCODER_RANGE 16383.0f

typedef enum {
    LKM_NO_COMMAND,
    LKM_HAS_COMMAND
} LKM_Motor_Command_e;

typedef enum {
    LKM_NO_COMMAND_SET                 = 0x00,
    LKM_CMD_MOTOR_OFF                  = 0x80,
    LKM_CMD_MOTOR_STOP                 = 0x81,
    LKM_CMD_MOTOR_ON                   = 0x88,

    LKM_CMD_READ_PID                   = 0x30,
    LKM_CMD_WRITE_PID_RAM              = 0x31,
    LKM_CMD_WRITE_PID_ROM              = 0x32,
    LKM_CMD_READ_ACCEL                 = 0x33,
    LKM_CMD_WRITE_ACCEL_RAM            = 0x34,

    LKM_CMD_SET_TORQUE                 = 0xA1,
    LKM_CMD_SET_SPEED                  = 0xA2,
    LKM_CMD_MULTI_ANGLE                = 0xA3,
    LKM_CMD_MULTI_ANGLE_SPEED_LIM      = 0xA4,
    LKM_CMD_SINGLE_ANGLE               = 0xA5,
    LKM_CMD_SINGLE_ANGLE_SPEED_LIM     = 0xA6,
    LKM_CMD_INCREMENT_ANGLE            = 0xA7,
    LKM_CMD_INCREMENT_ANGLE_SPEED_LIM  = 0xA8,

    LKM_CMD_READ_ENCODER               = 0x90,
    LKM_CMD_WRITE_ENCODER_ZERO_ROM     = 0x91,
    LKM_CMD_READ_MULTI_ANGLE           = 0x92,
    LKM_CMD_READ_SINGLE_ANGLE          = 0x94,

    LKM_CMD_READ_ERROR                 = 0x9A,
    LKM_CMD_CLEAR_ERROR                = 0x9B,
    LKM_CMD_READ_STATE                 = 0x9C,
    LKM_CMD_READ_MOTOR_CURR            = 0x9D,

} LKM_Motor_Command_Byte_e;

typedef enum {
    MG8016 // add more later if needed
} LKM_Motor_Type_e;

typedef struct LKM_Motor_Stats_s {
    /* Motor Stats */
    float torque_current; // Amps (A)
    float motor_speed; // Radians per Second 
    float encoder_pos; // Radians
    float reduction_ratio;
    uint8_t temp; // Celcius (C)
    LKM_Motor_Command_Byte_e command_byte;

} LKM_Motor_Stats_t;

typedef struct LKM_Motor_Handle_s {
    LKM_Motor_Type_e motor_type;
    uint8_t can_bus;
    uint8_t speed_controller_id;
    CAN_Instance_t * can_instance;

    /* Motor Config */
    LKM_Motor_Command_e command;
    LKM_Motor_Stats_t * stats;
    uint8_t control_mode;
    uint8_t enabled; // 1 if enabled, 0 if disabled
    uint8_t reversal; // 1 is forward, -1 is backward
    uint8_t speed_limit_enabled; // 1 if enabled, 0 if disabled
    float max_speed_limit;
    float encoder_offset;

    /* Motor Controller */ 
    PID_t * angle_pid; // Note, cannot set d value on LKM motors
    PID_t * velocity_pid;
    PID_t * torque_pid;
} LKM_Motor_Handle_t;

// init motor (enable motor), write PID, zero encoder, set up can etc. PID values are capped at 255, and only P and I are used (D is ignored).
LKM_Motor_Handle_t * LKM_Motor_Init(Motor_Config_t *config, LKM_Motor_Type_e type);

// send motor command
void LKM_Motor_Send(void);

// disable motor
void LKM_Motor_Disable(LKM_Motor_Handle_t * motor);

// enables motor
void LKM_Motor_Enable(LKM_Motor_Handle_t * motor);

// motor stop command
void LKM_Motor_Stop(LKM_Motor_Handle_t * motor);

// control by setting current
void LKM_Motor_Set_Torque_Current(LKM_Motor_Handle_t * motor, float torque_current);

// speed closed loop control
void LKM_Motor_Set_Velocity(LKM_Motor_Handle_t * motor, float velocity);

// go to absolute angle x (multi loop)
void LKM_Motor_Set_Absolute_Angle(LKM_Motor_Handle_t * motor, float abs_angle);

// go to absolute angle x (multi loop) w/ speed control parameter (in rad/s)
void LKM_Motor_Set_Absolute_Angle_Limit_Speed(LKM_Motor_Handle_t * motor, float abs_angle, float speed_limit);

// go to angle within one revolution (single loop)
void LKM_Motor_Set_Angle(LKM_Motor_Handle_t * motor, float angle);

// go to angle within one revolution (single loop) w/ speed control parameter
void LKM_Motor_Set_Angle_Limit_Speed(LKM_Motor_Handle_t * motor, float angle, float speed_limit);

// increment angle
void LKM_Motor_Increment_Angle(LKM_Motor_Handle_t * motor, float angle);

// increment angle w/ speed control parameter
void LKM_Motor_Increment_Angle_Limit_Speed(LKM_Motor_Handle_t * motor, float angle, float speed_limit);

// read stats from motor. LKM motors need to send request to get stats back from motors
void LKM_Motor_Read_Stats(LKM_Motor_Handle_t * motor);

// get encoder angle in radians
float LKM_Motor_Get_Total_Angle(LKM_Motor_Handle_t * motor);

// TODO: et absolute angle in radians. this sends a request to the motor
float LKM_Motor_Get_Absolute_Angle(LKM_Motor_Handle_t * motor);

// get motor speed in rad / s
float LKM_Motor_Get_Velocity(LKM_Motor_Handle_t * motor);

// enables all motors
void LKM_Motor_Enable_All();

// disables all motors
void LKM_Motor_Disable_All();

// read error state (implement later?)

// clear error state

#endif // LKM_MOTOR_H
