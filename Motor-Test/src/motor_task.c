#include "motor_task.h"
#include "dji_motor.h"
#include "lkm_motor.h"
#include "dm_motor.h"
#include "mf_motor.h"
#include "supercap.h"

extern Supercap_t g_supercap;

void Motor_Task_Loop() {
    LKM_Motor_Send();
    // MF_Motor_Send();
    // DM_Motor_Send();

    g_supercap.send_counter++;
    if (g_supercap.send_counter >= 100) {
        Supercap_Send();
        g_supercap.send_counter = 0;
    }
}

