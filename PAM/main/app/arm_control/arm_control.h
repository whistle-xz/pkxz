#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include <stdint.h>

void arm_control_init(void);
void arm_set_target_angle(float upper_deg, float lower_deg);
void arm_control_task(void *pvParameters);
void arm_tune_pressure_pid(float kp_inf, float kp_def, float ki, float kd);

void arm_set_target_angle(float upper_deg, float lower_deg);
void arm_set_remote_pressures(uint32_t press_c, uint32_t press_d);

uint32_t arm_get_remote_press_c(void);
uint32_t arm_get_remote_press_d(void);

#endif // ARM_CONTROL_H
