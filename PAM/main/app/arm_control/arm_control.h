#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include <stdint.h>

void arm_control_init(void);
void arm_set_target_angle(float angle);
void arm_control_task(void *pvParameters);

#endif // ARM_CONTROL_H