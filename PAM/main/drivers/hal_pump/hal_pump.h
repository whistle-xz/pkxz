#ifndef HAL_PUMP_H
#define HAL_PUMP_H

#include <stdbool.h> 

void pump_init(void);
void pump_control_loop(void *pvParameters);

// 【新增】：外部控制接口
void pump_set_enable(bool enable);

bool pump_is_enabled(void);

#endif