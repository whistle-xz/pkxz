#ifndef HAL_VALVES_H
#define HAL_VALVES_H

#include <stdint.h>

// 初始化所有电磁阀 PWM
void valves_init(void);

// 设置指定阀门的开启程度
// channel: 0~3 (对应 A_IN, A_OUT, B_IN, B_OUT)
// duty: 0 ~ 8191 (0=全关, 8191=全开)
void valve_set_duty(int channel, uint32_t duty);

// 辅助函数：全开或全关
void valve_open_full(int channel);
void valve_close_full(int channel);

#endif