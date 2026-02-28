#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>

// 初始化 I2C 和 多路选择引脚
void as5600_init(void);

// 设置当前位置为零点
void as5600_set_zero(void);

// 获取经过卡尔曼滤波后的角度
// channel: 0-7 (对应多路选择器通道，如果没有MUX，传0即可)
int16_t as5600_get_angle(int channel);

#endif
