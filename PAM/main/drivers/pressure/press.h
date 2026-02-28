#ifndef PRESS_H
#define PRESS_H

#include <stdint.h>

void pressure_sensor_init(void);

// 读取指定通道的气压值 (单位: kPa)
// channel: 0-7 (MUX通道)
uint32_t pressure_read_kpa(int channel);

#endif