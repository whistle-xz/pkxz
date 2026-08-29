#ifndef PRESS_H
#define PRESS_H

#include <stdint.h>

//定义错误码
#define PRESS_SENSOR_ERROR 0xFFFFFFFF

void pressure_sensor_init(void);

// 获取最新经过滤波的气压值 (供其他任务调用，非阻塞)
uint32_t get_filtered_pressure(int channel);

// 传感器后台读取任务
void sensor_task(void *pvParameters);

#endif