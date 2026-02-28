//全局硬件配置头文件，定义引脚和外设参数

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/i2c.h"

// 1. 气泵与压力开关 (QPM11)
#define PUMP_RELAY_PIN          GPIO_NUM_14   // 气泵继电器控制引脚
#define PRESSURE_SWITCH_PIN     GPIO_NUM_21   // QPM11 压力开关引脚 (NC常闭: 低压闭合/高压断开)

// 2. 电磁阀 PWM 配置 (两位三通阀)
#define VALVE_PWM_FREQ_HZ       50            // 50Hz
#define VALVE_LEDC_TIMER        LEDC_TIMER_0
#define VALVE_LEDC_MODE         LEDC_LOW_SPEED_MODE
#define VALVE_LEDC_RES          LEDC_TIMER_13_BIT // 13位分辨率 (0-8191)
#define VALVE_MAX_DUTY          8191

// 使用 ESP32-S3 侧边排列的 GPIO，避开 USB 和 JTAG
#define VALVE_A_IN_PIN          GPIO_NUM_10   // 肌肉A 进气阀
#define VALVE_A_OUT_PIN         GPIO_NUM_11   // 肌肉A 排气阀 
#define VALVE_B_IN_PIN          GPIO_NUM_12   // 肌肉B 进气阀
#define VALVE_B_OUT_PIN         GPIO_NUM_13   // 肌肉B 排气阀

// 3. 传感器通信引脚
// I2C (AS5600 角度)
#define I2C_MASTER_SDA_IO       GPIO_NUM_4
#define I2C_MASTER_SCL_IO       GPIO_NUM_5
#define I2C_MASTER_FREQ_HZ      400000
#define I2C_PORT_NUM            I2C_NUM_0

// UART (气压传感器)
#define UART_TX_PIN             GPIO_NUM_15   // 接传感器的 RX
#define UART_RX_PIN             GPIO_NUM_16   // 接传感器的 TX
#define UART_PORT_NUM           UART_NUM_1
#define UART_BAUD_RATE          9600

#endif // HARDWARE_CONFIG_H