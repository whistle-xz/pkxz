#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/i2c.h"

// ================= 1. 气泵与压力开关 =================
#define PUMP_RELAY_PIN          GPIO_NUM_14   // 气泵继电器
#define PRESSURE_SWITCH_PIN     GPIO_NUM_21   // QPM11 压力开关 (注意：避免与其他针脚冲突)

/// ================= 2. I2C 编码器 (AS5600) =================
#define I2C_MASTER_FREQ_HZ      400000

// 编码器 1 (大臂关节) - 使用 I2C_0
#define I2C_PORT_UPPER          I2C_NUM_0
#define I2C0_SDA_IO             GPIO_NUM_8    // 传感器1的 SDA
#define I2C0_SCL_IO             GPIO_NUM_9    // 传感器1的 SCL

// 编码器 2 (小臂关节) - 使用 I2C_1
#define I2C_PORT_LOWER          I2C_NUM_1
#define I2C1_SDA_IO             GPIO_NUM_1   // 传感器2的 SDA (确保空闲)
#define I2C1_SCL_IO             GPIO_NUM_2   // 传感器2的 SCL (确保空闲)

// ================= 3. 电磁阀 PWM 配置 (8 个气阀) =================
#define VALVE_PWM_FREQ_HZ       25            
#define VALVE_LEDC_RES          LEDC_TIMER_13_BIT 
#define VALVE_MAX_DUTY          8191
#define VALVE_LEDC_MODE         LEDC_LOW_SPEED_MODE

// 大臂气阀 (原肌肉 A/B)
#define VALVE_1_PIN             GPIO_NUM_10   // V1 : A 充气
#define VALVE_2_PIN             GPIO_NUM_11   // V2 : B 充气
#define VALVE_3_PIN             GPIO_NUM_12   // V3 : A 排气
#define VALVE_4_PIN             GPIO_NUM_13   // V4 : B 排气

// 小臂气阀 (新增肌肉 C/D)
#define VALVE_5_PIN             GPIO_NUM_4    // V5 : C 充气 (确保此引脚空闲)
#define VALVE_6_PIN             GPIO_NUM_5    // V6 : D 充气
#define VALVE_7_PIN             GPIO_NUM_6    // V7 : C 排气
#define VALVE_8_PIN             GPIO_NUM_7    // V8 : D 排气

// --- 通道号定义 (用于控制 valve_set_duty) ---
#define VALVE_A_INFLATE_CH      0
#define VALVE_B_INFLATE_CH      1
#define VALVE_A_EXHAUST_CH      2
#define VALVE_B_EXHAUST_CH      3
#define VALVE_C_INFLATE_CH      4
#define VALVE_D_INFLATE_CH      5
#define VALVE_C_EXHAUST_CH      6
#define VALVE_D_EXHAUST_CH      7

// ================= 4. 传感器通信引脚 (UART) =================
// 气压传感器 A
#define UART1_PORT_NUM          UART_NUM_1
#define UART1_TX_PIN            GPIO_NUM_15   
#define UART1_RX_PIN            GPIO_NUM_16   

// 气压传感器 B
#define UART2_PORT_NUM          UART_NUM_2
#define UART2_TX_PIN            GPIO_NUM_17   
#define UART2_RX_PIN            GPIO_NUM_18   
#define UART_BAUD_RATE          9600

// ================= 5. 与 STM32 通讯的 UART0 配置 =================
#define UART0_PORT_NUM          UART_NUM_0
#define UART0_TX_PIN            GPIO_NUM_43   // 接 STM32 的 USART6_RX
#define UART0_RX_PIN            GPIO_NUM_44   // 接 STM32 的 USART6_TX

// ================= 6. 控制参数 =================                  
#define BASE_PRESSURE           260.0f  // 基础气压 (kPa)

#endif 
