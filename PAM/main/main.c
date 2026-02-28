#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "hardware_config.h"
#include "hal_pump.h"
#include "hal_valves.h"
#include "as5600.h"
#include "press.h"
#include "arm_control.h"

static const char *TAG = "MAIN";
void app_main(void) {
    ESP_LOGI(TAG, "========= PAM Robot Arm System Start =========");

    // --- 1. 硬件抽象层 (HAL) 初始化 ---
    ESP_LOGI(TAG, "[1/3] Initializing HAL...");
    pump_init();    // 气泵 & 压力开关
    valves_init();  // 电磁阀 PWM

    // --- 2. 传感器层初始化 ---
    ESP_LOGI(TAG, "[2/3] Initializing Sensors...");
    as5600_init();          // 角度传感器
    pressure_sensor_init(); // 气压传感器

    // --- 3. 控制层初始化 ---
    ESP_LOGI(TAG, "[3/3] Initializing Control System...");
    arm_control_init();     // PID 参数初始化

    // --- 4. 启动 RTOS 任务 ---
    ESP_LOGI(TAG, "Starting Tasks...");

    // 任务 A: 气泵自动保压任务 (优先级 4)
    // 负责监控储气罐压力，自动启停气泵
    xTaskCreate(
        (TaskFunction_t)pump_control_loop, // 任务函数 (注意: 之前pump_control_loop如果是死循环直接用，如果不是需封装)
        "Pump_Task",                       // 任务名
        2048,                              // 栈大小
        NULL,                              // 参数
        4,                                 // 优先级
        NULL                               // 句柄
    );

    // 任务 B: 机械臂核心运动控制任务 (优先级 5 - 实时性高)
    // 负责 50Hz 的 PID 计算和阀门控制
    xTaskCreate(
        arm_control_task,
        "Arm_Ctrl_Task",
        4096,
        NULL,
        5,
        NULL
    );

    // 测试动作：让机械臂动起来
    // 延时 2 秒等待系统稳定
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "Command: Go to 30 degrees");
    arm_set_target_angle(30.0f);

    while (1) {
        // 主循环每 1 秒打印一次存活信息
        // 实际应用中可以处理 USB 命令或 WIFI 通信
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}