#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "hardware_config.h"
#include "hal_pump.h"
#include "hal_valves.h"
#include "press.h"
#include "arm_control.h"
#include "serial_debug.h"
#include "stm32_comm.h"

static const char *TAG = "MAIN";
void app_main(void) {
    ESP_LOGI(TAG, "========= PAM Robot Arm System Start =========");
    
    // ================= 第 0 步：强行夺取 GPIO 6 和 7 的控制权 =================
    // 无论框架默认把它们当成了什么(I2C等)，强行复位并拉低！
    gpio_reset_pin(6);
    gpio_set_direction(6, GPIO_MODE_OUTPUT);
    gpio_set_level(6, 0); 

    gpio_reset_pin(7);
    gpio_set_direction(7, GPIO_MODE_OUTPUT);
    gpio_set_level(7, 0);
    
    // 1. 初始化硬件
    pump_init();
    valves_init();

    // 2. 初始化传感器
    pressure_sensor_init();

    // 3. 初始化控制算法
    arm_control_init();

    // 4. 启动任务
    ESP_LOGI(TAG, "Starting Tasks...");

    // 任务 A: 气泵保压
    xTaskCreate(pump_control_loop, "Pump_Task", 2048, NULL, 4, NULL);

    // 【新增】任务 S: 传感器读取与滤波任务 (优先级最高，确保数据不断更)
    xTaskCreate(sensor_task, "Sensor_Task", 4096, NULL, 6, NULL);

    // 任务 B: 机械臂控制
    xTaskCreate(arm_control_task, "Arm_Ctrl_Task", 4096, NULL, 5, NULL);

    // 任务 C: USB 串口调试任务
    xTaskCreate(
        cli_task,       // 任务函数
        "CLI_Task",     // 任务名
        4096,           // 栈大小
        NULL,           // 参数
        3,              // 优先级
        NULL            // 句柄
    );
    
    
    // 【新增】任务 D: STM32 串口通讯任务
    xTaskCreate(
        stm32_comm_task,   // 任务函数
        "STM32_Comm",      // 任务名
        4096,              // 栈大小
        NULL,              // 参数
        4,                 // 优先级 (设为4，比 CLI 高一点，保证通讯不掉线)
        NULL               // 句柄
    );
    ESP_LOGI(TAG, "All Tasks Started.");
    
    // 主循环保持存活
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}