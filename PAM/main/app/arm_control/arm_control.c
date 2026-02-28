#include "arm_control.h"
#include "pid_algorithm.h"
#include "hardware_config.h"
#include "sensor_as5600.h"
#include "sensor_pressure.h"
#include "hal_valves.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "ARM_CTRL";

// 定义三个 PID 控制器
static pid_ctrl_t pid_angle;    ///< 角度环 PID
static pid_ctrl_t pid_press_A;  ///< 肌肉A 压力环 PID
static pid_ctrl_t pid_press_B;  ///< 肌肉B 压力环 PID

// 基础气压 (Base Pressure)，单位 kPa，保持肌肉的初始张力
#define BASE_PRESSURE 300.0f 

/**
 * @brief 初始化控制系统
 */
void arm_control_init(void) {
    // 1. 初始化角度环 PID
    // 输出: 压力差 (kPa)，假设最大允许差值为 150kPa
    pid_init(&pid_angle, 
             2.0f,  0.1f, 0.5f,   // Kp, Ki, Kd (需要根据实际调试)
             -150.0f, 150.0f);    // Output Min, Max
    pid_angle.dead_zone = 1.0f;   // 1度以内的误差忽略

    // 2. 初始化压力环 PID (肌肉A)
    // 输出: PWM 占空比 (0 ~ VALVE_MAX_DUTY)
    pid_init(&pid_press_A,
             15.0f, 0.5f, 0.0f,             // 压力环通常只需 PI 控制
             0.0f, (float)VALVE_MAX_DUTY);  // Output 0 ~ 8191

    // 3. 初始化压力环 PID (肌肉B)
    pid_init(&pid_press_B,
             15.0f, 0.5f, 0.0f,
             0.0f, (float)VALVE_MAX_DUTY);
             
    ESP_LOGI(TAG, "PID Controllers Initialized");
}

/**
 * @brief 设置目标角度
 * * @param angle 目标角度
 */
void arm_set_target_angle(float angle) {
    pid_angle.setpoint = angle;
}

/**
 * @brief 机械臂主控制循环任务
 * * @param pvParameters 参数
 */
void arm_control_task(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz 控制频率

    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // --- 1. 读取传感器数据 ---
        float current_angle = (float)as5600_get_angle(0); // 通道0
        
        // 读取气压 (kPa)
        float current_press_A = (float)pressure_read_kpa(0); // 假设通道0是肌肉A
        float current_press_B = (float)pressure_read_kpa(1); // 假设通道1是肌肉B

        // --- 2. 外环：位置环计算 ---
        // 目标：计算需要多大的“压力差”才能修正角度误差
        float delta_pressure = pid_compute(&pid_angle, current_angle);

        // --- 3. 压力分配 (拮抗控制) ---
        // 肌肉A 目标压力 = 基础压力 + delta
        // 肌肉B 目标压力 = 基础压力 - delta
        float target_press_A = BASE_PRESSURE + delta_pressure;
        float target_press_B = BASE_PRESSURE - delta_pressure;

        // 设置压力环的目标值
        pid_press_A.setpoint = target_press_A;
        pid_press_B.setpoint = target_press_B;

        // --- 4. 内环：压力环计算 ---
        // 计算阀门 PWM 占空比
        float duty_A = pid_compute(&pid_press_A, current_press_A);
        float duty_B = pid_compute(&pid_press_B, current_press_B);

        // --- 5. 执行控制 ---
        // 更新电磁阀 PWM
        // 这里假设肌肉只有进气阀控制压力，排气阀常开或由其他逻辑控制
        // 如果是标准的两位三通充放气控制，PID输出正值充气，负值放气，逻辑会更复杂
        // 这里简化为：单阀控制充气量，假设有微量排气或被动排气
        valve_set_duty(0, (uint32_t)duty_A); // 肌肉A 进气
        valve_set_duty(2, (uint32_t)duty_B); // 肌肉B 进气

        // 调试日志 (建议每 500ms 打印一次，不要太快)
        // ESP_LOGI(TAG, "Ang:%.1f Tgt:%.1f | P_A:%.0f T_A:%.0f | PWM_A:%d", 
        //          current_angle, pid_angle.setpoint, 
        //          current_press_A, target_press_A, (int)duty_A);

        // 保持 50Hz 循环频率
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}