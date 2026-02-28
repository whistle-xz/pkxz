//气泵控制模块，使用 GPIO 输出控制继电器

#include "hal_pump.h"
#include "hardware_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "PUMP_HAL";

void pump_init(void) {
    // 1. 配置继电器 (输出)
    gpio_reset_pin(PUMP_RELAY_PIN);
    gpio_set_direction(PUMP_RELAY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PUMP_RELAY_PIN, 0); // 默认关闭

    // 2. 配置 QPM11 压力开关 (输入)
    gpio_reset_pin(PRESSURE_SWITCH_PIN);
    gpio_set_direction(PRESSURE_SWITCH_PIN, GPIO_MODE_INPUT);
    // 关键：启用内部上拉电阻
    // 因为 QPM11 另一端接 GND。闭合时接地(0)，断开时被拉高(1)。
    gpio_set_pull_mode(PRESSURE_SWITCH_PIN, GPIO_PULLUP_ONLY);

    ESP_LOGI(TAG, "Pump Hardware Initialized (Relay: %d, Switch: %d)", PUMP_RELAY_PIN, PRESSURE_SWITCH_PIN);
}

void pump_control_loop(void) {
    // 读取开关状态
    int sw_state = gpio_get_level(PRESSURE_SWITCH_PIN);

    // QPM11 (NC常闭) 逻辑:
    // 气压低 -> 开关闭合 -> 导通到GND -> 读到 0
    // 气压高 -> 开关断开 -> 内部上拉   -> 读到 1
    
    if (sw_state == 0) {
        // 气压不足，开启气泵
        gpio_set_level(PUMP_RELAY_PIN, 1);
        // ESP_LOGD(TAG, "Pressure LOW -> Pump ON");
    } else {
        // 气压已满，关闭气泵
        gpio_set_level(PUMP_RELAY_PIN, 0);
        // ESP_LOGD(TAG, "Pressure HIGH -> Pump OFF");
    }
}
