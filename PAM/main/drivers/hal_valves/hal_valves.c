//电磁阀控制模块，使用 LEDC 进行 PWM 输出
//每个阀门对应一个 LEDC 通道，频率固定为 50Hz，分辨率为 13 位（0-8191）

#include "hal_valves.h"
#include "hardware_config.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "VALVE_HAL";

// 建立通道到 GPIO 的映射数组，方便索引
static const int valve_gpios[4] = {
    VALVE_A_IN_PIN, 
    VALVE_A_OUT_PIN, 
    VALVE_B_IN_PIN, 
    VALVE_B_OUT_PIN
};

void valves_init(void) {
    // 1. 配置定时器
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = VALVE_LEDC_MODE,
        .timer_num        = VALVE_LEDC_TIMER,
        .duty_resolution  = VALVE_LEDC_RES,
        .freq_hz          = VALVE_PWM_FREQ_HZ, 
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 2. 配置 4 个通道
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = VALVE_LEDC_MODE,
            .channel        = (ledc_channel_t)i, // 动态分配通道 0,1,2,3
            .timer_sel      = VALVE_LEDC_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = valve_gpios[i],    // 从数组取引脚
            .duty           = 0,                 // 默认关闭
            .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }
    ESP_LOGI(TAG, "PWM Valves Initialized (Freq: %dHz)", VALVE_PWM_FREQ_HZ);
}

void valve_set_duty(int channel, uint32_t duty) {
    if (channel < 0 || channel > 3) return;
    if (duty > VALVE_MAX_DUTY) duty = VALVE_MAX_DUTY;

    // 关键修复：使用传入的 channel，而不是写死 Channel 0
    ESP_ERROR_CHECK(ledc_set_duty(VALVE_LEDC_MODE, (ledc_channel_t)channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(VALVE_LEDC_MODE, (ledc_channel_t)channel));
}

void valve_open_full(int channel) {
    valve_set_duty(channel, VALVE_MAX_DUTY);
}

void valve_close_full(int channel) {
    valve_set_duty(channel, 0);
}