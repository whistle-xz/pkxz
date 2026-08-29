#include "hal_valves.h"
#include "hardware_config.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "VALVE_HAL";

static const int valve_gpios[8] = {
    VALVE_1_PIN, VALVE_2_PIN, VALVE_3_PIN, VALVE_4_PIN,
    VALVE_5_PIN, VALVE_6_PIN, VALVE_7_PIN, VALVE_8_PIN
};

// 缓存 8 个通道的占空比
static uint32_t current_duty[8] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                                   0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

void valves_init(void) {
    // 【第一步】：先独立、且仅初始化一次 4 个共享的定时器
    for (int i = 0; i < 4; i++) {
        ledc_timer_config_t ledc_timer = {
            .speed_mode       = VALVE_LEDC_MODE,
            .timer_num        = (ledc_timer_t)i, 
            .duty_resolution  = VALVE_LEDC_RES,
            .freq_hz          = VALVE_PWM_FREQ_HZ, 
            .clk_cfg          = LEDC_AUTO_CLK
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    }

    // 【第二步】：再初始化 8 个 PWM 通道，并挂载到已经稳定运行的定时器上
    for (int i = 0; i < 8; i++) {
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = VALVE_LEDC_MODE,
            .channel        = (ledc_channel_t)i, 
            .timer_sel      = (ledc_timer_t)(i % 4),  // 完美共享 0,1,2,3
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = valve_gpios[i],    
            .duty           = 0,                 
            .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
        
        // 强制刷新一次 0 占空比，确保 MOS 管绝对断电关死
        ESP_ERROR_CHECK(ledc_set_duty(VALVE_LEDC_MODE, (ledc_channel_t)i, 0));
        ESP_ERROR_CHECK(ledc_update_duty(VALVE_LEDC_MODE, (ledc_channel_t)i));
        current_duty[i] = 0;
    }
    
    ESP_LOGI(TAG, "8 PWM Valves Initialized Successfully!");
}

void valve_set_duty(int channel, uint32_t duty) {
    if (channel < 0 || channel > 7) return;
    
    // 防死锁绝杀
    if (duty >= VALVE_MAX_DUTY) duty = VALVE_MAX_DUTY - 1; 

    if (current_duty[channel] == duty) return; 
    current_duty[channel] = duty;

    ESP_ERROR_CHECK(ledc_set_duty(VALVE_LEDC_MODE, (ledc_channel_t)channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(VALVE_LEDC_MODE, (ledc_channel_t)channel));
}

void valve_open_full(int channel) { valve_set_duty(channel, VALVE_MAX_DUTY); }
void valve_close_full(int channel) { valve_set_duty(channel, 0); }