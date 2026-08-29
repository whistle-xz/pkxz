//气泵控制模块，使用 GPIO 输出控制继电器
#include "hal_pump.h"
#include "hardware_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h> // 需要引入 bool 类型

static const char *TAG = "PUMP_HAL";


// 【新增】：气泵全局使能标志，默认通电时为 false (强制关闭状态)
static volatile bool pump_global_enable = false; 

// 【新增】：对外提供的气泵开关接口
void pump_set_enable(bool enable) {
    pump_global_enable = enable;
    if (!enable) {
        // 如果被设置为关闭，立即强制断开 MOS管/继电器
        gpio_set_level(PUMP_RELAY_PIN, 0);
        ESP_LOGI(TAG, "===== 气泵系统已强制【关闭】 =====");
    } else {
        ESP_LOGI(TAG, "===== 气泵系统已【启用】(交由压力开关自动控制) =====");
    }
}

bool pump_is_enabled(void) {
    return pump_global_enable;
}

// 初始化气泵和压力开关 GPIO
void pump_init(void) {
    // 1. 配置继电器 (输出)
    gpio_reset_pin(PUMP_RELAY_PIN);
    gpio_set_direction(PUMP_RELAY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(PUMP_RELAY_PIN, 0); // 默认物理层面也关闭

    // 2. 配置 QPM11 压力开关 (输入)
    gpio_reset_pin(PRESSURE_SWITCH_PIN);
    gpio_set_direction(PRESSURE_SWITCH_PIN, GPIO_MODE_INPUT);
    // 关键：启用内部上拉电阻
    gpio_set_pull_mode(PRESSURE_SWITCH_PIN, GPIO_PULLUP_ONLY);

    ESP_LOGI(TAG, "Pump Hardware Initialized (Relay: %d, Switch: %d)", PUMP_RELAY_PIN, PRESSURE_SWITCH_PIN);
}
/*
void pump_control_loop(void *pvParameters) {
    ESP_LOGI(TAG, "Pump Task Started");
    static int last_relay_state = 0; // 记录真实状态
    int stable_count = 0; // 消抖计数器

    while (1) {
        
        // 【新增】：如果气泵未使能，跳过压力判断，死死锁住关闭状态
        if (!pump_global_enable) {
            if (last_relay_state != 0) {
                gpio_set_level(PUMP_RELAY_PIN, 0);
                last_relay_state = 0;
            }
            stable_count = 0;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue; // 直接进入下一次循环
        }
         

        // --- 以下是原本的自动打气逻辑 ---
        int sw_state = gpio_get_level(PRESSURE_SWITCH_PIN);

        // 目标状态与当前继电器状态不同时，才开始计数消抖
        if ((sw_state == 1 && last_relay_state != 1) || 
            (sw_state == 0 && last_relay_state != 0)) {
            stable_count++;

            // 连续 5 次 (500ms) 状态稳定，才真正切换继电器
            if (stable_count >= 5) {
                if (sw_state == 1) {
                    gpio_set_level(PUMP_RELAY_PIN, 1); 
                    last_relay_state = 1;
                    ESP_LOGI(TAG, "气压偏低，气泵 [启动]");
                } else {
                    gpio_set_level(PUMP_RELAY_PIN, 0); 
                    last_relay_state = 0;
                    ESP_LOGI(TAG, "气压达标，气泵 [停止]");
                }
                stable_count = 0; // 动作后清零
            }
        } else {
            stable_count = 0; // 状态波动，重新计数
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
*/

void pump_control_loop(void *pvParameters) {
    ESP_LOGI(TAG, "Pump Task Started - 终极稳定版");
    
    // 简单的滤波计数器，防止机械开关触点抖动导致继电器疯狂“哒哒哒”
    int filter_cnt = 0; 

    while (1) {
        // ================= 第 1 步：总开关安全拦截 =================
        // 如果网页上没发 "C 1" (没有打勾)，强制关闭气泵
        if (!pump_global_enable) {
            gpio_set_level(PUMP_RELAY_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue; 
        }

        // ================= 第 2 步：读取真实气压状态 =================
        int sw_state = gpio_get_level(PRESSURE_SWITCH_PIN);

        // ================= 第 3 步：极简防抖控制 =================
        if (sw_state == 0) { 
            // 读到 0 (缺气短路)，计数器增加
            filter_cnt++;
            if (filter_cnt > 3) {
                gpio_set_level(PUMP_RELAY_PIN, 1); // 【1 = 开启气泵】
                filter_cnt = 3; // 封顶防溢出
            }
        } else {
            // 读到 1 (气满断开)，计数器减少
            filter_cnt--;
            if (filter_cnt < 0) {
                gpio_set_level(PUMP_RELAY_PIN, 0); // 【0 = 关闭气泵】
                filter_cnt = 0; // 触底防溢出
            }
        }

        // 每次循环延时 50ms，连续 3 次(150ms)状态一致才动作，完美过滤杂波
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}