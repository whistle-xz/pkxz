#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "hardware_config.h"
#include "arm_control.h"
#include "press.h"
#include "stm32_comm.h"
#include "as5600.h" 

static const char *TAG = "STM_COMM";
#define BUF_SIZE 128

void stm32_comm_task(void *pvParameters) {
    uart_config_t uart_config = {
        .baud_rate = 115200, 
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART0_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART0_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART0_PORT_NUM, UART0_TX_PIN, UART0_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uint8_t rx_data[BUF_SIZE];
    char line_buf[256];
    int line_idx = 0;

    TickType_t last_feedback_time = xTaskGetTickCount();

    ESP_LOGI(TAG, "STM32 UART Communication Task Started");

    while (1) {
        // ==================================================
        // 1. 接收 STM32 控制指令 (解析目标角度)
        // ==================================================
        int len = uart_read_bytes(UART0_PORT_NUM, rx_data, BUF_SIZE - 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = (char)rx_data[i];
                if (c == '\n' || c == '\r') {
                    if (line_idx > 0) {
                        line_buf[line_idx] = '\0'; 
                        
                        // 【协议升级】：解析 "A <大臂角度> <小臂角度>" 
                        float target_angle_upper, target_angle_lower;
                        uint32_t pC, pD; 

                        // 1. 解析 A 指令：更新目标角度和气压
                        if (sscanf(line_buf, "A %f %f %lu %lu", &target_angle_upper, &target_angle_lower, &pC, &pD) == 4) {
                            arm_set_target_angle(target_angle_upper, target_angle_lower); 
                            arm_set_remote_pressures(pC, pD); // 顺手把气压也更新了，最稳妥
                        }
                        // 2. 解析 P 指令：悄悄更新 C 和 D 的气压 (STM32每50ms后台自动发送)
                        // 格式: "P 250 180"
                        else if (sscanf(line_buf, "P %lu %lu", &pC, &pD) == 2) {
                            arm_set_remote_pressures(pC, pD);
                        }
                        
                        line_idx = 0; 
                    }
                } else {
                    if (line_idx < sizeof(line_buf) - 1) {
                        line_buf[line_idx++] = c;
                    } else {
                        line_idx = 0; // 防溢出
                    }
                }
            }
        }

        // ==================================================
        // 2. 定期向 STM32 发送状态反馈 (遥测数据)
        // ==================================================
        if (xTaskGetTickCount() - last_feedback_time >= pdMS_TO_TICKS(20)) {
            // 获取当前关节真实角度
            int16_t ang_upper = as5600_get_angle(0);
            int16_t ang_lower = as5600_get_angle(1);

            // 获取 4 根肌肉的真实气压
            uint32_t pA = get_filtered_pressure(0); 
            uint32_t pB = get_filtered_pressure(1); 
            uint32_t pC = arm_get_remote_press_c(); 
            uint32_t pD = arm_get_remote_press_d();
            
            // 【协议升级】：向 STM32 发送完整的状态链
            // 格式: S <大臂角度> <小臂角度> <压A> <压B> <压C> <压D>
            char tx_buf[128];
            snprintf(tx_buf, sizeof(tx_buf), "S %d %d %lu %lu %lu %lu\n", 
                     ang_upper, ang_lower, 
                     (unsigned long)pA, (unsigned long)pB, 
                     (unsigned long)pC, (unsigned long)pD);
            
            uart_write_bytes(UART0_PORT_NUM, tx_buf, strlen(tx_buf));
            last_feedback_time = xTaskGetTickCount();
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}