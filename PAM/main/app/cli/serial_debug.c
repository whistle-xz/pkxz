#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "arm_control.h" 
#include "serial_debug.h"
#include "hal_pump.h"

static const char *TAG = "CLI";
#define BUF_SIZE 128

// 初始化 USB Serial JTAG
void cli_init(void) {
    ESP_LOGI(TAG, "Standard I/O CLI Initialized (via USB JTAG)");
}

// 解析命令函数
// 支持格式: 
// "P 10.0 0.5 0.0" -> 修改压力环 PID
// "T 250 150"      -> 修改目标气压: 肌肉A 250kPa, 肌肉B 150kPa
// 在 serial_debug.c 中
void parse_command(char *cmd) {
    char type;
    // 增加 val4
    float val1, val2, val3, val4;
    int n;

    if (sscanf(cmd, "%c", &type) != 1) return;

    switch (type) {
        case 'P':
        case 'p':
            // 解析 4 个浮点数: kp_inf, kp_def, ki, kd
            n = sscanf(cmd, "%*c %f %f %f %f", &val1, &val2, &val3, &val4);
            if (n == 4) {
                arm_tune_pressure_pid(val1, val2, val3, val4);
            } else {
                ESP_LOGW(TAG, "Invalid PID command: need 4 numbers (Kp_inf, Kp_def, Ki, Kd)");
            }
            break;
        case 'A':
        case 'a':
            // 设定目标角度命令: "A 45.0 -30.0" -> 大臂 45度，小臂 -30度
            n = sscanf(cmd, "%*c %f %f", &val1, &val2);
            if (n == 2) {
                arm_set_target_angle(val1, val2);
            } else {
                ESP_LOGW(TAG, "Invalid angle cmd: need 2 numbers");
            }
            break;
        default:
            ESP_LOGW(TAG, "Unknown Cmd: %s", cmd);
            break;
        
        case 'C':
        case 'c':
            // 设定气泵开关命令: "C 1" 打开，"C 0" 关闭
            n = sscanf(cmd, "%*c %f", &val1);
            if (n == 1) {
                if (val1 > 0.5f) {
                    pump_set_enable(true);
                } else {
                    pump_set_enable(false);
                }
            }
            break;
            
    }
}

/**
 * @brief USB 串口调试任务 (CLI)
 * @note 应在 FreeRTOS 中作为独立任务运行
 * @param pvParameters 任务参数 (未使用，传 NULL)
 */
void cli_task(void *pvParameters) {
    cli_init();
    char line_buf[BUF_SIZE];

    while (1) {
        // 使用标准 C 库从控制台（USB）读取一行，此函数会阻塞等待，不浪费 CPU
        if (fgets(line_buf, sizeof(line_buf), stdin) != NULL) {
            
            // 去除末尾的换行符 \r 或 \n
            line_buf[strcspn(line_buf, "\r\n")] = '\0';
            
            if (strlen(line_buf) > 0) {
                ESP_LOGI(TAG, "Recv: %s", line_buf);
                parse_command(line_buf);
            }
        }
        // 由于 fgets 是阻塞的，为了防止任务卡死某些机制，加上一点微小延时
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}