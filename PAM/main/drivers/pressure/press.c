#include "press.h"
#include "hardware_config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "kalman.h"  

static const char *TAG = "PRESS_SENSOR";

// 全局变量，保存最新滤波后的气压值
static volatile uint32_t current_press_A = 0;
static volatile uint32_t current_press_B = 0;

// 定义卡尔曼滤波器实例
static KalmanFilter kf_A;
static KalmanFilter kf_B;

void pressure_sensor_init(void) {
    // ... [保留你原有的 UART 初始化代码保持不变] ...
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART1_PORT_NUM, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART1_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART1_PORT_NUM, UART1_TX_PIN, UART1_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(UART2_PORT_NUM, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART2_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART2_PORT_NUM, UART2_TX_PIN, UART2_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // 初始化卡尔曼滤波器 
    // 参数：(实例, 初始值, 初始协方差P, 过程噪声Q, 测量噪声R)
    // Q越小越平滑，R越大越平滑。你可以根据实际抖动调整 Q 和 R
    kf_init(&kf_A, BASE_PRESSURE, 1.0f, 0.1f, 5.0f); 
    kf_init(&kf_B, BASE_PRESSURE, 1.0f, 0.1f, 5.0f);

    ESP_LOGI(TAG, "Pressure Sensors UART & Kalman Filter Initialized");
}

// 内部函数：读取单次原始气压 (去掉了冗长的错误打印，避免刷屏)
static uint32_t read_raw_pressure(int channel) {
    uart_port_t uart_num = (channel == 0) ? UART1_PORT_NUM : UART2_PORT_NUM;
    uint8_t data[32];
    
    uart_flush_input(uart_num); 
    uint8_t cmd_press[4] = {0x55, 0x04, 0x0D, 0x88};
    uart_write_bytes(uart_num, (const char *)cmd_press, 4);

    // 超时时间缩短为 30ms，保证循环帧率
    int len = uart_read_bytes(uart_num, data, 32, pdMS_TO_TICKS(30));

    if (len >= 8 && data[0] == 0xAA && data[2] == 0x09) { 
        uint32_t pressure_raw = (data[5] << 16) | (data[4] << 8) | (data[3]);
        uint32_t pressure_kpa = pressure_raw / 1000; 
        if (pressure_kpa > 800) return PRESS_SENSOR_ERROR;
        return pressure_kpa;
    }
    return PRESS_SENSOR_ERROR;
}

// 供外部极速读取的接口
uint32_t get_filtered_pressure(int channel) {
    return (channel == 0) ? current_press_A : current_press_B;
}

// 独立的传感器读取任务
void sensor_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz 读取频率

    while (1) {
        uint32_t raw_A = read_raw_pressure(0);
        uint32_t raw_B = read_raw_pressure(1);

        if (raw_A != PRESS_SENSOR_ERROR) {
            current_press_A = (uint32_t)kf_update(&kf_A, (float)raw_A);
        }
        if (raw_B != PRESS_SENSOR_ERROR) {
            current_press_B = (uint32_t)kf_update(&kf_B, (float)raw_B);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
