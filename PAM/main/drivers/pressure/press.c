#include "press.h"
#include "hardware_config.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "PRESS_SENSOR";
#define BUF_SIZE 1024

// 协议命令
#define CMD_CAL_P1 0x0D // 获取实时气压

// 多路选择
static void press_select_channel(int channel) {
    gpio_set_level(MUX_PRESS_A, channel & 0x01);
    gpio_set_level(MUX_PRESS_B, (channel >> 1) & 0x01);
    gpio_set_level(MUX_PRESS_C, (channel >> 2) & 0x01);
}

// CRC 校验算法 
static uint8_t calc_crc(uint8_t *arr, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
        crc ^= *arr++;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x01) crc = (crc >> 1) ^ 0x8c;
            else crc >>= 1;
        }
    }
    return crc;
}

void pressure_sensor_init(void) {
    // 1. 配置 UART
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, -1, -1);

    // 2. 配置 MUX 引脚
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<MUX_PRESS_A) | (1ULL<<MUX_PRESS_B) | (1ULL<<MUX_PRESS_C),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "Pressure Sensor Initialized");
}

uint32_t pressure_read_kpa(int channel) {
    press_select_channel(channel);
    
    // 1. 发送读取命令
    uint8_t cmd_buf[4] = {0x55, 0x04, CMD_CAL_P1, 0x00};
    cmd_buf[3] = calc_crc(cmd_buf, 3);
    uart_write_bytes(UART_PORT_NUM, (const char *)cmd_buf, 4);

    // 2. 等待响应
    vTaskDelay(pdMS_TO_TICKS(50)); // 传感器处理时间

    // 3. 读取数据
    uint8_t data[128];
    int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE, pdMS_TO_TICKS(50));

    if (len >= 5 && data[0] == 0xAA) {
        // 解析: AA(头) Len Type Data...
        // 假设数据类型是 P1 (0x09)
        // 注意数组索引保护
        if (len >= 8) { // 确保有足够数据
             uint32_t pressure = (data[5] << 16) | (data[4] << 8) | (data[3]);
             return pressure / 1000; // 返回 kPa
        }
    }
    
    // 读不到返回一个错误码或者上一次的值 (这里返回0表示异常)
    // ESP_LOGW(TAG, "Press Read Fail Ch:%d", channel);
    return 0; 
}