#include "as5600.h"
#include "hardware_config.h"
#include "kalman.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gpio.h"

static const char *TAG = "AS5600";
#define AS5600_ADDR 0x36
#define REG_ANGLE_H 0x0E
#define REG_ANGLE_L 0x0F

static KalmanFilter kf;
static uint16_t zero_offset = 0;
static uint8_t zero_set_flag = 0;

// 多路选择器通道切换
static void as5600_select_channel(int channel) {
    // 根据 hardware_config.h 中的定义设置 GPIO
    gpio_set_level(MUX_AS5600_A, channel & 0x01);
    gpio_set_level(MUX_AS5600_B, (channel >> 1) & 0x01);
    gpio_set_level(MUX_AS5600_C, (channel >> 2) & 0x01);
}

void as5600_init(void) {
    // 1. 配置 I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    // 2. 配置多路选择器引脚
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<MUX_AS5600_A) | (1ULL<<MUX_AS5600_B) | (1ULL<<MUX_AS5600_C),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    // 3. 初始化卡尔曼滤波
    kf_init(&kf, 0, 2, 1, 1);
    
    ESP_LOGI(TAG, "AS5600 Sensor Initialized");
}

static uint16_t read_raw_angle() {
    uint8_t data[2];
    uint8_t reg = REG_ANGLE_H; // 从高位寄存器开始读
    
    // I2C 写地址
    i2c_master_write_read_device(I2C_MASTER_NUM, AS5600_ADDR, &reg, 1, data, 2, pdMS_TO_TICKS(100));
    
    // 拼接高低位 (AS5600 数据格式: HighByte << 8 | LowByte)
    // 注意：你队友的代码是先读L再读H，标准做法通常是一次读2字节或者按序读
    // 这里使用标准 I2C 读取
    uint16_t angle = ((data[0] << 8) | data[1]) & 0x0FFF; // 12位精度
    return angle;
}

void as5600_set_zero(void) {
    as5600_select_channel(0); // 默认校准通道0
    uint16_t raw = read_raw_angle();
    zero_offset = raw;
    zero_set_flag = 1;
    ESP_LOGI(TAG, "Zero Point Set: %d", zero_offset);
}

int16_t as5600_get_angle(int channel) {
    as5600_select_channel(channel);
    
    uint16_t raw = read_raw_angle();
    
    // 卡尔曼滤波
    float filtered = kf_update(&kf, (float)raw);
    
    if (!zero_set_flag) return 0;

    // 计算相对角度
    int32_t diff = (int32_t)filtered - (int32_t)zero_offset;
    
    // 处理过零点问题 (0-4096 回绕)
    if (diff > 2048) diff -= 4096;
    else if (diff < -2048) diff += 4096;

    return (int16_t)diff;
}