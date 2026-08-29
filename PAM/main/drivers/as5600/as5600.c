#include "as5600.h"
#include "hardware_config.h"
#include "kalman.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "AS5600";
#define AS5600_ADDR 0x36
#define REG_ANGLE_H 0x0E

static KalmanFilter kf_upper; // 大臂卡尔曼
static KalmanFilter kf_lower; // 小臂卡尔曼
static uint16_t zero_offset[2] = {0, 0}; 
static uint8_t zero_set_flag[2] = {0, 0};

#define OFFSET_UPPER_RAW  2700  // 大臂绝对零点
#define OFFSET_LOWER_RAW  94    // 小臂绝对零点

void as5600_init(void) {
    // 1. 初始化大臂编码器所在的 I2C_0
    i2c_config_t conf0 = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C0_SDA_IO,
        .scl_io_num = I2C0_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_PORT_UPPER, &conf0);
    i2c_driver_install(I2C_PORT_UPPER, conf0.mode, 0, 0, 0);

    // 2. 初始化小臂编码器所在的 I2C_1
    i2c_config_t conf1 = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C1_SDA_IO,
        .scl_io_num = I2C1_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_PORT_LOWER, &conf1);
    i2c_driver_install(I2C_PORT_LOWER, conf1.mode, 0, 0, 0);

    // 3. 初始化两组卡尔曼滤波
    kf_init(&kf_upper, 0, 2, 1, 1);
    kf_init(&kf_lower, 0, 2, 1, 1);
    
    ESP_LOGI(TAG, "Dual AS5600 Sensors Initialized on I2C0 and I2C1");
}

// 核心读取函数，根据传入的通道选择不同的 I2C 端口
static uint16_t read_raw_angle(int channel) {
    uint8_t data[2] = {0};
    uint8_t reg = REG_ANGLE_H; 
    
    // 动态选择使用哪个 I2C 接口
    i2c_port_t i2c_port = (channel == 0) ? I2C_PORT_UPPER : I2C_PORT_LOWER;
    
    // 读数据
    esp_err_t err = i2c_master_write_read_device(i2c_port, AS5600_ADDR, &reg, 1, data, 2, pdMS_TO_TICKS(10));
    
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "CH%d Read Failed!", channel);
        return 0; // 传感器可能没接好
    }
    
    return ((data[0] << 8) | data[1]) & 0x0FFF; 
}

void as5600_set_zero(int channel) {
    if (channel < 0 || channel > 1) return;
    
    zero_offset[channel] = read_raw_angle(channel);
    zero_set_flag[channel] = 1;
    ESP_LOGI(TAG, "Zero Point Set for CH%d: %d", channel, zero_offset[channel]);
}

int16_t as5600_get_angle(int channel) {
    if (channel < 0 || channel > 1) return 0;

    uint16_t raw = read_raw_angle(channel);
    
    KalmanFilter *kf = (channel == 0) ? &kf_upper : &kf_lower;
    float filtered = kf_update(kf, (float)raw);
    
    // 【核心修改 1】：抛弃 zero_offset 数组和标志位，直接强行使用绝对零点！
    int32_t offset = (channel == 0) ? OFFSET_UPPER_RAW : OFFSET_LOWER_RAW;
    int32_t diff = (int32_t)filtered - offset;

    
    // 处理过零点回绕 (0-4096)
    if (diff > 2048) diff -= 4096;
    else if (diff < -2048) diff += 4096;

    float degree = (diff * 360.0f) / 4096.0f;
    
    // 【核心修改 2】：统一坐标系方向
    // 根据上一轮的分析，为了和你的 3D 运动学模型完全对齐，大小臂都需要反转！
    if (channel == 0) {
        return (int16_t)(-degree); // 大臂：往前倾斜为正
    } else {
        return (int16_t)(degree); // 小臂：往下垂为正，往上收缩(夹角变小)为负
    }
}