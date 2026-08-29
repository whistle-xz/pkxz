#include "arm_control.h"
#include "pid.h"
#include "hardware_config.h"
#include "press.h"
#include "hal_valves.h"
#include "hal_pump.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "as5600.h"
#include <math.h>

#define VALVE_MIN_DUTY 3600
#define DEAD_ZONE 25  
// 最大允许偏置气压：决定了肌肉最大能拉多大劲
#define MAX_DELTA_P 150.0f

static const char *TAG = "ARM_CTRL";

// --- 外环：角度 PID ---
static pid_ctrl_t pid_angle_upper; // 大臂角度
static pid_ctrl_t pid_angle_lower; // 小臂角度
static float target_angle_upper = 0.0f;
static float target_angle_lower = 0.0f;

// --- 内环：气压 PID ---
static pid_ctrl_t pid_press_A, pid_press_B; // 大臂拮抗组
static pid_ctrl_t pid_press_C, pid_press_D; // 小臂拮抗组

// 【新增】：用于接收 STM32 发过来的小臂气压数据
static uint32_t remote_press_C = BASE_PRESSURE;
static uint32_t remote_press_D = BASE_PRESSURE;

void arm_control_init(void) {
    // 1. 初始化 AS5600
    as5600_init();

    // 2. 初始化 4 个气压环 (使用我们之前的完美参数)
    pid_init(&pid_press_A, 15.0f, 10.0f, 0.5f, 3.0f, -(float)VALVE_MAX_DUTY, (float)VALVE_MAX_DUTY);
    pid_init(&pid_press_B, 15.0f, 10.0f, 0.5f, 3.0f, -(float)VALVE_MAX_DUTY, (float)VALVE_MAX_DUTY);
    pid_init(&pid_press_C, 8.0f, 5.0f, 0.1f, 1.0f, -(float)VALVE_MAX_DUTY, (float)VALVE_MAX_DUTY);
    pid_init(&pid_press_D, 8.0f, 5.0f, 0.1f, 1.0f, -(float)VALVE_MAX_DUTY, (float)VALVE_MAX_DUTY);
    
    // 3. 初始化 2 个角度环
    // 角度环输出的是 "Delta P" (需要增加或减少多少气压差)
    /*原始pid
    pid_init(&pid_angle_upper, 10.0f, 10.0f, 0.1f, 1.0f, -MAX_DELTA_P, MAX_DELTA_P);
    pid_init(&pid_angle_lower, 5.0f, 5.0f, 0.05f, 1.0f, -MAX_DELTA_P, MAX_DELTA_P);
    */
   
    pid_init(&pid_angle_upper, 10.0f, 10.0f, 0.1f, 1.5f, -MAX_DELTA_P, MAX_DELTA_P);
    pid_init(&pid_angle_lower, 5.0f, 5.0f, 0.1f, 1.0f, -MAX_DELTA_P, MAX_DELTA_P);
    
    pid_angle_upper.dead_zone = 1.5f;
    pid_angle_lower.dead_zone = 2.5f; // 小臂更轻，容忍度设大一点
        
    ESP_LOGI(TAG, "Cascaded Control Initialized!");

    vTaskDelay(pdMS_TO_TICKS(1000)); 
    as5600_set_zero(0); // 确立大臂零点
    as5600_set_zero(1); // 确立小臂零点
}

// 供串口调用的【角度】设定函数
void arm_set_target_angle(float upper_deg, float lower_deg) {
    if (target_angle_upper == upper_deg && target_angle_lower == lower_deg) {
        return; // 如果角度没变，直接拦截，绝不重置 PID！
    }

    target_angle_upper = upper_deg;
    target_angle_lower = lower_deg;

    pid_angle_upper.setpoint = upper_deg;
    pid_angle_lower.setpoint = lower_deg;
    
    // 清空两个环的历史积分
    pid_reset(&pid_angle_upper); 
    pid_reset(&pid_angle_lower);

    pid_reset(&pid_press_A); 
    pid_reset(&pid_press_B);
    pid_reset(&pid_press_C); 
    pid_reset(&pid_press_D);
    
    ESP_LOGI(TAG, "Target Angle Updated: Upper=%.1f, Lower=%.1f", upper_deg, lower_deg);
}

// 【新增】：供 stm32_comm 调用的【外援气压】设定函数
void arm_set_remote_pressures(uint32_t press_c, uint32_t press_d) {
    remote_press_C = press_c;
    remote_press_D = press_d;
}

// ======================= 肌肉驱动宏函数 =======================
// 为了代码整洁，将咱们那个完美的肌肉驱动逻辑封装成内联函数
static inline void drive_muscle(float ctrl, int ch_inflate, int ch_exhaust) {
    if (ctrl > DEAD_ZONE) {  
        // ====== 充气状态 (Inflate) ======
        // 前面的充气阀开启PWM，控制进气速度
        // 后面的排气阀必须全开(通电 MAX)，打通气路让气进入肌肉，并堵死排气口 R
        uint32_t duty = VALVE_MIN_DUTY + (uint32_t)((ctrl - DEAD_ZONE) * (VALVE_MAX_DUTY - VALVE_MIN_DUTY) / (VALVE_MAX_DUTY - DEAD_ZONE));
        if (duty > VALVE_MAX_DUTY) duty = VALVE_MAX_DUTY;
        
        valve_set_duty(ch_inflate, duty);           
        valve_set_duty(ch_exhaust, VALVE_MAX_DUTY); // 【修正】：守门员必须通电放行！
        
    } else if (ctrl < -DEAD_ZONE) {
        // ====== 排气状态 (Exhaust) ======
        // 前面的充气阀彻底断电关死，切断气源
        valve_set_duty(ch_inflate, 0); 
        
        // 我们计算出的 duty 越大，代表误差越大，需要排气越快。
        // 因为排气阀断电(0)才是全速排气，所以实际给的 PWM 应该“反相”：
        // 需要快排时给接近 0，需要慢排时给接近 MAX。
        uint32_t duty = VALVE_MIN_DUTY + (uint32_t)((-ctrl - DEAD_ZONE) * (VALVE_MAX_DUTY - VALVE_MIN_DUTY) / (VALVE_MAX_DUTY - DEAD_ZONE));
        if (duty > VALVE_MAX_DUTY) duty = VALVE_MAX_DUTY;
        
        valve_set_duty(ch_exhaust, VALVE_MAX_DUTY - duty); // 【修正】：反相控制排气速度
        
    } else {
        // ====== 死区保压状态 (Hold) ======
        // 前面的充气阀断电关死，切断气源
        // 后面的排气阀必须全开(通电 MAX)，堵死排气口，把气锁在肌肉内！
        valve_set_duty(ch_inflate, 0);              
        valve_set_duty(ch_exhaust, VALVE_MAX_DUTY); // 【修正】：守门员通电锁死气压！
    }
}
void arm_control_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); 
    uint8_t print_counter = 0;

    while (1) {
        // ================= 第零步：全局安全休眠拦截 =================
        // 如果气泵被强制关闭（发了 C 0，或者刚上电）
        if (!pump_is_enabled()) {
            // 充气阀全关 (断电 0)
            valve_set_duty(VALVE_A_INFLATE_CH , 0);
            valve_set_duty(VALVE_B_INFLATE_CH , 0);
            valve_set_duty(VALVE_C_INFLATE_CH , 0);
            valve_set_duty(VALVE_D_INFLATE_CH , 0);
            
            // 排气阀全开 (通电 MAX) -> 堵死排气孔，实现断气但保压！
            valve_set_duty(VALVE_A_EXHAUST_CH , 0);
            valve_set_duty(VALVE_B_EXHAUST_CH , 0);
            valve_set_duty(VALVE_C_EXHAUST_CH , 0);
            valve_set_duty(VALVE_D_EXHAUST_CH , 0);
            
            // ... 清除 PID ...
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue; 
        }

        // ================= 第一步：读取传感器 =================
        int16_t current_angle_upper = as5600_get_angle(0); // 大臂角度
        int16_t current_angle_lower = as5600_get_angle(1); // 小臂角度
        
        uint32_t press_A = get_filtered_pressure(0); // ESP32自己读
        uint32_t press_B = get_filtered_pressure(1); // ESP32自己读
        
        // 【修改点】：C 和 D 肌肉的气压直接使用 STM32 通过串口送过来的值！
        uint32_t press_C = remote_press_C; 
        uint32_t press_D = remote_press_D;

        // ================= 第二步：外环计算 (角度 PID) =================
        pid_angle_upper.setpoint = target_angle_upper;
        pid_angle_lower.setpoint = target_angle_lower;

        // 计算需要的偏置气压
        float delta_P_upper = pid_compute(&pid_angle_upper, (float)current_angle_upper);
        float delta_P_lower = pid_compute(&pid_angle_lower, (float)current_angle_lower);

        // 将偏置气压映射到两根肌肉，维持 BASE_PRESSURE
        float target_P_A = BASE_PRESSURE + delta_P_upper;
        float target_P_B = BASE_PRESSURE - delta_P_upper;
        float target_P_C = BASE_PRESSURE - delta_P_lower;
        float target_P_D = BASE_PRESSURE + delta_P_lower;

        //防止气压无限上升
        #define MAX_SAFE_PRESSURE 460.0f
        #define MIN_SAFE_PRESSURE 90.0f // 大气压左右

        // 限制大臂气压范围
        if (target_P_A > MAX_SAFE_PRESSURE) target_P_A = MAX_SAFE_PRESSURE;
        if (target_P_A < MIN_SAFE_PRESSURE) target_P_A = MIN_SAFE_PRESSURE;
        
        if (target_P_B > MAX_SAFE_PRESSURE) target_P_B = MAX_SAFE_PRESSURE;
        if (target_P_B < MIN_SAFE_PRESSURE) target_P_B = MIN_SAFE_PRESSURE;

        // 限制小臂气压范围
        if (target_P_C > MAX_SAFE_PRESSURE) target_P_C = MAX_SAFE_PRESSURE;
        if (target_P_C < MIN_SAFE_PRESSURE) target_P_C = MIN_SAFE_PRESSURE;

        if (target_P_D > MAX_SAFE_PRESSURE) target_P_D = MAX_SAFE_PRESSURE;
        if (target_P_D < MIN_SAFE_PRESSURE) target_P_D = MIN_SAFE_PRESSURE;

        // ================= 第三步：内环计算 (气压 PID) =================
        pid_press_A.setpoint = target_P_A;
        pid_press_B.setpoint = target_P_B;
        pid_press_C.setpoint = target_P_C;
        pid_press_D.setpoint = target_P_D;

        float ctrl_A = pid_compute(&pid_press_A, (float)press_A);
        float ctrl_B = pid_compute(&pid_press_B, (float)press_B);
        float ctrl_C = pid_compute(&pid_press_C, (float)press_C);
        float ctrl_D = pid_compute(&pid_press_D, (float)press_D);

        // 【抗抖动绝杀】：误差<3时，死区拦截
        if (abs((int)target_P_A - (int)press_A) <= 3) { ctrl_A = 0; pid_reset(&pid_press_A); }
        if (abs((int)target_P_B - (int)press_B) <= 3) { ctrl_B = 0; pid_reset(&pid_press_B); }
        if (abs((int)target_P_C - (int)press_C) <= 10) { ctrl_C = 0; pid_reset(&pid_press_C); }
        if (abs((int)target_P_D - (int)press_D) <= 10) { ctrl_D = 0; pid_reset(&pid_press_D); }

        // 打印调试信息 (打印角度，方便你上位机调试)
        if (++print_counter >= 10) {
            ESP_LOGI(TAG, "ANG U:%.1f->%d L:%.1f->%d | PRS A:%.0f B:%.0f C:%.0f D:%.0f", 
                     target_angle_upper, current_angle_upper, 
                     target_angle_lower, current_angle_lower,
                     (float)press_A, (float)press_B, (float)press_C, (float)press_D);
            print_counter = 0; 
        }

        // ================= 第四步：驱动阀门 =================
        // 传入参数：(控制量, 充气通道, 排气通道)
        drive_muscle(ctrl_A, 0, 2); // 肌肉 A: V1, V3
        drive_muscle(ctrl_B, 1, 3); // 肌肉 B: V2, V4
        drive_muscle(ctrl_C, 4, 6); // 肌肉 C: V5, V7
        drive_muscle(ctrl_D, 5, 7); // 肌肉 D: V6, V8

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// （保留你原来的调参函数，方便你想测气压PID的时候用）
void arm_tune_pressure_pid(float kp_inf, float kp_def, float ki, float kd) {
    pid_press_A.kp_inflate = kp_inf; pid_press_B.kp_inflate = kp_inf; 
    pid_press_C.kp_inflate = kp_inf; pid_press_D.kp_inflate = kp_inf; 
    
    pid_press_A.kp_deflate = kp_def; pid_press_B.kp_deflate = kp_def; 
    pid_press_C.kp_deflate = kp_def; pid_press_D.kp_deflate = kp_def; 

    pid_press_A.ki = ki; pid_press_A.kd = kd;
    pid_press_B.ki = ki; pid_press_B.kd = kd;
    pid_press_C.ki = ki; pid_press_C.kd = kd;
    pid_press_D.ki = ki; pid_press_D.kd = kd;
    
    pid_reset(&pid_press_A); pid_reset(&pid_press_B);
    pid_reset(&pid_press_C); pid_reset(&pid_press_D);
    ESP_LOGW(TAG, "Pressure PID Updated: P_inf=%.2f P_def=%.2f I=%.2f D=%.2f", 
             kp_inf, kp_def, ki, kd);
}

uint32_t arm_get_remote_press_c(void) {
    return remote_press_C;
}

uint32_t arm_get_remote_press_d(void) {
    return remote_press_D;
}