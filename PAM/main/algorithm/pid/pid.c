#include "pid.h"

/**
 * @brief 初始化 PID 控制器参数
 */
void pid_init(pid_ctrl_t *pid, float kp_inflate, float kp_deflate, float ki, float kd, float min, float max) {
    pid->kp_inflate = kp_inflate;
    pid->kp_deflate = kp_deflate;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = min;
    pid->out_max = max;
    pid->int_limit = (max - min) * 0.5f; 
    pid->dead_zone = 0.0f;               
    
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_measured = 0.0f;
}

void pid_reset(pid_ctrl_t *pid) {
    pid->integral = 0.0f;
    pid->prev_measured = 0.0f;
}

/**
 * @brief 计算 PID 输出
 */
float pid_compute(pid_ctrl_t *pid, float measured) {
    float output;
    float error = pid->setpoint - measured;

    // 1. 死区处理 
    if (error > -pid->dead_zone && error < pid->dead_zone) {
        error = 0.0f;
    }

    // 2. 积分计算 (带抗饱和限幅)
    pid->integral += error;
    if (pid->integral > pid->int_limit) pid->integral = pid->int_limit;
    else if (pid->integral < -pid->int_limit) pid->integral = -pid->int_limit;

    // 3. 微分计算
    float derivative = pid->prev_measured - measured;

    // 4. 【核心修改】双 Kp 逻辑切换
    float current_kp;
    if (error > 0) {
        // error > 0 说明 目标值 > 实际值，需要充气
        current_kp = pid->kp_inflate;
    } else {
        // error < 0 说明 目标值 < 实际值，需要排气降压
        current_kp = pid->kp_deflate;
    }

    // 5. PID 公式
    output = (current_kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    // 6. 输出限幅
    if (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;

    // 7. 保存当前误差供下次微分使用
    pid->prev_measured = measured;

    return output;
}