#include "pid.h"

/**
 * @brief 初始化 PID 控制器参数
 * * @param pid 指向 PID 结构体的指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param min 输出最小值
 * @param max 输出最大值
 */
void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd, float min, float max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = min;
    pid->out_max = max;
    pid->int_limit = (max - min) * 0.5f; // 默认积分限幅为量程的一半
    pid->dead_zone = 0.0f;               // 默认无死区
    
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

/**
 * @brief 重置 PID 状态 (清除积分和历史误差)
 * * @param pid 指向 PID 结构体的指针
 */
void pid_reset(pid_ctrl_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

/**
 * @brief 计算 PID 输出
 * * @param pid 指向 PID 结构体的指针
 * @param measured 当前测量值 (反馈值)
 * @return float PID 计算出的控制量
 */
float pid_compute(pid_ctrl_t *pid, float measured) {
    float output;
    float error = pid->setpoint - measured;

    // 1. 死区处理 (误差极小时忽略，防止阀门频繁抖动)
    if (error > -pid->dead_zone && error < pid->dead_zone) {
        error = 0.0f;
    }

    // 2. 积分计算 (带抗饱和限幅)
    pid->integral += error;
    if (pid->integral > pid->int_limit) pid->integral = pid->int_limit;
    else if (pid->integral < -pid->int_limit) pid->integral = -pid->int_limit;

    // 3. 微分计算
    float derivative = error - pid->prev_error;

    // 4. PID 公式
    output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    // 5. 输出限幅
    if (output > pid->out_max) output = pid->out_max;
    else if (output < pid->out_min) output = pid->out_min;

    // 6. 保存当前误差供下次微分使用
    pid->prev_error = error;

    return output;
}