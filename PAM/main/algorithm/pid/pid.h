#ifndef PID_H
#define PID_H

#include <stdint.h>

// 初始化 PID 结构体
typedef struct {
    float kp_inflate;   ///< 充气比例系数 (当 目标值 > 测量值 时使用)
    float kp_deflate;   ///< 排气比例系数 (当 目标值 < 测量值 时使用)
    float ki;           
    float kd;           
    
    float setpoint;     
    float integral;     
    float prev_measured;   
    
    float out_min;      
    float out_max;      
    float int_limit;    ///< 积分限幅 (抗饱和)
    float dead_zone;    ///< 死区 (误差在此范围内不动作)
} pid_ctrl_t;

// 【注意】参数增加了一个 kp_deflate
void pid_init(pid_ctrl_t *pid, float kp_inflate, float kp_deflate, float ki, float kd, float min, float max);

void pid_reset(pid_ctrl_t *pid);

float pid_compute(pid_ctrl_t *pid, float measured);

#endif