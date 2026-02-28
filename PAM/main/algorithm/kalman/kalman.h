#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float x;    // 状态估计值
    float P;    // 估计协方差
    float Q;    // 过程噪声
    float R;    // 测量噪声
} KalmanFilter;

void kf_init(KalmanFilter *kf, float initial_value, float P, float Q, float R);
float kf_update(KalmanFilter *kf, float z);

#endif