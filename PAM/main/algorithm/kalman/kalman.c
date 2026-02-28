#include "kalman.h"

void kf_init(KalmanFilter *kf, float initial_value, float P, float Q, float R) {
    kf->x = initial_value;
    kf->P = P;
    kf->Q = Q;
    kf->R = R;
}

float kf_update(KalmanFilter *kf, float z) {
    // 1. 预测更新
    kf->P = kf->P + kf->Q;
    // 2. 计算卡尔曼增益
    float K = kf->P / (kf->P + kf->R);
    // 3. 测量更新
    kf->x = kf->x + K * (z - kf->x);
    // 4. 更新误差协方差
    kf->P = (1.0f - K) * kf->P;

    return kf->x;
}