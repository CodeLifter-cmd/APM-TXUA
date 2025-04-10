// libraries/AP_AOA/AP_AOAFilter.h
#pragma once

class AOAKalmanFilter
{
public:
    // 使用原生二维数组替代MatrixN
    float P[2][2]; // 协方差矩阵
    float Q[2][2]; // 过程噪声
    float R[2][2]; // 测量噪声
    float x[2];    // 状态向量 [角度(rad), 距离(m)]
    AOAKalmanFilter();
    void reset();
    void predict(float dt);
    void update(float meas_angle, float meas_dist);
    float get_angle() const { return x[1]; }
    float get_distance() const { return x[0]; }
};
