#include "AP_AOAFilter.h"
#include "AP_Math/AP_Math.h"
AOAKalmanFilter::AOAKalmanFilter()
{
    reset();
}

void AOAKalmanFilter::reset()
{
    // 初始化协方差矩阵
    // for (int i = 0; i < 2; i++)
    // {
    //     for (int j = 0; j < 2; j++)
    //     {
    //         P[i][j] = (i == j) ? 10.0f : 0.0f; // 对角阵
    //     }
    // }
    P[0][0] = 0.05f;
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0017f;
    // 过程噪声矩阵
    Q[0][0] = 0.1f;
    Q[0][1] = 0.0f;
    Q[1][0] = 0.0f;
    Q[1][1] = 0.1f;

    // 测量噪声矩阵
    R[0][0] = 0.0025f;
    R[0][1] = 0.0f;
    R[1][0] = 0.0f;
    R[1][1] = 0.001f;

    // 状态向量
    x[0] = 0.0f; // 初始角度
    x[1] = 1.0f; // 初始距离
}

void AOAKalmanFilter::predict(float dt)
{
    // 手动实现矩阵运算
    float F[2][2] = {{1, -dt}, {0, 1}};
    float FP[2][2], FPFt[2][2];

    // F * P
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            FP[i][j] = F[i][0] * P[0][j] + F[i][1] * P[1][j];
        }
    }

    // (F*P) * F^T
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            FPFt[i][j] = FP[i][0] * F[j][0] + FP[i][1] * F[j][1];
        }
    }

    // 更新协方差矩阵
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            P[i][j] = FPFt[i][j] + Q[i][j];
        }
    }
}

void AOAKalmanFilter::update(float meas_angle, float meas_dist)
{
    // 手动实现卡尔曼增益计算
    float S[2][2], K[2][2];
    float y[2] = {meas_angle - x[0], meas_dist - x[1]};

    // 计算S = H*P*H' + R (H=I)
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            S[i][j] = P[i][j] + R[i][j];
        }
    }

    // 计算卡尔曼增益K = P * inv(S)
    float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (fabsf(det) < 1e-5)
        return; // 防止奇异矩阵

    float invS[2][2] = {
        {S[1][1] / det, -S[0][1] / det},
        {-S[1][0] / det, S[0][0] / det}};

    // K = P * invS
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            K[i][j] = P[i][0] * invS[0][j] + P[i][1] * invS[1][j];
        }
    }

    // 状态更新：x = x + K*y
    float x_new[2] = {0};
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            x_new[i] += K[i][j] * y[j];
        }
        x[i] += x_new[i];
    }

    // 协方差更新：P = (I - K)*P
    float I_minus_K[2][2] = {
        {1 - K[0][0], -K[0][1]},
        {-K[1][0], 1 - K[1][1]}};
    float new_P[2][2] = {0};

    // 计算 (I-K)*P
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            for (int k = 0; k < 2; k++)
            {
                new_P[i][j] += I_minus_K[i][k] * P[k][j];
            }
        }
    }

    // 更新协方差矩阵
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            P[i][j] = new_P[i][j];
        }
    }
} // end update()
