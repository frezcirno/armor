#pragma once
#include <cmath>
#include <iostream>
#include "debug.h"

constexpr float x = 0, y = 0, pitch = 0;
constexpr float g = 9.8;
constexpr float alpha = 1;
constexpr int iter_times = 30; 
/**
 * 弹道拟合
 */
class DDSolver {
  private:
    float k1;  // 和空气阻力有关的系数，等于(k0 / m)，pitchAdvance中用
    
  public:
    DDSolver(float k1 = 0.9) : k1(k1) {}

    /**
     * 只考虑重力的弹道模型，根据以下三个值得出pitch角
     * @param bulletSpeed 弹速，单位：m/s
     * @param x 目标离自己的水平距离，单位：m
     * @param y 目标离自己的垂直距离，正值表示比自己高，负值反之，单位：m
     * @retval 击打仰角，单位：rad
     */
    float pitchNaive(float bulletSpeed, float x, float y) {
        float iterY = y, pitch_;
        float diff;
        int t = 0;
        do {
            pitch_ = atan2(iterY, x);
            float vx_ = bulletSpeed * cos(pitch_);
            float vy_ = bulletSpeed * sin(pitch_);
            float t_ = x / vx_;
            float y_ = vy_ * t_ - 0.5 * g * t_ * t_;
            diff = y - y_;
            iterY += diff;
            // cout << pitch_ * 180 / M_PI << " " << iterY << " " << diff <<
            // endl;
        } while (abs(diff) >= 0.001 && ++t < 30);  // 误差小于1mm
        return pitch_;
    }

    /**
     * 考虑水平空气阻力（和重力）的弹道模型，根据以下三个值得出pitch角
     * @param bulletSpeed 弹速，单位：m/s
     * @param x 目标离自己的水平距离，单位：m
     * @param y 目标离自己的垂直距离，正值表示比自己高，负值反之，单位：m
     * @retval 击打仰角，单位：rad
     */
    bool pitchAdvance(float bulletSpeed, float x, float y,float& pitch_) {
        float iterY = y;
        float diff;
        int t = 0;
        if((x > bulletSpeed * bulletSpeed/ (2 * g))||(y > bulletSpeed * bulletSpeed / g)){
            return false;    
        }
        do {
            pitch_ = atan2(iterY, x);
            float vx_ = bulletSpeed * cos(pitch_);
            float vy_ = bulletSpeed * sin(pitch_);
            float t_ = (exp(x * k1) - 1) / (k1 * vx_);
            float y_ = vy_ * t_ - 0.5 * g * t_ * t_;
            diff = y - y_;
            iterY += alpha*diff;
            PRINT_INFO("%d %2f %d %d\n", y, pitch_ * 180 / M_PI, iterY, diff);
        } while (abs(diff) >= 0.001 && ++t < iter_times);  // 误差小于1mm
        return true;
    }

    /**
     * 在考虑水平空气阻力（和重力）的弹道模型中，根据一组已知的四个值反推k1
     * @param bulletSpeed 弹速，单位m/s
     * @param pitch 仰角，单位rad
     * @param x 目标离自己的水平距离，单位m
     * @param y 目标离自己的垂直距离，正值表示比自己高，负值反之，单位m
     */
    static float get_k1(float bulletSpeed, float pitch, float x, float y) {
        float vx0 = bulletSpeed * cos(pitch);
        float vy0 = bulletSpeed * sin(pitch);
        float t0 = (vy0 + sqrt(vy0 * vy0 - 2 * g * y)) / g;
        float k1 = 2 * (vx0 * t0 - x) / (x * x);
        float diff;
        float alpha = 0.01;  // 类似学习率
        int t = 0;
        do {
            float t_ = (exp(x * k1) - 1) / (k1 * vx0);
            float y_ = vy0 * t_ - 0.5 * g * t_ * t_;
            diff = y - y_;
            k1 -= alpha * diff;
            // cout << k1 << " " << diff << endl;
        } while (abs(diff) >= 0.001 && ++t < 1000);  // 误差小于1mm
        return k1;
    }
};
