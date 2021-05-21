#pragma once
#include "debug.h"
#include <cmath>
#include <iostream>

constexpr float g = 9.8;
constexpr float yself = 0.38;
/**
 * 弹道拟合
 */
class DDSolver {
  private:
    float k1;  // 和空气阻力有关的系数，等于(k0 / m)，pitchAdvance中用

  public:
    DDSolver(float k1 = 0.10) : k1(k1) {
        // k1 = DDSolver::get_k1(12, 10 * M_PI / 180, 3, 0.5); // 0.216194
        // PRINT_INFO("k1=%f\n", k1);
        // exit(0);
    }

    /**
     * 只考虑重力的弹道模型，根据以下三个值得出pitch角
     * @param bulletSpeed 弹速，单位：m/s
     * @param x 目标离自己的水平距离，单位：m
     * @param y 目标离自己的垂直距离，正值表示比自己高，负值反之，单位：m
     * @retval 击打仰角，单位：rad
     */
    bool pitchNaive(float bulletSpeed, float x, float y, float &pitch_) {
        float iterY = y;
        float delta = 0x3f3f3f3f, lastDelta;
        float alpha = 1;
        int t = 0;
        do {
            pitch_ = atan2(iterY, x);
            float vx_ = bulletSpeed * cos(pitch_);
            float vy_ = bulletSpeed * sin(pitch_);
            float t_ = x / vx_;
            float y_ = vy_ * t_ - 0.5 * g * t_ * t_;
            lastDelta = delta;
            delta = y - y_;
            if (std::abs(delta) > std::abs(lastDelta)) {
                alpha /= 10.0;
                if (alpha < 0.001) {
                    break;
                }
            }
            iterY += alpha * delta;
            // std::cout << pitch_ * 180 / M_PI << " " << iterY << " " << delta << std::endl;
        } while (abs(delta) >= 0.001 && ++t < 30);  // 误差小于1mm
        return true;
    }

    /**
     * 考虑水平空气阻力（和重力）的弹道模型，根据以下三个值得出pitch角
     * @param bulletSpeed 弹速，单位：m/s
     * @param x 目标离自己的水平距离，单位：m
     * @param y 目标离自己的垂直距离，正值表示比自己高，负值反之，单位：m
     * @retval 击打仰角，单位：rad
     */
    bool pitchAdvance(float bulletSpeed, float x, float y, float &pitch_) {
        using namespace std;

        float iterY = y;
        float delta = 0x3f3f3f3f, lastDelta;
        float alpha = 1;
        int t = 0;
        // PRINT_INFO("\n\n\n");
        // PRINT_INFO("x: %f, y: %f, k1=%f\n", x, y, k1);
        do {
            pitch_ = atan2(iterY, x);
            float vx_ = bulletSpeed * cos(pitch_);
            float vy_ = bulletSpeed * sin(pitch_);
            float t_ = (std::exp(x * k1) - 1) / (k1 * vx_);
            float y_ = vy_ * t_ - 0.5 * g * t_ * t_;
            lastDelta = delta;
            delta = y - y_;
            if (std::abs(delta) > std::abs(lastDelta)) {
                alpha /= 10.0;
                if (alpha < 0.001) {
                    break;
                }
            }
            iterY += alpha * delta;
            // PRINT_INFO("pitch=%2f, vx=%f, vy=%f, t=%f, iterY=%f, delta=%f, y_=%f \n", pitch_ * 180 / M_PI, vx_, vy_, t_, iterY, delta, y_);
        } while (abs(delta) >= 0.001 && ++t < 60);  // 误差小于1mm
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
        y -= yself;
        float vx0 = bulletSpeed * cos(pitch);
        float vy0 = bulletSpeed * sin(pitch);
        float t0 = (vy0 + sqrt(vy0 * vy0 - 2 * g * y)) / g;
        float k1 = 2 * (vx0 * t0 - x) / (x * x);
        float delta;
        int t = 0;
        float alpha = 0.005;  // 类似学习率
        PRINT_INFO("bulletSpeed=%f, pitch=%f, x=%f, y=%f, vx0=%f, vy0=%f, t0=%f, k1=%f\n", bulletSpeed, pitch, x, y, vx0, vy0, t0, k1);
        do {
            float t_ = (exp(x * k1) - 1) / (k1 * vx0);
            float y_ = vy0 * t_ - 0.5 * g * t_ * t_;
            delta = y - y_;
            k1 -= alpha * delta;
            PRINT_INFO("t=%f, y_=%f, k1=%f, delta=%f\n", t_, y_, k1, delta);
        } while (std::abs(delta) >= 0.001 && ++t < 100000);  // 误差小于1mm
        return k1;
    }
};
