// Copyright 2025 RC2026
// 基于 pb_omni_pid_pursuit_controller 移植
// 原作者: Lihan Chen
//
// Licensed under the Apache License, Version 2.0

#pragma once

class PID {
public:
    // kp - 比例增益
    // ki - 积分增益
    // kd - 微分增益
    // dt - 控制周期
    // max - 输出最大值
    // min - 输出最小值
    PID(double dt, double max, double min, double kp, double kd, double ki);

    // 计算控制输出
    double calculate(double set_point, double pv);
    void setSumError(double sum_error);

    // 动态更新参数
    void setGains(double kp, double kd, double ki);
    void setOutputLimits(double min, double max);
    void setIntegralLimits(double max_integral);  // [M2 修复] 积分限幅设置

    ~PID();

private:
    double dt_;
    double max_;
    double min_;
    double kp_;
    double kd_;
    double ki_;
    double pre_error_;
    double integral_;
    double max_integral_{1.0};  // [M2 修复] 积分限幅 (默认 1.0)
};
