// Copyright 2025 RC2026
// 基于 pb_omni_pid_pursuit_controller 移植
// 原作者: Lihan Chen
//
// Licensed under the Apache License, Version 2.0

#include "rc26_omni_controller/pid.hpp"

#include <cmath>

PID::PID(double dt, double max, double min, double kp, double kd, double ki)
: dt_(dt), max_(max), min_(min), kp_(kp), kd_(kd), ki_(ki), pre_error_(0), integral_(0)
{
}

double PID::calculate(double set_point, double pv)
{
  // 计算误差
  double error = set_point - pv;

  // 比例项
  double p_out = kp_ * error;

  // dt 有效性检查，防止除零和 NaN
  if (!std::isfinite(dt_) || dt_ <= 1e-9) {
    double output = p_out;
    if (output > max_)
      output = max_;
    else if (output < min_)
      output = min_;
    pre_error_ = error;
    return output;
  }

  // 积分项（先累加再限幅）
  integral_ += error * dt_;

  // 积分限幅（在计算输出前进行）[M2 修复：使用可配置限幅]
  if (integral_ > max_integral_) {
    integral_ = max_integral_;
  } else if (integral_ < -max_integral_) {
    integral_ = -max_integral_;
  }

  double i_out = ki_ * integral_;

  // 微分项
  double derivative = (error - pre_error_) / dt_;
  double d_out = kd_ * derivative;

  // 计算总输出
  double output = p_out + i_out + d_out;

  // 输出限幅
  if (output > max_)
    output = max_;
  else if (output < min_)
    output = min_;

  // 保存误差
  pre_error_ = error;

  return output;
}

void PID::setSumError(double sum_error) { integral_ = sum_error; }

void PID::setGains(double kp, double kd, double ki)
{
  kp_ = kp;
  kd_ = kd;
  ki_ = ki;
}

void PID::setOutputLimits(double min, double max)
{
  min_ = min;
  max_ = max;
}

void PID::setIntegralLimits(double max_integral)
{
  max_integral_ = max_integral;
}

PID::~PID() {}
