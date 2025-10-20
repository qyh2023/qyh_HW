//
// Created by user on 2025/10/18.
//
#include "pid.h"

// 带参数的构造函数
// PID::PID(
//     const float kp,
//     const float ki,
//     const float kd,
//     const float i_max,
//     const float out_max,
//     const float d_filter_k
// ):
//     kp_(kp),
//     ki_(ki),
//     kd_(kd),
//     i_max_(i_max),
//     out_max_(out_max),
//     d_filter_k_(d_filter_k),
//     ref_(0),
//     fdb_(0),
//     err_(0),
//     err_sum_(0),
//     last_err_(0),
//     pout_(0),
//     iout_(0),
//     dout_(0),
//     last_dout_(0),
//     output_(0) {}

// 重置 PID 控制器的状态
void PID::reset() {
    err_ = 0;
    err_sum_ = 0;
    last_err_ = 0;
    pout_ = 0;
    iout_ = 0;
    dout_ = 0;
    last_dout_ = 0;
    output_ = 0;
}

// 计算 PID 输出
float PID::calc(const float ref, const float fdb) {
    // 更新参考值和反馈值
    ref_ = ref;
    fdb_ = fdb;

    // 计算误差
    err_ = ref_ - fdb_;

    // 积分（累加误差）并加上限制
    err_sum_ += err_;
    if (err_sum_ > i_max_) {
        err_sum_ = i_max_;
    } else if (err_sum_ < -i_max_) {
        err_sum_ = -i_max_;
    }

    // 微分
    dout_ = kd_ * (err_ - last_err_);
    dout_ = dout_ * d_filter_k_ + last_dout_ * (1 - d_filter_k_); // 简单的滤波器

    // PID 各项计算
    pout_ = kp_ * err_;
    iout_ = ki_ * err_sum_;

    // 计算输出，限制输出
    output_ = output_ + pout_ + iout_ + dout_;
    if (output_ > out_max_) {
        output_ = out_max_;
    } else if (output_ < -out_max_) {
        output_ = -out_max_;
    }

    // 更新历史误差
    last_err_ = err_;
    last_dout_ = dout_;

    return output_;
}
