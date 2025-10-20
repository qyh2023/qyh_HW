//
// Created by user on 2025/10/18.
//

#ifndef QYH1001_PID_H
#define QYH1001_PID_H

// class PID {
// public:
//     PID(): PID(0, 0, 0, 0, 0) {}
//     PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k = 1);
//
//     void reset();
//     float calc(float ref, float fdb);
//
//     float kp_, ki_, kd_, d_filter_k_;
//     float i_max_, out_max_;
//     float output_;
//
// private:
//     float ref_, fdb_;
//     float err_, err_sum_, last_err_;
//     float pout_, iout_, dout_, last_dout_;
// };

class PID {
public:
    PID() {
        kp_ = 0;
        ki_ = 0;
        kd_ = 0;
        i_max_ = 0;
        out_max_ = 0;
        d_filter_k_ = 1;
        reset();
    }

    PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k = 1) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        i_max_ = i_max;
        out_max_ = out_max;
        d_filter_k_ = d_filter_k;
        reset();
    }

    void reset();
    float calc(float ref, float fdb);

    float kp_, ki_, kd_, d_filter_k_;
    float i_max_, out_max_;
    float output_;

private:
    float ref_, fdb_;
    float err_, err_sum_, last_err_;
    float pout_, iout_, dout_, last_dout_;
};

#endif //QYH1001_PID_H
