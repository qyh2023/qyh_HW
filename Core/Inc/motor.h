//
// Created by user on 2025/10/3.
//

#ifndef PROJECT1001_MOTOR_H
#define PROJECT1001_MOTOR_H

#include "can.h"
#include "pid.h"

float i_2_torque(float I);
float torque_2_i(float torque);
class M3508_Motor {
private:
    float accm_angle_ = 0.f; // deg 输出端(减速后)累计转动角度
    float delta_angle_ = 0.f; // deg 输出端(减速后)新转动的角度
    float bias_ = 0.f;

    float ecd_angle_ = 0.f; // deg 当前编码器角度
    float last_ecd_angle_ = 0.f; // deg 上次编码器角度
    float delta_ecd_angle_ = 0.f; // deg 编码器新转动的角度
    float accm_ecd_angle_ = 0.f; //编码器累计转动角度

    float rotate_speed_ = 0.f; // dps 反馈转子转速 (degree per second)
    float temp_ = 0.f; // °C  反馈电机温度

    float target_angle_ = 0, fdb_angle_ = 0;
    float target_speed_ = 0, fdb_speed_ = 0, feedforward_speed_ = 0;
    float target_torque_ = 0.f, fdb_torque_ = 0.f;
    float feedforward_intensity_ = 0, output_intensity_ = 0;

public:
    float torque_ = 0.f;
    float current_ = 0.f; // A   反馈转矩电流

    float ratio_; // 电机减速比
    explicit M3508_Motor(const float ratio): control_method_(TORQUE) {
        ratio_ = ratio;
        target_speed_ = 500;
    };
    PID spid_, ppid_, tpid_;
    void can_rx_msg_callback(const uint8_t rxdata[8]);
    void handle(void);
    static float normalize_angle(float angle);
    float torque_mode_cal_torque(float angle) const;

    static void float_to_hex(float target_i_tx);
    void SetPosition(float target_position, float feedforward_speed, float feedforward_intensity);
    void SetSpeed(float target_speed, float feedforward_intensity);
    void SetIntensity(float intensity);
    enum { TORQUE, SPEED, POSITION_SPEED, STOP } control_method_;
};

#endif //PROJECT1001_MOTOR_H