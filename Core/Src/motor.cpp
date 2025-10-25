//
// Created by user on 2025/10/20.
//
#include "motor.h"
#include "pid.h"
//#include <bits/locale_facets_nonio.h>
#include <cmath>
#define GRAVITY_ACCELERATION 9.80665f
#define M_PI 3.14159f

M3508_Motor Motor(19.2);
float target_torque = 0;
float target_I_A = 0;
float target_i_tx = 0;

extern uint8_t tx_data[8];
extern float linear_mapping(float in, int in_min, int in_max, float out_min, float out_max);
void float_to_hex();

// 应该是最底层，直接控制引脚了吧（？）
void M3508_Motor::SetPosition(float ppid_output, float feedforward_speed, float feedforward_intensity) {
    // 通过新角度施加电流    ppid_output的单位为力矩    feedforward_intensity的单位为力矩
    target_torque = ppid_output + feedforward_intensity;
    target_I_A = torque_2_i(target_torque);

    target_i_tx = linear_mapping(target_I_A, -20, 20, -16384, 16384);
    float_to_hex(target_i_tx);
}
void M3508_Motor::SetSpeed(float spid_output, float feedforward_intensity) {
    target_torque = spid_output + feedforward_intensity; //
    target_I_A = torque_2_i(target_torque);

    target_i_tx = linear_mapping(target_I_A, -20, 20, -16384, 16384);
    float_to_hex(target_i_tx);
}
void M3508_Motor::SetIntensity(float intensity) {
    target_torque = intensity; //
    target_I_A = torque_2_i(target_torque);

    target_i_tx = linear_mapping(target_I_A, -20, 20, -16384, 16384);
    float_to_hex(target_i_tx);
}

void M3508_Motor::handle(void) {
    float angle = normalize_angle(fdb_angle_); // 将 fdb_angle_ 转化到 -180~180
    float spid_output = spid_.calc(target_speed_, fdb_speed_); // 单位：力矩
    //float ppid_output = spid_.calc(target_speed_, fdb_speed_); // 单位：力矩
    switch (Motor.control_method_) {
        case TORQUE:
            SetIntensity(torque_mode_cal_torque(angle));
            break;
        case POSITION_SPEED:
            SetSpeed(spid_output, 0);
            break;

        case 0x01:
            SetSpeed(spid_output, 0);
            break;
        case STOP:
            // SetSpeed(0, 0);
            tx_data[0] = 0;
            tx_data[1] = 0;
            break;
            // case POSITION_SPEED:
            //     float ppid_output = ppid_.calc(target_speed_, fdb_speed_); // 单位：力矩
            //     SetPosition(ppid_output, 0, 0);
            //     break;

            // default:
            //     // 如果 expression 不匹配任何 case，执行此代码块
            //     // default 是可选的
            //     break;
    }
}

// 读到的电流 转化为力矩
float i_2_torque(float I) {
    float torque;
    if (I > 0. && I <= 8.) {
        torque = (I - 0.5) / 2.5;
    } else if (I >= 8. && I <= 11.3) {
        torque = (I + 1.9) / 3.3;
    } else if (I >= -8 && I <= 0) {
        torque = (I + 0.5) / 2.5;
    } else if (I >= -11.3 && I <= -8) {
        torque = (I - 1.9) / 3.3;
    } else {
        return 0;
    }
    return torque;
}

float torque_2_i(float torque) {
    float i;
    if (torque >= -3. && torque <= 3.) {
        i = (torque > 0) ? (2.5 * torque + 0.5) : (2.5 * torque - 0.5);
    } else if ((torque >= 3. && torque <= 4.) || (torque >= -4. && torque <= -3.)) {
        i = (torque > 0) ? (3.3 * torque - 1.9) : (3.3 * torque + 1.9);
    } else {
        return 0;
    }
    return i;
}

void M3508_Motor::float_to_hex(float val) {
    int rounded = static_cast<int>(std::lround(val));

    // 提取高 8 位和低 8 位（转成 int8_t）
    int8_t high_byte = static_cast<int8_t>((rounded >> 8) & 0xFF);
    int8_t low_byte = static_cast<int8_t>(rounded & 0xFF);

    tx_data[0] = static_cast<uint8_t>(high_byte);
    tx_data[1] = static_cast<uint8_t>(low_byte);
}

float M3508_Motor::normalize_angle(float angle) {
    // 对角度进行取模，确保在 [0, 360) 范围内
    angle = fmod(angle, 360.0f); // fmod 返回余数，角度在 0 到 360 之间

    // 如果角度大于 180，则转换为负角度
    if (angle > 180.0f) {
        angle -= 360.0f;
    }
    return angle;
}

// 计算力矩（T），单位为 N·m
float M3508_Motor::torque_mode_cal_torque(float angle) const {
    float k = 0.9; // 感觉转矩有点大，给个小系数
    // 转换角度到弧度
    float angle_rad = angle * M_PI / 180.0f;

    // 定义已知量
    float L = 0.05524f; // 力臂 L, 单位 m
    float m = 0.5f; // 质量 m, 单位 kg

    // 计算力矩 T = m * g * L * sin(angle)
    float torque = k * m * GRAVITY_ACCELERATION * L * sin(angle_rad);

    return torque; // 返回计算的力矩，单位 N·m
}