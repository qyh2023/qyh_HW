//
// Created by user on 2025/10/3.
//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "can.h"
#include "motor.h"
#include "tim.h"
#include "main.h"

extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t tx_data[8];
extern uint8_t rx_data[8];
M3508_Motor Motor(19.2);
extern uint32_t can_tx_mail_box_;

float linearMapping(int in, int in_min, int in_max, float out_min, float out_max) {
    float ans = out_min + (out_max - out_min) / (in_max - in_min) * (in - in_min);
    return ans;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    if (hcan->Instance == CAN1) {
        // 读取报文 header 和 data
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);

        //自定义函数：将 rx_data 解析为电机信息
        if (rx_header.StdId == 0x202) {
            Motor.canRxMsgCallback(rx_data);
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim6.Instance) {
        // 发送 CAN 报文
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, CAN_FilterFIFO0);
    }
}

void M3508_Motor::canRxMsgCallback(const uint8_t rxdata[8]) {
    // 0机械角度高8位、1机械角度低8位
    const uint16_t ecd_angle_raw = static_cast<uint16_t>(rxdata[0] << 8) | rxdata[1];
    float ecd_angle = 0.0;
    ecd_angle = linearMapping(ecd_angle_raw, 0, 8191, 0, 360);
    ecd_angle_ = ecd_angle;
    delta_ecd_angle_= ecd_angle_ - last_ecd_angle_;
    // 处理角度跳变
    if (delta_ecd_angle_ > 180.0f) {
        delta_ecd_angle_ -= 360.0f;
    } else if (delta_ecd_angle_ < -180.0f) {
        delta_ecd_angle_ += 360.0f;
    }
    // 输出端新转动的角度
    delta_angle_ = delta_ecd_angle_ /ratio_;
    // 输出端累计转动角度
    angle_ += delta_angle_;
    // 更新编码器角度
    last_ecd_angle_ = ecd_angle_;

    //2转子转速高8位，3转子转速低8位
    const auto rotate_speed_raw = static_cast<int16_t>((rxdata[2] << 8) | rxdata[3]);
    rotate_speed_ = rotate_speed_raw;

    //4实际转矩电流高8位，5实际转矩电流低8位
    int current_raw = (rxdata[4] << 8) | rxdata[5];
    current_ = linearMapping(current_raw, -16384, 16384, -20, 20);

    //6温度
    const int temp_raw = rxdata[6];
    temp_ = static_cast<float>(temp_raw);
    //7NULL

}
