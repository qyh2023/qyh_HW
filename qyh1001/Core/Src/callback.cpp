//
// Created by user on 2025/10/2.
//
#include "tim.h"
#include "gpio.h"
#include "main.h"
#include "usart.h"


// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//     if(GPIO_Pin == BUTTON_Pin){
//         uint32_t arr_value = __HAL_TIM_GetAutoreload(&htim1)+ 1;
//         uint32_t brightness =(__HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_2)+ 100)% arr_value;
//         __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, brightness);
//     }
// }

// uint32_t count = 0;
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//     if (htim == &htim1) {
//         count++;
//     }
// }
extern  uint8_t rx_msg[];

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//     if (huart == &huart8) {
//         if (rx_msg[0] == 'R') {
//             HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);
//         }else if (rx_msg[0] == 'M') {
//             HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET);
//         }
//         HAL_UART_Receive_IT(&huart8, rx_msg, 1);
//     }
// }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart == &huart8) {
        HAL_UART_Receive_IT(&huart8, rx_msg, 1);
        HAL_UART_Transmit(&huart8, rx_msg, 1, 1000);
    }
}
