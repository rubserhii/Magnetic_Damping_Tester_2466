#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f1xx.h"
#include <stdint.h>

typedef enum{
    FORWARD, 
    NEUTRAL, 
    REVERSE
} motor_cmd;

uint32_t CONTROL_readAccel(UART_HandleTypeDef *huart);
uint32_t CONTROL_readLoadCell(ADC_HandleTypeDef *hadc);
uint32_t CONTROL_readEncoder(TIM_TypeDef *TIMx);
void CONTROL_sendMotorCmd(motor_cmd motor_cmd);

#endif // __CONTROL_H