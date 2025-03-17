#include "control.h"

uint16_t ADC_RESOLUTION = 4095;

uint32_t CONTROL_readAccel(UART_HandleTypeDef *huart){
    // IMU serial communications
}

// returns raw bits associated
uint32_t CONTROL_readLoadCell(ADC_HandleTypeDef *hadc){
    HAL_StatusTypeDef status = HAL_ADC_PollForConversion(hadc, 100); // timeout (ms)
    uint16_t raw = HAL_ADC_GetValue(hadc);
    if (raw >= ADC_RESOLUTION) raw = ADC_RESOLUTION;

    return (uint32_t) raw;
}

uint32_t CONTROL_readEncoder((TIM_TypeDef *)TIMx){
    return (TIMx->CNT) >> 2; // TODO: test assumption of divide by 4 accounting for quadrature encoder
}

void CONTROL_sendMotorCmd(motor_cmd motor_cmd){
    // TODO
}