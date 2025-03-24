#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f1xx.h"
#include <stdint.h>

# define IMU_DATA_PACKET_SIZE 19 // bytes
# define IMU_STARTUP_PACKET_SIZE 78
# define IMU_ACCEL_LSB 9 // TODO: these are x, check with orientation, and if need to set orientation registers
# define IMU_ACCEL_MSB 10

# define ADC_RESOLUTION 4095

typedef enum{
    FORWARD, 
    REVERSE,
    NEUTRAL
} motor_cmd;

void CONTROL_initIMU(UART_HandleTypeDef *huart);
uint32_t CONTROL_processIMUpacket(UART_HandleTypeDef *huart);
uint32_t CONTROL_readLoadCell(ADC_HandleTypeDef *hadc);
uint32_t CONTROL_readEncoder(TIM_TypeDef *TIMx);
void CONTROL_sendMotorCmd(TIM_TypeDef *TIMx, motor_cmd motor_cmd);
void CONTROL_initMotor(TIM_TypeDef *TIMx);

#endif // __CONTROL_H