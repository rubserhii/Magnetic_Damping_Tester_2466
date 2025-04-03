#include "control.h"
#include "main.h"
#include <stdio.h>

uint8_t imu_uart_buffer[IMU_DATA_PACKET_SIZE];


void CONTROL_initIMU(UART_HandleTypeDef *huart){

    uint8_t temp_imu_init[IMU_STARTUP_PACKET_SIZE];

    // reset IMU
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, 0);
    HAL_Delay(1); // arbitrary, but should be long enough for IMU to recognize it needs to be reset
    HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, 1);
    // in UART-RCV mode, calibration automatically enabled for accel and magneto  

    if (HAL_UART_Receive(huart, temp_imu_init, IMU_STARTUP_PACKET_SIZE, 500) != HAL_OK){
        printf("IMU Init Failed!\r\n");
    } // blocking receive startup bytes (p43) from IMU
    
    printf("IMU Init Success\r\n");

    HAL_UART_Receive_IT(huart, imu_uart_buffer, IMU_DATA_PACKET_SIZE); // initial recieve to trigger interrupt on packet reception
}

// returns single acceleration measurement
uint32_t CONTROL_processIMUpacket(UART_HandleTypeDef *huart){
    
    uint16_t accel = (((uint16_t)imu_uart_buffer[IMU_ACCEL_MSB]) << 8) | (uint16_t)imu_uart_buffer[IMU_ACCEL_LSB]; 
    HAL_UART_Receive_IT(huart, imu_uart_buffer, IMU_DATA_PACKET_SIZE); // set trigger for interrupt after next packet rx'd
    
    return (uint32_t)accel;
}

// returns associated raw bits
uint32_t CONTROL_readLoadCell(ADC_HandleTypeDef *hadc){
    HAL_ADC_PollForConversion(hadc, 100); // timeout (ms)    
    uint16_t raw = HAL_ADC_GetValue(hadc);    
    if (raw >= ADC_RESOLUTION) raw = ADC_RESOLUTION;

    return (uint32_t) raw;
}

uint32_t CONTROL_readEncoder(void){
    return (TIM2->CNT) >> 1; // divide by 2
}

void CONTROL_sendMotorCmd(motor_dir direction, uint8_t duty_cycle){
    
    TIM3->CCR1 = (uint32_t)((duty_cycle * (TIM3->ARR + 1)) / 100); 

    switch (direction){
        case FORWARD:
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, FORWARD); // A high, B low
            break;

        case NEUTRAL:
            TIM3->CCR1 = 0;    
            break;

        case REVERSE:
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, REVERSE); // A low, B high
            break;
        }
}