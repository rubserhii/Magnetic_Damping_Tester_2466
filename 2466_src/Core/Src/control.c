#include "control.h"

uint32_t CONTROL_readAccel(UART_HandleTypeDef *huart){
    // IMU serial communications
}

uint32_t CONTROL_readLoadCell(ADC_HandleTypeDef *hadc){
    // polling read load cell ADC 
}

uint32_t CONTROL_readEncoder(void){
    encoder_count = TIM2->CNT; // check this, and make sure overflow is accounted for
}

void CONTROL_sendMotorCmd(motor_cmd motor_cmd){
    
}