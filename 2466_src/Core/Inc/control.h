#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f1xx.h"
#include <stdint.h>

typedef enum{
    FORWARD, 
    NEUTRAL, 
    REVERSE
} motor_cmd;

extern volatile uint32_t encoder_count;

uint32_t CONTROL_readAccel(void);
uint32_t CONTROL_readLoadCell(void);
uint32_t CONTROL_readEncoder(void);
void CONTROL_sendMotorCmd(motor_cmd motor_cmd);

#endif // __CONTROL_H