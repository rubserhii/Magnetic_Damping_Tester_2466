#ifndef __SENSORS_H
#define __SENSORS_H

#include "stm32f1xx.h"
#include <stdint.h>

uint32_t SENSORS_readAccel(void);
uint32_t SENSORS_readLoadCell(void);
uint32_t SENSORS_readEncoder(void);

#endif // __SENSORS_H