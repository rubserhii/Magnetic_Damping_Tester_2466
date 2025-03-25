#ifndef __FSM_H
#define __FSM_H

#include <stdint.h>
#include "main.h"

# define ACCEL_DATA_IDX 0
# define LOADCELL_DATA_IDX 1
# define ENCODER_DATA_IDX 2

typedef enum {
    INIT,
    PRE_DUT, 
    DUT, 
    POST_DUT, 
    DEINIT
  } FSM_state;
  
extern volatile uint32_t encoder_count;
extern FSM_state state;  
  
void FSM_init(void);
void FSM_pre_dut(void);
void FSM_dut(void);
void FSM_post_dut(void);
void FSM_deinit(UART_HandleTypeDef *huart);

#endif // __FSM_H
