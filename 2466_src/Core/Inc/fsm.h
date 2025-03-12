#ifndef __FSM_H
#define __FSM_H

#include "<stdint.h>"

# define ACCEL_DATA_IDX = 0;
# define LOADCELL_DATA_IDX = 1;
# define ENCODER_DATA_IDX = 2;

typedef enum {
    INIT,
    PRE_DUT, 
    DUT, 
    POST_DUT, 
    DEINIT
  } position_state;
  
extern position_state state; 
extern uint32_t dataBuffer[3][250]; // check datatype with imu
extern int16_t motor_pwm; 

void FSM_INIT(void);
void FSM_PRE_DUT(void);
void FSM_DUT(void);
void FSM_POST_DUT(void);
void FSM_DEINIT(void);

#endif // __FSM_H
