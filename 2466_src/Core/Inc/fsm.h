#ifndef __FSM_H
#define __FSM_H

typedef enum {
    INIT,
    PRE_DUT, 
    DUT, 
    POST_DUT, 
    DEINIT
  } position_state;
  
extern position_state state; 

// databuffer
// pwm

void FSM_INIT(void);
void FSM_PRE_DUT(void);
void FSM_DUT(void);
void FSM_POST_DUT(void);
void FSM_DEINIT(void);

#endif // __FSM_H
