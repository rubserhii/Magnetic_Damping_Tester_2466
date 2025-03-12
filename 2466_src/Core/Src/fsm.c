#include "fsm.h"


void FSM_INIT(void){
    // set pwm to zero
    state = PRE_DUT; 
}

void FSM_PRE_DUT(void){
    // set acceleration PWM
    // if(encoder position = start of DUT){
    //     state = DUT;
    // }
}

void FSM_DUT(void){

    // if(encoder position = end of DUT){
    //     state = POST_DUT;
    // }

}

void FSM_POST_DUT(void){
    // set brake PWM

    // if(velocity threshold check passes){
    //     set PWM zero
    //     state = DEINIT;
    // }

}

void FSM_DEINIT(void){
    // send data over serial
}