#include "fsm.h"


void FSM_init(void){
    // set pwm to zero
    state = PRE_DUT; 
}

void FSM_pre_dut(void){
    // set acceleration PWM
    // if(encoder position = start of DUT){
    //     state = DUT;
    // }
}

void FSM_dut(void){

    // if(encoder position = end of DUT){
    //     state = POST_DUT;
    // }

}

void FSM_post_dut(void){
    // set brake PWM

    // if(velocity threshold check passes){
    //     set PWM zero
    //     state = DEINIT;
    // }

}

void FSM_deinit(void){
    // send data over serial
}