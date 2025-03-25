#include "fsm.h"
#include "control.h"
#include <math.h>
#include <stdio.h>

#define START_DUT_ENC 709 // encoder counts
#define END_DUT_ENC 1552 // encoder counts
#define ENC_READING_FREQ 2500 // Hz
#define ENC_PULSES_PER_M 2362.2255 // 600 / (pi*pulley diameter)
#define STOP_VELOCITY_THRESHOLD 0.3 // m/s

#ifndef VELOCITY_RUN
#define VELOCITY_RUN 1 // m/s
#endif

static uint8_t motor_dutycycle = 10;

// PRIVATE FUNCTIONS

float get_encoder_velocity(void);


// STATE MACHINE FUNCTIONS

void FSM_init(void){
    state = PRE_DUT;
}

void FSM_pre_dut(void){
    
    if(get_encoder_velocity() >= VELOCITY_RUN+0.2){
        CONTROL_sendMotorCmd(NEUTRAL, 0);
        printf("neutral\r\n");
    }
    else{
        CONTROL_sendMotorCmd(FORWARD, motor_dutycycle);
        printf("accel\r\n");
    }
    if(encoder_count == START_DUT_ENC){
        state = DUT;
    }
}

void FSM_dut(void){

    CONTROL_sendMotorCmd(NEUTRAL, 0); // should be NEUTRAL

    if(encoder_count == END_DUT_ENC){
        state = POST_DUT;
    }

}

void FSM_post_dut(void){

    CONTROL_sendMotorCmd(NEUTRAL, 0); // neutral normally, can do reverse if rail friction isn't large enough to stop at high speeds
    
    if(get_encoder_velocity() <= STOP_VELOCITY_THRESHOLD){
        CONTROL_sendMotorCmd(NEUTRAL, 0);
        state = DEINIT;
    }

}

// HELPER FUNCTIONS

float get_encoder_velocity(void){
    static uint32_t last_encoder_reading = 0;
    
    float velocity = (float)((int32_t)(encoder_count - last_encoder_reading)) / (pow(10,-6)*(ENC_PULSES_PER_M)); // m/s
    last_encoder_reading = encoder_count;

    return velocity;
}