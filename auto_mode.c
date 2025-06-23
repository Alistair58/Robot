#include <math.h>
#include "pico/multicore.h"
#include "auto_mode.h"
#include "globals.h"
#include "driving_motors.h"
#include "stepper_motor.h"


void core1_main(void){ //auto mode
    float distance;
    int direction = 0;
    while(1){
        if(!auto_mode) return;
        turn_robot(2*M_PI,true);
        sleep_ms(10000);
        // stepper_motor_blocking(M_PI*0.5f,direction,false);
        // ultrasound_trig(&distance);
        // printf("\nDistance: %f",distance);
        // sleep_ms(1000);
        // stepper_motor_blocking(M_PI*0.5f,!direction,false);
        // direction ^= 1;
        // sleep_ms(2000);
    }

}

void set_auto_mode(uint8_t value){
    if(value==0 && auto_mode){
        auto_mode = false;
        multicore_reset_core1();
        update_throttle(50,50);
        reset_stepper();
    }
    else if(value==1 && !auto_mode){
        auto_mode = true;
        update_throttle(50,50);
        multicore_reset_core1();
        sleep_ms(100); //Seems to make it work more reliably
        multicore_launch_core1(core1_main);
    }
}



