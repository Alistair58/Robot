#include <math.h>
#include <stdio.h>
#include "pico/multicore.h"
#include "auto_mode.h"
#include "globals.h"
#include "driving_motors.h"
#include "stepper_motor.h"
#include "movement.h"


void core1_main(void){ //auto mode
    float distance;
    int direction = 0;
    while(1){
        printf("\nGoing forwards");
        straight(0,true);
        for(int i=0;i<10;i++){
            if(!auto_mode) return;
            printf("\nSleeping");
            sleep_ms(1000);
        }
    }

}

void set_auto_mode(uint8_t value){
    if(value==0 && auto_mode){
        auto_mode = false;
        sleep_ms(15); //let auto_mode finish
        printf("\nEnding auto_mode");
        update_throttle(50,50);
        reset_stepper();
    }
    else if(value==1 && !auto_mode){
        auto_mode = true;
        printf("\nAuto_mode enabled");
        update_throttle(50,50);
        multicore_reset_core1();
        multicore_fifo_pop_blocking(); //https://forums.raspberrypi.com/viewtopic.php?t=355675
        multicore_launch_core1(core1_main);
    }
}



