#include <math.h>
#include <stdio.h>
#include <setjmp.h>
#include "pico/multicore.h"
#include "auto_mode.h"
#include "globals.h"
#include "driving_motors.h"
#include "stepper_motor.h"
#include "movement.h"
jmp_buf core1_exit_buf;

bool core1_running = false;

void core1_main(void){ //auto mode
    if (setjmp(core1_exit_buf) != 0){
        //sets the jump position on the initial iteration which returns 0
        //and then core1_ended causes a jump with value 1
        printf("\nCore 1 exited");
        update_throttle(50,50);
        reset_stepper();
        printf("\nStepper reset");
        core1_running = false;
        return;
    }
    core1_running = true;
    float distance;
    int direction = 0;
    while(1){
        stepper_motor_blocking(M_PI_2,true,false);
        reset_stepper();
        sleep_ms(2000);
        stepper_motor_blocking(M_PI_2,false,false);
        reset_stepper();
        sleep_ms(2000);
        // printf("\nGoing forwards");
        // straight(0,true);
        // for(int i=0;i<10;i++){
        //     if(!auto_mode) return;
        //     printf("\nSleeping");
        //     sleep_ms(1000);
        // }
    }

}

void core1_ended(void){
    longjmp(core1_exit_buf, 1);
}

void set_auto_mode(uint8_t value){
    if(value==0 && auto_mode){
        auto_mode = false;
        printf("\nEnding auto_mode");
        while(core1_running){
            sleep_ms(5); //let auto_mode finish
        }
        multicore_reset_core1();
        printf("\nCore 1 reset");
        //Core1 must call core1_ended so that the state is reset.
        //Calling multicore_reset_core1() doesn't work reliably without this
        
    }
    else if(value==1 && !auto_mode){
        auto_mode = true;
        printf("\nAuto_mode enabled");
        update_throttle(50,50);
        multicore_reset_core1();
        multicore_launch_core1(core1_main);
    }
}



