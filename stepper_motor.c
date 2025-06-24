#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "stepper_motor.h"
#include "globals.h"

static float current_stepper_rotation = 0;
//positive is clockwise

void stepper_motor_init(void){
    //Init stepper motor pins
    gpio_init(SM_IN1);
    gpio_init(SM_IN2);
    gpio_init(SM_IN3);
    gpio_init(SM_IN4);
    gpio_set_dir(SM_IN1,GPIO_OUT);
    gpio_set_dir(SM_IN2,GPIO_OUT);
    gpio_set_dir(SM_IN3,GPIO_OUT);
    gpio_set_dir(SM_IN4,GPIO_OUT);
}

void stepper_motor_blocking(float radians,bool clockwise,bool reset){
    printf("\nStepper_motor_blocking: %f %d %d",radians,clockwise?1:0,reset?1:0);
    if(radians>((float)M_PI+0.05f)){
        printf("\nAngle is too large for stepper motor - it will break the wires");
        return;
    }
    float rotation_freq = 0.25f; //in hertz
    int phases = 8;
    int phases_per_cycle = 8;
    int gear_ratio = 64;
    float radians_per_cycle = (2*M_PI)/(gear_ratio*phases_per_cycle);
    uint64_t usDelay = (uint64_t) (((float)1/(phases*phases_per_cycle*rotation_freq*gear_ratio))*1000000);
    float radians_travelled = 0;
    if(!clockwise){
        while (radians_travelled<radians) {
            //Sequence taken from: https://wiki.seeedstudio.com/Gear_Stepper_Motor_Driver_Pack/
            //A is IN2, B is IN1, C is IN4, D is IN3
            for(int i=0;i<8;i++){
                if(!put_stepper(
                    (i<2 || i>6)?0:1,
                    (i>0 && i<4)?0:1,
                    (i>2 && i<6)?0:1,
                    (i>4)?0:1,
                usDelay,reset)) return;
            }
            radians_travelled+=radians_per_cycle;
            current_stepper_rotation-=radians_per_cycle;
        }
    }
    else{
        while (radians_travelled<radians) {
            //Reversed sequence
            for(int i=7;i>=0;i--){
                if(!put_stepper(
                    (i<2 || i>6)?0:1,
                    (i>0 && i<4)?0:1,
                    (i>2 && i<6)?0:1,
                    (i>4)?0:1,
                usDelay,reset)) return;
            }
            radians_travelled+=radians_per_cycle;
            current_stepper_rotation+=radians_per_cycle;
        }
    }
    printf("\nEnded stepper_motor_blocking");
    put_stepper(0,0,0,0,0,reset); 
}

bool put_stepper(int a,int b,int c,int d,uint64_t usDelay,bool reset){
    if((!auto_mode || !connected) && !reset){
        printf("\nPaused stepper");
        return false;
    }
    gpio_put(SM_IN2,a);
    gpio_put(SM_IN1,b);
    gpio_put(SM_IN4,c);
    gpio_put(SM_IN3,d);
    sleep_us(usDelay);
    return true;
}

void reset_stepper(void){
    sleep_ms(10);//Wait for the current rotation to stop
    printf("\nResetting stepper by: %f",current_stepper_rotation);
    if(current_stepper_rotation==0) return;
    stepper_motor_blocking(fabs(current_stepper_rotation),current_stepper_rotation<0,true);
    //if it is currently positive, we are currently clockwise and need to rotate anti-clockwise and so clockwise is false
}