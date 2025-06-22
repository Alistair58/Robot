#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "btstack.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

#define min(a,b) (((a)>(b))?(b):(a))
#define fcmp(a,b) ((a)+0.000001f>(b) && (a)-0.000001f<(b))
#include "controller_ble.h" //The gatt file compiled by cmake
#include "ultrasound.h"
int connected = 0;
bool auto_mode = false;
#include "driving_motors.h" //checks connected
#include "stepper_motor.h" //checks connected and auto_mode
static void gpio_pins_init(void);
void core1_main(void);
void set_auto_mode(uint8_t value);
#include "ble.h" //Calls update_throttle from driving_motors.h and set_auto_mode

//TODO
//Make notes
//Look into delayed output variable PID
//Make the robot turn reliably


int main(){
    stdio_init_all();
    // Initialise the Wi-Fi, Bluetooth and LED chip
    if (cyw43_arch_init()) {
        printf("CYW43 init failed\n");
        return 1;
    }
    // Turn on the LED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    gpio_pins_init();
    ble_setup();
    motor_speed_manage_blocking();
    return 0;
}

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

static void gpio_pins_init(){
    driving_motors_init();
    stepper_motor_init();
    ultrasound_init();
}

void set_auto_mode(uint8_t value){
    if(value==0 && auto_mode){
        auto_mode = false;
        multicore_reset_core1();
        update_throttle(50,50);
        reset_stepper();
    }
    else if(value==1 && !auto_mode){
        stop = false; //Otherwise the robot won't be able to move
        auto_mode = true;
        update_throttle(50,50);
        multicore_reset_core1();
        sleep_ms(5);
        multicore_launch_core1(core1_main);
    }
}





