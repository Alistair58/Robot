#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"
#include "hardware/pwm.h"
#include "motor_controller.h" //The gatt file compiled by cmake


//Facing the direction of the robot
const uint l_pwm = 21;
const uint l_forwards = 20;
const uint l_backwards = 19;
const uint r_pwm = 16;
const uint r_forwards = 17;
const uint r_backwards = 18;
uint l_slice_num; //Initialised in gpio_pins_init
uint l_channel;
uint r_slice_num;
uint r_channel;

static void gpio_pins_init(void);
void motor_control(uint8_t leftThrottle,uint8_t rightThrottle);
#include "ble.h" //Contains ble_setup and the function which calls motor_control


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
    while(1) {
        tight_loop_contents(); //Keep the device on
    }
    return 0;
}

static void gpio_pins_init(){
     //Enable PWM on GPIO 21
     gpio_set_function(l_pwm, GPIO_FUNC_PWM);
     l_slice_num = pwm_gpio_to_slice_num(l_pwm);
     l_channel = pwm_gpio_to_channel(l_pwm);
     gpio_set_function(r_pwm, GPIO_FUNC_PWM);
     r_slice_num = pwm_gpio_to_slice_num(r_pwm);
     r_channel = pwm_gpio_to_channel(r_pwm);
 
     //Init direction pins
     gpio_init(l_forwards);
     gpio_init(l_backwards);
     gpio_init(r_forwards);
     gpio_init(r_backwards);
     gpio_set_dir(l_forwards,GPIO_OUT);
     gpio_set_dir(l_backwards,GPIO_OUT);
     gpio_set_dir(r_forwards,GPIO_OUT);
     gpio_set_dir(r_backwards,GPIO_OUT);
}

void motor_control(uint8_t leftThrottle,uint8_t rightThrottle){ //Called by att_write_callback in ble.h
    //leftThrottle and rightThrottle are both 0<= <=100

    //Set direction pins
    gpio_put(l_forwards,(leftThrottle<50)?0:1);
    gpio_put(l_backwards,(leftThrottle<50)?1:0);
    gpio_put(r_forwards,(rightThrottle<50)?0:1);
    gpio_put(r_backwards,(rightThrottle<50)?1:0);

    // Set period of 1000 cycles (0 to 999 inclusive)
    pwm_set_wrap(l_slice_num, 999);
    pwm_set_wrap(r_slice_num, 999);
 
    uint16_t l_duty = (uint16_t) round(((float)(abs(leftThrottle-50))/50) * 1000);
    //Abs as we want the values furthest from 50 (0 and 100) to give the fastest outputs
    uint16_t r_duty = (uint16_t) round(((float)(abs(rightThrottle-50))/50) * 1000);
    pwm_set_chan_level(l_slice_num, l_channel, l_duty); //set duty cycle
    pwm_set_chan_level(r_slice_num, r_channel, r_duty);
    
    pwm_set_enabled(l_slice_num, true); // Set the PWM running
    pwm_set_enabled(r_slice_num, true);
}