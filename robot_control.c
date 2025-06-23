#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "ultrasound.h"
#include "stepper_motor.h" 
#include "driving_motors.h" 
#include "ble.h" 
#include "globals.h"

//TODO
//forwards function
//options for until it hits something or a fixed distance
//need to add stuck error message to turning
//Note that it is impossible to detect a stuck with wheels spinning
//Add receiver for this in the app

//Obstacle avoidance code
//Needs a stack

int connected = 0;
bool auto_mode = false;

static void gpio_pins_init(void);

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

static void gpio_pins_init(){
    driving_motors_init();
    stepper_motor_init();
    ultrasound_init();
}



