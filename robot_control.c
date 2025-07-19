#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "ultrasound.h"
#include "stepper_motor.h" 
#include "driving_motors.h" 
#include "ble.h" 
#include "globals.h"
#include "imu.h"


//TODO
//Stepper motor reset mechanism
// - BLE write characteristic sent from app
// - App displays message about finger and confirmation of finger
// - Rotates stepper pi/4 in both directions until it finds something at a short distance and then stops
//Debug obstacle avoidance

//Mechanical bugs:
//Battery low - affects BLE connection and green LED is dimmer
//Hall effect sensor moved - voltages for magnets are different and so spoke counts 
//  (and, by consequence, rotations and distances) will be off

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
    imu_i2c_init();
    ble_setup();
    motor_speed_manage_blocking();
    return 0;
}

static void gpio_pins_init(){
    driving_motors_init();
    stepper_motor_init();
    ultrasound_init();
}



