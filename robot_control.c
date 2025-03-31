#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"
#define BTSTACK_INCLUDED 1
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "controller_ble.h" //The gatt file compiled by cmake



//Facing the direction of the robot
#define DM_L_PWM 21
#define DM_L_FORWARDS 20
#define DM_L_BACKWARDS 19
#define DM_R_PWM 16
#define DM_R_FORWARDS 17
#define DM_R_BACKWARDS 18
uint l_slice_num; //Initialised in gpio_pins_init
uint l_channel;
uint r_slice_num;
uint r_channel;
#define HE_L_POWER 15
#define HE_R_POWER 22
#define HE_L_ADC_GPIO 27 //GPIO pin numbers
#define HE_R_ADC_GPIO 26
#define HE_L_ADC 1 //ADC pin numbers
#define HE_R_ADC 0

static float curr_l_speed = 0;
static float curr_r_speed = 0;

//TODO 
//test current speed detection
//add in changing pwm dependent on the wheel speed
//need the max speed so that the max throttle is max wheel speed

static void gpio_pins_init(void);
static void hall_effect_init(void);
void motor_control(uint8_t leftThrottle,uint8_t rightThrottle);
static void hall_effect_read_blocking(void);
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
    hall_effect_init();
    hall_effect_read_blocking();
    return 0;
}

static void gpio_pins_init(){
     //Enable PWM on GPIO 21
     gpio_set_function(DM_L_PWM, GPIO_FUNC_PWM);
     l_slice_num = pwm_gpio_to_slice_num(DM_L_PWM);
     l_channel = pwm_gpio_to_channel(DM_L_PWM);
     gpio_set_function(DM_R_PWM, GPIO_FUNC_PWM);
     r_slice_num = pwm_gpio_to_slice_num(DM_R_PWM);
     r_channel = pwm_gpio_to_channel(DM_R_PWM);
 
     //Init direction pins
     gpio_init(DM_L_FORWARDS);
     gpio_init(DM_L_BACKWARDS);
     gpio_init(DM_R_FORWARDS);
     gpio_init(DM_R_BACKWARDS);
     gpio_set_dir(DM_L_FORWARDS,GPIO_OUT);
     gpio_set_dir(DM_L_BACKWARDS,GPIO_OUT);
     gpio_set_dir(DM_R_FORWARDS,GPIO_OUT);
     gpio_set_dir(DM_R_BACKWARDS,GPIO_OUT);
}

void motor_control(uint8_t leftThrottle,uint8_t rightThrottle){ //Called by att_write_callback in ble.h
    //leftThrottle and rightThrottle are both 0<= <=100

    //Set direction pins
    gpio_put(DM_L_FORWARDS,(leftThrottle<50)?0:1);
    gpio_put(DM_L_BACKWARDS,(leftThrottle<50)?1:0);
    gpio_put(DM_R_FORWARDS,(rightThrottle<50)?0:1);
    gpio_put(DM_R_BACKWARDS,(rightThrottle<50)?1:0);

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

static void hall_effect_init(void){
    gpio_init(HE_L_POWER);
    gpio_init(HE_R_POWER);
    gpio_set_dir(HE_L_POWER,GPIO_OUT);
    gpio_set_dir(HE_R_POWER,GPIO_OUT);
    gpio_put(HE_L_POWER,1);
    gpio_put(HE_R_POWER,1);

    adc_init();
    adc_gpio_init(HE_L_ADC_GPIO);
    adc_gpio_init(HE_R_ADC_GPIO);
    
    //left sensor:
    //1.62v is about normal
    //>1.9v when right in front of a magnet
    //all 5 magnets behave similarly

    //right sensor:
    //1.52v is about normal
    //>1.75v when magnet

}

static void hall_effect_read_blocking(void){
    //currently recording the time between 2 spokes
    //therefore, can't tell the direction
    bool r_high = false;
    bool l_high = false;
    const float high_voltage = 1.75;
    const float low_voltage = 1.6;
    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    const float conversion_factor = 3.3f / (1 << 12);
    uint32_t last_r_high = 0;
    uint32_t last_l_high = 0;
    uint32_t curr_time = 0;
    uint16_t raw_adc_r = 0;
    uint16_t raw_adc_l = 0;
    float voltage_r = 0;
    float voltage_l = 0;
    const float spoke_angle = (float)2/5*M_PI; //5 spokes, therefore, roughly the angle between each is 2/5 pi
    uint32_t last_print_debug = time_us_32();
    while (1) {
        curr_time = time_us_32();
    
        //Right side
        adc_select_input(HE_R_ADC);
        raw_adc_r = adc_read();
        voltage_r = raw_adc_r*conversion_factor;
        if(r_high && voltage_r<low_voltage){
            r_high = false;
        }
        else if(!r_high && voltage_r>high_voltage){
            r_high = true;
            if(last_r_high){
                printf("\nTime since last spoke (R): %lu",curr_time-last_r_high);
                curr_r_speed = spoke_angle/((float)(curr_time-last_r_high)/1000000); //convert to seconds from micro
                printf("\nNew speed (R): %f",curr_r_speed);
            }
            last_r_high = curr_time;
        }
        else if(curr_time-last_r_high>3000000){ //either very slow or not moving
            curr_r_speed = 0;
            //reset
            last_r_high = 0; //this will be fixed when we hit the next spoke (also works if we are on a spoke)
            r_high = false; 
        }

        //Left side
        adc_select_input(HE_L_ADC);
        raw_adc_l = adc_read();
        voltage_l = raw_adc_l*conversion_factor;
        if(l_high && voltage_l<low_voltage){
            l_high = false;
        }
        else if(!l_high && voltage_l>high_voltage){
            l_high = true;
            if(last_l_high){
                printf("\nTime since last spoke (L): %lu",curr_time-last_l_high);
                curr_l_speed = spoke_angle/((float)(curr_time-last_l_high)/1000000);
                printf("\nNew speed (L): %f",curr_l_speed);
            }
            last_l_high = curr_time;
        }
        else if(curr_time-last_l_high>3000000){ 
            curr_l_speed = 0;
            last_l_high = 0;
            l_high = false; 
        }

        if(curr_time-last_print_debug>1000000){
            last_print_debug = curr_time;
            printf("Voltage_r: %f V, Time_since_high_r: %lus, curr_r_speed: %frads^-1\n", voltage_r, (curr_time-last_r_high)/1000000,curr_r_speed);
            printf("Voltage_l: %f V, Time_since_high_l: %lus, curr_l_speed: %frads^-1\n", voltage_l, (curr_time-last_l_high)/1000000,curr_l_speed);
            sleep_ms(10);
        }
    }
}