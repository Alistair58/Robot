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
#define min(a,b) (((a)>(b))?(b):(a))

static float curr_l_speed = 0;
static float curr_r_speed = 0;
static float max_speed = 20; //when spinning in the air, it reaches 20 rads^-1
uint8_t user_l_throttle = 50;
uint8_t user_r_throttle = 50;
float effective_l_throttle = 50;
float effective_r_throttle = 50;

static void gpio_pins_init(void);
static void hall_effect_init(void);
static void motor_speed_manage_blocking(void);
static void he_update_speed(int ADC_PIN,bool *high, uint32_t *last_high,int32_t curr_time);
static void manage_pwm(float curr_speed,uint8_t user_throttle,float *effective_throttle,uint32_t *last_pwm_change,uint32_t curr_time,uint slice_num,uint channel);
void update_throttle(uint8_t left_throttle_new,uint8_t right_throttle_new);
#include "ble.h" //Contains ble_setup and the function which calls update_throttle


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
    motor_speed_manage_blocking();
    return 0;
}

static void gpio_pins_init(){
     //Enable PWM
     gpio_set_function(DM_L_PWM, GPIO_FUNC_PWM);
     l_slice_num = pwm_gpio_to_slice_num(DM_L_PWM);
     l_channel = pwm_gpio_to_channel(DM_L_PWM);
     gpio_set_function(DM_R_PWM, GPIO_FUNC_PWM);
     r_slice_num = pwm_gpio_to_slice_num(DM_R_PWM);
     r_channel = pwm_gpio_to_channel(DM_R_PWM);
     // Set period of 1000 cycles (0 to 999 inclusive)
     pwm_set_wrap(l_slice_num, 999);
     pwm_set_wrap(r_slice_num, 999);
 
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

void update_throttle(uint8_t left_throttle_new,uint8_t right_throttle_new){ //Called by att_write_callback in ble.h
    //0<= throttle <=100
    user_l_throttle = left_throttle_new;
    user_r_throttle = right_throttle_new;
    if(user_l_throttle == 50) effective_l_throttle = 50;
    if(user_r_throttle == 50) effective_r_throttle = 50;
    //Set direction pins
    gpio_put(DM_L_FORWARDS,(user_l_throttle<50)?0:1);
    gpio_put(DM_L_BACKWARDS,(user_l_throttle<50)?1:0);
    gpio_put(DM_R_FORWARDS,(user_r_throttle<50)?0:1);
    gpio_put(DM_R_BACKWARDS,(user_r_throttle<50)?1:0);

    //Set the initial pwm duty cycle
    //This may be changed by the hall effect sensor
    uint16_t l_duty = (uint16_t) round(((float)(abs(user_l_throttle-50))/50) * 1000);
    //Abs as we want the values furthest from 50 (0 and 100) to give the fastest outputs
    uint16_t r_duty = (uint16_t) round(((float)(abs(user_r_throttle-50))/50) * 1000);
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

static void motor_speed_manage_blocking(void){
    //currently recording the time between 2 spokes
    //therefore, can't tell the direction
    bool r_high = false;
    bool l_high = false;
    uint32_t last_r_high = 0; //it starts counting when we hit a spoke
    uint32_t last_l_high = 0;
    uint32_t curr_time = time_us_32();
    uint32_t last_l_pwm_update = curr_time;
    uint32_t last_r_pwm_update = curr_time;
    //uint32_t last_print_debug = time_us_32();
    while (1) {
        curr_time = time_us_32();
        he_update_speed(HE_L_ADC,&l_high,&last_l_high,curr_time);
        he_update_speed(HE_R_ADC,&r_high,&last_r_high,curr_time);
        manage_pwm(curr_l_speed,user_l_throttle,&effective_l_throttle,&last_l_pwm_update,curr_time,l_slice_num,l_channel);
        manage_pwm(curr_r_speed,user_r_throttle,&effective_r_throttle,&last_r_pwm_update,curr_time,r_slice_num,r_channel);
        
        // if(curr_time-last_print_debug>1000000){
        //     last_print_debug = curr_time;
        //     printf("Voltage_r: %f V, Time_since_high_r: %lus, curr_r_speed: %frads^-1\n", voltage_r, (curr_time-last_r_high)/1000000,curr_r_speed);
        //     printf("Voltage_l: %f V, Time_since_high_l: %lus, curr_l_speed: %frads^-1\n", voltage_l, (curr_time-last_l_high)/1000000,curr_l_speed);
        //     sleep_ms(10);
        // }
    }
}

static void he_update_speed(int ADC_PIN,bool *high, uint32_t *last_high,int32_t curr_time){
    const int max_spoke_wait = 2000000;
    const float high_voltage = 1.75; //gap so it is more stable
    const float low_voltage = 1.65;
    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    const float conversion_factor = 3.3f / (1 << 12);
    const float spoke_angle = (float)2/5*M_PI; //5 spokes, therefore, roughly the angle between each is 2/5 pi

    adc_select_input(ADC_PIN);
    uint16_t raw_adc = adc_read();
    float voltage = raw_adc*conversion_factor;
        if(*high && voltage<low_voltage){
            *high = false;
        }
        else if(!*high && voltage>high_voltage){
            *high = true;
            if(*last_high){
                printf("\nTime since last spoke: %lu",curr_time-*last_high);
                curr_r_speed = spoke_angle/((float)(curr_time-*last_high)/1000000); //convert to seconds from micro
                printf("\nNew speed: %f",curr_r_speed);
            }
            *last_high = curr_time;
        }
        else if(curr_time-*last_high>max_spoke_wait){ //either very slow or not moving
            curr_r_speed = 0;
            //reset
            *last_high = 0; //this will be fixed when we hit the next spoke (also works if we are on a spoke)
            *high = false; 
        }
}

static void manage_pwm(float curr_speed,uint8_t user_throttle,float *effective_throttle,uint32_t *last_pwm_change,uint32_t curr_time,uint slice_num,uint channel){
    const uint32_t update_frequency = 10000; //10ms
    float desired_speed = max_speed*((float)(abs(user_throttle-50))/50);
    bool forwards = user_throttle>50;
    if(user_throttle==50) return;
    if(curr_time-*last_pwm_change>update_frequency){
        printf("\nProposed_change: %f",(desired_speed-curr_speed)/20);
        if(desired_speed>curr_speed){
            float proposed_change = (((forwards)?1:-1)*min((desired_speed-curr_speed)/20,5));
            if(abs(proposed_change)<1) *effective_throttle += proposed_change;
        }
        else if(desired_speed<curr_speed){
            float proposed_change = (((forwards)?-1:1)*min((curr_speed-desired_speed)/20,5));
            if(abs(proposed_change)<1) *effective_throttle += proposed_change;
        }
        if(*effective_throttle>50 != forwards) *effective_throttle = 50 + ((forwards)?1:-1); //if we over-corrected and changed direction, we don't want that

        uint16_t duty = (uint16_t) round(((float)(abs(*effective_throttle-50))/50) * 1000);
        pwm_set_chan_level(slice_num, channel, duty); //set duty cycle

        *last_pwm_change = curr_time;
  
    }
}