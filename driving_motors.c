#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "globals.h"
#include "driving_motors.h"
#include "stepper_motor.h"
#include "ultrasound.h"
#include "ble.h"
#include "auto_mode.h"
#include "data_structures.h"

//Vehicle-related
uint16_t l_spoke_count = 0; //Needed in movement.h
uint16_t r_spoke_count = 0;
const float wheel_radius = 0.034f; //in metres
static float curr_l_speed = 0;
static float curr_r_speed = 0;
static float max_speed = 3.5f; //when spinning in the air, it reaches 3.5 rads^-1
static uint8_t user_l_throttle = 50;
static uint8_t user_r_throttle = 50;
static float l_duty = 0; //negative for backwards
static float r_duty = 0;
//PID constants
//Proportion is good for making it change direction quickly 
//but as the speed updates causes massive changes
//Integral is good for maintaining the current speed and produces stable behaviour
//but if it is too high, it will be very slow to respond
//Derivative suffers from the drawback of proportion and so has a very low relative weight 
static const float p_weight = 90;
static const float i_weight = 1e-4;
static const float d_weight = 5e4;
static uint l_slice_num; //Initialised in gpio_pins_init
static uint l_channel;
static uint r_slice_num;
static uint r_channel;

void driving_motors_init(void){
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

    //Init motor direction pins
    gpio_init(DM_L_FORWARDS);
    gpio_init(DM_L_BACKWARDS);
    gpio_init(DM_R_FORWARDS);
    gpio_init(DM_R_BACKWARDS);
    gpio_set_dir(DM_L_FORWARDS,GPIO_OUT);
    gpio_set_dir(DM_L_BACKWARDS,GPIO_OUT);
    gpio_set_dir(DM_R_FORWARDS,GPIO_OUT);
    gpio_set_dir(DM_R_BACKWARDS,GPIO_OUT);

    //Init hall effect pins and ADC
    gpio_init(HE_L_POWER);
    gpio_init(HE_R_POWER);
    gpio_set_dir(HE_L_POWER,GPIO_OUT);
    gpio_set_dir(HE_R_POWER,GPIO_OUT);
    gpio_put(HE_L_POWER,1);
    gpio_put(HE_R_POWER,1);

    adc_init();
    adc_gpio_init(HE_L_ADC_GPIO);
    adc_gpio_init(HE_R_ADC_GPIO);
}

void update_throttle(uint8_t left_throttle_new,uint8_t right_throttle_new){ //Called by att_write_callback in ble.h
    //0<= throttle <=100
    if(user_l_throttle!=left_throttle_new || user_r_throttle!=right_throttle_new) printf("\nL: %d R: %d",left_throttle_new,right_throttle_new);
    user_l_throttle = left_throttle_new;
    user_r_throttle = right_throttle_new;
}

void motor_speed_manage_blocking(void){
    //currently recording the time between 2 spokes
    //therefore, can't tell the direction
    bool r_high = false;
    bool l_high = false;
    uint32_t last_r_high = 0; //it starts counting when we hit a spoke
    uint32_t last_l_high = 0;
    uint32_t curr_time = time_us_32();
    const int update_interval = 10; //10ms
    int counter = 0;
    const int pwm_update_ratio = 10; //higher = less updates
    pid_values l_pid_vals = {
        &curr_l_speed,
        &user_l_throttle,
        &l_duty,
        0,
        curr_time,
        curr_time,
        0,
        &curr_time,
        l_slice_num,
        l_channel,
        DM_L_FORWARDS,
        DM_L_BACKWARDS
    };
    pid_values r_pid_vals = {
        &curr_r_speed,
        &user_r_throttle,
        &r_duty,
        0,
        curr_time,
        curr_time,
        0,
        &curr_time,
        r_slice_num,
        r_channel,
        DM_R_FORWARDS,
        DM_R_BACKWARDS
    };
    while (1) {
        if(!connected){
            user_l_throttle = 50;
            user_r_throttle = 50;
        }
        curr_time = time_us_32();
        if(he_update_speed(HE_L_ADC,&l_high,&last_l_high,&curr_l_speed,curr_time)) l_spoke_count = (uint16_t)((uint32_t)l_spoke_count+1) & 0xffff;
        if(he_update_speed(HE_R_ADC,&r_high,&last_r_high,&curr_r_speed,curr_time)) r_spoke_count = (uint16_t)((uint32_t)r_spoke_count+1) & 0xffff;
        if(counter==0){
            manage_pwm(&l_pid_vals);
            manage_pwm(&r_pid_vals);
        }
        counter = (counter+1)%pwm_update_ratio;
        sleep_ms(update_interval);
    }
}

static bool he_update_speed(int ADC_PIN,bool *high, uint32_t *last_high,float *speed,int32_t curr_time){
    //returns true if a spoke has been seen
    const int max_spoke_wait = 1500000; //1.5s 2Pi/5/1.5 = 0.8378 rads^-1 is the smallest speed measurable
    //Sensors
    //1.6v is about normal
    //>1.8v when right in front of a magnet
    //When the larger, flatter side is facing the pole the voltage is largest
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
            *speed = spoke_angle/((float)(curr_time-*last_high)/1000000); //convert to seconds from micro
            printf("\nSpeed: %f",*speed);
            *last_high = curr_time;
            return true;
        }
        *last_high = curr_time;
    }
    else if(curr_time-*last_high>max_spoke_wait){ //either very slow or not moving
        *speed = 0;
        //reset
        *last_high = 0; //this will be fixed when we hit the next spoke (also works if we are on a spoke)
        *high = false; 
    }
    return false;
}

static float manage_pwm(pid_values *pid_vals){
    //Sets the duty cycle using PID such that the output speed is as the throttle desires
    float prev_duty = *(pid_vals->duty);
    float desired_speed = max_speed*((float)((*(pid_vals->user_throttle)-50))/50);
    float curr_speed = *(pid_vals->curr_speed);
    if(*(pid_vals->duty)<0) curr_speed*=-1; //The hall effect doesn't know the direction but pwm does
    float error = desired_speed-curr_speed;

    int dt = ((float)(*(pid_vals->curr_time)-(pid_vals->last_pwm_update))); //In microseconds
    //Proportion
    float proportion = error;
    //Integral
    pid_vals->integral += error*dt;
    //Derivative
    float derivative = (error-pid_vals->last_error)/dt;
    //Anti-windup
    if(abs(i_weight*(pid_vals->integral))>1000) pid_vals->integral = ((pid_vals->integral>0)?1:-1)*1000/i_weight;
    //Reset the integral in scenarios where the speed can't be reached e.g. it can't get to 3.5rads^-1
    if(pid_vals->integral>0!=desired_speed>0) pid_vals->integral = 0;
    //If we're changing direction, a large integral in the other direction doesn't help
    if(*(pid_vals->user_throttle)==50) pid_vals->integral = 0;
    //Sometimes get stuck at 300 duty for example and this can cause movement so slow that hall effect will count speed as 0 
    *(pid_vals->duty) = p_weight*proportion + i_weight*(pid_vals->integral) + d_weight*derivative;
    if(*(pid_vals->duty)>1000) *(pid_vals->duty) = 1000;
    if(*(pid_vals->duty)<-1000) *(pid_vals->duty) = -1000;
    //printf("\nuser_throttle: %d desired_speed: %f curr_speed: %f error: %f (all weighted) proportion: %f integral: %f derivative: %f duty: %f",
    //    *(pid_vals->user_throttle),desired_speed,curr_speed,error,p_weight*proportion,i_weight*(pid_vals->integral),d_weight*derivative,*(pid_vals->duty));
    //Set direction pins
    gpio_put(pid_vals->gpio_forwards,(*(pid_vals->duty)>0)?1:0);
    gpio_put(pid_vals->gpio_backwards,(*(pid_vals->duty)>0)?0:1);
    
    pwm_set_chan_level(pid_vals->slice_num, pid_vals->channel, abs((int)*(pid_vals->duty))); //set duty cycle
    pwm_set_enabled(pid_vals->slice_num, true); // Set the PWM running
    pid_vals->last_pwm_update = *(pid_vals->curr_time);
    //printf("dutyChanged: %d, duty>=1000: %d, time since last duty change: %lu, speed==0: %d",(int)*(pid_vals->duty)!=(int)prev_duty,fabs(*(pid_vals->duty))>=1000,(*(pid_vals->curr_time)-pid_vals->last_duty_change),*(pid_vals->curr_speed)==0);
    if((int)*(pid_vals->duty)!=(int)prev_duty){
        pid_vals->last_duty_change = *(pid_vals->curr_time);
    }
    else if(fabs(*(pid_vals->duty))>=1000 && (*(pid_vals->curr_time)-pid_vals->last_duty_change)>2e6 && *(pid_vals->curr_speed)==0){
        //If we're giving it the beans and have been for the last 2 seconds and aren't moving
        update_throttle(50,50);
        send_stuck_notification();
        set_auto_mode(0);
    }
}


