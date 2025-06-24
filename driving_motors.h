#ifndef DRIVING_MOTORS_H
#define DRIVING_MOTOR_H

#include "pico/stdlib.h" //uint
#include "data_structures.h" //movement_stack

//Facing the direction of the robot
//Driving motors
#define DM_L_PWM 21
#define DM_L_BACKWARDS 20
#define DM_L_FORWARDS 19
#define DM_R_PWM 16
#define DM_R_BACKWARDS 17
#define DM_R_FORWARDS 18

//Hall effect sensor
#define HE_L_POWER 15
#define HE_R_POWER 22
#define HE_L_ADC_GPIO 27 //GPIO pin numbers
#define HE_R_ADC_GPIO 26
#define HE_L_ADC 1 //ADC pin numbers
#define HE_R_ADC 0


typedef struct pid_values{ //makes a really messy function signature without this
    float *curr_speed;
    uint8_t *user_throttle;
    float *duty;
    float integral;
    uint32_t last_pwm_update;
    uint32_t last_duty_change;
    float last_error;
    uint32_t *curr_time;
    uint slice_num;
    uint channel;
    uint8_t gpio_forwards;
    uint8_t gpio_backwards;
}pid_values;

void driving_motors_init(void);
void update_throttle(uint8_t left_throttle_new,uint8_t right_throttle_new);
void motor_speed_manage_blocking(void);
static bool he_update_speed(int ADC_PIN,bool *high,uint32_t *last_high,float *speed,int32_t curr_time);
static float manage_pwm(pid_values *pid_vals);
void turn_robot(float radians, bool clockwise);
float straight(float distance,bool obstacle_avoidance_enabled);
void check_side(bool right,int *side_check_count,bool *tried,bool *object_in_front,movement_stack *ms);
int obstacle_avoidance();

//Needed in movement.h
extern uint16_t l_spoke_count; 
extern uint16_t r_spoke_count;
extern const float wheel_radius; //in metres

#endif