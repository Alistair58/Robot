//Facing the direction of the robot
//Driving motors
#define DM_L_PWM 21
#define DM_L_BACKWARDS 20
#define DM_L_FORWARDS 19
#define DM_R_PWM 16
#define DM_R_BACKWARDS 17
#define DM_R_FORWARDS 18
uint l_slice_num; //Initialised in gpio_pins_init
uint l_channel;
uint r_slice_num;
uint r_channel;

//Hall effect sensor
#define HE_L_POWER 15
#define HE_R_POWER 22
#define HE_L_ADC_GPIO 27 //GPIO pin numbers
#define HE_R_ADC_GPIO 26
#define HE_L_ADC 1 //ADC pin numbers
#define HE_R_ADC 0

//Vehicle-related
static float curr_l_speed = 0;
static float curr_r_speed = 0;
static float max_speed = 3.5f; //when spinning in the air, it reaches 3.5 rads^-1
static uint8_t user_l_throttle = 50;
static uint8_t user_r_throttle = 50;
static uint16_t l_spoke_count = 0;
static uint16_t r_spoke_count = 0;
static float l_duty = 0; //negative for backwards
static float r_duty = 0;
bool smoothing = false;
bool stop = false;
//PID constants
//Proportion is good for making it change direction quickly 
//but as the speed updates causes massive changes
//Integral is good for maintaining the current speed and produces stable behaviour
//but if it is too high, it will be very slow to respond
//Derivative suffers from the drawback of proportion and so has a very low relative weight 

static const float p_weight = 90;
static const float i_weight = 1e-4;
static const float d_weight = 5e4;

void driving_motors_init(void);
void update_throttle(uint8_t left_throttle_new,uint8_t right_throttle_new);
void motor_speed_manage_blocking(void);
static bool he_update_speed(int ADC_PIN,bool *high,uint32_t *last_high,float *speed,int32_t curr_time);
static float manage_pwm(float curr_speed,uint8_t user_throttle,float *duty,uint32_t *last_pwm_change,uint32_t curr_time,uint slice_num,uint channel,uint8_t gpio_forwards,uint8_t gpio_backwards,float *integral,float last_error);
void turn_robot(float radians, bool clockwise);


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
    uint32_t last_l_pwm_update = curr_time;
    uint32_t last_r_pwm_update = curr_time;
    const int update_interval = 10; //10ms
    int counter = 0;
    const int pwm_update_ratio = 10; //higher = less updates
    float l_integral = 0;
    float r_integral = 0;
    float last_l_error = 0;
    float last_r_error = 0;
    while (1) {
        if(!connected){
            user_l_throttle = 50;
            user_r_throttle = 50;
        }
        if(!smoothing && !stop){
            curr_time = time_us_32();
            if(he_update_speed(HE_L_ADC,&l_high,&last_l_high,&curr_l_speed,curr_time)) l_spoke_count = (uint16_t)((uint32_t)l_spoke_count+1) & 0xffff;
            if(he_update_speed(HE_R_ADC,&r_high,&last_r_high,&curr_r_speed,curr_time)) r_spoke_count = (uint16_t)((uint32_t)r_spoke_count+1) & 0xffff;
            if(counter==0) last_l_error = manage_pwm(curr_l_speed,user_l_throttle,&l_duty,&last_l_pwm_update,curr_time,l_slice_num,l_channel,DM_L_FORWARDS,DM_L_BACKWARDS,&l_integral,last_l_error);
            if(counter==0) last_r_error = manage_pwm(curr_r_speed,user_r_throttle,&r_duty,&last_r_pwm_update,curr_time,r_slice_num,r_channel,DM_R_FORWARDS,DM_R_BACKWARDS,&r_integral,last_r_error);
            counter = (counter+1)%pwm_update_ratio;
        }
        
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



static float manage_pwm(float curr_speed,uint8_t user_throttle,float *duty,uint32_t *last_pwm_change,uint32_t curr_time,uint slice_num,uint channel,uint8_t gpio_forwards,uint8_t gpio_backwards,float *integral,float last_error){
    //Sets the duty cycle using PID such that the output speed is as the throttle desires
    float desired_speed = max_speed*((float)((user_throttle-50))/50);
    if(*duty<0) curr_speed*=-1; //The hall effect doesn't know the direction but pwm does
    float error = desired_speed-curr_speed;

    int dt = ((float)(curr_time-(*last_pwm_change))); //In microseconds
    //Proportion
    float proportion = error;
    //Integral
    *integral += error*dt;
    //Derivative
    float derivative = (error-last_error)/dt;
    //Anti-windup
    if(abs(i_weight*(*integral))>1000) *integral = ((*integral>0)?1:-1)*1000/i_weight;
    //Reset the integral in scenarios where the speed can't be reached e.g. it can't get to 3.5rads^-1
    if(*integral>0!=desired_speed>0) *integral = 0;
    //If we're changing direction, a large integral in the other direction doesn't help
    if(user_throttle==50) *integral = 0;
    //Sometimes get stuck at 300 duty for example and this can cause movement so slow that hall effect will count speed as 0 
    *duty = p_weight*proportion + i_weight*(*integral) + d_weight*derivative;
    if(*duty>1000) *duty = 1000;
    if(*duty<-1000) *duty = -1000;
    printf("\nuser_throttle: %d desired_speed: %f curr_speed: %f error: %f (all weighted) proportion: %f integral: %f derivative: %f duty: %f",user_throttle,desired_speed,curr_speed,error,p_weight*proportion,i_weight*(*integral),d_weight*derivative,*duty);
    //Set direction pins
    gpio_put(gpio_forwards,(*duty>0)?1:0);
    gpio_put(gpio_backwards,(*duty>0)?0:1);
    
    pwm_set_chan_level(slice_num, channel, abs((int)*duty)); //set duty cycle
    pwm_set_enabled(slice_num, true); // Set the PWM running
    *last_pwm_change = curr_time;
}


void turn_robot(float radians, bool clockwise){
    //set one motor one direction and the other the other way
    //count the number of spokes seen, want same number for both wheels
    //Constants will be empirical

    const float spokes_in_full_rotation =  11;
    //For a hard surface (tested on the chair mat) with the plastic lego wheels
    float target_spokes = floor(spokes_in_full_rotation*(radians/(2*M_PI)));
    
    uint16_t l_start_spoke_count = l_spoke_count;
    uint16_t r_start_spoke_count = r_spoke_count;
    int32_t l_s_count = 0;
    int32_t r_s_count = 0;
    uint32_t curr_time = time_us_32();
    const int update_interval = 10; //10ms
    const int forwards_throttle = 90;
    const int backwards_throttle = 0;
    //TODO 
    //add stuck BLE 
    update_throttle((clockwise)?forwards_throttle:backwards_throttle,(clockwise)?backwards_throttle:forwards_throttle);
    while (1) {
        if(auto_mode && connected && !stop){
            l_s_count = (int32_t)l_spoke_count - (int32_t)l_start_spoke_count;
            if(l_s_count<0) l_s_count+= 0xffff; //spoke count can reset to zero if it goes past 65535
            r_s_count = (int32_t)r_spoke_count - (int32_t)r_start_spoke_count;
            if(r_s_count<0) r_s_count+= 0xffff;
            
            if(((int32_t)target_spokes) <= l_s_count && ((int32_t)target_spokes) <= r_s_count){
                update_throttle(50,50);
                return;
            }
            else if(((int32_t)target_spokes) == l_s_count){
                update_throttle(50,(clockwise)?backwards_throttle:forwards_throttle);
            }
            else if(((int32_t)target_spokes) == r_s_count){
                update_throttle((clockwise)?forwards_throttle:backwards_throttle,50);
            }
            sleep_ms(update_interval);
        }
        else{
            break;
        }   
    }
}