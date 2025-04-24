//Facing the direction of the robot
//Driving motors
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
//Hall effect sensor
#define HE_L_POWER 15
#define HE_R_POWER 22
#define HE_L_ADC_GPIO 27 //GPIO pin numbers
#define HE_R_ADC_GPIO 26
#define HE_L_ADC 1 //ADC pin numbers
#define HE_R_ADC 0


static float curr_l_speed = 0;
static float curr_r_speed = 0;
static float max_speed = 20; //when spinning in the air, it reaches 20 rads^-1
uint8_t user_l_throttle = 50;
uint8_t user_r_throttle = 50;
float effective_l_throttle = 50;
float effective_r_throttle = 50;
uint16_t l_spoke_count = 0;
uint16_t r_spoke_count = 0;

void driving_motors_init(void);
void update_throttle(uint8_t left_throttle_new,uint8_t right_throttle_new);
void update_throttle_smooth(uint8_t left_throttle_new,uint8_t right_throttle_new);
void _update_throttle_smooth(uint8_t left_throttle_new,uint8_t right_throttle_new,bool *done);
void motor_speed_manage_blocking(void);
static bool he_update_speed(int ADC_PIN,bool *high, uint32_t *last_high,int32_t curr_time);
static void manage_pwm(float curr_speed,uint8_t user_throttle,float *effective_throttle,uint32_t *last_pwm_change,uint32_t curr_time,uint slice_num,uint channel);
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

void update_throttle_smooth(uint8_t left_throttle_new,uint8_t right_throttle_new){ //Called by att_write_callback in ble.h
    //0<= throttle <=100
    //Updates the throttle smoothly (if the new throttle is not 50)
    printf("\nL: %d R: %d",left_throttle_new,right_throttle_new);
    uint16_t smooth_update_interval = 20; //20ms
    //50 -> 100 = 20*(50/2) = 500ms which I think is nice
    uint8_t max_update = 2; //
    //Set direction pins
    gpio_put(DM_L_FORWARDS,(left_throttle_new<50)?0:1);
    gpio_put(DM_L_BACKWARDS,(left_throttle_new<50)?1:0);
    gpio_put(DM_R_FORWARDS,(right_throttle_new<50)?0:1);
    gpio_put(DM_R_BACKWARDS,(right_throttle_new<50)?1:0);

    

    int l_throttle_change = (int)left_throttle_new-user_l_throttle;
    int r_throttle_change = (int)right_throttle_new-user_r_throttle;
    if(left_throttle_new==50){
        effective_l_throttle = 50;
        pwm_set_chan_level(l_slice_num, l_channel, 0);
        pwm_set_enabled(l_slice_num, true);
        user_l_throttle = 50;
        l_throttle_change = 0; //instant braking
    }
    if(right_throttle_new == 50){
        effective_r_throttle = 50;
        pwm_set_chan_level(r_slice_num, r_channel, 0);
        pwm_set_enabled(r_slice_num, true);
        user_r_throttle = 50;
        r_throttle_change = 0;
    }
    while(!(l_throttle_change==0 && r_throttle_change==0)){ 
        if(l_throttle_change!=0){
            if(abs(l_throttle_change)<max_update){
                user_l_throttle+= l_throttle_change;
                l_throttle_change = 0;
            }
            else{
                l_throttle_change += (l_throttle_change<0)?max_update:-max_update;
                user_l_throttle+=(l_throttle_change<0)?-max_update:max_update;
            }
        }
        if(r_throttle_change!=0){
            if(abs(r_throttle_change)<max_update){
                user_r_throttle+=r_throttle_change;
                r_throttle_change = 0;
            }
            else{
                r_throttle_change += (r_throttle_change<0)?max_update:-max_update;
                user_r_throttle+=(r_throttle_change<0)?-max_update:max_update;
            }
        } 
        printf("\nSmooth update, l_throttle: %d l_throttle_change: %d r_throttle: %d r_throttle_change: %d",user_l_throttle,l_throttle_change,user_r_throttle,r_throttle_change);  
        //Set the initial pwm duty cycle
        //This may be changed by the hall effect sensor
        uint16_t l_duty = (uint16_t) round(((float)(abs(user_l_throttle-50))/50) * 1000);
        //Abs as we want the values furthest from 50 (0 and 100) to give the fastest outputs
        uint16_t r_duty = (uint16_t) round(((float)(abs(user_r_throttle-50))/50) * 1000);
        pwm_set_chan_level(l_slice_num, l_channel, l_duty); //set duty cycle
        pwm_set_chan_level(r_slice_num, r_channel, r_duty);

        pwm_set_enabled(l_slice_num, true); // Set the PWM running
        pwm_set_enabled(r_slice_num, true);

        sleep_ms(smooth_update_interval);
    }
}

void _update_throttle_smooth(uint8_t left_throttle_new,uint8_t right_throttle_new,bool *done){
    update_throttle_smooth(left_throttle_new,right_throttle_new);
    *done = true;
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
    while (1) {
        if(!connected){
            user_l_throttle = 50;
            user_r_throttle = 50;
        }
        curr_time = time_us_32();
        if(he_update_speed(HE_L_ADC,&l_high,&last_l_high,curr_time)) l_spoke_count = (uint16_t)((uint32_t)l_spoke_count+1) & 0xffff;
        if(he_update_speed(HE_R_ADC,&r_high,&last_r_high,curr_time)) r_spoke_count = (uint16_t)((uint32_t)r_spoke_count+1) & 0xffff;
        manage_pwm(curr_l_speed,user_l_throttle,&effective_l_throttle,&last_l_pwm_update,curr_time,l_slice_num,l_channel);
        manage_pwm(curr_r_speed,user_r_throttle,&effective_r_throttle,&last_r_pwm_update,curr_time,r_slice_num,r_channel);
        sleep_ms(update_interval);
    }
}

static bool he_update_speed(int ADC_PIN,bool *high, uint32_t *last_high,int32_t curr_time){
    //returns true if a spoke has been seen
    const int max_spoke_wait = 2000000; //2s
    //left sensor:
    //1.62v is about normal
    //>1.9v when right in front of a magnet
    //all 5 magnets behave similarly

    //right sensor:
    //1.52v is about normal
    //>1.75v when magnet
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
                curr_r_speed = spoke_angle/((float)(curr_time-*last_high)/1000000); //convert to seconds from micro
                *last_high = curr_time;
                return true;
            }
            *last_high = curr_time;
        }
        else if(curr_time-*last_high>max_spoke_wait){ //either very slow or not moving
            curr_r_speed = 0;
            //reset
            *last_high = 0; //this will be fixed when we hit the next spoke (also works if we are on a spoke)
            *high = false; 
        }
        return false;
}



static void manage_pwm(float curr_speed,uint8_t user_throttle,float *effective_throttle,uint32_t *last_pwm_change,uint32_t curr_time,uint slice_num,uint channel){
    float desired_speed = max_speed*((float)(abs(user_throttle-50))/50);
    bool forwards = user_throttle>50;
    if(user_throttle==50) return;
    if(desired_speed>curr_speed){
        float proposed_change = (((forwards)?1:-1)*min((desired_speed-curr_speed)/20,1));
        *effective_throttle += proposed_change;
        if(user_throttle==user_r_throttle) printf("\nCurr speed: %f desired: %f effective throttle: %f proposed change: %f",curr_speed,desired_speed,*effective_throttle,proposed_change);   
    }
    else if(desired_speed<curr_speed){
        float proposed_change = (((forwards)?-1:1)*min((curr_speed-desired_speed)/20,1));
        *effective_throttle += proposed_change;
    }
    if(*effective_throttle>100) *effective_throttle = 100;
    if(*effective_throttle<0) *effective_throttle = 0;
    if(*effective_throttle>50 != forwards) *effective_throttle = 50 + ((forwards)?1:-1); //if we over-corrected and changed direction, we don't want that
    
    uint16_t duty = (uint16_t) round(((float)(abs(*effective_throttle-50))/50) * 1000);
    pwm_set_chan_level(slice_num, channel, duty); //set duty cycle
    *last_pwm_change = curr_time;
}


void turn_robot(float radians, bool clockwise){
    //set one motor one direction and the other the other way
    //count the number of spokes seen, want same number for both wheels
    //I think that the distance needed for a turn will vary based on friction
    //and hence the constants will be empirical

    const float spokes_in_full_rotation = 200;
    float target_spokes = floor(spokes_in_full_rotation*(radians/(2*M_PI)));
    
    uint16_t l_start_spoke_count = l_spoke_count;
    uint16_t r_start_spoke_count = r_spoke_count;
    int32_t l_s_count = 0;
    int32_t r_s_count = 0;
    uint32_t curr_time = time_us_32();
    const int update_interval = 10; //10ms

    //TODO 
    //add stuck BLE 
    update_throttle_smooth((clockwise)?80:20,(clockwise)?20:80);
    while (1) {
        l_s_count = (int32_t)l_spoke_count - (int32_t)l_start_spoke_count;
        if(l_s_count<0) l_s_count+= 0xffff;
        r_s_count = (int32_t)r_spoke_count - (int32_t)r_start_spoke_count;
        if(r_s_count<0) r_s_count+= 0xffff;
        
        if(((int32_t)target_spokes) <= l_s_count && ((int32_t)target_spokes) <= r_s_count){
            update_throttle(50,50);
            return;
        }
        else if(((int32_t)target_spokes) == l_s_count){
            update_throttle_smooth(50,(clockwise)?20:80);
        }
        else if(((int32_t)target_spokes) == r_s_count){
            update_throttle_smooth((clockwise)?80:20,50);
        }
        sleep_ms(update_interval);
    }

}