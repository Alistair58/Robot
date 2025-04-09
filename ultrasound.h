#define US_TRIG 4
#define US_ECHO 5
#define speed_of_sound 343 //ms^-1


static uint32_t rising_edge_time;
static bool waiting = false;
float *distance;

void ultrasound_init(void);
void ultrasound_trig(float *dest);
void ultrasound_rising_edge(uint gpio,uint32_t events);
void ultrasound_falling_edge(uint gpio,uint32_t events);

void ultrasound_init(void){
    //Init ultrasound pins
    gpio_init(US_TRIG);
    gpio_init(US_ECHO);
    gpio_set_dir(US_TRIG,GPIO_OUT);
    gpio_set_dir(US_ECHO,GPIO_IN);
    gpio_pull_down(US_ECHO);
}

void ultrasound_trig(float *dest){ //Will store the result in dest
    distance = dest;
    *distance = -1;
    gpio_put(US_TRIG,1);
    sleep_us(10);
    gpio_put(US_TRIG,0);
    gpio_set_irq_enabled_with_callback(US_ECHO,GPIO_IRQ_EDGE_RISE,true,&ultrasound_rising_edge);
}

void ultrasound_rising_edge(uint gpio,uint32_t events){
    if(!waiting){
        rising_edge_time = time_us_32();
        printf("\nRising edge detected");
        gpio_set_irq_enabled_with_callback(US_ECHO,GPIO_IRQ_EDGE_FALL,true,&ultrasound_falling_edge);
        waiting = true;
    }
}

void ultrasound_falling_edge(uint gpio,uint32_t events){
    uint32_t falling_edge_time = time_us_32();
    double time_diff_seconds = (falling_edge_time-rising_edge_time)/1000000.0; //10^6
    *distance = (float) speed_of_sound*time_diff_seconds*0.5f; //Assuming a straight line
    printf("\nFalling edge; distance: %f",*distance);
    printf("\nEnd time: %luus Start time: %luus",falling_edge_time,rising_edge_time);
    printf("\nTime difference: %lfs",(time_diff_seconds));
    waiting = false;
}