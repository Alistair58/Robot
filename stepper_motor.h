//Stepper motor
#define SM_IN1 0
#define SM_IN2 1
#define SM_IN3 2
#define SM_IN4 3


static float current_stepper_rotation = 0;
//positive is clockwise


void stepper_motor_init(void);
void stepper_motor_blocking(float radians,bool clockwise,bool reset);
void reset_stepper(void);


void stepper_motor_init(void){
    //Init stepper motor pins
    gpio_init(SM_IN1);
    gpio_init(SM_IN2);
    gpio_init(SM_IN3);
    gpio_init(SM_IN4);
    gpio_set_dir(SM_IN1,GPIO_OUT);
    gpio_set_dir(SM_IN2,GPIO_OUT);
    gpio_set_dir(SM_IN3,GPIO_OUT);
    gpio_set_dir(SM_IN4,GPIO_OUT);
}

void stepper_motor_blocking(float radians,bool clockwise,bool reset){
    if(radians>((float)M_PI+0.05f)){
        printf("\nAngle is too large for stepper motor - it will break the wires");
        return;
    }
    float rotation_freq = 0.25f; //in hertz
    int phases = 8;
    int phases_per_cycle = 8;
    int gear_ratio = 64;
    float radians_per_cycle = (2*M_PI)/(gear_ratio*phases_per_cycle);
    uint64_t usDelay = (uint64_t) (((float)1/(phases*phases_per_cycle*rotation_freq*gear_ratio))*1000000);
    float radians_travelled = 0;
    if(!clockwise){
        while (radians_travelled<radians) {
            //Sequence taken from: https://wiki.seeedstudio.com/Gear_Stepper_Motor_Driver_Pack/
            //A is IN2, B is IN1, C is IN4, D is IN3
            if(!connected && !reset) return;
            gpio_put(SM_IN2,0);//A
            gpio_put(SM_IN1,1);
            gpio_put(SM_IN4,1);
            gpio_put(SM_IN3,1);
            sleep_us(usDelay);
            gpio_put(SM_IN2,0);
            gpio_put(SM_IN1,0);//AB
            gpio_put(SM_IN4,1);
            gpio_put(SM_IN3,1);
            sleep_us(usDelay);
            gpio_put(SM_IN2,1);
            gpio_put(SM_IN1,0);//B
            gpio_put(SM_IN4,1);
            gpio_put(SM_IN3,1);
            sleep_us(usDelay);
            gpio_put(SM_IN2,1);
            gpio_put(SM_IN1,0);
            gpio_put(SM_IN4,0);//BC
            gpio_put(SM_IN3,1);
            sleep_us(usDelay);
            gpio_put(SM_IN2,1);
            gpio_put(SM_IN1,1);
            gpio_put(SM_IN4,0);//C
            gpio_put(SM_IN3,1);
            sleep_us(usDelay);
            gpio_put(SM_IN2,1);
            gpio_put(SM_IN1,1);
            gpio_put(SM_IN4,0);//CD
            gpio_put(SM_IN3,0);
            sleep_us(usDelay);
            gpio_put(SM_IN2,1);
            gpio_put(SM_IN1,1);
            gpio_put(SM_IN4,1);//D
            gpio_put(SM_IN3,0);
            sleep_us(usDelay);
            gpio_put(SM_IN2,0);
            gpio_put(SM_IN1,1);
            gpio_put(SM_IN4,1);//DA
            gpio_put(SM_IN3,0);
            sleep_us(usDelay);
            radians_travelled+=radians_per_cycle;
            current_stepper_rotation-=radians_per_cycle;
        }
    }
    else{
        while (radians_travelled<radians) {
            if(!connected && !reset) return;
            //Reversed sequence
            gpio_put(SM_IN2,0);
            gpio_put(SM_IN1,1);
            gpio_put(SM_IN4,1);//DA
            gpio_put(SM_IN3,0);
            sleep_us(usDelay);
            gpio_put(SM_IN2,1);
            gpio_put(SM_IN1,1);
            gpio_put(SM_IN4,1);//D
            gpio_put(SM_IN3,0);
            sleep_us(usDelay);
            gpio_put(SM_IN2,1);
            gpio_put(SM_IN1,1);
            gpio_put(SM_IN4,0);//CD
            gpio_put(SM_IN3,0);
            sleep_us(usDelay);
            gpio_put(SM_IN2,1);
            gpio_put(SM_IN1,1);
            gpio_put(SM_IN4,0);//C
            gpio_put(SM_IN3,1);
            sleep_us(usDelay);
            gpio_put(SM_IN2,1);
            gpio_put(SM_IN1,0);
            gpio_put(SM_IN4,0);//BC
            gpio_put(SM_IN3,1);
            sleep_us(usDelay);
            gpio_put(SM_IN2,1);
            gpio_put(SM_IN1,0);//B
            gpio_put(SM_IN4,1);
            gpio_put(SM_IN3,1);
            sleep_us(usDelay);
            gpio_put(SM_IN2,0);
            gpio_put(SM_IN1,0);//AB
            gpio_put(SM_IN4,1);
            gpio_put(SM_IN3,1);
            sleep_us(usDelay);
            gpio_put(SM_IN2,0);//A
            gpio_put(SM_IN1,1);
            gpio_put(SM_IN4,1);
            gpio_put(SM_IN3,1);
            sleep_us(usDelay);
            radians_travelled+=radians_per_cycle;
            current_stepper_rotation+=radians_per_cycle;
        }
    }
    gpio_put(SM_IN2,0);
    gpio_put(SM_IN1,0);
    gpio_put(SM_IN4,0);
    gpio_put(SM_IN3,0);   
}

void reset_stepper(void){
    stepper_motor_blocking(abs(current_stepper_rotation),current_stepper_rotation<0,true);
    //if it is currently positive, we are currently clockwise and need to rotate anti-clockwise and so clockwise is false
}