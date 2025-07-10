#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

//Stepper motor
#define SM_IN1 0
#define SM_IN2 1
#define SM_IN3 2
#define SM_IN4 3

void stepper_motor_init(void);
void stepper_motor_blocking(float radians,bool clockwise,bool reset);
void reset_stepper(void);
bool put_stepper(int a,int b,int c,int d,uint64_t usDelay,bool reset);
void turret_calibration(void);


#endif