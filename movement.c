#include<stdio.h>
#include<math.h>
#include "pico/stdlib.h"
#include "driving_motors.h"
#include "movement.h"
#include "data_structures.h"
#include "globals.h"
#include "ble.h"
#include "stepper_motor.h"
#include "ultrasound.h"
#include "auto_mode.h"
#include "heading.h"

static const float p_weight = 30; //makes it go
static const float i_overshoot_weight = 8e-6; //Gets it started if we overshot
static const float i_undershoot_weight = 2e-6; //Keeps it going so we don't undershoot
static const float d_weight = 4e6; //damps it

void turn_robot(float radians, bool clockwise,int num_wheels_turning){
    //More unpredictable but less translational movement
    //set one motor one direction and the other the other way
    float heading_change = 0;
    float prev_heading = heading;
    const int update_interval = 40; //40ms - roughly the heading update period
    float throttle = 0; // between -50 and 50 i.e. we could be turning the other way
    uint32_t last_time = time_us_32();
    float last_error = radians;
    float integral = 0;
    uint16_t settle_count = 0;
    uint16_t settle_threshold = 10; //40*10 = 400ms
    float epsilon = 4*M_PI/180; //+-4 degrees
    while (1) {
        if(auto_mode && connected){
            if(heading_change+epsilon >= radians && heading_change-epsilon <= radians ){
                settle_count++; //Like a boxing K.O. count
                if(settle_count>=settle_threshold){
                    update_throttle(50,50);
                    printf("\n\n\n\n\nSTOPPED\n\n\n\n\n");
                    return;
                }
            }
            else{
                settle_count = 0;
            }
            heading_change += heading-prev_heading;
            printf("\nHeading change: %f",heading_change);
            float error = radians-heading_change; //also proportional
            uint32_t curr_time = time_us_32();
            int dt = curr_time-last_time; //in microseconds
            float derivative = (float)(error-last_error)/dt;
            integral += error*dt;
            //anti-windup
            if(error<0 != last_error<0){
                integral = 0; 
            }
            throttle =  p_weight*error +
                        d_weight*derivative; 
            if(error>0 == clockwise){ //i.e. we're undershooting 
                printf("\nUndershooting error>0 %d clockwise %d",error>0,clockwise);
                //Integral causes overshooting on the way there and so isn't very helpful
                //Undershoot weight is typically lower
                if(fabs(integral*i_undershoot_weight)>20.0){
                    integral = ((integral>0)?1:-1) * 20.0/i_undershoot_weight;
                }
                throttle += i_undershoot_weight*integral;
            }
            else{ //i.e. we're overshooting
                printf("\nOvershooting error>0 %d clockwise %d",error>0,clockwise);
                //But is useful for correcting an overshoot
                if(fabs(integral*i_overshoot_weight)>20.0){
                    integral = ((integral>0)?1:-1) * 20.0/i_overshoot_weight;
                }
                throttle += i_overshoot_weight*integral;
            }
            printf("\nP: %f IU: %f IO: %f D: %f Throttle: %f",p_weight*error,i_undershoot_weight*integral,i_overshoot_weight*integral,d_weight*derivative,throttle);
            if(throttle>50) throttle = 50;
            if(throttle<-50) throttle = -50;
            if(num_wheels_turning==1){
                update_throttle((clockwise)?50+throttle:50,(clockwise)?50:50+throttle);
            }
            else{ //default is 2
                update_throttle((clockwise)?50+throttle:50-throttle,(clockwise)?50-throttle:50+throttle);
            }
            prev_heading = heading;
            last_time = curr_time;
            last_error = error;
            sleep_ms(update_interval);
        }
        else{
            core1_ended();
        }   
    }
}

void turn_robot_sleep(float radians, bool clockwise,int num_wheels_turning,int sleep_millis){
    sleep_ms(sleep_millis); //Stops the turn and straight merging which causes an unpredictable turn
    turn_robot(radians,clockwise,num_wheels_turning);
    sleep_ms(sleep_millis); 
}

void turn_robot_safe(float radians, bool clockwise){
    turn_robot_sleep(radians,clockwise,2,1000);
}

float straight(float distance,bool obstacle_avoidance_enabled){ 
    //Distance must be in metres
    //Distance of 0 means go forwards until it finds an obstacle
    //A negative distance means go backwards
    //returns how far the robot actually travelled (may be shorter than argument due to obstacle)
    //obstacle avoidance doesn't work if we're going backwards

    //16 spokes and so the distance of each one is r*(pi/8)
    //r = 3.4cm 
    float target_spokes;
    if(distance==0){ 
        target_spokes = 0;
    }
    else{
        target_spokes = fabs(distance)/(wheel_radius*M_PI/8); //number of r*thetas required
        if(target_spokes<1){
            printf("\nDistance is too small to measure accurately");
            return 0;
        }
    }
    int32_t l_start_spoke_count = l_spoke_count;
    int32_t r_start_spoke_count = r_spoke_count;
    int32_t l_s_count = 0;
    int32_t r_s_count = 0;
    uint32_t curr_time = time_us_32();
    const int update_interval = 10; //10ms
    const int forwards_throttle = 90;
    const int backwards_throttle = 10;
    float us_distance = 0;
    printf("\nTarget spokes: %f",target_spokes);
    reset_stepper();
    if(distance>=0){
        update_throttle(forwards_throttle,forwards_throttle);
    }
    else{
        update_throttle(backwards_throttle,backwards_throttle);
    }
    while(1){
        if(auto_mode && connected){
            printf("\nIterating");
            l_s_count = l_spoke_count - l_start_spoke_count;
            r_s_count = r_spoke_count - r_start_spoke_count;
            us_distance = ultrasound_blocking();
            if(us_distance<0.15){ //Also will stop if us_distance = -1
                printf("\nObject is too close");
                update_throttle(50,50);
                if(obstacle_avoidance_enabled){
                    float obstacle_distance = obstacle_avoidance();
                    int32_t obstacle_spokes = roundf((distance)/(wheel_radius*M_PI/8));
                    l_s_count += obstacle_spokes;
                    r_s_count += obstacle_spokes;
                }
                else{
                    return ((float)(l_s_count+r_s_count)/2)*(wheel_radius*M_PI/8);
                    //return the distance travelled
                }
                
            }
            if(target_spokes>0){
                printf("\ntarget_spokes>0");
                if(((int32_t)target_spokes) <= l_s_count && ((int32_t)target_spokes) <= r_s_count){
                    printf("\nSpokes met");
                    update_throttle(50,50);
                    return ((float)(l_s_count+r_s_count)/2)*(wheel_radius*M_PI/8);
                }
                else if(((int32_t)target_spokes) == l_s_count){
                    update_throttle(50,(distance>0)?forwards_throttle:backwards_throttle);
                }
                else if(((int32_t)target_spokes) == r_s_count){
                    update_throttle((distance>0)?forwards_throttle:backwards_throttle,50);
                }
            }
            sleep_ms(update_interval);
        }  
        else{
            printf("\nNo auto_mode or not connected");
            core1_ended(); //calls exit and so return value is never used
            return -1;
        }
    }   
}

int obstacle_avoidance(){
    bool object_in_front = true;
    bool right_invalid = false;
    bool left_invalid = false;
    movement_stack ms = new_movement_stack(20); //stores the movements that we've made
    int right_side_check_count = 0;
    int left_side_check_count = 0;
    int forwards_distance = 0;
    while(object_in_front){
        if(!right_invalid){
            check_side(true,&right_side_check_count,&right_invalid,&object_in_front,&ms);
        }
        else if(!left_invalid){
            check_side(false,&left_side_check_count,&left_invalid,&object_in_front,&ms);
        }
        else{
            //reverse 
            ms_clear(&ms);
            float distance_reversed = straight(-0.15,false);
            forwards_distance += distance_reversed;
            right_invalid = false;
            left_invalid = false;
            if(distance_reversed>(-0.15+0.03)){
                send_stuck_notification(); 
                return forwards_distance;
                //If we can't reverse or go left and right
            }
        }
    }
    bool object_to_side = true;
    while(object_to_side){
        //we are now facing the front and need to travel perpendicular to the face of the obstacle
        //until we have gone past it
        forwards_distance += straight(0.15,false); 
        //TODO
        //TURN ON OBSTACLE AVOIDANCE
        //e.g. in the scenario we meet an object that looks like a step from a bird's eye view
        stepper_motor_blocking(M_PI_2,right_invalid,false);
        //if the right is invalid, then we are on the left 
        //and so the object is on the right and so stepper requires CW rotation
        float distance = ultrasound_blocking();
        reset_stepper();
        if(distance>0.20){ //we can return back to the main path
            //little bit of extra room for turning
            straight(0.25,false); 
            break;
        }
        
    }
    //rejoin the main path
    ms_backtrack(&ms);
    return forwards_distance;
    
}


void check_side(bool right,int *side_check_count,bool *side_invalid,bool *object_in_front,movement_stack *ms){
    if(*side_check_count==0){
        //if we haven't gone this way yet,then we are facing  the front
        stepper_motor_blocking(M_PI_2,right,false); //look to the side
    }
    float side_distance = ultrasound_blocking();
    reset_stepper();
    if(side_distance>0.3){
        if(*side_check_count==0){
            turn_robot_safe(M_PI_2,right); //turn to the side
            //Push the opposite direction as we don't want a true backtrack
            //We want the robot to face the original direction after it's finished
            movement turn_to_front = {TURN,((right)?-1:1)*M_PI_2};
            ms_push(ms,turn_to_front);
        }
        (*side_check_count)++;
        //TODO
        //TURN ON OBSTACLE AVOIDANCE
        float distance_travelled = straight(0.15,false); 
        movement along_side = {STRAIGHT,distance_travelled};
        ms_push(ms,along_side);
        if(distance_travelled<(0.15-0.03)){
            *side_invalid = true; //if we stopped prematurely, there's an obstacle
            //TODO turn ms_backtrack back on
            //ms_backtrack(ms);
        }
        else{
            stepper_motor_blocking(M_PI_2,!right,false); //look left (to the original front)
            float front_distance = ultrasound_blocking();
            reset_stepper();
            if(front_distance>0.3){
                //Space in front
                turn_robot_safe(M_PI_2,!right);//turn back
                movement turn_to_side = {TURN,((right)?-1:1)*M_PI_2};
                ms_push(ms,turn_to_side);
                *object_in_front = false;
            }
        }
    }
}

void ms_backtrack(movement_stack *ms){
    //We need to do a 180 so that the backtrack can be performed whilst facing the direction of movement
    turn_robot_safe(M_PI,true);
    while(!ms_isEmpty(ms)){
        movement mv = ms_pop(ms);
        switch(mv.type){
            case STRAIGHT:
                //TODO
                //TURN ON OBSTACLE AVOIDANCE
                straight(mv.value,false);
                break;
            case TURN:
                turn_robot_safe(fabs(mv.value),(mv.value<0)); //turn the opposite way
                break;
            default:
                break;
        }
    }
}
