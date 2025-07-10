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

void turn_robot(float radians, bool clockwise){
    //set one motor one direction and the other the other way
    //count the number of spokes seen, want same number for both wheels
    //Constants will be empirical

    const float spokes_in_full_rotation =  39.2;
    float target_spokes = roundf(spokes_in_full_rotation*(radians/(2*M_PI)));
    //roundf allows for the robot to equally under- and over-rotate and so on average it ends up in the right place
    //As opposed to floor

    uint16_t l_start_spoke_count = l_spoke_count;
    uint16_t r_start_spoke_count = r_spoke_count;
    int32_t l_s_count = 0;
    int32_t r_s_count = 0;
    uint32_t curr_time = time_us_32();
    const int update_interval = 10; //10ms
    const int forwards_throttle = 90;
    const int backwards_throttle = 10;
    update_throttle((clockwise)?forwards_throttle:backwards_throttle,(clockwise)?backwards_throttle:forwards_throttle);
    while (1) {
        if(auto_mode && connected){
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
            core1_ended();
        }   
    }

    //ONLY TURNING ONE WHEEL VERSION:
    // const float spokes_in_full_rotation =  80?;
    // //For a hard surface (tested on the chair mat) with the plastic lego wheels
    // float target_spokes = roundf(spokes_in_full_rotation*(radians/(2*M_PI)));
    // //roundf allows for the robot to equally under and over rotate and so on average it ends up in the right place
    // //as opposed to floor
    // uint16_t start_spoke_count = clockwise?l_spoke_count:r_spoke_count;
    // int32_t s_count = 0;
    // uint16_t *spoke_count = clockwise?&l_spoke_count:&r_spoke_count;
    // uint32_t curr_time = time_us_32();
    // const int update_interval = 10; //10ms
    // const int forwards_throttle = 90;
    // update_throttle((clockwise)?forwards_throttle:50,(clockwise)?50:forwards_throttle);
    // while (1) {
    //     if(auto_mode && connected){
    //         s_count = (int32_t)*spoke_count - (int32_t)start_spoke_count;
    //         if(s_count<0) s_count+= 0xffff; //spoke count can reset to zero if it goes past 65535
    //         if(((int32_t)target_spokes) <= s_count && ((int32_t)target_spokes)){
    //             update_throttle(50,50);
    //             return;
    //         }
    //         sleep_ms(update_interval);
    //     }
    //     else{
    //         core1_ended();
    //     }   
    // }
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
    uint16_t l_start_spoke_count = l_spoke_count;
    uint16_t r_start_spoke_count = r_spoke_count;
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
            l_s_count = (int32_t)l_spoke_count - (int32_t)l_start_spoke_count;
            if(l_s_count<0) l_s_count+= 0xffff; //spoke count can reset to zero if it goes past 65535
            r_s_count = (int32_t)r_spoke_count - (int32_t)r_start_spoke_count;
            if(r_s_count<0) r_s_count+= 0xffff;
            us_distance = ultrasound_blocking();
            if(us_distance<0.15){ //Also will stop if us_distance = -1
                printf("\nObject is too close");
                update_throttle(50,50);
                if(obstacle_avoidance_enabled){
                    return obstacle_avoidance();
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
        if(distance>0.15){ //we can return back to the main path
            break;
        }
        
    }
    //rejoin the main path
    //ms_backtrack(&ms);
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
            turn_robot(M_PI_2,right); //turn to the side
            movement turn_to_front = {TURN,((right)?1:-1)*M_PI_2};
            ms_push(ms,turn_to_front);
        }
        (*side_check_count)++;
        float distance_travelled = straight(0.15,false); 
        //TODO
        //TURN ON OBSTACLE AVOIDANCE
        movement along_side = {STRAIGHT,distance_travelled};
        ms_push(ms,along_side);
        if(distance_travelled<(0.15-0.03)){
            *side_invalid = true; //if we stopped prematurely, there's an obstacle
            ms_backtrack(ms);
        }
        else{
            stepper_motor_blocking(M_PI_2,!right,false); //look left (to the original front)
            float front_distance = ultrasound_blocking();
            reset_stepper();
            if(front_distance>0.3){
                //Space in front
                turn_robot(M_PI_2,!right);//turn back
                movement turn_to_side = {TURN,((right)?-1:1)*M_PI_2};
                ms_push(ms,turn_to_side);
                *object_in_front = false;
            }
        }
    }
}

void ms_backtrack(movement_stack *ms){
    while(!ms_isEmpty(ms)){
        movement mv = ms_pop(ms);
        switch(mv.type){
            case STRAIGHT:
                turn_robot(M_PI,true); // so that we can go forwards instead of backwards
                straight(mv.value,false); 
                //TODO
                //TURN ON OBSTACLE AVOIDANCE
                //go back the way we came
                turn_robot(M_PI,true);
                break;
            case TURN:
                turn_robot(fabs(mv.value),(mv.value<0)); //turn the opposite way 
                break;
            default:
                break;
        }
    }
}
