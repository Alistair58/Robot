#ifndef MOVEMENT_H
#define MOVEMENT_H
#include "data_structures.h"

void turn_robot(float radians, bool clockwise,int num_wheels_turning);
void turn_robot_sleep(float radians, bool clockwise,int num_wheels_turning,int sleep_millis); //Sleeps after turning the robot
void turn_robot_safe(float radians, bool clockwise); //1-wheeled turn with 1s sleep
float straight(float distance,bool obstacle_avoidance_enabled);
int obstacle_avoidance();
void check_side(bool right,int *side_check_count,bool *side_invalid,bool *object_in_front,movement_stack *ms);
void ms_backtrack(movement_stack *ms);



#endif