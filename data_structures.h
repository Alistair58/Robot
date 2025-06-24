#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include "pico/stdlib.h"

typedef enum movement_type{
    INVALID = -1,
    STRAIGHT,
    TURN,
}movement_type;

typedef struct movement{
    movement_type type;
    float value; //e.g. distance for straight and angle of rotation for turn (positive for clockwise)
}movement;

typedef struct movement_stack{
    movement *movements;
    int size;
    int sp;
}movement_stack;

movement_stack new_movement_stack(int size);
bool ms_isFull(movement_stack *ms);
bool ms_isEmpty(movement_stack *ms);
bool ms_push(movement_stack *ms,movement mv);
movement ms_pop(movement_stack *ms);
void ms_clear(movement_stack *ms);



#endif