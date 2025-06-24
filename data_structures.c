#include <stdlib.h>
#include "data_structures.h"
#include "globals.h"


movement_stack new_movement_stack(int size){
    movement *movements = (movement*) calloc(size,sizeof(movement));
    if(!movements){
        calloc_error();
    }
    movement_stack ms = {movements,size,-1};
    return ms;

}

bool ms_isFull(movement_stack *ms){
    return ms->sp == (ms->size-1);
}
bool ms_isEmpty(movement_stack *ms){
    return ms->sp == -1;
}
bool ms_push(movement_stack *ms,movement mv){
    if(ms_isFull(ms)) return false;
    ms->movements[++(ms->sp)] = mv;
}
movement ms_pop(movement_stack *ms){
    if(ms_isEmpty(ms)){
        movement result = {INVALID,-1};
        return result;
    }
    return ms->movements[(ms->sp)--];
}

void ms_clear(movement_stack *ms){
    ms->sp = -1;
}
