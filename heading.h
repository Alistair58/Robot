#ifndef HEADING_H
#define HEADING_H

#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>
#include "imu.h"
#include "driving_motors.h"

typedef struct kalman_values{
    double prev_variance;
    uint32_t last_update;
} kalman_values;

extern float heading;
void calculate_heading(kalman_values *k_vals);
double wrap(double x,double y);

#endif