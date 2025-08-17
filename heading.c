#include "heading.h"



float heading = 0;

const double var_r_s = 4e-4;
const double var_r_m = 7.5e-5;
const double var_w_g = 0.015;

static double wrap(double x,double y);

void calculate_heading(kalman_values *k_vals){
    //Kalman filter
    //See Notion (Stage 2) for the maths which the variables are named after
    const float spokes_in_full_rotation = 77.8;
    double rotation_fraction = ((double)(l_spoke_count-r_spoke_count))/spokes_in_full_rotation; 
    //between -pi and pi
    double r_s = 2*M_PI*(rotation_fraction - round(rotation_fraction)) + mag_start_heading;
    double r_m = get_mag_heading();
    double w_g = get_yaw_rate();
    double time_diff = (double)(time_us_32()-k_vals->last_update)/1e6; //in seconds
    double x_a_priori = heading + w_g*time_diff*time_diff;
    double p_a_priori = k_vals->prev_variance + var_w_g;
    double denominator = ((var_r_s+var_r_m)*p_a_priori+var_r_s*var_r_m);

    printf("\nr_s: %f r_m: %f w_g: %f time_diff: %f x_a_priori: %f",r_s,r_m,w_g,time_diff,x_a_priori);

    heading = (float)(x_a_priori + (var_r_m*p_a_priori*wrap(r_s,x_a_priori)+var_r_s*p_a_priori*wrap(r_m,x_a_priori))
    / denominator);
    while(heading<-M_PI) heading += 2*M_PI;
    while(heading>M_PI) heading -= 2*M_PI;
    k_vals->prev_variance = p_a_priori*(1-(var_r_m*p_a_priori+var_r_s*p_a_priori)/(denominator));
    k_vals->last_update = time_us_32();
    printf("\nHeading: %f",heading);
}

static double wrap(double x,double y){
    //Return the shortest clockwise angle to go from y to x
    double dir_1 = x-y;
    double dir_2 = -(2*M_PI-dir_1);
    return (fabs(dir_1)<fabs(dir_2)) ? dir_1 : dir_2;
}