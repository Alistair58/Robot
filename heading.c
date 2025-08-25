#include "heading.h"



float heading = 0;

const double var_r_s = 8e-2;
const double var_r_m = 1e-2;
const double var_w_g = 1e-5;

static double r_s_offset = 0;

static double integral = 0;

void calculate_heading(kalman_values *k_vals){
    //Kalman filter
    //See Notion (Stage 2) for the maths which the variables are named after
    const float spokes_in_full_rotation = 77.8;
    double rotation_fraction = ((double)(l_spoke_count-r_spoke_count))/spokes_in_full_rotation; 

    double r_s = 2*M_PI*rotation_fraction + mag_start_heading + r_s_offset;
    double w_g = get_yaw_rate();
    double time_diff = (double)(time_us_32()-k_vals->last_update)/1e6; //in seconds
    //Has to be here as get_mag_heading is really slow
    k_vals->last_update = time_us_32(); 
    double x_a_priori = heading + w_g*time_diff;
    integral += w_g*time_diff;
    printf("\nIntegral: %f",integral);
    //r_m is between -pi and pi and so we need to "unwrap" it for the filter to work
    double r_m = get_mag_heading();
    r_m += 2*M_PI*round((x_a_priori-r_m)/(2*M_PI));

    if(fabs(r_m-r_s)>M_PI_2){ //probably a bit of wheelspin - bring it in line with magnetometer
        r_s_offset = r_m - (2*M_PI*rotation_fraction + mag_start_heading);
    }
    double p_a_priori = k_vals->prev_variance + var_w_g*time_diff*time_diff;
    double denominator = ((var_r_s+var_r_m)*p_a_priori+var_r_s*var_r_m);

    printf("\nr_s: %f r_m: %f w_g: %f time_diff: %f",r_s,r_m,w_g,time_diff);

    heading = (float)(x_a_priori + (var_r_m*p_a_priori*wrap(r_s,x_a_priori)+var_r_s*p_a_priori*wrap(r_m,x_a_priori))
    / denominator);
    k_vals->prev_variance = p_a_priori*(1-(var_r_m*p_a_priori+var_r_s*p_a_priori)/(denominator));
    
    printf("\nHeading: %f",heading);
}

double wrap(double x,double y){
    //Return the shortest clockwise angle to go from y to x
    double dir_1 = x-y;
    double dir_2 = -(2*M_PI-dir_1);
    return (fabs(dir_1)<fabs(dir_2)) ? dir_1 : dir_2;
}