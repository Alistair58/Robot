#ifndef ULTRASOUND_H
#define ULTRASOUND_H

#define US_TRIG 4
#define US_ECHO 5
#define speed_of_sound 343 //ms^-1

void ultrasound_init(void);
void ultrasound_trig(float *dest);
void ultrasound_rising_edge(uint gpio,uint32_t events);
void ultrasound_falling_edge(uint gpio,uint32_t events);
float ultrasound_blocking();

extern bool us_in_progress;

#endif