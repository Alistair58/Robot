#ifndef AUTO_MODE_H
#define AUTO_MODE_H

extern void (*core1_action)(void);


void core1_main(void);
void core1_ended(void);
void set_auto_mode(uint8_t value);
void auto_mode_actions(void);

#endif