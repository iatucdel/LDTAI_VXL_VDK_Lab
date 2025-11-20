#ifndef _INOUT_H_
#define _INOUT_H_
#include "global.h"

extern uint8_t long_pressed_buf [3];



extern void display_seg1();
extern void display_seg2();
extern void main_fsm();
extern void led_fsm();

#endif /*_INOUT_H_*/
