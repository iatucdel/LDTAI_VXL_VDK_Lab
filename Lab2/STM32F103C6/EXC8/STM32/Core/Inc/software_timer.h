#ifndef SOFTWARE_TIMER_H_
#define SOFTWARE_TIMER_H_
/* Includes */
#include "stdint.h"

/* Define */
#define MAX_TIMERS 20
typedef struct{
	int name;
	int val;
} sw_timer;
/* Variables */
extern uint8_t timer_flag[MAX_TIMERS];


/* Functions */
extern void timer_isr();
extern uint8_t TimerExpired(int num);
extern void set_timer(int num, int ms);
extern void set_timer_init(int num, int ms);
extern void Check_timer();
#endif /* SOFTWARE_TIMER_H_ */
