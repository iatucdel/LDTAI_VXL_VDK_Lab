#include "software_timer.h"
#include "tim.h"
/*DATA STRUCTURE8*/
uint8_t timer_flag[MAX_TIMERS];
sw_timer timer_arr[MAX_TIMERS];
int timer_arr_cnt = 0;
void timer_arr_add(sw_timer tim){
	uint8_t pos = 0;
	while(pos < timer_arr_cnt && tim.val>=timer_arr[pos].val){
		tim.val -= timer_arr[pos++].val;
	}
	if (pos < timer_arr_cnt) {
	    timer_arr[pos].val -= tim.val;
	}
	int i;
	for(i = timer_arr_cnt; i>pos;--i){
		timer_arr[i] = timer_arr[i-1];
	}
	timer_arr[pos] = tim;
	++timer_arr_cnt;
}
sw_timer timer_arr_pop(){
	sw_timer output = timer_arr[0];
	int i = 1;
	while(i<timer_arr_cnt){
		timer_arr[i-1] = timer_arr[i];
		++i;
	}
	timer_arr[0].val += output.val;
	--timer_arr_cnt;
	return output;
}
/*DATA STRUCTURE8*/
void Check_timer(){
	if(timer_arr_cnt>0 && timer_arr[0].val<=0){
		timer_flag[timer_arr[0].name] = 1;
	}
}
void timer_isr(){
	--timer_arr[0].val;
}
uint8_t TimerExpired(int num){
	if(timer_flag[num] == 1) {
		timer_flag[num] = 0;
		return 1;
	}
	return 0;
}
void set_timer(int num, int ms){
	sw_timer tim = timer_arr_pop();
	tim.name = num;
	tim.val += ms;
	timer_arr_add(tim);
}

void set_timer_init(int num, int ms){
	sw_timer tim;
	tim.name = num;
	tim.val = ms;
	timer_arr_add(tim);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {  if(htim->Instance == TIM2){
	 timer_isr();
  }
 }









