#include "global.h"

uint32_t led_cnt = 0;
uint16_t led7_table[10] = {0b1000000, 0b1111001, 0b0100100, 0b0110000, 0b0011001,
							0b0010010,0b0000010, 0b1111000, 0b0000000, 0b0010000};

uint8_t long_pressed_buf [3] = {0,0,0};
uint8_t seg1_con = 0;
uint8_t seg2_con = 0;


uint8_t read_long_pressed(uint8_t idx){
	if(long_pressed_buf[idx]){
		long_pressed_buf[idx] = 0;
		return 1;
	}
	return 0;
}

void main_fsm(void){
	switch(fsm_state){
		case FSM_INIT:
			L1_red_time = 11;
			L1_yellow_time = 3;
			L1_green_time = 12;

			L2_red_time = 15;
			L2_yellow_time = 3;
			L2_green_time = 8;

			seg1_state = 0;
			seg2_state = 0;

			fsm_state = FSM_BLINKING_YELLOW;
			led_state = LED_BLINKING_YELLOW;
			break;
		case FSM_BLINKING_YELLOW:
			if(is_button_pressed_one(0) && !is_button_pressed_500ms(0)){
				fsm_state = FSM_CONFIG_RED;
				led_state = LED_CONFIG_RED;
				L1_red_time_tmp = 0;
				L1_yellow_time_tmp = 0;
				L1_green_time_tmp = 0;
			}
			if(is_button_pressed_one(1)){
				fsm_state = FSM_NORMAL;
				led_state = LED_NORMAL_GR;
			}
			break;
		case FSM_NORMAL:
			if(is_button_pressed_one(1)){
				fsm_state = FSM_MANUAL_GR;
				led_state = LED_MANUAL_GR;
			}
			break;
		case FSM_MANUAL_GR:
			if(is_button_pressed_one(1)){
				fsm_state = FSM_BLINKING_YELLOW;
				led_state = LED_BLINKING_YELLOW;
			}
			if(is_button_pressed_one(2)){
				fsm_state = FSM_MANUAL_YR;
				led_state = LED_MANUAL_YR;
			}
			break;
		case FSM_MANUAL_YR:
			if(is_button_pressed_one(1)){
				fsm_state = FSM_BLINKING_YELLOW;
				led_state = LED_BLINKING_YELLOW;
			}
			break;
		case FSM_MANUAL_RG:
			if(is_button_pressed_one(1)){
				fsm_state = FSM_BLINKING_YELLOW;
				led_state = LED_BLINKING_YELLOW;
			}
			if(is_button_pressed_one(2)){
				fsm_state = FSM_MANUAL_RY;
				led_state = LED_MANUAL_RY;
			}
			break;
		case FSM_MANUAL_RY:
			if(is_button_pressed_one(1)){
				fsm_state = FSM_BLINKING_YELLOW;
				led_state = LED_BLINKING_YELLOW;
			}
			break;
		case FSM_CONFIG_RED:
			if((read_long_pressed(1)|| is_button_pressed_one(1)) && L1_red_time_tmp < 99){
				++L1_red_time_tmp;
				seg1_val = L1_red_time_tmp;
			}
			if((read_long_pressed(2)|| is_button_pressed_one(2)) && L1_red_time_tmp > 0){
				--L1_red_time_tmp;
				seg1_val = L1_red_time_tmp;
			}
			if(is_button_pressed_one(0)){
				fsm_state = FSM_CONFIG_YELLOW;
				led_state = LED_CONFIG_YELLOW;
			}
			if(read_long_pressed(0)){
				fsm_state = FSM_END_CONFIG;

			}
			break;
		case FSM_CONFIG_YELLOW:
			if((read_long_pressed(1)|| is_button_pressed_one(1)) && L1_yellow_time_tmp < 99){
				++L1_yellow_time_tmp;
				seg1_val = L1_yellow_time_tmp;
			}
			if((read_long_pressed(2)|| is_button_pressed_one(2)) && L1_yellow_time_tmp > 0){
				--L1_yellow_time_tmp;
				seg1_val = L1_yellow_time_tmp;
			}
			if(is_button_pressed_one(0)){
				fsm_state = FSM_CONFIG_GREEN;
				led_state = LED_CONFIG_GREEN;
			}
			if(read_long_pressed(0)){
				fsm_state = FSM_END_CONFIG;

			}
			break;
		case FSM_CONFIG_GREEN:
			if((read_long_pressed(1)|| is_button_pressed_one(1)) && L1_green_time_tmp < 99){
				++L1_green_time_tmp;
				seg1_val = L1_green_time_tmp;
			}
			if((read_long_pressed(2)|| is_button_pressed_one(2)) && L1_green_time_tmp > 0){
				--L1_green_time_tmp;
				seg1_val = L1_green_time_tmp;
			}
			if(is_button_pressed_one(0)){
				fsm_state = FSM_CONFIG_RED;
				led_state = LED_CONFIG_RED;
			}
			if(read_long_pressed(0)){
				fsm_state = FSM_END_CONFIG;

			}
			break;
		case FSM_END_CONFIG:
			if((L1_yellow_time_tmp + 2 < L1_red_time_tmp) &&
				(L1_yellow_time_tmp>0) && (L1_green_time_tmp >0)){
				L1_red_time = L1_red_time_tmp;
				L1_yellow_time = L1_yellow_time_tmp;
				L1_green_time = L1_green_time_tmp;
				L2_red_time = L1_green_time + L1_yellow_time ;
				L2_yellow_time = L1_yellow_time;
				L2_green_time = L1_red_time - L1_yellow_time;
			}
			fsm_state = FSM_BLINKING_YELLOW;
			led_state = LED_BLINKING_YELLOW;
			break;
		default:
			fsm_state = FSM_INIT;

	}
}

void trafic_led_off(){
	HAL_GPIO_WritePin(GPIOA, L1_RED_Pin | L1_YELLOW_Pin | L1_GREEN_Pin |
							L2_RED_Pin | L2_YELLOW_Pin | L2_GREEN_Pin, SET);
}
void trafic_led_YELLOW(){
	HAL_GPIO_WritePin(GPIOA, L1_RED_Pin | L1_YELLOW_Pin | L1_GREEN_Pin |
							L2_RED_Pin | L2_YELLOW_Pin | L2_GREEN_Pin, SET);
	HAL_GPIO_WritePin(GPIOA, L1_YELLOW_Pin, RESET);
}
void trafic_led_RED(){
	HAL_GPIO_WritePin(GPIOA, L1_RED_Pin | L1_YELLOW_Pin | L1_GREEN_Pin |
							L2_RED_Pin | L2_YELLOW_Pin | L2_GREEN_Pin, SET);
	HAL_GPIO_WritePin(GPIOA, L1_RED_Pin, RESET);
}
void trafic_led_GREEN(){
	HAL_GPIO_WritePin(GPIOA, L1_RED_Pin | L1_YELLOW_Pin | L1_GREEN_Pin |
							L2_RED_Pin | L2_YELLOW_Pin | L2_GREEN_Pin, SET);
	HAL_GPIO_WritePin(GPIOA, L1_GREEN_Pin, RESET);
}


void trafic_led_GR(){
	HAL_GPIO_WritePin(GPIOA, L1_RED_Pin | L1_YELLOW_Pin | L1_GREEN_Pin |
							L2_RED_Pin | L2_YELLOW_Pin | L2_GREEN_Pin, SET);
	HAL_GPIO_WritePin(GPIOA, L1_GREEN_Pin | L2_RED_Pin, RESET);
}
void trafic_led_YR(){
	HAL_GPIO_WritePin(GPIOA, L1_RED_Pin | L1_YELLOW_Pin | L1_GREEN_Pin |
							L2_RED_Pin | L2_YELLOW_Pin | L2_GREEN_Pin, SET);
	HAL_GPIO_WritePin(GPIOA, L1_YELLOW_Pin | L2_RED_Pin, RESET);
}

void trafic_led_RG(){
	HAL_GPIO_WritePin(GPIOA, L1_RED_Pin | L1_YELLOW_Pin | L1_GREEN_Pin |
							L2_RED_Pin | L2_YELLOW_Pin | L2_GREEN_Pin, SET);
	HAL_GPIO_WritePin(GPIOA, L1_RED_Pin | L2_GREEN_Pin, RESET);
}
void trafic_led_RY(){
	HAL_GPIO_WritePin(GPIOA, L1_RED_Pin | L1_YELLOW_Pin | L1_GREEN_Pin |
							L2_RED_Pin | L2_YELLOW_Pin | L2_GREEN_Pin, SET);
	HAL_GPIO_WritePin(GPIOA, L1_RED_Pin | L2_YELLOW_Pin, RESET);
}

void turn_off_seg1(){
	HAL_GPIO_WritePin(L1_CON1_GPIO_Port, L1_CON1_Pin | L1_CON2_Pin, RESET);
}
void turn_off_seg2(){
	HAL_GPIO_WritePin(L2_CON1_GPIO_Port, L2_CON1_Pin | L2_CON2_Pin, RESET);
}


void led_fsm(void){
	switch(led_state){
		case LED_BLINKING_YELLOW:

			seg1_state = 0;
			seg2_state = 0;
			led_cnt = (YELLOW_BLINK_PERIOD*1000)/LED_period;
			trafic_led_off();
			HAL_GPIO_TogglePin(L1_YELLOW_GPIO_Port, L1_YELLOW_Pin | L2_YELLOW_Pin);
			led_state = LED_BLINKING_YELLOW_1;
			break;
		case LED_BLINKING_YELLOW_1:

			--led_cnt;
			if(led_cnt <= 0){
				led_cnt = (YELLOW_BLINK_PERIOD*1000)/LED_period;
				HAL_GPIO_TogglePin(L1_YELLOW_GPIO_Port, L1_YELLOW_Pin | L2_YELLOW_Pin);
			}
			break;
		case LED_CONFIG_RED:
			seg1_state = 1;
			seg2_state = 0;
			seg1_val = L1_red_time_tmp;
			trafic_led_RED();
			led_state = LED_IDLE;

			break;
		case LED_CONFIG_YELLOW:
			seg1_state = 1;
			seg2_state = 0;
			seg1_val = L1_yellow_time_tmp;
			turn_off_seg1();
			trafic_led_YELLOW();
			led_state = LED_IDLE;

			break;
		case LED_CONFIG_GREEN:
			seg1_state = 1;
			seg2_state = 0;
			seg1_val = L1_green_time_tmp;
			trafic_led_GREEN();
			led_state = LED_IDLE;

			break;
		case LED_IDLE:

			break;
		case LED_NORMAL_GR:

			led_cnt = (L1_green_time * 1000)/LED_period;
			seg1_state = 1;
			seg2_state = 1;
			seg1_val = L1_green_time;
			seg2_val = L2_red_time;
			turn_off_seg1();
			turn_off_seg2();
			trafic_led_GR();
			led_state = LED_NORMAL_GR_1;
			break;
		case LED_NORMAL_GR_1:

			--led_cnt;
			seg1_val = (led_cnt * LED_period)/1000 + 1;
			seg2_val = seg1_val + L1_yellow_time;
			if(led_cnt <= 0) led_state = LED_NORMAL_YR;
			break;
		case LED_NORMAL_YR:

			led_cnt = (L1_yellow_time * 1000)/LED_period;
			seg1_val = L1_yellow_time;
			turn_off_seg1();
			turn_off_seg2();
			trafic_led_YR();
			led_state = LED_NORMAL_YR_1;
			break;
		case LED_NORMAL_YR_1:

			--led_cnt;
			seg1_val = (led_cnt * LED_period)/1000 + 1;
			seg2_val = seg1_val ;
			if(led_cnt <= 0) led_state = LED_NORMAL_RG;
			break;
		case LED_NORMAL_RG:

			led_cnt = (L2_green_time * 1000)/LED_period;
			seg1_val = L1_red_time;
			seg2_val = L2_green_time;
			turn_off_seg1();
			turn_off_seg2();
			trafic_led_RG();
			led_state = LED_NORMAL_RG_1;
			break;
		case LED_NORMAL_RG_1:

			--led_cnt;
			seg2_val = (led_cnt * LED_period)/1000 + 1;
			seg1_val = seg2_val + L2_yellow_time ;
			if(led_cnt <= 0) led_state = LED_NORMAL_RY;
			break;
		case LED_NORMAL_RY:
			seg2_val = L2_yellow_time;

			led_cnt = (L2_yellow_time * 1000)/LED_period;
			turn_off_seg1();
			turn_off_seg2();
			trafic_led_RY();
			led_state = LED_NORMAL_RY_1;
			break;
		case LED_NORMAL_RY_1:

			--led_cnt;
			seg2_val = (led_cnt * LED_period)/1000 + 1;
			seg1_val = seg2_val ;
			if(led_cnt <= 0) led_state = LED_NORMAL_GR;
			break;
		case LED_MANUAL_GR:
			seg1_state = 0;
			seg2_state = 0;
			trafic_led_GR();
			fsm_state = FSM_MANUAL_GR;
			led_state = LED_IDLE;

			break;
		case LED_MANUAL_YR:

			led_cnt = (YELLOW_PERIOD * 1000)/LED_period;
			seg1_state = 1;
			seg2_state = 0;
			seg1_val = YELLOW_PERIOD;
			turn_off_seg1();
			trafic_led_YR();
			led_state = LED_MANUAL_YR_1;
			break;
		case LED_MANUAL_YR_1:

			--led_cnt;
			seg1_val = (led_cnt * LED_period)/1000 + 1;
			if(led_cnt <= 0) led_state = LED_MANUAL_RG;
			break;
		case LED_MANUAL_RG:
			seg1_state = 0;
			seg2_state = 0;
			trafic_led_RG();
			led_state = LED_IDLE;
			fsm_state = FSM_MANUAL_RG;

			break;
		case LED_MANUAL_RY:

			led_cnt = (YELLOW_PERIOD * 1000)/LED_period;
			seg1_state = 0;
			seg2_state = 1;
			seg2_val = YELLOW_PERIOD;
			turn_off_seg2();
			trafic_led_RY();
			led_state = LED_MANUAL_RY_1;
			break;
		case LED_MANUAL_RY_1:
			--led_cnt;
			seg2_val = (led_cnt * LED_period)/1000 + 1;
			if(led_cnt <= 0) led_state = LED_MANUAL_GR;
			break;
		default:



	}
}


void display_seg1(void){
	turn_off_seg1();
	if(seg1_state == 0) return;
	uint16_t tens = seg1_val/10;
	uint16_t units = seg1_val - tens*10;
	uint16_t out;
	if(seg1_con){
		out = led7_table[units];
		uint16_t buf = 0b1111111;
		HAL_GPIO_WritePin(GPIOB, buf, RESET);
		HAL_GPIO_WritePin(GPIOB, out, SET);
		HAL_GPIO_WritePin(L1_CON1_GPIO_Port, L1_CON1_Pin, SET);
		seg1_con = 0;
	}else{
		out = led7_table[tens];
		uint16_t buf = 0b1111111;
		HAL_GPIO_WritePin(GPIOB, buf, RESET);
		HAL_GPIO_WritePin(GPIOB, out, SET);
		HAL_GPIO_WritePin(L1_CON2_GPIO_Port, L1_CON2_Pin, SET);
		seg1_con = 1;
	}
}

void display_seg2(void){
	turn_off_seg2();
	if(seg2_state == 0) return;
	uint16_t tens = seg2_val/10;
	uint16_t units = seg2_val - tens*10;
	uint16_t out;
	if(seg2_con){
		out = led7_table[units];
		uint16_t buf = 0b1111111;
		buf= buf<<9;
		out= out<<9;
		HAL_GPIO_WritePin(GPIOB, buf, RESET);
		HAL_GPIO_WritePin(GPIOB, out, SET);
		HAL_GPIO_WritePin(L2_CON1_GPIO_Port, L2_CON1_Pin, SET);
		seg2_con = 0;
	}else{
		out = led7_table[tens];
		uint16_t buf = 0b1111111;
		buf= buf<<9;
		out= out<<9;
		HAL_GPIO_WritePin(GPIOB, buf, RESET);
		HAL_GPIO_WritePin(GPIOB, out, SET);
		HAL_GPIO_WritePin(L2_CON2_GPIO_Port, L2_CON2_Pin, SET);
		seg2_con = 1;
	}

}





