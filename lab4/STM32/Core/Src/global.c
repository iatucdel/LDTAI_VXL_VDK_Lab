#include "global.h"


uint8_t fsm_state = FSM_INIT;
uint8_t led_state = LED_BLINKING_YELLOW;


uint8_t L1_red_time = 0;
uint8_t L1_yellow_time = 0;
uint8_t L1_green_time = 0;
uint8_t seg1_val = 0;
uint8_t seg1_state = 0;


uint8_t L2_red_time = 0;
uint8_t L2_yellow_time = 0;
uint8_t L2_green_time = 0;
uint8_t seg2_val = 0;
uint8_t seg2_state = 0;

uint8_t L1_red_time_tmp;
uint8_t L1_yellow_time_tmp;
uint8_t L1_green_time_tmp;
