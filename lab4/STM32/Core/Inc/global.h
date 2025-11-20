#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_

#include "main.h"
#include "button.h"

//TIME DEFINE ms
#define FSM_period 13
#define BUTTON_TIME 10
#define YELLOW_PERIOD 3 //s
#define YELLOW_BLINK_PERIOD 1
#define LED_IDLE_TIME 13
#define LED_period 40
//TIMER LIST
#define RED_LED_TIM 0
#define FSM_TIM 1
#define TRAFIC_LIGHT_TIM 2
#define SEG7_1_TIM 3
#define SEG7_2_TIM 4
#define BUTTON_TIM 8
#define LONG_PRESS_BUT1_TIM 5
#define LONG_PRESS_BUT2_TIM 6
#define LONG_PRESS_BUT3_TIM 7

// FSM STATE
#define FSM_INIT 0
#define FSM_BLINKING_YELLOW 1
#define FSM_CONFIG_RED 2
#define FSM_CONFIG_YELLOW 3
#define FSM_CONFIG_GREEN 4
#define FSM_NORMAL 5
#define FSM_MANUAL_RG 6
#define FSM_MANUAL_RY 7
#define FSM_MANUAL_GR 8
#define FSM_MANUAL_YR 9
#define FSM_END_CONFIG 10

// LED STATE
#define LED_BLINKING_YELLOW 0

#define LED_CONFIG_RED 1
#define LED_CONFIG_YELLOW 2
#define LED_CONFIG_GREEN 3

#define LED_NORMAL_RG 4
#define LED_NORMAL_RY 5
#define LED_NORMAL_GR 6
#define LED_NORMAL_YR 7
#define LED_NORMAL_RG_1 16
#define LED_NORMAL_RY_1 13
#define LED_NORMAL_GR_1 14
#define LED_NORMAL_YR_1 15



#define LED_MANUAL_RG 8
#define LED_MANUAL_RY 9
#define LED_MANUAL_GR 10
#define LED_MANUAL_YR 11

#define LED_MANUAL_YR_1 17
#define LED_MANUAL_RY_1 18

#define LED_IDLE 12

#define LED_BLINKING_YELLOW_1 19
////////////////////////////
extern uint8_t fsm_state;
extern uint8_t led_state;

extern uint8_t L1_red_time;
extern uint8_t L1_yellow_time;
extern uint8_t L1_green_time;
extern uint8_t seg1_val;
extern uint8_t seg1_state;


extern uint8_t L2_red_time;
extern uint8_t L2_yellow_time;
extern uint8_t L2_green_time;
extern uint8_t seg2_val;
extern uint8_t seg2_state;


extern uint8_t L1_red_time_tmp;
extern uint8_t L1_yellow_time_tmp;
extern uint8_t L1_green_time_tmp;


#endif /* INC_GLOBAL_H_ */
