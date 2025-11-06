/*
 * button.h
 *
 *  Created on: Oct 5, 2023
 *      Author: KAI
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "main.h"
#include "stdint.h"
#define NORMAL_STATE GPIO_PIN_SET
#define PRESSED_STATE GPIO_PIN_RESET

#define No_Of_Buttons 3


#define DURATION_LP 50

extern void button_reading();
extern uint8_t is_button_pressed (uint8_t index);
extern uint8_t is_button_pressed_one (uint8_t index);
extern uint8_t is_button_pressed_500ms(uint8_t index);
#endif /* INC_BUTTON_H_ */

