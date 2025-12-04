/*
 * uart_adc.h
 *
 *  Created on: Dec 4, 2025
 *      Author: taiko
 */

#ifndef INC_UART_ADC_H_
#define INC_UART_ADC_H_
#include "main.h"
#include "usart.h"
#include "adc.h"
#include "gpio.h"
#define TIME_UNIT 50
#define TIME_COM_OUT 3000


extern uint8_t temp;
extern uint8_t buffer_flag;
extern void command_parser_fsm();
extern void uart_communiation_fsm();
extern void get_ADC_value();
#endif /* INC_UART_ADC_H_ */
