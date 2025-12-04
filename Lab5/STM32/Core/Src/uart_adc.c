/*
 * uart_adc.c
 *
 *  Created on: Dec 4, 2025
 *      Author: taiko
 */
#include "uart_adc.h"
#include <stdio.h>
#define MAX_BUFFER_SIZE 30

#define CP_FSM_IDLE 0
#define CP_FSM_START 1
#define CP_FSM_RST_1 2
#define CP_FSM_RST_2 3
#define CP_FSM_RST_3 6
#define CP_FSM_OK_1 4
#define CP_FSM_OK_2 5

#define COM_FSM_IDLE 0
#define COM_FSM_SEND_DATA 1
#define COM_FSM_WAITTING 2
#define COM_FSM_SEND_OLD_DATA 3
uint8_t parser_status = 0;
char buffer[MAX_BUFFER_SIZE];
uint8_t temp = 0;
uint8_t command_index = 0;

uint8_t com_status = 0;

uint8_t buffer_flag = 0;

uint16_t ADC_val = 0;
uint16_t old_ADC_val = 0;
uint16_t com_cnt = 0;
char str_response[30];

void get_ADC_value(){
	ADC_val = HAL_ADC_GetValue(&hadc1);
}

void command_parser_fsm() {
	if(temp == '!') parser_status = CP_FSM_START;
	switch(parser_status){
	case CP_FSM_IDLE:
		break;
	case CP_FSM_START:
		if(temp == 'R') parser_status = CP_FSM_RST_1;
		else if(temp == 'O') parser_status = CP_FSM_OK_1;
		else if(temp == '!') parser_status = CP_FSM_START;
		else parser_status = CP_FSM_IDLE;
		break;
	case CP_FSM_RST_1:
		if(temp == 'S') parser_status = CP_FSM_RST_2;
		else parser_status = CP_FSM_IDLE;
		break;
	case CP_FSM_RST_2:
		if(temp == 'T') parser_status = CP_FSM_RST_3;
		else parser_status = CP_FSM_IDLE;
		break;
	case CP_FSM_RST_3:
		if(temp == '#') com_status = COM_FSM_SEND_DATA;
		parser_status = CP_FSM_IDLE;
		break;
	case CP_FSM_OK_1:
		if(temp == 'K') parser_status = CP_FSM_OK_2;
		else parser_status = CP_FSM_IDLE;
		break;
	case CP_FSM_OK_2:
		if(temp == '#') com_status = COM_FSM_IDLE;
		parser_status = CP_FSM_IDLE;
		break;
	default:
		break;
	}
}


void uart_communiation_fsm(){
	switch(com_status){
	case COM_FSM_IDLE:
		break;
	case COM_FSM_SEND_DATA:
		int len = sprintf(str_response, "!ADC=%lu#\r\n", ADC_val);
		HAL_UART_Transmit(&huart2, (uint8_t*)str_response, len, 100);
		old_ADC_val = ADC_val;
		com_cnt = 3000/TIME_UNIT;
		com_status = COM_FSM_WAITTING;
		break;
	case COM_FSM_WAITTING:
		--com_cnt;
		if(com_cnt <= 0) com_status = COM_FSM_SEND_OLD_DATA;
		break;
	case COM_FSM_SEND_OLD_DATA:
		int len2 = sprintf(str_response, "!ADC=%lu#\r\n", old_ADC_val);
		HAL_UART_Transmit(&huart2, (uint8_t*)str_response, len2, 100);
		com_cnt = 3000/TIME_UNIT;
		com_status = COM_FSM_WAITTING;
		break;
	default:
		break;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART2){
		//HAL_UART_Transmit(&huart2, &temp, 1, 50);
		buffer[command_index++] = temp;
		if(command_index == 30) command_index = 0;

		buffer_flag = 1;
		HAL_UART_Receive_IT(&huart2, &temp, 1);
	}
}
