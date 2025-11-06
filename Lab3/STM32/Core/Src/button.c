#include "button.h"
static GPIO_PinState But_Buffer[No_Of_Buttons] = {NORMAL_STATE};
uint8_t But_Buffer_age[No_Of_Buttons] = {0};

static GPIO_PinState But_DB_Buffer0[No_Of_Buttons] = {NORMAL_STATE};
static GPIO_PinState But_DB_Buffer1[No_Of_Buttons] = {NORMAL_STATE};
static GPIO_PinState But_DB_Buffer2[No_Of_Buttons] = {NORMAL_STATE};

static GPIO_PinState LP_But_Buffer[No_Of_Buttons]= {NORMAL_STATE};
static uint16_t LP_But_Counter[No_Of_Buttons] = {0};

GPIO_TypeDef* But_Port_arr[No_Of_Buttons] = {BTN0_GPIO_Port,BTN1_GPIO_Port,BTN2_GPIO_Port};
uint16_t But_Pin_arr[No_Of_Buttons] = {BTN0_Pin,BTN1_Pin,BTN2_Pin};

void button_reading(){
	for(uint8_t i = 0; i<No_Of_Buttons;++i){
		But_DB_Buffer2[i] = But_DB_Buffer1[i];
		But_DB_Buffer1[i] = But_DB_Buffer0[i];
		But_DB_Buffer0[i] = HAL_GPIO_ReadPin(But_Port_arr[i], But_Pin_arr[i]);
		if(But_DB_Buffer2[i] == But_DB_Buffer1[i] && But_DB_Buffer2[i] == But_DB_Buffer0[i]){
			But_Buffer[i] = But_DB_Buffer0[i];
		}
		if(But_Buffer[i] == PRESSED_STATE){
			if(LP_But_Counter[i]<DURATION_LP){
				++LP_But_Counter[i];
			}
			else{
				LP_But_Buffer[i] = PRESSED_STATE;
			}
		}
		else{
			LP_But_Counter[i] = 0;
			LP_But_Buffer[i] = NORMAL_STATE;
			But_Buffer_age[i] = 0;
		}
	}
}
uint8_t is_button_pressed (uint8_t index){
	if(index >= No_Of_Buttons) return 0;
	return (But_Buffer[index] == PRESSED_STATE);
}
uint8_t is_button_pressed_one (uint8_t index){
	if(index >= No_Of_Buttons) return 0;
	uint8_t out = (!But_Buffer_age[index]) && (But_Buffer[index] == PRESSED_STATE);
	But_Buffer_age[index] = (But_Buffer[index] == PRESSED_STATE);
	return out;
}
uint8_t is_button_pressed_500ms (uint8_t index){
	if(index >= No_Of_Buttons) return 0xFF;
	return (LP_But_Buffer[index] == PRESSED_STATE);
}
