#include "SCH.h"
#define SCH_MAX_TASK 40
uint32_t Task_ID = 1;
uint8_t Task_cnt = 0;

typedef void (*Task_Function_t)(void);
Task_Function_t task_arr[SCH_MAX_TASK];

typedef struct{
	void(*pTask)(void);
	int32_t Delay;
	int32_t Period;
	uint8_t RunMe;
	int32_t TaskID;
}sTask;
sTask SCH_tasks_G[SCH_MAX_TASK];


void SCH_Init(void){
	uint8_t i;
	Task_cnt = 0;
	Task_ID = 1;
	for(i = 0; i < SCH_MAX_TASK; ++i){
		SCH_tasks_G[i].pTask = 0;
		SCH_tasks_G[i].Delay = 0;
		SCH_tasks_G[i].Period = 0;
		SCH_tasks_G[i].RunMe = 0;
	}
}

void SCH_Update(void){
	if(Task_cnt <= 0) return ;
	--SCH_tasks_G[0].Delay;

}

unsigned char SCH_Add_Task(void (* pFunction) () , uint32_t DELAY, uint32_t PERIOD){
	if(Task_cnt >= SCH_MAX_TASK) return 3;
	if(pFunction == 0) return 1;
	sTask newTask;
	newTask.pTask = pFunction;
	newTask.Delay = DELAY;
	newTask.Period = PERIOD;
	newTask.TaskID = Task_ID;
	newTask.RunMe = 0;
	uint8_t i = 0 ;
	while(i < Task_cnt && (newTask.Delay > SCH_tasks_G[i].Delay ||
			(newTask.Delay == SCH_tasks_G[i].Delay && newTask.Period <= SCH_tasks_G[i].Period) )){
		newTask.Delay -= SCH_tasks_G[i].Delay;
		++i;
	}
	if(i < Task_cnt){
		SCH_tasks_G[i].Delay -= newTask.Delay;
		uint8_t j;
		for(j = Task_cnt; j >i;--j){
			SCH_tasks_G[j] = SCH_tasks_G[j-1];
		}
	}
		SCH_tasks_G[i] = newTask;
		++Task_cnt;
		++Task_ID;
		return 0;
}
unsigned char SCH_Delete_Task(uint32_t TASK_ID){
	uint8_t i = 0;
	while(i < Task_cnt && SCH_tasks_G[i].TaskID != TASK_ID ){
		++i;
	}
	if(i >= Task_cnt) return 1;

	--Task_cnt;
	if(i < Task_cnt) SCH_tasks_G[i+1].Delay += SCH_tasks_G[i].Delay;
	while(i < Task_cnt){
		SCH_tasks_G[i] = SCH_tasks_G[i+1];
		++i;
	}
	SCH_tasks_G[Task_cnt].pTask = 0;
	SCH_tasks_G[Task_cnt].Delay = 0;
	SCH_tasks_G[Task_cnt].Period = 0;
	SCH_tasks_G[Task_cnt].RunMe = 0;
	SCH_tasks_G[Task_cnt].TaskID = 0;
	return 0;
}

void SCH_Dispatch_Tasks(void){
	if(Task_cnt <= 0) return ;
	int num_of_task = 0;
	while(Task_cnt > 0 && SCH_tasks_G[0].Delay <= 0){

		sTask temp = SCH_tasks_G[0];
		++temp.RunMe;
		if(temp.Period){
			temp.Delay = temp.Period;

		uint8_t i = 1;
		while((i<Task_cnt) && (temp.Delay > SCH_tasks_G[i].Delay ||
				(temp.Delay == SCH_tasks_G[i].Delay && temp.Period <= SCH_tasks_G[i].Period)) ){
				temp.Delay -= SCH_tasks_G[i].Delay;
				SCH_tasks_G[i-1] = SCH_tasks_G[i];
				++i;
			}
		SCH_tasks_G[i-1] = temp;
		if(i < Task_cnt){
				SCH_tasks_G[i].Delay -= SCH_tasks_G[i-1].Delay;
			}
		}
		else{SCH_Delete_Task(SCH_tasks_G[0].TaskID);}
		task_arr[num_of_task] = temp.pTask;
		++num_of_task;
	}
	for(int i = 0;i < num_of_task;++i){
		if (task_arr[i] != 0)
		(*task_arr[i])();
		task_arr[i] = 0;
	}



}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {  if(htim->Instance == TIM2){
	 SCH_Update();
  }
 }




