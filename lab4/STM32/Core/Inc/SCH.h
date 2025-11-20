#ifndef _SCH_H_
#define _SCH_H_
#include "main.h"
#include "global.h"


extern void SCH_Init(void);
extern void SCH_Update(void);
extern void SCH_Dispatch_Tasks(void);
extern unsigned char SCH_Add_Task(void (* pFunction)(void), uint32_t DELAY, uint32_t PERIOD);
extern unsigned char SCH_Delete_Task(uint32_t TASK_ID);

#endif
