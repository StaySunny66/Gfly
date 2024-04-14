#include "MainFlyTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "gfly.h"
#include "stdio.h"

// 控制消息队列

QueueHandle_t CmdQueue;


static TaskHandle_t GflyTask_Handle = NULL;
static TaskHandle_t Fly_PidTask_Handle = NULL;

static void GflyTask(void* parameter);
static void Fly_PidTask(void* parameter);


// **************************************** //
// Gfly 飞控主要代码
// code 高旭阳
// Date 2024.4.3
// **************************************** // 


// PID参数
struct Pid {

		float Kp;
		float Ki;
		float Kd;
};


// 启动任务函数
void start_Main_Fly_Task(){
	
	CmdQueue = xQueueCreate(10, sizeof(uint8_t)*7);
	
	
	xTaskCreate((TaskFunction_t )GflyTask,  /* 任务入口函数 */
						 (const char*    )"GflyTask", /* 任务名字 */
						 (uint16_t       )512,           /* 任务栈大小 */
						 (void*          )NULL,          /* 任务入口函数参数 */
						 (UBaseType_t    )1,             /* 任务的优先级 */
						 (TaskHandle_t*  )&GflyTask_Handle);/* 任务控制块指针 */
						 
						 
	xTaskCreate((TaskFunction_t )Fly_PidTask,  /* 任务入口函数 */
						 (const char*    )"Fly_PidTask", /* 任务名字 */
						 (uint16_t       )512,           /* 任务栈大小 */
						 (void*          )NULL,          /* 任务入口函数参数 */
						 (UBaseType_t    )1,             /* 任务的优先级 */
						 (TaskHandle_t*  )&Fly_PidTask_Handle);/* 任务控制块指针 */


	
	


			return;
}



// Gfly 主线程
// 控制 线程

// 

static void GflyTask(void* parameter){
	uint8_t cmd[7];
	for(;;){
		if (xQueueReceive(CmdQueue, &cmd, portMAX_DELAY) == pdTRUE) {
           cmdPrash(cmd);  
					
     }		
	}
}





// PID 主线程
// 在这里，实现无人机的飞控主代码
static void Fly_PidTask(void* parameter){

	for(;;){
		
		
		
	  // 在这里读取获取传感器数据
		
		//导入 Gfly 飞行代码
		//double_PID();
		

    vTaskDelay(5);   /* 延时500个tick */

	}
}






