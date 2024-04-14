#include "SeniorTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "iic.h"
#include "myiic.h"
#include "mpu6050.h"
#include "ms5611.h"
#include <semphr.h>



static TaskHandle_t Mpu6050Task_Handle = NULL;
static TaskHandle_t MS5611Task_Handle = NULL;
static TaskHandle_t InitTask_Handle = NULL;

SemaphoreHandle_t xMutex ;

static void MS5611Task(void* parameter){
	

	
	for(;;){
		
		if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
           
			
			   printf("fly high = %f \r\n",get_High());
         xSemaphoreGive(xMutex); // 释放信号量
      }
		vTaskDelay(100);  // 500 ms 计算一次高度

	}
	
}


static void Mpu6050Task(void* parameter){
	
	
	
	

	/// 角速度
	short gyrox,gyroy,gyroz;
		
	//角度
	float current_yaw   =  0;   // 偏航角
	float current_pitch =  0;   // 俯仰角
	float current_roll  =  0;   // 翻滚角

	for(;;){
		
		if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
           
				Read_DMP(&current_pitch,&current_roll,&current_yaw);      // 获取三个角度
        MPU_Get_Gyroscope(&gyroy,&gyrox,&gyroz);                  // 获取三个角速度
        xSemaphoreGive(xMutex);                                   // 释放信号量
			
     }
		printf("P : %.2f  R :%.2f  Y :%.2f  GX: %d GY: %d GZ: %d \r\n ",
						current_pitch,current_roll,current_yaw,gyrox,gyroy,gyroz);
		vTaskDelay(8);  //  1 tick  = 1ms 
	}
}







static void Start_Senior_Task(void* parameter){
	
	   printf("hi");
	
     xMutex = xSemaphoreCreateMutex();
	   IIC_Init();
			
		 MS561101BA_Init();
		 caculate_High(10);
			
		 MPU6050_initialize();     //=====MPU6050初始化	
		 DMP_Init();

		
		 xTaskCreate((TaskFunction_t )Mpu6050Task,  /* 任务入口函数 */
								(const char*    )"Mpu6050Task", /* 任务名字 */
								(uint16_t       )512,           /* 任务栈大小 */
								(void*          )NULL,          /* 任务入口函数参数 */
								(UBaseType_t    )1,             /* 任务的优先级 */
								(TaskHandle_t*  )&Mpu6050Task_Handle);/* 任务控制块指针 */
		 xTaskCreate((TaskFunction_t )MS5611Task,  /* 任务入口函数 */
								(const char*    )"MS5611Task", /* 任务名字 */
								(uint16_t       )512,           /* 任务栈大小 */
								(void*          )NULL,          /* 任务入口函数参数 */
								(UBaseType_t    )1,             /* 任务的优先级 */
								(TaskHandle_t*  )&MS5611Task_Handle);/* 任务控制块指针 */
								
		vTaskDelete(NULL); // 删除当前任务	
		return ;
}





void initSeniorTask(){
	
	   printf("initSenior");

		 xTaskCreate((TaskFunction_t )Start_Senior_Task,  /* 任务入口函数 */
								(const char*    )"InitTask_Handle", /* 任务名字 */
								(uint16_t       )512,           /* 任务栈大小 */
								(void*          )NULL,          /* 任务入口函数参数 */
								(UBaseType_t    )0,             /* 任务的优先级 */
								(TaskHandle_t*  )&InitTask_Handle);/* 任务控制块指针 */
    printf("initSeniorFinish");
     return ;

}


