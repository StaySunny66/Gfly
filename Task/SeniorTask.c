#include "SeniorTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "iic.h"
#include "myiic.h"
#include "mpu6050.h"
#include "ms5611.h"




static TaskHandle_t Mpu6050Task_Handle = NULL;
static TaskHandle_t MS5611Task_Handle = NULL;


static void MS5611Task(void* parameter){
	
		MS561101BA_Init();
	
	// 获取基准高度
	
	caculate_High(10);
	
	for(;;){
		
		printf("fly high = %f \r\n",get_High());
		
		
		vTaskDelay(100);  // 500 ms 计算一次高度

	}
	
}


static void Mpu6050Task(void* parameter){
	
	IIC_Init();
	//MPU6050_initialize();     //=====MPU6050初始化	
	//DMP_Init();

	/// 角速度
	short gyrox,gyroy,gyroz;
		
	//角度
	float current_yaw   =  0;   // 偏航角
	float current_pitch =  0;   // 俯仰角
	float current_roll  =  0;   // 翻滚角

	for(;;){
		
		//	Read_DMP(&current_pitch,&current_roll,&current_yaw);      // 获取三个角度
    //	MPU_Get_Gyroscope(&gyroy,&gyrox,&gyroz);                  // 获取三个角速度
		//	printf("P : %.2f  R :%.2f  Y :%.2f  GX: %d GY: %d GZ: %d \r\n ",
				//		current_pitch,current_roll,current_yaw,gyrox,gyroy,gyroz);
		
		  vTaskDelay(8);  //  1 tick  = 1ms 
	}
}




void Start_Senior_Task(){
	

	
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
				
		 return ;
}





