#include "SeniorTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "iic.h"
#include "myiic.h"
#include "mpu6050.h"
#include "ms5611.h"
#include <semphr.h>



/// ������ȡ��Щ����
extern float current_yaw   ;   // ƫ����
extern float current_pitch ;   // ������
extern float current_roll  ;   // ������

extern short gyrox;
extern short gyroy;
extern short gyroz;

extern float Now_High;




static TaskHandle_t Mpu6050Task_Handle = NULL;
static TaskHandle_t MS5611Task_Handle = NULL;
static TaskHandle_t InitTask_Handle = NULL;

SemaphoreHandle_t xMutex ;

static void MS5611Task(void* parameter){
	

	
	for(;;){
		
		if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
           
			
			   Now_High = get_High();
         xSemaphoreGive(xMutex); // �ͷ��ź���
      }
		vTaskDelay(100);  // 500 ms ����һ�θ߶�

	}
	
}


static void Mpu6050Task(void* parameter){
	
	for(;;){
		
		
		if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
           
				Read_DMP(&current_pitch,&current_roll,&current_yaw);      // ��ȡ�����Ƕ�
        MPU_Get_Gyroscope(&gyroy,&gyrox,&gyroz);                  // ��ȡ�������ٶ�
        xSemaphoreGive(xMutex);                                   // �ͷ��ź���
			
     }

		 
		   vTaskDelay(8);  //  1 tick  = 1ms 
		 
	}
}







static void Start_Senior_Task(void* parameter){
	
     xMutex = xSemaphoreCreateMutex();
	   IIC_Init();
			
		 MS561101BA_Init();
		 caculate_High(10);
			
		 MPU6050_initialize();     //=====MPU6050��ʼ��	
		 DMP_Init();

		
		 xTaskCreate((TaskFunction_t )Mpu6050Task,  /* ������ں��� */
								(const char*    )"Mpu6050Task", /* �������� */
								(uint16_t       )512,           /* ����ջ��С */
								(void*          )NULL,          /* ������ں������� */
								(UBaseType_t    )1,             /* ��������ȼ� */
								(TaskHandle_t*  )&Mpu6050Task_Handle);/* ������ƿ�ָ�� */
		 xTaskCreate((TaskFunction_t )MS5611Task,  /* ������ں��� */
								(const char*    )"MS5611Task", /* �������� */
								(uint16_t       )512,           /* ����ջ��С */
								(void*          )NULL,          /* ������ں������� */
								(UBaseType_t    )1,             /* ��������ȼ� */
								(TaskHandle_t*  )&MS5611Task_Handle);/* ������ƿ�ָ�� */
								
		vTaskDelete(NULL); // ɾ����ǰ����	
		return ;
}





void initSeniorTask(){
	

		 xTaskCreate((TaskFunction_t )Start_Senior_Task,  /* ������ں��� */
								(const char*    )"InitTask_Handle", /* �������� */
								(uint16_t       )512,           /* ����ջ��С */
								(void*          )NULL,          /* ������ں������� */
								(UBaseType_t    )0,             /* ��������ȼ� */
								(TaskHandle_t*  )&InitTask_Handle);/* ������ƿ�ָ�� */
								
								
     return ;

}


