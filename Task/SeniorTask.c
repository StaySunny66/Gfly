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
         xSemaphoreGive(xMutex); // �ͷ��ź���
      }
		vTaskDelay(100);  // 500 ms ����һ�θ߶�

	}
	
}


static void Mpu6050Task(void* parameter){
	
	
	
	

	/// ���ٶ�
	short gyrox,gyroy,gyroz;
		
	//�Ƕ�
	float current_yaw   =  0;   // ƫ����
	float current_pitch =  0;   // ������
	float current_roll  =  0;   // ������

	for(;;){
		
		if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
           
				Read_DMP(&current_pitch,&current_roll,&current_yaw);      // ��ȡ�����Ƕ�
        MPU_Get_Gyroscope(&gyroy,&gyrox,&gyroz);                  // ��ȡ�������ٶ�
        xSemaphoreGive(xMutex);                                   // �ͷ��ź���
			
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
	
	   printf("initSenior");

		 xTaskCreate((TaskFunction_t )Start_Senior_Task,  /* ������ں��� */
								(const char*    )"InitTask_Handle", /* �������� */
								(uint16_t       )512,           /* ����ջ��С */
								(void*          )NULL,          /* ������ں������� */
								(UBaseType_t    )0,             /* ��������ȼ� */
								(TaskHandle_t*  )&InitTask_Handle);/* ������ƿ�ָ�� */
    printf("initSeniorFinish");
     return ;

}


