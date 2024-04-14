#include "MainFlyTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "gfly.h"
#include "stdio.h"

// ������Ϣ����

QueueHandle_t CmdQueue;


static TaskHandle_t GflyTask_Handle = NULL;
static TaskHandle_t Fly_PidTask_Handle = NULL;

static void GflyTask(void* parameter);
static void Fly_PidTask(void* parameter);


// **************************************** //
// Gfly �ɿ���Ҫ����
// code ������
// Date 2024.4.3
// **************************************** // 


// PID����
struct Pid {

		float Kp;
		float Ki;
		float Kd;
};


// ����������
void start_Main_Fly_Task(){
	
	CmdQueue = xQueueCreate(10, sizeof(uint8_t)*7);
	
	
	xTaskCreate((TaskFunction_t )GflyTask,  /* ������ں��� */
						 (const char*    )"GflyTask", /* �������� */
						 (uint16_t       )512,           /* ����ջ��С */
						 (void*          )NULL,          /* ������ں������� */
						 (UBaseType_t    )1,             /* ��������ȼ� */
						 (TaskHandle_t*  )&GflyTask_Handle);/* ������ƿ�ָ�� */
						 
						 
	xTaskCreate((TaskFunction_t )Fly_PidTask,  /* ������ں��� */
						 (const char*    )"Fly_PidTask", /* �������� */
						 (uint16_t       )512,           /* ����ջ��С */
						 (void*          )NULL,          /* ������ں������� */
						 (UBaseType_t    )1,             /* ��������ȼ� */
						 (TaskHandle_t*  )&Fly_PidTask_Handle);/* ������ƿ�ָ�� */


	
	


			return;
}



// Gfly ���߳�
// ���� �߳�

// 

static void GflyTask(void* parameter){
	uint8_t cmd[7];
	for(;;){
		if (xQueueReceive(CmdQueue, &cmd, portMAX_DELAY) == pdTRUE) {
           cmdPrash(cmd);  
					
     }		
	}
}





// PID ���߳�
// �����ʵ�����˻��ķɿ�������
static void Fly_PidTask(void* parameter){

	for(;;){
		
		
		
	  // �������ȡ��ȡ����������
		
		//���� Gfly ���д���
		//double_PID();
		

    vTaskDelay(5);   /* ��ʱ500��tick */

	}
}






