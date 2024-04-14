#include "delay.h"
#include "usart.h"
#include "stm32f4xx.h"
#include "motor.h"
#include "myiic.h"
#include "mpu6050.h"
#include "Gfly.h"
#include "MS5611.h"
#include "FreeRTOS.h"
#include "task.h"
#include "UsartTask.h"
#include "SeniorTask.h"
#include "led.h"
#include "MainFlyTask.h"


//���ŵ�÷��������ɽ��ѩ
//����֦Ӱ
//����������
//С�����������߬�ų���
//���紵��
//��ͨ�����
//�������������
//���������価һ�غ�
//�Ҽһ��������ϻ���
//����һ����ճ�

//                   - ë���ס�������ҥ��


//  2024 4.3  ���FreeRTOS ��ֲ

//  ԭ������ֱ����ucos��ʾ��ʡ����  �κ� ucosii ��֧��ʱ��Ƭ�����㷨  
//  �ҵ����㷨��������  �� ��ֲfreeRTOS �ٷ��ṩ����ֲ�ļ�  ����д��� ��ֲ˳��
//   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); ����FreeRTOS ����λ4 λ��ռ���ȼ�  
//   �ж��� ʹ��os API ����  �ж����ȼ�Ҫ���� �����ļ��е� ���ȼ�

// 

//  PCB ��· ���� ӳ��   U6 ��> DEBUG_U   U2��> UART   U1��> Lora

//



// 2024 4.14 �����ض���




static void AppTaskCreate(void);/* AppTask���� */
static void AppTask(void* parameter);
 /* ���������� */
static TaskHandle_t AppTask_Handle = NULL;



int main(void)
{
	
	BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4

	USART1_init(9600);  
	USART2_init(19200);    //��ʼ������2   �����ƽӿ�
	USART6_init(230400);  
  motorInit();
	
	 xReturn = xTaskCreate((TaskFunction_t )AppTask,  /* ������ں��� */
                        (const char*    )"AppTask",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )1, /* ��������ȼ� */
                        (TaskHandle_t*  )&AppTask_Handle);/* ������ƿ�ָ�� */ 
												
												
	 Start_Usart_Task();
	 initSeniorTask();
	 start_Main_Fly_Task();
												
	
												
												
												
												
												
  
    vTaskStartScheduler();   /* �������񣬿������� */
	  while(1);   /* ��������ִ�е����� */    

}


static void AppTask(void* parameter)
{	
	
	  Led_init();
    while (1)
    {
        Led_Set(SYS_LED,LED_OFF);
        vTaskDelay(500);   /* ��ʱ500��tick */
        Led_Set(SYS_LED,LED_ON);
        vTaskDelay(500);   /* ��ʱ500��tick */		 
       // printf("hi");			
    }
}




