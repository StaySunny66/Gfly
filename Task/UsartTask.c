#include "UsartTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "stdio.h"
#include "stm32f4xx.h"
#include "led.h"


// ���ڶ���
QueueHandle_t Usart1RxQueue;
QueueHandle_t Usart2RxQueue;
QueueHandle_t Usart6RxQueue;



extern QueueHandle_t CmdQueue;


extern int flow_x;
extern int flow_y;	



static TaskHandle_t Usart1Task_Handle = NULL;
static TaskHandle_t Usart2Task_Handle = NULL;
static TaskHandle_t Usart6Task_Handle = NULL;


static void Usart1RecTask(void* parameter);
static void Usart2RecTask(void* parameter);
static void Usart6RecTask(void* parameter);


void Start_Usart_Task(){
	

	/// ��ʼ������
	Usart1RxQueue = xQueueCreate(50, sizeof(uint8_t));
	Usart2RxQueue = xQueueCreate(50, sizeof(uint8_t));
	Usart6RxQueue = xQueueCreate(50, sizeof(uint8_t));

	xTaskCreate((TaskFunction_t )Usart1RecTask,  /* ������ں��� */
						 (const char*    )"Usart1RecTask", /* �������� */
						 (uint16_t       )512,           /* ����ջ��С */
						 (void*          )NULL,          /* ������ں������� */
						 (UBaseType_t    )1,             /* ��������ȼ� */
						 (TaskHandle_t*  )&Usart1Task_Handle);/* ������ƿ�ָ�� */
						 
						 
	xTaskCreate((TaskFunction_t )Usart2RecTask,  /* ������ں��� */
						 (const char*    )"Usart2RecTask", /* �������� */
						 (uint16_t       )512,           /* ����ջ��С */
						 (void*          )NULL,          /* ������ں������� */
						 (UBaseType_t    )1,             /* ��������ȼ� */
						 (TaskHandle_t*  )&Usart2Task_Handle);/* ������ƿ�ָ�� */
						 
						 
	xTaskCreate((TaskFunction_t )Usart6RecTask,  /* ������ں��� */
						 (const char*    )"Usart6RecTask", /* �������� */
						 (uint16_t       )512,           /* ����ջ��С */
						 (void*          )NULL,          /* ������ں������� */
						 (UBaseType_t    )1,             /* ��������ȼ� */
						 (TaskHandle_t*  )&Usart6Task_Handle);/* ������ƿ�ָ�� */

}



static void Usart1RecTask(void* parameter){
  uint8_t T;
	
	unsigned char  C_T[7];
	int cmd_index = -1;

	for(;;){
					
			if (xQueueReceive(Usart1RxQueue, &T, portMAX_DELAY) == pdTRUE) {
                // ������յ�������
				
			if (T == 0xff) {
				
          //��ȡ�� ��ʼ����������ʼλ
          if (cmd_index == -1){
								cmd_index = 0;
						    Led_Set(ACK_LED,LED_ON);
					
					} 
          else {
                 cmd_index = 0;
							   printf("Basic Rec Err/r/n");
						    Led_Set(ACK_LED,LED_OFF);
           }
					
        } else {
            //��ȡ�� ���� 0XFF  �ǿ��ܾ�������� ~
            if(cmd_index != -1) C_T[cmd_index++] = T;
        }

        if (cmd_index == 5) {
            cmd_index = -1;
					   //printf("call");
					  //cmdPrash(C_T);
					
					Led_Set(ACK_LED,LED_OFF);
					
					xQueueSend(CmdQueue, &C_T, 1);
					portYIELD();
					
					  

        }
                	
       }	 
	}
}
static void Usart2RecTask(void* parameter){
	
	unsigned char Rt;
	unsigned char RtFLAG = 0;  // �������ڼ�λ��
	unsigned char data[4];
	unsigned char CH;
	for(;;){
		
		Start:
		if (xQueueReceive(Usart2RxQueue, &Rt, portMAX_DELAY) == pdTRUE) {
	
		  // printf("RT = %x  flag %d\r\n",Rt,RtFLAG);
		
		   if(RtFLAG==0){
			   if(Rt==0xFE) RtFLAG = 1;
				 goto Start;
			 
			 }
			 
			 if(RtFLAG==1){
			   if(Rt==0x04) RtFLAG = 2;
			   goto Start;
			 }
			 
			 if(RtFLAG==2){
			      data[0] = Rt;
				    RtFLAG ++;
				 goto Start;
			 
			 }
			 if(RtFLAG==3){
			      data[1] = Rt;
				    RtFLAG ++;
				 goto Start;
			 
			 }
			 if(RtFLAG==4){
			      data[2] = Rt;
				    RtFLAG ++;
				 goto Start;
			 
			 }
			 if(RtFLAG==5){
			      data[3] = Rt;
				    RtFLAG ++;
				 goto Start;
			 
			 }
			 
			 if(RtFLAG==6){  // ׼����ʼУ������
				 
         RtFLAG ++;
				 goto Start;
			 
			 }
			 
			 
			 if(RtFLAG==7){
			 
			    if(Rt<=30){
					  // RtFLAG = 0;
					   goto Start;
					}
					RtFLAG++;
					goto Start;
					
			  }
			 
				if(RtFLAG==8){
					
				   if(Rt==0xAA){
							 RtFLAG =0 ;
							// printf("X:%dY:%d\r\n",(short)(data[1]<<8|data[0]),(short)(data[3]<<8|data[2]));
					    flow_x = (short)(data[1]<<8|data[0]);
						  flow_y = (short)(data[3]<<8|data[2]);
					 } 
						 RtFLAG =0 ;

				}
		
     }
				
	}
}
static void Usart6RecTask(void* parameter){
	uint8_t receivedData;
	for(;;){
		if (xQueueReceive(Usart6RxQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
                
              // ���������һ��
					    printf("%c",receivedData);
					 
					
    }
			
	}
}




void USART6_IRQHandler(void){
	
	
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
	{
		
		uint8_t data = USART_ReceiveData(USART6);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(Usart6RxQueue, &data, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		
	}
}

//  Lora �ӿ�  ����LORA �ӿ������5v ���� �ʵ���ʱ�޷�ʹ��  
void USART1_IRQHandler(void){
	
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		
		uint8_t data = USART_ReceiveData(USART1);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(Usart6RxQueue, &data, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		
	}
}

// U2 ���� �������ݽ���
void USART2_IRQHandler(void){
	
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		
		 
		
		uint8_t data = USART_ReceiveData(USART2);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(Usart2RxQueue, &data, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		
	}
}
