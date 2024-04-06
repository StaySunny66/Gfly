#include "led.h"
#include "stm32f4xx.h"


// ������ 2024 ����LED �Ƴ�ʼ��  ��ʼ����LEDΪ�ر�״̬


void Led_init(){
	
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOAʱ��
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOC10��GPIOC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PC10��PC11
	
	GPIO_SetBits(GPIOC,  GPIO_Pin_11);
	GPIO_SetBits(GPIOC,  GPIO_Pin_10);
	
  //GPIO_ResetBits(GPIOC, GPIO_Pin_11);
  //GPIO_ResetBits(GPIOC, GPIO_Pin_10);

}

// led ״̬����   LED ���  LED ״̬ 
void Led_Set(unsigned char LED,unsigned char STATE){
	
	switch(LED){
	
		case ACK_LED: 
			if(STATE==LED_ON)
			  GPIO_ResetBits(GPIOC, GPIO_Pin_11);
			else
			  GPIO_SetBits(GPIOC,  GPIO_Pin_11);
		break;
			
			
		case SYS_LED:
			if(STATE==LED_ON)
					GPIO_ResetBits(GPIOC, GPIO_Pin_10);	
			else
					GPIO_SetBits(GPIOC,  GPIO_Pin_10);
		break;
	
	}

}
	
	
