#include "led.h"
#include "stm32f4xx.h"


// 高旭阳 2024 板载LED 灯初始化  初始化后LED为关闭状态


void Led_init(){
	
  GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOA时钟
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOC10与GPIOC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PC10，PC11
	
	GPIO_SetBits(GPIOC,  GPIO_Pin_11);
	GPIO_SetBits(GPIOC,  GPIO_Pin_10);
	
  //GPIO_ResetBits(GPIOC, GPIO_Pin_11);
  //GPIO_ResetBits(GPIOC, GPIO_Pin_10);

}

// led 状态设置   LED 编号  LED 状态 
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
	
	
