#include "delay.h"



static uint8_t  UsNumber=0;							   
static uint16_t MsNumber=0;							

void delay_Init(unsigned int SYSCLK){
	
	
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks); //获取时钟频率
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	  // 84 / 8 
	UsNumber=clocks.HCLK_Frequency/HSE_VALUE;		 //UsNumber=10;				
	MsNumber=(u16)UsNumber*1000;					
}								    
								    

    								   
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*UsNumber; 						 
	SysTick->VAL=0x00;        					
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	
	SysTick->VAL =0X00;      					
}

void delay_ms(u16 nms)
{
	u32 temp;
	SysTick->LOAD = (u32)nms * MsNumber;
	SysTick->VAL = 0x00;
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
	do
	{
		temp = SysTick->CTRL;
	} while ((temp & 0x01) && !(temp & (1 << 16)));
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
	SysTick->VAL = 0X00;
}
void delay_s(u16 ns)
{
	u16 i;
	while (ns--)
	{
		i = 0;
		while (i++ < 10)
		{
			delay_ms(100);
		}
	}
}
