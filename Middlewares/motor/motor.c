#include "motor.h"
// P A 6 7 PB 01    TIM3 C1 C2 C3 C4    

/// PA 6 TM3 CH1  // P_CH1
/// PA 7 TM3 CH2  // P_CH2
/// PB 0 TM3 CH3  // P_CH3
/// PB 1 TM3 CH4  // P_CH4



void init(){
	
	  ///  ��ʼ������ 
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // ʹ�� TIM3 �� GPIO ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
	
			// �������Ÿ��ù���
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);


    
	 // ���� TIM3_CH1��TIM3_CH2��TIM3_CH3��TIM3_CH4 ��Ӧ������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		


	
    // ���� TIM3 Ϊ PWM ģʽ
    TIM_TimeBaseStructure.TIM_Prescaler = 83;  //1000000  20ms 1ms - 2ms 
    TIM_TimeBaseStructure.TIM_Period = 19999;    // 100
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // ���� TIM3_CH1��TIM3_CH2��TIM3_CH3��TIM3_CH4 Ϊ PWM ģʽ
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;  // ��ʼռ�ձ�Ϊ0%
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);

    // ʹ�� TIM3_CH1��TIM3_CH2��TIM3_CH3��TIM3_CH4 ����Ƚ�ͨ��
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
		
	  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPEʹ�� 


    // ʹ�� TIM3
    TIM_Cmd(TIM3, ENABLE);

}





void  motorInit(void){
	
	init();

}


int value_Fotmate(int value){

      //if( value >254 ) value = 180;
	    if( value < 0  ) value = 0;
	 
	  return value;
}




//  ռ�ձ� ��Χ   0.65  -   0.72


//  7000    ��Ч��Χ   4550  -  5040  

// 0 - 7000
// 4550 - 5040 
// ���� ��Χ value  0 - 254;
void  setMotor(u8 Value1,u8 Value2,u8 Value3,u8 Value4){
	    static unsigned char tem1,tem2,tem3,tem4; 
	    // ֱ�� return;
	    if(!(tem1 ==  Value1 && tem2 ==  Value2 &&tem3 ==  Value3 &&tem4 ==  Value4 )){
				
				tem1 =  Value1 ; 
				tem2 =  Value2 ;
				tem3 =  Value3 ;
				tem4 =  Value4 ;
				printf("-> LOG Motor No1= %d No2= %d No3= %d No4= %d  \r",Value1,Value2,Value3,Value4);
			
			}
			float v1 = 1000.0f * ((float)(value_Fotmate(Value1) /254.0f)); 
			float v2 = 1000.0f * ((float)(value_Fotmate(Value2) /254.0f)); 
			float v3 = 1000.0f * ((float)(value_Fotmate(Value3) /254.0f)); 
			float v4 = 1000.0f * ((float)(value_Fotmate(Value4) /254.0f)); 
		
			TIM_SetCompare1(TIM3, (int)v1+13150);   
			TIM_SetCompare2(TIM3, (int)v2+13150);
			TIM_SetCompare3(TIM3, (int)v3+13150);	
			TIM_SetCompare4(TIM3, (int)v4+13150);
			
		//  printf("-> LOG Motor v = %d  \r\n ",(int)v4+6500);

			
			return ;
	    

}


//    �߾�������
void setMotor_H_def(int Value1,int Value2,int Value3,int Value4){
	    static unsigned int t1,t2,t3,t4; 
	    // ֱ�� return;
	    if(!(t1 ==  Value1 && t2 ==  Value2 &&t3 ==  Value3 &&t4 ==  Value4 )){
				
				t1 =  Value1 ; 
				t2 =  Value2 ;
				t3 =  Value3 ;
				t4 =  Value4 ;
				printf("-> LOG High Def Mo No1= %d No2= %d No3= %d No4= %d  \r",Value1,Value2,Value3,Value4);
			
			}
		
			TIM_SetCompare1(TIM3, t1+13100);   
			TIM_SetCompare2(TIM3, t2+13100);
			TIM_SetCompare3(TIM3, t3+13100);	
			TIM_SetCompare4(TIM3, t4+13100);

			return ;
	    

}


