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


//三九的梅花红了满山的雪
//萧条枝影
//月牙照人眠
//小伙赶着马车手里攥着长鞭
//江风吹过
//他通红的脸
//锣鼓声声正月正
//爆竹声里落尽一地红
//家家户户都点上花灯
//又是一年好收成

//                   - 毛不易《东北民谣》


//  2024 4.3  完成FreeRTOS 移植

//  原本想着直接用ucos的示例省点事  奈何 ucosii 不支持时间片调度算法  
//  且调度算法存在问题  故 移植freeRTOS 官方提供了移植文件  不用写汇编 移植顺利
//   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 根据FreeRTOS 设置位4 位抢占优先级  
//   中断中 使用os API 函数  中断优先级要低于 配置文件中的 优先级

// 

//  PCB 电路 串口 映射   U6 —> DEBUG_U   U2—> UART   U1—> Lora

//



// 2024 4.14 串口重定向




static void AppTaskCreate(void);/* AppTask任务 */
static void AppTask(void* parameter);
 /* 创建任务句柄 */
static TaskHandle_t AppTask_Handle = NULL;



int main(void)
{
	
	BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4

	USART1_init(9600);  
	USART2_init(19200);    //初始化串口2   光流计接口
	USART6_init(230400);  
  motorInit();
	
	 xReturn = xTaskCreate((TaskFunction_t )AppTask,  /* 任务入口函数 */
                        (const char*    )"AppTask",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )1, /* 任务的优先级 */
                        (TaskHandle_t*  )&AppTask_Handle);/* 任务控制块指针 */ 
												
												
	 Start_Usart_Task();
	 initSeniorTask();
	 start_Main_Fly_Task();
												
	
												
												
												
												
												
  
    vTaskStartScheduler();   /* 启动任务，开启调度 */
	  while(1);   /* 正常不会执行到这里 */    

}


static void AppTask(void* parameter)
{	
	
	  Led_init();
    while (1)
    {
        Led_Set(SYS_LED,LED_OFF);
        vTaskDelay(500);   /* 延时500个tick */
        Led_Set(SYS_LED,LED_ON);
        vTaskDelay(500);   /* 延时500个tick */		 
       // printf("hi");			
    }
}




