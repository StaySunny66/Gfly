#ifndef __MS5611_H
#define __MS5611_H


#include "stdint.h"
#include "sys.h"


/*函数声明----------------------------------------------------------------*/
void MS561101BA_Reset(void);
void MS561101BA_readPROM(void);
uint32_t MS561101BA_DO_CONVERSION(u8 command);
void MS561101BA_GetTemperature(u8 OSR_Temp);
void MS561101BA_GetPressure(u8 OSR_Pres);
void MS561101BA_Init(void);
void SampleANDExchange(void);
void caculate_High(int N);
float get_High();
float get_High_real();

//IO方向设置
 

#define SDA_IN1()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}	//PB7输入模式
#define SDA_OUT1() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;} //PB7输出模式


//IO操作函数	 
#define IIC_SCL1    PBout(6) //SCL
#define IIC_SDA1    PBout(7) //SDA	 
#define READ_SDA1   PBin(7)  //输入SDA 



#define MS561101BA_ADDR   0x77   //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)
// 定义MS561101BA内部地址
// registers of the device
#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
#define MS561101BA_RESET 0x1E
// D1 and D2 result size (bytes)
#define MS561101BA_D1D2_SIZE 3
// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08
//#define  MS561101BA_D1_OSR_256 0x40 
//#define  MS561101BA_D1_OSR_512 0x42 
//#define  MS561101BA_D1_OSR_1024 0x44 
//#define  MS561101BA_D1_OSR_2048 0x46 
#define  MS561101BA_D1_OSR_4096 0x48 

//#define  MS561101BA_D2_OSR_256 0x50 
//#define  MS561101BA_D2_OSR_512 0x52 
//#define  MS561101BA_D2_OSR_1024 0x54 
//#define  MS561101BA_D2_OSR_2048 0x56 
#define  MS561101BA_D2_OSR_4096 0x58 

#define MS561101BA_PROM_BASE_ADDR 0xA0 // by adding ints from 0 to 6 we can read all the prom configuration values. 
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry.


#endif

 




