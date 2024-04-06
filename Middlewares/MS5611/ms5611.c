#include "ms5611.h"
#include "stm32f4xx.h"
#include "stdio.h"
#include "iic.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"


/*变量声明----------------------------------------------------------------*/
uint16_t Cal_C[8];  //用于存放PROM中的6组数据	
uint32_t D1_Pres,D2_Temp; // 存放数字压力和温度
float Pressure = 0;				//温度补偿大气压
float dT,Temperature2;//实际和参考温度之间的差异,实际温度,中间值

float Temperature = 0;
double OFF,SENS;  //实际温度抵消,实际温度灵敏度
float Aux,OFF2,SENS2;  //温度校验值

float zero_high;  //地面高度

uint32_t ex_Pressure;			//串口读数转换值
uint8_t  exchange_num[8];



#define ALPHA 0.4 // 滤波器系数，范围在0到1之间

float lowPassFilter(float input, float prevOutput) {
    float output = ALPHA * input + (1 - ALPHA) * prevOutput;
    return output;
}






/// gxyos 适配
/// iic 写一个 指令 用于复位

// ms5611 发送指令模式 
unsigned char iic_send_cmd(unsigned char Addr,unsigned char cmd){
	IIC_Start();	
	
	IIC_Send_Byte(Addr << 1 );
	
	if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
		printf("\r\n  ERR --> Reset The MS5611 not have a ack /r/n");
   }
	
	 
	IIC_Send_Byte(cmd);
	 
	if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
   }

	return 0;


}




/************************************************************   
　　* 函数名:MS561101BA_Reset   
　　* 描述 : 复位  
　　* 输入  :无   
　　* 输出  :无    
　　*/ 
void MS561101BA_Reset(void)
{
	iic_send_cmd(MS561101BA_ADDR,MS561101BA_RESET);
  
}



/************************************************************   
* 函数名:MS561101BA_readPROM   
* 描述 : 从PROM读取出厂校准数据
* 输入  :无   
* 输出  :无    */ 


void MS561101BA_readPROM(void)
{   
		 
		u8 tt[2];
								
	  for (u8 i=0;i<=MS561101BA_PROM_REG_COUNT;i++) 
	 {
     
			 IICreadBytes(MS561101BA_ADDR<<1,MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE),2,tt);
		 	 printf("\n The MS561101BA is reading PROM :  %d %d\r\n",tt[0],tt[1]);
			 Cal_C[i] = tt[0]<<8|tt[1];
		 
		}
	   printf("\r\n  PROM   Val : \r\n");
     printf("\r\nC1 = %d\r\nC2 = %d\r\nC3 = %d\r\nC4 = %d\r\nC5 = %d\r\nC6 = %d\r\n"
		,Cal_C[1],Cal_C[2],Cal_C[3],Cal_C[4],Cal_C[5],Cal_C[6]);  
}





// 计算高度的函数
double calculateAltitude(double pressure,float temperature) {
    const double sea_level_pressure = 1013.25; // 标准海平面气压（mbar）
    const double gravity = 9.80665; // 重力加速度（m/s^2）
    const double gas_constant = 287.053; // 干空气的气体常数（J/(kg·K)）

    double altitude = (1 - pow(pressure / sea_level_pressure, 1 / 5.255)) * (288.15 / 0.0065);
    return altitude;
}



float calculateAltitude1(float pressure, float temperature) {
    float seaLevelPressure = 1013.25; // 海平面标准大气压力，单位为 mbar
    float altitude;
    
    // 计算海拔高度
    altitude = 44330 * (1 - pow((pressure / seaLevelPressure), 0.1903));
    
    // 考虑温度对海拔的影响（这是一个简化的模型）
    altitude = altitude + (temperature - 15); // 假设每摄氏度的温度变化对应10米的海拔变化
    
    return altitude;
}

// MS5611 读取PROM 数据

// 严格按照说明书的格式进行


void MS561101BA_readPROM1(void)
{   
	
	
	
	
	 unsigned char T[2];  // 临时变量
	
	
	
	 for(int  i = 0 ;i<=7;i++){
	 
	 
					// IIC 开始信号
				
				 IIC_Start();
				
					// 写地址 
				 IIC_Send_Byte(MS561101BA_ADDR<<1);  //发送写地址
		     
		 
		     if (!IIC_Wait_Ack()) {    // 等待IIC 响应
						IIC_Stop();
						printf("1 ERR MS5611 not have a ack \r\n");
						return ;
			    }
				  // 写入读取的指令 此处为地址
					
		     IIC_Send_Byte(MS561101BA_PROM_BASE_ADDR|(i<<1));  //发送写地址
	 
					
					if (!IIC_Wait_Ack()) {    // 等待IIC 响应
						IIC_Stop();
						printf("2 ERR MS5611 not have a ack \r\n");
						return ;
			    }
					
										
					
				 IIC_Start(); //  开始 
				
					// 写地址 
				 IIC_Send_Byte(MS561101BA_ADDR<<1|1);  //发送地址   读
					
				 if (!IIC_Wait_Ack()) {    // 等待IIC 响应
						IIC_Stop();
						printf("3 ERR MS5611 not have a ack \r\n");
						return ;
			    }
				 
					T[0] = IIC_Read_Byte(1);
					
					
					T[1] = IIC_Read_Byte(0);
					
	        Cal_C[i] = T[0]<<8|T[1];
					
				  printf("\n The MS561101BA is reading PROM :  %d %d\r\n",	T[0],T[1]);

	    
	 
	 
	 }
	

	
	
	
  
	   printf("\r\n  PROM   Val : \r\n");
     printf("\r\nC1 = %d\r\nC2 = %d\r\nC3 = %d\r\nC4 = %d\r\nC5 = %d\r\nC6 = %d\r\n"
		,Cal_C[1],Cal_C[2],Cal_C[3],Cal_C[4],Cal_C[5],Cal_C[6]);  
}



/************************************************************   
* 函数名:MS561101BA_DO_CONVERSION   
* 描述 :  
* 输入  :无   
* 输出  :无    
*/
uint32_t MS561101BA_DO_CONVERSION(uint8_t command)
{
	    //printf("cmd = %x",command);
	    unsigned char t[3];
	    uint32_t data; 
	    char len ;
	    len = 3;
	    /// 发送转换命令
	    iic_send_cmd(MS561101BA_ADDR,command);
	    /// 延时10 ms 完成转换
	    //delay_ms(10);
			vTaskDelay(10); 
			/// 发送读取命令
			iic_send_cmd(MS561101BA_ADDR,0x00);
			
			//开始读取
      IIC_Start();	  // iic读取信号
			
			IIC_Send_Byte(MS561101BA_ADDR<<1|1);  // send read IIC address  发送 IIC 读取地址 
	
			if (!IIC_Wait_Ack()) {    // 等待IIC 响应
						IIC_Stop();
						printf("ERR MS5611 not have a ack \r\n");

						return 1;
			 }
			
			 //  iic 响应完成  开始读取 3 个字节的数据
			 
			 t[0]  = IIC_Read_Byte(1);
			 t[1]  = IIC_Read_Byte(1);
			 t[2]  = IIC_Read_Byte(0);
			 
			 data = t[0]<<16|t[1]<<8|t[2];
			 
       //printf(" data = %x \r\n",data);
		   return data;
	
			
}

/************************************************************   
* 函数名:MS561101BA_GetTemperature   
* 描述 : 读取数字温度
* 输入  :过采样率   
* 输出  :无    
*/
void MS561101BA_GetTemperature(u8 OSR_Temp)
{
   
			D2_Temp= MS561101BA_DO_CONVERSION(OSR_Temp);	
			//delay_ms(100);

			dT=D2_Temp - (((uint32_t)Cal_C[5])<<8);
	
	
			Temperature=2000+dT*((uint32_t)Cal_C[6])/8388608;	//算出温度值的100倍，2001表示20.01°
	    //printf("Temperature = %lf \r\n ",Temperature);
	
//	
//	     if(Temperature==0){   // 如果 第一次 则计算的结果一样
//			
//			   Temperature=2000+dT*((uint32_t)Cal_C[6])/8388608;
//			}


//      Temperature = lowPassFilter(2000+dT*((uint32_t)Cal_C[6])/8388608,Temperature) ;


}


/************************************************************   
* 函数名:MS561101BA_GetPressure   
* 描述 : 读取数字气压
* 输入  :过采样率   
* 输出  :无    
*/
void MS561101BA_GetPressure(u8 OSR_Pres)
{
	
		D1_Pres= MS561101BA_DO_CONVERSION(OSR_Pres);

		//delay_ms(100); 
			
		OFF=(uint32_t)(Cal_C[2]<<16)+((uint32_t)Cal_C[4]*dT)/128.0;
		SENS=(uint32_t)(Cal_C[1]<<15)+((uint32_t)Cal_C[3]*dT)/256.0;
	//温度补偿
if(Temperature < 2000)// second order temperature compensation when under 20 degrees C
{
			Temperature2 = (dT*dT) / 0x80000000;
			Aux = (Temperature-2000)*(Temperature-2000);
			OFF2 = 2.5*Aux;
			SENS2 = 1.25*Aux;
if(Temperature < -1500)
	{
			Aux = (Temperature+1500)*(Temperature+1500);
			OFF2 = OFF2 + 7*Aux;
			SENS2 = SENS + 5.5*Aux;
	}
}else  //(Temperature > 2000)
{
			Temperature2 = 0;
			OFF2 = 0;
			SENS2 = 0;
}
	
			Temperature = Temperature - Temperature2;
			OFF = OFF - OFF2;
			SENS = SENS - SENS2;	

		Pressure=(D1_Pres*SENS/2097152.0-OFF)/32768.0;

//      if(Pressure==0){   // 如果 第一次 则计算的结果一样
//			
//			   Pressure = (D1_Pres*SENS/2097152.0-OFF)/32768.0;
//			}


//      Pressure = lowPassFilter((D1_Pres*SENS/2097152.0-OFF)/32768.0,Pressure) ;

}

/************************************************************   
* 函数名:MS561101BA_Init   
* 描述 : MS561101BA初始化
* 输入  :无   
* 输出  :无    
*/ 
void MS561101BA_Init(void)
{
			MS561101BA_Reset();
			vTaskDelay(100);
			MS561101BA_readPROM();
			vTaskDelay(100);
} 

/************************************************************   
* 函数名:SampleANDExchange   
* 描述 : 读取数据并转换串口发送
* 输入  :无   
* 输出  :无    
*/ 
void SampleANDExchange(void) 
{
			uint8_t i=0;
			MS561101BA_GetTemperature(MS561101BA_D2_OSR_4096);//0x58
			MS561101BA_GetPressure(MS561101BA_D1_OSR_4096);		//0x48
	
	    printf("pressure = %f \r\n",Pressure/50);
	    printf("T : %4.3f°C\r\n ",Temperature/100);
	
	    printf("High = %f m",calculateAltitude(Pressure/50,26));
	
  	
}


// 函数作用 获取地面海拔高度

// 实现方式  开机初始化时调用  取 采样n次


void caculate_High(int N){
	
	double High_sum = 0;
	for(int i = 0; i < N; i++){
		 MS561101BA_GetTemperature(MS561101BA_D2_OSR_4096);//0x58
		 MS561101BA_GetPressure(MS561101BA_D1_OSR_4096);		//0x48
	   High_sum += calculateAltitude(Pressure/50,26);
		 vTaskDelay(100);

	}
	zero_high = High_sum / (float)N;
	
	printf("Inf : -> get the zero High %f \r\n ",zero_high);
	

}


// 获取距离地面的高度
float get_High(){
	
	
	   MS561101BA_GetTemperature(MS561101BA_D2_OSR_4096);//0x58
		 MS561101BA_GetPressure(MS561101BA_D1_OSR_4096);//0x48
	
	   return  calculateAltitude(Pressure/50,26) - zero_high;

}


// 获取距离地面的高度
float get_High_real(){
	
	
	   MS561101BA_GetTemperature(MS561101BA_D2_OSR_4096);//0x58
		 MS561101BA_GetPressure(MS561101BA_D1_OSR_4096);//0x48
	
	   return  calculateAltitude(Pressure/50,26);

}