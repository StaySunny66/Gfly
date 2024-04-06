#include "ms5611.h"
#include "stm32f4xx.h"
#include "stdio.h"
#include "iic.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"


/*��������----------------------------------------------------------------*/
uint16_t Cal_C[8];  //���ڴ��PROM�е�6������	
uint32_t D1_Pres,D2_Temp; // �������ѹ�����¶�
float Pressure = 0;				//�¶Ȳ�������ѹ
float dT,Temperature2;//ʵ�ʺͲο��¶�֮��Ĳ���,ʵ���¶�,�м�ֵ

float Temperature = 0;
double OFF,SENS;  //ʵ���¶ȵ���,ʵ���¶�������
float Aux,OFF2,SENS2;  //�¶�У��ֵ

float zero_high;  //����߶�

uint32_t ex_Pressure;			//���ڶ���ת��ֵ
uint8_t  exchange_num[8];



#define ALPHA 0.4 // �˲���ϵ������Χ��0��1֮��

float lowPassFilter(float input, float prevOutput) {
    float output = ALPHA * input + (1 - ALPHA) * prevOutput;
    return output;
}






/// gxyos ����
/// iic дһ�� ָ�� ���ڸ�λ

// ms5611 ����ָ��ģʽ 
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
����* ������:MS561101BA_Reset   
����* ���� : ��λ  
����* ����  :��   
����* ���  :��    
����*/ 
void MS561101BA_Reset(void)
{
	iic_send_cmd(MS561101BA_ADDR,MS561101BA_RESET);
  
}



/************************************************************   
* ������:MS561101BA_readPROM   
* ���� : ��PROM��ȡ����У׼����
* ����  :��   
* ���  :��    */ 


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





// ����߶ȵĺ���
double calculateAltitude(double pressure,float temperature) {
    const double sea_level_pressure = 1013.25; // ��׼��ƽ����ѹ��mbar��
    const double gravity = 9.80665; // �������ٶȣ�m/s^2��
    const double gas_constant = 287.053; // �ɿ��������峣����J/(kg��K)��

    double altitude = (1 - pow(pressure / sea_level_pressure, 1 / 5.255)) * (288.15 / 0.0065);
    return altitude;
}



float calculateAltitude1(float pressure, float temperature) {
    float seaLevelPressure = 1013.25; // ��ƽ���׼����ѹ������λΪ mbar
    float altitude;
    
    // ���㺣�θ߶�
    altitude = 44330 * (1 - pow((pressure / seaLevelPressure), 0.1903));
    
    // �����¶ȶԺ��ε�Ӱ�죨����һ���򻯵�ģ�ͣ�
    altitude = altitude + (temperature - 15); // ����ÿ���϶ȵ��¶ȱ仯��Ӧ10�׵ĺ��α仯
    
    return altitude;
}

// MS5611 ��ȡPROM ����

// �ϸ���˵����ĸ�ʽ����


void MS561101BA_readPROM1(void)
{   
	
	
	
	
	 unsigned char T[2];  // ��ʱ����
	
	
	
	 for(int  i = 0 ;i<=7;i++){
	 
	 
					// IIC ��ʼ�ź�
				
				 IIC_Start();
				
					// д��ַ 
				 IIC_Send_Byte(MS561101BA_ADDR<<1);  //����д��ַ
		     
		 
		     if (!IIC_Wait_Ack()) {    // �ȴ�IIC ��Ӧ
						IIC_Stop();
						printf("1 ERR MS5611 not have a ack \r\n");
						return ;
			    }
				  // д���ȡ��ָ�� �˴�Ϊ��ַ
					
		     IIC_Send_Byte(MS561101BA_PROM_BASE_ADDR|(i<<1));  //����д��ַ
	 
					
					if (!IIC_Wait_Ack()) {    // �ȴ�IIC ��Ӧ
						IIC_Stop();
						printf("2 ERR MS5611 not have a ack \r\n");
						return ;
			    }
					
										
					
				 IIC_Start(); //  ��ʼ 
				
					// д��ַ 
				 IIC_Send_Byte(MS561101BA_ADDR<<1|1);  //���͵�ַ   ��
					
				 if (!IIC_Wait_Ack()) {    // �ȴ�IIC ��Ӧ
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
* ������:MS561101BA_DO_CONVERSION   
* ���� :  
* ����  :��   
* ���  :��    
*/
uint32_t MS561101BA_DO_CONVERSION(uint8_t command)
{
	    //printf("cmd = %x",command);
	    unsigned char t[3];
	    uint32_t data; 
	    char len ;
	    len = 3;
	    /// ����ת������
	    iic_send_cmd(MS561101BA_ADDR,command);
	    /// ��ʱ10 ms ���ת��
	    //delay_ms(10);
			vTaskDelay(10); 
			/// ���Ͷ�ȡ����
			iic_send_cmd(MS561101BA_ADDR,0x00);
			
			//��ʼ��ȡ
      IIC_Start();	  // iic��ȡ�ź�
			
			IIC_Send_Byte(MS561101BA_ADDR<<1|1);  // send read IIC address  ���� IIC ��ȡ��ַ 
	
			if (!IIC_Wait_Ack()) {    // �ȴ�IIC ��Ӧ
						IIC_Stop();
						printf("ERR MS5611 not have a ack \r\n");

						return 1;
			 }
			
			 //  iic ��Ӧ���  ��ʼ��ȡ 3 ���ֽڵ�����
			 
			 t[0]  = IIC_Read_Byte(1);
			 t[1]  = IIC_Read_Byte(1);
			 t[2]  = IIC_Read_Byte(0);
			 
			 data = t[0]<<16|t[1]<<8|t[2];
			 
       //printf(" data = %x \r\n",data);
		   return data;
	
			
}

/************************************************************   
* ������:MS561101BA_GetTemperature   
* ���� : ��ȡ�����¶�
* ����  :��������   
* ���  :��    
*/
void MS561101BA_GetTemperature(u8 OSR_Temp)
{
   
			D2_Temp= MS561101BA_DO_CONVERSION(OSR_Temp);	
			//delay_ms(100);

			dT=D2_Temp - (((uint32_t)Cal_C[5])<<8);
	
	
			Temperature=2000+dT*((uint32_t)Cal_C[6])/8388608;	//����¶�ֵ��100����2001��ʾ20.01��
	    //printf("Temperature = %lf \r\n ",Temperature);
	
//	
//	     if(Temperature==0){   // ��� ��һ�� �����Ľ��һ��
//			
//			   Temperature=2000+dT*((uint32_t)Cal_C[6])/8388608;
//			}


//      Temperature = lowPassFilter(2000+dT*((uint32_t)Cal_C[6])/8388608,Temperature) ;


}


/************************************************************   
* ������:MS561101BA_GetPressure   
* ���� : ��ȡ������ѹ
* ����  :��������   
* ���  :��    
*/
void MS561101BA_GetPressure(u8 OSR_Pres)
{
	
		D1_Pres= MS561101BA_DO_CONVERSION(OSR_Pres);

		//delay_ms(100); 
			
		OFF=(uint32_t)(Cal_C[2]<<16)+((uint32_t)Cal_C[4]*dT)/128.0;
		SENS=(uint32_t)(Cal_C[1]<<15)+((uint32_t)Cal_C[3]*dT)/256.0;
	//�¶Ȳ���
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

//      if(Pressure==0){   // ��� ��һ�� �����Ľ��һ��
//			
//			   Pressure = (D1_Pres*SENS/2097152.0-OFF)/32768.0;
//			}


//      Pressure = lowPassFilter((D1_Pres*SENS/2097152.0-OFF)/32768.0,Pressure) ;

}

/************************************************************   
* ������:MS561101BA_Init   
* ���� : MS561101BA��ʼ��
* ����  :��   
* ���  :��    
*/ 
void MS561101BA_Init(void)
{
			MS561101BA_Reset();
			vTaskDelay(100);
			MS561101BA_readPROM();
			vTaskDelay(100);
} 

/************************************************************   
* ������:SampleANDExchange   
* ���� : ��ȡ���ݲ�ת�����ڷ���
* ����  :��   
* ���  :��    
*/ 
void SampleANDExchange(void) 
{
			uint8_t i=0;
			MS561101BA_GetTemperature(MS561101BA_D2_OSR_4096);//0x58
			MS561101BA_GetPressure(MS561101BA_D1_OSR_4096);		//0x48
	
	    printf("pressure = %f \r\n",Pressure/50);
	    printf("T : %4.3f��C\r\n ",Temperature/100);
	
	    printf("High = %f m",calculateAltitude(Pressure/50,26));
	
  	
}


// �������� ��ȡ���溣�θ߶�

// ʵ�ַ�ʽ  ������ʼ��ʱ����  ȡ ����n��


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


// ��ȡ�������ĸ߶�
float get_High(){
	
	
	   MS561101BA_GetTemperature(MS561101BA_D2_OSR_4096);//0x58
		 MS561101BA_GetPressure(MS561101BA_D1_OSR_4096);//0x48
	
	   return  calculateAltitude(Pressure/50,26) - zero_high;

}


// ��ȡ�������ĸ߶�
float get_High_real(){
	
	
	   MS561101BA_GetTemperature(MS561101BA_D2_OSR_4096);//0x58
		 MS561101BA_GetPressure(MS561101BA_D1_OSR_4096);//0x48
	
	   return  calculateAltitude(Pressure/50,26);

}