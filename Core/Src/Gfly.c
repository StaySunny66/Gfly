#include "Gfly.h"
#include "stdint.h"
#include "stm32f4xx.h"
#include <stdlib.h>
#include <stdio.h>
#include "delay.h"
#include "usart.h"
#include "motor.h"
#include "myiic.h"
#include "mpu6050.h"
#include <math.h>

// Gfly �����ļ�  

#define SysLed_Err 1;
#define SysLed_Normal 2;
#define SysLed_Lock 6;

/// ��������� 

//  ����  ����  ָ��λ��1 ��ʼ 

#define TXCMD 0x01
#define TXDATA1 0x02
#define TXDATA2 0x03
#define TXDATA3 0x04
#define TXDATA4 0x05

//  ����  ����  ָ�� λ�� 0 ��ʼ 
#define RECCMD 0x00
#define RECDATA1 0x01
#define RECDATA2 0x02
#define RECDATA3 0x03
#define RECDATA4 0x04
// �����¼�ִ�����ڵ�λ MS 
#define CallMainThreadTime 1
#define CallEventThreadTime 600
// ������� ����
#define MAX_THROTTLE 180
// ������� ���� �߾�������
#define MAX_THROTTLE_H_DEF 900

// ������ֵ  ����
#define MAX_I 10.0f

// ��С���ֿ�ʼ����
#define MIN_I_EN 25






// ����ʱ�߶ȼ�¼
float Basic_High = 0.0f;

// ��ֱ�ٶ�
float v_speed = 0.0f;


void sendPid();
void show_PID();
void Gfly_show();
void double_PID();
void Signal_PID();


// PID ����
unsigned int PID_SEND = 0;
// ���а�ȫ��
unsigned char FLY_BEGIN = 0;
unsigned char PID_EN = ENABLE;

unsigned int ErrCount;
unsigned int SendCount;
unsigned int RecCount;

unsigned char  a ;
 
 
 
 
/// <summary>
///  ������һ�η������� ��ͬ�����·� ���ٴ���
///  ������
/// </summary>
int   throttleValue = 0;    // ����
float pitchValue = 0;     // ����
float rollValue = 0;      // ����
float yawValue = 0;       // ƫ��


		
		
/// ���������� ����///////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
///// ȫ�ֱ����������ļ���ʹ�õ��� ///////////////////////////////////////////////
		
//�Ƕ�
float current_yaw   =  0;   // ƫ����
float current_pitch =  0;   // ������
float current_roll  =  0;   // ������

/// ���ٶ�
short gyrox,gyroy,gyroz;

// ����ԭʼ����
int flow_x;
int flow_y;	
//�¶�  Ԥ��
float temperature1 = 0.0f; 
float temperature2;  
float temperature3;     
float temperature4; 
		
// ��ѹ
float pressure = 0.0f;

// ���θ߶�
float Now_High = 0.0f;

		
		
/// �����Ҫ����̬
float yaw_C   =  0;    // ƫ����
float pitch_C =  0;    // ������
float roll_C  =  0;    // ������		
int throttle_C = 50;  // ��������
		
		
/// PID
int m1;
int m2;
int m3;
int m4;


/// �ڻ� PID ����
float PID_INNER_YAW_KP  = -0.01f ;
float PID_INNER_YAW_KI = 0.0f ;
float PID_INNER_YAW_KD = 0.0f ;

float PID_INNER_PITCH_KP = -0.10f ;
float PID_INNER_PITCH_KI = 0.0f ;
float PID_INNER_PITCH_KD =-0.03f ;

float PID_INNER_ROLL_KP = 0.10f ;
float PID_INNER_ROLL_KI = 0.0f ;
float PID_INNER_ROLL_KD = 0.03f ;

float PID_INNER_HIGH_KP = 0.0f ;
float PID_INNER_HIGH_KI = 0.0f ;
float PID_INNER_HIGH_KD = 0.0f ;

/// �⻷ PID ����
float PID_OUTER_YAW_KP = -5.0f ;
float PID_OUTER_YAW_KI = 0.0f ;
float PID_OUTER_YAW_KD = 0.1f ;
float PID_OUTER_PITCH_KP = -6.5f ;
float PID_OUTER_PITCH_KI = 0.0f ;
float PID_OUTER_PITCH_KD = 0.002f ;
float PID_OUTER_ROLL_KP = 7.5f ;
float PID_OUTER_ROLL_KI = 0.0f ;
float PID_OUTER_ROLL_KD = 0.0f ;
float PID_OUTER_HIGH_KP = 0.0f ;
float PID_OUTER_HIGH_KI = 0.0f ;
float PID_OUTER_HIGH_KD = 0.0f ;

// ��е 0 ��
float ZERO_roll = 3.0f, ZERO_pitch = 5.5f, ZERO_yaw = 0.0f;	


//// �����趨����

//   1       2
//
//
//
//   3       4  
void apply_motor_output(float output_pitch, float output_roll, float output_yaw){
	
			if(FLY_BEGIN == 0){
		 	   setMotor_H_def(0,0,0,0);
         m1 = 0; m2 = 0;	m3 = 0;	m4 = 0;	
         throttle_C	 = 0;			
			   return;
		   }
	
	     m1 = (float)(throttle_C / 255.0f) * 1000 + output_roll + output_pitch - output_yaw  ;
	     m2 = (float)(throttle_C / 255.0f) * 1000 - output_roll + output_pitch + output_yaw  ;
	     m3 = (float)(throttle_C / 255.0f) * 1000 - output_roll - output_pitch - output_yaw  ;
       m4 = (float)(throttle_C / 255.0f) * 1000 + output_roll - output_pitch + output_yaw  ;
		 
		  if( m1<0 ) m1 = 0; if( m1> 1000) m1 = 1000; 
			if( m2<0 ) m2 = 0; if( m2> 1000) m2 = 1000;
			if( m3<0 ) m3 = 0; if( m3> 1000) m3 = 1000;
			if( m4<0 ) m4 = 0; if( m4> 1000) m4 = 1000;	

		  
	    setMotor_H_def(m1,m2,m3,m4);
}





/// Gfly �����ʼ��
void Gfly_SoftWare_Init(){
	
	/// ��ȡFlash����
	
	
	
	/// ��ӡ�汾
	Gfly_show();
	
	//  ��ӡ��ǰ����̨����
	show_PID();

}


 
void Gfly_Event_Thread(){
	printf("P : %.2f  R :%.2f  Y :%.2f  GX: %d GY: %d GZ: %d  High: %f  X:%d Y:%d  \r\n ",
					current_pitch,current_roll,current_yaw,gyrox,gyroy,gyroz,Now_High,flow_x,flow_y);
	
	
		    //printf(" m1 %d m2 %d m3 %d m4 %d \r\n",m1,m2,m3,m4);

	 basicDataSend(0,current_pitch,current_roll,current_yaw);
	 throttleDataSend((m1/1000.0f)*254,(m2/1000.0f)*254,(m3/1000.0f)*254,(m4/1000.0f)*254);
	 sendPid();


}

//  Gfly Safty check function 

void Gfly_Safety_Check(){


	    if(current_roll>65|| current_roll<-65)  {
				    if(FLY_BEGIN == 0) return;
					  FLY_BEGIN = 0;    // ��������
				    throttle_C = 0;   // ���Ź��� 
				    GFLY_LOG("/r/n  The Gfly CORE CLOSED Because of Safety LOCKED ������������ /r/n");
				
		  }
}


void Gfly_show(){
	
	GFLY_LOG(" *********   *********  *         **       **    \r\n");
	GFLY_LOG(" *           *          *          **     **     \r\n");
	GFLY_LOG(" *   ^^_^^   *          *           **   **      \r\n");
	GFLY_LOG(" *           *********  *            ** **       \r\n");
	GFLY_LOG(" *           *          *             ***        \r\n");
	GFLY_LOG(" *     ****  *          *             ***        \r\n");
	GFLY_LOG(" *        *  *          *             ***        \r\n");
	GFLY_LOG(" **********  *          *********     ***        \r\n");
  GFLY_LOG(" ------------------------------------------------\r\n");
	GFLY_LOG(" @StaySunny fly.shilight.cn                      \r\n");
	GFLY_LOG(" Gfly Version: Gfly_V4_OS_V2.0                  \r\n");
	GFLY_LOG(" Build Date  : 2024.4.14    \r\n");
	GFLY_LOG(" FreeRTOS Version  : V202212.01\r\n");



}

// �⻷���� PID ����
//////////////////////////////////////////// PID �������� ////////////////////////////////////////////////
// ���� �⻷����
// ���� �����Ƕ� 
// ��� �⻷�������ֵ(���ٶ�ֵ)
float PID_OUTER_Roll_I = 0 ;  //�������⻷����ֵ  
float roll_outer_cac(float hope_roll_val,float current_roll){

	float return_value;
	float err;
	err = (hope_roll_val + ZERO_roll - current_roll);
	
	// ������Ź�С�����л��ֿ���
	if(throttle_C <= 30){
		PID_OUTER_Roll_I = 0;
	}else{
	  PID_OUTER_Roll_I += err;
	}
	// �������Ʒ���
	if(PID_OUTER_Roll_I >= MAX_I){
		 PID_OUTER_Roll_I = MAX_I;
	}
	// �������Ʒ���
	if(PID_OUTER_Roll_I <= -MAX_I){
		 PID_OUTER_Roll_I = MAX_I;
	}
	return_value = /* P */ PID_OUTER_ROLL_KP * err +   
								 /* I */ PID_OUTER_ROLL_KI * PID_OUTER_Roll_I  +     //����
	               /* D */ PID_OUTER_ROLL_KD * gyroy;    // �⻷ ���ٶ� ��Ϊ΢��ֵ

	// �����ۻ�	
	return return_value;
}


float PID_INNER_Roll_I = 0 ;  //�������ڻ�����ֵ  
float PID_LAST_Gyroy = 0 ;
// �������ڻ����� ���ٶ�
// ���� �⻷���ٶ�(����ֵ)  ʵ�ʽ��ٶ�ֵ
// ��� ����Ƕ���ֵ
int roll_inner_cac(float hope_Gyroy,float current_Gyroy){
   float err; 
	 int return_value;
	err  =  current_Gyroy - hope_Gyroy;
	// ������Ź�С�����л��ֿ���
	if(throttle_C <= 30){
		PID_INNER_Roll_I = 0;
	}else{
	  PID_INNER_Roll_I += err;
	}
	// �������Ʒ���
	if(PID_INNER_Roll_I >= MAX_I){
		 PID_INNER_Roll_I = MAX_I;
	}
	// �������Ʒ���
	if(PID_INNER_Roll_I <= -MAX_I){
		 PID_INNER_Roll_I = MAX_I;
	}
	
	return_value = /* P */ PID_INNER_ROLL_KP * err +   
								 /* I */ PID_INNER_ROLL_KI * PID_INNER_Roll_I  +     //����
	               /* D */ PID_INNER_ROLL_KD * current_Gyroy - PID_LAST_Gyroy ;    // �⻷ ���ٶ� ��Ϊ΢��ֵ
  PID_LAST_Gyroy = current_Gyroy;
	return return_value;
}



//////////////////////////////////////////// PID �������� ////////////////////////////////////////////////
// ���� �⻷����
// ���� �����Ƕ� 
// ��� �⻷�������ֵ(���ٶ�ֵ)
float PID_OUTER_Pitch_I = 0 ;  //�������⻷����ֵ  
float pitch_outer_cac(float hope_pitch_val,float current_pitch){

	float return_value;
	float err;
	err = (hope_pitch_val+ ZERO_pitch - current_pitch);
	
	// ������Ź�С�����л��ֿ���
	if(throttle_C <= 30){
		PID_OUTER_Pitch_I = 0;
	}else{
	  PID_OUTER_Pitch_I += err;
	}
	// �������Ʒ���
	if(PID_OUTER_Pitch_I >= MAX_I){
		 PID_OUTER_Pitch_I = MAX_I;
	}
	// �������Ʒ���
	if(PID_OUTER_Pitch_I <= -MAX_I){
		 PID_OUTER_Pitch_I = MAX_I;
	}
	return_value = /* P */ PID_OUTER_PITCH_KP * err +   
								 /* I */ PID_OUTER_PITCH_KI * PID_OUTER_Pitch_I  +     //����
	               /* D */ PID_OUTER_PITCH_KD * gyrox;    // �⻷ ���ٶ� ��Ϊ΢��ֵ

	// �����ۻ�	
	return return_value;
}


float PID_INNER_Pitch_I = 0 ;  //�������ڻ�����ֵ  
float PID_LAST_Gyrox = 0 ;
// �������ڻ����� ���ٶ�
// ���� �⻷���ٶ�(����ֵ)  ʵ�ʽ��ٶ�ֵ
// ��� ����Ƕ���ֵ
int pitch_inner_cac(float hope_Gyrox,float current_Gyrox){
   float err; 
	 int return_value;
	err  =  current_Gyrox - hope_Gyrox;
	// ������Ź�С�����л��ֿ���
	if(throttle_C <= 30){
		PID_INNER_Pitch_I = 0;
	}else{
	  PID_INNER_Pitch_I += err;
	}
	// �������Ʒ���
	if(PID_INNER_Pitch_I >= MAX_I){
		 PID_INNER_Pitch_I = MAX_I;
	}
	// �������Ʒ���
	if(PID_INNER_Pitch_I <= -MAX_I){
		 PID_INNER_Pitch_I = MAX_I;
	}
	
	return_value = /* P */ PID_INNER_PITCH_KP * err +   
								 /* I */ PID_INNER_PITCH_KI * PID_INNER_Pitch_I  +     //����
	               /* D */ PID_INNER_PITCH_KD * current_Gyrox - PID_LAST_Gyrox ;    // �⻷ ���ٶ� ��Ϊ΢��ֵ
	PID_LAST_Gyrox = current_Gyrox;
	return return_value;
}

//////////////////////////////////////////// PID ƫ���ǿ��� ////////////////////////////////////////////////
// ���� �⻷����
// ���� �����Ƕ� 
// ��� �⻷�������ֵ(���ٶ�ֵ)

float PID_OUTER_Yaw_I = 0 ;  //�������⻷����ֵ  

float yaw_outer_cac(float hope_yaw_val,float current_yaw){

	float return_value;
	float err;
	err = (hope_yaw_val - current_yaw);
	
	// ������Ź�С�����л��ֿ���
	if(throttle_C <= 30){
		PID_OUTER_Yaw_I = 0;
	}else{
	  PID_OUTER_Yaw_I += err;
	}
	// �������Ʒ���
	if(PID_OUTER_Yaw_I >= MAX_I){
		 PID_OUTER_Yaw_I = MAX_I;
	}
	// �������Ʒ���
	if(PID_OUTER_Yaw_I <= -MAX_I){
		 PID_OUTER_Yaw_I = MAX_I;
	}
	return_value = /* P */ PID_OUTER_YAW_KP * err +   
								 /* I */ PID_OUTER_YAW_KI * PID_OUTER_Yaw_I  +     //����
	               /* D */ PID_OUTER_YAW_KD * gyroz;    // �⻷ ���ٶ� ��Ϊ΢��ֵ

	// �����ۻ�	
	return return_value;
}


float PID_INNER_Yaw_I = 0 ;  //�������ڻ�����ֵ  
float PID_LAST_Gyroz = 0 ;
// �������ڻ����� ���ٶ�
// ���� �⻷���ٶ�(����ֵ)  ʵ�ʽ��ٶ�ֵ
// ��� ����Ƕ���ֵ
int yaw_inner_cac(float hope_Gyroz,float current_Gyroz){
   float err; 
	 int return_value;
	err  =  current_Gyroz - hope_Gyroz;
	// ������Ź�С�����л��ֿ���
	if(throttle_C <= 30){
		PID_INNER_Yaw_I = 0;
	}else{
	  PID_INNER_Yaw_I += err;
	}
	// �������Ʒ���
	if(PID_INNER_Yaw_I >= MAX_I){
		 PID_INNER_Yaw_I = MAX_I;
	}
	// �������Ʒ���
	if(PID_INNER_Yaw_I <= -MAX_I){
		 PID_INNER_Yaw_I = MAX_I;
	}
	
	return_value = /* P */ PID_INNER_YAW_KP * err +   
								 /* I */ PID_INNER_YAW_KI * PID_INNER_Yaw_I  +     //����
	               /* D */ PID_INNER_YAW_KD * current_Gyroz - PID_LAST_Gyroz ;    // �⻷ ���ٶ� ��Ϊ΢��ֵ
  PID_LAST_Gyroz = current_Gyroz;
	return return_value;
}






/// PID����
void double_PID(){
	
	// �������
	
	static float last_h = 0.0f;  // ��һ�εĸ߶�ֵ
	
  float Pitch_OUT,Roll_OUT,Yaw_OUT;
	
	
	///// OS ����
	
	
	
	
	// ��ȡ��������ֵ
	unsigned int press; 
	int temp;
	// ��ȡ��ǰ��̬�Ƕ�  
	//Read_DMP(&current_pitch,&current_roll,&current_yaw);      // ��ȡ�����Ƕ�
	//MPU_Get_Gyroscope(&gyroy,&gyrox,&gyroz);                // ��ȡ���ٶ�
	
                                      
	temperature1 = temp/100.0f;   // �¶ȸ�ʽ��
	pressure = press/100.0f;      // ��ѹ��ʽ��
	
	
	//Now_High = get_High(16,pressure);
	
	
	// ����0
	if(last_h!=0){
	    
	  	//  ����ٶ� �ܺ�����������Ӱ��    // 0.001   1ms
		 v_speed  =  (last_h - Now_High) / 0.01 ;
	
	}
	
	last_h = Now_High;
	
	// ��ȫ���   
	Gfly_Safety_Check();
   
	
	
	// ���PID ����Ϊ�Ͽ� PID ����Ϊ 0
	if(PID_EN==DISABLE){
			// Just set the throttle value to the motor
			apply_motor_output(0, 0, 0);
			return;
	}
	// ��������ֵ
	//  Pitch ǰ�� ����
	
	Pitch_OUT = 0;Roll_OUT = 0;Yaw_OUT = 0;
	
	Pitch_OUT = pitch_outer_cac(pitch_C,current_pitch);
	Pitch_OUT = pitch_inner_cac(gyrox,Pitch_OUT);
	
	Roll_OUT = roll_outer_cac(roll_C,current_roll);
	Roll_OUT = roll_inner_cac(gyroy,Roll_OUT);
	
	Yaw_OUT = yaw_outer_cac(yaw_C,current_yaw);
	Yaw_OUT = yaw_inner_cac(gyroz,Yaw_OUT);
	
	 
	
	// Ӧ�õ��
	
	apply_motor_output(Pitch_OUT, Roll_OUT, Yaw_OUT);


	
}




////////    ����PID�㷨
// ������
// ���� �����ĽǶ� ��ǰ�Ƕ�
float pitch_I = 0.0f;
float last_pitch_pid_v = 0.0f;
float pid_signal_pitch(float hope,float now_val){
	float err , out_v;
	err = hope - now_val;
	
	// ������Ź�С�����л��ֿ���
	if(throttle_C <= 30){
		pitch_I = 0;
	}else{
	  pitch_I += err;
	}
	// �������Ʒ���
	if(pitch_I >= MAX_I){
		 pitch_I = MAX_I;
	}
	// �������Ʒ���
	if(pitch_I <= -MAX_I){
		 pitch_I = MAX_I;
	}
	out_v = /* P */ PID_OUTER_PITCH_KP * err +   
					/* I */ PID_OUTER_PITCH_KI * pitch_I  +     //����
	        /* D */ PID_OUTER_PITCH_KD * (now_val - last_pitch_pid_v);    // �⻷ ���ٶ� ��Ϊ΢��ֵ

	last_pitch_pid_v = now_val;

	return out_v;

}

// ������
// ���� �����ĽǶ� ��ǰ�Ƕ�
float roll_I = 0.0f;
float last_roll_pid_v = 0.0f;
float pid_signal_roll(float hope,float now_val){
	float err , out_v;
	err = hope - now_val;
	
	// ������Ź�С�����л��ֿ���
	if(throttle_C <= 30){
		roll_I = 0;
	}else{
	  roll_I += err;
	}
	// �������Ʒ���
	if(roll_I >= MAX_I){
		 roll_I = MAX_I;
	}
	// �������Ʒ���
	if(roll_I <= -MAX_I){
		 roll_I = MAX_I;
	}
	out_v = /* P */ PID_OUTER_ROLL_KP * err +   
					/* I */ PID_OUTER_ROLL_KI * roll_I  +     //����
	        /* D */ PID_OUTER_ROLL_KD * (now_val - last_roll_pid_v);    // �⻷ ���ٶ� ��Ϊ΢��ֵ

	last_roll_pid_v = now_val;

	return out_v;

}


// ƫ����
// ���� �����ĽǶ� ��ǰ�Ƕ�
float yaw_I = 0.0f;
float last_yaw_pid_v = 0.0f;
float pid_signal_yaw(float hope,float now_val){
	float err , out_v;
	err = hope - now_val;
	
	// ������Ź�С�����л��ֿ���
	if(throttle_C <= 30){
		yaw_I = 0;
	}else{
	  yaw_I += err;
	}
	// �������Ʒ���
	if(yaw_I >= MAX_I){
		 yaw_I = MAX_I;
	}
	// �������Ʒ���
	if(yaw_I <= -MAX_I){
		 yaw_I = MAX_I;
	}
	out_v = /* P */ PID_OUTER_YAW_KP * err +   
					/* I */ PID_OUTER_YAW_KI * yaw_I  +     //����
	        /* D */ PID_OUTER_YAW_KD * (now_val - last_yaw_pid_v);    // �⻷ ���ٶ� ��Ϊ΢��ֵ

	last_yaw_pid_v = now_val;

	return out_v;

}



///  ����PID�㷨 

void  Signal_PID(){
        
	
			 Gfly_Safety_Check();
	     float Pitch_OUT,Roll_OUT,Yaw_OUT;

	     // get the senior data;
	     unsigned int press; 
	     int temp;
		   // ��ȡ��ǰ��̬�Ƕ�  
	     Read_DMP(&current_pitch,&current_roll,&current_yaw);
			 //BMP280_ReadPressureTemperature(&press,&temp);            //bmp280��ȡ��ѹֵ���¶�
		   MPU_Get_Gyroscope(&gyroy,&gyrox,&gyroz);                // ��ȡ���ٶ�


	     temperature1 = temp/100.0f;
			 pressure = press/100.0f;
	
	
	      // IF THE PID DISCONNECT TO THE MOTOR
	      if(PID_EN==DISABLE){
				    // Just set the throttle value to the motor
				  apply_motor_output(0, 0, 0);

				    return;
				}
				
				
				Pitch_OUT = 0;Roll_OUT = 0;Yaw_OUT = 0;
	
				Pitch_OUT = pid_signal_pitch(pitch_C,current_pitch);
				
				Roll_OUT = pid_signal_roll(roll_C,current_roll);
				
				Yaw_OUT = pid_signal_yaw(yaw_C,current_yaw);

			apply_motor_output(Pitch_OUT, Roll_OUT, Yaw_OUT);

				
				
       

}




void  PID_I_Clean(){

  // PID ��������
 //error_roll_prev = 0;  integral_roll = 0;
 //error_pitch_prev = 0; integral_pitch = 0;
 //error_yaw_prev = 0;   integral_yaw = 0;


}

/// ��ң���������豸״̬

void  RecData(){

		// ��������汾
    // ���� OS ֧��
	  // ���� PID ����
	  PID_SEND = 3;
	  

}

void show_PID(){


	
    GFLY_LOG("\r\n          PID Table            \r\n");
	  GFLY_LOG("|-------|------------|------------|\r\n");
		GFLY_LOG("|-PITCH-|---INNER----|---OUTER----|\r\n");
		GFLY_LOG("|   Kp  |   %2.5f  |   %2.5f  |\r\n",PID_INNER_PITCH_KP,PID_OUTER_PITCH_KP);
	  GFLY_LOG("|-------|------------|------------|\r\n");
		GFLY_LOG("|   Ki  |   %2.5f  |   %2.5f  |\r\n",PID_INNER_PITCH_KI,PID_OUTER_PITCH_KI);
		GFLY_LOG("|-------|------------|------------|\r\n");
	  GFLY_LOG("|   Kd  |   %2.5f  |   %2.5f  |\r\n",PID_INNER_PITCH_KD,PID_OUTER_PITCH_KD);
		GFLY_LOG("|-------|------------|------------|\r\n");
		GFLY_LOG("|--YAW--|---INNER----|---OUTER----|\r\n");
		GFLY_LOG("|   Kp  |   %2.5f  |   %2.5f  |\r\n",PID_INNER_YAW_KP,PID_OUTER_YAW_KP);
	  GFLY_LOG("|-------|------------|------------|\r\n");
		GFLY_LOG("|   Ki  |   %2.5f  |   %2.5f  |\r\n",PID_INNER_YAW_KI,PID_OUTER_YAW_KI);
		GFLY_LOG("|-------|------------|------------|\r\n");
	  GFLY_LOG("|   Kd  |   %2.5f  |   %2.5f  |\r\n",PID_INNER_YAW_KD,PID_OUTER_YAW_KD);
		GFLY_LOG("|-------|------------|------------|\r\n");
		GFLY_LOG("|-ROLL--|---INNER----|---OUTER----|\r\n");
		GFLY_LOG("|   Kp  |   %2.5f  |   %2.5f  |\r\n",PID_INNER_ROLL_KP,PID_OUTER_ROLL_KP);
	  GFLY_LOG("|-------|------------|------------|\r\n");
		GFLY_LOG("|   Ki  |   %2.5f  |   %2.5f  |\r\n",PID_INNER_ROLL_KI,PID_OUTER_ROLL_KI);
		GFLY_LOG("|-------|------------|------------|\r\n");
	  GFLY_LOG("|   Kd  |   %2.5f  |   %2.5f  |\r\n",PID_INNER_ROLL_KD,PID_OUTER_ROLL_KD);
		GFLY_LOG("|-------|------------|------------|\r\n");
		GFLY_LOG("|-HIGH--|---INNER----|---OUTER----|\r\n");
		GFLY_LOG("|   Kp  |   %2.5f  |   %2.5f  |\r\n",PID_INNER_HIGH_KP,PID_OUTER_HIGH_KP);
	  GFLY_LOG("|-------|------------|------------|\r\n");
		GFLY_LOG("|   Ki  |   %2.5f  |   %2.5f  |\r\n",PID_INNER_HIGH_KI,PID_OUTER_HIGH_KI);
		GFLY_LOG("|-------|------------|------------|\r\n");
	  GFLY_LOG("|   Kd  |   %2.5f  |   %2.5f  |\r\n",PID_INNER_HIGH_KD,PID_OUTER_HIGH_KD);
		GFLY_LOG("|-------|------------|------------|\r\n");
    





}





void sendPid(){
	   
	  // PID����λ
	  
	
	  if(PID_SEND < 3)  return;
	  show_PID();
	  PID_SEND = 0;
	
	  pidSendData(PID_SET_P, PID_SET_INNER | PID_SET_PITCH, PID_INNER_PITCH_KP);
    pidSendData(PID_SET_I, PID_SET_INNER | PID_SET_PITCH, PID_INNER_PITCH_KI);
    pidSendData(PID_SET_D, PID_SET_INNER | PID_SET_PITCH, PID_INNER_PITCH_KD);
	
	  pidSendData(PID_SET_P, PID_SET_INNER | PID_SET_YAW, PID_INNER_YAW_KP);
    pidSendData(PID_SET_I, PID_SET_INNER | PID_SET_YAW, PID_INNER_YAW_KI);
    pidSendData(PID_SET_D, PID_SET_INNER | PID_SET_YAW, PID_INNER_YAW_KD);
	
	  pidSendData(PID_SET_P, PID_SET_INNER | PID_SET_ROLL, PID_INNER_ROLL_KP);
    pidSendData(PID_SET_I, PID_SET_INNER | PID_SET_ROLL, PID_INNER_ROLL_KI);
    pidSendData(PID_SET_D, PID_SET_INNER | PID_SET_ROLL, PID_INNER_ROLL_KD);
	
		pidSendData(PID_SET_P, PID_SET_INNER | PID_SET_HIGH, PID_INNER_HIGH_KP);
    pidSendData(PID_SET_I, PID_SET_INNER | PID_SET_HIGH, PID_INNER_HIGH_KI);
    pidSendData(PID_SET_D, PID_SET_INNER | PID_SET_HIGH, PID_INNER_HIGH_KD);
		
		
		pidSendData(PID_SET_P, PID_SET_OUTER | PID_SET_PITCH, PID_OUTER_PITCH_KP);
    pidSendData(PID_SET_I, PID_SET_OUTER | PID_SET_PITCH, PID_OUTER_PITCH_KI);
    pidSendData(PID_SET_D, PID_SET_OUTER | PID_SET_PITCH, PID_OUTER_PITCH_KD);
	
	  pidSendData(PID_SET_P, PID_SET_OUTER | PID_SET_YAW, PID_OUTER_YAW_KP);
    pidSendData(PID_SET_I, PID_SET_OUTER | PID_SET_YAW, PID_OUTER_YAW_KI);
    pidSendData(PID_SET_D, PID_SET_OUTER | PID_SET_YAW, PID_OUTER_YAW_KD);
	
	  pidSendData(PID_SET_P, PID_SET_OUTER | PID_SET_ROLL, PID_OUTER_ROLL_KP);
    pidSendData(PID_SET_I, PID_SET_OUTER | PID_SET_ROLL, PID_OUTER_ROLL_KI);
    pidSendData(PID_SET_D, PID_SET_OUTER | PID_SET_ROLL, PID_OUTER_ROLL_KD);
	
		pidSendData(PID_SET_P, PID_SET_OUTER | PID_SET_HIGH, PID_OUTER_HIGH_KP);
    pidSendData(PID_SET_I, PID_SET_OUTER | PID_SET_HIGH, PID_OUTER_HIGH_KI);
    pidSendData(PID_SET_D, PID_SET_OUTER | PID_SET_HIGH, PID_OUTER_HIGH_KD);

	
}


/////  ����֧�� ����
/// żУ�� 0 �ĸ��� ����
//  ż 1 �� 0
unsigned char evenParity(unsigned char byte) {

    unsigned char count = 0;
    
    while (byte) {
        count += byte & 1;
        byte >>= 1;
    }

    return (count % 2 == 0);
}


unsigned char dataEvenParity(unsigned char  packet[]){
    int i = 0;
    unsigned char checkData = 0x00;

    for(i = 0 ; i < 3 ; i++ ){
        
        checkData |= evenParity(packet[i+2]);
        checkData <<= 1 ; 

    }
    return checkData;
}


unsigned char dataEvenParityRec(unsigned char  packet[]){
    int i = 0;
    unsigned char checkData = 0x00;

    for(i = 0 ; i < 3 ; i++ ){
        
        checkData |= evenParity(packet[i+1]);
        checkData <<= 1 ; 

    }
    return checkData;
}


/// ���ͺ���

void sendPacket(unsigned char * packet){
	  
	  int i = 0;
	  packet[0] = 0xff;

    
    for(i = 0;i < 6 ; i++) {
			while (!USART_GetFlagStatus(USART2, USART_FLAG_TC));
			USART_SendData(USART2, (uint8_t)packet[i]);
		}
     
    SendCount++;

}





///// ����������İ�װ


///�������ݷ���
void basicDataSend(unsigned char throttle, float pitch, float roll ,float yaw ){
     unsigned char packet[6];
	
	   static unsigned char old_throttle =0 ;
  	 static unsigned  char old_pitch = 0 ;
     static unsigned  char old_roll =  0 ;
	   static unsigned  char old_yaw =   0 ;
	
	   unsigned  char v1 = throttle;
	   unsigned  char v2 = (pitch - (-60.0f)) / 0.5f;
	   unsigned  char v3 = (roll - (-60.0f)) / 0.5f;
	   unsigned  char v4 = (yaw - (-180.0f)) / 2;
	
	   if( old_throttle == v1 && old_pitch == v2 && old_roll == v3 && old_yaw ==v4 ){
		 
		    return;
		    
		 }
		 
		 old_throttle = v1;
		 old_pitch = v2;
		 old_roll =  v3;
		 old_yaw =   v4;


     
     //ָ���װ
     packet[1] = BASEDATA;
    
     //���ݷ�װ
     packet[2] = v1;
     packet[3] = v2;
     packet[4] = v3;
     packet[5] = v4;
   

     ///żУ���װ
     packet[1] |= dataEvenParity(packet);
     
     //����ָ��
     sendPacket(packet);

}

///�������ݽ���
void basicDataPrash(unsigned char packet[]){
     
     throttle_C =   packet[1];
     pitch_C    =  (packet[2] * 0.25f) -30.0f ;
     roll_C     =  (packet[3] * 0.25f) -30.0f ;
     yaw_C        =  (packet[4] * 0.25f) -30.0f ;
	
		printf(">G>>Basic Data  %d %f %f %f  \r\n",throttle_C,pitch_C,roll_C,yaw_C);
	  //setMotor(0,throttle_C);


}


void throttleDataSend(unsigned char throttle1,unsigned char throttle2,unsigned char throttle3,unsigned char throttle4){
     
     unsigned char packet[6];
	
	   static unsigned char v1 = 0,v2 = 0,v3 = 0,v4 = 0;
	   if( throttle1 == v1 && throttle2 == v2 && throttle3 == v3 && throttle4 ==v4 ){
		 
		    //return;
		    
		 }
		 
		v1=throttle1;
		v2=throttle2;
		v3=throttle3;
		v4=throttle4;
		 
		 
	
	
     //ָ���װ
     packet[1] = THROTTLEDETAIL;
    
     //���ݷ�װ
     packet[2] = throttle1;
     packet[3] = throttle2;
     packet[4] = throttle3;
     packet[5] = throttle4;

     ///żУ���װ
     packet[1] |= dataEvenParity(packet);
     
     //����ָ��
     sendPacket(packet);
}


///�������ݽ���
void throttleDataPrash(unsigned char packet[]){
	
	   int throttle1;
     int throttle2;
     int throttle3;
     int throttle4;
     
     throttle1 =  packet[2];
     throttle2 =  packet[3];
     throttle3 =  packet[4];
     throttle4 =  packet[5];
}



void pressDataSend(float pressData){
     unsigned char packet[6];
	   int data;
     unsigned char flag;
     //ָ���װ
     packet[1] = PRESSDATA;
    
     //���ݷ�װ
     data = (int)(pressData*100);
     if( data>=0 )  flag = 0x01;
     else flag = 0x00;


     packet[2] = flag;
     packet[3] = (data/100)/254;  
     packet[4] = (data/100)%254;
     packet[5] =  data%100;

     ///żУ���װ
     packet[1] |= dataEvenParity(packet);
     
     //����ָ��
     sendPacket(packet);

}


///�������ݽ���
void pressDataPrash(unsigned char packet[]){

    float pressData = packet[3] * 254 + packet[4] ;
    pressData += packet[5] / 100.0f;
    if(packet[2] == 0x00) pressData = -pressData;
    // ��ѹת�����
    // ִ����Ĳ���    

     
     
}



void temperatureDataSend(float temp1,float temp2,float temp3,float temp4){
     unsigned char packet[6]; 
     //ָ���װ
     packet[1] = THROTTLEDETAIL;
    
     //���ݷ�װ
     packet[2] =  (temp1 - (-20.0f)) / 0.25f;
     packet[3] =  (temp2 - (-20.0f))/ 0.25f;
     packet[4] =  (temp3 - (-20.0f)) / 0.25f;
     packet[5] =  (temp4 - (-20.0f)) / 0.25f;

     ///żУ���װ
     packet[1] |= dataEvenParity(packet);
     
     //����ָ��
     sendPacket(packet);

}

///�¶����ݽ���
void temperatureDataPrash(unsigned char packet[]){

   
    temperature1 =   packet[2];
    temperature2 =  (packet[3] * 0.25f) -20.0f ;
    temperature3 =  (packet[4] * 0.25f) -20.0f ;
    temperature4 =  (packet[5] * 0.25f) -20.0f ;
    // �¶�ת�����
    // ִ����Ĳ���    

        
}





// PID ���� ��Χ �� 160.00000 
void pidSendData(unsigned char P_I_D_opt,unsigned char Witch_opt,float val){
     //ָ���װ
	   int va;
     unsigned char packet[6];
     packet[1] = PID_SET;
     //���ݷ�װ

     packet[TXDATA1] =  P_I_D_opt | Witch_opt;
     if(val>=0)   packet[TXDATA1] |= PID_FLAG_POS;
	   if (val < 0){
			 
          va = (int)(-val * (100000.0f));
     }
     else {
          va = (int)(val * (100000.0f));
     }
     packet[TXDATA2] =  va / 254 / 254 ;
     packet[TXDATA3] =  va / 254 % 254 ;
     packet[TXDATA4] =  va % 254 ;
     ///żУ���װ
     packet[1] |= dataEvenParity(packet);
     //����ָ��
     sendPacket(packet);
}

// PID ���� PID ���� ��Χ �� 160.00000 -->> �� 16000000
void pidSendPrash(unsigned char packet[]){
     //ָ���װ
     
     float va;
     unsigned char P_I_D_opt;
     unsigned char Witch_opt;
	   unsigned char  Witch_pid;

     unsigned char flag;

     P_I_D_opt = packet[RECDATA1] & 0x07;  /// P I D ?
     Witch_opt = packet[RECDATA1] & 0x10;  /// �� ��  
     flag      = packet[RECDATA1] & 0x08;  /// �� ��
	   Witch_pid = packet[RECDATA1] & 0xE0;  /// �ĸ����ɶȵ�PIDֵ

     va = packet[RECDATA4] + packet[RECDATA3]*254 + packet[RECDATA2]*254*254;
     va = va / 100000.0f;
     if(flag==PID_FLAG_NEG)  va = -va;
	
	
  	GFLY_LOG("Rec ->  PID Setting Message v = %f \r\n",va);
    GFLY_LOG("Rec ->  %X %X %X \r\n",P_I_D_opt,Witch_opt,Witch_pid);

     
  if (Witch_opt == PID_SET_INNER)
  {
		GFLY_LOG("Rec ->  PID inner v = %f \r\n",va);

      switch (P_I_D_opt)
      {
          case PID_SET_P:
              switch (Witch_pid) {

                  case PID_SET_ROLL:

                      PID_INNER_ROLL_KP = va;
                      
                      break;
                  case PID_SET_PITCH:

                      PID_INNER_PITCH_KP = va;

                      break;
                  case PID_SET_YAW:

                      PID_INNER_YAW_KP = va;

                      break;
                  case PID_SET_HIGH:

                      PID_INNER_HIGH_KP = va;


                      break;

              }
              break;
          case PID_SET_I:
              switch (Witch_pid)
              {

                  case PID_SET_ROLL:

                      PID_INNER_ROLL_KI = va;

                      break;
                  case PID_SET_PITCH:

                      PID_INNER_PITCH_KI = va;

                      break;
                  case PID_SET_YAW:

                      PID_INNER_YAW_KI = va;

                      break;
                  case PID_SET_HIGH:

                      PID_INNER_HIGH_KI = va;


                      break;

              }
              break;
          case PID_SET_D:

              switch (Witch_pid)
              {

                  case PID_SET_ROLL:

                      PID_INNER_ROLL_KD = va;

                      break;
                  case PID_SET_PITCH:

                      PID_INNER_PITCH_KD = va;

                      break;
                  case PID_SET_YAW:

                      PID_INNER_YAW_KD = va;

                      break;
                  case PID_SET_HIGH:

                      PID_INNER_HIGH_KD = va;


                      break;

              }


              break;

          default:
              break;
					
      }
			
			


  }
  if (Witch_opt == PID_SET_OUTER)
  {
      switch (P_I_D_opt)
      {
          case PID_SET_P:
              switch (Witch_pid)
              {

                  case PID_SET_ROLL:

                      PID_OUTER_ROLL_KP = va;

                      break;
                  case PID_SET_PITCH:

                      PID_OUTER_PITCH_KP = va;

                      break;
                  case PID_SET_YAW:

                      PID_OUTER_YAW_KP = va;

                      break;
                  case PID_SET_HIGH:

                      PID_OUTER_HIGH_KP = va;


                      break;

              }
              break;
          case PID_SET_I:
              switch (Witch_pid)
              {

                  case PID_SET_ROLL:

                      PID_OUTER_ROLL_KI = va;

                      break;
                  case PID_SET_PITCH:

                      PID_OUTER_PITCH_KI = va;

                      break;
                  case PID_SET_YAW:

                      PID_OUTER_YAW_KI = va;

                      break;
                  case PID_SET_HIGH:

                      PID_OUTER_HIGH_KI = va;


                      break;

              }
              break;
          case PID_SET_D:

              switch (Witch_pid)
              {

                  case PID_SET_ROLL:

                      PID_OUTER_ROLL_KD = va;

                      break;
                  case PID_SET_PITCH:

                      PID_OUTER_PITCH_KD = va;

                      break;
                  case PID_SET_YAW:

                      PID_OUTER_YAW_KD = va;

                      break;
                  case PID_SET_HIGH:

                      PID_OUTER_HIGH_KD = va;


                      break;

              }


              break;

          default:
              break;
      }


  }

     
     
			PID_SEND ++ ;
     /// PID ����ָ��
     
}


/// ������Ӧ 
void sendAck(unsigned char ackCmd,unsigned char data1,unsigned char data2,unsigned char data3){
    unsigned char packet[6];
    packet[1] = FLYACK;
    
    //���ݷ�װ
    packet[2] =  ackCmd;
    packet[3] =  data1;
    packet[4] =  data2; 
    packet[5] =  data3;

    ///żУ���װ
    packet[1] |= dataEvenParity(packet);
     
    //����ָ��
    sendPacket(packet);

}

/// ���յ���Ӧ 
void AckPrash(unsigned char packet[]){
    unsigned char ackCmd;
    ackCmd = packet[2];

}



/// ���ý��� 
void settingDataPrash(unsigned char packet[]){
    
	sendAck(FLY_SETTING,packet[RECDATA1],0,0);

    switch(packet[RECDATA1]){
			
			
    
			case FLY_UNLOCK :
				
				   FLY_BEGIN = 1;
					 GFLY_LOG("Rec - > Gfly Unlocked \r\n");


			break;
			case FLY_LOCK :
				
			     FLY_BEGIN = 0;
			     GFLY_LOG("Rec - > Gfly Locked \r\n");
			
			
			break;
			case PID_ENABLE :
				
			     PID_EN = GFLY_ENABLE;
			     GFLY_LOG("Rec - > PID_ENABLE \r\n");
			
			
			break;
			case PID_DISABLE :
				
			     PID_EN = GFLY_DISABLE;
			     GFLY_LOG("Rec - > PID_DISABLE \r\n");
			
			
			break;
			case PID_I_CLEAN :
				
			     PID_I_Clean();
				
			     FLY_BEGIN = 0;
			     GFLY_LOG("Rec - > PID_I_CLEAN \r\n");
			
			
			break;
			case GFLY_RECK_DATA :
				
			     FLY_BEGIN = 0;
			     GFLY_LOG("Rec - > GFLY_RECK_DATA \r\n");
			
			
			break;
			case PID_RESET :
				
			     FLY_BEGIN = 0;
			     GFLY_LOG("Rec - > Gfly PID_RESET \r\n");
			
			
			break;
			case GFLY_OPEN_LIGHT :
				
			     FLY_BEGIN = 0;
			     GFLY_LOG("Rec - > Gfly GFLY_OPEN_LIGHT \r\n");
			
			
			break;
			
			
		}



}

/// ���Ͱ汾��Ϣ 
void sendVersionData(){
    unsigned char packet[6];
    packet[1] = VERSION;
    
    //���ݷ�װ
    packet[2] =  VERSION_BIG;
    packet[3] =  VERSION_SMALL;
    packet[4] =  OS_SUPPORT;
    packet[5] =  0x00;

    ///żУ���װ
    packet[1] |= dataEvenParity(packet);
    //����ָ��
    sendPacket(packet);
}

/// ���Ͱ汾��Ϣ 
void versionPrash(unsigned char * packet){

    unsigned char versionBig;
    unsigned char versionSmall;
    unsigned char osSupport;
    
    versionBig = packet[2];
    versionSmall = packet[2];
    osSupport = packet[2];
    // ok


}




/// ����GPS��Ϣ
void sendGpsData(unsigned char GPS_opt,unsigned char driection,float gpsData){
    unsigned char packet[6];
    int va = (int)(gpsData*100000.0f);

    packet[1] = GPS_opt;
    //���ݷ�װ
    packet[2] =  va / 254 / 254 / 254 % 32;
    if(va>=0) packet[2] |= GPS_FLAG_POS; 
    packet[3] =  va / 254 / 254 % 254;
    packet[4] =  va / 254 % 254;
    packet[5] =  va % 254;

    ///żУ���װ
    packet[1] |= dataEvenParity(packet);
     
    //����ָ��
    sendPacket(packet);

}

// ����ָ��
void cmdPrash(unsigned char recPacket[]){
     //ָ�����У��
     if((recPacket[0]&0x0F) != dataEvenParityRec(recPacket)){
        ErrCount ++;
			 	GFLY_LOG("GFLY-> Rec ERROR \r\n");

        return ;       
     }
        RecCount ++;
		 
		 GFLY_LOG("GFLY-> Rec ID  %x \r\n",recPacket[0]&0xF0);

     switch (recPacket[0]&0xF0)
     {
        case BASEDATA :
            /* code */
            basicDataPrash(recPacket);
            break;
        case THROTTLEDETAIL :
            throttleDataPrash(recPacket);
            /* code */
            break;
        case PRESSDATA :
            pressDataPrash(recPacket);
            /* code */
            break;
        case TEMPERATURE :
            temperatureDataPrash(recPacket);
            /* code */
            break;
        case PID_SET :
            pidSendPrash(recPacket);
            /* code */
            break;
        case FLYACK :
            AckPrash(recPacket);
            /* code */
            break;
        case VERSION : 
            versionPrash(recPacket);
            /* code */
            break;
        case HIGHTDEFFULE :  
            /* code */
            break;
        case GPS_W :   
            /* code */
            break;
        case GPS_J : 
            /* code */
            break;
				case FLY_SETTING : 
					settingDataPrash(recPacket);
            /* code */
            break;
     }
		 GPIO_SetBits(GPIOC, GPIO_Pin_11); 


}


int getSendCount(){
	return  SendCount;


}


int getRecCount(){
	return  RecCount;

}



void countRest() {

        ErrCount = 0;
        SendCount = 0;
        RecCount = 0;

    }