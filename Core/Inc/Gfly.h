#ifndef _GFLY_H_
#define _GFLY_H_


////// �ɿ�����λ
// ����ָ��
#define LOCKED 0x00 // ����ָ��
#define UNLOCK 0x01 // ����ָ��


#define GFLY_ENABLE 0X01
#define GFLY_DISABLE 0X00


#define SETTING  0XA0  //GFLY ����ָ��

#define FLY_LOCK  0x00  //�ɻ�����
#define FLY_UNLOCK   0x01 //�ɻ�����

#define PID_ENABLE  0x02//PIDʹ��
#define PID_DISABLE  0x03 //PIDʧ��
#define PID_I_CLEAN  0x04 //��������
#define GFLY_RECK_DATA  0x05 //��ȡ��������
#define PID_RESET  0x06 //PID�ָ�Ĭ��
#define GFLY_OPEN_LIGHT 0x07 //�����ƹ�




#define VERSION_BIG 1
#define VERSION_SMALL 0
#define OS_SUPPORT 0x00  // 0x00 NO OS  0X01 FreeRTOS  0x01 uCOS

#define CMD_START 0XFF
#define BASEDATA 0X00 
#define THROTTLEDETAIL 0X10  
#define PRESSDATA 0X20
#define TEMPERATURE 0X30

#define PID_SET 0x40 
////////////////PID DATA BIT //////////////
#define PID_SET_P 0x04
#define PID_SET_I 0x02
#define PID_SET_D 0x01
#define PID_SET_INNER 0x00  // �ڻ�
#define PID_SET_OUTER 0x10  // �⻷


#define PID_SET_YAW   0x00    // PID ƫ���ǲ���
#define PID_SET_PITCH   0x20  // PID �����ǲ���
#define PID_SET_ROLL   0x40   // PID �����ǲ���
#define PID_SET_HIGH   0x60   // PID �߶Ȳ���

#define PID_FLAG_POS 0x08
#define PID_FLAG_NEG 0x00
////////////////////////////////////////



#define FLYACK 0x50
#define VERSION 0x60
#define HIGHTDEFFULE 0X70

#define GPS_J  0X80
#define GPS_W  0X90

#define FLY_SETTING  0XA0
#define GPS_FLAG_POS 0x20 // +
#define GPS_FLAG_NEG 0x00 // -





#define GFLY_LOG printf


// Gfly ����
void Gfly_Tick(void);


/// GFLY Ӳ����ʼ��
void  Gfly_HardWare_Init(void);

/// Gfly �����ʼ��
void Gfly_SoftWare_Init(void);
/// GFLY  
/// Gfly �ɿ�������
void Gfly_Main_Thread(void);
/// 
void Gfly_Event_Thread(void);


// PID ״̬����
void sendPid(void);

void double_PID();


// �ɰ� PID ���㺯��
void  PIDcHAT();

// ��ʼ������
void gflyInit();

// ����ʱ���ź�
void Tick_Set();

// żУ�麯��
unsigned char evenParity(unsigned char byte);

// ����żУ�麯��
unsigned char dataEvenParity(unsigned char *packet);

// �������ݰ�
void sendPacket();

// ���ͻ�������
void basicDataSend(unsigned char throttle, float pitch, float roll, float yaw);

// ������������
void basicDataPrash(unsigned char packet[]);

// ������������
void throttleDataSend(unsigned char throttle1, unsigned char throttle2, unsigned char throttle3, unsigned char throttle4);

// ������������
void throttleDataPrash(unsigned char packet[]);

// ������ѹ����
void pressDataSend(float pressData);

// ������ѹ����
void pressDataPrash(unsigned char packet[]);

// �����¶�����
void temperatureDataSend(float temp1, float temp2, float temp3, float temp4);

// �����¶�����
void temperatureDataPrash(unsigned char packet[]);

// ���� PID ����
void pidSendData(unsigned char P_I_D_opt, unsigned char Witch_opt, float val);

// ���� PID ����
void pidSendPrash(unsigned char packet[]);

// ������Ӧ
void sendAck(unsigned char ackCmd,unsigned char data1,unsigned char data2,unsigned char data3);

// ������Ӧ
void AckPrash(unsigned char packet[]);

// ���Ͱ汾��Ϣ
void sendVersionData();

// �����汾��Ϣ
void versionPrash(unsigned char packet[]);

// ���� GPS ����
void sendGpsData(unsigned char GPS_opt, unsigned char direction, float gpsData);

// ����ָ��
void cmdPrash(unsigned char recPacket[]);     

void cmdPrash(unsigned char packet[]);
int getSendCount(void);
int getRecCount(void);




#endif

