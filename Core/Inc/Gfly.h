#ifndef _GFLY_H_
#define _GFLY_H_


////// 飞控设置位
// 开关指令
#define LOCKED 0x00 // 锁定指令
#define UNLOCK 0x01 // 解锁指令


#define GFLY_ENABLE 0X01
#define GFLY_DISABLE 0X00


#define SETTING  0XA0  //GFLY 控制指令

#define FLY_LOCK  0x00  //飞机锁定
#define FLY_UNLOCK   0x01 //飞机解锁

#define PID_ENABLE  0x02//PID使能
#define PID_DISABLE  0x03 //PID失能
#define PID_I_CLEAN  0x04 //积分清零
#define GFLY_RECK_DATA  0x05 //获取基本数据
#define PID_RESET  0x06 //PID恢复默认
#define GFLY_OPEN_LIGHT 0x07 //开启灯光




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
#define PID_SET_INNER 0x00  // 内环
#define PID_SET_OUTER 0x10  // 外环


#define PID_SET_YAW   0x00    // PID 偏航角参数
#define PID_SET_PITCH   0x20  // PID 翻滚角参数
#define PID_SET_ROLL   0x40   // PID 翻滚角参数
#define PID_SET_HIGH   0x60   // PID 高度参数

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


// Gfly 心跳
void Gfly_Tick(void);


/// GFLY 硬件初始化
void  Gfly_HardWare_Init(void);

/// Gfly 软件初始化
void Gfly_SoftWare_Init(void);
/// GFLY  
/// Gfly 飞控主任务
void Gfly_Main_Thread(void);
/// 
void Gfly_Event_Thread(void);


// PID 状态发送
void sendPid(void);

void double_PID();


// 旧版 PID 运算函数
void  PIDcHAT();

// 初始化函数
void gflyInit();

// 设置时钟信号
void Tick_Set();

// 偶校验函数
unsigned char evenParity(unsigned char byte);

// 数据偶校验函数
unsigned char dataEvenParity(unsigned char *packet);

// 发送数据包
void sendPacket();

// 发送基本数据
void basicDataSend(unsigned char throttle, float pitch, float roll, float yaw);

// 解析基本数据
void basicDataPrash(unsigned char packet[]);

// 发送油门数据
void throttleDataSend(unsigned char throttle1, unsigned char throttle2, unsigned char throttle3, unsigned char throttle4);

// 解析油门数据
void throttleDataPrash(unsigned char packet[]);

// 发送气压数据
void pressDataSend(float pressData);

// 解析气压数据
void pressDataPrash(unsigned char packet[]);

// 发送温度数据
void temperatureDataSend(float temp1, float temp2, float temp3, float temp4);

// 解析温度数据
void temperatureDataPrash(unsigned char packet[]);

// 发送 PID 数据
void pidSendData(unsigned char P_I_D_opt, unsigned char Witch_opt, float val);

// 解析 PID 数据
void pidSendPrash(unsigned char packet[]);

// 发送响应
void sendAck(unsigned char ackCmd,unsigned char data1,unsigned char data2,unsigned char data3);

// 解析响应
void AckPrash(unsigned char packet[]);

// 发送版本信息
void sendVersionData();

// 解析版本信息
void versionPrash(unsigned char packet[]);

// 发送 GPS 数据
void sendGpsData(unsigned char GPS_opt, unsigned char direction, float gpsData);

// 处理指令
void cmdPrash(unsigned char recPacket[]);     

void cmdPrash(unsigned char packet[]);
int getSendCount(void);
int getRecCount(void);




#endif

