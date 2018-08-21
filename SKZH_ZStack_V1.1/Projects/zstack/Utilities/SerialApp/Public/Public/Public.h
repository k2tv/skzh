#ifndef PUBLIC_H
#define PUBLIC_H

#ifdef __cplusplus
extern "C"
{
#endif

#define    MSG_PERIOD    0xAA
#define    MSG_RSP       0xDD

/****************************  
typedef enum                 //中软原定义
{
  Coor = 0x00,   //网关
  Hall,       //霍尔
  PhotoRes,   //光照
  TempAndHum, //温湿度
  Shake,      //震动
  Reed,       //干簧管
  Accele,     //加速度
  Smoke,      //烟雾
  Doppler,    //多普勒
  Motor,      //电机
  LED_PWM,    //LED调光
  Sound,      //声音
  Voltage,    //电压  保留
  Current,    //电流  保留
  Touch,      //触摸
  Ultrasound, //超声波
  RFID_1356,  //13.56M射频卡
  RFID_125K,  //125K射频卡
  Flame,      //火焰
  Particle,   //微粒
  Color,      //颜色
  Gyroscope,  //陀螺仪
  IR_Code,    //红外编解码
  Alcohol,     //酒精
  Relay,
  RFID_900M
}DeviceAddrList;
*********************************/

 typedef enum
{
  Coor = 0x00,   //网关
  Hall =0x01,       //霍尔
  PhotoRes=0x02,   //光照
  TempAndHum=0x03, //温湿度
  Shake=0x04,      //震动
  Reed=0x05,       //干簧管
  Accele=0x06,     //加速度
  Smoke=0x07,      //烟雾
  Doppler=0x08,    //多普勒
  Motor=0x09,      //电机
  LED_PWM=0x0a,    //LED调光
  Sound=0x0b,      //声音
  Voltage=0x0c,    //电压  保留
  Current=0x0d,    //电流  保留
  Touch=0x0e,      //触摸
  Ultrasound=0x0f, //超声波
  RFID_1356=0x10,  //13.56M射频卡
  RFID_125K=0x11,  //125K射频卡
  Flame=0x12,      //火焰
  Particle=0x13,   //微粒
  Color=0x14,      //颜色
  Gyroscope=0x15,  //陀螺仪
  IR_Code=0x16,    //红外编解码
  Alcohol=0x17,     //酒精
  Relay=0x18,
  RFID_900M=0x19
}DeviceAddrList; 
  
/****************
typedef struct  //中软原定义
{
  uint8 Header_1;
  uint8 Header_2;
  uint8 NodeSeq;
  uint8 NodeID;
  uint8 Command;
  uint8 Data[10];
  uint8 Tailer;
}UART_Format;
*****************/

/***修改后***/
typedef struct        
{
  uint8 Header;       //帧头
  uint8 Len;          //消息长度
  uint8 NodeSeq;      //节点编号
  uint8 NodeID;       //模块ID
  uint8 Command;      //命令
  uint8 Data[10];     //数据域
  uint8 Verify;       //校验和
}UART_Format;

#ifdef __cplusplus
}
#endif

#endif 
