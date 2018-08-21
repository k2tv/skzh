#ifndef PUBLIC_H
#define PUBLIC_H

#ifdef __cplusplus
extern "C"
{
#endif

#define    MSG_PERIOD    0xAA
#define    MSG_RSP       0xDD

/****************************  
typedef enum                 //����ԭ����
{
  Coor = 0x00,   //����
  Hall,       //����
  PhotoRes,   //����
  TempAndHum, //��ʪ��
  Shake,      //��
  Reed,       //�ɻɹ�
  Accele,     //���ٶ�
  Smoke,      //����
  Doppler,    //������
  Motor,      //���
  LED_PWM,    //LED����
  Sound,      //����
  Voltage,    //��ѹ  ����
  Current,    //����  ����
  Touch,      //����
  Ultrasound, //������
  RFID_1356,  //13.56M��Ƶ��
  RFID_125K,  //125K��Ƶ��
  Flame,      //����
  Particle,   //΢��
  Color,      //��ɫ
  Gyroscope,  //������
  IR_Code,    //��������
  Alcohol,     //�ƾ�
  Relay,
  RFID_900M
}DeviceAddrList;
*********************************/

 typedef enum
{
  Coor = 0x00,   //����
  Hall =0x01,       //����
  PhotoRes=0x02,   //����
  TempAndHum=0x03, //��ʪ��
  Shake=0x04,      //��
  Reed=0x05,       //�ɻɹ�
  Accele=0x06,     //���ٶ�
  Smoke=0x07,      //����
  Doppler=0x08,    //������
  Motor=0x09,      //���
  LED_PWM=0x0a,    //LED����
  Sound=0x0b,      //����
  Voltage=0x0c,    //��ѹ  ����
  Current=0x0d,    //����  ����
  Touch=0x0e,      //����
  Ultrasound=0x0f, //������
  RFID_1356=0x10,  //13.56M��Ƶ��
  RFID_125K=0x11,  //125K��Ƶ��
  Flame=0x12,      //����
  Particle=0x13,   //΢��
  Color=0x14,      //��ɫ
  Gyroscope=0x15,  //������
  IR_Code=0x16,    //��������
  Alcohol=0x17,     //�ƾ�
  Relay=0x18,
  RFID_900M=0x19
}DeviceAddrList; 
  
/****************
typedef struct  //����ԭ����
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

/***�޸ĺ�***/
typedef struct        
{
  uint8 Header;       //֡ͷ
  uint8 Len;          //��Ϣ����
  uint8 NodeSeq;      //�ڵ���
  uint8 NodeID;       //ģ��ID
  uint8 Command;      //����
  uint8 Data[10];     //������
  uint8 Verify;       //У���
}UART_Format;

#ifdef __cplusplus
}
#endif

#endif 
