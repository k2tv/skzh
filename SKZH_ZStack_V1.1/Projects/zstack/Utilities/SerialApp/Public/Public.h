#ifndef PUBLIC_H
#define PUBLIC_H

#ifdef __cplusplus
extern "C"
{
#endif

#define    MSG_PERIOD    0xAA
#define    MSG_RSP       0xDD
#define    MSG_RSC       0XCC

/****************************  
typedef enum                 //ԭ����
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
  Coor         = 0x00,   //����
  Hall         = 0x01,       //����
  TempHumLight = 0x02, //��ʪ��//20131017
  Shake        = 0x03,      //��
  Smoke        = 0x04,      //����
  PIR          = 0x05,        //���͵����
  Motor        = 0x06,      //���
  Touch        = 0x07,      //����
  Ultrasound   = 0x08, //������
  LED_PWM      = 0x09,    //LED����
  Relay        = 0x0a,      //�̵���
  Current      = 0x0b,    //�������  
  Voltage      = 0x0c,    //��ѹ���  
  VoltageOut   = 0x0d, //��ѹ���
  MilliVoltMeter  =0x0E,        //�������20140107
  Accele       =0x0F,       //������ٶ�20140512
  Dioxidecar   =0x10,     //CO2Ũ�ȼ��20140515
  Rain         =0x11,    //��μ��20141029
  RFID_1356    =0X12,    //RFID 13.56MHz��Ƶ20141029
}DeviceAddrList; 
  


typedef struct        
{
  uint8 Header;       //֡ͷ
  uint8 Len;          //��Ϣ����
  uint8 NodeSeq;      //�ڵ���
  uint8 NodeID;       //ģ��ID
  uint8 Command;      //����
  uint8 Data[10];     //������
  uint8 Verify;       //У���
}UART_Format;         //��������Ϣ�շ�,����λ��10��









typedef struct        
{
  uint8 Header;       //֡ͷ
  uint8 Len;          //��Ϣ����
  uint8 NodeSeq;      //�ڵ���
  uint8 NodeID;       //ģ��ID
  uint8 Command;      //����
  uint8 Verify;       //У���
}UART_Format_End0;    //��������0λ����Ϣ�շ�

typedef struct        
{
  uint8 Header;       //֡ͷ
  uint8 Len;          //��Ϣ����
  uint8 NodeSeq;      //�ڵ���
  uint8 NodeID;       //ģ��ID
  uint8 Command;      //����
  uint8 Data[1];      //������
  uint8 Verify;       //У���
}UART_Format_End1;    //��������1λ����Ϣ�շ�


typedef struct        
{
  uint8 Header;       //֡ͷ
  uint8 Len;          //��Ϣ����
  uint8 NodeSeq;      //�ڵ���
  uint8 NodeID;       //ģ��ID
  uint8 Command;      //����
  uint8 Data[2];      //������
  uint8 Verify;       //У���
}UART_Format_End2;    //��������2λ����Ϣ�շ�


typedef struct        
{
  uint8 Header;       //֡ͷ
  uint8 Len;          //��Ϣ����
  uint8 NodeSeq;      //�ڵ���
  uint8 NodeID;       //ģ��ID
  uint8 Command;      //����
  uint8 Data[4];      //������
  uint8 Verify;       //У���
}UART_Format_End4;    //��������4λ����Ϣ�շ�


typedef struct        
{
  uint8 Header;       //֡ͷ
  uint8 Len;          //��Ϣ����
  uint8 NodeSeq;      //�ڵ���
  uint8 NodeID;       //ģ��ID
  uint8 Command;      //����
  uint8 Data[6];      //������
  uint8 Verify;       //У���
}UART_Format_End6;    //��������6λ����Ϣ�շ�

typedef struct        
{
  uint8 Header;       //֡ͷ
  uint8 Len;          //��Ϣ����
  uint8 NodeSeq;      //�ڵ���
  uint8 NodeID;       //ģ��ID
  uint8 Command;      //����
  uint8 Data[8];      //������
  uint8 Verify;       //У���
}UART_Format_End8;    //��������8λ����Ϣ�շ�

typedef struct        
{
  uint8 Header;       //֡ͷ
  uint8 Len;          //��Ϣ����
  uint8 NodeSeq;      //�ڵ���
  uint8 NodeID;       //ģ��ID
  uint8 Command;      //����
  uint8 Data[9];      //������
  uint8 Verify;       //У���
}UART_Format_End9;    //��������8λ����Ϣ�շ�


typedef struct        
{
  uint8 Header;       //֡ͷ
  uint8 Len;          //��Ϣ����
  uint8 NodeSeq;      //�ڵ���
  uint8 NodeID;       //ģ��ID
  uint8 Command;      //����
  uint8 Verify;       //У���
}UART_Format_Control;  //��������0λ�Ļ���������Ϣ��Ϣ�շ�

typedef struct        
{
  uint8 Header;       //֡ͷ
  uint8 Len;          //��Ϣ����
  uint8 NodeSeq;      //�ڵ���
  uint8 NodeID;       //ģ��ID
  uint8 Command[2];      //����
  uint8 Verify;       //У���
}UART_Format_Control2;    //��������2λ����Ϣ�շ�

typedef struct        
{
  uint8 Header;       //֡ͷ
  uint8 Len;          //��Ϣ����
  uint8 NodeSeq;      //�ڵ���
  uint8 NodeID;       //ģ��ID
  uint8 Command[9];      //����
  uint8 Verify;       //У���
}UART_Format_Control9;    //��������9λ����Ϣ�շ�


typedef struct        
{
  uint8 Header;       //֡ͷ
  uint8 Len;          //��Ϣ����
  uint8 NodeSeq;      //�ڵ���
  uint8 NodeID;       //ģ��ID
  uint8 Command[14];     //������
  
}UART_Format_SerialApp;  

typedef struct        
{
  uint8 Header;       //֡ͷ
  uint8 Len;          //��Ϣ����
  uint8 NodeSeq;      //�ڵ���
  uint8 NodeID;       //ģ��ID
  uint8 Data[14];     //������
  
}UART_Format_AF;




#ifdef __cplusplus
}
#endif

#endif 
