/***************************************************************************************************************
* �� �� ����OursApp.h
��
* ��    �ܣ���Ҫ������OURS��˾PC�����ͨ��Э����صĶ����ʵ���豸ID�ȡ�        
*
* ��    ����V1.0
* ��    �ߣ�WU XIANHAI
* ��    �ڣ�2011.3.22
* �¶�˹������ҳ��www.ourselec.com
******************************************************************************************************************/

#ifndef OURSDEF_H
#define OURSDEF_H

#ifdef __cplusplus
extern "C"
{
#endif
  
#define MY_PAN_ID     2530          //����ʵ���豸��PANID
#define MY_WSN_TYPE   1          //�豸���ͣ������ݿ�õ���������ɻỰID��


//**************************************************************************
/* Э������·������Ӧ�ó�����Ϣ����*/
//һ����Ϣ����
#define FETCH_NODE_INFO                0x01     //��ȡ�ڵ���Ϣ
#define NODE_SYSTEMRESET               0x02     //�ڵ�����
#define FETCH_NODE_SENSOR_CAL          0x03     //��ȡ������У׼����

//һ��Ⱥ����Ϣ�����
#define FETCH_GROUPNODE_INFO           0x11     //��ȡ�ڵ���Ϣ
#define GROUPNODE_SYSTEMRESET          0x12     //�ڵ�����
#define TURNOFF_APP                    0x13     //�ر����нڵ�Ӧ�ù���

//������Ϣ����
#define SENDUP_NODE_INFO               0x21     //�ڵ���Ϣ(�ϴ�)
#define SENDUP_NODE_SBOARD_DATA        0x22     //�ϴ��ڵ���չ��������Դ����
#define SENDDOMN_NODE_SBOARD_DATA      0x23     //�´��ڵ���չ����Դ����
#define SENDUP_NODE_BBOARD_DATA        0x24     //�ϴ��ڵ�װ���Դ����
#define SENDDOMN_NODE_BBOARD_DATA      0x25     //�´��ڵ�װ���Դ����
#define SENDUP_NODE_PADDR_ADDR         0x26     //�ڵ㸸�ӹ�ϵ��Ϣ

#define SENDDOMN_GROUPNODE_LIST        0x27     //�ڵ������Ϣ
  
#define SET_NODE_PANID                 0x28     //�޸Ľڵ�PANID

#define SENSOR_DATA_CAL                0x29     //����������У׼

//Э��������·�ɽڵ���Ϣ״̬
#define UNALLOW_COOR_REV               0x00     //���������
#define ALLOW_COOR_REV                 0x01     //�������

//��չ�������屻�ض���  
#define APP_ADC                        0x01     //ʹ��AD����
#define APP_IIC                        0x02     //ʹ��IIC����  
#define APP_IIC_CPU                    0x03     //ʹ��IIC��CPU����
#define APP_UART                       0x04     //ʹ�ô��ڿ���


//**************************************************************************

//***************************************************************************
/* Э�������м�����֮���ͨ����Ϣ���� */
#define HEARTBEAT_DETECT               0x01     //�������
#define FIRST_APP_LINK                 0x02     //��һ���ϲ�Ӧ�����ӷ���
#define LAST_APP_ULINK                 0x03     //���һ���ϲ�Ӧ�������Ͽ�

#define EQU_ID_CODE                    0x11     //�豸ʶ����Ϣ
#define EQU_NUMBER_CODE                0x12     //�豸�����Ϣ

#define SEND_SEV                       0x01     //��SEV�����豸ʶ����Ϣ
#define REV_SEV_MSG                    0x02     //�ȴ�SEV�����豸�����Ϣ
#define SEND_HEARTBEAT                 0x03     //��������

//***************************************************************************

//***************************************************************************
/* Э������·������Ӧ�ò�֮���ͨ�Ű� */
//�ڵ�һ����Ϣ
typedef struct
{
  uint8 Hdr;                       //ͷ    
  uint8 Len;                       //����
  uint16 TransportID;              //�ỰID
  uint8 MSGCode;                   //��Ϣ����
  uint16 NodeAddr;                 //�ڵ��ַ
  uint8 Checksum;                  //У���
}PCGeneralMSGPacket_t;

//�ڵ�һ��Ӧ����Ϣ
typedef struct
{
  uint8 Hdr;                       //ͷ
  uint8 Len;                       //����
  uint16 TransportID;              //�ỰID
  uint8 MSGCode;                   //��Ϣ����
  uint16 NodeAddr;                 //�ڵ��ַ
  uint8 ReplyCode;                 //Ӧ�����
  uint8 Checksum;                  //У���
}PCMSGReplyPacket_t;

//һ��Ⱥ���飩����Ϣ
typedef struct
{
  uint8 Hdr;                       //ͷ    
  uint8 Len;                       //����
  uint16 TransportID;              //�ỰID
  uint8 MSGCode;                   //��Ϣ����
  uint16 GroupID;                  //���
  uint8 Checksum;                  //У���
}PCGroupMSGPacket_t;

//�ڵ���Ϣ(�ϴ�)
typedef struct
{
  uint8 Hdr;                       //ͷ
  uint8 Len;                       //����
  uint16 TransportID;              //�ỰID
  uint8 MSGCode;                   //��Ϣ����
  uint16 NodeAddr;                 //�ڵ��ַ
  uint8 IEEEAddr[8];               //IEEE��ַ
  int8 NodeRSSI;                   //�ź�ǿ��
  uint8 NodeLQI;                   //��·����
  uint16 Panid;                    //����ID
  uint8 TemType;                   //ģ������   1Ϊ���ܰ� 2Ϊ��ͨ��ذ�
  uint16 SBoardType;               //������������
  uint8 Checksum;                  //У���
}PCNodeMSGPacket_t;

//�ڵ㸸�ӹ�ϵ��Ϣ
typedef struct
{
  uint8 Hdr;                       //ͷ
  uint8 Len;                       //����
  uint16 TransportID;              //�ỰID
  uint8 MSGCode;                   //��Ϣ����
  uint16 NodeAddr;                 //�ڵ��ַ
  uint16 NodePAddr;                //���ڵ��ַ
  uint8 Checksum;                  //У���
}PCNodeAddrPacket_t;

//�ϴ���չģ����Դ������Ϣ
typedef struct
{
  uint8 Hdr;                       //ͷ
  uint8 Len;                       //����
  uint16 TransportID;              //�ỰID
  uint8 MSGCode;                   //��Ϣ����
  uint16 NodeAddr;                 //�ڵ��ַ
  uint16 ModeID;                   //ģ�����
  uint16 *data;                     //����
  uint8 Checksum;                  //У���
}SendUpSBoardDataPacket_t;

//�ϴ���չģ����Դ������Ϣ2������λ8λ�ģ�
typedef struct
{
  uint8 Hdr;                       //ͷ
  uint8 Len;                       //����
  uint16 TransportID;              //�ỰID
  uint8 MSGCode;                   //��Ϣ����
  uint16 NodeAddr;                 //�ڵ��ַ
  uint16 ModeID;                   //ģ�����
  uint8 *data;                     //����
  uint8 Checksum;                  //У���
}SendUpSBoardDataPacket2_t;

//�´���չģ��������Ϣ
typedef struct
{
  uint8 Hdr;                       //ͷ
  uint8 Len;                       //����
  uint16 TransportID;              //�ỰID
  uint8 MSGCode;                   //��Ϣ����
  uint16 NodeAddr;                 //�ڵ��ַ
  uint16 ModeID;                   //ģ�����
  uint8 *data;                     //����
  uint8 Checksum;                  //У���
}SendDownSBoardDataPacket_t;

//�������ýڵ�PANID
typedef struct
{
  uint8 Hdr;                       //ͷ
  uint8 Len;                       //����
  uint16 TransportID;              //�ỰID
  uint8 MSGCode;                   //��Ϣ����
  uint16 NodeAddr;                 //�ڵ��ַ
  uint16 SETPANID;                  //ģ�����
  uint8 Checksum;                  //У���
}SetPANIDPacket_t;

typedef struct
{
  uint8 Hdr;                       //ͷ
  uint8 Len;                       //����
  uint16 TransportID;              //�ỰID
  uint8 MSGCode;                   //��Ϣ����
  uint16 NodeAddr;                 //�ڵ��ַ
  uint16 ModeID;                   //ģ�����
  uint16 Maxdata;                  //ģ�����
  uint16 Zerodata;                 //��ֵ
  uint16 Mindata;                  //��Сֵ
  uint8 Checksum;                  //У���
}SensorCalPacket_t;

//ADC������Դ���ݶ�
typedef struct
{
  uint8 ChComb;                    //ͨ�����
  uint8 SampleRate;                //��������
  uint16 SamplingSpeed;            //��������
  uint16 SampleMode;               //����ģʽ
  uint16 SampleGap;                //�ɼ����
  uint16 SampleCount;              //��������
  uint8 PacketLen;                 //���ݰ�����
}ADControlData_t;

//DA������Դ���ݶ�
typedef struct
{
  uint8 ChComb;                    //ͨ�����
  uint16 dataA;                    //ͨ��A����
  uint16 dataB;                    //ͨ��B����
  uint16 dataC;                    //ͨ��C����
  uint16 dataD;                    //ͨ��D����
}DAControlData_t;

//���ڿ�����Դ���ݶ�
typedef struct
{
  uint8 UARTmode;                   //ģʽ
  uint8 UARTid;                     //���ں�
  uint8 UART_BR;                    //������
  uint8 CTR;                        //��żУ��
  uint8 databit;                    //����λ
  uint8 stopbit;                    //ֹͣλ
  uint8 flowctr;                    //����������
}UARTontrolData_t;

//������Դ���ݶ�
typedef struct
{
  uint8 UARTmode;                   //ģʽ
  uint8 UARTid;                     //���ں�
  uint8 DataLen;                    //���ݳ���
  uint8 *data;                      //����
}UARTData_t;

//�ϴ�ĸ����Դ������Ϣ
typedef struct
{
  uint8 Hdr;                       //ͷ
  uint8 Len;                       //����
  uint16 TransportID;              //�ỰID
  uint8 MSGCode;                   //��Ϣ����
  uint16 NodeAddr;                 //�ڵ��ַ
  uint8 TemType;                   //ģ������
  uint16 ResID;                    //��Դ����
  uint8 *data;                     //����
  uint8 Checksum;                  //У���
}SendUpTemBoardDataPacket_t;

//�´�ĸ����Դ������Ϣ
typedef struct
{
  uint8 Hdr;                       //ͷ
  uint8 Len;                       //����
  uint16 TransportID;              //�ỰID
  uint8 MSGCode;                   //��Ϣ����
  uint16 NodeAddr;                 //�ڵ��ַ
  uint8 TemType;                   //ģ������
  uint16 ResID;                   //��Դ����
  uint8 *data;                     //����
  uint8 Checksum;                  //У���
}SendDownTemBoardDataPacket_t;

//�ڵ������Ϣ
typedef struct
{
  uint8 Hdr;                       //ͷ
  uint8 Len;                       //����
  uint16 TransportID;              //�ỰID
  uint8 MSGCode;                   //��Ϣ����
  uint8 GroupID;                   //���   0x00 - 0xfe 
  uint16 *NodeAddr;                //�����ڸ��ڵ��ַ
  uint8 Checksum;                  //У���
}GroupingMSGPacket_t;

//***************************************************************************


//***************************************************************************
/* Э����������֮���ͨ�Ű� */
//�豸ʶ��
typedef struct
{
  uint8 Hdr;                       //ͷ
  uint8 Len;                       //����
  uint16 TransportID;              //�ỰID
  uint8 MSGCode;                   //��Ϣ����
  uint8 AppType;                   //Ӧ������
  uint8 EQUType;                   //�豸����
  uint8 Heartbeat_Timeout;         //������ⳬʱ
  uint8 Heartbeat_Period;          //�������������
  uint8 Checksum;                  //У���
}SEVLinkCoordPacket_t;

//һ����Ϣ
typedef struct
{
  uint8 Hdr;                       //ͷ    
  uint8 Len;                       //����
  uint16 TransportID;              //�ỰID
  uint8 MSGCode;                   //��Ϣ����
  uint8 Checksum;                  //У���
}SEVGeneralMSGPacket_t;

//һ��Ӧ����Ϣ
typedef struct
{
  uint8 Hdr;                       //ͷ    
  uint8 Len;                       //����
  uint16 TransportID;              //�ỰID
  uint8 MSGCode;                   //��Ϣ����
  uint8 ReplyCode;                 //Ӧ�����
  uint8 Checksum;                  //У���
}SEVMSGReplyPacket_t;

//�豸�����Ϣ
typedef struct
{
  uint8 Hdr;                       //ͷ    
  uint8 Len;                       //����
  uint16 TransportID;              //�ỰID
  uint8 MSGCode;                   //��Ϣ����
  uint8 NumberCode;                //�豸���
  uint8 Checksum;                  //У���
}SEVEQUNumberPacket_t;


//**************************************************************************

//***************************************************************************
//��Դ���붨��
//AD��ʽ
#define resCode_Voltage_Single		((0x10<<8)+0x00)
#define resCode_Voltage_Dual		((0x10<<8)+0x01)
#define resCode_Current_Single		((0x10<<8)+0x10)
#define resCode_Current_Dual		((0x10<<8)+0x11)
#define resCode_MinCurrent_Single	((0x10<<8)+0x18)
#define resCode_MinCurrent_Dual		((0x10<<8)+0x19)
#define resCode_Light_Single		((0x10<<8)+0x20)
#define resCode_Pressure		((0x10<<8)+0x30)
#define resCode_Alcohol			((0x10<<8)+0x31)
#define resCode_Pressure_Alcohol	((0x10<<8)+0x32)
//IIC�ӿڷ�ʽ
#define resCode_Temp_Humidity		((0x20<<8)+0x00)
#define resCode_Temp_Humidity_Light	((0x20<<8)+0x01)
#define resCode_Voltage_Output		((0x20<<8)+0x10)
#define resCode_Relay_GPIN		((0x20<<8)+0x20)
//IIC+�ⲿCPUʵ��
#define resCode_Photoelectric		((0x21<<8)+0x00)
#define resCode_IR_Output		((0x21<<8)+0x01)
#define resCode_IR_Photoelectric	((0x21<<8)+0x03)
#define resCode_Ultrasonic		((0x21<<8)+0x10)
//���ڽӿ�
#define resCode_RS232_Wireless		((0x30<<8)+0x00)
#define resCode_RDID			((0x30<<8)+0x10)
#define resCode_RS232_Coordinator	((0x31<<8)+0x00)

//��Դ�壬���ܰ�
#define resCode_Relay			((0x01<<8)+0x00)
#define resCode_Led			((0x02<<8)+0x00)
#define resCode_Buzzer			((0x03<<8)+0x00)
//���ܰ� ����һ��
#define resCode_AD			((0x04<<8)+0x00)

//***************************************************************************
#ifdef __cplusplus
}
#endif

#endif /* OURSDEF_H */





