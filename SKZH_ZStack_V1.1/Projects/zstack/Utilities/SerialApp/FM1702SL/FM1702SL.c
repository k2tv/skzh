/**************************************Copyright(c)****************************************
* ����������������            �ൺɽ���ǻ���Ϣ�Ƽ����޹�˾ ���������� ����                *
* ��������������������������                          ������������������������������������*
* ������������������                  www.iotsk.com ������������������                  ��*
* ������������������������������������������������������                          ��������*
*----------------------------------------File Info----------------------------------------*
*�� �� �� ��FM1702SL.c������������                                                         *
*�޸����� ��2014.10.29  ����������������������������                                      *
*�� �� �� ��V1.0��������������������������������                                          *
*��    �� ��RFID-13.56MHz��Ƶģ�飬��ɶԱ�ǩ�Ķ�д������ʵ�ֲֿ�����ƶ�֧����Ӧ��                    *
*                                                                                         *
******************************************************************************************/

/******************************************************************************************
*                                       INCLUDES                                          *
******************************************************************************************/

#include "AF.h"
#include "OnBoard.h"
#include "OSAL_Tasks.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "hal_drivers.h"
#include "hal_key.h"
#if defined ( LCD_SUPPORTED )
   #include "hal_lcd.h"
#endif
#include "hal_led.h"
#include "hal_uart.h"
#include "Public.h"
#include "fm1702sl.h"
#include "spi.h"
#include <string.h>
#include "hal.h"
#include "stdio.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#if !defined( SERIAL_APP_PORT )
#define SERIAL_APP_PORT  0
#endif

#if !defined( SERIAL_APP_BAUD )
#define SERIAL_APP_BAUD  HAL_UART_BR_19200
#endif

// When the Rx buf space is less than this threshold, invoke the Rx callback.
#if !defined( SERIAL_APP_THRESH )
#define SERIAL_APP_THRESH  64
#endif

#if !defined( SERIAL_APP_RX_SZ )
#define SERIAL_APP_RX_SZ  128
#endif

#if !defined( SERIAL_APP_TX_SZ )
#define SERIAL_APP_TX_SZ  128
#endif

// Millisecs of idle time after a byte is received before invoking Rx callback.
#if !defined( SERIAL_APP_IDLE )
#define SERIAL_APP_IDLE  6
#endif

// Loopback Rx bytes to Tx for throughput testing.
#if !defined( SERIAL_APP_LOOPBACK )
#define SERIAL_APP_LOOPBACK  FALSE
#endif

// This is the max byte count per OTA message.
#if !defined( SERIAL_APP_TX_MAX )
#define SERIAL_APP_TX_MAX  20
#endif

#define SERIAL_APP_RSP_CNT  4
#define M1AREA 		0x01     //Ҫ������������


#define ON  1
#define OFF 0

// This list should be filled with Application specific Cluster IDs.
const cId_t SerialApp_ClusterList[SERIALAPP_MAX_CLUSTERS] =
{
  SERIALAPP_CLUSTERID1,
  SERIALAPP_CLUSTERID2
};

const SimpleDescriptionFormat_t SerialApp_SimpleDesc =
{
  SERIALAPP_ENDPOINT,              //  int   Endpoint;
  SERIALAPP_PROFID,                //  uint16 AppProfId[2];
  SERIALAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SERIALAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SERIALAPP_FLAGS,                 //  int   AppFlags:4;
  SERIALAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)SerialApp_ClusterList,  //  byte *pAppInClusterList;
  SERIALAPP_MAX_CLUSTERS,          //  byte  AppNumOutClusters;
  (cId_t *)SerialApp_ClusterList   //  byte *pAppOutClusterList;
};

const endPointDesc_t SerialApp_epDesc =
{
  SERIALAPP_ENDPOINT,
 &SerialApp_TaskID,
  (SimpleDescriptionFormat_t *)&SerialApp_SimpleDesc,
  noLatencyReqs
};

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

uint8 SerialApp_TaskID;    // Task ID for internal task/event processing.
devStates_t  SampleApp_NwkState;
static UART_Format UART0_Format;
static UART_Format UART0_Format1;
static UART_Format_End6 Rsp;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint8 SerialApp_MsgID;
static afAddrType_t SerialApp_TxAddr;
static uint8 SerialApp_TxBuf[SERIAL_APP_TX_MAX];
static uint8 SerialApp_TxLen;
static uint8 RevBuffer[24];
static uint8 RevBuffer1[24];
uint8  CardNo[5];
/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt );
void SerialApp_OTAData(afAddrType_t *txaddr,uint8 ID,void *p,uint8 len);
static void SerialApp_CallBack(uint8 port, uint8 event);
void  WriteReg(int8 reg, int8 data);
int8  ReadReg(int8 reg_ad);
void  InitFM1702SL(void);
static void GPIOInit(void);
uint8 Request(uint8 mode);
void  Fifo_Write(uint8 *s, uint8 count);
uint8 FM1702BusSel(void);
uint8 Clear_FIFO(void);
uint8 Fifo_Read(uint8 *buff);
uint8 Command_Send(uint8 count, uint8 *buff, uint8 Comm_Set);
uint8 Judge_Req(uint8 *buff);
uint8 ReadCardNum(void);
uint8 AntiColl(void);
uint8 Check_UID(void);
uint8 Select_Card(void);
void  keyto(void);
uint8 block_numset(uint8 block_num);
uint8 Load_key_CPY(uint8 *buff);
int8  M500HostCodeKey(uint8 *uncoded, uint8 *coded);   
uint8 Authentication(uint8 *UID, uint8 SecNR);
uint8 MIF_READ(uint8 *buff, uint8 Block_Adr);
uint8 MIF_Write(uint8 *buff, uint8 Block_Adr);
uint8 CheckSum(uint8 *data,uint8 len);
extern void Buzzer(uint8 operation);
extern void FLASHLED(uint8 led);
extern void LED(uint8 led,uint8 operation);
extern void ledInit(void);


static void GPIOInit(void)
{
  P1DIR |= 0x04;  //P12  output
 // P0DIR |= 0x01;  //P00
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//����: rc531_register_write                                                   // 
//����: �ú���ʵ��ͨ��SPI�ӿڶ�RC531��һ���Ĵ���д��ֵ                         // 
//                                                                             // 
//����:                                                                        // 
//     RC531Ŀ��Ĵ�����ַ��д��ֵ                                             // 
//                                                                             // 
//���:                                                                        // 
//     N/A                                                                     // 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
void WriteReg(int8 reg_ad, int8 reg_data)
{   
    sck=0;                               //ʱ��
    reg_ad <<= 1;
    cs=0;                              //�ӿ�ѡͨ
    reg_ad &= 0x7F;                      //���λΪ0��ʾд
    SPIOneByte(reg_ad);     //д��ַ
    SPIOneByte(reg_data);  //д����
    cs=1;
    return;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
//����: rc531_register_read                                                    // 
//����: �ú���ʵ��ͨ��SPI�ӿڶ�ȡRC531��һ���Ĵ�����ֵ                         // 
//                                                                             // 
//����:                                                                        // 
//     RC531Ŀ��Ĵ�����ַ                                                     // 
//                                                                             // 
//���:                                                                        // 
//     Ŀ��Ĵ�����ֵ                                                          // 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
int8 ReadReg(int8 reg_ad)
{   
  int8 temp;
  sck=0;       //ʱ��
  MicroWait(6);
  cs=0;      //�ӿ�ѡͨ��
  reg_ad <<= 1;                          //reg_ad����һλ��ֵ��reg_ad
  reg_ad |= 0x80;                        //reg_ad����0X80���������ֵ��reg_ad   ���λΪ1��ʾ��
  SPIOneByte(reg_ad);
  temp=SPIOneByte(0x00);
  cs=1;
  return (temp);
}
/*����ѡ��*/
uint8 FM1702BusSel(void)
{
  uint8 i, temp;
  WriteReg(PageSel, 0x80); //ѡ��Ĵ�����0
  for(i = 0; i < 0xff; i++)
  {
    temp = ReadReg(RFIDCommand); //��ȡ����ִ�е�����
    if(temp == 0x00)	//���û��ָ��ִ��
    {	
      WriteReg(PageSel,0x00); //�ָ��Ĵ���������Ѱַ
      return TRUE;	
    }
  }
  return FALSE;
}

/*********************************************************/
//���ͣ�FIFO��������д����
//���룺д����ֽڸ��� ��Ҫд���ֵ�Ĵ���׵�ַ 
//�����
/********************************************************/
void Fifo_Write(uint8 *s, uint8 count)
{
  uint8 i,temp;
  for(i=0;i<count;i++)
  {
    temp=*(s+i);
    WriteReg(FIFODaTa, temp);
    MicroWait(100);
  }
}

uint8 Fifo_Read(uint8 *buff)
{
  uint8	temp;
  uint8	i;
  temp = ReadReg(FIFOLength);
  if(temp == 0)
  {
    return 0;
  }
  if(temp >= 24)		
  {
    temp = 24;	
  }
  for(i = 0; i < temp; i++)
  {
    *(buff + i) = ReadReg(FIFODaTa);
  }
  return temp;  
}

uint8 Judge_Req(uint8 *buff)
{
  uint8	temp1, temp2;
  temp1 = *buff;
  temp2 = *(buff + 1);
  if((temp1 == 0x02) || (temp1 == 0x04) || (temp1 == 0x05) || (temp1 == 0x53) || (temp1 == 0x03))
  {
    if (temp2 == 0x00)
    {
      return TRUE;
    }
  }
  return FALSE;
}
/////////////////////////////////////////////////////////
void InitFM1702SL(void)
{
  FM1702_RSTPD=1;
  MicroWait(1000);			
  FM1702_RSTPD=0;				 
  MicroWait(1000);	//��λ1702SL
    	
  while(ReadReg(RFIDCommand) != 0)
  {
    MicroWait(10); 
  }
  FM1702BusSel();  
  WriteReg(InterruptEn,0x7f);		     
  WriteReg(InterruptRq,0x7f);            	
  WriteReg(TxControl,0x5b);
  WriteReg(RxControl2,0x01);
  WriteReg(RxWait,0x07);
  WriteReg(CryptoSelect,0x00);
}  

/****************************************************************/
/*����: Clear_FIFO */
/*����: �ú���ʵ�����FM1715��FIFO������*/
/*����: N/A */
/*���: TRUE, FIFO����� */
/* FALSE, FIFOδ����� */
/****************************************************************/
uint8 Clear_FIFO(void)
{
  uint8  temp;
  uint16 i;
  temp = ReadReg(Control);
  temp = (temp | 0x01);
  WriteReg(Control, temp);
  for(i = 0; i < RF_TimeOut; i++) //���FIFO�Ƿ����
  {
    temp = ReadReg(FIFOLength);
    if(temp == 0)
    {
      return TRUE;
    }
  }
  return FALSE;
}
/****************************************************************/
/*����: Command_Send */
/*����: �ú���ʵ����FM1715��������Ĺ��� */
/*����: count, ����������ĳ���*/
/* buff, ָ����������ݵ�ָ�� */
/* Comm_Set, ������ */
/*���: TRUE, �����ȷִ��*/
/* FALSE, ����ִ�д���*/
/****************************************************************/
uint8 Command_Send(uint8 count, uint8 *buff, uint8 Comm_Set)
{
	uint16   j;
	uint8  temp;
	WriteReg(RFIDCommand,0x00);
	Clear_FIFO();
	Fifo_Write(buff, count);   //��26Hд��FIFO
	WriteReg(RFIDCommand,Comm_Set);		/* ����ִ�� */
	for(j = 0; j < RF_TimeOut; j++) /* �������ִ�з� */
	{
	  temp = ReadReg(RFIDCommand);
	  if(temp == 0x00)//���TEMPΪ��ִ��IF��䣬�����˳�IF���
	  {
	    return TRUE;//�˳�FOR��䣨���������أ�
	  }
	}
	return FALSE;
}

//A����ļ��ܸ�ʽ��00 4���ֽڵ����к� ��ţ������ţ�����6���ֽڵ���Կ
void keyto(void)
{
  uint8 i;
  RevBuffer[4] = block_numset(3);
  for(i = 0; i < 7; i++)
    RevBuffer[5 + i] = 0xff;		//Ĭ������
}

uint8 block_numset(uint8 block_num)
{ 	
  unsigned char temp,i;
  i = M1AREA;
  temp = block_num;
  while(i)
  {
    temp=temp+4;
    i--;
  }
  return temp;
}
/****************************************************************/
/*����: Request */
/*����: �ú���ʵ�ֶԷ���FM1702������Χ֮�ڵĿ�Ƭ��Request����*/
/*����: mode: ALL(�������FM1702������Χ֮�ڵĿ�Ƭ) */
/* STD(�����FM1702������Χ֮�ڴ���HALT״̬�Ŀ�Ƭ) */
/*���: FM1702_NOTAGERR: �޿�*/
/* FM1702_OK: Ӧ����ȷ*/
/* FM1702_REQERR: Ӧ�����*/
/****************************************************************/
uint8 Request(uint8 mode)
{	
  uint8 temp;	
  RevBuffer[0] = mode; //00x26
  WriteReg(0x11,0x58);//�ط������
  WriteReg(0x11,0x5b);//���������
  WriteReg(0x0f,0x07);          //address 0FH  ����λ��֡��ʽ	
  temp = ReadReg(0x09);
  temp &= 0xf7;
  WriteReg(0x09,temp);
  WriteReg(0x22,0x03);
  
  temp = Command_Send(1, RevBuffer, Transceive); //д0x26��FIFO
  if(temp == FALSE)
  {
    return FALSE;	
  }
  temp = Fifo_Read(RevBuffer);    //��ȡFIFO��������ݼ����ݳ���	
  if(temp == 2)
  {
    temp = Judge_Req(RevBuffer); //�Կ�Ƭ��λӦ���źŵ��жϷ���ֵ�Ƿ���ȷ
    if(temp == TRUE)
    {
      return TRUE;	
    }
  }
  return FALSE;
}

/****************************************************************/
/*����: Load_keyE */
/*����: �ú���ʵ�ְ�E2���������FM1702��keyRevBuffer��*/
/*����: Secnr: EE��ʼ��ַ*/
/*���: True: ��Կװ�سɹ�*/
/* False: ��Կװ��ʧ��*/
/****************************************************************/
uint8 Load_key_CPY(uint8 *buff)
{
  int8  status;
  uint8 coded_keys[12];
  uint8 temp;
  M500HostCodeKey(buff, coded_keys);////////////////	
  temp=Command_Send(0x0c,coded_keys,LoadKey);/* LoadKey����Կ��FIFO���渴�Ƶ�KEY���� 0x19*/
  if(temp == FALSE)       //0x0cΪ12���ֽڳ��� //coded_keysΪָ���ַ�Ĵ���
  {
    return FM1702_LOADKEYERR;
  }
  status = (ReadReg(ErrorFlag)) & 0x40;//�ж�loadkeyִ���Ƿ���ȷ
  if (status==0x40)
    return FM1702_AUTHERR;
  return FM1702_OK;
}

// ת����Կ��ʽ
///////////////////////////////////////////////////////////////////////
int8 M500HostCodeKey(uint8 *uncoded, uint8 *coded)   
{
  uint8 cnt = 0;
  uint8 ln  = 0;     
  uint8 hn  = 0;      
  for (cnt = 0; cnt < 6; cnt++)
  {
    ln = uncoded[cnt] & 0x0F;
    hn = uncoded[cnt] >> 4;
    coded[cnt * 2 + 1] = (~ln << 4) | ln;
    coded[cnt * 2 ] = (~hn << 4) | hn;
  }
  return 0;
}

/*��ȡ����*/
uint8 ReadCardNum(void)
{
  uint8 st;
  st= AntiColl();
  if(st!=0)				 
    return (1);
  st = Select_Card();  //ѡ��Ƭ
  if(st!=0)
    return (1);
  keyto();
  st = Load_key_CPY(&RevBuffer[5]);
  if(st!=0)
    return (1);		       
  st = Authentication(CardNo, RevBuffer[4]);
  if(st!=0)
    return (1);
  return (0);
}

/****************************************************************/
/*����: Authentication */
/*����: �ú���ʵ��������֤�Ĺ���*/
/*����: UID: ��Ƭ���кŵ�ַ*/
/* SecNR: ������*/
/*���: FM1702_NOTAGERR: �޿�*/
/* FM1702_PARITYERR: ��żУ���*/
/* FM1702_CRCERR: CRCУ���*/
/* FM1702_OK: Ӧ����ȷ*/
/* FM1702_AUTHERR: Ȩ����֤�д�*/
/****************************************************************/
uint8 Authentication(uint8 *UID, uint8 SecNR)
{	
  uint8 i;
  uint8 temp, temp1;	
  RevBuffer[0] = RF_CMD_AUTH_LA;   //����A   
  RevBuffer[1] = SecNR;  
  for(i = 0; i < 4; i++)
  RevBuffer[2 + i] = UID[i];//�����кŷ��뻺�����Ƿ�Ϊͬһ�ſ�	  
  WriteReg(ChannelRedundancy, 0x0f);  //������֤��ͨ��
//************ Authent1=0x0c ��֤������֤���̵�1�� ****************//	
  temp = Command_Send(6, RevBuffer, Authent1);
  if(temp == FALSE)   			
    return FM1702_AUTHERR;	
//************ Authent2=0x14 ��֤������֤���̵�2�� ****************//
  temp = Command_Send(0, RevBuffer, Authent2);//����������ڶ�����ͨ��
  if(temp == FALSE)			
    return FM1702_AUTHERR;
  temp1 = ReadReg(Control);//�����Ʊ�ʶ�Ĵ���Ϊ0X08ʱ��ʾ���ܵ�Ԫ�򿪣�
  temp1 = temp1 & 0x08;	//ͨ�����λ�ڲ���1 ��CRYPTO1ON��
  if(temp1 == 0x08)
    return (0);
  return FM1702_AUTHERR;
}

/****************************************************************/
/*����: Select_Card */
/*����: �ú���ʵ�ֶԷ���FM1702������Χ֮�ڵ�ĳ�ſ�Ƭ����ѡ��*/
/*����: N/A */
/*���: FM1702_NOTAGERR: �޿�*/
/* FM1702_PARITYERR: ��żУ���*/
/* FM1702_CRCERR: CRCУ���*/
/* FM1702_BYTECOUNTERR: �����ֽڴ���*/
/* FM1702_OK: Ӧ����ȷ*/
/* FM1702_SELERR: ѡ������*/
/****************************************************************/
uint8 Select_Card(void)
{
  uint8 temp, i;	

  RevBuffer[0] = RF_CMD_SELECT;//д0x93����ʱ�رշ����Լ�����CRCУ���Լ�������У��
  RevBuffer[1] = 0x70; //������PICC_REQSTD ����ʱRegBitFraming Ҫװ��0x07
  for(i = 0; i < 5; i++)
  {
    RevBuffer[i + 2] = CardNo[i];  //��5�������к�ת���뻺����
  }
  WriteReg(ChannelRedundancy, 0x0f);//ѡ������У��������ģʽ	
  temp = Command_Send(7, RevBuffer, Transceive);//Transceive=1EH���������
  if(temp == FALSE)			
  {
    return(1);
  }
  else
  {
    Fifo_Read(RevBuffer);	/* ��FIFO�ж�ȡӦ����Ϣ, ��ȡ��Ƭ����*/
    temp = *RevBuffer;			//���ؿ�Ƭ����S50��:08,S70��:18
    if((temp == 0x18) || (temp == 0x08) || (temp == 0x28) || (temp == 0x53))	/* �ж�Ӧ���ź��Ƿ���ȷ */
      return(0);
    else
      return(FM1702_SELERR);
  }
}

/****************************************************************/
/*����: Check_UID */
/*����: �ú���ʵ�ֶ��յ��Ŀ�Ƭ�����кŵ��ж�*/
/*����: N/A */
/*���: TRUE: ���к���ȷ*/
/* FALSE: ���кŴ���*/
/****************************************************************/
uint8 Check_UID(void)
{
  uint8 temp;
  uint8 i;
  temp = 0x00;
  for(i = 0; i < 5; i++)
  {
    temp = temp ^ CardNo[i];//�Կ���������㣬��������ͬΪ0����ͬΪ1
  }						//��ȷ���Ϊ0
  if(temp == 0)
  {
    return TRUE;
  }
  return FALSE;
}

/****************************************************************/
/*����: AntiColl */
/*����: �ú���ʵ�ֶԷ���FM1702������Χ֮�ڵĿ�Ƭ�ķ���ͻ���*/
/*����: N/A */
/*���: FM1702_NOTAGERR: �޿�*/
/* FM1702_BYTECOUNTERR: �����ֽڴ���*/
/* FM1702_SERNRERR: ��Ƭ���к�Ӧ�����*/
/* FM1702_OK: ��ƬӦ����ȷ*/
/****************************************************************/
uint8 AntiColl(void)
{
  uint8	temp;
  uint8	i;
  RevBuffer[0] = RF_CMD_ANTICOL;    //дѰ��ͨѶ����93H
  RevBuffer[1] = 0x20;              //дѰ��ͨѶ����20H
  WriteReg(ChannelRedundancy, 0x03); //address 22H  ѡ������У�����������
  temp = Command_Send(2, RevBuffer, Transceive);//Transceive=0X1E����
  while(1)
  {
    if(temp == FALSE)
    {
      return(1);   //�޿�
    }	  
    temp = ReadReg(FIFOLength);   //��0x04��ķ��ؿ��ų���
    if(temp == 0)
    {
      return FM1702_BYTECOUNTERR; //�����ֽڳ��ȴ���
    }
    Fifo_Read(RevBuffer);           //��ȡ����  					
    for(i = 0; i < temp; i++)
    {
      CardNo[i] = RevBuffer[i]; //����ʱ�Ĵ����еĿ��ŷ���UIDר�üĴ�����
    }		
    temp = Check_UID();			/* У���յ���UID */
    if(temp == FALSE)
    {
      return(FM1702_SERNRERR);
    }
    return(0);
  }
}

/****************************************************************/
/*����: MIF_Read */
/*����: �ú���ʵ�ֶ�MIFARE�������ֵ*/
/*����: buff: �������׵�ַ*/
/* Block_Adr: ���ַ*/
/*���: FM1702_NOTAGERR: �޿�*/
/* FM1702_PARITYERR: ��żУ���*/
/* FM1702_CRCERR: CRCУ���*/
/* FM1702_BYTECOUNTERR: �����ֽڴ���*/
/* FM1702_OK: Ӧ����ȷ*/
/****************************************************************/
uint8 MIF_READ(uint8 *buff, uint8 Block_Adr)
{
	uint8 temp;
	WriteReg(0x22,0x0f);
	buff[0] = RF_CMD_READ;   //0x30
	buff[1] = Block_Adr;
	temp = Command_Send(2, buff, Transceive);
	if(temp == 0)
	{			
	  return (1);  //����
	}

	temp = ReadReg(0x04);
	if(temp == 0x10)	
	{
	  Fifo_Read(buff);
	  return (0); //��ȷ
	}
	return (1);
}

/****************************************************************/
/*����: MIF_Write */
/*����: �ú���ʵ��дMIFARE�������ֵ*/
/*����: buff: �������׵�ַ*/
/* Block_Adr: ���ַ*/
/*���: FM1702_NOTAGERR: �޿�*/
/* FM1702_BYTECOUNTERR: �����ֽڴ���*/
/* FM1702_NOTAUTHERR: δ��Ȩ����֤*/
/* FM1702_EMPTY: �����������*/
/* FM1702_CRCERR: CRCУ���*/
/* FM1702_PARITYERR: ��żУ���*/
/* FM1702_WRITEERR: д�������ݳ���*/
/* FM1702_OK: Ӧ����ȷ*/
/****************************************************************/
uint8 MIF_Write(uint8 *buff, uint8 Block_Adr)
{
	uint8	temp;
	uint8	*F_buff;

	WriteReg(0x23,0x63);
	WriteReg(0x12,0x3f);
	F_buff = buff + 0x10;
	WriteReg(0x22,0x07);    /* Note: this line is for 1702, different from RC500*/
	*F_buff = RF_CMD_WRITE;
	*(F_buff + 1) = Block_Adr;
	temp = Command_Send(2, F_buff, Transceive);
	if(temp == FALSE)
	{
		return(1);
	}

	temp = ReadReg(0x04);
	if(temp == 0)
	{
		return(FM1702_BYTECOUNTERR);
	}

	Fifo_Read(F_buff);
	temp = *F_buff;
	switch(temp)
	{
	case 0x00:	return(FM1702_NOTAUTHERR);	
	case 0x04:	return(FM1702_EMPTY);
	case 0x0a:	break;
	case 0x01:	return(FM1702_CRCERR);
	case 0x05:	return(FM1702_PARITYERR);
	default:	return(FM1702_WRITEERR);
	}

	temp = Command_Send(16, buff, Transceive);
	if(temp == TRUE)
	{
		return(0);
	}
	else
	{
		temp = ReadReg(0x0a);
		if((temp & 0x02) == 0x02)
			return(FM1702_PARITYERR);
		else if((temp & 0x04) == 0x04)
			return(FM1702_FRAMINGERR);
		else if((temp & 0x08) == 0x08)
			return(FM1702_CRCERR);
		else
			return(FM1702_WRITEERR);
	}
}

/*********************************************************************
 * @fn      SerialApp_Init
 *
 * @brief   This is called during OSAL tasks' initialization.
 *
 * @param   task_id - the Task ID assigned by OSAL.
 *
 * @return  none
 */
void SerialApp_Init( uint8 task_id )
{
  halUARTCfg_t uartConfig;
  SerialApp_TaskID = task_id;
  afRegister( (endPointDesc_t *)&SerialApp_epDesc );
  RegisterForKeys( task_id );
  HalUARTInitSPI();
  GPIOInit(); 
  InitFM1702SL();
  ledInit();
  
  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = SERIAL_APP_BAUD;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = SERIAL_APP_THRESH; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = SERIAL_APP_RX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = SERIAL_APP_TX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = SERIAL_APP_IDLE;   // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = SerialApp_CallBack;
  HalUARTOpen (SERIAL_APP_PORT, &uartConfig);
  
  UART0_Format.Header   = 0x40;
  UART0_Format.Len      = 0x10;
  UART0_Format.NodeSeq  = 0x01;
  UART0_Format.NodeID   = RFID_1356; //FM1702SL������
  
  
  UART0_Format1.Header   = 0x40;
  UART0_Format1.Len      = 0x10;
  UART0_Format1.NodeSeq  = 0x01;
  UART0_Format1.NodeID   = RFID_1356;//FM1702SL������
  
  Rsp.Header   = 0x40;
  Rsp.Len      = 0x0C;
  Rsp.NodeSeq  = 0x01;
  Rsp.NodeID   = RFID_1356;//FM1702SL������ 
  
  SerialApp_TxAddr.addrMode =(afAddrMode_t)Addr16Bit;//���͵�ַ��ʼ��
  SerialApp_TxAddr.endPoint = SERIALAPP_ENDPOINT;
  SerialApp_TxAddr.addr.shortAddr = 0x0000;
  TXPOWER = 0xf5;
}

/*********************************************************************
 * @fn      SerialApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events   - Bit map of events to process.
 *
 * @return  Event flags of all unprocessed events.
 */
UINT16 SerialApp_ProcessEvent( uint8 task_id, UINT16 events )
{
  uint8 num1=0,num2=0;
  
  (void)task_id;  // Intentionally unreferenced parameter
  
  if ( events & SYS_EVENT_MSG )
  {
    afIncomingMSGPacket_t *MSGpkt;

    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SerialApp_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {         
      case KEY_CHANGE:
        //SerialApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
        break;

      case AF_INCOMING_MSG_CMD:
        SerialApp_ProcessMSGCmd( MSGpkt );
        break;

      case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
         if(SampleApp_NwkState == DEV_END_DEVICE) //�ж���ǰ�豸����
         {
            
            osal_set_event(SerialApp_TaskID, PERIOD_EVT); //����������Ϣ
            osal_set_event(SerialApp_TaskID, RFID_EVT);
            LED(1,ON);
          }
        break;
      default:
        break;
      }

      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    return ( events ^ SYS_EVENT_MSG );
  }
  
  if ( events & PERIOD_EVT ) //������Ϣ����
  {
    UART0_Format.Command = MSG_PERIOD;
    UART0_Format.Data[0] = 0x00;
    UART0_Format.Data[1] = 0x00; 
    UART0_Format.Data[2] = 0x00;
    UART0_Format.Data[3] = 0x00; 
    UART0_Format.Data[4] = 0x00;
    UART0_Format.Data[5] = 0x00;
    UART0_Format.Data[6] = 0x00;
    UART0_Format.Data[7] = 0x00; 
    UART0_Format.Data[8] = 0x00;
    UART0_Format.Data[9] = 0x00;
    
    num1 = CheckSum(&UART0_Format.Header,UART0_Format.Len);
    UART0_Format.Verify  = num1;
    
    SerialApp_OTAData(&SerialApp_TxAddr, SERIALAPP_CLUSTERID1, &UART0_Format, sizeof(UART_Format));
    osal_start_timerEx(SerialApp_TaskID, PERIOD_EVT, 5000);
    FLASHLED(2);
    return ( events ^ PERIOD_EVT );
  }
  
  if ( events & SERIALAPP_SEND_EVT ) //����RF��Ϣ
  {
    num2 = CheckSum(&UART0_Format1.Header,UART0_Format1.Len);
    UART0_Format1.Verify  = num2;
    
    SerialApp_OTAData(&SerialApp_TxAddr,SERIALAPP_CLUSTERID1, &UART0_Format1, sizeof(UART_Format));
    FLASHLED(4);
                      
    return ( events ^ SERIALAPP_SEND_EVT );
  }
  
  if ( events & RFID_EVT )  
  {
    uint8 state;
    static uint8 hasCardOld = FALSE;
    static uint8 hasCardNow = FALSE;
    hasCardOld = hasCardNow; 
    hasCardNow = Request(RF_CMD_REQUEST_STD);
    if(hasCardOld != hasCardNow) //����1˵����⵽��
    {
      state = ReadCardNum(); //�����ţ��洢��CardNo��������
      if(!state) //������ȷ  
      {
        Buzzer(ON); 
        UART0_Format1.Command = SEND;
        UART0_Format1.Data[0] = CardNo[0]; //d2345678
        UART0_Format1.Data[1] = CardNo[1]; 
        UART0_Format1.Data[2] = CardNo[2];
        UART0_Format1.Data[3] = CardNo[3]; 
        MIF_READ(RevBuffer, block_numset(1));  //�������ݿ�0�����ݷ���RevBuffer����
        UART0_Format1.Data[4] = RevBuffer[2];   //���ڿ�0�еĵ�һ���ֽ�Ϊ���ϱ�Ÿ�λ
        UART0_Format1.Data[5] = RevBuffer[3];   //���ڿ�0�еĵڶ����ֽ�Ϊ���ϱ�ŵ�λ
        UART0_Format1.Data[6] = RevBuffer[4];   //���ڿ�0�еĵ������ֽ�Ϊ��ݸ�λ
        UART0_Format1.Data[7] = RevBuffer[5];   //���ڿ�0�еĵ��ĸ��ֽ�Ϊ��ݵ�λ
        UART0_Format1.Data[8] = RevBuffer[6];   //���ڿ�0�еĵ�����ֽ�Ϊ�·�
        UART0_Format1.Data[9] = RevBuffer[7];   //���ڿ�0�еĵ������ֽ�Ϊ��
        
        osal_set_event(SerialApp_TaskID, SERIALAPP_SEND_EVT);
        MicroWait(50000);
        MicroWait(50000);
        Buzzer(OFF);
      }
      else
      {
        CardNo[0] = CardNo[1] = CardNo[2] = CardNo[3] = 0;
      }
    }
   osal_start_timerEx(SerialApp_TaskID, RFID_EVT, 200);
    return ( events ^ RFID_EVT );
  }

  return ( 0 );  // Discard unknown events.
}

/*********************************************************************
 * @fn      SerialApp_ProcessMSGCmd
 *
 * @brief   Data message processor callback. This function processes
 *          any incoming data - probably from other devices. Based
 *          on the cluster ID, perform the intended action.
 *
 * @param   pkt - pointer to the incoming message packet
 *
 * @return  TRUE if the 'pkt' parameter is being used and will be freed later,
 *          FALSE otherwise.
 */
void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt )  //������յ���RF��Ϣ
{
  static UART_Format_End6 *receiveData;
  static uint8  secNR;
  uint8  status;
  uint8 num1=0,num=0;
   
  Rsp.Command  = MSG_RSP;

  switch ( pkt->clusterId )
  {
   case SERIALAPP_CLUSTERID1:  //�������������������    
     receiveData = (UART_Format_End6 *)(pkt->cmd.Data);
     
     num = CheckSum((uint8*)receiveData,receiveData->Len);
     
      if((receiveData->Header==0x40)&&(receiveData->Verify==num)) //У���ͷ��β
     {
       if(receiveData->NodeID == RFID_1356) //�յ����ݺ�ͨ�����ڷ��͸�RFIDģ��
       {
        /**
 	 do{	
	     status = Request(RF_CMD_REQUEST_STD);
         }while(!status);	*/
         
         status = Request(RF_CMD_REQUEST_STD); 
         FLASHLED(6);
         status = ReadCardNum();//�ȴ���ȡ���� 
         if(!status)
         {
           if(receiveData->Command == STORAGE)
           {         
             secNR  = block_numset(1);            //�������ݿ�1
             MIF_READ(RevBuffer, secNR);          //�������ݿ�0�����ݷ���RevBuffer����
             RevBuffer[2]=receiveData->Data[0];                 //���ڿ�0�еĵ�һ���ֽ�Ϊ���ϱ�Ÿ�λ
             RevBuffer[3]=receiveData->Data[1];                 //���ڿ�0�еĵڶ����ֽ�Ϊ���ϱ�ŵ�λ
             RevBuffer[4]=receiveData->Data[2];                 //���ڿ�0�еĵ������ֽ�Ϊ�����ݸ�λ
             RevBuffer[5]=receiveData->Data[3];                 //���ڿ�0�еĵ��ĸ��ֽ�Ϊ�����ݵ�λ
             RevBuffer[6]=receiveData->Data[4];                 //���ڿ�0�еĵ�����ֽ�Ϊ�·�
             RevBuffer[7]=receiveData->Data[5];                 //���ڿ�0�еĵ������ֽ�Ϊ��
  
              /////д���ݿ�Ĳ���////////////
             secNR  = block_numset(1);      //�������ݿ�1
             MIF_Write(RevBuffer, secNR); //д��
             
           }
           else if(receiveData->Command == OUTBOUND)
           {
             secNR  = block_numset(1);            //�������ݿ�1
             MIF_READ(RevBuffer,secNR);  //�������ݿ�0�����ݷ���RevBuffer����
             RevBuffer[2]=receiveData->Data[0];                 //���ڿ�0�еĵ�һ���ֽ�Ϊ���ϱ�Ÿ�λ
             RevBuffer[3]=receiveData->Data[1];                 //���ڿ�0�еĵڶ����ֽ�Ϊ���ϱ�ŵ�λ
             RevBuffer[4]=receiveData->Data[2];                 //���ڿ�0�еĵ������ֽ�Ϊ�����ݸ�λ
             RevBuffer[5]=receiveData->Data[3];                 //���ڿ�0�еĵ��ĸ��ֽ�Ϊ�����ݵ�λ
             RevBuffer[6]=receiveData->Data[4];                 //���ڿ�0�еĵ�����ֽ�Ϊ�·�
             RevBuffer[7]=receiveData->Data[5];                 //���ڿ�0�еĵ������ֽ�Ϊ��
  
              /////д���ݿ�Ĳ���////////////
             secNR  = block_numset(1);      //�������ݿ�1
             MIF_Write(RevBuffer, secNR); //д��
           }           
     

           MIF_READ(RevBuffer1, block_numset(1));  //�������ݿ�0�����ݷ���RevBuffer����
             Rsp.Data[0]  =RevBuffer1[2];
             Rsp.Data[1]  =RevBuffer1[3];
             Rsp.Data[2]  =RevBuffer1[4];
             Rsp.Data[3]  =RevBuffer1[5];
             Rsp.Data[4]  =RevBuffer1[6];
             Rsp.Data[5]  =RevBuffer1[7];
               
             num1 = CheckSum(&Rsp.Header,Rsp.Len);
             Rsp.Verify  = num1;
           SerialApp_OTAData(&SerialApp_TxAddr, SERIALAPP_CLUSTERID1, &Rsp, 12);
           FLASHLED(5);
         }
       }
     }
    break;

  case SERIALAPP_CLUSTERID2:
    break;

    default:
      break;
  }
}

/*********************************************************************
 */

void SerialApp_OTAData(afAddrType_t *txaddr, uint8 cID, void *p, uint8 len) //���ͺ���
{
  if (afStatus_SUCCESS != AF_DataRequest(txaddr, //���͵�ַ
                                           (endPointDesc_t *)&SerialApp_epDesc, //endpoint����
                                            cID, //clusterID
                                            len, p, //�������ݰ��ĳ��Ⱥ͵�ַ
                                            &SerialApp_MsgID, 0, AF_DEFAULT_RADIUS))
  {
  }
  else
  {
    HalLedBlink(HAL_LED_1,1,50,200);
  }
}

/*********************************************************************
 * @fn      SerialApp_CallBack
 *
 * @brief   Send data OTA.
 *
 * @param   port - UART port.
 * @param   event - the UART port event flag.
 *
 * @return  none
 */
static void SerialApp_CallBack(uint8 port, uint8 event)
{
  (void)port;

  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) && !SerialApp_TxLen) //���ڽ��յ����ݰ�
  {
    SerialApp_TxLen = HalUARTRead(SERIAL_APP_PORT, SerialApp_TxBuf, SERIAL_APP_TX_MAX); //���������ݶ���buf
    SerialApp_TxLen = 0;  
  }
}

/*****************************************************************************************
*�������� ���ۼ�У��ͺ���				                                 
*��ڲ��� ��*data - ���ݰ���ַ
            len   - ���ݰ�����
*�� �� ֵ ��sum   - �ۼӺ�                                                                                
*˵    �� �������ݰ������ۼӺ�У��                                                             
*****************************************************************************************/

uint8 CheckSum(uint8 *data,uint8 len)
{
  uint8 i,sum=0;
  for(i=0;i<(len-1);i++)
  {
    sum+=data[i];
  }
  return sum;
}
                     

/*********************************************************************
*********************************************************************/
