/**************************************Copyright(c)****************************************
* ����������������            �ൺɽ���ǻ���Ϣ�Ƽ����޹�˾ ���������� ����                *
* ��������������������������                          ������������������������������������*
* ������������������                  www.iotsk.com ������������������                  ��*
* ������������������������������������������������������                          ��������*
*----------------------------------------File Info----------------------------------------*
*�� �� �� ��Dioxidecar.c������������                                                         *
*�޸����� ��2014.05.15  ����������������������������                                      *
*�� �� �� ��V1.0��������������������������������                                          *
*��    �� ��CO2Ũ�ȼ��ģ����ն˽ڵ���򣬰Ѳ�õ�Ũ��ֵ������͸�Э����                    *
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
#include "Dioxidecar.h"
#include <string.h>
#include "hal.h"
#include "stdio.h"


/********************************************************************************************
 *                                LOCAL VARIABLES
 *******************************************************************************************/

extern char s_write_byte(unsigned char value);
extern char s_read_byte(unsigned char ack);
extern void s_transstart(void);
extern void s_connectionreset(void);
extern char s_measure( unsigned char *p_checksum, unsigned char mode);
void initIO(void);
extern void th_read(int *t,int *h );
extern void LED(uint8 led,uint8 operation);
extern void ledInit(void);
extern void FLASHLED(uint8 led);
/******************************************************************************************
 *                                        MACROS
 *****************************************************************************************/
#define noACK 0
#define ACK   1

#define STATUS_REG_W 0x06
#define STATUS_REG_R 0x07
#define MEASURE_TEMP 0x03
#define MEASURE_HUMI 0x05
#define RESET        0x1e

#define SCL          P1_0     //SHT10ʱ��
#define SDA          P1_1     //SHT10������

#define ON  1
#define OFF 0

/******************************************************************************************
 *                                       CONSTANTS
 ******************************************************************************************/

#if !defined( SERIAL_APP_PORT )
#define SERIAL_APP_PORT  0
#endif

#if !defined( SERIAL_APP_BAUD )
#define SERIAL_APP_BAUD  HAL_UART_BR_115200
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

/*******************************************************************************************
 *                                     TYPEDEFS
 ******************************************************************************************/

/*******************************************************************************************
 *                                GLOBAL VARIABLES
 ******************************************************************************************/

uint8 SerialApp_TaskID;    // Task ID for internal task/event processing.
devStates_t  SampleApp_NwkState;
static UART_Format UART0_Format;
static UART_Format_End4 UART0_Format1;

/*******************************************************************************************
 *                               EXTERNAL VARIABLES
 ******************************************************************************************/

/*******************************************************************************************
 *                               EXTERNAL FUNCTIONS
 ******************************************************************************************/

/*******************************************************************************************
 *                                LOCAL VARIABLES
 ******************************************************************************************/

static uint8 SerialApp_MsgID;
static afAddrType_t SerialApp_TxAddr;
static uint8 SerialApp_TxBuf[SERIAL_APP_TX_MAX];
static uint8 SerialApp_TxLen;

/******************************************************************************************
 *                                LOCAL FUNCTIONS
 ******************************************************************************************/

static void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt );
void SerialApp_OTAData(afAddrType_t *txaddr,uint8 ID,void *p,uint8 len);
static void SerialApp_CallBack(uint8 port, uint8 event);
static uint8 CheckSum(uint8 *data,uint8 len);

/*****************************************************************************************
*�������� �����ڳ�ʼ������					                          
*��ڲ��� ��task_id - OSAL���������ID��		                          
*�� �� ֵ ����							                          
*˵    �� ����OSAL��ʼ����ʱ�򱻵���                             
*****************************************************************************************/

void SerialApp_Init( uint8 task_id )
{
  halUARTCfg_t uartConfig;

  SerialApp_TaskID = task_id;

  afRegister( (endPointDesc_t *)&SerialApp_epDesc );

  RegisterForKeys( task_id );
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
  UART0_Format.NodeID   = Dioxidecar;//������̼Ũ��;//
  
  UART0_Format1.Header   = 0x40;
  UART0_Format1.Len      = 0x0a;
  UART0_Format1.NodeSeq  = 0x01;
  UART0_Format1.NodeID   = Dioxidecar;//������̼Ũ�� ;//
  
  SerialApp_TxAddr.addrMode =(afAddrMode_t)Addr16Bit;//���͵�ַ��ʼ��
  SerialApp_TxAddr.endPoint = SERIALAPP_ENDPOINT;
  SerialApp_TxAddr.addr.shortAddr = 0x0000;
  TXPOWER = 0xf5;
}

/*****************************************************************************************
*�������� ���û������¼���������				                          
*��ڲ��� ��task_id - OSAL������¼�ID��                                      
*           events  - �¼�                                      
*�� �� ֵ ���¼���־		                          
*˵    �� ��                                                                              
*****************************************************************************************/

UINT16 SerialApp_ProcessEvent( uint8 task_id, UINT16 events )
{ 
  int adc0_value[2],temp1,temp2;
  uint8 num=0;
  int shiwei=0,cdisplay=0;
  int ppm=0;
  double ppmdou=0;
  
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
            osal_set_event(SerialApp_TaskID, Dioxidecar_READ_EVT); //�������
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
    
    num = CheckSum(&UART0_Format.Header,UART0_Format.Len);
    UART0_Format.Verify  = num;

    
    
    SerialApp_OTAData(&SerialApp_TxAddr, SERIALAPP_CLUSTERID1, &UART0_Format, sizeof(UART_Format));
    osal_start_timerEx(SerialApp_TaskID, PERIOD_EVT, 5000);
    FLASHLED(2);
    return ( events ^ PERIOD_EVT );
  }
  
  if ( events & SERIALAPP_SEND_EVT ) //����RF��Ϣ
  {
    num = CheckSum(&UART0_Format1.Header,UART0_Format1.Len);
    UART0_Format1.Verify  = num;

    
    SerialApp_OTAData(&SerialApp_TxAddr,SERIALAPP_CLUSTERID1, &UART0_Format1, 10);
    FLASHLED(4);

    return ( events ^ SERIALAPP_SEND_EVT );
  }

  if ( events & Dioxidecar_READ_EVT )  //�鿴������״̬
  { 
    
    //ͨ��0
    ADC_ENABLE_CHANNEL(ADC_AIN0);                          // ʹ��AIN0ΪADC����
    
    ADC_SINGLE_CONVERSION(ADC_REF_AVDD | ADC_12_BIT | ADC_AIN0);

    ADC_SAMPLE_SINGLE();                                   // ����һ����һת��

    while(!ADC_SAMPLE_READY());                            // �ȴ�ת�����

    ADC_ENABLE_CHANNEL(ADC_AIN0);                          // ��ֹAIN0

    temp1 = ADCH; 
    temp2 = ADCL>>4;
 
    ADCH=0;
    ADCL=0;
     
    shiwei=temp1*256+temp2;
      
    cdisplay=(int)(shiwei/10);
    
     if(cdisplay>1072)
	   {  
	   	   ppm=400;
	   }
	  if((1049<cdisplay)&&(cdisplay<=1072))
	   {
	   		ppmdou = (500-((cdisplay-1049)*(100/(1072-1049))));
			ppm = (int)ppmdou;

	   }
	  if((1036<cdisplay)&&(cdisplay<=1049))
	   {
	   	   ppmdou = (600-((cdisplay-1036)*(100/(1049-1036))));
	   	   //ppmdou = (500-(ppmd-1049)*(100/(1072-1049)));
			ppm = (int)ppmdou;
	   
	   }
	   
	   if((1023<cdisplay)&&(cdisplay<=1036))
	   {
	   	   ppmdou = (700-(cdisplay-1023)*(100/(1036-1023)));
		   ppm = (int)ppmdou;
	   } 
	  if((1016<cdisplay)&&(cdisplay<=1023))
	   {
	   	   ppmdou = (800-((cdisplay-1016)*(100/(1023-1016))));
		   ppm = (int)ppmdou;
	   }
	   
	   if((1006<cdisplay)&&(cdisplay<=1016))
	   {
	   	   ppmdou = (900-((cdisplay-1006)*(100/(1016-1006))));
	   	   ppm = (int)ppmdou;
	   } 
	if((1000<cdisplay)&&(cdisplay<=1006))
	   {
	   	  ppmdou = (1000-((cdisplay-1000)*(100/(1016-1000))));
	   	  ppm = (int)ppmdou;
	   } 
	   if((966<cdisplay)&&(cdisplay<=1000))
	   {
	   	  ppmdou = (2000-((cdisplay-966)*(1000/(1000-966))));
	   	  ppm = (int)ppmdou;
	   } 
	   if((947<cdisplay)&&(cdisplay<=966))  //2000-3000
	   {
	   	  ppmdou = (3000-((cdisplay-947)*(1000/(966-947))));
	   	  ppm = (int)ppmdou;
	   } 
	  if((924<cdisplay)&&(cdisplay<=947))//3000-4000
	   {
	   	  ppmdou = (4000-((cdisplay-924)*(1000/(947-924))));
		  ppm = (int)ppmdou;
	   } 
	   if((907<cdisplay)&&(cdisplay<=924))//4000-6000
	   {
	   	  ppmdou = (6000-(cdisplay-907)*(2000/(924-907)));
	   	  ppm = (int)ppmdou;
	   }
	   if(cdisplay<874)//8000-10000
	   {
	   	  ppm = 9999;
	   } 
    
     adc0_value[0]=ppm/100;
     adc0_value[1]=ppm%100;
    
     UART0_Format1.Command = 0x01;
    
                                       
     UART0_Format1.Data[0] = adc0_value[0];
     UART0_Format1.Data[1] = adc0_value[1];
 
     UART0_Format1.Data[2] = 0X00;
     UART0_Format1.Data[3] = 0X00;
      

     osal_set_event(SerialApp_TaskID, SERIALAPP_SEND_EVT);

     osal_start_timerEx(SerialApp_TaskID, Dioxidecar_READ_EVT, 500);
    
    return ( events ^ Dioxidecar_READ_EVT );
  }

  return ( 0 );  // Discard unknown events.
}

/*****************************************************************************************
*�������� ����Ϣ��������				                                 
*��ڲ��� ��pkt   - ָ����յ���������Ϣ���ݰ���ָ��                                 
*�� �� ֵ ��TRUE  - ���ָ�뱻Ӧ�ò��ͷ�            
*           FALSE - ����	                                                          
*˵    �� �������յ���������Ϣ��������Ϣͨ�����ڷ��͸�����                               
*****************************************************************************************/

void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt )  //�������յ���RF��Ϣ
{ 
  uint8 num=0;
  static UART_Format *receiveData;
  switch ( pkt->clusterId )
  {
   case SERIALAPP_CLUSTERID1:  //��������������������    
     receiveData = (UART_Format *)(pkt->cmd.Data);
     HalLedBlink(HAL_LED_1,1,50,200);
     
     num = CheckSum((uint8*)receiveData,receiveData->Len);
     
   if((receiveData->Header==0x40)&&(receiveData->Verify==num)) //У���ͷ��β

     
     
     {
      
     }
    break;

  case SERIALAPP_CLUSTERID2:
    break;

    default:
      break;
  }
}

/*****************************************************************************************
*�������� ��������Ϣ���ͺ���				                                 
*��ڲ��� ��*txaddr - ���͵�ַ
*           cID     - clusterID
*           *p      - �������ݰ��ĵ�ַ
*           len     - �������ݰ��ĳ���
*�� �� ֵ ����                                                                                
*˵    �� ������õ����ݰ���ͨ�����߷��ͳ�ȥ                                                            
*****************************************************************************************/


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

/*****************************************************************************************
*�������� �����ڻص�����				                                 
*��ڲ��� ��port  -�˿ں�
*           event -�¼���
*�� �� ֵ ����                                                                              
*˵    �� ���Ѵ�����Ϣͨ�����߷��ͳ�ȥ                                                             
*****************************************************************************************/

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

