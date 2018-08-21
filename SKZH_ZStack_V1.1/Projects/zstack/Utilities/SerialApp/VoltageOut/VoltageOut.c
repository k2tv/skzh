/**************************************Copyright(c)****************************************
* ����������������            �ൺɽ���ǻ���Ϣ�Ƽ����޹�˾ ���������� ����                *
* ��������������������������                          ������������������������������������*
* ������������������                  www.iotsk.com ������������������                  ��*
* ������������������������������������������������������                          ��������*
*----------------------------------------File Info----------------------------------------*
*�� �� �� ��VoltageOut.c������������                                                      *
*�޸����� ��2013.11.21  ����������������������������                                      *
*�� �� �� ��V1.0����������������������������������                                        *
*��    �� ����ѹ���ģ����Ƴ���                                                          *
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
#include "Relay.h"
#include "hal_adc.h"
#include "ioCC2530.h" 
#include "hal_mcu.h"

#define SCL          P1_0       //IICʱ����
#define SDA          P1_1       //IIC������

#define ON           0x01       //��ѹ���ģ�����״̬
#define OFF          0x00

//����IO������ƺ���
#define IO_DIR_PORT_PIN(port, pin, dir)  \
   do {                                  \
      if (dir == IO_OUT)                 \
         P##port##DIR |= (0x01<<(pin));  \
      else                               \
         P##port##DIR &= ~(0x01<<(pin)); \
   }while(0)


#define IO_IN   0           //����
#define IO_OUT  1           //���

uint8 ack;	            //Ӧ���־λ

/******************************************************************************************
 *                                       MACROS
 *****************************************************************************************/

/******************************************************************************************
 *                                      CONSTANTS
 *****************************************************************************************/

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

/******************************************************************************************
 *                                      TYPEDEFS
 *****************************************************************************************/

/******************************************************************************************
 *                                  GLOBAL VARIABLES
 *****************************************************************************************/
static uint16 Vol1;
static uint16 Vol2;
static uint16 Vol3;
static uint16 Vol4;
uint8 SerialApp_TaskID;    // Task ID for internal task/event processing.
devStates_t  SampleApp_NwkState;
static UART_Format UART0_Format;



/*******************************************************************************************
 *                                 EXTERNAL VARIABLES  
 ******************************************************************************************/

/*******************************************************************************************
 *                                 EXTERNAL FUNCTIONS  
 ******************************************************************************************/

/*******************************************************************************************
 *                                  LOCAL VARIABLES    
 *******************************************************************************************/

static uint8 SerialApp_MsgID;
static afAddrType_t SerialApp_TxAddr;
static uint8 SerialApp_TxBuf[SERIAL_APP_TX_MAX];
static uint8 SerialApp_TxLen;
/********************************************************************************************
 *                                   LOCAL FUNCTIONS   
 *******************************************************************************************/

static void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt );
void SerialApp_OTAData(afAddrType_t *txaddr,uint8 ID,void *p,uint8 len);

static void SerialApp_CallBack(uint8 port, uint8 event);
static uint8 CheckSum(uint8 *data,uint8 len);
static uint8 VolOutCheck(uint8 sla,uint8 suba,uint8 *s);
extern void LED(uint8 led,uint8 operation);
extern void ledInit(void);
extern void FLASHLED(uint8 led);
extern uint8 IRcvStr(uint8 sla,uint8 suba,uint8 *s,uint8 no); 
extern uint8 VoltageOutput(uint16 Voltage,uint8 Port);
extern void QWait(void);
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
  
 
  ledInit();
  afRegister( (endPointDesc_t *)&SerialApp_epDesc );

  RegisterForKeys( task_id );

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
  
  
  //��������ʼ��
  UART0_Format.Header   = '@';
  UART0_Format.Len      = 0x10;
  UART0_Format.NodeSeq  = 0x01;
  UART0_Format.NodeID   = VoltageOut;
  
  VoltageOutput(0x00ff,0xd0);
  VoltageOutput(0x00ff,0xd2);
  VoltageOutput(0x00ff,0xd4);
  VoltageOutput(0x00ff,0xd6);
  
  
  
  SerialApp_TxAddr.addrMode =(afAddrMode_t)Addr16Bit;//���͵�ַ��ʼ��
  SerialApp_TxAddr.endPoint = SERIALAPP_ENDPOINT;
  SerialApp_TxAddr.addr.shortAddr = 0x0000;
  TXPOWER = 0xf5;
}

/*****************************************************************************************
*�������� ���û������¼�������				                          
*��ڲ��� ��task_id - OSAL������¼�ID��                                      
*           events  - �¼�                                      
*�� �� ֵ ���¼���־		                          
*˵    �� ��                                                                              
*****************************************************************************************/

UINT16 SerialApp_ProcessEvent( uint8 task_id, UINT16 events )
{  
  uint8 num=0;
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
        FLASHLED(3);
        break;

      case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if(SampleApp_NwkState == DEV_END_DEVICE) //�ж���ǰ�豸����
          {
            
            
            osal_set_event(SerialApp_TaskID, PERIOD_EVT); //����������Ϣ
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
    SerialApp_OTAData(&SerialApp_TxAddr,SERIALAPP_CLUSTERID1, &UART0_Format, sizeof(UART_Format));
    
    return ( events ^ SERIALAPP_SEND_EVT );
  }

  return ( 0 );  // Discard unknown events.
}

/*****************************************************************************************
*�������� ����Ϣ������				                                 
*��ڲ��� ��pkt   - ָ����յ���������Ϣ���ݰ���ָ��                                 
*�� �� ֵ ��TRUE  - ���ָ�뱻Ӧ�ò��ͷ�            
*           FALSE - ����	                                                          
*˵    �� �������յ���������Ϣ��������Ϣͨ�����ڷ��͸�����                               
*****************************************************************************************/

void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt )  //������յ���RF��Ϣ
{ 
  //int i=0;
  //PCA9554SRelayInit();
  uint8 num=0;
  uint8 VoltageVal[2];
  
  static UART_Format_Control9 *receiveData;
  static UART_Format_End9 Rsp;
  
  Rsp.Header   = '@';
  Rsp.Len      = 0x0f;
  Rsp.NodeSeq  = 0x01;
  Rsp.NodeID   = VoltageOut;
  Rsp.Command  = MSG_RSP;
  
  
  
  switch ( pkt->clusterId )
  {
   case SERIALAPP_CLUSTERID1:  //�������������������    
     receiveData = (UART_Format_Control9 *)(pkt->cmd.Data);
     
     
     num = CheckSum((uint8*)receiveData,receiveData->Len);
     if((receiveData->Header=='@')&&(receiveData->Verify=num)) //У���ͷ��β
     {
       if(receiveData->NodeID == VoltageOut) //ID�ж�
       {
         //��8λ�ĸ�λ�͵�λָ��ϲ���16λָ��
         Vol1 = ((uint16)receiveData->Command[1])<<8  + (uint16)receiveData->Command[2];
         Vol2 = ((uint16)receiveData->Command[3])<<8  + (uint16)receiveData->Command[4];
         Vol3 = ((uint16)receiveData->Command[5])<<8  + (uint16)receiveData->Command[6];
         Vol4 = ((uint16)receiveData->Command[7])<<8  + (uint16)receiveData->Command[8];
         
         if((receiveData->Command[0] & 0x48)==0x48)
         { 
                                            
            VoltageOutput(Vol1,0xd0);//�����·�ָ����Ƶ�һ·��ѹ
            if(VolOutCheck(0x98,0xd0,&receiveData->Command[0]+1))
            {
               VoltageOutput(Vol1,0xd0);//�ٴθ����·�ָ����Ƶ�һ·��ѹ 
            }
            
         }
         
         QWait();
         
         if((receiveData->Command[0] & 0x48)==0x48)
         { 
                                            
            VoltageOutput(Vol1,0xd0);//�����·�ָ����Ƶ�һ·��ѹ
            if(VolOutCheck(0x98,0xd0,&receiveData->Command[0]+1))
            {
               VoltageOutput(Vol1,0xd0);//�ٴθ����·�ָ����Ƶ�һ·��ѹ 
            }
            
         }
         
         QWait();
         
         if((receiveData->Command[0] & 0x44)==0x44)
         { 
            
            VoltageOutput(Vol2,0xd2);//�����·�ָ����Ƶڶ�·��ѹ
            
            if(VolOutCheck(0x98,0xd2,&receiveData->Command[0]+3))
            {
               VoltageOutput(Vol2,0xd2);//�ٴθ����·�ָ����Ƶڶ�·��ѹ 
            }
            
         }
         
         QWait();
         
                  if((receiveData->Command[0] & 0x44)==0x44)
         { 
            
            VoltageOutput(Vol2,0xd2);//�����·�ָ����Ƶڶ�·��ѹ
            
            if(VolOutCheck(0x98,0xd2,&receiveData->Command[0]+3))
            {
               VoltageOutput(Vol2,0xd2);//�ٴθ����·�ָ����Ƶڶ�·��ѹ 
            }
            
         }
         
         QWait();
         
         if((receiveData->Command[0] & 0x42)==0x42)
         { 
            
            VoltageOutput(Vol3,0xd4);//�����·�ָ����Ƶ���·��ѹ
            
            if(VolOutCheck(0x98,0xd4,&receiveData->Command[0]+5))
            {
               VoltageOutput(Vol3,0xd4);//�ٴθ����·�ָ����Ƶ���·��ѹ 
            }
            
         }
      
         QWait();
         
         if((receiveData->Command[0] & 0x42)==0x42)
         { 
            
            VoltageOutput(Vol3,0xd4);//�����·�ָ����Ƶ���·��ѹ
            
            if(VolOutCheck(0x98,0xd4,&receiveData->Command[0]+5))
            {
               VoltageOutput(Vol3,0xd4);//�ٴθ����·�ָ����Ƶ���·��ѹ 
            }
            
         }
      
         QWait();
         
         if((receiveData->Command[0] & 0x41)==0x41)
         { 
            
            VoltageOutput(Vol4,0xd6);//�����·�ָ����Ƶ���·��ѹ
            
            if(VolOutCheck(0x98,0xd6,&receiveData->Command[0]+7))
            {
               VoltageOutput(Vol4,0xd6);//�ٴθ����·�ָ����Ƶ���·��ѹ 
            }          
                         
           
         } 
         
         QWait();
         if((receiveData->Command[0] & 0x41)==0x41)
         { 
            
            VoltageOutput(Vol4,0xd6);//�����·�ָ����Ƶ���·��ѹ
            
            if(VolOutCheck(0x98,0xd6,&receiveData->Command[0]+7))
            {
               VoltageOutput(Vol4,0xd6);//�ٴθ����·�ָ����Ƶ���·��ѹ 
            }          
                         
           
         } 
         
         QWait();
         if(receiveData->Command[0] == 0x0C)
         {
           
                                  //��ѯָ������κδ���ֻ���ص�ǰ״̬
           
         }
           
           Rsp.Data[0] = 0x4f; 
           
           IRcvStr(0x98,0xd0,VoltageVal,2);//��ȡ��һ·��ǰ��ѹֵ
           Rsp.Data[1] = VoltageVal[0];
           Rsp.Data[2] = VoltageVal[1];
           
           IRcvStr(0x98,0xd2,VoltageVal,2);//��ȡ�ڶ�·��ǰ��ѹֵ
           Rsp.Data[3] = VoltageVal[0];
           Rsp.Data[4] = VoltageVal[1];
           
           
           IRcvStr(0x98,0xd4,VoltageVal,2);//��ȡ����·��ǰ��ѹֵ
           Rsp.Data[5] = VoltageVal[0];
           Rsp.Data[6] = VoltageVal[1];
           
          
           IRcvStr(0x98,0xd6,VoltageVal,2);//��ȡ����·��ǰ��ѹֵ
           Rsp.Data[7] = VoltageVal[0];
           Rsp.Data[8] = VoltageVal[1];
         
         
         num = CheckSum(&Rsp.Header,Rsp.Len);
         Rsp.Verify  = num;
         SerialApp_OTAData(&SerialApp_TxAddr, SERIALAPP_CLUSTERID1, &Rsp, sizeof(UART_Format_End9));
         FLASHLED(4);
       }
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


/******************************************************************************
 * �������ƣ�VolOutCheck
 * ������������ȡ��ǰ�ĵ�ѹ���ĳһ·�ĵ�ѹֵ����ͬ��������ĵ�ѹֵ���бȽ�          
 * ��    ����sla  - ��������ַ
 *           suba - �ӵ�ַ
 *           *s   - ָ�������ֵ�ָ��
 * �� �� ֵ��a    ����ǰ��ѹֵ��ָ���ͬ����1�����򷵻�0
 *****************************************************************************/
uint8 VolOutCheck(uint8 sla,uint8 suba,uint8 *s)
{   
  
    int a=0;
    uint8 voltage[2];
    
    IRcvStr( sla, suba,voltage,2);
    
    if(( *s != voltage[0] )&&( *(s+1) != voltage[1] ))
    {
      a=1;
    }
    
    return(a);
}

