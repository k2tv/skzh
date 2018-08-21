/**************************************Copyright(c)****************************************
* ����������������            �ൺɽ���ǻ���Ϣ�Ƽ����޹�˾ ���������� ����                *
* ��������������������������                          ������������������������������������*
* ������������������                  www.iotsk.com ������������������                  ��*
* ������������������������������������������������������                          ��������*
*----------------------------------------File Info----------------------------------------*
*�� �� �� ��Motor.c������������                                                           *
*�޸����� ��2013.11.21  ����������������������������                                      *
*�� �� �� ��V1.0��������������������������������                                          *
*��    �� �����ģ����ն˽ڵ����                                                        *
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
#include "motor.h"

/*****************************************************************************************
 *                                        MACROS
 ****************************************************************************************/

/*****************************************************************************************
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
 *                                       TYPEDEFS
 ******************************************************************************************/

/*******************************************************************************************
 *                                   GLOBAL VARIABLES
 *******************************************************************************************/

uint8 SerialApp_TaskID;    // Task ID for internal task/event processing.
devStates_t  SampleApp_NwkState;
static UART_Format UART0_Format;



/*******************************************************************************************
 *                                  EXTERNAL VARIABLES
 *******************************************************************************************/

/********************************************************************************************
 *                                  EXTERNAL FUNCTIONS
 *******************************************************************************************/

/********************************************************************************************
 *                                    LOCAL VARIABLES
 ********************************************************************************************/

static uint8 SerialApp_MsgID;
static afAddrType_t SerialApp_TxAddr;
static uint8 SerialApp_TxBuf[SERIAL_APP_TX_MAX];
static uint8 SerialApp_TxLen;

/*******************************************************************************************
 *                                    LOCAL FUNCTIONS
 *******************************************************************************************/

static void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt );
void SerialApp_OTAData(afAddrType_t *txaddr,uint8 ID,void *p,uint8 len);
static void SerialApp_CallBack(uint8 port, uint8 event);
static void Motor_Init(void);
static void LED_Init(void);
extern uint8 CheckSum(uint8 *data,uint8 len);
static void Motor_Control(uint8 state); //�������
static void LED_Control(uint8 option);   //LED����
extern void ledInit(void);
extern void FLASHLED(uint8 FLASHnum);
extern void LED(uint8 led,uint8 operation);

/*****************************************************************************************
*�������� ���������IO�ڳ�ʼ��				                                 
*��ڲ��� ����                                
*�� �� ֵ ����                                                                                
*˵    �� ����                                                            
*****************************************************************************************/

static void Motor_Init(void)
{
  P0DIR |= 0x03;
  LG9110_MF = LOW;
  LG9110_MR = LOW;
}

/*****************************************************************************************
*�������� ��LED����IO�ڳ�ʼ��				                                 
*��ڲ��� ����                                
*�� �� ֵ ����                                                                               
*˵    �� ����                                                             
*****************************************************************************************/

static void LED_Init(void)
{
  P0DIR |= 0xf0;
  LED_1 = HIGH;
  LED_2 = HIGH;
  LED_3 = HIGH;
  LED_4 = HIGH;
}

/*****************************************************************************************
*�������� ��������ƺ���				                                 
*��ڲ��� ��                                
*�� �� ֵ ��                                                                                
*˵    �� ��                                                             
*****************************************************************************************/

static void Motor_Control(uint8 command) //�������
{
  if(command == FRONT)
  {
    LG9110_MF = HIGH;
    LG9110_MR = LOW;
  }
  else if(command == BACK)
  {
    LG9110_MF = LOW;
    LG9110_MR = HIGH;
  }
  else
  {
    LG9110_MF = LOW;
    LG9110_MR = LOW;
  }
}

/*****************************************************************************************
*�������� ��LED���ƺ���				                                 
*��ڲ��� ��                                
*�� �� ֵ ��                                                                                
*˵    �� ��                                                             
*****************************************************************************************/

static void LED_Control(uint8 option)   //LED����
{
  switch(option)
  {
  case LED_1_ON:
    LED_1 = ON;
      break;
  case LED_1_OFF:
    LED_1 = OFF;
    break;
  case LED_2_ON:
    LED_2 = ON;
      break;
  case LED_2_OFF:
    LED_2 = OFF;
    break;
  case LED_3_ON:
    LED_3 = ON;
      break;
  case LED_3_OFF:
    LED_3 = OFF;
    break;
  case LED_4_ON:
    LED_4 = ON;
      break;
  case LED_4_OFF:
    LED_4 = OFF;
    break;
  case LED_ALL_OFF:
    LED_1 = OFF;
    LED_2 = OFF;
    LED_3 = OFF;
    LED_4 = OFF;
    break;
  case LED_ALL_ON:
    LED_1 = ON;
    LED_2 = ON;
    LED_3 = ON;
    LED_4 = ON;
    break;
  default:break;
  }
}



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
  Motor_Init();
  LED_Init();
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
  
  //���ģ����������ʼ��
  
  UART0_Format.Header   = '@';
  UART0_Format.Len      = 0x10;
  UART0_Format.NodeSeq  = 0x01;
  UART0_Format.NodeID   = Motor;
  
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
          if(SampleApp_NwkState == DEV_END_DEVICE)       //�ж���ǰ�豸����
          {
            
            osal_set_event(SerialApp_TaskID, PERIOD_EVT); //����������Ϣ
            LED(1,1);
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
    num = CheckSum(&UART0_Format.Header,UART0_Format.Len);
    UART0_Format.Verify  = num;

    SerialApp_OTAData(&SerialApp_TxAddr, SERIALAPP_CLUSTERID1, &UART0_Format, sizeof(UART_Format));
    osal_start_timerEx(SerialApp_TaskID, PERIOD_EVT, 5000);
    FLASHLED(2);
    return ( events ^ PERIOD_EVT );
  }

  if ( events & SERIALAPP_SEND_EVT )  //����������ͨ��RF��Ϣ����
  { 
    num = CheckSum(&UART0_Format.Header,UART0_Format.Len);
    UART0_Format.Verify  = num;

    SerialApp_OTAData(&SerialApp_TxAddr,SERIALAPP_CLUSTERID1,SerialApp_TxBuf, sizeof(UART_Format));
    FLASHLED(4);
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
  uint8 num1,num2=0;
  static UART_Format_Control *receiveData;  //��
  static UART_Format_End1 Rsp;           //����
  
  //������Ϣ�����
  
  Rsp.Header   = '@';
  Rsp.Len      = 0x07;
  Rsp.NodeSeq  = 0x01;
  Rsp.NodeID   = Motor;
  Rsp.Command  = MSG_RSP;
  Rsp.Data[0]  = 0;
  switch ( pkt->clusterId )
  {
   case SERIALAPP_CLUSTERID1:  //�������������������    
     receiveData = (UART_Format_Control *)(pkt->cmd.Data);
     
     
     num1 = CheckSum((uint8*)receiveData,receiveData->Len);
     
   if((receiveData->Header==0x40)&&(receiveData->Verify==num1)) //У���ͷ��β

     {
       if(receiveData->NodeID == Motor) //IDУ��
       {
         //�����·�����������п���
         if(receiveData->Command == LED_1_ON)
         {
           
           LED_Control(LED_1_ON);
           if( LED_1 != ON )
           {
              LED_Control(LED_1_ON);
           }
           
         }
         else if(receiveData->Command == LED_1_OFF)
         {
           
           LED_Control(LED_1_OFF);
           if( LED_1 != OFF )
           {
              LED_Control(LED_1_OFF);
           }
           
         }
         else if(receiveData->Command == LED_2_ON)
         {
           LED_Control(LED_2_ON);
           if( LED_2 != ON )
           {
              LED_Control(LED_2_ON);
           }
         }
         else if(receiveData->Command == LED_2_OFF)
         {
           LED_Control(LED_2_OFF);
           if( LED_2 != OFF )
           {
              LED_Control(LED_2_OFF);
           }
         }
         else if(receiveData->Command == LED_3_ON)
         {
           LED_Control(LED_3_ON);
           if( LED_3 != ON )
           {
              LED_Control(LED_3_ON);
           }
         }
         else if(receiveData->Command == LED_3_OFF)
         {
           LED_Control(LED_3_OFF);
           if( LED_3 != OFF )
           {
              LED_Control(LED_3_OFF);
           }
         }
         else if(receiveData->Command == LED_4_ON)
         {
           LED_Control(LED_4_ON);
           if( LED_4 != ON )
           {
              LED_Control(LED_4_ON);
           }
         }
         else if(receiveData->Command == LED_4_OFF)
         {
           LED_Control(LED_4_OFF);
           if( LED_4 != OFF )
           {
              LED_Control(LED_4_OFF);
           }
         }
         else if(receiveData->Command == FRONT)
         {
           Motor_Control(FRONT);
           if( (LG9110_MF != HIGH) && (LG9110_MR != LOW) )
           {
              Motor_Control(FRONT);
           }
         }
         else if(receiveData->Command == BACK)
         {
           Motor_Control(BACK);
           if( (LG9110_MF != LOW) && (LG9110_MR != HIGH) )
           {
              Motor_Control(BACK);
           }
         }
         else if(receiveData->Command == HALT)
         {
           Motor_Control(HALT);
           if( (LG9110_MF != LOW) && (LG9110_MR != LOW) )
           {
              Motor_Control(HALT);
           }
         }
         else if(receiveData->Command == LED_ALL_ON)
         {
           LED_Control(LED_ALL_ON);
           if( (LED_1 != ON) && (LED_2 != ON) && (LED_3 != ON) &&(LED_4 != ON))
           {
              LED_Control(LED_ALL_ON);
           }
         }
         else if(receiveData->Command == LED_ALL_OFF)
         {
           LED_Control(LED_ALL_OFF);
           if( (LED_1 != OFF) && (LED_2 != OFF) && (LED_3 != OFF) &&(LED_4 != OFF))
           {
              LED_Control(LED_ALL_OFF);
           }
         }
         else if(receiveData->Command == MSG_RSC)
         { 
           
         } 
          
           //������Ϣ�������
           if (LED_1==0)
           {
              Rsp.Data[0]|=0x80;
           }
           if (LED_2==0)
           {
              Rsp.Data[0]|=0x40;
           }
           if (LED_3==0)
           {
              Rsp.Data[0]|=0x20;
           }
           if (LED_4==0)
           {
              Rsp.Data[0]|=0x10;
           }
           if (LG9110_MF)
           {
              Rsp.Data[0]|=0x08;
           }
           if (LG9110_MR)
           {
              Rsp.Data[0]|=0x04;
           }
         
                                     
         num2 = CheckSum(&Rsp.Header,Rsp.Len);
         Rsp.Verify  = num2;
         SerialApp_OTAData(&SerialApp_TxAddr, SERIALAPP_CLUSTERID1, &Rsp, sizeof(UART_Format_End1));
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
/***********************************************/