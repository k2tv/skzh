/**************************************Copyright(c)****************************************
* 　　　　　　　　            青岛山科智汇信息科技有限公司 　　　　　 　　                *
* 　　　　　　　　　　　　　                          　　　　　　　　　　　　　　　　　　*
* 　　　　　　　　　                  www.iotsk.com 　　　　　　　　　                  　*
* 　　　　　　　　　　　　　　　　　　　　　　　　　　　                          　　　　*
*----------------------------------------File Info----------------------------------------*
*文 件 名 ：Motor.c　　　　　　                                                           *
*修改日期 ：2013.11.21  　　　　　　　　　　　　　　                                      *
*版 本 号 ：V1.0　　　　　　　　　　　　　　　　                                          *
*描    述 ：电机模块的终端节点程序                                                        *
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
static void Motor_Control(uint8 state); //电机控制
static void LED_Control(uint8 option);   //LED控制
extern void ledInit(void);
extern void FLASHLED(uint8 FLASHnum);
extern void LED(uint8 led,uint8 operation);

/*****************************************************************************************
*函数名称 ：电机控制IO口初始化				                                 
*入口参数 ：无                                
*返 回 值 ：无                                                                                
*说    明 ：无                                                            
*****************************************************************************************/

static void Motor_Init(void)
{
  P0DIR |= 0x03;
  LG9110_MF = LOW;
  LG9110_MR = LOW;
}

/*****************************************************************************************
*函数名称 ：LED控制IO口初始化				                                 
*入口参数 ：无                                
*返 回 值 ：无                                                                               
*说    明 ：无                                                             
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
*函数名称 ：电机控制函数				                                 
*入口参数 ：                                
*返 回 值 ：                                                                                
*说    明 ：                                                             
*****************************************************************************************/

static void Motor_Control(uint8 command) //电机控制
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
*函数名称 ：LED控制函数				                                 
*入口参数 ：                                
*返 回 值 ：                                                                                
*说    明 ：                                                             
*****************************************************************************************/

static void LED_Control(uint8 option)   //LED控制
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
*函数名称 ：串口初始化函数					                          
*入口参数 ：task_id - OSAL分配的任务ID号		                          
*返 回 值 ：无							                          
*说    明 ：在OSAL初始化的时候被调用                             
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
  
  //电机模块心跳包初始化
  
  UART0_Format.Header   = '@';
  UART0_Format.Len      = 0x10;
  UART0_Format.NodeSeq  = 0x01;
  UART0_Format.NodeID   = Motor;
  
  SerialApp_TxAddr.addrMode =(afAddrMode_t)Addr16Bit;//发送地址初始化
  SerialApp_TxAddr.endPoint = SERIALAPP_ENDPOINT;
  SerialApp_TxAddr.addr.shortAddr = 0x0000;
  TXPOWER = 0xf5;
}

/*****************************************************************************************
*函数名称 ：用户任务事件处理函数				                          
*入口参数 ：task_id - OSAL分配的事件ID号                                      
*           events  - 事件                                      
*返 回 值 ：事件标志		                          
*说    明 ：                                                                              
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
          if(SampleApp_NwkState == DEV_END_DEVICE)       //判定当前设备类型
          {
            
            osal_set_event(SerialApp_TaskID, PERIOD_EVT); //启动周期消息
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
  
  if ( events & PERIOD_EVT ) //周期消息处理
  {
    UART0_Format.Command = MSG_PERIOD;
    num = CheckSum(&UART0_Format.Header,UART0_Format.Len);
    UART0_Format.Verify  = num;

    SerialApp_OTAData(&SerialApp_TxAddr, SERIALAPP_CLUSTERID1, &UART0_Format, sizeof(UART_Format));
    osal_start_timerEx(SerialApp_TaskID, PERIOD_EVT, 5000);
    FLASHLED(2);
    return ( events ^ PERIOD_EVT );
  }

  if ( events & SERIALAPP_SEND_EVT )  //将串口数据通过RF消息发送
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
*函数名称 ：消息处理函数				                                 
*入口参数 ：pkt   - 指向接收到的无线消息数据包的指针                                 
*返 回 值 ：TRUE  - 如果指针被应用并释放            
*           FALSE - 其他	                                                          
*说    明 ：处理收到的无线消息，并把消息通过串口发送给网关                               
*****************************************************************************************/

void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt )  //处理接收到的RF消息
{ 
  uint8 num1,num2=0;
  static UART_Format_Control *receiveData;  //收
  static UART_Format_End1 Rsp;           //返回
  
  //返回信息包组包
  
  Rsp.Header   = '@';
  Rsp.Len      = 0x07;
  Rsp.NodeSeq  = 0x01;
  Rsp.NodeID   = Motor;
  Rsp.Command  = MSG_RSP;
  Rsp.Data[0]  = 0;
  switch ( pkt->clusterId )
  {
   case SERIALAPP_CLUSTERID1:  //处理各个传感器节数据    
     receiveData = (UART_Format_Control *)(pkt->cmd.Data);
     
     
     num1 = CheckSum((uint8*)receiveData,receiveData->Len);
     
   if((receiveData->Header==0x40)&&(receiveData->Verify==num1)) //校验包头包尾

     {
       if(receiveData->NodeID == Motor) //ID校验
       {
         //根据下发控制命令进行控制
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
          
           //返回消息组包过程
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
*函数名称 ：无线消息发送函数				                                 
*入口参数 ：*txaddr - 发送地址
*           cID     - clusterID
*           *p      - 发送数据包的地址
*           len     - 发送数据包的长度
*返 回 值 ：无                                                                                
*说    明 ：把组好的数据包，通过无线发送出去                                                            
*****************************************************************************************/


void SerialApp_OTAData(afAddrType_t *txaddr, uint8 cID, void *p, uint8 len) //发送函数
{
  if (afStatus_SUCCESS != AF_DataRequest(txaddr, //发送地址
                                           (endPointDesc_t *)&SerialApp_epDesc, //endpoint描述
                                            cID, //clusterID
                                            len, p, //发送数据包的长度和地址
                                            &SerialApp_MsgID, 0, AF_DEFAULT_RADIUS))
  {
  }
  else
  {
    HalLedBlink(HAL_LED_1,1,50,200);
  }
}

/*****************************************************************************************
*函数名称 ：串口回调函数				                                 
*入口参数 ：port  -端口号
*           event -事件号
*返 回 值 ：无                                                                              
*说    明 ：把串口消息通过无线发送出去                                                             
*****************************************************************************************/

static void SerialApp_CallBack(uint8 port, uint8 event)
{
  (void)port;

  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) && !SerialApp_TxLen) //串口接收到数据包
  {
    SerialApp_TxLen = HalUARTRead(SERIAL_APP_PORT, SerialApp_TxBuf, SERIAL_APP_TX_MAX); //将串口数据读入buf
    SerialApp_TxLen = 0;  
  }
}

/*****************************************************************************************
*函数名称 ：累加校验和函数				                                 
*入口参数 ：*data - 数据包地址
            len   - 数据包长度
*返 回 值 ：sum   - 累加和                                                                                
*说    明 ：对数据包进行累加和校验                                                             
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