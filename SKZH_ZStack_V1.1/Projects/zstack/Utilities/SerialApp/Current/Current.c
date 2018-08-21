/**************************************Copyright(c)****************************************
* 　　　　　　　　            青岛山科智汇信息科技有限公司 　　　　　 　　                *
* 　　　　　　　　　　　　　                          　　　　　　　　　　　　　　　　　　*
* 　　　　　　　　　                  www.iotsk.com 　　　　　　　　　                  　*
* 　　　　　　　　　　　　　　　　　　　　　　　　　　　                          　　　　*
*----------------------------------------File Info----------------------------------------*
*文 件 名 ：　　　　　　                                                                  *
*修改日期 ：  　　　　　　　　　　　　　　                                                *
*版 本 号 ：　　　　　　　　　　　　　　　　　                                            *
*描    述 ：                                                                              *
*                                                                                         *
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
#include "Touch.h"
#include <string.h>



#include "hal.h"

#include "stdio.h"

#define noACK 0
#define ACK   1

#define STATUS_REG_W 0x06
#define STATUS_REG_R 0x07
#define MEASURE_TEMP 0x03
#define MEASURE_HUMI 0x05
#define RESET        0x1e

#define SCL          P1_0     //SHT10时钟
#define SDA          P1_1     //SHT10数据线

unsigned char d1,d2,d3,d4,d5,d6,d7;

void Wait(unsigned int ms);
void QWait(void)  ;
char s_write_byte(unsigned char value);
char s_read_byte(unsigned char ack);
void s_transstart(void);
void s_connectionreset(void);
char s_measure( unsigned char *p_checksum, unsigned char mode);
void initIO(void);
void th_read(int *t,int *h );
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

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

uint8 SerialApp_TaskID;    // Task ID for internal task/event processing.
devStates_t  SampleApp_NwkState;
static UART_Format UART0_Format;
static UART_Format_End4 UART0_Format1;

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
static uint8 TouchStatusOld;
static uint8 TouchStatusNow;
/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt );
void SerialApp_OTAData(afAddrType_t *txaddr,uint8 ID,void *p,uint8 len);
static void SerialApp_CallBack(uint8 port, uint8 event);
extern uint8 CheckSum(uint8 *data,uint8 len);
static void PhotoInit(void);

static void PhotoInit(void) //P00  AD   P01   开关
{
  P0SEL &= ~0x01;  //P01设置为通用IO
  P0DIR &= ~0x01;  //将P01设置为输入模式 
  if(TOUCH_GPIO == HIGH)
    TouchStatusNow = TOUCH_NO;
  else
    TouchStatusNow = TOUCH_YES;
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
  PhotoInit();

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
  UART0_Format.NodeID   = Current;
  
  UART0_Format1.Header   = 0x40;
  UART0_Format1.Len      = 0x0a;
  UART0_Format1.NodeSeq  = 0x01;
  UART0_Format1.NodeID   = Current;
  
  SerialApp_TxAddr.addrMode =(afAddrMode_t)Addr16Bit;//发送地址初始化
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
  
  INT8 adc0_value[2],adc1_value[2];
  
  
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
        break;

      case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if(SampleApp_NwkState == DEV_END_DEVICE) //判定当前设备类型
          {
            HalLedSet(HAL_LED_1,HAL_LED_MODE_OFF);
            HalLedBlink(HAL_LED_2,5,50,200);
            osal_set_event(SerialApp_TaskID, PERIOD_EVT); //启动周期消息
            osal_set_event(SerialApp_TaskID, TOUCH_READ_EVT); //启动光照检测
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
    UART0_Format.Data[0] = 0x00;
    UART0_Format.Data[1] = 0x00;
    
    num = CheckSum(&UART0_Format.Header,UART0_Format.Len);
    UART0_Format.Verify  = num;

    
    
    SerialApp_OTAData(&SerialApp_TxAddr, SERIALAPP_CLUSTERID1, &UART0_Format, sizeof(UART_Format));
    osal_start_timerEx(SerialApp_TaskID, PERIOD_EVT, 5000);
    return ( events ^ PERIOD_EVT );
  }
  
  if ( events & SERIALAPP_SEND_EVT ) //发送RF消息
  {
    num = CheckSum(&UART0_Format1.Header,UART0_Format1.Len);
    UART0_Format1.Verify  = num;

    
    SerialApp_OTAData(&SerialApp_TxAddr,SERIALAPP_CLUSTERID1, &UART0_Format1, 10);

    return ( events ^ SERIALAPP_SEND_EVT );
  }

  if ( events & TOUCH_READ_EVT )  //查看传感器状态
  {  
    
    
     
    ADC_ENABLE_CHANNEL(ADC_AIN0);                          // 使能AIN0为ADC输入通道

    
    
    
    
    ADC_SINGLE_CONVERSION(ADC_REF_AVDD | ADC_8_BIT | ADC_AIN0);

    ADC_SAMPLE_SINGLE();                                   // 启动一个单一转换

    while(!ADC_SAMPLE_READY());                            // 等待转换完成

    ADC_ENABLE_CHANNEL(ADC_AIN0);                          // 禁止AIN0

    adc0_value[0] = ADCH; 
    adc0_value[1] = ADCL;
    
    ADCH=0;
    ADCL=0;
    
    
    ADC_ENABLE_CHANNEL(ADC_AIN1);                          // 使能AIN1为ADC输入通道

    ADC_SINGLE_CONVERSION(ADC_REF_AVDD | ADC_8_BIT | ADC_AIN1);

    ADC_SAMPLE_SINGLE();                                   // 启动一个单一转换

    while(!ADC_SAMPLE_READY());                            // 等待转换完成

    ADC_ENABLE_CHANNEL(ADC_AIN1);                          // 禁止AIN1

    adc1_value[0] = ADCH; 
    adc1_value[1] = ADCL;
    ADCH=0;
    ADCL=0;
    
    
    
    
     UART0_Format1.Command = 0x01;                                   
     UART0_Format1.Data[0] = adc0_value[0];
     UART0_Format1.Data[1] = adc0_value[1];
     UART0_Format1.Data[2] = adc1_value[0];
     UART0_Format1.Data[3] = adc1_value[1];
    
    
    
   
     osal_set_event(SerialApp_TaskID, SERIALAPP_SEND_EVT);

    osal_start_timerEx(SerialApp_TaskID, TOUCH_READ_EVT, 500);
    
    return ( events ^ TOUCH_READ_EVT );
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
void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt )  //处理接收到的RF消息
{ 
  uint8 num=0;
  static UART_Format *receiveData;
  switch ( pkt->clusterId )
  {
   case SERIALAPP_CLUSTERID1:  //处理各个传感器节数据    
     receiveData = (UART_Format *)(pkt->cmd.Data);
     HalLedBlink(HAL_LED_1,1,50,200);
     
     num = CheckSum((uint8*)receiveData,receiveData->Len);
     
   if((receiveData->Header==0x40)&&(receiveData->Verify==num)) //校验包头包尾

     
     
     {
      
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

  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) && !SerialApp_TxLen) //串口接收到数据包
  {
    SerialApp_TxLen = HalUARTRead(SERIAL_APP_PORT, SerialApp_TxBuf, SERIAL_APP_TX_MAX); //将串口数据读入buf
    SerialApp_TxLen = 0;  
  }
}

/*********************************************************************
*********************************************************************/
extern uint8 CheckSum(uint8 *data,uint8 len);
/*****************累加校验和函数***************/
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











/**************************************************************************************************
 * 函数名称：Wait
 *
 * 功能描述：延时函数（不精确延时）
 *
 * 参    数：ms -- 延时时间
 *
 * 返 回 值：无
 **************************************************************************************************/
void Wait(unsigned int ms)
{
                    
   unsigned char g,k;
   while(ms)
   {
      
	  for(g=0;g<=167;g++)
	   {
	     for(k=0;k<=48;k++);
	   }
      ms--;                            
   }
} 

/**************************************************************************************************
 * 函数名称：QWait
 *
 * 功能描述：延时函数（大约1us的延时）
 *
 * 参    数：无
 *
 * 返 回 值：无
 **************************************************************************************************/
void QWait()     
{
    asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");
    asm("NOP");

}

/**************************************************************************************************
 * 函数名称：initIO
 *
 * 功能描述：SHT10串行通信IO初始化
 *
 * 参    数：无
 *
 * 返 回 值：无
 **************************************************************************************************/
void initIO(void)
{
  IO_DIR_PORT_PIN(1, 0, IO_OUT);
  IO_DIR_PORT_PIN(1, 1, IO_OUT);
  P1INP |= 0x03;
  SDA = 1; SCL = 0;
}

/**************************************************************************************************
 * 函数名称：s_write_byte
 *
 * 功能描述：从SHT10写一个字节
 *
 * 参    数：value -- 需写入的字节值
 *
 * 返 回 值：error -- 操作是否成功
 **************************************************************************************************/
char s_write_byte(unsigned char value)
{ 
  unsigned char i,error=0;  
  IO_DIR_PORT_PIN(1, 0, IO_OUT);      //时钟和数据IO设置为输出
  IO_DIR_PORT_PIN(1, 1, IO_OUT);
  for (i=0x80;i>0;i/=2)               //将一个字节的8位逐一输出        
  {
     if (i & value)
  	 SDA=1;          
     else
    	 SDA=0;                        
    SCL = 1;                        
    QWait();QWait();QWait();QWait();QWait();
    SCL = 0;
    asm("NOP"); asm("NOP");
  }
  SDA = 1; 
  IO_DIR_PORT_PIN(1, 1, IO_IN);      //将数据线设置为输入，以准备接收SHT10的ACK
  SCL = 1;  asm("NOP");                          
  error = SDA; 
  QWait();QWait();QWait();
  IO_DIR_PORT_PIN(1, 1, IO_OUT);     //将数据线恢复为输出状态
  SDA = 1; 
  SCL = 0;        
  
  return error;                                   
}

/**************************************************************************************************
 * 函数名称：s_read_byte
 *
 * 功能描述：从SHT10读取一个字节
 *
 * 参    数：ack -- 读取数据后，向SHT10发送ACK
 *
 * 返 回 值：val -- 读取的字节值
 **************************************************************************************************/
char s_read_byte(unsigned char ack)
{
  IO_DIR_PORT_PIN(1, 0, IO_OUT);     //时钟和数据IO设置为输出
  IO_DIR_PORT_PIN(1, 1, IO_OUT);
  unsigned char i,val=0;
  SDA= 1;
  IO_DIR_PORT_PIN(1, 1, IO_IN);      //将数据线设置为输入，以准备接收SHT10的数据
  for (i=0x80;i>0;i/=2) 
  {
    SCL = 1;
    if (SDA)
     val = (val | i);
    else
      val = (val | 0x00);
    SCL = 0;
    QWait();QWait();QWait();QWait();QWait();
  }
  IO_DIR_PORT_PIN(1, 1, IO_OUT);     //将数据线恢复为输出状态
  SDA = !ack;
  SCL = 1;
  QWait();QWait();QWait();QWait();QWait();
  SCL = 0;
  SDA = 1;
  
  return val;                       //返回读取的值
}

/**************************************************************************************************
 * 函数名称：s_transstart
 *
 * 功能描述：启动SHT10，开始与SHT10通信
 *
 * 参    数：无
 *
 * 返 回 值：无
 **************************************************************************************************/
void s_transstart(void)
{
  IO_DIR_PORT_PIN(1, 0, IO_OUT);
  IO_DIR_PORT_PIN(1, 1, IO_OUT);
   SDA = 1; SCL = 0;
   QWait();QWait();
   SCL = 1;QWait();QWait();
   SDA = 0;QWait();QWait(); 
   SCL = 0;QWait();QWait();QWait();QWait();QWait();
   SCL = 1;QWait();QWait();
   SDA = 1;QWait();QWait();
   SCL = 0;QWait();QWait();
}

/**************************************************************************************************
 * 函数名称：s_connectionreset
 *
 * 功能描述：与SHT10通信复位
 *
 * 参    数：无
 *
 * 返 回 值：无
 **************************************************************************************************/
void s_connectionreset(void)
{
  IO_DIR_PORT_PIN(1, 0, IO_OUT);
  IO_DIR_PORT_PIN(1, 1, IO_OUT);
  unsigned char i;
  SDA = 1; SCL= 0;
  for(i=0;i<9;i++)
  {
    SCL = 1;QWait();QWait();
    SCL = 0;QWait();QWait();
  }
  s_transstart();
}

/**************************************************************************************************
 * 函数名称：s_measure
 *
 * 功能描述：发送命令、读取SHT10温度或湿度数据
 *
 * 参    数：*p_checksum -- 校验和
 *           mode -- 读取数据类型（3为温度，5为湿度）
 *
 * 返 回 值：er -- 操作结果
 **************************************************************************************************/
char s_measure( unsigned char *p_checksum, unsigned char mode)
{
  unsigned er=0;
  unsigned int i,j;
  s_transstart();                              //启动传输
  switch(mode)
  {
    case 3	:er+=s_write_byte(3);break;    //发送温度读取命令
    case 5	:er+=s_write_byte(5);break;    //发送湿度读取命令
    default     :break;
  }
  IO_DIR_PORT_PIN(1, 1, IO_IN);                //将数据线设置为输入，以准备接收SHT10的ACK
  for(i=0;i<65535;i++)
  {
    for(j=0;j<65535;j++)
    {if(SDA == 0)
    {break;}}
    if(SDA == 0)
    {break;}
  }
  
  if(SDA)                                     //SDA没有拉低，错误信息加1
    
  {er += 1;}
  d1 = s_read_byte(ACK);                     //数据读取
  d2 = s_read_byte(ACK);
  d3 = s_read_byte(noACK);
  return er;
}

/**************************************************************************************************
 * 函数名称：th_read
 *
 * 功能描述：调用相应函数，读取温度和数据数据并校验和计算
 *
 * 参    数：*t -- 温度值
 *           *h -- 湿度值
 *
 * 返 回 值：无
 **************************************************************************************************/
void th_read(int *t,int *h )
{
  unsigned char error,checksum;
  float humi,temp;
  int tmp;
  initIO();
  
  s_connectionreset();                  //启动传输
    error=0;
   error+=s_measure(&checksum,5);       //读取湿度数据并校验
    humi = d1*256+d2;
    
    error+=s_measure(&checksum,3);      //读取温度数据并校验
    temp = d1*256+d2;
    if(error!=0) s_connectionreset();   //读取失败，通信复位
    else                                //读取成功，计算数据
    {      
       temp = temp*0.01  -  44.0 ;
       humi = (temp - 25) * (0.01 + 0.00008 * humi) -0.0000028 * humi * humi + 0.0405 * humi-4;
       if(humi>100)
       {humi = 100;}
       if(humi<0.1)
       {humi = 0.1;}
    }
    
    tmp=(int)(temp*10)%10;
    
    if(tmp>4)
    {
     temp=temp+1; 
    }
    else
    {
       temp=temp;
    }
    
  *t=(int)temp;
  
   tmp=(int)(humi*10)%10;
    
    if(humi>4)
    {
     humi=humi+1; 
    }
    else
    {
       humi=humi;
    }
    
  *h=(int)humi;
  
}

