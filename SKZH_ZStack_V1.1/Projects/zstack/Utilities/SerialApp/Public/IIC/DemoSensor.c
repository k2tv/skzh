/***************************************************************************************************************
* 文 件 名：DemoSensor.c
×
* 功    能：SensorDemo用户应用层，主要是和网关设备建立绑定和采集传感器数据，周期性的向网关设备
*           报告，接收协调器发送的控制命令，控制被控对象。
*
*
* 注    意：
*           
*           
*           
*           
*
* 版    本：V1.0
* 作    者：WU XIANHAI
* 日    期：2011.3.22
* 奥尔斯电子主页：www.ourselec.com
******************************************************************************************************************/

/******************************************************************************
 * INCLUDES
 */

#include "ZComDef.h"
#include "OSAL.h"
#include "sapi.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_adc.h"
#include "hal_mcu.h"
#include "hal_uart.h"
#include "DemoApp.h"
#include "stdio.h"
#include "OSAL_Nv.h"

/******************************************************************************
 * CONSTANTS
 */
#define REPORT_FAILURE_LIMIT                10 //数据发送失败的最大次数，大于则重新绑定设备
#define ACK_REQ_INTERVAL                    5 //每发送5次数据包，请求一个ACK

// 设备应用层状态
#define APP_INIT                            0    // 初始化
#define APP_START                           1    // 加入网络成功
#define APP_BIND                            2    // 绑定成功
#define APP_REPORT                          4    // 开始发送数据


//用户自定义处理事件  Bit mask of events ( from 0x0000 to 0x00FF )
#define MY_START_EVT                        0x0001 //启动设备事件，主要是协调器
#define MY_REPORT_EVT                       0x0002 //发送数据事件
#define MY_FIND_COLLECTOR_EVT               0x0004 //发现协调器并绑定事件
#define MY_SB_MSG_EVT                       0x0008 //扩展传感器板控制事件
#define MY_PB_MSG_EVT                       0x0010 //电池板控制事件

/******************************************************************************
 * LOCAL VARIABLES
 */
static uint8 appState = APP_INIT;             //设备状态
static uint8 reportFailureNr = 0;             //发送数据失败计
static uint16 parentShortAddr;                // 父亲节点短地址
static uint16 myShortAddr;                    // 本地设备短地址 
extern uint8 motherboardtype;                 //母板类型
static uint8 mypanid = 0;                     //PANID设置标志
static uint8 adchannel = 0;                   //ADC通道
static uint16 SampleMode = 0;                 //采样模式
static uint16 SamplingSpeed = 0;              //采样速度
static uint16 SensormodeID = 0;               //模块ID
static uint8 appControlobject = 0;            //扩展模块所需采样功能

/******************************************************************************
 * GLOBAL VARIABLES
 */

//数据采集节点输入输出命令数
#define NUM_OUT_CMD_SENSOR                1
#define NUM_IN_CMD_SENSOR                 0

//协调器输出命令列表
const cId_t zb_OutCmdList[NUM_OUT_CMD_SENSOR] =
{
  SENSOR_REPORT_CMD_ID
};

// 数据采集节点简单描述
const SimpleDescriptionFormat_t zb_SimpleDesc =
{
  MY_ENDPOINT_ID,             //  Endpoint
  MY_PROFILE_ID,              //  Profile ID
  DEV_ID_SENSOR,              //  Device ID
  DEVICE_VERSION_SENSOR,      //  Device Version
  0,                          //  Reserved
  NUM_IN_CMD_SENSOR,          //  Number of Input Commands
  (cId_t *) NULL,             //  Input Command List
  NUM_OUT_CMD_SENSOR,         //  Number of Output Commands
  (cId_t *) zb_OutCmdList     //  Output Command List
};

/******************************************************************************
 * LOCAL FUNCTIONS
 */
void uartRxCB( uint8 port, uint8 event );
static void sendReport(uint8 *pData,uint8 len);
extern void ctrPCA9554FLASHLED4(uint8 FLASHnum);   
extern void ctrPCA9554FLASHLED5(uint8 FLASHnum);   
extern void ctrPCA9554LED(uint8 led,uint8 operation);
extern void ctrPCA9554Buzzer(uint8 operation);
extern uint8 ctrPCA9554Relay(uint8 cmd);
extern uint8 ctrPCA9554SRelay(uint8 cmd);
extern uint8 read24L01byte(uint8 addr);
extern void th_read(int *t,int *h );
extern uint8 Ctr5573VoltageOutput(uint16 Voltage,uint8 Port);
static void SendDeviceInfo(void);
static void SensorControl(uint16 SensorID,uint8 *ControlMSG);
static void ADCSamplingControl(uint8 channel,uint16 SampleMode,uint16 SamplingSpeed,uint16 SensorID);
static void IICControlSensor(uint8 channel,uint16 SampleMode,uint16 SamplingSpeed,uint16 SensorID);
static void MotherboardControl(uint16 SensorID,uint8 *ControlMSG); 
static void SendSensorInfo(void);

/**************************************************************************************************
 * 函数名称：zb_HandleOsalEvent
 *
 * 功能描述：用户任务事件处理函数
 *
 * 参    数：events - 当前需要处理的事件
 *           
 *
 * 返 回 值：无
 **************************************************************************************************/
void zb_HandleOsalEvent( uint16 event )
{
  
  if(event & SYS_EVENT_MSG)                                                   //系统信息事件（一般无操作）
  {
    
  }
  
  if( event & ZB_ENTRY_EVENT )                                               //进入网络建立和一些用户需要使用的外围设备初始化事件
  { 
    ctrPCA9554LED(5,1);
    initUart(uartRxCB,HAL_UART_BR_115200);                                    //初始化UART 
    
    zb_StartRequest();                                                       //启动设备建立网络（在该启动属于设备的自动启动，需编译HOLD_AUTO_START选项）
  }
  
  if ( event & MY_REPORT_EVT )                                               //开始向协调器报告发送数据 （由绑定确认函数zb_BindConfirm 设置该事件）
  { 
    if ( appState == APP_REPORT ) 
    {
      PCNodeAddrPacket_t PCNodeAddrPacket;                                   //定时报告父子节点关系
      uint8 i,num = 0;
      
      PCNodeAddrPacket.Hdr = '@';
      PCNodeAddrPacket.Len = 10;
      PCNodeAddrPacket.TransportID = 0;
      PCNodeAddrPacket.MSGCode = SENDUP_NODE_PADDR_ADDR;
      PCNodeAddrPacket.NodeAddr = myShortAddr;
      PCNodeAddrPacket.NodePAddr = parentShortAddr;
 
      for(i=0;i<9;i++)
      {
        num = num + ((uint8*)(&PCNodeAddrPacket))[i];
      }
      PCNodeAddrPacket.Checksum = num;
      sendReport((uint8*)(&PCNodeAddrPacket),10);                             //调用发送函数
      ctrPCA9554FLASHLED5(1);
      osal_start_timerEx( sapi_TaskID, MY_REPORT_EVT, 2000 );                //发送数据周期设定
    }
  }
  if ( event & MY_FIND_COLLECTOR_EVT )                                        // 非协调器设备与协调器建立绑定事件
  {
    if ( appState==APP_REPORT )                                               //删除原来的绑定
    {
      zb_BindDevice( FALSE, SENSOR_REPORT_CMD_ID, (uint8 *)NULL );
    }
    
    appState = APP_BIND;                                                     //设置设备为绑定状态
    
    ctrPCA9554LED(4,1);
    
    zb_BindDevice( TRUE, SENSOR_REPORT_CMD_ID, (uint8 *)NULL );              //寻找和并与协调器绑定
  }
  
    if ( event & MY_SB_MSG_EVT )                                             //传感器板控制事件
  {
    if (appControlobject == APP_ADC)                                         //使用AD获取数据的传感器
    {
     ADCSamplingControl(adchannel,SampleMode,SamplingSpeed,SensormodeID);    //调用AD采样控制函数
    }
    else if (appControlobject == APP_IIC)                                    //使用IIC获取数据的传感器或控制
    {
      IICControlSensor(adchannel,SampleMode,SamplingSpeed,SensormodeID);
    }
    else if (appControlobject == APP_IIC_CPU)                                //使用IIC和外部CPU获取数据的传感器或控制
    {
      
    }
    else if (appControlobject == APP_UART)
    {
      
    }
  }
  
  if ( event & MY_PB_MSG_EVT )                                               //电池板控制事件
  {
    
  }
}

/**************************************************************************************************
 * 函数名称：zb_HandleKeys
 *
 * 功能描述：按键事件处理函数
 *
 * 参    数：shift - shift/alt键 该功能基本未使用
 *           keys - 输入按键值
 *
 * 返 回 值：无
 **************************************************************************************************/
 uint16 zgConfigPANIDSetting = 0;
void zb_HandleKeys( uint8 shift, uint8 keys )
{
  char  s[16];
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )                                               //SW1按键事件处理
    {
      mypanid = 1;                                                           //设备PANID设置
      zgConfigPANIDSetting = 0;
      GUI_ClearScreen();                                                      //LCD清屏
      GUI_PutString5_7(25,6,"OURS-CC2530");                                   //在LCD上显示相应的文字             
      GUI_PutString5_7(5,22," ZigBee PRO");                          
      GUI_PutString5_7(5,35,"PANID Setting");
      GUI_PutString5_7(5,48," 0");
      LCM_Refresh();
    }
    if ( keys & HAL_KEY_SW_2 )                                               //SW2按键事件处理
    {
     if(mypanid)                                                             //PANID设置时，为加1功能
     {
       zgConfigPANIDSetting += 1;
       sprintf(s,(char *)" %d         ",zgConfigPANIDSetting);  
       GUI_PutString5_7(5,48,(char *)s);                                     //显示结果
       LCM_Refresh();
       
     }
    }
    if ( keys & HAL_KEY_SW_3 )                                               //SW3按键处理事件
    {
      if(mypanid)
     {
       zgConfigPANIDSetting += 10;                                           //PANID设置时，为加10功能
       sprintf(s,(char *)" %d         ",zgConfigPANIDSetting); 
       GUI_PutString5_7(5,48,(char *)s);                                     //显示结果
       LCM_Refresh();     
     }
      
    }
    if ( keys & HAL_KEY_SW_4 )                                               //SW4按键处理事件
    {
      if(mypanid)
     {
       zgConfigPANIDSetting += 100;
       sprintf(s,(char *)" %d         ",zgConfigPANIDSetting); 
       GUI_PutString5_7(5,48,(char *)s);                                    //显示结果
       LCM_Refresh();      
     }
    }
    if ( keys & HAL_KEY_SW_5 )                                               //SW5按键处理事件
    {
      if(mypanid)                                                            //PANID设置时，为取消设置
      {
        zgConfigPANIDSetting = 0;
        GUI_PutString5_7(5,48,"Unset   ");
        LCM_Refresh();
      }
    }
    if ( keys & HAL_KEY_SW_6 )                                               //SW6按键处理事件
    {
      if(mypanid)                                                            //PANID设置时，为设置完成键
      {
        if(zgConfigPANIDSetting)
        {
         osal_nv_write(ZCD_NV_PANID, 0, 2, &zgConfigPANIDSetting);
        }
        zb_SystemReset();         
      }
    }
  }
}

/**************************************************************************************************
 * 函数名称：zb_StartConfirm
 *
 * 功能描述：设备启动确认函数
 *
 * 参    数：status - 设备启动状态，如果为ZB_SUCCESS表示设备启动成功，其他值则为启动失败
 *           
 *
 * 返 回 值：无
 **************************************************************************************************/
void zb_StartConfirm( uint8 status )
{
  char  s[16];
  uint8 ShortAddrH;
  uint16 ShortAddrL;
  if ( status == ZB_SUCCESS )                                                //判断设备是否启动成功（成功和失败都将调用）
  {
    appState = APP_START;                                                    //设置设备在APP层为启动状态

    zb_GetDeviceInfo(ZB_INFO_PARENT_SHORT_ADDR, &parentShortAddr);           //获取该节点的父亲短地址
 
    zb_GetDeviceInfo(ZB_INFO_SHORT_ADDR, &myShortAddr);                      //获取本设备的短地址 add by wu 2010.9.20
    #if defined ( LCD_SUPPORTED )                                            //更新LCD显示内容
        GUI_SetColor(1,0);
        GUI_ClearScreen();
        LCM_Refresh();
        if(myShortAddr > 0x7fff)
        {
          ShortAddrH = myShortAddr/10000;
          ShortAddrL = myShortAddr%10000;
          sprintf(s,(char *)" %d%d     ",ShortAddrH,ShortAddrL);
        }
        else
        sprintf(s,(char *)" %d     ",myShortAddr);
        GUI_PutString5_7(75,32,(char *)s);                                   //显示结果
        
        osal_nv_read(ZCD_NV_PANID, 0, 2, &zgConfigPANID);
        sprintf(s,(char *)" %d     ",zgConfigPANID);
        GUI_PutString5_7(50,44,(char *)s);              
        GUI_PutString5_7(25,8,"OURS-CC2530");
        GUI_PutString5_7(5,20,"SensorDemo");
        GUI_PutString5_7(5,32,"ShortAddr:");
        GUI_PutString5_7(5,44,"PANID:");
        LCM_Refresh();
    #endif
  
    osal_set_event( sapi_TaskID, MY_FIND_COLLECTOR_EVT );                    //设置与协调器绑定事件
  }
}

/**************************************************************************************************
 * 函数名称：zb_SendDataConfirm
 *
 * 功能描述：数据发送确认函数
 *
 * 参    数：handle - 数据发送处理的ID
 *           status - 发送结果（成功与否状态）
 *
 * 返 回 值：无
 **************************************************************************************************/
void zb_SendDataConfirm( uint8 handle, uint8 status )
{
  if(status != ZB_SUCCESS)                                                   //发送失败
  {
    if ( ++reportFailureNr >= REPORT_FAILURE_LIMIT )                         //失败次数大于等于4次
    {
       
       osal_stop_timerEx( sapi_TaskID, MY_REPORT_EVT );                      //停止发送父子节点关系信息     
       osal_stop_timerEx( sapi_TaskID, MY_SB_MSG_EVT );                      //停止数据采集事件
       
       osal_set_event( sapi_TaskID, MY_FIND_COLLECTOR_EVT );                 //尝试寻找新的协调器，重新绑定设备
       reportFailureNr=0;                                                    //失败次数计数清零
    }
  }
  else                                                                       //发送成功
  { 
    reportFailureNr=0;                                                       //失败次数计数清零
  }
}

/**************************************************************************************************
 * 函数名称：zb_BindConfirm
 *
 * 功能描述：设备绑定确认，开始发送数据
 *
 * 参    数：commandId - 绑定命令的ID
 *           status - 设备启动状态，如果为ZB_SUCCESS表示设备启动成功，其他值则为启动失败
 *           
 *
 * 返 回 值：无
 **************************************************************************************************/
void zb_BindConfirm( uint16 commandId, uint8 status )
{
  if( status == ZB_SUCCESS )                                                 //绑定成功
  {   
    appState = APP_REPORT;                                                   //设备设置为开始发送数据状态

     osal_set_event( sapi_TaskID, MY_REPORT_EVT );                          //设置发送信息事件
     
     if(appControlobject)                                                   //如果开启了数据采用，重新绑定后，继续执行原来的命令
     {
       osal_set_event( sapi_TaskID, MY_SB_MSG_EVT ); 
     }
  }
  else                                                                       //绑定失败，重新绑定
  {
    osal_start_timerEx( sapi_TaskID, MY_FIND_COLLECTOR_EVT, 2000 );
  }
}

/**************************************************************************************************
 * 函数名称：zb_ReceiveDataIndication
 *
 * 功能描述：数据接收处理函数
 *
 * 参    数：source - 发送设备的源短地址
 *           commandId - 命令ID
 *           lqi —链路质量
 *           rssi - 信号强度
 *           len - 接收的数据包长度
 *           pData - 数据缓存指针
 *
 * 返 回 值：无
 **************************************************************************************************/
void zb_ReceiveDataIndication( uint16 source, uint16 command, uint8 lqi, int8 rssi,uint16 len, uint8 *pData  )
{
   static PCGeneralMSGPacket_t PCGeneralMSGPacket;    
   static PCGroupMSGPacket_t PCGroupMSGPacket;
   uint8 i,num = 0;
  if (command == myShortAddr)                                              //本设备数据确认 
  {     
    if(*(pData+4)<=0x10)                                                    //一般信息
    {
      osal_memcpy((void *)&PCGeneralMSGPacket,pData,len);                   //复制保存数据
      num = 0;
      for(i=0;i<(len-1);i++)                                                //计算校验和
      {
        num += ((uint8*)(&PCGeneralMSGPacket))[i];
      }     
      if(PCGeneralMSGPacket.Checksum == num)
      {   
        switch(PCGeneralMSGPacket.MSGCode)
        {
          case FETCH_NODE_INFO :                                           //获取节点信息
            SendDeviceInfo();            
            break;
          case NODE_SYSTEMRESET :                                           //节点重启
           // zb_SystemReset(); 
            zb_StartRequest();  
           break;
          case FETCH_NODE_SENSOR_CAL :                                      //获取传感器校准参数
            SendSensorInfo();
           break;
          default: break;          
        }
      }
    }//end 一般消息
    else if ((*(pData+4)>0x10)&&(*(pData+4)<=0x30))                          //控制信息
    {
      SendDownSBoardDataPacket_t SendDownSBoardDataPacket;
      SendDownTemBoardDataPacket_t SendDownTemBoardDataPacket;
      SetPANIDPacket_t SetPANIDPacket;
      switch (*(pData+4))
      {        
        case SENDDOMN_NODE_SBOARD_DATA:                                     //PC下传节点扩展板资源数据   
          osal_memcpy((void *)&SendDownSBoardDataPacket,pData,len);         //复制保存数据
            SensorControl(SendDownSBoardDataPacket.ModeID,pData+9);         //本地扩展板控制
        break;
        case SENDDOMN_NODE_BBOARD_DATA:                                     //PC下传节点底板资源数据
        osal_memcpy((void *)&SendDownTemBoardDataPacket,pData,len);         //复制保存数据        
        MotherboardControl(SendDownTemBoardDataPacket.ResID,pData+10);      //本地扩展板控制   
        break;
        case SENDDOMN_GROUPNODE_LIST:                                       //PC节点分组消息(暂无)
          
        break;
        case SET_NODE_PANID:
          osal_memcpy((void *)&SetPANIDPacket,pData,len);                   //复制保存数据
          if(SetPANIDPacket.SETPANID)
          {
            if(SetPANIDPacket.SETPANID < 0x3fff)
            {
              osal_nv_write(ZCD_NV_PANID, 0, 2, &(SetPANIDPacket.SETPANID));
              zb_SystemReset(); 
            }
          }   
         break;
        default : break;
      }
    }//end 控制信息
    
    else   //组信息
    {
      
    } // end 组信息 本地信息处理完成         
  }
  else  //群信息
  {
    if(command == 0xffff)
    {
      SendDownSBoardDataPacket_t SendDownSBoardDataPacket;
      if(len>8)           //组网性能测试
      {
        osal_memcpy((void *)&SendDownSBoardDataPacket,pData,len);        //复制保存数据
        SensorControl(SendDownSBoardDataPacket.ModeID,pData+9); //本地扩展板控制
      }
      else
      {
       osal_memcpy((void *)&PCGroupMSGPacket,pData,len);                                    //复制保存数据
       num = 0;
       for(i=0;i<(len-1);i++)                             //计算校验和
       {
         num += ((uint8*)(&PCGroupMSGPacket))[i];
       }
      if((PCGroupMSGPacket.Checksum == num)&&(PCGroupMSGPacket.MSGCode == TURNOFF_APP))
      {
       osal_stop_timerEx( sapi_TaskID, MY_SB_MSG_EVT );
      }
      }
    }
  }  
}

/**************************************************************************************************
 * 函数名称：uartRxCB
 *
 * 功能描述：接收UART数据
 *
 * 参    数：port - UART端口号
 *           event- UART事件（接收或发送）
 *
 * 返 回 值：无
 **************************************************************************************************/
void uartRxCB( uint8 port, uint8 event )
{
  uint8 pBuf[RX_BUF_LEN];
  uint16 len = 0;
  uint8 i;
  uint8 *pdata = NULL;
  SendUpSBoardDataPacket2_t SendUpSBoardDataPacket2 ; 
  if ( event != HAL_UART_TX_EMPTY )                                          //串口接收事件确认
  {   
    len = HalUARTRead( HAL_UART_PORT_0, pBuf, RX_BUF_LEN );                  //读数据
      for(i=3;i<(len+3);i++)
      {
        *(pdata+i) = pBuf[i-3];
      }
    SendUpSBoardDataPacket2.Hdr = 0x40;
    SendUpSBoardDataPacket2.TransportID = 0;
    SendUpSBoardDataPacket2.MSGCode = SENDUP_NODE_SBOARD_DATA;
    SendUpSBoardDataPacket2.NodeAddr = myShortAddr;
    SendUpSBoardDataPacket2.ModeID = 0x3000;
    *pdata = 0x02;
    *(pdata+1) = 0x01;
    *(pdata+2) = len;
    SendUpSBoardDataPacket2.Len = 13+len;
    osal_memcpy((void *)((&SendUpSBoardDataPacket2.Hdr)+9),pdata,len+3);    
    sendReport((uint8*)(&SendUpSBoardDataPacket2),SendUpSBoardDataPacket2.Len); //调用发送函数
  }
}

/**************************************************************************************************
 * 函数名称：sendReport
 *
 * 功能描述：构造发送数据的信息包，并调用无线发送函数来发送数据包
 *
 * 参    数：无
 *           
 *
 * 返 回 值：无
 **************************************************************************************************/
static void sendReport(uint8 *pData,uint8 len)
{
  static uint8 reportNr=0;
  uint8 txOptions;

  if ( ++reportNr<ACK_REQ_INTERVAL && reportFailureNr==0 ) 
  {
    txOptions = AF_TX_OPTIONS_NONE;
  }
  else 
  {
    txOptions = AF_MSG_ACK_REQUEST;
    reportNr = 0;
  }
  
  zb_SendDataRequest( 0xFFFE, SENSOR_REPORT_CMD_ID, len, pData, 0, txOptions, 0 );
}


/**************************************************************************************************
 * 函数名称：SendDeviceInfo
 *
 * 功能描述：获取设备信息，并无线发送给协调器
 *
 * 参    数：无
 *           
 *
 * 返 回 值：无
 **************************************************************************************************/
static uint8 sensoridL = 0;
static uint8 sensoridH = 0;
static void SendDeviceInfo(void)
{
  static PCNodeMSGPacket_t PCNodeMSGPacket;
  static uint8 *myIEEEAddr = NULL;                       // 本地设备IEEE地址 
  uint8 i;
  PCNodeMSGPacket.Hdr = '@';
  PCNodeMSGPacket.Len = 23;
  PCNodeMSGPacket.TransportID = 0;
  PCNodeMSGPacket.MSGCode = SENDUP_NODE_INFO;
  PCNodeMSGPacket.NodeAddr = myShortAddr;
  zb_GetDeviceInfo(ZB_INFO_IEEE_ADDR, myIEEEAddr);                         //获取设备IEEE地址 
   for(i=0;i<8;i++)
  {
     PCNodeMSGPacket.IEEEAddr[i] = *(myIEEEAddr+i); 
  }
  PCNodeMSGPacket.NodeRSSI = 0;
  PCNodeMSGPacket.NodeLQI = 0;
  osal_nv_read(ZCD_NV_PANID, 0, 2, &zgConfigPANID);
  PCNodeMSGPacket.Panid = zgConfigPANID;
  PCNodeMSGPacket.TemType = motherboardtype;                                 //母板类型
  if(sensoridH == 0)
  {
   sensoridH = read24L01byte(0x01);
   sensoridL = read24L01byte(0x02);
   if((sensoridH == 0)&&(sensoridL == 0))                                    //读不出来信息，即为电压检测模块
   {
     if (HalAdcRead (HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_14)>3000)
     {
      sensoridH = 0x10;
      sensoridL = 0x01;
     }
   }
  }
 // PCNodeMSGPacket.SBoardType = resCode_Voltage_Single;                       //传感器板类型  
  PCNodeMSGPacket.SBoardType = BUILD_UINT16(sensoridL,sensoridH);           //传感器板类型 
  sendReport((uint8*)(&PCNodeMSGPacket),23);                                  //调用发送函数 
}

/**************************************************************************************************
 * 函数名称：SendSensorInfo
 *
 * 功能描述：获取扩展板传感器数据校准参数
 *
 * 参    数：无
 *           
 *
 * 返 回 值：无
 **************************************************************************************************/
static void SendSensorInfo(void)
{
  SensorCalPacket_t SensorCalPacket = {0x40,16,0,SENSOR_DATA_CAL,myShortAddr,0,0,0,0,0};
  SensorCalPacket.ModeID = BUILD_UINT16(sensoridL,sensoridH);    //扩展传感板类型
  if(SensorCalPacket.ModeID == resCode_Voltage_Dual)
  {
    SensorCalPacket.Maxdata = 8192;
    SensorCalPacket.Zerodata = 4096;
    SensorCalPacket.Mindata = 0;
  }
  else if(SensorCalPacket.ModeID == resCode_Current_Dual)
  {
    SensorCalPacket.Maxdata = 8192;
    SensorCalPacket.Zerodata = 0;
    SensorCalPacket.Mindata = 0;
  }
  else
  {
    
  }
  sendReport((uint8*)(&SensorCalPacket),16);                                  //调用发送函数 
}

/**************************************************************************************************
 * 函数名称：SensorControl
 *
 * 功能描述：解析PC端发送的扩展板控制信息，然后传递给相应的控制函数。
 *
 * 参    数：SensorID - 被控传感器或其他器件编号
 *           ControlMSG - PC发送的控制信息
 *           
 *
 * 返 回 值：无
 **************************************************************************************************/
//uint16 fortest = 0;
static void SensorControl(uint16 SensorID,uint8 *ControlMSG)
{
   ADControlData_t ADControlData;
   UARTontrolData_t UARTontrolData;
   UARTData_t UARTData;
   DAControlData_t DAControlData;
   SendUpSBoardDataPacket2_t SendUpSBoardDataPacket2;
  uint8 Relaystate = 0;
  SendUpSBoardDataPacket2.Hdr = '@';
  SendUpSBoardDataPacket2.TransportID = 0;
  SendUpSBoardDataPacket2.MSGCode = SENDUP_NODE_SBOARD_DATA;
  SendUpSBoardDataPacket2.NodeAddr = myShortAddr;
 // fortest = 0;
  if(SensorID < 0x2010)
  {
    osal_memcpy((void *)&ADControlData,(ADControlData_t *)ControlMSG,11);
    SampleMode = ADControlData.SampleMode;
    SamplingSpeed = ADControlData.SamplingSpeed;
    SensormodeID = SensorID;  
  }
   switch(SensorID)
   {
     case resCode_Voltage_Single:
      adchannel = 0x80;
      appControlobject = APP_ADC;
      osal_set_event( sapi_TaskID, MY_SB_MSG_EVT );          
    break;
    case resCode_Voltage_Dual:       
      adchannel = 0xC0;
      appControlobject = APP_ADC;
      osal_set_event( sapi_TaskID, MY_SB_MSG_EVT );
    break;
    case resCode_Current_Single:
      adchannel = 0x40;
      appControlobject = APP_ADC;
      osal_set_event( sapi_TaskID, MY_SB_MSG_EVT );
    break;
    case resCode_Current_Dual:
      adchannel = 0xC0;
      appControlobject = APP_ADC;
      osal_set_event( sapi_TaskID, MY_SB_MSG_EVT );
    break;
    case resCode_Light_Single:
      adchannel = 0x80;
      appControlobject = APP_ADC;
      osal_set_event( sapi_TaskID, MY_SB_MSG_EVT );
    break;
    case resCode_Pressure:
      adchannel = 0x80;
      appControlobject = APP_ADC;
      osal_set_event( sapi_TaskID, MY_SB_MSG_EVT );
    break;
    case resCode_Alcohol:
      adchannel = 0x80;
      appControlobject = APP_ADC;
      osal_set_event( sapi_TaskID, MY_SB_MSG_EVT );
    break;
    case resCode_Pressure_Alcohol:
      adchannel = 0xC0;
      appControlobject = APP_ADC;
      osal_set_event( sapi_TaskID, MY_SB_MSG_EVT );
    break;
    case resCode_Temp_Humidity:
      adchannel = 0x0F;
      if(ADControlData.SamplingSpeed < 500)
      {
        SamplingSpeed = 500;
      }
      else
      {
       SamplingSpeed = ADControlData.SamplingSpeed;
      }
      appControlobject = APP_ADC;
      osal_set_event( sapi_TaskID, MY_SB_MSG_EVT );
    break;
    case resCode_Temp_Humidity_Light:
      adchannel = 0x8F;
      if(ADControlData.SamplingSpeed < 500)
      {
        SamplingSpeed = 500;
      }
      else
      {
       SamplingSpeed = ADControlData.SamplingSpeed;
      }
      appControlobject = APP_ADC;
      osal_set_event( sapi_TaskID, MY_SB_MSG_EVT );
    break;
    case resCode_Voltage_Output:                 //电压输出     
      osal_memcpy((void *)&DAControlData,(DAControlData_t *)ControlMSG,9);
      if(DAControlData.ChComb & 0x80)
      {
        Ctr5573VoltageOutput(0,0xd0);
        Ctr5573VoltageOutput(0,0xd2);
        Ctr5573VoltageOutput(0,0xd4);
        Ctr5573VoltageOutput(0,0xd6);
      }
      else if(DAControlData.ChComb & 0x40)
      {
        if (DAControlData.ChComb & 0x08) Ctr5573VoltageOutput(DAControlData.dataA,0xd0);
        if (DAControlData.ChComb & 0x04) Ctr5573VoltageOutput(DAControlData.dataB,0xd2);
        if (DAControlData.ChComb & 0x02) Ctr5573VoltageOutput(DAControlData.dataC,0xd4);
        if (DAControlData.ChComb & 0x01) Ctr5573VoltageOutput(DAControlData.dataD,0xd6);
      }
         
    break;
    case resCode_Relay_GPIN:
      Relaystate = ctrPCA9554SRelay(*ControlMSG);
      SendUpSBoardDataPacket2.data = (uint8 *)Relaystate;
      SendUpSBoardDataPacket2.Len = 11;
      SendUpSBoardDataPacket2.ModeID = resCode_Relay_GPIN;        
      sendReport((uint8*)(&SendUpSBoardDataPacket2),SendUpSBoardDataPacket2.Len);                     //调用发送函数     
    break;
    case resCode_Photoelectric:
      
    break;
    case resCode_IR_Output:
     
    break;
    case resCode_IR_Photoelectric:
      
    break;
    case resCode_Ultrasonic:
     
    break;  
    case resCode_RS232_Wireless:
      if(*ControlMSG == 0x01)
      {
        osal_memcpy((void *)&UARTontrolData,(UARTontrolData_t *)ControlMSG,7);
        
        initUart(uartRxCB,UARTontrolData.UART_BR);
      }
      else
      {
        osal_memcpy((void *)&UARTData,(UARTData_t *)ControlMSG,*(ControlMSG+2)+3);
         HalUARTWrite(HAL_UART_PORT_0,(uint8 *)((&UARTData.UARTmode)+3), UARTData.DataLen);  //通过串口向PC机发送设备识别信息
      }
    
    break;
    case resCode_RDID:
      
    break;
    case resCode_RS232_Coordinator:
     
    break; 
    default:break;
   }  
}


/**************************************************************************************************
 * 函数名称：MotherboardControl
 *
 * 功能描述：解析PC端发送的底板控制信息，然后传递给相应的控制函数。
 *
 * 参    数：SensorID - 被控传感器或其他器件编号
 *           ControlMSG - PC发送的控制信息
 *           
 *
 * 返 回 值：无
 **************************************************************************************************/
static void MotherboardControl(uint16 SensorID,uint8 *ControlMSG)
{
  uint8 ctrled[4];
  uint8 Relaystate = 0;
  SendUpTemBoardDataPacket_t SendUpTemBoardDataPacket;
  SendUpTemBoardDataPacket.Hdr = '@';
  SendUpTemBoardDataPacket.TransportID = 0;
  SendUpTemBoardDataPacket.MSGCode = SENDUP_NODE_BBOARD_DATA;
  SendUpTemBoardDataPacket.NodeAddr = myShortAddr;
  SendUpTemBoardDataPacket.TemType = motherboardtype;
  //SendUpTemBoardDataPacket.Checksum = 0;
  switch(SensorID)
  {
    case resCode_Relay:
      Relaystate = ctrPCA9554Relay(*ControlMSG);
      SendUpTemBoardDataPacket.data = (uint8 *)Relaystate;
      SendUpTemBoardDataPacket.Len = 12;
      SendUpTemBoardDataPacket.ResID = resCode_Relay;     
      sendReport((uint8*)(&SendUpTemBoardDataPacket),SendUpTemBoardDataPacket.Len);                     //调用发送函数
    break;
    case resCode_Led:
    osal_memcpy((void *)ctrled,(uint8 *)ControlMSG,4);
    if(ctrled[0]) ctrPCA9554LED(0,1);
    else ctrPCA9554LED(0,0);
    if(ctrled[1]) ctrPCA9554LED(1,1);
    else ctrPCA9554LED(1,0);
    if(ctrled[2]) ctrPCA9554LED(2,1);
    else ctrPCA9554LED(2,0);
    if(ctrled[3]) ctrPCA9554LED(3,1);
    else ctrPCA9554LED(3,0);
    osal_memcpy((void *)&(SendUpTemBoardDataPacket.data),(uint8 *)ctrled,4);
    SendUpTemBoardDataPacket.Len = 15;
    SendUpTemBoardDataPacket.ResID = resCode_Led;     
  
    sendReport((uint8*)(&SendUpTemBoardDataPacket),SendUpTemBoardDataPacket.Len);                     //调用发送函数
    break;
    case resCode_Buzzer:
     if(*ControlMSG)
     {
       ctrPCA9554Buzzer(1);
       SendUpTemBoardDataPacket.data = (uint8 *)0x01;
     }
     else
     {
       ctrPCA9554Buzzer(0);
       SendUpTemBoardDataPacket.data = (uint8 *)0x00;
     }
     
     SendUpTemBoardDataPacket.Len = 12;
     SendUpTemBoardDataPacket.ResID = resCode_Buzzer;
   
    sendReport((uint8*)(&SendUpTemBoardDataPacket),SendUpTemBoardDataPacket.Len);                     //调用发送函数
    break;
   default:break;
  }
}

/**************************************************************************************************
 * 函数名称：IICControlSensor
 *
 * 功能描述：根据PC发送的用IIC控制传感器的命令，采集相应传感器模块的传感器数据
 *
 * 参    数：channel - AD采样所使用的通道
 *           SampleMode - 采样模式
 *           SamplingSpeed —采样速度
 *           SensorID - 节点扩展板传感器ID
 *
 *           
 *
 * 返 回 值：无
 **************************************************************************************************/
static void IICControlSensor(uint8 channel,uint16 SampleMode,uint16 SamplingSpeed,uint16 SensorID)
{
  //SendUpSBoardDataPacket_t SendUpSBoardDataPacket;
  if(channel)
  {
    
  }
  else
  {
    
  }
  
}

/**************************************************************************************************
 * 函数名称：ADCSamplingControl
 *
 * 功能描述：根据PC发送的AD采样命令，采集各个传感器模块的传感器数据
 *
 * 参    数：channel - AD采样所使用的通道
 *           SampleMode - 采样模式
 *           SamplingSpeed —采样速度
 *           SensorID - 节点扩展板传感器ID
 *
 *           
 *
 * 返 回 值：无
 **************************************************************************************************/ 
static void ADCSamplingControl(uint8 channel,uint16 SampleMode,uint16 SamplingSpeed,uint16 SensorID)
{
   SendUpSBoardDataPacket_t SendUpSBoardDataPacket;
   uint16 *ADCdata = NULL;
   int tempera = 0;
   int humidity = 0;
  
    if (channel == 0x80)  //使用0通道
   {  
     // fortest++;
      SendUpSBoardDataPacket.data = (uint16*)HalAdcRead (HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_14);
      //SendUpSBoardDataPacket.data = (uint16*)fortest;
      SendUpSBoardDataPacket.Len = 12;  
   }
    else if(channel == 0x40)  //使用1通道
   {
      SendUpSBoardDataPacket.data = (uint16*)HalAdcRead (HAL_ADC_CHANNEL_1, HAL_ADC_RESOLUTION_14);
      SendUpSBoardDataPacket.Len = 12;  
   }
   else if(channel == 0xC0)  //使用两个通道
   {
      *ADCdata = HalAdcRead (HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_14);         
      *(ADCdata + 1) = HalAdcRead (HAL_ADC_CHANNEL_1, HAL_ADC_RESOLUTION_14);
      osal_memcpy((void *)&(SendUpSBoardDataPacket.data),ADCdata,4);
      SendUpSBoardDataPacket.Len = 14;
   }
   else if(channel == 0x0f) //只有IIC读取的传感器数据，没有ADC形式的传感器
   {     
      th_read(&tempera,&humidity);      
      *ADCdata = (uint16)tempera;         
      *(ADCdata + 1) = (uint16)humidity;
      osal_memcpy((void *)&(SendUpSBoardDataPacket.data),ADCdata,4);
      SendUpSBoardDataPacket.Len = 14;
   }
   else if(channel == 0x8f) //IIC读取的传感器数据和使用0通道
   {
      th_read(&tempera,&humidity);      
      *ADCdata = (uint16)tempera;         
      *(ADCdata + 1) = (uint16)humidity;
      *(ADCdata + 2) = HalAdcRead (HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_14);
      osal_memcpy((void *)&(SendUpSBoardDataPacket.data),ADCdata,6);
      SendUpSBoardDataPacket.Len = 16;
   }
   else if(channel == 0x4f) //IIC读取的传感器数据和使用1通道
   {
     
   }
   else if(channel == 0xcf) //IIC读取的传感器数据和1,2两个通道
   {
     
   }
   SendUpSBoardDataPacket.Hdr = '@';
   SendUpSBoardDataPacket.TransportID = 0;
   SendUpSBoardDataPacket.MSGCode = SENDUP_NODE_SBOARD_DATA;
   SendUpSBoardDataPacket.NodeAddr = myShortAddr;
   SendUpSBoardDataPacket.ModeID = SensorID;
     
    if (SampleMode == 0) //停止采样
   {
      osal_stop_timerEx( sapi_TaskID, MY_SB_MSG_EVT );
      ctrPCA9554LED(4,1);
      appControlobject = 0;
     // sendReport((uint8*)(&SendUpSBoardDataPacket),SendUpSBoardDataPacket.Len);                     //调用发送函数
   }
   else if(SampleMode == 1) //单次采样
   {
     sendReport((uint8*)(&SendUpSBoardDataPacket),SendUpSBoardDataPacket.Len);                     //调用发送函数
     osal_stop_timerEx( sapi_TaskID, MY_SB_MSG_EVT );    
     appControlobject = 0;
   }
    else       //连续采样 SamplingSpeed> 100ms
   {
     
     sendReport((uint8*)(&SendUpSBoardDataPacket),SendUpSBoardDataPacket.Len);                     //调用发送函数
     if (SamplingSpeed < 100)
    {
       osal_start_timerEx( sapi_TaskID, MY_SB_MSG_EVT, 100 );
    }
    else if (SamplingSpeed > 5000)
    {
      osal_start_timerEx( sapi_TaskID, MY_SB_MSG_EVT, 5000 );
    }
     else
     {
       osal_start_timerEx( sapi_TaskID, MY_SB_MSG_EVT, SamplingSpeed );
     }
   // if (fortest == 2000)
   // {
    //  osal_stop_timerEx( sapi_TaskID, MY_SB_MSG_EVT ); 
    //  appControlobject = 0;
    //  fortest = 0;
    //}
   }
}




