/***************************************************************************************************************
* 文 件 名：DemoCollector.c
×
* 功    能：SensorDemo的协调器用户应用层，该文件主要调用Z-Stack的一些API，使协调器能够建立和维护网络，并通过UART与PC机通信，
*           将收集到的各个节的数据传给PC机。
*
*
* 注    意：
*                     
*           
*
* 版    本：V1.0
* 作    者：WU XIANHAI
* 日    期：2011.3.21
* 奥尔斯电子主页：www.ourselec.com
******************************************************************************************************************/

/******************************************************************************
 * INCLUDES
 */

#include "ZComDef.h"
#include "OSAL.h"
#include "OSAL_Nv.h"
#include "sapi.h"
#include "hal_key.h"
#include "hal_led.h"
#include "hal_lcd.h"
#include "hal_adc.h"
#include "hal_uart.h"
#include "DemoApp.h"
#include "OursApp.h"
#include "OnBoard.h"
#include "stdio.h"

/******************************************************************************
 * CONSTANTS
 */

#define REPORT_FAILURE_LIMIT                10            //数据发送失败的最大次数，大于则重新绑定设备
#define ACK_REQ_INTERVAL                    5             // 每发送5个数据包，请求一次ACK

// Stack Profile
#define ZIGBEE_2007                         0x0040
#define ZIGBEE_PRO_2007                     0x0041

#ifdef ZIGBEEPRO
#define STACK_PROFILE                       ZIGBEE_PRO_2007             
#else 
#define STACK_PROFILE                       ZIGBEE_2007
#endif

// 应用状态
#define APP_INIT                            0
#define APP_START                           2
#define APP_BINDED                          3    

// 应用层系统事件
#define MY_START_EVT                        0x0001       //启动事件
#define MY_REPORT_EVT                       0x0002       //数据报告事件
#define MY_FIND_COLLECTOR_EVT               0x0004       //发现和绑定协调器事件
#define MY_COOR_LINK_SEV_EVT                0x0008       //与中间服务器建立通信事件 add by wu 2010.12.09
#define COOR_SB_MSG_EVT                     0x0010       //扩展传感器板控制事件
#define COOR_PB_MSG_EVT                     0x0020       //电池板控制事件

/* 全局变量 */
static uint8 appState =             APP_INIT;           //应用层处理事件状态
static uint8 applinkState =         SEND_SEV;           //与中间服务器建立通信事件状态
static uint8 appcoorrevState =      UNALLOW_COOR_REV;   //协调器接收路由节点消息状态
//static uint8 reportState =          FALSE;            //发送数据状态
static uint8 isGateWay =            FALSE;              //协调器模式
//static uint16 myReportPeriod =      500;              
static uint8 WSNNumber = 0;                             //设备序号
static uint16 mywsnid = 0;                              //会话ID （由设备序号和设备类型组合而成）
static uint8 waittimeout = 0;                           //设备识别等待超时
static uint8 heartbeattimeout = 0;                      //心跳超时

static uint8 reportFailureNr = 0;                       //数据包发送失败记录
static uint16 parentShortAddr = 0;                      //父亲设备的短地址
extern uint8 motherboardtype;                           //母板类型
extern uint8 resetpanid;
extern uint16 mysetpanid;

static uint8 adchannel = 0;                             //ADC通道
static uint16 SampleMode = 0;                           //采样模式
static uint16 SamplingSpeed = 0;                        //采样速度
static uint16 SensormodeID = 0;                         //模块ID
static uint8 appControlobject = 0;                      //扩展模块所需采样功能

/******************************************************************************
 * LOCAL FUNCTIONS
 */
static void sendDummyReport(uint8 *pData,uint16 Addr,uint8 len);
static void SEVuartRxHandle(uint8 *MSGCode,uint8 num);
static void APPuartRxHandle(uint8 *MSGCode,uint8 num);
extern void ctrPCA9554FLASHLED4(uint8 FLASHnum);   
extern void ctrPCA9554FLASHLED5(uint8 FLASHnum);  
extern void ctrPCA9554LED(uint8 led,uint8 operation);
extern void ctrPCA9554Buzzer(uint8 operation);
extern uint8 ctrPCA9554Relay(uint8 cmd);
extern uint8 ctrPCA9554SRelay(uint8 cmd);
extern uint8 read24L01byte(uint8 addr);
extern void th_read(int *t,int *h );
extern uint8 Ctr5573VoltageOutput(uint16 Voltage,uint8 Port);
static void FandSCheck(uint8 *pBuf);
static void SendDeviceInfo(void);
static void SensorControl(uint16 SensorID,uint8 *ControlMSG);
static void ADCSamplingControl(uint8 channel,uint16 SampleMode,uint16 SamplingSpeed,uint16 SensorID);
static void ResetAllNodeApp(void);
static void MotherboardControl(uint16 SensorID,uint8 *ControlMSG);
static void SendSensorInfo(void);

/******************************************************************************
 * GLOBAL VARIABLES
 */
// 协调器的输入输出命令数
#define NUM_OUT_CMD_COLLECTOR                2
#define NUM_IN_CMD_COLLECTOR                 2

// 协调器输入命令列表
const cId_t zb_InCmdList[NUM_IN_CMD_COLLECTOR] =
{
  SENSOR_REPORT_CMD_ID,
  DUMMY_REPORT_CMD_ID
};
// 协调器输出命令列表
const cId_t zb_OutCmdList[NUM_IN_CMD_COLLECTOR] =
{
  SENSOR_REPORT_CMD_ID,
  DUMMY_REPORT_CMD_ID
};

//协调器简单描述
const SimpleDescriptionFormat_t zb_SimpleDesc =
{
  MY_ENDPOINT_ID,             //  Endpoint
  MY_PROFILE_ID,              //  Profile ID
  DEV_ID_COLLECTOR,           //  Device ID
  DEVICE_VERSION_COLLECTOR,   //  Device Version
  0,                          //  Reserved
  NUM_IN_CMD_COLLECTOR,       //  Number of Input Commands
  (cId_t *) zb_InCmdList,     //  Input Command List
  NUM_OUT_CMD_COLLECTOR,      //  Number of Output Commands
  (cId_t *) zb_OutCmdList     //  Output Command List
};

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
  uint8 logicalType;
   
   if ( appState == APP_INIT  )                                               //协调器第一次启动时，设置设备类型为协调器，然后重启设备
      {
       zb_ReadConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType);  //获取设备类型
        if (logicalType != ZG_DEVICETYPE_COORDINATOR)
        {
          logicalType = ZG_DEVICETYPE_COORDINATOR;                             //设置设备类型为协调器
          zb_WriteConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType); //写入NV中                
          zb_SystemReset();                                                    //重启设备
       }
      }
   
  if(event & SYS_EVENT_MSG)                                                   //系统信息事件（一般无操作）
  {
    
  }
  
  if( event & ZB_ENTRY_EVENT )                                               //进入网络建立和一些用户需要使用的外围设备初始化事件
  {                                                                          //该事件在执行SAPI_Init函数的最后注册
    initUart(uartRxCB,HAL_UART_BR_115200);                                    //初始化串口
                                  
    ctrPCA9554LED(5,1);                                                      //点亮LED1 指示将启动建立网络
    
    zb_ReadConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType);  //获取设备类型

    zb_StartRequest();                                                       //启动设备建立网络（在该启动属于设备的自动启动，需编译HOLD_AUTO_START选项）
  }
  
  if ( event & MY_START_EVT )                                                //用户自定义启动设备事件，当启动确认函数zb_StartConfirm 的状态为非成功状态时，触发该事件重启设备
  {
    zb_StartRequest();
  }
  
  if ( event & MY_REPORT_EVT )                                               //报告本设备采集到的信息事件
  {
    
    if (isGateWay)                                                           //网关设备发送数据
    {
      
    }
    else if(appState == APP_BINDED)                                         //非网关设备
    {   
    //  osal_start_timerEx( sapi_TaskID, MY_REPORT_EVT, myReportPeriod );
    }
  }
  if ( event & MY_FIND_COLLECTOR_EVT )                                       // 非协调器设备与协调器建立绑定事件
  { 
    if (!isGateWay) 
    {
      zb_BindDevice( TRUE, DUMMY_REPORT_CMD_ID, (uint8 *)NULL );             //与协调器建立绑定
    }
  }
  if ( event & MY_COOR_LINK_SEV_EVT )                                        //PC机连接，设备识别过程
  {
    if (applinkState == SEND_SEV)                                            //向SEV发送设备识别消息状态
    {
      SEVLinkCoordPacket_t SEVLinkCoordPacket;                                  //定义设备识别数据信息包
    
      SEVLinkCoordPacket.Hdr = '@';                                             //WSN与PC机连接信息赋值
      SEVLinkCoordPacket.Len = 10;
      SEVLinkCoordPacket.TransportID = 0;
      SEVLinkCoordPacket.MSGCode = EQU_ID_CODE;
      SEVLinkCoordPacket.AppType = 1;
      SEVLinkCoordPacket.EQUType = 1;
      SEVLinkCoordPacket.Heartbeat_Timeout = 30;
      SEVLinkCoordPacket.Heartbeat_Period = 5;
      SEVLinkCoordPacket.Checksum = 0x80;
    
      HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SEVLinkCoordPacket, sizeof(SEVLinkCoordPacket));  //通过串口向PC机发送设备识别信息
      waittimeout = 0;
      heartbeattimeout = 0;
      appcoorrevState = UNALLOW_COOR_REV;
      if (appControlobject)
      {
       osal_stop_timerEx( sapi_TaskID, COOR_SB_MSG_EVT ); 
       appControlobject = 0;
      }
      osal_start_timerEx( sapi_TaskID, MY_COOR_LINK_SEV_EVT, 500 );          //设置时间500ms
    }
    
    else if (applinkState == SEND_HEARTBEAT)                                 //发送心跳状态
    {
       SEVGeneralMSGPacket_t SEVhearbeatPacket;
       
       SEVhearbeatPacket.Hdr = '@';
       SEVhearbeatPacket.Len = 6;
       SEVhearbeatPacket.TransportID = 0;
       SEVhearbeatPacket.MSGCode = HEARTBEAT_DETECT;
       SEVhearbeatPacket.Checksum = 0x47;
       
       HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SEVhearbeatPacket, sizeof(SEVhearbeatPacket));  //通过串口发送心跳信息
            
       ++heartbeattimeout;                                                    //心跳接收计时，心跳超时为15s  
       if (heartbeattimeout == 6)
       {
         applinkState = SEND_SEV;                                             //接收心跳超时，设置为向SEV发送设备识别消息状态
         osal_stop_timerEx( sapi_TaskID, COOR_SB_MSG_EVT ); 
        // ResetAllNodeApp();           //发送让所有节点复位消息
       }
       osal_start_timerEx( sapi_TaskID, MY_COOR_LINK_SEV_EVT, 5000 );        //设置心跳周期5s
    }
    else if (applinkState == REV_SEV_MSG)                                    //等待SEV发送设备序号消息状态
    {
      SEVMSGReplyPacket_t SEVMSGReplyPacket;
       if(WSNNumber)                                                         //收到设备序号消息
       {
         mywsnid = BUILD_UINT16(WSNNumber,MY_WSN_TYPE);                      //组合会话ID 
         applinkState = SEND_HEARTBEAT;                                      //收到设备序号消息，设置为发送心跳状态
         WSNNumber = 0;
         SEVMSGReplyPacket.Hdr = '@';    
         SEVMSGReplyPacket.Len = 7;
         SEVMSGReplyPacket.TransportID = 0;
         SEVMSGReplyPacket.MSGCode = EQU_NUMBER_CODE;
         SEVMSGReplyPacket.ReplyCode = 0;
         SEVMSGReplyPacket.Checksum = 0x59;         
         HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SEVMSGReplyPacket, sizeof(SEVMSGReplyPacket));  // 回答
       }
       else
       {
         waittimeout++;
       }      
      if (waittimeout == 12)
      {
        applinkState = SEND_SEV;   
      } 
      osal_start_timerEx( sapi_TaskID, MY_COOR_LINK_SEV_EVT, 500 );           //设置时间500ms
    } 
  }
  
  if ( event & COOR_SB_MSG_EVT )                                              //传感器板控制事件
  {
    if (appControlobject == APP_ADC)                                          //使用AD获取数据的传感器
    {
     ADCSamplingControl(adchannel,SampleMode,SamplingSpeed,SensormodeID);    //调用AD采样控制函数
    }
    else if (appControlobject == APP_IIC)                                    //使用IIC获取数据的传感器或控制
    {
      
    }
    else if (appControlobject == APP_IIC_CPU)                                //使用IIC和外部CPU获取数据的传感器或控制
    {
      
    }
    
  }
  
  if ( event & COOR_PB_MSG_EVT )                                             //电池板控制事件
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
static uint16 zgConfigPANIDSetting = 0;
static uint8 mypanid = 0;
void zb_HandleKeys( uint8 shift, uint8 keys )
{
  static uint8 allowBind=FALSE;
  static uint8 allowJoin=TRUE;
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
    if ( keys & HAL_KEY_SW_1 )                                                //SW1按键事件处理
    {
      mypanid = 1;
      if(mypanid)                                                             //设备PANID设置
      {
        zgConfigPANIDSetting = 0;
        GUI_ClearScreen();                                                    //LCD清屏
        GUI_PutString5_7(25,6,"OURS-CC2530");                                 //在LCD上显示相应的文字             
        GUI_PutString5_7(5,22," ZigBee PRO");                          
        GUI_PutString5_7(5,35,"PANID Setting");
        GUI_PutString5_7(5,48," 0");
        LCM_Refresh();
      }
    }
    if ( keys & HAL_KEY_SW_2 )                                               //SW2按键事件处理 
    {
      if(mypanid)                                                            //PANID设置时，为加1功能
      {
        zgConfigPANIDSetting += 1;
        sprintf(s,(char *)" %d          ",zgConfigPANIDSetting);  
         GUI_PutString5_7(5,48,(char *)s);                                   //显示结果
         LCM_Refresh();
      }
      else                                                                   //协调器类型选择（网关设备或普通协调器）
      {
       allowBind ^= 1;
       if (allowBind)                                                         //设置为协调器模式
       {
        zb_AllowBind( 0x00 );                                                //关闭绑定功能（此应用中没有开启使用多个协调器功能）
        isGateWay = FALSE;       
        #if defined ( LCD_SUPPORTED )                                        //更新显示内容
        GUI_SetColor(1,0);
        GUI_ClearScreen();
        LCM_Refresh();
        GUI_PutString5_7(5,8,"OURS-CC2530");
        GUI_PutString5_7(5,30,"SensorDemo");
        GUI_PutString5_7(5,50,"Collector");
        LCM_Refresh();             
        #endif
       }
       else                                                                   //设置为网关模式（即可与PC机通信的协调器）
       {       
        zb_AllowBind( 0xFF );                                                //打开绑定功能
        isGateWay = TRUE;
        #if defined ( LCD_SUPPORTED )                                        //更新显示内容
        GUI_SetColor(1,0);
        GUI_ClearScreen();
        LCM_Refresh();
        GUI_PutString5_7(5,8,"OURS-CC2530");
        GUI_PutString5_7(5,30,"SensorDemo");
        GUI_PutString5_7(5,50,"Gateway Mode");
        LCM_Refresh();
        #endif
       }
      }
    }
    if ( keys & HAL_KEY_SW_3 )                                               //SW3按键处理事件
    {
      if(mypanid)                                                            //PANID设置时，为加10功能
     {
       zgConfigPANIDSetting += 10;
       sprintf(s,(char *)" %d          ",zgConfigPANIDSetting);  
         GUI_PutString5_7(5,48,(char *)s);                                   //显示结果
         LCM_Refresh();
       
     }              
    }
    if ( keys & HAL_KEY_SW_4 )                                               //SW4按键处理事件
    {
      if(mypanid)                                                            //PANID设置时，为加100功能
      {
        zgConfigPANIDSetting += 100;
        sprintf(s,(char *)" %d          ",zgConfigPANIDSetting);  
         GUI_PutString5_7(5,48,(char *)s);                                   //显示结果
         LCM_Refresh();
      }
      else                                                                   //允许或不允许节点加入网络
      {
      allowJoin ^= 1;
      if(allowJoin)
      {
        NLME_PermitJoiningRequest(0xFF);
      }
      else {
        NLME_PermitJoiningRequest(0);
      }
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
     if ( keys & HAL_KEY_SW_6 )                                              //SW6按键处理事件
    {
      if(mypanid)                                                            //PANID设置时，为设置完成键
      {
        if(zgConfigPANIDSetting)
        {       
          NLME_InitNV();        //清除NV
          NLME_SetDefaultNV(); //设置默认NV条目  
          if ( osal_nv_item_init( ZCD_NV_PANID, 2, &zgConfigPANID )== ZSUCCESS ) 
         {            
           zgConfigPANID = zgConfigPANIDSetting;  
           osal_nv_write( ZCD_NV_PANID, 0, sizeof(zgConfigPANID), &zgConfigPANID );             
           zgWriteStartupOptions( ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_NETWORK_STATE );
           zb_SystemReset(); 
         } 
        }
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
  if ( status == ZB_SUCCESS )                                                //判断设备是否启动成功（成功和失败都将调用）
  {   
     zb_AllowBind( 0xFF );                                                   //允许设备绑定 add by wu 2010.9.15
     ctrPCA9554LED(4,1);                            
     isGateWay = TRUE;                                                       //设置该设备为网关设备（协调器）  
     appState = APP_START;                                                    //设置设备在APP层为启动状态
    
     osal_set_event( sapi_TaskID, MY_FIND_COLLECTOR_EVT );                    //与协调器绑定（即非网关协调器）
       
     zb_GetDeviceInfo(ZB_INFO_PARENT_SHORT_ADDR, &parentShortAddr);           //获取该节点的父亲短地址
       
    #if defined ( LCD_SUPPORTED )                                            //更新LCD显示内容
        GUI_SetColor(1,0);
        GUI_ClearScreen();
        LCM_Refresh();       
        osal_nv_read(ZCD_NV_PANID, 0, 2, &zgConfigPANID);
        sprintf(s,(char *)" %d     ",zgConfigPANID);
        GUI_PutString5_7(50,44,(char *)s);                
        GUI_PutString5_7(25,8,"OURS-CC2530");
        GUI_PutString5_7(5,20,"SensorDemo");
        GUI_PutString5_7(5,32,"Gateway Mode");
        GUI_PutString5_7(5,44,"PANID:");
        LCM_Refresh();
    #endif
    osal_set_event( sapi_TaskID, MY_COOR_LINK_SEV_EVT );                     // 启WSN协调器与PC连接的设备识别信息请求
    applinkState = SEND_SEV;                                                 //设置为SEV发送设备识别消息
  }
  else                                                                       //启动失败
  {
    osal_start_timerEx( sapi_TaskID, MY_START_EVT, 10 );                    //延时一段时间后，触发MY_START_EVT事件，重启设备
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
  if ( status != ZB_SUCCESS && !isGateWay )                                  //发送失败
  {
    if ( ++reportFailureNr>=REPORT_FAILURE_LIMIT )                           //失败次数大于等于4次
    {   
      // osal_stop_timerEx( sapi_TaskID, MY_REPORT_EVT );                      //停止数据发送，并延时一段时间
       osal_stop_timerEx( sapi_TaskID, MY_COOR_LINK_SEV_EVT );
            
       zb_BindDevice( FALSE, DUMMY_REPORT_CMD_ID, (uint8 *)NULL );           //删除原来的绑定
       
       osal_set_event( sapi_TaskID, MY_FIND_COLLECTOR_EVT );                 //尝试寻找新的设备，重新绑定设备
       reportFailureNr=0;                                                    //失败次数计数清零
       ResetAllNodeApp();                                                    
    }
  }
  else if ( !isGateWay )                                                     //成功
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
    appState = APP_BINDED;                                                   //设置为绑定状态
    
     // osal_set_event( sapi_TaskID, MY_REPORT_EVT );                          //设置协调器本地报告

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
 *           lqi -节点LQI值
 *           rssi —节点RSSI值
 *           len - 接收的数据包长度
 *           pData - 数据缓存指针
 *
 * 返 回 值：无
 **************************************************************************************************/
void zb_ReceiveDataIndication( uint16 source, uint16 command, uint8 lqi, int8 rssi,uint16 len, uint8 *pData  )
{ 
  uint8 i,num = 0;
  
  if(appcoorrevState == ALLOW_COOR_REV)                                      //是否允许协调器接收消息 
  {       
   if(len == *(pData+1))
   {       
      *(pData+2) = LO_UINT16(mywsnid);
      *(pData+3) = HI_UINT16(mywsnid);
      *(pData+len-1) += (HI_UINT16(mywsnid)+LO_UINT16(mywsnid));
      if (*(pData+4) == SENDUP_NODE_INFO)
      {
        *(pData+15) = rssi;
        *(pData+16) = lqi;
      }
      for(i=0;i<(len-1);i++)
      {
        num += *(pData+i);
      }
      *(pData+len-1) = num;
      HalUARTWrite(HAL_UART_PORT_0,(uint8 *)pData, *(pData+1));             //串口转发
     // if (*(pData+4) == SENDUP_NODE_PADDR_ADDR) //父子关系校验
     // {
     //   FandSCheck(pData);    //调用校验函数
    // }
   } 
   ctrPCA9554FLASHLED5(1);
  }  
}

/**************************************************************************************************
 * 函数名称：FandSCheck
 *
 * 功能描述：用于校验父子关系的正确性
 *
 * 参    数：pBuf - 需要校验的节点关系地址数据
 *           
 *
 * 返 回 值：无
 **************************************************************************************************/
static uint8 FandSCheckcount = 0;
static uint16 FandSCheckpaddr = 0;
static uint16 FandSCheckaddr = 0;
static void FandSCheck(uint8 *pBuf)   
{
  PCNodeAddrPacket_t PCNodeAddrPacket;
  PCGeneralMSGPacket_t PCGeneralMSGPacket;
  uint8 i,num = 0;
  osal_memcpy((void *)&PCNodeAddrPacket,pBuf,10);      //保存父子节点关系信息
   if(FandSCheckcount)
  {
     if(FandSCheckpaddr==PCNodeAddrPacket.NodeAddr)    //校验父亲节点是否存在
     {
       FandSCheckcount = 0;                            //存在，则开始新的校验
     }
     else                                              //校验次数加1
     {
        FandSCheckcount++;
     }
     if (FandSCheckcount == 12)                       //校验次数到10次后，没有收到其父亲节点信息，认为该父亲节点不存在
     {
        FandSCheckcount = 0;                          //发送重启该节点信息
        PCGeneralMSGPacket.Hdr = '@';
        PCGeneralMSGPacket.Len = 8;
        PCGeneralMSGPacket.TransportID = PCNodeAddrPacket.TransportID;
        PCGeneralMSGPacket.MSGCode = NODE_SYSTEMRESET;
        PCGeneralMSGPacket.NodeAddr = FandSCheckaddr;
        for(i=0;i<7;i++)                              //计算信息包校验和
       {
        num = num + ((uint8*)(&PCGeneralMSGPacket))[i];
       }
        PCGeneralMSGPacket.Checksum = num;
       sendDummyReport((uint8*)(&PCGeneralMSGPacket),PCGeneralMSGPacket.NodeAddr,8); //无线发送重启命令     
     }
   }
   else
  {
    if(PCNodeAddrPacket.NodePAddr)                 //判断父节点地址是否为协调器          
    {
     FandSCheckpaddr = PCNodeAddrPacket.NodePAddr;  //保存校验节点的父地址
     FandSCheckaddr = PCNodeAddrPacket.NodeAddr;    //保存校验节点地址
     FandSCheckcount++;                             //开始校验
    }
    else
    {
      FandSCheckcount = 0;                          //父节点地址为协调器不需要校验
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
  uint16 len;
  uint8 Checksum = 0;
  uint8 i;
  
  if ( event != HAL_UART_TX_EMPTY )                                          //串口接收事件确认
  {   
    len = HalUARTRead( HAL_UART_PORT_0, pBuf, RX_BUF_LEN );                  //读数据
    
    if ( len > 0 ) 
    {
      if ( pBuf[0] == '@')                                                
      {
        for(i=0;i<(pBuf[1] - 1);i++)                                          //计算校验和    
        {
         Checksum += pBuf[i];
        }
        if ( pBuf[2] )                                                       //与应用层的通信                                                   
        {
          APPuartRxHandle(pBuf,Checksum);
        }
        else                                                                  //与中间服务层的通信
        {
          SEVuartRxHandle(pBuf,Checksum);
        }                                               
      }
    }
  }
}

/**************************************************************************************************
 * 函数名称：SEVuartRxHandle
 *
 * 功能描述：处理协调器接收到中间服务器发送来的信息
 *
 * 参    数：MSGCode - UART接收到的信息包
 *           num - 信息包的校验和       
 *
 * 返 回 值：无
 **************************************************************************************************/
static void SEVuartRxHandle(uint8 *MSGCode,uint8 num)
{
  SEVMSGReplyPacket_t SEVMSGReplyPacket;
  SEVEQUNumberPacket_t SEVEQUNumberPacket;
  SEVGeneralMSGPacket_t SEVGeneralMSGPacket;
  switch (MSGCode[4])
  {
    case HEARTBEAT_DETECT:   //心跳检测
      osal_memcpy((void *)&SEVGeneralMSGPacket,&MSGCode[0],6);
     if(SEVGeneralMSGPacket.Checksum == num)
     {
       heartbeattimeout = 0;            
     }break;
     
    case FIRST_APP_LINK:     //第一个上层应用连接服务
      osal_memcpy((void *)&SEVGeneralMSGPacket,&MSGCode[0],6);
     if(SEVGeneralMSGPacket.Checksum == num)
     {
       appcoorrevState = ALLOW_COOR_REV;  
       
       SEVMSGReplyPacket.Hdr = '@';    
       SEVMSGReplyPacket.Len = 7;
       SEVMSGReplyPacket.TransportID = 0;
       SEVMSGReplyPacket.MSGCode = FIRST_APP_LINK;
       SEVMSGReplyPacket.ReplyCode = 0;
       SEVMSGReplyPacket.Checksum = 0x49;
       HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SEVMSGReplyPacket, sizeof(SEVMSGReplyPacket));  // 回答
     if (isGateWay)  //如果是GateWay模式 则改变显示内容
    {
     // Update the display
     #if defined ( LCD_SUPPORTED )   
     GUI_SetColor(1,0);
     GUI_ClearScreen();
     LCM_Refresh();
     GUI_PutString5_7(5,8,"OURS-CC2530");
     GUI_PutString5_7(5,22,"SensorDemo");
     GUI_PutString5_7(5,44,"Report rcvd");
     LCM_Refresh();
    #endif
     }
     }break;
      
    case LAST_APP_ULINK:     //最后一个上层应用与服务断开
      osal_memcpy((void *)&SEVGeneralMSGPacket,&MSGCode[0],6);
     if(SEVGeneralMSGPacket.Checksum == num)
     {
       appcoorrevState = UNALLOW_COOR_REV;    
       
       SEVMSGReplyPacket.Hdr = '@';
       SEVMSGReplyPacket.Len = 7;
       SEVMSGReplyPacket.TransportID = 0;
       SEVMSGReplyPacket.MSGCode = LAST_APP_ULINK;
       SEVMSGReplyPacket.ReplyCode = 0;
       SEVMSGReplyPacket.Checksum = 0x4A;
       HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SEVMSGReplyPacket, sizeof(SEVMSGReplyPacket)); //回答
       osal_stop_timerEx( sapi_TaskID, COOR_SB_MSG_EVT );     //停止本地报告信息
     }break;
      
    case EQU_ID_CODE:        //服务器应答消息
     osal_memcpy((void *)&SEVMSGReplyPacket,&MSGCode[0],7);
     if(SEVMSGReplyPacket.Checksum == num)
     {
       if (SEVMSGReplyPacket.ReplyCode == 0) //应答成功
       {
         applinkState = REV_SEV_MSG;  //设置为等待SEV发送设备序号消息
       }
       else      //应答失败
       {
         
       }
     }break;
     
    case EQU_NUMBER_CODE:     //设备序号消息
     osal_memcpy((void *)&SEVEQUNumberPacket,&MSGCode[0],7);
     if(SEVEQUNumberPacket.Checksum == num)
     {
       WSNNumber = SEVEQUNumberPacket.NumberCode;
              
     }break;
     
     default: break;
  }
}

/**************************************************************************************************
 * 函数名称：APPuartRxHandle
 *
 * 功能描述：处理协调器接收到PC端应用层软件发送来的信息
 *
 * 参    数：MSGCode - UART接收到的信息包
 *           num - 信息包的校验和       
 *
 * 返 回 值：无
 **************************************************************************************************/
static void APPuartRxHandle(uint8 *MSGCode,uint8 num)
{ 
  if(appcoorrevState == ALLOW_COOR_REV)     //是否与服务器层建立连接
  {
    static PCGeneralMSGPacket_t PCGeneralMSGPacket;
    static SendDownSBoardDataPacket_t SendDownSBoardDataPacket;
    static SendDownTemBoardDataPacket_t SendDownTemBoardDataPacket; 
    static SetPANIDPacket_t SetPANIDPacket;
    if (MSGCode[4] <= 0x10)      //一般消息
    {    
       osal_memcpy((void *)&PCGeneralMSGPacket,&MSGCode[0],8);   //保存数据
      if(PCGeneralMSGPacket.Checksum == num)                     //数据包校验
     {
       if(PCGeneralMSGPacket.NodeAddr)                          //非本地信息
       {
        sendDummyReport((uint8*)(&PCGeneralMSGPacket),PCGeneralMSGPacket.NodeAddr,8); //无线发送
       }
       else          //本地信息
       {     
          if (PCGeneralMSGPacket.MSGCode == FETCH_NODE_INFO)         //获取协调器信息
          {
             SendDeviceInfo();
          }
          else if (PCGeneralMSGPacket.MSGCode == NODE_SYSTEMRESET)        //重启协调器
          {
             zb_SystemReset(); 
          }
          else if (PCGeneralMSGPacket.MSGCode == FETCH_NODE_SENSOR_CAL)   //获取传感器校准参数
          {
            SendSensorInfo();
          }
       }
     }
    }
    else if(MSGCode[4] > 0x20)  //其他消息
    {
      switch (MSGCode[4])
      {        
        case SENDDOMN_NODE_SBOARD_DATA:   //PC下传节点扩展板资源数据  
        osal_memcpy((void *)&SendDownSBoardDataPacket,&MSGCode[0],MSGCode[1]);   //保存数据
          if(SendDownSBoardDataPacket.NodeAddr)                          //非本地信息
         {
          sendDummyReport((uint8*)&MSGCode[0],SendDownSBoardDataPacket.NodeAddr,
                          SendDownSBoardDataPacket.Len); //无线发送
          }
         else          //本地信息
         { 
            SensorControl(SendDownSBoardDataPacket.ModeID,&MSGCode[9]); //本地扩展板控制
         }
        break;
        case SENDDOMN_NODE_BBOARD_DATA:   //PC下传节点底板资源数据
        osal_memcpy((void *)&SendDownTemBoardDataPacket,&MSGCode[0],MSGCode[1]);   //保存数据
          if(SendDownTemBoardDataPacket.NodeAddr)                          //非本地信息
         {
          sendDummyReport((uint8*)(&MSGCode[0]),SendDownTemBoardDataPacket.NodeAddr,
                          SendDownTemBoardDataPacket.Len); //无线发送
          }
         else          //本地信息
         {
           MotherboardControl(SendDownTemBoardDataPacket.ResID,&MSGCode[10]); //本地母板控制
           
         }
        break;
        case SENDDOMN_GROUPNODE_LIST:   //PC节点分组消息(暂无)
          
        break;
        case SET_NODE_PANID:
          osal_memcpy((void *)&SetPANIDPacket,&MSGCode[0],MSGCode[1]);   //保存数据
           if(SetPANIDPacket.NodeAddr)                          //非本地信息
         {
          sendDummyReport((uint8*)(&MSGCode[0]),SetPANIDPacket.NodeAddr,
                          SetPANIDPacket.Len); //无线发送
          }
         else          //本地信息
         {
           if(SetPANIDPacket.SETPANID)
          {
            if(SetPANIDPacket.SETPANID < 0x3fff)
            {   
               NLME_InitNV();        //清除NV
               NLME_SetDefaultNV(); //设置默认NV条目  
              if ( osal_nv_item_init( ZCD_NV_PANID, 2, &zgConfigPANID )== ZSUCCESS ) 
             {            
               zgConfigPANID = SetPANIDPacket.SETPANID;  
               osal_nv_write( ZCD_NV_PANID, 0, sizeof(zgConfigPANID), &zgConfigPANID );             
               zgWriteStartupOptions( ZG_STARTUP_SET, ZCD_STARTOPT_DEFAULT_NETWORK_STATE );
               zb_SystemReset(); 
              } 
            }
          }         
         }
        break;
        default : break;
      }
    }
     else   //一般群发消息(暂无)
     {
       
     }
  }
}

/**************************************************************************************************
 * 函数名称：sendDummyReport
 *
 * 功能描述：协调器无线发送数据
 *
 * 参    数：pData - 发送的数据包信息
 *           Addr - 节点地址
 *           len - 数据包长度
 *
 * 返 回 值：无
 **************************************************************************************************/
static void sendDummyReport(uint8 *pData,uint16 Addr,uint8 len)
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

  zb_SendDataRequest( 0xFFFF, Addr, len, pData, 0, txOptions, 0 );
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
   uint8 *myIEEEAddr = NULL;
   uint8 i;
   PCNodeMSGPacket.Hdr = '@';
   PCNodeMSGPacket.Len = 23;
   PCNodeMSGPacket.TransportID = mywsnid;
   PCNodeMSGPacket.MSGCode = SENDUP_NODE_INFO;
   PCNodeMSGPacket.NodeAddr = 0;
   zb_GetDeviceInfo(ZB_INFO_IEEE_ADDR, myIEEEAddr);                         //获取设备IEEE地址 add by wu 2010.11.30
    for(i=0;i<8;i++)
   {
      PCNodeMSGPacket.IEEEAddr[i] = *(myIEEEAddr+i);
   }
   PCNodeMSGPacket.NodeRSSI = 0;
   PCNodeMSGPacket.NodeLQI = 0;
   osal_nv_read(ZCD_NV_PANID, 0, 2, &zgConfigPANID);
   PCNodeMSGPacket.Panid = zgConfigPANID;
   PCNodeMSGPacket.TemType = motherboardtype;             //母板类型
   if (sensoridH == 0)
   {
    sensoridH = read24L01byte(0x01);
    sensoridL = read24L01byte(0x02);
     if((sensoridH == 0)&&(sensoridL == 0))                //读不出来信息，即为电压检测模块
    {
      if (HalAdcRead (HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_14)>3000)//判断是否有电压检测模块（暂时）
     {
      sensoridH = 0x10;
      sensoridL = 0x01;
     }
    }
     if((sensoridH == 0x30)&&(sensoridL == 0x00))          //读取协调器串口代码
   {
     sensoridH = read24L01byte(0x03);
     sensoridL = read24L01byte(0x04);
   }
   }
   PCNodeMSGPacket.SBoardType = BUILD_UINT16(sensoridL,sensoridH);  //扩展传感板类型
   PCNodeMSGPacket.Checksum = 0;
    for(i=0;i<22;i++)
   {
      PCNodeMSGPacket.Checksum += ((uint8*)(&PCNodeMSGPacket))[i];
   }  
   HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&PCNodeMSGPacket, sizeof(PCNodeMSGPacket));            //通过串口向PC机发送数据
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
  uint8 i;
  SensorCalPacket_t SensorCalPacket = {0x40,16,mywsnid,SENSOR_DATA_CAL,0,0,0,0,0,0};
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
  for(i=0;i<16;i++)
   {
      SensorCalPacket.Checksum += ((uint8*)(&SensorCalPacket))[i];
   } 
  HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SensorCalPacket, sizeof(SensorCalPacket));            //通过串口向PC机发送数据
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
static void SensorControl(uint16 SensorID,uint8 *ControlMSG)
{
   ADControlData_t ADControlData;
   DAControlData_t DAControlData;
  SendUpSBoardDataPacket2_t SendUpSBoardDataPacket2;
  uint8 i,num = 0;
  uint8 Relaystate = 0;
  SendUpSBoardDataPacket2.Hdr = '@';
  SendUpSBoardDataPacket2.TransportID = mywsnid;
  SendUpSBoardDataPacket2.MSGCode = SENDUP_NODE_SBOARD_DATA;
  SendUpSBoardDataPacket2.NodeAddr = 0;
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
      osal_set_event( sapi_TaskID, COOR_SB_MSG_EVT );         
    break;
    case resCode_Voltage_Dual:       
      adchannel = 0xC0;
      appControlobject = APP_ADC;
      osal_set_event( sapi_TaskID, COOR_SB_MSG_EVT );
    break;
    case resCode_Current_Single:
      adchannel = 0x40;
      appControlobject = APP_ADC;
      osal_set_event( sapi_TaskID, COOR_SB_MSG_EVT );
    break;
    case resCode_Current_Dual:
      adchannel = 0xC0;
      osal_set_event( sapi_TaskID, COOR_SB_MSG_EVT );
    break;
    case resCode_Light_Single:
      adchannel = 0x80;
      appControlobject = APP_ADC;
      osal_set_event( sapi_TaskID, COOR_SB_MSG_EVT );
    break;
    case resCode_Pressure:
      adchannel = 0x80;
      appControlobject = APP_ADC;
      osal_set_event( sapi_TaskID, COOR_SB_MSG_EVT );
    break;
    case resCode_Alcohol:
      adchannel = 0x80;
      appControlobject = APP_ADC;
      osal_set_event( sapi_TaskID, COOR_SB_MSG_EVT );
    break;
    case resCode_Pressure_Alcohol:
      adchannel = 0xC0;
      appControlobject = APP_ADC;
      osal_set_event( sapi_TaskID, COOR_SB_MSG_EVT );
    break;
    case resCode_Temp_Humidity:
      
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
      osal_set_event( sapi_TaskID, COOR_SB_MSG_EVT );
    break;
    case resCode_Voltage_Output:
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
     for(i = 0;i < (SendUpSBoardDataPacket2.Len - 1);i++) 
     {
      num += ((uint8*)(&SendUpSBoardDataPacket2))[i];
     }
    ((uint8*)(&SendUpSBoardDataPacket2))[SendUpSBoardDataPacket2.Len - 1] = num;    
    HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SendUpSBoardDataPacket2, SendUpSBoardDataPacket2.Len);  //通过串口向PC机发送设备识别信息
    break;
    case resCode_Photoelectric:
      
    break;
    case resCode_IR_Output:
     
    break;
    case resCode_IR_Photoelectric:
      
    break;
    case resCode_Ultrasonic:
     
    break;    
    default:break;
   } 
}


/**************************************************************************************************
 * 函数名称：MotherboardControl
 *
 * 功能描述：解析PC端发送的底板板控制信息，然后传递给相应的控制函数。
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
  SendUpTemBoardDataPacket_t SendUpTemBoardDataPacket;
  uint8 i,num = 0;
  uint8 Relaystate = 0;
  SendUpTemBoardDataPacket.Hdr = '@';
  SendUpTemBoardDataPacket.TransportID = mywsnid;
  SendUpTemBoardDataPacket.MSGCode = SENDUP_NODE_BBOARD_DATA;
  SendUpTemBoardDataPacket.NodeAddr = 0;
  SendUpTemBoardDataPacket.TemType = motherboardtype;
  SendUpTemBoardDataPacket.Checksum = 0;
  switch(SensorID)
  {
    case resCode_Relay:
     Relaystate = ctrPCA9554Relay(*ControlMSG);
      SendUpTemBoardDataPacket.data = (uint8 *)Relaystate;
      SendUpTemBoardDataPacket.Len = 12;
      SendUpTemBoardDataPacket.ResID = resCode_Relay;     
     for(i = 0;i < (SendUpTemBoardDataPacket.Len - 1);i++) 
     {
      num += ((uint8*)(&SendUpTemBoardDataPacket))[i];
     }
    ((uint8*)(&SendUpTemBoardDataPacket))[SendUpTemBoardDataPacket.Len - 1] = num;    
    HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SendUpTemBoardDataPacket, SendUpTemBoardDataPacket.Len);  //通过串口向PC机发送回复信息
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
     for(i = 0;i < (SendUpTemBoardDataPacket.Len - 1);i++) 
     {
      num += ((uint8*)(&SendUpTemBoardDataPacket))[i];
     }
    ((uint8*)(&SendUpTemBoardDataPacket))[SendUpTemBoardDataPacket.Len - 1] = num;    
    HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SendUpTemBoardDataPacket, SendUpTemBoardDataPacket.Len);  //通过串口向PC机发送回复信息
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
    for(i = 0;i < (SendUpTemBoardDataPacket.Len - 1);i++) 
   {
     num += ((uint8*)(&SendUpTemBoardDataPacket))[i];
   }
    ((uint8*)(&SendUpTemBoardDataPacket))[SendUpTemBoardDataPacket.Len - 1] = num;  
   
   HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SendUpTemBoardDataPacket, SendUpTemBoardDataPacket.Len);  //通过串口向PC机发送回复信息
    break;
    case resCode_AD:
    
    break;
   default:break;
  }
}
/**************************************************************************************************
 * 函数名称：ADCSamplingControl
 *
 * 功能描述：根据PC发送的AD采用命令，采集各个传感器模块的传感器数据
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
   uint8 i,num = 0;
   int tempera;
   int humidity;
    if (channel == 0x80)  //使用0通道
   {  
      SendUpSBoardDataPacket.data = (uint16*)HalAdcRead (HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_14);
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
   SendUpSBoardDataPacket.TransportID = mywsnid;
   SendUpSBoardDataPacket.MSGCode = SENDUP_NODE_SBOARD_DATA;
   SendUpSBoardDataPacket.NodeAddr = 0;
   SendUpSBoardDataPacket.ModeID = SensorID;
   for(i = 0;i < (SendUpSBoardDataPacket.Len - 1);i++) 
   {
     num += ((uint8*)(&SendUpSBoardDataPacket))[i];
   }
    ((uint8*)(&SendUpSBoardDataPacket))[SendUpSBoardDataPacket.Len - 1] = num;  
   
   
    if (SampleMode == 0) //停止采样
   {
      osal_stop_timerEx( sapi_TaskID, COOR_SB_MSG_EVT );
      ctrPCA9554LED(4,1);
   }
    else if (SampleMode == 1) //单次采样或停止采样
   {
      osal_stop_timerEx( sapi_TaskID, COOR_SB_MSG_EVT );
      HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SendUpSBoardDataPacket, SendUpSBoardDataPacket.Len);  //通过串口向PC机发送数据
   }
    else       //连续采样 SamplingSpeed> 100ms
   {
     HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SendUpSBoardDataPacket, SendUpSBoardDataPacket.Len);  //通过串口向PC机发送数据
     if (SamplingSpeed < 100)
    {
       osal_start_timerEx( sapi_TaskID, COOR_SB_MSG_EVT, 100 );                 //最快采样间隔100
    }
    else if (SamplingSpeed > 5000)
    {
      osal_start_timerEx( sapi_TaskID, COOR_SB_MSG_EVT, 5000 );                 //最慢采样间隔
    }
    else
    {
      osal_start_timerEx( sapi_TaskID, COOR_SB_MSG_EVT, SamplingSpeed );
    }
   }
}
/**************************************************************************************************
 * 函数名称：ResetAllNode
 *
 * 功能描述：重启所有的节点
 *
 * 参    数：无
 *           
 *
 * 返 回 值：无
 **************************************************************************************************/
static void ResetAllNodeApp(void)
{
  PCGroupMSGPacket_t PCGroupMSGPacket;
  uint8 i;
  
  PCGroupMSGPacket.Hdr = '@';
  PCGroupMSGPacket.Len = 8;
  PCGroupMSGPacket.TransportID = mywsnid;
  PCGroupMSGPacket.MSGCode = TURNOFF_APP;
  PCGroupMSGPacket.GroupID = 0xFFFF;
  PCGroupMSGPacket.Checksum = 0;
  for(i=0;i<7;i++)
  {
    PCGroupMSGPacket.Checksum += ((uint8*)(&PCGroupMSGPacket))[i];
  }
  sendDummyReport((uint8*)&PCGroupMSGPacket,PCGroupMSGPacket.GroupID,
                          PCGroupMSGPacket.Len); //无线发送
}
