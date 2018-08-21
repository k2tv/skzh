/***************************************************************************************************************
* �� �� ����DemoCollector.c
��
* ��    �ܣ�SensorDemo��Э�����û�Ӧ�ò㣬���ļ���Ҫ����Z-Stack��һЩAPI��ʹЭ�����ܹ�������ά�����磬��ͨ��UART��PC��ͨ�ţ�
*           ���ռ����ĸ����ڵ����ݴ���PC����
*
*
* ע    �⣺
*                     
*           
*
* ��    ����V1.0
* ��    �ߣ�WU XIANHAI
* ��    �ڣ�2011.3.21
* �¶�˹������ҳ��www.ourselec.com
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

#define REPORT_FAILURE_LIMIT                10            //���ݷ���ʧ�ܵ������������������°��豸
#define ACK_REQ_INTERVAL                    5             // ÿ����5�����ݰ�������һ��ACK

// Stack Profile
#define ZIGBEE_2007                         0x0040
#define ZIGBEE_PRO_2007                     0x0041

#ifdef ZIGBEEPRO
#define STACK_PROFILE                       ZIGBEE_PRO_2007             
#else 
#define STACK_PROFILE                       ZIGBEE_2007
#endif

// Ӧ��״̬
#define APP_INIT                            0
#define APP_START                           2
#define APP_BINDED                          3    

// Ӧ�ò�ϵͳ�¼�
#define MY_START_EVT                        0x0001       //�����¼�
#define MY_REPORT_EVT                       0x0002       //���ݱ����¼�
#define MY_FIND_COLLECTOR_EVT               0x0004       //���ֺͰ�Э�����¼�
#define MY_COOR_LINK_SEV_EVT                0x0008       //���м����������ͨ���¼� add by wu 2010.12.09
#define COOR_SB_MSG_EVT                     0x0010       //��չ������������¼�
#define COOR_PB_MSG_EVT                     0x0020       //��ذ�����¼�

/* ȫ�ֱ��� */
static uint8 appState =             APP_INIT;           //Ӧ�ò㴦���¼�״̬
static uint8 applinkState =         SEND_SEV;           //���м����������ͨ���¼�״̬
static uint8 appcoorrevState =      UNALLOW_COOR_REV;   //Э��������·�ɽڵ���Ϣ״̬
//static uint8 reportState =          FALSE;            //��������״̬
static uint8 isGateWay =            FALSE;              //Э����ģʽ
//static uint16 myReportPeriod =      500;              
static uint8 WSNNumber = 0;                             //�豸���
static uint16 mywsnid = 0;                              //�ỰID �����豸��ź��豸������϶��ɣ�
static uint8 waittimeout = 0;                           //�豸ʶ��ȴ���ʱ
static uint8 heartbeattimeout = 0;                      //������ʱ

static uint8 reportFailureNr = 0;                       //���ݰ�����ʧ�ܼ�¼
static uint16 parentShortAddr = 0;                      //�����豸�Ķ̵�ַ
extern uint8 motherboardtype;                           //ĸ������
extern uint8 resetpanid;
extern uint16 mysetpanid;

static uint8 adchannel = 0;                             //ADCͨ��
static uint16 SampleMode = 0;                           //����ģʽ
static uint16 SamplingSpeed = 0;                        //�����ٶ�
static uint16 SensormodeID = 0;                         //ģ��ID
static uint8 appControlobject = 0;                      //��չģ�������������

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
// Э�������������������
#define NUM_OUT_CMD_COLLECTOR                2
#define NUM_IN_CMD_COLLECTOR                 2

// Э�������������б�
const cId_t zb_InCmdList[NUM_IN_CMD_COLLECTOR] =
{
  SENSOR_REPORT_CMD_ID,
  DUMMY_REPORT_CMD_ID
};
// Э������������б�
const cId_t zb_OutCmdList[NUM_IN_CMD_COLLECTOR] =
{
  SENSOR_REPORT_CMD_ID,
  DUMMY_REPORT_CMD_ID
};

//Э����������
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
 * �������ƣ�zb_HandleOsalEvent
 *
 * �����������û������¼�������
 *
 * ��    ����events - ��ǰ��Ҫ������¼�
 *           
 *
 * �� �� ֵ����
 **************************************************************************************************/
void zb_HandleOsalEvent( uint16 event )
{
  uint8 logicalType;
   
   if ( appState == APP_INIT  )                                               //Э������һ������ʱ�������豸����ΪЭ������Ȼ�������豸
      {
       zb_ReadConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType);  //��ȡ�豸����
        if (logicalType != ZG_DEVICETYPE_COORDINATOR)
        {
          logicalType = ZG_DEVICETYPE_COORDINATOR;                             //�����豸����ΪЭ����
          zb_WriteConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType); //д��NV��                
          zb_SystemReset();                                                    //�����豸
       }
      }
   
  if(event & SYS_EVENT_MSG)                                                   //ϵͳ��Ϣ�¼���һ���޲�����
  {
    
  }
  
  if( event & ZB_ENTRY_EVENT )                                               //�������罨����һЩ�û���Ҫʹ�õ���Χ�豸��ʼ���¼�
  {                                                                          //���¼���ִ��SAPI_Init���������ע��
    initUart(uartRxCB,HAL_UART_BR_115200);                                    //��ʼ������
                                  
    ctrPCA9554LED(5,1);                                                      //����LED1 ָʾ��������������
    
    zb_ReadConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType);  //��ȡ�豸����

    zb_StartRequest();                                                       //�����豸�������磨�ڸ����������豸���Զ������������HOLD_AUTO_STARTѡ�
  }
  
  if ( event & MY_START_EVT )                                                //�û��Զ��������豸�¼���������ȷ�Ϻ���zb_StartConfirm ��״̬Ϊ�ǳɹ�״̬ʱ���������¼������豸
  {
    zb_StartRequest();
  }
  
  if ( event & MY_REPORT_EVT )                                               //���汾�豸�ɼ�������Ϣ�¼�
  {
    
    if (isGateWay)                                                           //�����豸��������
    {
      
    }
    else if(appState == APP_BINDED)                                         //�������豸
    {   
    //  osal_start_timerEx( sapi_TaskID, MY_REPORT_EVT, myReportPeriod );
    }
  }
  if ( event & MY_FIND_COLLECTOR_EVT )                                       // ��Э�����豸��Э�����������¼�
  { 
    if (!isGateWay) 
    {
      zb_BindDevice( TRUE, DUMMY_REPORT_CMD_ID, (uint8 *)NULL );             //��Э����������
    }
  }
  if ( event & MY_COOR_LINK_SEV_EVT )                                        //PC�����ӣ��豸ʶ�����
  {
    if (applinkState == SEND_SEV)                                            //��SEV�����豸ʶ����Ϣ״̬
    {
      SEVLinkCoordPacket_t SEVLinkCoordPacket;                                  //�����豸ʶ��������Ϣ��
    
      SEVLinkCoordPacket.Hdr = '@';                                             //WSN��PC��������Ϣ��ֵ
      SEVLinkCoordPacket.Len = 10;
      SEVLinkCoordPacket.TransportID = 0;
      SEVLinkCoordPacket.MSGCode = EQU_ID_CODE;
      SEVLinkCoordPacket.AppType = 1;
      SEVLinkCoordPacket.EQUType = 1;
      SEVLinkCoordPacket.Heartbeat_Timeout = 30;
      SEVLinkCoordPacket.Heartbeat_Period = 5;
      SEVLinkCoordPacket.Checksum = 0x80;
    
      HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SEVLinkCoordPacket, sizeof(SEVLinkCoordPacket));  //ͨ��������PC�������豸ʶ����Ϣ
      waittimeout = 0;
      heartbeattimeout = 0;
      appcoorrevState = UNALLOW_COOR_REV;
      if (appControlobject)
      {
       osal_stop_timerEx( sapi_TaskID, COOR_SB_MSG_EVT ); 
       appControlobject = 0;
      }
      osal_start_timerEx( sapi_TaskID, MY_COOR_LINK_SEV_EVT, 500 );          //����ʱ��500ms
    }
    
    else if (applinkState == SEND_HEARTBEAT)                                 //��������״̬
    {
       SEVGeneralMSGPacket_t SEVhearbeatPacket;
       
       SEVhearbeatPacket.Hdr = '@';
       SEVhearbeatPacket.Len = 6;
       SEVhearbeatPacket.TransportID = 0;
       SEVhearbeatPacket.MSGCode = HEARTBEAT_DETECT;
       SEVhearbeatPacket.Checksum = 0x47;
       
       HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SEVhearbeatPacket, sizeof(SEVhearbeatPacket));  //ͨ�����ڷ���������Ϣ
            
       ++heartbeattimeout;                                                    //�������ռ�ʱ��������ʱΪ15s  
       if (heartbeattimeout == 6)
       {
         applinkState = SEND_SEV;                                             //����������ʱ������Ϊ��SEV�����豸ʶ����Ϣ״̬
         osal_stop_timerEx( sapi_TaskID, COOR_SB_MSG_EVT ); 
        // ResetAllNodeApp();           //���������нڵ㸴λ��Ϣ
       }
       osal_start_timerEx( sapi_TaskID, MY_COOR_LINK_SEV_EVT, 5000 );        //������������5s
    }
    else if (applinkState == REV_SEV_MSG)                                    //�ȴ�SEV�����豸�����Ϣ״̬
    {
      SEVMSGReplyPacket_t SEVMSGReplyPacket;
       if(WSNNumber)                                                         //�յ��豸�����Ϣ
       {
         mywsnid = BUILD_UINT16(WSNNumber,MY_WSN_TYPE);                      //��ϻỰID 
         applinkState = SEND_HEARTBEAT;                                      //�յ��豸�����Ϣ������Ϊ��������״̬
         WSNNumber = 0;
         SEVMSGReplyPacket.Hdr = '@';    
         SEVMSGReplyPacket.Len = 7;
         SEVMSGReplyPacket.TransportID = 0;
         SEVMSGReplyPacket.MSGCode = EQU_NUMBER_CODE;
         SEVMSGReplyPacket.ReplyCode = 0;
         SEVMSGReplyPacket.Checksum = 0x59;         
         HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SEVMSGReplyPacket, sizeof(SEVMSGReplyPacket));  // �ش�
       }
       else
       {
         waittimeout++;
       }      
      if (waittimeout == 12)
      {
        applinkState = SEND_SEV;   
      } 
      osal_start_timerEx( sapi_TaskID, MY_COOR_LINK_SEV_EVT, 500 );           //����ʱ��500ms
    } 
  }
  
  if ( event & COOR_SB_MSG_EVT )                                              //������������¼�
  {
    if (appControlobject == APP_ADC)                                          //ʹ��AD��ȡ���ݵĴ�����
    {
     ADCSamplingControl(adchannel,SampleMode,SamplingSpeed,SensormodeID);    //����AD�������ƺ���
    }
    else if (appControlobject == APP_IIC)                                    //ʹ��IIC��ȡ���ݵĴ����������
    {
      
    }
    else if (appControlobject == APP_IIC_CPU)                                //ʹ��IIC���ⲿCPU��ȡ���ݵĴ����������
    {
      
    }
    
  }
  
  if ( event & COOR_PB_MSG_EVT )                                             //��ذ�����¼�
  {
    
  }  
}

/**************************************************************************************************
 * �������ƣ�zb_HandleKeys
 *
 * ���������������¼�������
 *
 * ��    ����shift - shift/alt�� �ù��ܻ���δʹ��
 *           keys - ���밴��ֵ
 *
 * �� �� ֵ����
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
    if ( keys & HAL_KEY_SW_1 )                                                //SW1�����¼�����
    {
      mypanid = 1;
      if(mypanid)                                                             //�豸PANID����
      {
        zgConfigPANIDSetting = 0;
        GUI_ClearScreen();                                                    //LCD����
        GUI_PutString5_7(25,6,"OURS-CC2530");                                 //��LCD����ʾ��Ӧ������             
        GUI_PutString5_7(5,22," ZigBee PRO");                          
        GUI_PutString5_7(5,35,"PANID Setting");
        GUI_PutString5_7(5,48," 0");
        LCM_Refresh();
      }
    }
    if ( keys & HAL_KEY_SW_2 )                                               //SW2�����¼����� 
    {
      if(mypanid)                                                            //PANID����ʱ��Ϊ��1����
      {
        zgConfigPANIDSetting += 1;
        sprintf(s,(char *)" %d          ",zgConfigPANIDSetting);  
         GUI_PutString5_7(5,48,(char *)s);                                   //��ʾ���
         LCM_Refresh();
      }
      else                                                                   //Э��������ѡ�������豸����ͨЭ������
      {
       allowBind ^= 1;
       if (allowBind)                                                         //����ΪЭ����ģʽ
       {
        zb_AllowBind( 0x00 );                                                //�رհ󶨹��ܣ���Ӧ����û�п���ʹ�ö��Э�������ܣ�
        isGateWay = FALSE;       
        #if defined ( LCD_SUPPORTED )                                        //������ʾ����
        GUI_SetColor(1,0);
        GUI_ClearScreen();
        LCM_Refresh();
        GUI_PutString5_7(5,8,"OURS-CC2530");
        GUI_PutString5_7(5,30,"SensorDemo");
        GUI_PutString5_7(5,50,"Collector");
        LCM_Refresh();             
        #endif
       }
       else                                                                   //����Ϊ����ģʽ��������PC��ͨ�ŵ�Э������
       {       
        zb_AllowBind( 0xFF );                                                //�򿪰󶨹���
        isGateWay = TRUE;
        #if defined ( LCD_SUPPORTED )                                        //������ʾ����
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
    if ( keys & HAL_KEY_SW_3 )                                               //SW3���������¼�
    {
      if(mypanid)                                                            //PANID����ʱ��Ϊ��10����
     {
       zgConfigPANIDSetting += 10;
       sprintf(s,(char *)" %d          ",zgConfigPANIDSetting);  
         GUI_PutString5_7(5,48,(char *)s);                                   //��ʾ���
         LCM_Refresh();
       
     }              
    }
    if ( keys & HAL_KEY_SW_4 )                                               //SW4���������¼�
    {
      if(mypanid)                                                            //PANID����ʱ��Ϊ��100����
      {
        zgConfigPANIDSetting += 100;
        sprintf(s,(char *)" %d          ",zgConfigPANIDSetting);  
         GUI_PutString5_7(5,48,(char *)s);                                   //��ʾ���
         LCM_Refresh();
      }
      else                                                                   //���������ڵ��������
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
    if ( keys & HAL_KEY_SW_5 )                                               //SW5���������¼�
    {
      if(mypanid)                                                            //PANID����ʱ��Ϊȡ������
      {
        zgConfigPANIDSetting = 0;
        GUI_PutString5_7(5,48,"Unset   ");
        LCM_Refresh();
      }
    }
     if ( keys & HAL_KEY_SW_6 )                                              //SW6���������¼�
    {
      if(mypanid)                                                            //PANID����ʱ��Ϊ������ɼ�
      {
        if(zgConfigPANIDSetting)
        {       
          NLME_InitNV();        //���NV
          NLME_SetDefaultNV(); //����Ĭ��NV��Ŀ  
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
 * �������ƣ�zb_StartConfirm
 *
 * �����������豸����ȷ�Ϻ���
 *
 * ��    ����status - �豸����״̬�����ΪZB_SUCCESS��ʾ�豸�����ɹ�������ֵ��Ϊ����ʧ��
 *           
 *
 * �� �� ֵ����
 **************************************************************************************************/
void zb_StartConfirm( uint8 status )
{ 
  char  s[16];
  if ( status == ZB_SUCCESS )                                                //�ж��豸�Ƿ������ɹ����ɹ���ʧ�ܶ������ã�
  {   
     zb_AllowBind( 0xFF );                                                   //�����豸�� add by wu 2010.9.15
     ctrPCA9554LED(4,1);                            
     isGateWay = TRUE;                                                       //���ø��豸Ϊ�����豸��Э������  
     appState = APP_START;                                                    //�����豸��APP��Ϊ����״̬
    
     osal_set_event( sapi_TaskID, MY_FIND_COLLECTOR_EVT );                    //��Э�����󶨣���������Э������
       
     zb_GetDeviceInfo(ZB_INFO_PARENT_SHORT_ADDR, &parentShortAddr);           //��ȡ�ýڵ�ĸ��׶̵�ַ
       
    #if defined ( LCD_SUPPORTED )                                            //����LCD��ʾ����
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
    osal_set_event( sapi_TaskID, MY_COOR_LINK_SEV_EVT );                     // ��WSNЭ������PC���ӵ��豸ʶ����Ϣ����
    applinkState = SEND_SEV;                                                 //����ΪSEV�����豸ʶ����Ϣ
  }
  else                                                                       //����ʧ��
  {
    osal_start_timerEx( sapi_TaskID, MY_START_EVT, 10 );                    //��ʱһ��ʱ��󣬴���MY_START_EVT�¼��������豸
  }
}

/**************************************************************************************************
 * �������ƣ�zb_SendDataConfirm
 *
 * �������������ݷ���ȷ�Ϻ���
 *
 * ��    ����handle - ���ݷ��ʹ����ID
 *           status - ���ͽ�����ɹ����״̬��
 *
 * �� �� ֵ����
 **************************************************************************************************/
void zb_SendDataConfirm( uint8 handle, uint8 status )
{
  if ( status != ZB_SUCCESS && !isGateWay )                                  //����ʧ��
  {
    if ( ++reportFailureNr>=REPORT_FAILURE_LIMIT )                           //ʧ�ܴ������ڵ���4��
    {   
      // osal_stop_timerEx( sapi_TaskID, MY_REPORT_EVT );                      //ֹͣ���ݷ��ͣ�����ʱһ��ʱ��
       osal_stop_timerEx( sapi_TaskID, MY_COOR_LINK_SEV_EVT );
            
       zb_BindDevice( FALSE, DUMMY_REPORT_CMD_ID, (uint8 *)NULL );           //ɾ��ԭ���İ�
       
       osal_set_event( sapi_TaskID, MY_FIND_COLLECTOR_EVT );                 //����Ѱ���µ��豸�����°��豸
       reportFailureNr=0;                                                    //ʧ�ܴ�����������
       ResetAllNodeApp();                                                    
    }
  }
  else if ( !isGateWay )                                                     //�ɹ�
  {
    reportFailureNr=0;                                                       //ʧ�ܴ�����������
  }
}

/**************************************************************************************************
 * �������ƣ�zb_BindConfirm
 *
 * �����������豸��ȷ�ϣ���ʼ��������
 *
 * ��    ����commandId - �������ID
 *           status - �豸����״̬�����ΪZB_SUCCESS��ʾ�豸�����ɹ�������ֵ��Ϊ����ʧ��
 *           
 *
 * �� �� ֵ����
 **************************************************************************************************/
void zb_BindConfirm( uint16 commandId, uint8 status )
{
  if( status == ZB_SUCCESS )                                                 //�󶨳ɹ�
  {
    appState = APP_BINDED;                                                   //����Ϊ��״̬
    
     // osal_set_event( sapi_TaskID, MY_REPORT_EVT );                          //����Э�������ر���

  }
  else                                                                       //��ʧ�ܣ����°�
  {
    osal_start_timerEx( sapi_TaskID, MY_FIND_COLLECTOR_EVT, 2000 );
  }
}

/**************************************************************************************************
 * �������ƣ�zb_ReceiveDataIndication
 *
 * �������������ݽ��մ�����
 *
 * ��    ����source - �����豸��Դ�̵�ַ
 *           commandId - ����ID
 *           lqi -�ڵ�LQIֵ
 *           rssi ���ڵ�RSSIֵ
 *           len - ���յ����ݰ�����
 *           pData - ���ݻ���ָ��
 *
 * �� �� ֵ����
 **************************************************************************************************/
void zb_ReceiveDataIndication( uint16 source, uint16 command, uint8 lqi, int8 rssi,uint16 len, uint8 *pData  )
{ 
  uint8 i,num = 0;
  
  if(appcoorrevState == ALLOW_COOR_REV)                                      //�Ƿ�����Э����������Ϣ 
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
      HalUARTWrite(HAL_UART_PORT_0,(uint8 *)pData, *(pData+1));             //����ת��
     // if (*(pData+4) == SENDUP_NODE_PADDR_ADDR) //���ӹ�ϵУ��
     // {
     //   FandSCheck(pData);    //����У�麯��
    // }
   } 
   ctrPCA9554FLASHLED5(1);
  }  
}

/**************************************************************************************************
 * �������ƣ�FandSCheck
 *
 * ��������������У�鸸�ӹ�ϵ����ȷ��
 *
 * ��    ����pBuf - ��ҪУ��Ľڵ��ϵ��ַ����
 *           
 *
 * �� �� ֵ����
 **************************************************************************************************/
static uint8 FandSCheckcount = 0;
static uint16 FandSCheckpaddr = 0;
static uint16 FandSCheckaddr = 0;
static void FandSCheck(uint8 *pBuf)   
{
  PCNodeAddrPacket_t PCNodeAddrPacket;
  PCGeneralMSGPacket_t PCGeneralMSGPacket;
  uint8 i,num = 0;
  osal_memcpy((void *)&PCNodeAddrPacket,pBuf,10);      //���游�ӽڵ��ϵ��Ϣ
   if(FandSCheckcount)
  {
     if(FandSCheckpaddr==PCNodeAddrPacket.NodeAddr)    //У�鸸�׽ڵ��Ƿ����
     {
       FandSCheckcount = 0;                            //���ڣ���ʼ�µ�У��
     }
     else                                              //У�������1
     {
        FandSCheckcount++;
     }
     if (FandSCheckcount == 12)                       //У�������10�κ�û���յ��丸�׽ڵ���Ϣ����Ϊ�ø��׽ڵ㲻����
     {
        FandSCheckcount = 0;                          //���������ýڵ���Ϣ
        PCGeneralMSGPacket.Hdr = '@';
        PCGeneralMSGPacket.Len = 8;
        PCGeneralMSGPacket.TransportID = PCNodeAddrPacket.TransportID;
        PCGeneralMSGPacket.MSGCode = NODE_SYSTEMRESET;
        PCGeneralMSGPacket.NodeAddr = FandSCheckaddr;
        for(i=0;i<7;i++)                              //������Ϣ��У���
       {
        num = num + ((uint8*)(&PCGeneralMSGPacket))[i];
       }
        PCGeneralMSGPacket.Checksum = num;
       sendDummyReport((uint8*)(&PCGeneralMSGPacket),PCGeneralMSGPacket.NodeAddr,8); //���߷�����������     
     }
   }
   else
  {
    if(PCNodeAddrPacket.NodePAddr)                 //�жϸ��ڵ��ַ�Ƿ�ΪЭ����          
    {
     FandSCheckpaddr = PCNodeAddrPacket.NodePAddr;  //����У��ڵ�ĸ���ַ
     FandSCheckaddr = PCNodeAddrPacket.NodeAddr;    //����У��ڵ��ַ
     FandSCheckcount++;                             //��ʼУ��
    }
    else
    {
      FandSCheckcount = 0;                          //���ڵ��ַΪЭ��������ҪУ��
    }
   }
}
/**************************************************************************************************
 * �������ƣ�uartRxCB
 *
 * ��������������UART����
 *
 * ��    ����port - UART�˿ں�
 *           event- UART�¼������ջ��ͣ�
 *
 * �� �� ֵ����
 **************************************************************************************************/
void uartRxCB( uint8 port, uint8 event )
{
  uint8 pBuf[RX_BUF_LEN];
  uint16 len;
  uint8 Checksum = 0;
  uint8 i;
  
  if ( event != HAL_UART_TX_EMPTY )                                          //���ڽ����¼�ȷ��
  {   
    len = HalUARTRead( HAL_UART_PORT_0, pBuf, RX_BUF_LEN );                  //������
    
    if ( len > 0 ) 
    {
      if ( pBuf[0] == '@')                                                
      {
        for(i=0;i<(pBuf[1] - 1);i++)                                          //����У���    
        {
         Checksum += pBuf[i];
        }
        if ( pBuf[2] )                                                       //��Ӧ�ò��ͨ��                                                   
        {
          APPuartRxHandle(pBuf,Checksum);
        }
        else                                                                  //���м������ͨ��
        {
          SEVuartRxHandle(pBuf,Checksum);
        }                                               
      }
    }
  }
}

/**************************************************************************************************
 * �������ƣ�SEVuartRxHandle
 *
 * ��������������Э�������յ��м����������������Ϣ
 *
 * ��    ����MSGCode - UART���յ�����Ϣ��
 *           num - ��Ϣ����У���       
 *
 * �� �� ֵ����
 **************************************************************************************************/
static void SEVuartRxHandle(uint8 *MSGCode,uint8 num)
{
  SEVMSGReplyPacket_t SEVMSGReplyPacket;
  SEVEQUNumberPacket_t SEVEQUNumberPacket;
  SEVGeneralMSGPacket_t SEVGeneralMSGPacket;
  switch (MSGCode[4])
  {
    case HEARTBEAT_DETECT:   //�������
      osal_memcpy((void *)&SEVGeneralMSGPacket,&MSGCode[0],6);
     if(SEVGeneralMSGPacket.Checksum == num)
     {
       heartbeattimeout = 0;            
     }break;
     
    case FIRST_APP_LINK:     //��һ���ϲ�Ӧ�����ӷ���
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
       HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SEVMSGReplyPacket, sizeof(SEVMSGReplyPacket));  // �ش�
     if (isGateWay)  //�����GateWayģʽ ��ı���ʾ����
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
      
    case LAST_APP_ULINK:     //���һ���ϲ�Ӧ�������Ͽ�
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
       HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SEVMSGReplyPacket, sizeof(SEVMSGReplyPacket)); //�ش�
       osal_stop_timerEx( sapi_TaskID, COOR_SB_MSG_EVT );     //ֹͣ���ر�����Ϣ
     }break;
      
    case EQU_ID_CODE:        //������Ӧ����Ϣ
     osal_memcpy((void *)&SEVMSGReplyPacket,&MSGCode[0],7);
     if(SEVMSGReplyPacket.Checksum == num)
     {
       if (SEVMSGReplyPacket.ReplyCode == 0) //Ӧ��ɹ�
       {
         applinkState = REV_SEV_MSG;  //����Ϊ�ȴ�SEV�����豸�����Ϣ
       }
       else      //Ӧ��ʧ��
       {
         
       }
     }break;
     
    case EQU_NUMBER_CODE:     //�豸�����Ϣ
     osal_memcpy((void *)&SEVEQUNumberPacket,&MSGCode[0],7);
     if(SEVEQUNumberPacket.Checksum == num)
     {
       WSNNumber = SEVEQUNumberPacket.NumberCode;
              
     }break;
     
     default: break;
  }
}

/**************************************************************************************************
 * �������ƣ�APPuartRxHandle
 *
 * ��������������Э�������յ�PC��Ӧ�ò��������������Ϣ
 *
 * ��    ����MSGCode - UART���յ�����Ϣ��
 *           num - ��Ϣ����У���       
 *
 * �� �� ֵ����
 **************************************************************************************************/
static void APPuartRxHandle(uint8 *MSGCode,uint8 num)
{ 
  if(appcoorrevState == ALLOW_COOR_REV)     //�Ƿ���������㽨������
  {
    static PCGeneralMSGPacket_t PCGeneralMSGPacket;
    static SendDownSBoardDataPacket_t SendDownSBoardDataPacket;
    static SendDownTemBoardDataPacket_t SendDownTemBoardDataPacket; 
    static SetPANIDPacket_t SetPANIDPacket;
    if (MSGCode[4] <= 0x10)      //һ����Ϣ
    {    
       osal_memcpy((void *)&PCGeneralMSGPacket,&MSGCode[0],8);   //��������
      if(PCGeneralMSGPacket.Checksum == num)                     //���ݰ�У��
     {
       if(PCGeneralMSGPacket.NodeAddr)                          //�Ǳ�����Ϣ
       {
        sendDummyReport((uint8*)(&PCGeneralMSGPacket),PCGeneralMSGPacket.NodeAddr,8); //���߷���
       }
       else          //������Ϣ
       {     
          if (PCGeneralMSGPacket.MSGCode == FETCH_NODE_INFO)         //��ȡЭ������Ϣ
          {
             SendDeviceInfo();
          }
          else if (PCGeneralMSGPacket.MSGCode == NODE_SYSTEMRESET)        //����Э����
          {
             zb_SystemReset(); 
          }
          else if (PCGeneralMSGPacket.MSGCode == FETCH_NODE_SENSOR_CAL)   //��ȡ������У׼����
          {
            SendSensorInfo();
          }
       }
     }
    }
    else if(MSGCode[4] > 0x20)  //������Ϣ
    {
      switch (MSGCode[4])
      {        
        case SENDDOMN_NODE_SBOARD_DATA:   //PC�´��ڵ���չ����Դ����  
        osal_memcpy((void *)&SendDownSBoardDataPacket,&MSGCode[0],MSGCode[1]);   //��������
          if(SendDownSBoardDataPacket.NodeAddr)                          //�Ǳ�����Ϣ
         {
          sendDummyReport((uint8*)&MSGCode[0],SendDownSBoardDataPacket.NodeAddr,
                          SendDownSBoardDataPacket.Len); //���߷���
          }
         else          //������Ϣ
         { 
            SensorControl(SendDownSBoardDataPacket.ModeID,&MSGCode[9]); //������չ�����
         }
        break;
        case SENDDOMN_NODE_BBOARD_DATA:   //PC�´��ڵ�װ���Դ����
        osal_memcpy((void *)&SendDownTemBoardDataPacket,&MSGCode[0],MSGCode[1]);   //��������
          if(SendDownTemBoardDataPacket.NodeAddr)                          //�Ǳ�����Ϣ
         {
          sendDummyReport((uint8*)(&MSGCode[0]),SendDownTemBoardDataPacket.NodeAddr,
                          SendDownTemBoardDataPacket.Len); //���߷���
          }
         else          //������Ϣ
         {
           MotherboardControl(SendDownTemBoardDataPacket.ResID,&MSGCode[10]); //����ĸ�����
           
         }
        break;
        case SENDDOMN_GROUPNODE_LIST:   //PC�ڵ������Ϣ(����)
          
        break;
        case SET_NODE_PANID:
          osal_memcpy((void *)&SetPANIDPacket,&MSGCode[0],MSGCode[1]);   //��������
           if(SetPANIDPacket.NodeAddr)                          //�Ǳ�����Ϣ
         {
          sendDummyReport((uint8*)(&MSGCode[0]),SetPANIDPacket.NodeAddr,
                          SetPANIDPacket.Len); //���߷���
          }
         else          //������Ϣ
         {
           if(SetPANIDPacket.SETPANID)
          {
            if(SetPANIDPacket.SETPANID < 0x3fff)
            {   
               NLME_InitNV();        //���NV
               NLME_SetDefaultNV(); //����Ĭ��NV��Ŀ  
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
     else   //һ��Ⱥ����Ϣ(����)
     {
       
     }
  }
}

/**************************************************************************************************
 * �������ƣ�sendDummyReport
 *
 * ����������Э�������߷�������
 *
 * ��    ����pData - ���͵����ݰ���Ϣ
 *           Addr - �ڵ��ַ
 *           len - ���ݰ�����
 *
 * �� �� ֵ����
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
 * �������ƣ�SendDeviceInfo
 *
 * ������������ȡ�豸��Ϣ�������߷��͸�Э����
 *
 * ��    ������
 *           
 *
 * �� �� ֵ����
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
   zb_GetDeviceInfo(ZB_INFO_IEEE_ADDR, myIEEEAddr);                         //��ȡ�豸IEEE��ַ add by wu 2010.11.30
    for(i=0;i<8;i++)
   {
      PCNodeMSGPacket.IEEEAddr[i] = *(myIEEEAddr+i);
   }
   PCNodeMSGPacket.NodeRSSI = 0;
   PCNodeMSGPacket.NodeLQI = 0;
   osal_nv_read(ZCD_NV_PANID, 0, 2, &zgConfigPANID);
   PCNodeMSGPacket.Panid = zgConfigPANID;
   PCNodeMSGPacket.TemType = motherboardtype;             //ĸ������
   if (sensoridH == 0)
   {
    sensoridH = read24L01byte(0x01);
    sensoridL = read24L01byte(0x02);
     if((sensoridH == 0)&&(sensoridL == 0))                //����������Ϣ����Ϊ��ѹ���ģ��
    {
      if (HalAdcRead (HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_14)>3000)//�ж��Ƿ��е�ѹ���ģ�飨��ʱ��
     {
      sensoridH = 0x10;
      sensoridL = 0x01;
     }
    }
     if((sensoridH == 0x30)&&(sensoridL == 0x00))          //��ȡЭ�������ڴ���
   {
     sensoridH = read24L01byte(0x03);
     sensoridL = read24L01byte(0x04);
   }
   }
   PCNodeMSGPacket.SBoardType = BUILD_UINT16(sensoridL,sensoridH);  //��չ���а�����
   PCNodeMSGPacket.Checksum = 0;
    for(i=0;i<22;i++)
   {
      PCNodeMSGPacket.Checksum += ((uint8*)(&PCNodeMSGPacket))[i];
   }  
   HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&PCNodeMSGPacket, sizeof(PCNodeMSGPacket));            //ͨ��������PC����������
}

/**************************************************************************************************
 * �������ƣ�SendSensorInfo
 *
 * ������������ȡ��չ�崫��������У׼����
 *
 * ��    ������
 *           
 *
 * �� �� ֵ����
 **************************************************************************************************/
static void SendSensorInfo(void)
{
  uint8 i;
  SensorCalPacket_t SensorCalPacket = {0x40,16,mywsnid,SENSOR_DATA_CAL,0,0,0,0,0,0};
  SensorCalPacket.ModeID = BUILD_UINT16(sensoridL,sensoridH);    //��չ���а�����
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
  HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SensorCalPacket, sizeof(SensorCalPacket));            //ͨ��������PC����������
}

/**************************************************************************************************
 * �������ƣ�SensorControl
 *
 * ��������������PC�˷��͵���չ�������Ϣ��Ȼ�󴫵ݸ���Ӧ�Ŀ��ƺ�����
 *
 * ��    ����SensorID - ���ش������������������
 *           ControlMSG - PC���͵Ŀ�����Ϣ
 *           
 *
 * �� �� ֵ����
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
    HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SendUpSBoardDataPacket2, SendUpSBoardDataPacket2.Len);  //ͨ��������PC�������豸ʶ����Ϣ
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
 * �������ƣ�MotherboardControl
 *
 * ��������������PC�˷��͵ĵװ�������Ϣ��Ȼ�󴫵ݸ���Ӧ�Ŀ��ƺ�����
 *
 * ��    ����SensorID - ���ش������������������
 *           ControlMSG - PC���͵Ŀ�����Ϣ
 *           
 *
 * �� �� ֵ����
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
    HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SendUpTemBoardDataPacket, SendUpTemBoardDataPacket.Len);  //ͨ��������PC�����ͻظ���Ϣ
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
    HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SendUpTemBoardDataPacket, SendUpTemBoardDataPacket.Len);  //ͨ��������PC�����ͻظ���Ϣ
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
   
   HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SendUpTemBoardDataPacket, SendUpTemBoardDataPacket.Len);  //ͨ��������PC�����ͻظ���Ϣ
    break;
    case resCode_AD:
    
    break;
   default:break;
  }
}
/**************************************************************************************************
 * �������ƣ�ADCSamplingControl
 *
 * ��������������PC���͵�AD��������ɼ�����������ģ��Ĵ���������
 *
 * ��    ����channel - AD������ʹ�õ�ͨ��
 *           SampleMode - ����ģʽ
 *           SamplingSpeed �������ٶ�
 *           SensorID - �ڵ���չ�崫����ID
 *
 *           
 *
 * �� �� ֵ����
 **************************************************************************************************/
static void ADCSamplingControl(uint8 channel,uint16 SampleMode,uint16 SamplingSpeed,uint16 SensorID)
{
   SendUpSBoardDataPacket_t SendUpSBoardDataPacket;
   uint16 *ADCdata = NULL;
   uint8 i,num = 0;
   int tempera;
   int humidity;
    if (channel == 0x80)  //ʹ��0ͨ��
   {  
      SendUpSBoardDataPacket.data = (uint16*)HalAdcRead (HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_14);
      SendUpSBoardDataPacket.Len = 12;  
   }
    else if(channel == 0x40)  //ʹ��1ͨ��
   {
      SendUpSBoardDataPacket.data = (uint16*)HalAdcRead (HAL_ADC_CHANNEL_1, HAL_ADC_RESOLUTION_14);
      SendUpSBoardDataPacket.Len = 12;  
   }
   else if(channel == 0xC0)  //ʹ������ͨ��
   {
      *ADCdata = HalAdcRead (HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_14);         
      *(ADCdata + 1) = HalAdcRead (HAL_ADC_CHANNEL_1, HAL_ADC_RESOLUTION_14);
      osal_memcpy((void *)&(SendUpSBoardDataPacket.data),ADCdata,4);
      SendUpSBoardDataPacket.Len = 14;
   }
   else if(channel == 0x0f) //ֻ��IIC��ȡ�Ĵ��������ݣ�û��ADC��ʽ�Ĵ�����
   {
      th_read(&tempera,&humidity);      
      *ADCdata = (uint16)tempera;         
      *(ADCdata + 1) = (uint16)humidity;
      osal_memcpy((void *)&(SendUpSBoardDataPacket.data),ADCdata,4);
      SendUpSBoardDataPacket.Len = 14;
   }
      else if(channel == 0x8f) //IIC��ȡ�Ĵ��������ݺ�ʹ��0ͨ��
   {
      th_read(&tempera,&humidity);      
      *ADCdata = (uint16)tempera;         
      *(ADCdata + 1) = (uint16)humidity;
      *(ADCdata + 2) = HalAdcRead (HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_14);
      osal_memcpy((void *)&(SendUpSBoardDataPacket.data),ADCdata,6);
      SendUpSBoardDataPacket.Len = 16;
   }
   else if(channel == 0x4f) //IIC��ȡ�Ĵ��������ݺ�ʹ��1ͨ��
   {
     
   }
   else if(channel == 0xcf) //IIC��ȡ�Ĵ��������ݺ�1,2����ͨ��
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
   
   
    if (SampleMode == 0) //ֹͣ����
   {
      osal_stop_timerEx( sapi_TaskID, COOR_SB_MSG_EVT );
      ctrPCA9554LED(4,1);
   }
    else if (SampleMode == 1) //���β�����ֹͣ����
   {
      osal_stop_timerEx( sapi_TaskID, COOR_SB_MSG_EVT );
      HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SendUpSBoardDataPacket, SendUpSBoardDataPacket.Len);  //ͨ��������PC����������
   }
    else       //�������� SamplingSpeed> 100ms
   {
     HalUARTWrite(HAL_UART_PORT_0,(uint8 *)&SendUpSBoardDataPacket, SendUpSBoardDataPacket.Len);  //ͨ��������PC����������
     if (SamplingSpeed < 100)
    {
       osal_start_timerEx( sapi_TaskID, COOR_SB_MSG_EVT, 100 );                 //���������100
    }
    else if (SamplingSpeed > 5000)
    {
      osal_start_timerEx( sapi_TaskID, COOR_SB_MSG_EVT, 5000 );                 //�����������
    }
    else
    {
      osal_start_timerEx( sapi_TaskID, COOR_SB_MSG_EVT, SamplingSpeed );
    }
   }
}
/**************************************************************************************************
 * �������ƣ�ResetAllNode
 *
 * �����������������еĽڵ�
 *
 * ��    ������
 *           
 *
 * �� �� ֵ����
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
                          PCGroupMSGPacket.Len); //���߷���
}
