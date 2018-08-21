/***************************************************************************************************************
* �� �� ����DemoSensor.c
��
* ��    �ܣ�SensorDemo�û�Ӧ�ò㣬��Ҫ�Ǻ������豸�����󶨺Ͳɼ����������ݣ������Ե��������豸
*           ���棬����Э�������͵Ŀ���������Ʊ��ض���
*
*
* ע    �⣺
*           
*           
*           
*           
*
* ��    ����V1.0
* ��    �ߣ�WU XIANHAI
* ��    �ڣ�2011.3.22
* �¶�˹������ҳ��www.ourselec.com
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
#define REPORT_FAILURE_LIMIT                10 //���ݷ���ʧ�ܵ������������������°��豸
#define ACK_REQ_INTERVAL                    5 //ÿ����5�����ݰ�������һ��ACK

// �豸Ӧ�ò�״̬
#define APP_INIT                            0    // ��ʼ��
#define APP_START                           1    // ��������ɹ�
#define APP_BIND                            2    // �󶨳ɹ�
#define APP_REPORT                          4    // ��ʼ��������


//�û��Զ��崦���¼�  Bit mask of events ( from 0x0000 to 0x00FF )
#define MY_START_EVT                        0x0001 //�����豸�¼�����Ҫ��Э����
#define MY_REPORT_EVT                       0x0002 //���������¼�
#define MY_FIND_COLLECTOR_EVT               0x0004 //����Э���������¼�
#define MY_SB_MSG_EVT                       0x0008 //��չ������������¼�
#define MY_PB_MSG_EVT                       0x0010 //��ذ�����¼�

/******************************************************************************
 * LOCAL VARIABLES
 */
static uint8 appState = APP_INIT;             //�豸״̬
static uint8 reportFailureNr = 0;             //��������ʧ�ܼ�
static uint16 parentShortAddr;                // ���׽ڵ�̵�ַ
static uint16 myShortAddr;                    // �����豸�̵�ַ 
extern uint8 motherboardtype;                 //ĸ������
static uint8 mypanid = 0;                     //PANID���ñ�־
static uint8 adchannel = 0;                   //ADCͨ��
static uint16 SampleMode = 0;                 //����ģʽ
static uint16 SamplingSpeed = 0;              //�����ٶ�
static uint16 SensormodeID = 0;               //ģ��ID
static uint8 appControlobject = 0;            //��չģ�������������

/******************************************************************************
 * GLOBAL VARIABLES
 */

//���ݲɼ��ڵ��������������
#define NUM_OUT_CMD_SENSOR                1
#define NUM_IN_CMD_SENSOR                 0

//Э������������б�
const cId_t zb_OutCmdList[NUM_OUT_CMD_SENSOR] =
{
  SENSOR_REPORT_CMD_ID
};

// ���ݲɼ��ڵ������
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
  
  if(event & SYS_EVENT_MSG)                                                   //ϵͳ��Ϣ�¼���һ���޲�����
  {
    
  }
  
  if( event & ZB_ENTRY_EVENT )                                               //�������罨����һЩ�û���Ҫʹ�õ���Χ�豸��ʼ���¼�
  { 
    ctrPCA9554LED(5,1);
    initUart(uartRxCB,HAL_UART_BR_115200);                                    //��ʼ��UART 
    
    zb_StartRequest();                                                       //�����豸�������磨�ڸ����������豸���Զ������������HOLD_AUTO_STARTѡ�
  }
  
  if ( event & MY_REPORT_EVT )                                               //��ʼ��Э�������淢������ ���ɰ�ȷ�Ϻ���zb_BindConfirm ���ø��¼���
  { 
    if ( appState == APP_REPORT ) 
    {
      PCNodeAddrPacket_t PCNodeAddrPacket;                                   //��ʱ���游�ӽڵ��ϵ
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
      sendReport((uint8*)(&PCNodeAddrPacket),10);                             //���÷��ͺ���
      ctrPCA9554FLASHLED5(1);
      osal_start_timerEx( sapi_TaskID, MY_REPORT_EVT, 2000 );                //�������������趨
    }
  }
  if ( event & MY_FIND_COLLECTOR_EVT )                                        // ��Э�����豸��Э�����������¼�
  {
    if ( appState==APP_REPORT )                                               //ɾ��ԭ���İ�
    {
      zb_BindDevice( FALSE, SENSOR_REPORT_CMD_ID, (uint8 *)NULL );
    }
    
    appState = APP_BIND;                                                     //�����豸Ϊ��״̬
    
    ctrPCA9554LED(4,1);
    
    zb_BindDevice( TRUE, SENSOR_REPORT_CMD_ID, (uint8 *)NULL );              //Ѱ�ҺͲ���Э������
  }
  
    if ( event & MY_SB_MSG_EVT )                                             //������������¼�
  {
    if (appControlobject == APP_ADC)                                         //ʹ��AD��ȡ���ݵĴ�����
    {
     ADCSamplingControl(adchannel,SampleMode,SamplingSpeed,SensormodeID);    //����AD�������ƺ���
    }
    else if (appControlobject == APP_IIC)                                    //ʹ��IIC��ȡ���ݵĴ����������
    {
      IICControlSensor(adchannel,SampleMode,SamplingSpeed,SensormodeID);
    }
    else if (appControlobject == APP_IIC_CPU)                                //ʹ��IIC���ⲿCPU��ȡ���ݵĴ����������
    {
      
    }
    else if (appControlobject == APP_UART)
    {
      
    }
  }
  
  if ( event & MY_PB_MSG_EVT )                                               //��ذ�����¼�
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
    if ( keys & HAL_KEY_SW_1 )                                               //SW1�����¼�����
    {
      mypanid = 1;                                                           //�豸PANID����
      zgConfigPANIDSetting = 0;
      GUI_ClearScreen();                                                      //LCD����
      GUI_PutString5_7(25,6,"OURS-CC2530");                                   //��LCD����ʾ��Ӧ������             
      GUI_PutString5_7(5,22," ZigBee PRO");                          
      GUI_PutString5_7(5,35,"PANID Setting");
      GUI_PutString5_7(5,48," 0");
      LCM_Refresh();
    }
    if ( keys & HAL_KEY_SW_2 )                                               //SW2�����¼�����
    {
     if(mypanid)                                                             //PANID����ʱ��Ϊ��1����
     {
       zgConfigPANIDSetting += 1;
       sprintf(s,(char *)" %d         ",zgConfigPANIDSetting);  
       GUI_PutString5_7(5,48,(char *)s);                                     //��ʾ���
       LCM_Refresh();
       
     }
    }
    if ( keys & HAL_KEY_SW_3 )                                               //SW3���������¼�
    {
      if(mypanid)
     {
       zgConfigPANIDSetting += 10;                                           //PANID����ʱ��Ϊ��10����
       sprintf(s,(char *)" %d         ",zgConfigPANIDSetting); 
       GUI_PutString5_7(5,48,(char *)s);                                     //��ʾ���
       LCM_Refresh();     
     }
      
    }
    if ( keys & HAL_KEY_SW_4 )                                               //SW4���������¼�
    {
      if(mypanid)
     {
       zgConfigPANIDSetting += 100;
       sprintf(s,(char *)" %d         ",zgConfigPANIDSetting); 
       GUI_PutString5_7(5,48,(char *)s);                                    //��ʾ���
       LCM_Refresh();      
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
    if ( keys & HAL_KEY_SW_6 )                                               //SW6���������¼�
    {
      if(mypanid)                                                            //PANID����ʱ��Ϊ������ɼ�
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
  uint8 ShortAddrH;
  uint16 ShortAddrL;
  if ( status == ZB_SUCCESS )                                                //�ж��豸�Ƿ������ɹ����ɹ���ʧ�ܶ������ã�
  {
    appState = APP_START;                                                    //�����豸��APP��Ϊ����״̬

    zb_GetDeviceInfo(ZB_INFO_PARENT_SHORT_ADDR, &parentShortAddr);           //��ȡ�ýڵ�ĸ��׶̵�ַ
 
    zb_GetDeviceInfo(ZB_INFO_SHORT_ADDR, &myShortAddr);                      //��ȡ���豸�Ķ̵�ַ add by wu 2010.9.20
    #if defined ( LCD_SUPPORTED )                                            //����LCD��ʾ����
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
        GUI_PutString5_7(75,32,(char *)s);                                   //��ʾ���
        
        osal_nv_read(ZCD_NV_PANID, 0, 2, &zgConfigPANID);
        sprintf(s,(char *)" %d     ",zgConfigPANID);
        GUI_PutString5_7(50,44,(char *)s);              
        GUI_PutString5_7(25,8,"OURS-CC2530");
        GUI_PutString5_7(5,20,"SensorDemo");
        GUI_PutString5_7(5,32,"ShortAddr:");
        GUI_PutString5_7(5,44,"PANID:");
        LCM_Refresh();
    #endif
  
    osal_set_event( sapi_TaskID, MY_FIND_COLLECTOR_EVT );                    //������Э�������¼�
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
  if(status != ZB_SUCCESS)                                                   //����ʧ��
  {
    if ( ++reportFailureNr >= REPORT_FAILURE_LIMIT )                         //ʧ�ܴ������ڵ���4��
    {
       
       osal_stop_timerEx( sapi_TaskID, MY_REPORT_EVT );                      //ֹͣ���͸��ӽڵ��ϵ��Ϣ     
       osal_stop_timerEx( sapi_TaskID, MY_SB_MSG_EVT );                      //ֹͣ���ݲɼ��¼�
       
       osal_set_event( sapi_TaskID, MY_FIND_COLLECTOR_EVT );                 //����Ѱ���µ�Э���������°��豸
       reportFailureNr=0;                                                    //ʧ�ܴ�����������
    }
  }
  else                                                                       //���ͳɹ�
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
    appState = APP_REPORT;                                                   //�豸����Ϊ��ʼ��������״̬

     osal_set_event( sapi_TaskID, MY_REPORT_EVT );                          //���÷�����Ϣ�¼�
     
     if(appControlobject)                                                   //������������ݲ��ã����°󶨺󣬼���ִ��ԭ��������
     {
       osal_set_event( sapi_TaskID, MY_SB_MSG_EVT ); 
     }
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
 *           lqi ����·����
 *           rssi - �ź�ǿ��
 *           len - ���յ����ݰ�����
 *           pData - ���ݻ���ָ��
 *
 * �� �� ֵ����
 **************************************************************************************************/
void zb_ReceiveDataIndication( uint16 source, uint16 command, uint8 lqi, int8 rssi,uint16 len, uint8 *pData  )
{
   static PCGeneralMSGPacket_t PCGeneralMSGPacket;    
   static PCGroupMSGPacket_t PCGroupMSGPacket;
   uint8 i,num = 0;
  if (command == myShortAddr)                                              //���豸����ȷ�� 
  {     
    if(*(pData+4)<=0x10)                                                    //һ����Ϣ
    {
      osal_memcpy((void *)&PCGeneralMSGPacket,pData,len);                   //���Ʊ�������
      num = 0;
      for(i=0;i<(len-1);i++)                                                //����У���
      {
        num += ((uint8*)(&PCGeneralMSGPacket))[i];
      }     
      if(PCGeneralMSGPacket.Checksum == num)
      {   
        switch(PCGeneralMSGPacket.MSGCode)
        {
          case FETCH_NODE_INFO :                                           //��ȡ�ڵ���Ϣ
            SendDeviceInfo();            
            break;
          case NODE_SYSTEMRESET :                                           //�ڵ�����
           // zb_SystemReset(); 
            zb_StartRequest();  
           break;
          case FETCH_NODE_SENSOR_CAL :                                      //��ȡ������У׼����
            SendSensorInfo();
           break;
          default: break;          
        }
      }
    }//end һ����Ϣ
    else if ((*(pData+4)>0x10)&&(*(pData+4)<=0x30))                          //������Ϣ
    {
      SendDownSBoardDataPacket_t SendDownSBoardDataPacket;
      SendDownTemBoardDataPacket_t SendDownTemBoardDataPacket;
      SetPANIDPacket_t SetPANIDPacket;
      switch (*(pData+4))
      {        
        case SENDDOMN_NODE_SBOARD_DATA:                                     //PC�´��ڵ���չ����Դ����   
          osal_memcpy((void *)&SendDownSBoardDataPacket,pData,len);         //���Ʊ�������
            SensorControl(SendDownSBoardDataPacket.ModeID,pData+9);         //������չ�����
        break;
        case SENDDOMN_NODE_BBOARD_DATA:                                     //PC�´��ڵ�װ���Դ����
        osal_memcpy((void *)&SendDownTemBoardDataPacket,pData,len);         //���Ʊ�������        
        MotherboardControl(SendDownTemBoardDataPacket.ResID,pData+10);      //������չ�����   
        break;
        case SENDDOMN_GROUPNODE_LIST:                                       //PC�ڵ������Ϣ(����)
          
        break;
        case SET_NODE_PANID:
          osal_memcpy((void *)&SetPANIDPacket,pData,len);                   //���Ʊ�������
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
    }//end ������Ϣ
    
    else   //����Ϣ
    {
      
    } // end ����Ϣ ������Ϣ�������         
  }
  else  //Ⱥ��Ϣ
  {
    if(command == 0xffff)
    {
      SendDownSBoardDataPacket_t SendDownSBoardDataPacket;
      if(len>8)           //�������ܲ���
      {
        osal_memcpy((void *)&SendDownSBoardDataPacket,pData,len);        //���Ʊ�������
        SensorControl(SendDownSBoardDataPacket.ModeID,pData+9); //������չ�����
      }
      else
      {
       osal_memcpy((void *)&PCGroupMSGPacket,pData,len);                                    //���Ʊ�������
       num = 0;
       for(i=0;i<(len-1);i++)                             //����У���
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
  uint16 len = 0;
  uint8 i;
  uint8 *pdata = NULL;
  SendUpSBoardDataPacket2_t SendUpSBoardDataPacket2 ; 
  if ( event != HAL_UART_TX_EMPTY )                                          //���ڽ����¼�ȷ��
  {   
    len = HalUARTRead( HAL_UART_PORT_0, pBuf, RX_BUF_LEN );                  //������
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
    sendReport((uint8*)(&SendUpSBoardDataPacket2),SendUpSBoardDataPacket2.Len); //���÷��ͺ���
  }
}

/**************************************************************************************************
 * �������ƣ�sendReport
 *
 * �������������췢�����ݵ���Ϣ�������������߷��ͺ������������ݰ�
 *
 * ��    ������
 *           
 *
 * �� �� ֵ����
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
  static uint8 *myIEEEAddr = NULL;                       // �����豸IEEE��ַ 
  uint8 i;
  PCNodeMSGPacket.Hdr = '@';
  PCNodeMSGPacket.Len = 23;
  PCNodeMSGPacket.TransportID = 0;
  PCNodeMSGPacket.MSGCode = SENDUP_NODE_INFO;
  PCNodeMSGPacket.NodeAddr = myShortAddr;
  zb_GetDeviceInfo(ZB_INFO_IEEE_ADDR, myIEEEAddr);                         //��ȡ�豸IEEE��ַ 
   for(i=0;i<8;i++)
  {
     PCNodeMSGPacket.IEEEAddr[i] = *(myIEEEAddr+i); 
  }
  PCNodeMSGPacket.NodeRSSI = 0;
  PCNodeMSGPacket.NodeLQI = 0;
  osal_nv_read(ZCD_NV_PANID, 0, 2, &zgConfigPANID);
  PCNodeMSGPacket.Panid = zgConfigPANID;
  PCNodeMSGPacket.TemType = motherboardtype;                                 //ĸ������
  if(sensoridH == 0)
  {
   sensoridH = read24L01byte(0x01);
   sensoridL = read24L01byte(0x02);
   if((sensoridH == 0)&&(sensoridL == 0))                                    //����������Ϣ����Ϊ��ѹ���ģ��
   {
     if (HalAdcRead (HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_14)>3000)
     {
      sensoridH = 0x10;
      sensoridL = 0x01;
     }
   }
  }
 // PCNodeMSGPacket.SBoardType = resCode_Voltage_Single;                       //������������  
  PCNodeMSGPacket.SBoardType = BUILD_UINT16(sensoridL,sensoridH);           //������������ 
  sendReport((uint8*)(&PCNodeMSGPacket),23);                                  //���÷��ͺ��� 
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
  SensorCalPacket_t SensorCalPacket = {0x40,16,0,SENSOR_DATA_CAL,myShortAddr,0,0,0,0,0};
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
  sendReport((uint8*)(&SensorCalPacket),16);                                  //���÷��ͺ��� 
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
    case resCode_Voltage_Output:                 //��ѹ���     
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
      sendReport((uint8*)(&SendUpSBoardDataPacket2),SendUpSBoardDataPacket2.Len);                     //���÷��ͺ���     
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
         HalUARTWrite(HAL_UART_PORT_0,(uint8 *)((&UARTData.UARTmode)+3), UARTData.DataLen);  //ͨ��������PC�������豸ʶ����Ϣ
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
 * �������ƣ�MotherboardControl
 *
 * ��������������PC�˷��͵ĵװ������Ϣ��Ȼ�󴫵ݸ���Ӧ�Ŀ��ƺ�����
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
      sendReport((uint8*)(&SendUpTemBoardDataPacket),SendUpTemBoardDataPacket.Len);                     //���÷��ͺ���
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
  
    sendReport((uint8*)(&SendUpTemBoardDataPacket),SendUpTemBoardDataPacket.Len);                     //���÷��ͺ���
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
   
    sendReport((uint8*)(&SendUpTemBoardDataPacket),SendUpTemBoardDataPacket.Len);                     //���÷��ͺ���
    break;
   default:break;
  }
}

/**************************************************************************************************
 * �������ƣ�IICControlSensor
 *
 * ��������������PC���͵���IIC���ƴ�����������ɼ���Ӧ������ģ��Ĵ���������
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
   int tempera = 0;
   int humidity = 0;
  
    if (channel == 0x80)  //ʹ��0ͨ��
   {  
     // fortest++;
      SendUpSBoardDataPacket.data = (uint16*)HalAdcRead (HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_14);
      //SendUpSBoardDataPacket.data = (uint16*)fortest;
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
   SendUpSBoardDataPacket.TransportID = 0;
   SendUpSBoardDataPacket.MSGCode = SENDUP_NODE_SBOARD_DATA;
   SendUpSBoardDataPacket.NodeAddr = myShortAddr;
   SendUpSBoardDataPacket.ModeID = SensorID;
     
    if (SampleMode == 0) //ֹͣ����
   {
      osal_stop_timerEx( sapi_TaskID, MY_SB_MSG_EVT );
      ctrPCA9554LED(4,1);
      appControlobject = 0;
     // sendReport((uint8*)(&SendUpSBoardDataPacket),SendUpSBoardDataPacket.Len);                     //���÷��ͺ���
   }
   else if(SampleMode == 1) //���β���
   {
     sendReport((uint8*)(&SendUpSBoardDataPacket),SendUpSBoardDataPacket.Len);                     //���÷��ͺ���
     osal_stop_timerEx( sapi_TaskID, MY_SB_MSG_EVT );    
     appControlobject = 0;
   }
    else       //�������� SamplingSpeed> 100ms
   {
     
     sendReport((uint8*)(&SendUpSBoardDataPacket),SendUpSBoardDataPacket.Len);                     //���÷��ͺ���
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




