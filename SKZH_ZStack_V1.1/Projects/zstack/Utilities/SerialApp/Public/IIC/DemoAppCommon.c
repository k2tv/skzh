/***************************************************************************************************************
* �� �� ����DemoAppCommon.c
��
* ��    �ܣ�SensorDemo�û�Ӧ�ò㹫���ĺ�����        
*
* ��    ����V1.0
* ��    �ߣ�WU XIANHAI
* ��    �ڣ�2010.10.25
* �¶�˹������ҳ��www.ourselec.com
******************************************************************************************************************/

#include "hal_uart.h"
#include "DemoApp.h"

/**************************************************************************************************
 * �������ƣ�initUart
 *
 * ������������ʼ������
 *
 * ��    ����pf - ָ��UART ���жϽ��պ���
 *
 * �� �� ֵ����
 **************************************************************************************************/
void initUart(halUARTCBack_t pf,uint8 UARTBR)
{
  halUARTCfg_t uartConfig;
  
  uartConfig.configured           = TRUE;
 // uartConfig.baudRate             = HAL_UART_BR_115200;
  uartConfig.baudRate             = UARTBR;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 48;
  uartConfig.rx.maxBufSize        = RX_BUF_LEN;
  uartConfig.tx.maxBufSize        = 256;  
  
  uartConfig.idleTimeout          = 6;   
  uartConfig.intEnable            = TRUE;              
  uartConfig.callBackFunc         = pf;
  
  HalUARTOpen (HAL_UART_PORT_0, &uartConfig);
}


