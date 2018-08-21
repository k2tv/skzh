/***************************************************************************************************************
* 文 件 名：DemoAppCommon.c
×
* 功    能：SensorDemo用户应用层公共的函数。        
*
* 版    本：V1.0
* 作    者：WU XIANHAI
* 日    期：2010.10.25
* 奥尔斯电子主页：www.ourselec.com
******************************************************************************************************************/

#include "hal_uart.h"
#include "DemoApp.h"

/**************************************************************************************************
 * 函数名称：initUart
 *
 * 功能描述：初始化串口
 *
 * 参    数：pf - 指向UART 的中断接收函数
 *
 * 返 回 值：无
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


