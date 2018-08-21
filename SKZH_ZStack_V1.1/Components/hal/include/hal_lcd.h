/**************************************************************************************************
  Filename:       hal_lcd.h
  Revised:        $Date: 2007-07-06 10:42:24 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Description:    This file contains the interface to the LCD Service.


  Copyright 2005-2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

#ifndef HAL_LCD_H
#define HAL_LCD_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 *                                          INCLUDES
 **************************************************************************************************/
#include "hal_board.h"



/**************************************************************************************************
 *                                          MACROS
 **************************************************************************************************/
#define     RED          0XF800	  //红色
#define     GREEN        0X07E0	  //绿色
#define     BLUE         0X001F	  //蓝色
#define     WHITE        0XFFFF	  //白色
#define     BLACK        0X0000	  //黑色
#define     YELLOW       0XFFE0	  //黄色
#define     ORANGE       0XFC08	  //橙色
#define     GRAY  	 0X8430   //灰色
#define     LGRAY        0XC618	  //浅灰色
#define     DARKGRAY     0X8410	  //深灰色
#define     PORPO        0X801F	  //紫色
#define     PINK         0XF81F	  //粉红色
#define     GRAYBLUE     0X5458   //灰蓝色
#define     LGRAYBLUE    0XA651   //浅灰蓝色
#define     DARKBLUE     0X01CF	  //深灰蓝色
#define     LIGHTBLUE    0X7D7C	  //浅蓝色


/**************************************************************************************************
 *                                         TYPEDEFS
 **************************************************************************************************/


/**************************************************************************************************
 *                                     GLOBAL VARIABLES
 **************************************************************************************************/
extern uint16  PointColor;
extern uint16  BackColor;

/**************************************************************************************************
 *                                     FUNCTIONS - API
 **************************************************************************************************/

/*
 * Initialize LCD Service
 */
extern void LCD_WR_DATA(uint16 val); //指定数据
extern void LCD_WR_REG(uint16 reg);  //指定寄存器
extern void LCD_WR_REG_DATA(uint16 REG, uint16 VALUE);
extern void HalLcdInit(void);
extern void LCD_Clear(uint32 color);
extern void LCD_DisplayOn(void);
extern void LCD_DisplayOff(void);
extern void LCD_XYRAM(uint16 xstart ,uint16 ystart ,uint16 xend ,uint16 yend);
extern void LCD_SetC(uint16 x, uint16 y);
extern void LCD_DrawPoint(uint16 x,uint16 y);
extern void LCD_DrawCircle(uint8 x0, uint16 y0, uint8 r);
extern void LCD_DrawLine(uint16 x1, uint16 y1, uint16 x2, uint16 y2);
extern void LCD_DrawRectage(uint16 xstart,uint16 ystart,uint16 xend,uint16 yend,uint16 color);
extern void LCD_Fill(uint16 xstart ,uint16 ystart ,uint16 xend ,uint16 yend ,uint16 color);
extern void LCD_ShowNum(uint8 x,uint16 y,uint32 num,uint8 len);
extern void LCD_ShowString(uint16 x,uint16 y,uint8 *p);


/**************************************************************************************************
**************************************************************************************************/

#ifdef __cplusplus
}
#endif

#endif
