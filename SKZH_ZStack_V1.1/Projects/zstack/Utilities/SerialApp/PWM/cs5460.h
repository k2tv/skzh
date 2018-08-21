/******************************************************************************
 * cs5460.h
 *
 * Copyright 1994-2006 YiJiao Technology Co.,Ltd.
 *
 * DESCRIPTION: CS5460 ��֧����
 *
 * modification history
 * --------------------
 * 01a, 2011-06-14, cuiqingwei written
 * --------------------
 ******************************************************************************/

#ifndef _CS5460_H
#define _CS5460_H

/*-------------------------------------*
 *	             �ӿڶ��� ��           *
 *----------------------------------------------------------------------------*/
 
#define st(x)               do { x } while (__LINE__ == -1)

#define PWM_OUT1()          st(P0DIR|=0x08;P0_3=1;) 
#define PWM_OUT0()          st(P0DIR|=0x08;P0_3=0;)

#define INIT_IOS()    		st(P1DIR&=~0x80;P1DIR|=0x78;)   

#define SET_RST_1()     	st(P1DIR|=0x08;P1_3=1;)     // RST
#define SET_RST_0()     	st(P1DIR|=0x08;P1_3=0;)

#define SET_CS_1()     		st(P1DIR|=0x10;P1_4=1;)     // CS
#define SET_CS_0()     		st(P1DIR|=0x10;P1_4=0;)

#define SET_SCK_1()     	st(P1DIR|=0x20;P1_5=1;)     // SCK
#define SET_SCK_0()     	st(P1DIR|=0x20;P1_5=0;)

#define SET_SDI_1()     	st(P1DIR|=0x40;P1_6=1;)     // MOSI
#define SET_SDI_0()     	st(P1DIR|=0x40;P1_6=0;)

#define GET_SDO_X()         (P1_7)  

/*-------------------------------------*
 *	             �궨�� ��             *
 *----------------------------------------------------------------------------*/
#define  Bit_DRDY            			0x80    // 0b10000000	// ���ݾ�������У׼��ת�����ڽ���ʱ����λ  
#define  Bit_Reg_WR						0x40	//


#define  CMD_Start_ADC                  0xE8  	// 0B11101000 	// ��ʼת��
#define  CMD_PowerUp_Halt           	0xA0  	// 0B10100000 	// �ϵ硢��ͣ����

#define  CMD_Calib_AC_Gain      		0xDE    // 0B11011110 	// AC����У׼����
#define  CMD_Calib_AC_Offset     		0xDD    // 0B11011011	// ACƫ����У׼����
#define  CMD_Calib_DC_Offset     		0xD9    // 0B11011001 	// DCƫ����У׼����

#define   REG_Config     				0x40    // 0b01000000  Cfg = 0b00000 	// ���üĴ���
#define   REG_Status    				0x1E    // 0b01011110  Sdt = 0b01111 	// ״̬�Ĵ���

#define   REG_Vgn     					0x08    // 0b00001000  Vgn = 0b00100 	// ����ѹУ׼����
#define   REG_Ign         				0x04    // 0b00000100  Ign = 0b00010 	// ������У׼����

#define   REG_VACoff     				0x22    // 0b00100010  VACoff = 0b10001	// ����ѹ����У׼ƫ��
#define   REG_IACoff         			0x20    // 0b00100000  IACoff = 0b10000	// ����������У׼ƫ��

#define   REG_VDCoff     				0x06    // 0b00000110  VDCoff = 0b00011 // ����ѹֱ��У׼ƫ��
#define   REG_IDCoff         			0x02    // 0b00000010  IDCoff = 0b00001 // ������ֱ��У׼ƫ��

#define   REG_Pavg						0x14	// 0b00010100  Pavg = 0b01010	// ƽ������(���һ�μ������ڵ��ۼ�ֵ)	 
#define   REG_Irms						0x16	// 0b00010110  Irms = 0b01011 	// ������Чֵ(���һ�μ������ڵ�ֵ)		 
#define   REG_Vrms						0x18	// 0b00011000  Vrms = 0b01100 	// ��ѹ��Чֵ(���һ�μ������ڵ�ֵ)		 

/*-------------------------------------*
 *	             ����궨�� ��         *
 *----------------------------------------------------------------------------*/
#define float_long(x)     				(unsigned long)(x)
#define gain_decimal(x) 				(float)(x)/16777216  						// 2^24=16777216  

#define Pavg_Calc(x) 					SignedConversion(x)/8388608*36666.6667		// 2^23=8388608  Pactive reg=1=36666.6667W,  because:  Pactive reg=13200W=0.36
#define Vrms_Calc(x) 					(float)(x)/16777216*366.66667     			// 2^24=16777216 Vrms reg=1=366.66667V,  because:  Vrms reg=220V=0.6         
#define Irms_Calc(x) 					(float)(x)/16777216*100           			// Irms reg=1=100A     

#define Poweroffset_Update(x) 			SignedConversion(x)/8388608*1000      		//
#define phase_Update() 					computation_phase_offset()/50+0.0179		// 60Hz: 0.0215, 0.04 ; 50Hz:  0.0179, 0.04

/*-------------------------------------*
 *	            typedef  ��            *
 *----------------------------------------------------------------------------*/
typedef union          
{        
	unsigned char auc[4];     
	unsigned long ans;
}Dual;

/*-------------------------------------*
 *	            ��������  ��            *
 *----------------------------------------------------------------------------*/	

/*-------------------------------------*
 *	            �������� ��            *
 *----------------------------------------------------------------------------*/	
extern void Init_Cs5460();
extern float User_Get_Pavg();
extern float User_Get_Vrms();
extern float User_Get_Irms();

#endif

/*------------------------------------------------------------------------------
										  0ooo                                     
								ooo0	 (	 )                                     
								(	)	  ) /                                      
								 \ (	 (_/                                       
								  \_)		 Modify By:cuiqingwei [gary]                  
------------------------------------------------------------------------------*/
