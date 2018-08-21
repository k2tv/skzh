/******************************************************************************
 * cs5460.c
 *
 * Copyright 1994-2006 YiJiao Technology Co.,Ltd.
 *
 * DESCRIPTION: CS5460 驱支程序
 *
 * modification history
 * --------------------
 * 01a, 2011-06-14, cuiqingwei written
 * --------------------
 ******************************************************************************/

/*-------------------------------------*
 *	               头文件          　  *
 *----------------------------------------------------------------------------*/

// Include chip specific IO definition file
#include <ioCC2530.h>
#include "cs5460.h"		

/*-------------------------------------*
 *	            变量定义 　            *
 *----------------------------------------------------------------------------*/

/*-------------------------------------*
 *	            函数声明 　            *
 *----------------------------------------------------------------------------*/	
/////////////////////////////// 基础函数 ///////////////////////////////////////
/*-------------------------------------*
 * @fn      _delay_us
 *
 * @brief   wait for x us. @ 32MHz MCU clock it takes 32 "nop"s for 1 us _delay_us.
 *
 * @param   x us. range[0-65536]
 *
 * @return  None
 *----------------------------------------------------------------------------*/	 
static void _delay_us(unsigned int us)   
{   
    while (--us)    
    {     
        asm ( "nop" );   
    }   
}    

static void _delay_s(unsigned int s)   
{   
	unsigned int i;
	
    while (--s)    
    {     
    	for (i=0; i<1000; i++)
    	{
        	_delay_us(1000);
        }
    }   
}  
/*-------------------------------------*
 *	        有符号数换算函数           *
 *----------------------------------------------------------------------------*/

static float SignedConversion(unsigned long value)  
{
	Dual zbq;
	unsigned char fh_bit;

	zbq.ans = value;

	fh_bit = zbq.auc[2]&0x80;   	// 只提取最高位

    if (fh_bit==0x80)      		    // 负数
	{   
	    zbq.ans = ~zbq.ans+1;
	    zbq.auc[3] = 0x00;
        
	    return -(float)zbq.ans;
 	}
	else if (fh_bit!=0x80) 
	{
	 	zbq.auc[3] = 0x00;
        
		return (float)zbq.ans;
	}
    
	return (float)zbq.ans;
}
/*
float computation_phase_offset(void)  // binary complement HEX convert to float
{
	unsigned char backbyte;
	
	backbyte=phase_offset_value;
	
    if(backbyte>=0x80)      //   is negative
	{
	    backbyte=~backbyte+1;
	    return -(float)backbyte;
 	}
	else return (float)backbyte;
	
}
*/ 
/*-------------------------------------*
 *	          写8位串行数据        　  *
 *----------------------------------------------------------------------------*/  
static void Spi_Transfer_Byte(unsigned char data)
{
    unsigned char i;
    
    SET_SCK_0();
    
    for(i=0;i<8;i++)   
    {      
        SET_SCK_0();
        if (data&0x80)   
        {
            SET_SDI_1();	// MOSI=1;   
    	}
        else  
        {
            SET_SDI_0();	// MOSI=0;   
      	}
        data <<= 1;       
        SET_SCK_1();
    }
    SET_SCK_0();
}

/*-------------------------------------*
 *	          读8位串行数据        　  *
 *----------------------------------------------------------------------------*/  
static unsigned char Spi_Receive_Byte()
{
    unsigned char i,data=0;
    
    /* 循环8次，写入数据 */ 
    for (i=0; i<8; i++)   
    { 
        SET_SCK_0();
        _delay_us(2);
        SET_SCK_1();
        
        data <<= 1;
        if(GET_SDO_X())   
        {
            data |= 0x01;
        } 
        else
        {
            data |= 0x00;
        }    
    }
    
    return (data);
}

/////////////////////////////// 接口函数 ///////////////////////////////////////
/*-------------------------------------*
 *	            写寄存器           　  *
 *----------------------------------------------------------------------------*/ 
static void Write_Cs5460_Command(unsigned char cmd)
{ 
    SET_CS_1();
    _delay_us(10);
    SET_SCK_0();
    _delay_us(10);
    SET_CS_0();
    _delay_us(10);
    
	Spi_Transfer_Byte(cmd);
 
    SET_SCK_0();
    SET_SDI_0();
    SET_CS_1();
}

/*-------------------------------------*
 *	            写寄存器           　  *
 *----------------------------------------------------------------------------*/ 
static void Write_Cs5460_Register(unsigned char cmd, unsigned char high,unsigned char mid,unsigned char low)
{ 
    SET_CS_1();
    _delay_us(10);
    SET_SCK_0();
    _delay_us(10);
    SET_CS_0();
    _delay_us(10);
    
    cmd |= Bit_Reg_WR;
	Spi_Transfer_Byte(cmd);
	Spi_Transfer_Byte(high);
	Spi_Transfer_Byte(mid);
 	Spi_Transfer_Byte(low);
 
    SET_SCK_0();
    SET_SDI_0();
    SET_CS_1();
}

/*-------------------------------------*
 *	            读寄存器           　  *
 *----------------------------------------------------------------------------*/ 
static void Read_Cs5460_Register(unsigned char cmd, unsigned char *pData)
{
    unsigned char i;
    
    SET_CS_1();
    _delay_us(10);
    SET_SCK_0();
    _delay_us(10);
    SET_CS_0();
    _delay_us(10);
    
    Spi_Transfer_Byte(cmd);
    
    SET_SDI_1();
    for (i=0; i<3; i++)
    {
        *(pData+i) = Spi_Receive_Byte();
    }

    SET_SCK_0();
    SET_SDI_0();
    SET_CS_1();
}

/*-----------------------------------------------------*
 *	CS5460等待状态寄存器最高位DRDY位置1，表明转换完毕  *
 *----------------------------------------------------------------------------*/
static void Wait_DRDY_High()
{
    unsigned char buf[4];
    
    while(1)
    {
        Read_Cs5460_Register(REG_Status,buf);
        
        if(buf[0]&Bit_DRDY)
        {
            break;  
        }
    }
}

/*-------------------------------------*
 *	             同步字            　  *
 *----------------------------------------------------------------------------*/     
static void Sync_Cs5460()   
{   
    SET_CS_1();
    _delay_us(10);
    SET_SCK_0();
    _delay_us(10);
    SET_CS_0();
    _delay_us(10);
    
	Spi_Transfer_Byte(0xff);	// SYNC1
	Spi_Transfer_Byte(0xff);	// SYNC1
	Spi_Transfer_Byte(0xff);	// SYNC1
 	Spi_Transfer_Byte(0xfe);	// SYNC0
 
    SET_SCK_0();
    SET_SDI_0();
    SET_CS_1(); 
}  

/*-------------------------------------*
 *	             复位              　  *
 *----------------------------------------------------------------------------*/     
static void Reset_Cs5460()   
{       
	SET_RST_1();  
    _delay_us(1000);      	
	SET_RST_0();            // 复位
    _delay_us(30000);      	// 30ms
    SET_RST_1();  
}  

/*-------------------------------------*
 *	            读取数据           　  *
 *----------------------------------------------------------------------------*/
static long Read_Cs5460_Value(unsigned char cmd)
{
	Dual temp;
	unsigned char ch[3];
	
    Wait_DRDY_High();
    Read_Cs5460_Register(cmd,(void *)ch);

	temp.auc[3] = 0;
    temp.auc[2] = ch[0];  // high
	temp.auc[1] = ch[1];  // mid
	temp.auc[0] = ch[2];  // low

	return (temp.ans);
}
/////////////////////////////// 校准函数 ///////////////////////////////////////
/*-------------------------------------*
 *	          CS5460校准函数      　   *
 *----------------------------------------------------------------------------*/  
 
static void write_ACV_gain_ride_coefficient()
{
    unsigned char ch[3];
    Dual ride, rtn;
    float  zzz;
   
    Read_Cs5460_Register(REG_Vgn,(void*)ch); 						// read voltage gain register value
    ride.auc[3] = 0x00;
    ride.auc[2] = ch[0];  	// high
  	ride.auc[1] = ch[1];  	// mid
   	ride.auc[0] = ch[2];  	// low
   	
    ride.ans <<= 2;
 	
    switch(ride.auc[3])
  	{
 		case 0:
			zzz = gain_decimal(ride.ans)*16792720;	 				// 16777216*1.00092=16792720.6426
			ride.ans = float_long(zzz);   
			rtn.ans  = ride.ans>>2;
			break;

		case 1:
			ride.auc[3] = 0x00;
			zzz = (gain_decimal(ride.ans)+1)*16792720;
   			ride.ans = float_long(zzz); 
   			rtn.ans  = ride.ans>>2;
   			break;
       				
    	case 2:
    		ride.auc[3] = 0x00;
   			zzz = (gain_decimal(ride.ans)+2)*16792720;    
   			ride.ans = float_long(zzz);    
   			rtn.ans  = ride.ans>>2;
   			break;
       				
       	case 3:
 		    ride.auc[3] = 0x00;
   			zzz = (gain_decimal(ride.ans)+3)*16792720;
    		ride.ans = float_long(zzz);    
   			rtn.ans  = ride.ans>>2;
   			break;
    
		default:
   			return;
       				
    }
    
    ch[0] = rtn.auc[2];  // high
	ch[1] = rtn.auc[1];  // mid
	ch[2] = rtn.auc[0];  // low

	// Write_EEPROM(OFFSET_OF(EEPROM_DATA,eprom_V_ac_gain),3, ch);
	Write_Cs5460_Register(REG_Vgn,ch[0],ch[1],ch[2]);				// write new voltage gain register value
	
}

static void write_ACI_gain_ride_coefficient()
{
    unsigned char ch[3];
    Dual ride, rtn;
    float zzz;
   
    Read_Cs5460_Register(REG_Ign,(void*)ch); 						// read current gain register value
    ride.auc[3] = 0x00;
    ride.auc[2] = ch[0];  // high
  	ride.auc[1] = ch[1];  // mid
   	ride.auc[0] = ch[2];  // low
   	
    ride.ans <<= 2;
 	
    switch(ride.auc[3])
  	{
 		case 0:
			zzz = gain_decimal(ride.ans)*16792720;    				// 16777216*1.00092=16792720.6426
			ride.ans = float_long(zzz);   
			rtn.ans  = ride.ans>>2;
			break;

		case 1:
			ride.auc[3] = 0x00;
			zzz = (gain_decimal(ride.ans)+1)*16792720;
			ride.ans = float_long(zzz); 
			rtn.ans  = ride.ans>>2;
			break;
       				
    	case 2:
    		ride.auc[3] = 0x00;
   			zzz = (gain_decimal(ride.ans)+2)*16792720;    
   			ride.ans = float_long(zzz);    
   			rtn.ans  = ride.ans>>2;
   			break;
       				
       	case 3:
 		    ride.auc[3] = 0x00;
   			zzz = (gain_decimal(ride.ans)+3)*16792720;
    		ride.ans = float_long(zzz);    
   			rtn.ans  = ride.ans>>2;
   			break;
    
		default:
   			return;
       				
    }
    
    ch[0] = rtn.auc[2];  // high
	ch[1] = rtn.auc[1];  // mid
	ch[2] = rtn.auc[0];  // low

	//Write_EEPROM(OFFSET_OF(EEPROM_DATA,eprom_I_ac_gain),3, ch);
	Write_Cs5460_Register(REG_Ign,ch[0],ch[1],ch[2]);				// write new current gain register value
	
}

static void write_AC_offset_to_eeprom()
{
    unsigned char ch[3];

    Read_Cs5460_Register(REG_VACoff,(void *)ch);					// read AC voltage offset register value
    //Write_EEPROM(OFFSET_OF(EEPROM_DATA,eprom_V_ac_off),3,ch);
	
    Read_Cs5460_Register(REG_IACoff,(void *)ch);					// read AC current offset register value
    //Write_EEPROM(OFFSET_OF(EEPROM_DATA,eprom_I_ac_off),3,ch);
}

static void write_DC_offset_to_eeprom()
{
    unsigned char ch[3];

    Read_Cs5460_Register(REG_VDCoff,(void *)ch); 					// read DC voltage offset register value
   	//Write_EEPROM(OFFSET_OF(EEPROM_DATA,eprom_V_dc_off),3,ch);
   
    Read_Cs5460_Register(REG_IDCoff,(void*)ch); 					// read DC current offset register value
	//Write_EEPROM(OFFSET_OF(EEPROM_DATA,eprom_I_dc_off),3,ch);
}

 /*-------------------------------------*
 *	           交流增益校准         　  *
 *----------------------------------------------------------------------------*/     
void Clibration_Cs5460_AC_Gain()
{
	Write_Cs5460_Command(CMD_PowerUp_Halt);							// stop computation, Power-Up/Halt
	_delay_us(1);
	
	//Write_Cs5460_Register(0x68,0x40,0x00,0x00);					// write system gain register value =1;
	
	Write_Cs5460_Register(REG_Vgn,0x40,0x00,0x00);					// write voltage gain register default value 1.0
	Write_Cs5460_Register(REG_Ign,0x40,0x00,0x00);					// write current gain register default value 1.0

	Write_Cs5460_Command(CMD_Calib_AC_Gain);         				// voltage and current channel AC_gain calibration        
	_delay_s(10);
	
    write_ACV_gain_ride_coefficient();
    write_ACI_gain_ride_coefficient();
	
  	Write_Cs5460_Command(CMD_Start_ADC); 		    				// start conversion

}

/*-------------------------------------*
 *	           交流偏移校准         　 *
 *----------------------------------------------------------------------------*/ 
void Clibration_Cs5460_AC_Offset()
{
    Write_Cs5460_Command(CMD_PowerUp_Halt);							// stop computationd, Power-Up/Halt
    _delay_us(1);
	
	Write_Cs5460_Register(REG_VACoff,0x00,0x00,0x00); 				// write AC voltage offset register default value
	Write_Cs5460_Register(REG_IACoff,0x00,0x00,0x00); 				// write AC current offset register default value
	
	Write_Cs5460_Command(CMD_Calib_AC_Offset); 						// voltage and current channel AC_offset calibration
    _delay_s(10);
    
    write_AC_offset_to_eeprom();
    
  	Write_Cs5460_Command(CMD_Start_ADC); 							// start conversion
}

/*-------------------------------------*
 *	           直流偏移校准         　 *
 *----------------------------------------------------------------------------*/ 
void Clibration_Cs5460_DC_Offset()
{
    Write_Cs5460_Command(CMD_PowerUp_Halt);							// stop computationd, Power-Up/Halt
   	_delay_us(1);
    
	Write_Cs5460_Register(REG_VDCoff,0x00,0x00,0x00);				// write DC voltage offset register default value
	Write_Cs5460_Register(REG_IDCoff,0x00,0x00,0x00);				// write DC current offset register default value
	
	Write_Cs5460_Command(CMD_Calib_DC_Offset);       				// voltage and current channel DC_offset calibration
    _delay_s(10);
    
    write_DC_offset_to_eeprom();
	
  	Write_Cs5460_Command(CMD_Start_ADC); 							// start conversion
    
}
/////////////////////////////// 用户函数 /////////////////////////////////////// 
/*-------------------------------------*
 *	           CS5460初始化         　  *
 *----------------------------------------------------------------------------*/ 
void Init_Cs5460()
{
	INIT_IOS();
     
	Reset_Cs5460();
    
    Sync_Cs5460();
     
  	/* 写配置寄存器 
       GI=1,电流通道增益=50
       中断形式：00-高电平 08-低电平 10-下降沿 18-上升沿
       DCLK=MCLK/1 */
    Write_Cs5460_Register(REG_Config,0x01,0x00,0x61);
 
    
  //  Write_Cs5460_Register(0x6c,0x00,0x01,0x40);  	// write pulse width  E3 = 80ms , 0x000140
	
    Write_Cs5460_Command(CMD_Start_ADC);    		// 启动Cs5460，执行连续周期计算
    
    Clibration_Cs5460_AC_Gain();
}

/*-------------------------------------*
 *	平均功率(最后一次计算周期的累计值) *
 *----------------------------------------------------------------------------*/
float User_Get_Pavg()
{ 
	return(Pavg_Calc(Read_Cs5460_Value(REG_Pavg))/1.00185); 
}

/*-------------------------------------*
 *	 电流有效值(最后一次计算周期的值)  *
 *----------------------------------------------------------------------------*/
float User_Get_Irms()
{
	return(Irms_Calc(Read_Cs5460_Value(REG_Irms))/1.000924); 
}
/*-------------------------------------*
 *	 电压有效值(最后一次计算周期的值)  *
 *----------------------------------------------------------------------------*/
float User_Get_Vrms()
{
	return(Vrms_Calc(Read_Cs5460_Value(REG_Vrms))/1.000924); 
}

/*------------------------------------------------------------------------------
										  0ooo                                     
								ooo0	 (	 )                                     
								(	)	  ) /                                      
								 \ (	 (_/                                       
								  \_)		 Modify By:cuiqingwei [gary]                  
------------------------------------------------------------------------------*/
