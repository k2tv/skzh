/**********************************************************************************************************
* 文 件 名：iic.C
* 功    能：此程序是I2C操作平台（主方式的软件平台）的底层的C子程序,如发送数据及接收数据,应答位发送
* 硬件连接：采用CC2530的I/O口模拟IIC的SCL和SDA
*
*           P1.0 ------ SCL
*           P1.1 ------ SDA
*           
* 版    本：V1.0
* 
* 
* 
**************************************************************************************************************/
#include "ioCC2530.h"
#include "hal_mcu.h"

#define SCL          P1_0 
#define SDA          P1_1

#define IO_DIR_PORT_PIN(port, pin, dir)  \
   do {                                  \
      if (dir == IO_OUT)                 \
         P##port##DIR |= (0x01<<(pin));  \
      else                               \
         P##port##DIR &= ~(0x01<<(pin)); \
   }while(0)


#define IO_IN   0
#define IO_OUT  1

//#define TMP275_I2CADDR  0x92

static uint8 ack;	         /*应答标志位*/

void QWait(void);
//static void Wait(unsigned int ms);
static void Start_I2c(void);
static void Stop_I2c(void);
static void  SendByte(uint8 c);
uint8  RcvByte(void);
static void Ack_I2c(uint8 a);
static uint8 ISendByte(uint8 sla,uint8 c);
static uint8 ISendStr(uint8 sla,uint8 suba,uint8 *s,uint8 no);
uint8 IRcvStr(uint8 sla,uint8 suba,uint8 *s,uint8 no);
//static uint8 IRcvStr_test(uint8 sla,uint8 *s,uint8 no);
//float iicreadtpm275(void);
void ledInit(void);
void RelayInit(void);
void FLASHLED(uint8 led);  
void FLASHLED4(uint8 FLASHnum);   //LED4闪烁
void FLASHLED5(uint8 FLASHnum);   //LED5闪烁
void LED(uint8 led,uint8 operation);
void Buzzer(uint8 operation);
uint8 Relay(uint8 cmd);
uint8 SRelay(uint8 cmd);
uint8 IRcvByte(uint8 sla,uint8 *c);
uint8 Key(void);
void Relays(uint8 relay,uint8 operation);
uint8 write24L01(uint8 *data,uint8 addr,uint8 Len);
uint8 read24L01byte(uint8 addr);
uint8 VoltageOutput(uint16 Voltage,uint8 Port);
void RelayInit(void);
char s_write_byte(unsigned char value);
char s_read_byte(unsigned char ack);



uint8 ledstate   = 0;
uint8 Relaystate = 0;
void QWait()     //1us的延时
{
    asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");
    asm("NOP");

}

//static void Wait(unsigned int ms)
//{
                    
//   unsigned char g,k;
 //  while(ms)
 //  {
      
	//  for(g=0;g<=167;g++)
	 //  {
	 //    for(k=0;k<=48;k++);
	 //  }
     // ms--;                            
   //}
//} 

/*******************************************************************
                     起动总线函数               
函数原型: void  Start_I2c();  

功    能: 启动I2C总线,即发送I2C起始条件.
  
********************************************************************/
static void Start_I2c()
{
  IO_DIR_PORT_PIN(1, 0, IO_OUT);    //设置P1.0为输出
  IO_DIR_PORT_PIN(1, 1, IO_OUT);    //设置P1.1为输出
  
  SDA=1;   /*发送起始条件的数据信号*/
  asm("NOP");
  SCL=1;
  QWait();    /*起始条件建立时间大于4.7us,延时*/
  QWait();
  QWait();
  QWait();
  QWait();    
  SDA=0;   /*发送起始信号*/
  QWait();    /* 起始条件锁定时间大于4μs*/
  QWait();
  QWait();
  QWait();
  QWait();       
  SCL=0;   /*钳住I2C总线，准备发送或接收数据 */
  asm("NOP");
  asm("NOP");
}

/*******************************************************************
                      结束总线函数               
函数原型: void  Stop_I2c();  

功    能: 结束I2C总线,即发送I2C结束条件.
  
********************************************************************/
static void Stop_I2c()
{
  IO_DIR_PORT_PIN(1, 0, IO_OUT);    //设置P1.0为输出
  IO_DIR_PORT_PIN(1, 1, IO_OUT);    //设置P1.1为输出
  SDA=0;  /*发送结束条件的数据信号*/
  asm("NOP");   /*发送结束条件的时钟信号*/
  SCL=1;  /*结束条件建立时间大于4μs*/
  QWait();
  QWait();
  QWait();
  QWait();
  QWait();
  SDA=1;  /*发送I2C总线结束信号*/
  QWait();
  QWait();
  QWait();
  QWait();
}


/*******************************************************************
                 字节数据传送函数               
函数原型: void  SendByte(uchar c);

功    能: 将数据c发送出去,可以是地址,也可以是数据,发完后等待应答,并对
          此状态位进行操作.(不应答或非应答都使ack=0 假)     
          发送数据正常，ack=1; ack=0表示被控器无应答或损坏。
********************************************************************/
static void  SendByte(uint8 c)
{
 uint8 BitCnt;
 IO_DIR_PORT_PIN(1, 0, IO_OUT);    //设置P1.0为输出
 IO_DIR_PORT_PIN(1, 1, IO_OUT);    //设置P1.1为输出
 for(BitCnt=0;BitCnt<8;BitCnt++)  /*要传送的数据长度为8位*/
    {
     if((c<<BitCnt)&0x80)SDA=1;   /*判断发送位*/
       else  SDA=0;                
      asm("NOP");
      asm("NOP");
      asm("NOP");
     SCL=1;               /*置时钟线为高，通知被控器开始接收数据位*/
      QWait(); 
      QWait();               /*保证时钟高电平周期大于4μs*/
      QWait();
      QWait();
      QWait();         
     SCL=0; 
    }    
    QWait();
    QWait();
    QWait();
    SDA=1;               /*8位发送完后释放数据线，准备接收应答位*/
    asm("NOP");
    IO_DIR_PORT_PIN(1, 1, IO_IN);  
    SCL=1;
    QWait();          
    //QWait();
    //QWait();
    //QWait();
    if(SDA==1)ack=0;     
    else ack=1;        /*判断是否接收到应答信号*/
    SCL=0;   
    QWait();
    QWait();
    IO_DIR_PORT_PIN(1, 1, IO_OUT);
}

/*******************************************************************
                 字节数据传送函数               
函数原型: uchar  RcvByte();

功    能: 用来接收从器件传来的数据,并判断总线错误(不发应答信号)，
          发完后请用应答函数。  
********************************************************************/	
static uint8  RcvByte()
{
  uint8 retc;
  uint8 BitCnt;
  IO_DIR_PORT_PIN(1, 0, IO_OUT);    //设置P1.0为输出
 
  retc=0; 
 // SDA=1;             /*置数据线为输入方式*/
  IO_DIR_PORT_PIN(1, 1, IO_IN);
  for(BitCnt=0;BitCnt<8;BitCnt++)
      {
        asm("NOP");          
        SCL=0;       /*置时钟线为低，准备接收数据位*/
        QWait();
        QWait();         /*时钟低电平周期大于4.7μs*/
        QWait();
        QWait();
        QWait();
        SCL=1;       /*置时钟线为高使数据线上数据有效*/
        QWait();
        QWait();
        retc=retc<<1;
        if(SDA==1)retc=retc+1; /*读数据位,接收的数据位放入retc中 */
        QWait();
        QWait(); 
      }
  SCL=0;    
  QWait();
  QWait();
  IO_DIR_PORT_PIN(1, 1, IO_OUT);
  return(retc);
}

/********************************************************************
                     应答子函数
原型:  void Ack_I2c(uint a);
 
功能:主控器进行应答信号,(可以是应答或非应答信号)
********************************************************************/
static void Ack_I2c(uint8 a)
{
  IO_DIR_PORT_PIN(1, 0, IO_OUT);    //设置P1.0为输出
  IO_DIR_PORT_PIN(1, 1, IO_OUT);    //设置P1.1为输出
  if(a==0)SDA=0;     /*在此发出应答或非应答信号 */
  else SDA=1;
  QWait();
  //QWait();      //change by wu 2011.3.1
  //QWait();      
  SCL=1;
  QWait();
  QWait();              /*时钟低电平周期大于4μs*/
  QWait();
  QWait();
  QWait();  
  SCL=0;                /*清时钟线，钳住I2C总线以便继续接收*/
  QWait();
  //QWait();  
  IO_DIR_PORT_PIN(1, 1, IO_IN);// change by wu 2011.3.1
}

/*******************************************************************
                    向无子地址器件发送字节数据函数               
函数原型: uint  ISendByte(uchar sla,ucahr c);

功    能:  从启动总线到发送地址，数据，结束总线的全过程,从器件地址sla.
           如果返回1表示操作成功，否则操作有误。

注    意： 使用前必须已结束总线。
********************************************************************/
static uint8 ISendByte(uint8 sla,uint8 c)
{
   Start_I2c();               /*启动总线*/
   SendByte(sla);            /*发送器件地址*/
     if(ack==0)return(0);
   SendByte(c);               /*发送数据*/
     if(ack==0)return(0);
  Stop_I2c();                 /*结束总线*/ 
  return(1);
}

/*******************************************************************
                    向有子地址器件发送多字节数据函数               
函数原型: uint  ISendStr(uchar sla,uchar suba,ucahr *s,uchar no);  

功    能: 从启动总线到发送地址，子地址,数据，结束总线的全过程,从器件
          地址sla，子地址suba，发送内容是s指向的内容，发送no个字节。
          如果返回1表示操作成功，否则操作有误。

注    意：使用前必须已结束总线。
********************************************************************/
static uint8 ISendStr(uint8 sla,uint8 suba,uint8 *s,uint8 no)
{
   uint8 i;

   Start_I2c();               /*启动总线*/
   SendByte(sla);            /*发送器件地址*/
     if(ack==0)return(0);
   SendByte(suba);            /*发送器件子地址*/
     if(ack==0)return(0);

   for(i=0;i<no;i++)
    {   
     SendByte(*s);               /*发送数据*/
       if(ack==0)return(0);
     s++;
    } 
 Stop_I2c();                 /*结束总线*/ 
  return(1);
}


/*******************************************************************
                    向无子地址器件读字节数据函数               
函数原型: uint  IRcvByte(uchar sla,ucahr *c);  

功    能: 从启动总线到发送地址，读数据，结束总线的全过程,从器件地
          址sla，返回值在c. 如果返回1表示操作成功，否则操作有误。

注    意：使用前必须已结束总线。
********************************************************************/
uint8 IRcvByte(uint8 sla,uint8 *c)
{
   Start_I2c();                /*启动总线*/
   SendByte(sla+1);           /*发送器件地址*/
 
   if(ack==0)return(0);
   *c=RcvByte();               /*读取数据*/
   Ack_I2c(1);               /*发送非就答位*/
   Stop_I2c();                  /*结束总线*/ 
   return(1);
}

/*******************************************************************
                    向有子地址器件读取多字节数据函数               
函数原型: uint  ISendStr(uchar sla,uchar suba,ucahr *s,uchar no); 

功    能: 从启动总线到发送地址，子地址,读数据，结束总线的全过程,从器件
          地址sla，子地址suba，读出的内容放入s指向的存储区，读no个字节。
          如果返回1表示操作成功，否则操作有误。

注    意：使用前必须已结束总线。
********************************************************************/
uint8 IRcvStr(uint8 sla,uint8 suba,uint8 *s,uint8 no)
{
   uint8 i;

   Start_I2c();               /*启动总线*/
   SendByte(sla);            /*发送器件地址*/
   if(ack==0)return(0);
   SendByte(suba);            /*发送器件子地址*/
   if(ack==0)return(0);
   Start_I2c();
   SendByte(sla+1);
   if(ack==0)return(0);
   for(i=0;i<no-1;i++)
    {   
      *s=RcvByte();               /*发送数据*/
      Ack_I2c(0);                /*发送就答位*/  
      s++;
    } 
   *s=RcvByte();
   Ack_I2c(1);                 /*发送非应位*/
   Stop_I2c();                    /*结束总线*/ 
   return(1);
}

//static uint8 IRcvStr_test(uint8 sla,uint8 *s,uint8 no)
//{
   //uint8 i;

//   Start_I2c();               /*启动总线*/
//   SendByte(sla);            /*发送器件地址*/
//   if(ack==0)return(0);
   //SendByte(suba);            /*发送器件子地址*/
  // if(ack==0)return(0);
   //Start_I2c();
  // SendByte(sla+1);
  // if(ack==0)return(0);
//   for(i=0;i<no-1;i++)
//    {   
//      *s=RcvByte();               /*发送数据*/
 //     Ack_I2c(0);                /*发送就答位*/  
 //     s++;
//    } 
//   while(no > 0) 
//   {
 //   *s++ = RcvByte();
 //    if(no > 1)  Ack_I2c(0);                /*发送就答位*/ 
 //    else Ack_I2c(1);                 /*发送非应位*/
 //    no--;
 //  }
//   *s=RcvByte();
//   Ack_I2c(1);                 /*发送非应位*/
//   Stop_I2c();                    /*结束总线*/ 
 //  return(1);
//}

//float iicreadtpm275(void)
//{ 
//   uint8 data[2];
//   float tmp;
//    ISendByte(TMP275_I2CADDR,0);
//    IRcvStr_test(0x93,data,2);
//    tmp = (float)data[0] + (float)(data[1]>>4) * 0.0625;
//    return tmp; 
//}

void ledInit(void)
{
  uint8 output = 0x00;
  uint8 *data = 0;
  if(ISendStr(0x40,0x03,&output,1))  //写配置
  {
    output = 0xbf;
    if(ISendStr(0x40,0x01,&output,1))
    {
      if(IRcvByte(0x40,data))
      {
        ledstate = *data;
      }
    }
  }
}
/******************************************************************************
 * 函数名称：FLASHLED
 *
 * 功能描述：通过IIC总线控制PCA9554的输出，在输出控制前，先判断所控制PCA9554输出
 *           寄存器的值，然后对所控制的位进行取反控制。
 *                    
 *
 * 参    数：led - 对应PCA9554的输出端口
 *           
 *
 * 返 回 值：无
 *           
 *
 * 注    意：PCA9554的地址为：0x40
 *****************************************************************************/ 

void FLASHLED(uint8 led)   
{
  uint8 output = 0x00;
  uint8 *data = 0;
  if(ISendStr(0x40,0x03,&output,1))    //写配置
  {
    switch(led)
    {
     case 3:     
     output = ledstate & 0x01;  //判断所控制位的当前状态 
      if (output)                      //如果当前该位状态为1，则改变输出为0
     {
       output = ledstate & 0xfe;
     }
      else                             //如果当前状态为0，则改变输出为1
     {
       output = ledstate | 0x01;
     }
     break;
     case 4:     
     output = ledstate & 0x02;
      if (output)
     {
       output = ledstate & 0xfd;
     }
      else
     {
       output = ledstate | 0x02;
     }
     break;
     case 5:     
     output = ledstate & 0x08;
      if (output)
     {
       output = ledstate & 0xf7;
     }
      else
     {
       output = ledstate | 0x08;
     }
     break;
     case 6:     
     output = ledstate & 0x04;
      if (output)
     {
       output = ledstate & 0xfb;
     }
      else
     {
       output = ledstate | 0x04;
     }
     break;
     case 1:     
     output = ledstate & 0x20;
      if (output)
     {
       output = ledstate & 0xdf;
     }
      else
     {
       output = ledstate | 0x20;
     }
     break;
     case 2:     
     output = ledstate & 0x10;
      if (output)
     {
       output = ledstate & 0xef;
     }
      else
     {
       output = ledstate | 0x10;
     }
     break;
     default: break;
    }
     if(ISendStr(0x40,0x01,&output,1))    //写输出寄存器
    {
      if(IRcvByte(0x40,data))             //读输出寄存器
      {
        ledstate = *data;          //保存输出寄存器当前状态。
      }
    }
  }
}
void FLASHLED4(uint8 FLASHnum)   //LED4闪烁
{
  uint8 output = 0x00;
  uint8 *data = 0;
  if(ISendStr(0x40,0x03,&output,1))  //写配置
  {
    if(FLASHnum)
    {
     output = ledstate & 0x20;
     if (output)
    {
      output = ledstate & 0xdf;
    }
    else
    {
      output = ledstate | 0x20;
    }
     if(ISendStr(0x40,0x01,&output,1))
    {
      if(IRcvByte(0x40,data))
      {
        ledstate = *data;
      }
    }
    //FLASHnum --;
   // Wait(50);
    }
  }
}

void FLASHLED5(uint8 FLASHnum)   //LED5闪烁
{
  uint8 output = 0x00;
  uint8 *data = 0;
  if(ISendStr(0x40,0x03,&output,1))  //写配置
  {
    if(FLASHnum)
    {
    output = ledstate & 0x10;
    if (output)
    {
      output = ledstate & 0xef;
    }
    else
    {
      output = ledstate | 0x10;
    }
     if(ISendStr(0x40,0x01,&output,1))
    {
      if(IRcvByte(0x40,data))
      {
        ledstate = *data;
      }
    }
   // FLASHnum --;
   // Wait(50);
    }
  }
}

/******************************************************************************
 * 函数名称：LED
 *
 * 功能描述：通过IIC总线控制PCA9554的输出，进而控制相应的LED。
 *                    
 *
 * 参    数：LED - 所控制的LED
 *           operation - 开或关操作
 *
 * 返 回 值：无
 *           
 *
 * 注    意：PCA9554的地址为：0x40
 *****************************************************************************/ 
void LED(uint8 led,uint8 operation)
{
  uint8 output = 0x00;
  uint8 *data = 0;
  if(ISendStr(0x40,0x03,&output,1))  //配置PCA9554寄存器
  {
    switch(led)
    {
      case 1:                        //LED0控制
         if (operation)
        {
          output = ledstate & 0xdf;
        }
        else
        {
          output = ledstate | 0x20;
        }
        
      break;
       case 2:                      //LED1控制
        if (operation)
        {
          output = ledstate & 0xef;
        }
        else
        {
          output = ledstate | 0x10;
        }
       
      break;
       case 3:                     //LED2控制
        if (operation)
        {
          output = ledstate & 0xfe;
        }
        else
        {
          output = ledstate | 0x01;
        }
        
      break;
       case 4:                     //LED3控制
        if (operation)
        {
          output = ledstate & 0xfd;
        }
        else
        {
          output = ledstate | 0x02;
        }
        
      break;
       case 5:                    //LED4控制
        if (operation)
        {
          output = ledstate & 0xfb;
        }
        else
        {
          output = ledstate | 0x04;
        }
        
      break;
       case 6:                   //LED5控制
        if (operation)
        {
          output = ledstate & 0xf7;
        }
        else
        {
          output = ledstate | 0x08;
        }
        
      break;

     default:break;
    }
    if(ISendStr(0x40,0x01,&output,1)) //写PCA9554输出寄存器
    {
      if(IRcvByte(0x40,data))         //读PCA9554输出寄存器
      {
        ledstate = *data;
      }
    }
  }
}


void Buzzer(uint8 operation)
{
  uint8 output = 0x00;
  uint8 *data = 0;
  if(ISendStr(0x40,0x03,&output,1))  //写配置
  {
        if (operation)
        {
          output = ledstate | 0x40;
        }
        else
        {
          output = ledstate & 0xbf;
        }
      if(ISendStr(0x40,0x01,&output,1))
     {
      if(IRcvByte(0x40,data))
      {
        ledstate = *data;
      }
    }
  }
}

uint8 Relay(uint8 cmd)
{
  uint8 output = 0x00;
  uint8 *data = 0;
  if(ISendStr(0x44,0x03,&output,1))  //写配置
  {
    output = cmd & 0x0f;
    if(ISendStr(0x44,0x01,&output,1))
    {
      if(IRcvByte(0x44,data))
      {
        return *data;
      }
    }
  }
  return 0;
}

uint8 SRelay(uint8 cmd)
{
  uint8 output = 0x00;
  uint8 *data = 0;
  if(ISendStr(0x48,0x03,&output,1))  //写配置
  {
    output = cmd & 0x0f;
    if(ISendStr(0x48,0x01,&output,1))
    {
      if(IRcvByte(0x48,data))
      {
        return *data;
      }
    }
  }
  return 0;
}

uint8 Key()
{
  uint8 input = 0xff;
  uint8 *data = 0;
  if(ISendStr(0x42,0x03,&input,1))  //写配置
  {   
    if(ISendByte(0x42,0x00))  //发送命令
    {
     if(IRcvByte(0x42,data))
     {
       return *data;
      }
    }
  }
  return 0;
}

uint8 write24L01(uint8 *data,uint8 addr,uint8 Len)
{
  if(ISendStr(0xa0,addr,data,Len))
  {
    return 1;
  }
  else
  {
     return 0;
  } 
}

uint8 read24L01byte(uint8 addr)
{
  uint8 data = 0;
  if(ISendByte(0xa0,addr))
  {
    Start_I2c();                /*启动总线*/
   SendByte(0xa1);           /*发送器件地址*/
   if(ack==0)
   {
     return(0);
   }
   else
   {
   data = RcvByte();               /*读取数据*/
   Ack_I2c(1);               /*发送非就答位*/
   Stop_I2c();                  /*结束总线*/ 
   return data;
   }
  }
  return 0;
}

uint8 VoltageOutput(uint16 Voltage,uint8 Port)
{
   IO_DIR_PORT_PIN(1, 2, IO_OUT);    //设置P1.0为输出
   P1_2 = 1;
   Start_I2c();               /*启动总线*/
   SendByte(0x98);            /*发送器件地址*/
   if(ack==0)return(0);
   SendByte(Port);               /*发送控制字节*/
   if(ack==0)return(0);
   SendByte(HI_UINT16(Voltage));               /*发送数据H*/
   if(ack==0)return(0);
   SendByte(LO_UINT16(Voltage));               /*发送数据L*/
   if(ack==0)return(0);
   Stop_I2c();                 /*结束总线*/ 
   P1_2 = 0;
   return(1);
}


/******************************************************************************
 * 函数名称：Relays
 *
 * 功能描述：通过IIC总线控制PCA9554的输出，进而控制相应的LED。
 *                    
 *
 * 参    数：relay     - 所控制的Relay
 *           operation - 开或关操作
 *
 * 返 回 值：无
 *           
 *
 * 注    意：PCA9554的地址为：0x48
 *****************************************************************************/ 
void Relays(uint8 relay,uint8 operation)
{
  uint8 output = 0x00;
  uint8 *data = 0;
  if(ISendStr(0x48,0x03,&output,1))  //配置PCA9554寄存器
  {
    switch(relay)
    {
      case 1:                        
        if (operation)
        {
          output = Relaystate & 0xfe;
        }
        else
        {
          output = Relaystate | 0x01;
        }
      break;
       case 2:                      
        if (operation)
        {
          output = Relaystate & 0xfd;
        }
        else
        {
          output = Relaystate | 0x02;
        }
      break;
       case 3:                     
        if (operation)
        {
          output = Relaystate & 0xfb;
        }
        else
        {
          output = Relaystate | 0x04;
        }
      break;
       case 4:                     
        if (operation)
        {
          output = Relaystate & 0xf7;
        }
        else
        {
          output = Relaystate | 0x08;
        }
      break;
       

     default:break;
    }
    if(ISendStr(0x48,0x01,&output,1)) //写PCA9554输出寄存器
    {
      if(IRcvByte(0x48,data))         //读PCA9554输出寄存器
      {
        Relaystate = *data;
      }
    }
  }
}

void RelayInit(void)
{
  uint8 output = 0x00;
  uint8 *data = 0;
  if(ISendStr(0x48,0x03,&output,1))  //写配置
  {
    output = 0xbf;
    if(ISendStr(0x48,0x01,&output,1))
    {
      if(IRcvByte(0x48,data))
      {
        Relaystate = *data;
      }
    }
  }
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



