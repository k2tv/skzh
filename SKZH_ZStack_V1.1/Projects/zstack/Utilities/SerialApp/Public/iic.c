/**********************************************************************************************************
* �� �� ����iic.C
* ��    �ܣ��˳�����I2C����ƽ̨������ʽ�����ƽ̨���ĵײ��C�ӳ���,�緢�����ݼ���������,Ӧ��λ����
* Ӳ�����ӣ�����CC2530��I/O��ģ��IIC��SCL��SDA
*
*           P1.0 ------ SCL
*           P1.1 ------ SDA
*           
* ��    ����V1.0
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

static uint8 ack;	         /*Ӧ���־λ*/

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
void FLASHLED4(uint8 FLASHnum);   //LED4��˸
void FLASHLED5(uint8 FLASHnum);   //LED5��˸
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
void QWait()     //1us����ʱ
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
                     �����ߺ���               
����ԭ��: void  Start_I2c();  

��    ��: ����I2C����,������I2C��ʼ����.
  
********************************************************************/
static void Start_I2c()
{
  IO_DIR_PORT_PIN(1, 0, IO_OUT);    //����P1.0Ϊ���
  IO_DIR_PORT_PIN(1, 1, IO_OUT);    //����P1.1Ϊ���
  
  SDA=1;   /*������ʼ�����������ź�*/
  asm("NOP");
  SCL=1;
  QWait();    /*��ʼ��������ʱ�����4.7us,��ʱ*/
  QWait();
  QWait();
  QWait();
  QWait();    
  SDA=0;   /*������ʼ�ź�*/
  QWait();    /* ��ʼ��������ʱ�����4��s*/
  QWait();
  QWait();
  QWait();
  QWait();       
  SCL=0;   /*ǯסI2C���ߣ�׼�����ͻ�������� */
  asm("NOP");
  asm("NOP");
}

/*******************************************************************
                      �������ߺ���               
����ԭ��: void  Stop_I2c();  

��    ��: ����I2C����,������I2C��������.
  
********************************************************************/
static void Stop_I2c()
{
  IO_DIR_PORT_PIN(1, 0, IO_OUT);    //����P1.0Ϊ���
  IO_DIR_PORT_PIN(1, 1, IO_OUT);    //����P1.1Ϊ���
  SDA=0;  /*���ͽ��������������ź�*/
  asm("NOP");   /*���ͽ���������ʱ���ź�*/
  SCL=1;  /*������������ʱ�����4��s*/
  QWait();
  QWait();
  QWait();
  QWait();
  QWait();
  SDA=1;  /*����I2C���߽����ź�*/
  QWait();
  QWait();
  QWait();
  QWait();
}


/*******************************************************************
                 �ֽ����ݴ��ͺ���               
����ԭ��: void  SendByte(uchar c);

��    ��: ������c���ͳ�ȥ,�����ǵ�ַ,Ҳ����������,�����ȴ�Ӧ��,����
          ��״̬λ���в���.(��Ӧ����Ӧ��ʹack=0 ��)     
          ��������������ack=1; ack=0��ʾ��������Ӧ����𻵡�
********************************************************************/
static void  SendByte(uint8 c)
{
 uint8 BitCnt;
 IO_DIR_PORT_PIN(1, 0, IO_OUT);    //����P1.0Ϊ���
 IO_DIR_PORT_PIN(1, 1, IO_OUT);    //����P1.1Ϊ���
 for(BitCnt=0;BitCnt<8;BitCnt++)  /*Ҫ���͵����ݳ���Ϊ8λ*/
    {
     if((c<<BitCnt)&0x80)SDA=1;   /*�жϷ���λ*/
       else  SDA=0;                
      asm("NOP");
      asm("NOP");
      asm("NOP");
     SCL=1;               /*��ʱ����Ϊ�ߣ�֪ͨ��������ʼ��������λ*/
      QWait(); 
      QWait();               /*��֤ʱ�Ӹߵ�ƽ���ڴ���4��s*/
      QWait();
      QWait();
      QWait();         
     SCL=0; 
    }    
    QWait();
    QWait();
    QWait();
    SDA=1;               /*8λ��������ͷ������ߣ�׼������Ӧ��λ*/
    asm("NOP");
    IO_DIR_PORT_PIN(1, 1, IO_IN);  
    SCL=1;
    QWait();          
    //QWait();
    //QWait();
    //QWait();
    if(SDA==1)ack=0;     
    else ack=1;        /*�ж��Ƿ���յ�Ӧ���ź�*/
    SCL=0;   
    QWait();
    QWait();
    IO_DIR_PORT_PIN(1, 1, IO_OUT);
}

/*******************************************************************
                 �ֽ����ݴ��ͺ���               
����ԭ��: uchar  RcvByte();

��    ��: �������մ���������������,���ж����ߴ���(����Ӧ���ź�)��
          ���������Ӧ������  
********************************************************************/	
static uint8  RcvByte()
{
  uint8 retc;
  uint8 BitCnt;
  IO_DIR_PORT_PIN(1, 0, IO_OUT);    //����P1.0Ϊ���
 
  retc=0; 
 // SDA=1;             /*��������Ϊ���뷽ʽ*/
  IO_DIR_PORT_PIN(1, 1, IO_IN);
  for(BitCnt=0;BitCnt<8;BitCnt++)
      {
        asm("NOP");          
        SCL=0;       /*��ʱ����Ϊ�ͣ�׼����������λ*/
        QWait();
        QWait();         /*ʱ�ӵ͵�ƽ���ڴ���4.7��s*/
        QWait();
        QWait();
        QWait();
        SCL=1;       /*��ʱ����Ϊ��ʹ��������������Ч*/
        QWait();
        QWait();
        retc=retc<<1;
        if(SDA==1)retc=retc+1; /*������λ,���յ�����λ����retc�� */
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
                     Ӧ���Ӻ���
ԭ��:  void Ack_I2c(uint a);
 
����:����������Ӧ���ź�,(������Ӧ����Ӧ���ź�)
********************************************************************/
static void Ack_I2c(uint8 a)
{
  IO_DIR_PORT_PIN(1, 0, IO_OUT);    //����P1.0Ϊ���
  IO_DIR_PORT_PIN(1, 1, IO_OUT);    //����P1.1Ϊ���
  if(a==0)SDA=0;     /*�ڴ˷���Ӧ����Ӧ���ź� */
  else SDA=1;
  QWait();
  //QWait();      //change by wu 2011.3.1
  //QWait();      
  SCL=1;
  QWait();
  QWait();              /*ʱ�ӵ͵�ƽ���ڴ���4��s*/
  QWait();
  QWait();
  QWait();  
  SCL=0;                /*��ʱ���ߣ�ǯסI2C�����Ա��������*/
  QWait();
  //QWait();  
  IO_DIR_PORT_PIN(1, 1, IO_IN);// change by wu 2011.3.1
}

/*******************************************************************
                    �����ӵ�ַ���������ֽ����ݺ���               
����ԭ��: uint  ISendByte(uchar sla,ucahr c);

��    ��:  ���������ߵ����͵�ַ�����ݣ��������ߵ�ȫ����,��������ַsla.
           �������1��ʾ�����ɹ��������������

ע    �⣺ ʹ��ǰ�����ѽ������ߡ�
********************************************************************/
static uint8 ISendByte(uint8 sla,uint8 c)
{
   Start_I2c();               /*��������*/
   SendByte(sla);            /*����������ַ*/
     if(ack==0)return(0);
   SendByte(c);               /*��������*/
     if(ack==0)return(0);
  Stop_I2c();                 /*��������*/ 
  return(1);
}

/*******************************************************************
                    �����ӵ�ַ�������Ͷ��ֽ����ݺ���               
����ԭ��: uint  ISendStr(uchar sla,uchar suba,ucahr *s,uchar no);  

��    ��: ���������ߵ����͵�ַ���ӵ�ַ,���ݣ��������ߵ�ȫ����,������
          ��ַsla���ӵ�ַsuba������������sָ������ݣ�����no���ֽڡ�
          �������1��ʾ�����ɹ��������������

ע    �⣺ʹ��ǰ�����ѽ������ߡ�
********************************************************************/
static uint8 ISendStr(uint8 sla,uint8 suba,uint8 *s,uint8 no)
{
   uint8 i;

   Start_I2c();               /*��������*/
   SendByte(sla);            /*����������ַ*/
     if(ack==0)return(0);
   SendByte(suba);            /*���������ӵ�ַ*/
     if(ack==0)return(0);

   for(i=0;i<no;i++)
    {   
     SendByte(*s);               /*��������*/
       if(ack==0)return(0);
     s++;
    } 
 Stop_I2c();                 /*��������*/ 
  return(1);
}


/*******************************************************************
                    �����ӵ�ַ�������ֽ����ݺ���               
����ԭ��: uint  IRcvByte(uchar sla,ucahr *c);  

��    ��: ���������ߵ����͵�ַ�������ݣ��������ߵ�ȫ����,��������
          ַsla������ֵ��c. �������1��ʾ�����ɹ��������������

ע    �⣺ʹ��ǰ�����ѽ������ߡ�
********************************************************************/
uint8 IRcvByte(uint8 sla,uint8 *c)
{
   Start_I2c();                /*��������*/
   SendByte(sla+1);           /*����������ַ*/
 
   if(ack==0)return(0);
   *c=RcvByte();               /*��ȡ����*/
   Ack_I2c(1);               /*���ͷǾʹ�λ*/
   Stop_I2c();                  /*��������*/ 
   return(1);
}

/*******************************************************************
                    �����ӵ�ַ������ȡ���ֽ����ݺ���               
����ԭ��: uint  ISendStr(uchar sla,uchar suba,ucahr *s,uchar no); 

��    ��: ���������ߵ����͵�ַ���ӵ�ַ,�����ݣ��������ߵ�ȫ����,������
          ��ַsla���ӵ�ַsuba�����������ݷ���sָ��Ĵ洢������no���ֽڡ�
          �������1��ʾ�����ɹ��������������

ע    �⣺ʹ��ǰ�����ѽ������ߡ�
********************************************************************/
uint8 IRcvStr(uint8 sla,uint8 suba,uint8 *s,uint8 no)
{
   uint8 i;

   Start_I2c();               /*��������*/
   SendByte(sla);            /*����������ַ*/
   if(ack==0)return(0);
   SendByte(suba);            /*���������ӵ�ַ*/
   if(ack==0)return(0);
   Start_I2c();
   SendByte(sla+1);
   if(ack==0)return(0);
   for(i=0;i<no-1;i++)
    {   
      *s=RcvByte();               /*��������*/
      Ack_I2c(0);                /*���;ʹ�λ*/  
      s++;
    } 
   *s=RcvByte();
   Ack_I2c(1);                 /*���ͷ�Ӧλ*/
   Stop_I2c();                    /*��������*/ 
   return(1);
}

//static uint8 IRcvStr_test(uint8 sla,uint8 *s,uint8 no)
//{
   //uint8 i;

//   Start_I2c();               /*��������*/
//   SendByte(sla);            /*����������ַ*/
//   if(ack==0)return(0);
   //SendByte(suba);            /*���������ӵ�ַ*/
  // if(ack==0)return(0);
   //Start_I2c();
  // SendByte(sla+1);
  // if(ack==0)return(0);
//   for(i=0;i<no-1;i++)
//    {   
//      *s=RcvByte();               /*��������*/
 //     Ack_I2c(0);                /*���;ʹ�λ*/  
 //     s++;
//    } 
//   while(no > 0) 
//   {
 //   *s++ = RcvByte();
 //    if(no > 1)  Ack_I2c(0);                /*���;ʹ�λ*/ 
 //    else Ack_I2c(1);                 /*���ͷ�Ӧλ*/
 //    no--;
 //  }
//   *s=RcvByte();
//   Ack_I2c(1);                 /*���ͷ�Ӧλ*/
//   Stop_I2c();                    /*��������*/ 
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
  if(ISendStr(0x40,0x03,&output,1))  //д����
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
 * �������ƣ�FLASHLED
 *
 * ����������ͨ��IIC���߿���PCA9554����������������ǰ�����ж�������PCA9554���
 *           �Ĵ�����ֵ��Ȼ��������Ƶ�λ����ȡ�����ơ�
 *                    
 *
 * ��    ����led - ��ӦPCA9554������˿�
 *           
 *
 * �� �� ֵ����
 *           
 *
 * ע    �⣺PCA9554�ĵ�ַΪ��0x40
 *****************************************************************************/ 

void FLASHLED(uint8 led)   
{
  uint8 output = 0x00;
  uint8 *data = 0;
  if(ISendStr(0x40,0x03,&output,1))    //д����
  {
    switch(led)
    {
     case 3:     
     output = ledstate & 0x01;  //�ж�������λ�ĵ�ǰ״̬ 
      if (output)                      //�����ǰ��λ״̬Ϊ1����ı����Ϊ0
     {
       output = ledstate & 0xfe;
     }
      else                             //�����ǰ״̬Ϊ0����ı����Ϊ1
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
     if(ISendStr(0x40,0x01,&output,1))    //д����Ĵ���
    {
      if(IRcvByte(0x40,data))             //������Ĵ���
      {
        ledstate = *data;          //��������Ĵ�����ǰ״̬��
      }
    }
  }
}
void FLASHLED4(uint8 FLASHnum)   //LED4��˸
{
  uint8 output = 0x00;
  uint8 *data = 0;
  if(ISendStr(0x40,0x03,&output,1))  //д����
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

void FLASHLED5(uint8 FLASHnum)   //LED5��˸
{
  uint8 output = 0x00;
  uint8 *data = 0;
  if(ISendStr(0x40,0x03,&output,1))  //д����
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
 * �������ƣ�LED
 *
 * ����������ͨ��IIC���߿���PCA9554�����������������Ӧ��LED��
 *                    
 *
 * ��    ����LED - �����Ƶ�LED
 *           operation - ����ز���
 *
 * �� �� ֵ����
 *           
 *
 * ע    �⣺PCA9554�ĵ�ַΪ��0x40
 *****************************************************************************/ 
void LED(uint8 led,uint8 operation)
{
  uint8 output = 0x00;
  uint8 *data = 0;
  if(ISendStr(0x40,0x03,&output,1))  //����PCA9554�Ĵ���
  {
    switch(led)
    {
      case 1:                        //LED0����
         if (operation)
        {
          output = ledstate & 0xdf;
        }
        else
        {
          output = ledstate | 0x20;
        }
        
      break;
       case 2:                      //LED1����
        if (operation)
        {
          output = ledstate & 0xef;
        }
        else
        {
          output = ledstate | 0x10;
        }
       
      break;
       case 3:                     //LED2����
        if (operation)
        {
          output = ledstate & 0xfe;
        }
        else
        {
          output = ledstate | 0x01;
        }
        
      break;
       case 4:                     //LED3����
        if (operation)
        {
          output = ledstate & 0xfd;
        }
        else
        {
          output = ledstate | 0x02;
        }
        
      break;
       case 5:                    //LED4����
        if (operation)
        {
          output = ledstate & 0xfb;
        }
        else
        {
          output = ledstate | 0x04;
        }
        
      break;
       case 6:                   //LED5����
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
    if(ISendStr(0x40,0x01,&output,1)) //дPCA9554����Ĵ���
    {
      if(IRcvByte(0x40,data))         //��PCA9554����Ĵ���
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
  if(ISendStr(0x40,0x03,&output,1))  //д����
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
  if(ISendStr(0x44,0x03,&output,1))  //д����
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
  if(ISendStr(0x48,0x03,&output,1))  //д����
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
  if(ISendStr(0x42,0x03,&input,1))  //д����
  {   
    if(ISendByte(0x42,0x00))  //��������
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
    Start_I2c();                /*��������*/
   SendByte(0xa1);           /*����������ַ*/
   if(ack==0)
   {
     return(0);
   }
   else
   {
   data = RcvByte();               /*��ȡ����*/
   Ack_I2c(1);               /*���ͷǾʹ�λ*/
   Stop_I2c();                  /*��������*/ 
   return data;
   }
  }
  return 0;
}

uint8 VoltageOutput(uint16 Voltage,uint8 Port)
{
   IO_DIR_PORT_PIN(1, 2, IO_OUT);    //����P1.0Ϊ���
   P1_2 = 1;
   Start_I2c();               /*��������*/
   SendByte(0x98);            /*����������ַ*/
   if(ack==0)return(0);
   SendByte(Port);               /*���Ϳ����ֽ�*/
   if(ack==0)return(0);
   SendByte(HI_UINT16(Voltage));               /*��������H*/
   if(ack==0)return(0);
   SendByte(LO_UINT16(Voltage));               /*��������L*/
   if(ack==0)return(0);
   Stop_I2c();                 /*��������*/ 
   P1_2 = 0;
   return(1);
}


/******************************************************************************
 * �������ƣ�Relays
 *
 * ����������ͨ��IIC���߿���PCA9554�����������������Ӧ��LED��
 *                    
 *
 * ��    ����relay     - �����Ƶ�Relay
 *           operation - ����ز���
 *
 * �� �� ֵ����
 *           
 *
 * ע    �⣺PCA9554�ĵ�ַΪ��0x48
 *****************************************************************************/ 
void Relays(uint8 relay,uint8 operation)
{
  uint8 output = 0x00;
  uint8 *data = 0;
  if(ISendStr(0x48,0x03,&output,1))  //����PCA9554�Ĵ���
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
    if(ISendStr(0x48,0x01,&output,1)) //дPCA9554����Ĵ���
    {
      if(IRcvByte(0x48,data))         //��PCA9554����Ĵ���
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
  if(ISendStr(0x48,0x03,&output,1))  //д����
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
 * �������ƣ�s_write_byte
 *
 * ������������SHT10дһ���ֽ�
 *
 * ��    ����value -- ��д����ֽ�ֵ
 *
 * �� �� ֵ��error -- �����Ƿ�ɹ�
 **************************************************************************************************/
char s_write_byte(unsigned char value)
{ 
  unsigned char i,error=0;  
  IO_DIR_PORT_PIN(1, 0, IO_OUT);      //ʱ�Ӻ�����IO����Ϊ���
  IO_DIR_PORT_PIN(1, 1, IO_OUT);
  for (i=0x80;i>0;i/=2)               //��һ���ֽڵ�8λ��һ���        
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
  IO_DIR_PORT_PIN(1, 1, IO_IN);      //������������Ϊ���룬��׼������SHT10��ACK
  SCL = 1;  asm("NOP");                          
  error = SDA; 
  QWait();QWait();QWait();
  IO_DIR_PORT_PIN(1, 1, IO_OUT);     //�������߻ָ�Ϊ���״̬
  SDA = 1; 
  SCL = 0;        
  
  return error;                                   
}

/**************************************************************************************************
 * �������ƣ�s_read_byte
 *
 * ������������SHT10��ȡһ���ֽ�
 *
 * ��    ����ack -- ��ȡ���ݺ���SHT10����ACK
 *
 * �� �� ֵ��val -- ��ȡ���ֽ�ֵ
 **************************************************************************************************/
char s_read_byte(unsigned char ack)
{
  IO_DIR_PORT_PIN(1, 0, IO_OUT);     //ʱ�Ӻ�����IO����Ϊ���
  IO_DIR_PORT_PIN(1, 1, IO_OUT);
  unsigned char i,val=0;
  SDA= 1;
  IO_DIR_PORT_PIN(1, 1, IO_IN);      //������������Ϊ���룬��׼������SHT10������
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
  IO_DIR_PORT_PIN(1, 1, IO_OUT);     //�������߻ָ�Ϊ���״̬
  SDA = !ack;
  SCL = 1;
  QWait();QWait();QWait();QWait();QWait();
  SCL = 0;
  SDA = 1;
  
  return val;                       //���ض�ȡ��ֵ
}



