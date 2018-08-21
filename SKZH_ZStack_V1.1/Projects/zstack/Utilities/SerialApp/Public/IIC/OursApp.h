/***************************************************************************************************************
* 文 件 名：OursApp.h
×
* 功    能：主要包含与OURS公司PC端软件通信协调相关的定义和实验设备ID等。        
*
* 版    本：V1.0
* 作    者：WU XIANHAI
* 日    期：2011.3.22
* 奥尔斯电子主页：www.ourselec.com
******************************************************************************************************************/

#ifndef OURSDEF_H
#define OURSDEF_H

#ifdef __cplusplus
extern "C"
{
#endif
  
#define MY_PAN_ID     2530          //本组实验设备的PANID
#define MY_WSN_TYPE   1          //设备类型（由数据库得到，用于组成会话ID）


//**************************************************************************
/* 协调器和路由器与应用程序消息代码*/
//一般消息代码
#define FETCH_NODE_INFO                0x01     //获取节点信息
#define NODE_SYSTEMRESET               0x02     //节点重启
#define FETCH_NODE_SENSOR_CAL          0x03     //获取传感器校准参数

//一般群发消息代码表
#define FETCH_GROUPNODE_INFO           0x11     //获取节点信息
#define GROUPNODE_SYSTEMRESET          0x12     //节点重启
#define TURNOFF_APP                    0x13     //关闭所有节点应用功能

//其他消息代码
#define SENDUP_NODE_INFO               0x21     //节点信息(上传)
#define SENDUP_NODE_SBOARD_DATA        0x22     //上传节点扩展传感器资源数据
#define SENDDOMN_NODE_SBOARD_DATA      0x23     //下传节点扩展板资源数据
#define SENDUP_NODE_BBOARD_DATA        0x24     //上传节点底板资源数据
#define SENDDOMN_NODE_BBOARD_DATA      0x25     //下传节点底板资源数据
#define SENDUP_NODE_PADDR_ADDR         0x26     //节点父子关系消息

#define SENDDOMN_GROUPNODE_LIST        0x27     //节点分组消息
  
#define SET_NODE_PANID                 0x28     //修改节点PANID

#define SENSOR_DATA_CAL                0x29     //传感器数据校准

//协调器接收路由节点消息状态
#define UNALLOW_COOR_REV               0x00     //不允许接收
#define ALLOW_COOR_REV                 0x01     //允许接收

//扩展传感器板被控对象  
#define APP_ADC                        0x01     //使用AD控制
#define APP_IIC                        0x02     //使用IIC控制  
#define APP_IIC_CPU                    0x03     //使用IIC加CPU控制
#define APP_UART                       0x04     //使用串口控制


//**************************************************************************

//***************************************************************************
/* 协调器与中间服务层之间的通信消息代码 */
#define HEARTBEAT_DETECT               0x01     //心跳检测
#define FIRST_APP_LINK                 0x02     //第一个上层应用连接服务
#define LAST_APP_ULINK                 0x03     //最后一个上层应用与服务断开

#define EQU_ID_CODE                    0x11     //设备识别消息
#define EQU_NUMBER_CODE                0x12     //设备序号消息

#define SEND_SEV                       0x01     //向SEV发送设备识别消息
#define REV_SEV_MSG                    0x02     //等待SEV发送设备序号消息
#define SEND_HEARTBEAT                 0x03     //发送心跳

//***************************************************************************

//***************************************************************************
/* 协调器或路由器与应用层之间的通信包 */
//节点一般消息
typedef struct
{
  uint8 Hdr;                       //头    
  uint8 Len;                       //长度
  uint16 TransportID;              //会话ID
  uint8 MSGCode;                   //消息代码
  uint16 NodeAddr;                 //节点地址
  uint8 Checksum;                  //校验和
}PCGeneralMSGPacket_t;

//节点一般应答消息
typedef struct
{
  uint8 Hdr;                       //头
  uint8 Len;                       //长度
  uint16 TransportID;              //会话ID
  uint8 MSGCode;                   //消息代码
  uint16 NodeAddr;                 //节点地址
  uint8 ReplyCode;                 //应答代码
  uint8 Checksum;                  //校验和
}PCMSGReplyPacket_t;

//一般群（组）发消息
typedef struct
{
  uint8 Hdr;                       //头    
  uint8 Len;                       //长度
  uint16 TransportID;              //会话ID
  uint8 MSGCode;                   //消息代码
  uint16 GroupID;                  //组号
  uint8 Checksum;                  //校验和
}PCGroupMSGPacket_t;

//节点信息(上传)
typedef struct
{
  uint8 Hdr;                       //头
  uint8 Len;                       //长度
  uint16 TransportID;              //会话ID
  uint8 MSGCode;                   //消息代码
  uint16 NodeAddr;                 //节点地址
  uint8 IEEEAddr[8];               //IEEE地址
  int8 NodeRSSI;                   //信号强度
  uint8 NodeLQI;                   //链路质量
  uint16 Panid;                    //网络ID
  uint8 TemType;                   //模板类型   1为智能板 2为普通电池板
  uint16 SBoardType;               //传感器板类型
  uint8 Checksum;                  //校验和
}PCNodeMSGPacket_t;

//节点父子关系消息
typedef struct
{
  uint8 Hdr;                       //头
  uint8 Len;                       //长度
  uint16 TransportID;              //会话ID
  uint8 MSGCode;                   //消息代码
  uint16 NodeAddr;                 //节点地址
  uint16 NodePAddr;                //父节点地址
  uint8 Checksum;                  //校验和
}PCNodeAddrPacket_t;

//上传扩展模块资源数据消息
typedef struct
{
  uint8 Hdr;                       //头
  uint8 Len;                       //长度
  uint16 TransportID;              //会话ID
  uint8 MSGCode;                   //消息代码
  uint16 NodeAddr;                 //节点地址
  uint16 ModeID;                   //模块代码
  uint16 *data;                     //数据
  uint8 Checksum;                  //校验和
}SendUpSBoardDataPacket_t;

//上传扩展模块资源数据消息2（数据位8位的）
typedef struct
{
  uint8 Hdr;                       //头
  uint8 Len;                       //长度
  uint16 TransportID;              //会话ID
  uint8 MSGCode;                   //消息代码
  uint16 NodeAddr;                 //节点地址
  uint16 ModeID;                   //模块代码
  uint8 *data;                     //数据
  uint8 Checksum;                  //校验和
}SendUpSBoardDataPacket2_t;

//下传扩展模块数据消息
typedef struct
{
  uint8 Hdr;                       //头
  uint8 Len;                       //长度
  uint16 TransportID;              //会话ID
  uint8 MSGCode;                   //消息代码
  uint16 NodeAddr;                 //节点地址
  uint16 ModeID;                   //模块代码
  uint8 *data;                     //数据
  uint8 Checksum;                  //校验和
}SendDownSBoardDataPacket_t;

//重新设置节点PANID
typedef struct
{
  uint8 Hdr;                       //头
  uint8 Len;                       //长度
  uint16 TransportID;              //会话ID
  uint8 MSGCode;                   //消息代码
  uint16 NodeAddr;                 //节点地址
  uint16 SETPANID;                  //模块代码
  uint8 Checksum;                  //校验和
}SetPANIDPacket_t;

typedef struct
{
  uint8 Hdr;                       //头
  uint8 Len;                       //长度
  uint16 TransportID;              //会话ID
  uint8 MSGCode;                   //消息代码
  uint16 NodeAddr;                 //节点地址
  uint16 ModeID;                   //模块代码
  uint16 Maxdata;                  //模块代码
  uint16 Zerodata;                 //零值
  uint16 Mindata;                  //最小值
  uint8 Checksum;                  //校验和
}SensorCalPacket_t;

//ADC控制资源数据段
typedef struct
{
  uint8 ChComb;                    //通道组合
  uint8 SampleRate;                //采样精度
  uint16 SamplingSpeed;            //采样速率
  uint16 SampleMode;               //采样模式
  uint16 SampleGap;                //采集间隔
  uint16 SampleCount;              //采样点数
  uint8 PacketLen;                 //数据包长度
}ADControlData_t;

//DA控制资源数据段
typedef struct
{
  uint8 ChComb;                    //通道组合
  uint16 dataA;                    //通道A数据
  uint16 dataB;                    //通道B数据
  uint16 dataC;                    //通道C数据
  uint16 dataD;                    //通道D数据
}DAControlData_t;

//串口控制资源数据段
typedef struct
{
  uint8 UARTmode;                   //模式
  uint8 UARTid;                     //串口号
  uint8 UART_BR;                    //波特率
  uint8 CTR;                        //奇偶校验
  uint8 databit;                    //数据位
  uint8 stopbit;                    //停止位
  uint8 flowctr;                    //数据流控制
}UARTontrolData_t;

//串口资源数据段
typedef struct
{
  uint8 UARTmode;                   //模式
  uint8 UARTid;                     //串口号
  uint8 DataLen;                    //数据长度
  uint8 *data;                      //数据
}UARTData_t;

//上传母板资源数据消息
typedef struct
{
  uint8 Hdr;                       //头
  uint8 Len;                       //长度
  uint16 TransportID;              //会话ID
  uint8 MSGCode;                   //消息代码
  uint16 NodeAddr;                 //节点地址
  uint8 TemType;                   //模板类型
  uint16 ResID;                    //资源代码
  uint8 *data;                     //数据
  uint8 Checksum;                  //校验和
}SendUpTemBoardDataPacket_t;

//下传母板资源数据消息
typedef struct
{
  uint8 Hdr;                       //头
  uint8 Len;                       //长度
  uint16 TransportID;              //会话ID
  uint8 MSGCode;                   //消息代码
  uint16 NodeAddr;                 //节点地址
  uint8 TemType;                   //模板类型
  uint16 ResID;                   //资源代码
  uint8 *data;                     //数据
  uint8 Checksum;                  //校验和
}SendDownTemBoardDataPacket_t;

//节点分组消息
typedef struct
{
  uint8 Hdr;                       //头
  uint8 Len;                       //长度
  uint16 TransportID;              //会话ID
  uint8 MSGCode;                   //消息代码
  uint8 GroupID;                   //组号   0x00 - 0xfe 
  uint16 *NodeAddr;                //该组内个节点地址
  uint8 Checksum;                  //校验和
}GroupingMSGPacket_t;

//***************************************************************************


//***************************************************************************
/* 协调器与服务层之间的通信包 */
//设备识别
typedef struct
{
  uint8 Hdr;                       //头
  uint8 Len;                       //长度
  uint16 TransportID;              //会话ID
  uint8 MSGCode;                   //消息代码
  uint8 AppType;                   //应用类型
  uint8 EQUType;                   //设备类型
  uint8 Heartbeat_Timeout;         //心跳检测超时
  uint8 Heartbeat_Period;          //服务发送心跳间隔
  uint8 Checksum;                  //校验和
}SEVLinkCoordPacket_t;

//一般消息
typedef struct
{
  uint8 Hdr;                       //头    
  uint8 Len;                       //长度
  uint16 TransportID;              //会话ID
  uint8 MSGCode;                   //消息代码
  uint8 Checksum;                  //校验和
}SEVGeneralMSGPacket_t;

//一般应答消息
typedef struct
{
  uint8 Hdr;                       //头    
  uint8 Len;                       //长度
  uint16 TransportID;              //会话ID
  uint8 MSGCode;                   //消息代码
  uint8 ReplyCode;                 //应答代码
  uint8 Checksum;                  //校验和
}SEVMSGReplyPacket_t;

//设备序号消息
typedef struct
{
  uint8 Hdr;                       //头    
  uint8 Len;                       //长度
  uint16 TransportID;              //会话ID
  uint8 MSGCode;                   //消息代码
  uint8 NumberCode;                //设备序号
  uint8 Checksum;                  //校验和
}SEVEQUNumberPacket_t;


//**************************************************************************

//***************************************************************************
//资源代码定义
//AD方式
#define resCode_Voltage_Single		((0x10<<8)+0x00)
#define resCode_Voltage_Dual		((0x10<<8)+0x01)
#define resCode_Current_Single		((0x10<<8)+0x10)
#define resCode_Current_Dual		((0x10<<8)+0x11)
#define resCode_MinCurrent_Single	((0x10<<8)+0x18)
#define resCode_MinCurrent_Dual		((0x10<<8)+0x19)
#define resCode_Light_Single		((0x10<<8)+0x20)
#define resCode_Pressure		((0x10<<8)+0x30)
#define resCode_Alcohol			((0x10<<8)+0x31)
#define resCode_Pressure_Alcohol	((0x10<<8)+0x32)
//IIC接口方式
#define resCode_Temp_Humidity		((0x20<<8)+0x00)
#define resCode_Temp_Humidity_Light	((0x20<<8)+0x01)
#define resCode_Voltage_Output		((0x20<<8)+0x10)
#define resCode_Relay_GPIN		((0x20<<8)+0x20)
//IIC+外部CPU实现
#define resCode_Photoelectric		((0x21<<8)+0x00)
#define resCode_IR_Output		((0x21<<8)+0x01)
#define resCode_IR_Photoelectric	((0x21<<8)+0x03)
#define resCode_Ultrasonic		((0x21<<8)+0x10)
//串口接口
#define resCode_RS232_Wireless		((0x30<<8)+0x00)
#define resCode_RDID			((0x30<<8)+0x10)
#define resCode_RS232_Coordinator	((0x31<<8)+0x00)

//电源板，智能板
#define resCode_Relay			((0x01<<8)+0x00)
#define resCode_Led			((0x02<<8)+0x00)
#define resCode_Buzzer			((0x03<<8)+0x00)
//智能板 增加一个
#define resCode_AD			((0x04<<8)+0x00)

//***************************************************************************
#ifdef __cplusplus
}
#endif

#endif /* OURSDEF_H */





