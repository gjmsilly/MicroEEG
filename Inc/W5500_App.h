#ifndef  _W5500_App_H_
#define  _W5500_App_H_

#include "stdint.h"

/***************----- 网络参数变量定义 -----***************/
extern uint8_t Gateway_IP[4];	//网关IP地址 
extern uint8_t Sub_Mask[4];	//子网掩码 
extern uint8_t Phy_Addr[6];	//物理地址(MAC) 
extern uint8_t IP_Addr[4];	//本机IP地址 

extern uint16_t S0_Port;	//端口0的端口号(5000) 
extern uint8_t S0_DIP[4];		//端口0目的IP地址 
extern uint16_t S0_DPort;	//端口0目的端口号(6000) 

extern uint8_t UDP_DIPR[4];	//UDP(广播)模式,目的主机IP地址
extern uint16_t UDP_DPORT;	//UDP(广播)模式,目的主机端口号
extern uint16_t UDP_DPORT_[2];	//UDP(广播)模式,所有目的主机端口号
/***************----- 端口的运行模式 -----***************/
extern uint8_t S0_Mode;	//端口0的运行模式,0:TCP服务器模式,1:TCP客户端模式,2:UDP(广播)模式
#define TCP_SERVER		0x00	//TCP服务器模式
#define TCP_CLIENT		0x01	//TCP客户端模式 
#define UDP_MODE		0x02	//UDP(广播)模式 

/***************----- 端口的运行状态 -----***************/
extern uint8_t S0_State;	//端口0状态记录,1:端口完成初始化,2端口完成连接(可以正常传输数据) 
#define S_INIT			0x01	//端口完成初始化 
#define S_CONN			0x02	//端口完成连接,可以正常传输数据 

/***************----- 端口收发数据的状态 -----***************/
extern uint8_t S0_Data;		//端口0接收和发送数据的状态,1:端口接收到数据,2:端口发送数据完成 
#define S_RECEIVE		0x01		//端口接收到一个数据包 
#define S_TRANSMITOK	0x02		//端口发送一个数据包完成 

/***************----- 端口数据缓冲区 -----***************/
extern uint8_t Rx_Buffer[2048];	//端口接收数据缓冲区 
extern uint8_t Tx_Buffer[2048];	//端口发送数据缓冲区 

extern uint8_t W5500_Interrupt;	//W5500中断标志(0:无中断,1:有中断)

void W5500_Init(void);
void W5500_Initialization(void);
void W5500_RST(void);
void Load_Net_Parameters(void);
uint8_t Detect_Gateway(void);

void Socket_Init(uint8_t s);
uint8_t Socket_Connect(uint8_t s);
uint8_t Socket_Listen(uint8_t s);
uint8_t Socket_UDP(uint8_t s);
void W5500_Socket_Set(void);
void Process_Socket_Data(uint8_t s);

void W5500_Interrupt_Process(void);

#endif   // _W5500_App_H_
