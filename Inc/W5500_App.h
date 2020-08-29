#ifndef  _W5500_App_H_
#define  _W5500_App_H_

/*********************************************************************
 * INCLUDES
 */
#include "stdint.h"

/*********************************************************************
 * TYPEDEFS
 */
/* 网络配置参数 */    //8.26 地址对齐问题 debug看看
typedef struct
{
	uint8_t Gateway_IP[4];	//网关IP地址 
	uint8_t Sub_Mask[4];		//子网掩码 
	uint8_t IP_Addr[4];			//源IP地址，即本机IP地址
	uint8_t Phy_Addr[6];		//源MAC地址，即本机IP地址 	
}NETWORKParam_t;

/* Socket n配置参数 */
typedef struct
{
	uint16_t Sn_Port;				//Socket n端口号
	
	//TCP模式	
	uint16_t Sn_DPort;			//Socket n目的端口号 
	uint8_t  Sn_DIP[4];			//Socket n目的IP地址 
	
	//UDP模式
	uint16_t UDP_DPORT;			//Socket n目的端口号
	uint8_t  UDP_DIPR[4];		//Socket n目的IP地址
	uint8_t  _UDP_DIPR_[2]; //字节对齐
}SOCKETnParam_t;

/* Socket n状态参数（中断用）*/
typedef struct
{
	uint8_t Sn_Mode;				/*Socket n工作模式: 		- TCP服务器 						0 
																									- TCP客户端 						1 
																									- UDP										2 */
	uint8_t Sn_State;				/*Socket n运行状态: 		- Socket n完成初始化 		1
																									- Socket n完成连接      2 */ 
	uint8_t Sn_Data;				/*Socket n收发数据状态: - Socket n接收到数据		1
																									- Socket n发送数据完成	2 */
}SOCKETnState_t;

/*********************************************************************
 * Macros
 */
#define TRUE						0xff
#define FALSE						0x00

#define Interrupt				0x0a
#define Polling					0x0b

#ifndef _W5500_MODE_
#define _W5500_MODE_    Polling 
#endif

/* Socket n工作模式 */
#define TCP_SERVER			0x00				//TCP服务器模式
#define TCP_CLIENT			0x01				//TCP客户端模式 
#define UDP_MODE				0x02				//UDP(广播)模式 

#if(_W5500_MODE_ == Interrupt)
/* Socket n中断寄存器描述 */
#define IR_SEND_OK			0x10				//send命令完成中断
#define IR_TIMEOUT			0x08        //超时中断
#define IR_RECV					0x04        //接收到数据中断
#define IR_DISCON				0x02        //接收到FIN或FIN/ACK中断
#define IR_CON					0x01        //连接建立中断

/* Socket n运行状态 */
#define S_INIT					0x01				//Socket n完成初始化 
#define S_CONN					0x02				//Socket n完成连接,可以正常传输数据 

/* Socket n收发数据状态 */
#define S_RECEIVE	 			0x01				//Socket n接收到一个数据包 
#define S_TRANSMITOK 		0x02				//Socket n完成一个数据包的发送 
#endif 
 /*******************************************************************************
 * EXTERNAL VARIABLES
 */

extern uint8_t Rx_Buffer[2048];			//接收数据缓冲区 
extern uint8_t Tx_Buffer[2048];			//发送数据缓冲区

extern NETWORKParam_t *Pnet_param;  //网络参数配置
extern SOCKETnParam_t	*Ps0_param;   //Socket 0参数配置
extern SOCKETnParam_t	*Ps1_param;   //Socket 1参数配置

#if(_W5500_MODE_ == Interrupt )
extern uint8_t W5500_Interrupt;			//W5500中断标志 0-无，1-有中断
extern SOCKETnState_t *Ps0_state;   //Socket 0状态
extern SOCKETnState_t *Ps1_state;   //Socket 1状态
#endif 

/*******************************************************************************
 * FUNCTIONS
 */
void W5500_Init(void);
void W5500_Config(void);
void W5500_RST(void);
void W5500_Load_Net_Parameters(void);
uint8_t Detect_Gateway(void);
void W5500_Socket_Init(uint8_t sn);

void DO_TCP_Server(uint8_t sn);
void DO_UDP(uint8_t sn);

#if(_W5500_MODE_ == Interrupt )
//中断方式
void W5500_Socket_State(uint8_t sn);
void W5500_Interrupt_Process(void);
#endif

#endif   // _W5500_App_H_
