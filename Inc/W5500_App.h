#ifndef  _W5500_APP_H_
#define  _W5500_APP_H_

/*********************************************************************
 * INCLUDES
 */
#include "stdint.h"

/*********************************************************************
 * TYPEDEFS
 */
/* 网络配置参数 */    		//8.26 地址对齐问题 debug
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
	uint16_t Sn_Port;				//Socket n端口号/源端口号
	
	//本机为TCP客户端模式时配置	
	uint16_t Sn_DPort;			//Socket n目的端口号 
	uint8_t  Sn_DIP[4];			//Socket n目的IP地址 
	
	//本机为UDP模式时配置
	uint16_t UDP_DPORT;			//Socket n目的端口号
	uint8_t  UDP_DIPR[4];		//Socket n目的IP地址
	uint8_t  _UDP_DIPR_[2]; //字节对齐
}SOCKETnParam_t;

/*********************************************************************
 * Macros
 */
#define TRUE							0x00
#define FALSE							0xff

// 端口状态
#define Sn_OPEN						0x00	//!< 端口已打开
#define Sn_LISTEN					0x01	//!< 端口正在监听
#define Sn_CLOSE					0x02	//!< 端口已关闭
#define TCP_PROCESS				0x03	//!< TCP端口正在处理数据
#define TCP_COMPLETE			0x04	//!< TCP端口数据处理完毕

/*******************************************************************************
 * FUNCTIONS
 */
void W5500_Init(void);
void W5500_Config(void);
void W5500_RST(void);
void W5500_Load_Net_Parameters(void);
uint8_t Detect_Gateway(void);
void W5500_Socket_Init(uint8_t sn);

uint8_t TCPServer_Service(uint8_t sn);
uint8_t UDP_Service(uint8_t sn);


#endif   // _w5500_app_H_
