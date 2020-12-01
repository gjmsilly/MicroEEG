/**
 * @file    w5500_service.c
 * @author  gjmsilly
 * @brief   W5500服务 - 以太网通讯服务
 * @version 1.0
 * @date    2020-08-29
 *
 * @copyright (c) 2020 gjmsilly
 *
 */

/*******************************************************************************
 * INCLUDES
 */
#include <string.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_gpio.h"
#include "main.h"
#include "w5500_service.h"
#include "w5500.h"
#include "wizchip_conf.h"
#include "socket.h"

/******************************************************************************
 * GLOBAL VARIABLES
 */  
uint8_t TCP_Rx_Buff[TCP_Rx_Buff_Size];				//TCP接收缓冲区 
uint8_t TCP_Tx_Buff[TCP_Tx_Buff_Size];				//TCP发送缓冲区
uint8_t UDP_DTx_Buff[UDPD_Tx_Buff_Size];			//UDP数据发送缓冲区
uint8_t UDP_TrgRx_Buff[UDP_TrgRx_Buff_Size];	//UDP事件发送缓冲区
uint8_t UDP_TrgTx_Buff[UDP_TrgTx_Buff_Size];	//UDP事件接收缓冲区
uint8_t *pUDP_DTx_Buff;												//UDP发送数据帧指针

 /*****************************************************************************
 * LOCAL VARIABLES
 */
NETWORKParam_t net_param,*Pnet_param;    		//网络参数配置
SOCKETnParam_t sn_param[3],*Psn_param;   		//Socket n参数配置(n=0,1)

static void W5500_Load_Net_Parameters(void);
static void W5500_RST(void);
static void W5500_Config(void);
static uint8_t Detect_Gateway(void);
static void W5500_Socket_Init(uint8_t sn);

/*******************************************************************************
 * FUNCTIONS
 */
 
/*******************************************************************************
* 函数名  : W5500_Init
*
* 描述    : W5500初始化
*
* 输入    : 无
*
* 返回值  : 无
*
* 说明    : 无
*******************************************************************************/
void W5500_Init(void)
{
	W5500_Load_Net_Parameters();	//装载网络参数	
	W5500_RST();									//硬件复位
	
	W5500_Config();								//初始化W5500通用寄存器区
	Detect_Gateway();							//检查网关服务器 
	W5500_Socket_Init(0);					//Socket 0配置 - TCP 
	W5500_Socket_Init(1);     		//Socket 1配置 - UDP发 
	W5500_Socket_Init(2);     		//Socket 2配置 - UDP收 
	
	pUDP_DTx_Buff = UDP_DTx_Buff; 	//!< UDP发送缓冲指针
}

/*******************************************************************************
* 函数名  : W5500_RST
*
* 描述    : W5500_RST引脚初始化配置
*
* 输入    : 无
*
* 返回值  : 无
*
* 说明    : 低电平有效，低电平至少保持500us以上
*******************************************************************************/
static void W5500_RST(void)
{
	LL_GPIO_ResetOutputPin(W5500_nRST_GPIO_Port,W5500_nRST_Pin);//复位引脚拉低
	HAL_Delay(50);
	LL_GPIO_SetOutputPin(W5500_nRST_GPIO_Port,W5500_nRST_Pin);//复位引脚拉高
	HAL_Delay(100);
	while((WIZCHIP_READ(PHYCFGR)&PHYCFGR_LNK_ON)==0);//等待以太网连接完成

} 

/*******************************************************************************
* 函数名  : W5500_Config
*
* 描述    : 初始化W5500通用寄存器
*
* 输入    : 无
*
* 返回值  : 无
*
* 说明    : 调用本函数前需先配置网络参数 - @ref W5500_Load_Net_Parameters
*******************************************************************************/
static void W5500_Config(void)
{

  setMR(MR_RST);	//软件复位W5500,置1有效,复位后自动清0
	HAL_Delay(10);	//延时10ms


	//设置网关(Gateway)的IP地址
	setGAR(Pnet_param->Gateway_IP);

	//设置子网掩码(MASK)，子网掩码用于子网运算
	setSUBR(Pnet_param->Sub_Mask);		
	
	//设置MAC(物理)地址,用于唯一标识网络设备的物理地址值
	//按照OUI的规定，前3个字节为厂商代码，后三个字节为产品序号
	//可自己定义MAC地址，注意第一个字节必须为偶数
	setSHAR(Pnet_param->Phy_Addr);		

	//设置源（本机）IP地址
	//注意：网关IP必须与本机IP属于同一个子网，否则本机将无法找到网关
	setSIPR(Pnet_param->IP_Addr);		

	//设置重试时间，默认2000(200ms) 
	//每一单位数值为100微秒,初始化时值设为4000,即400ms
	setRTR(2000);

	//设置重试次数，默认为8次 
	//如果重发的次数超过设定值,则产生超时中断(相关的端口中断寄存器中的Sn_IR 超时位(TIMEOUT)置“1”)
	setRCR(8);

	//中断屏蔽设置
	setIMR(0);//屏蔽IP冲突中断，UDP目的地址不能抵达中断，PPPoE，Magic Packet中断	
}

/*******************************************************************************
* 函数名  : W5500_Load_Net_Parameters
*
* 描述    : 配置网络参数
*
* 输入    : 无
*
* 返回值  : 无
*
* 说明    : * 在W5500 初始化前调用本函数以预先装载寄存器配置值 *
*						@Gateway_IP		网关IP地址 								32bit				192.168.1.1
*						@Sub_Mask			子网掩码 									32bit       255.255.255.0
*						@Phy_Addr			MAC地址										48bit       0c-29-ab-7c-00-01
*						@IP_Addr			源/本机IP地址							32bit       192.168.1.10						
*						@Sn_Port			源端口号
*														- Socket 0													7001 (default)
*														- Socket 1													7002 (default)
*														- Socket 2													7003 (default)
*						@Sn_Mode			工作模式											
*														- Socket 0                     			TCP 服务器
*														- Socket 1													UDP
*														- Socket 2													UDP
*						@Sn_DIP				目的主机IP地址						32bit			 （TCP client时配置） 
*						@UDP_DPORT		目的主机端口号												
*														- Socket 1													7002 (defualt)
*														- Socket 2													7002 (defualt)
*						@UDP_DIPR     目的主机IP地址						
*														- Socket 1              32bit				192.168.1.101
*														- Socket 2              32bit				192.168.1.101
*******************************************************************************/
static void W5500_Load_Net_Parameters(void)
{
	Pnet_param =&net_param;
	Psn_param = sn_param;
	uint8_t *Plen;
	
	//加载网关参数		
	Pnet_param->Gateway_IP[0] = 192;
	Pnet_param->Gateway_IP[1] = 168;
	Pnet_param->Gateway_IP[2] = 1;
	Pnet_param->Gateway_IP[3] = 1;
	
	//加载子网掩码
	Pnet_param->Sub_Mask[0] = 255;
	Pnet_param->Sub_Mask[1] = 255;
	Pnet_param->Sub_Mask[2] = 255;
	Pnet_param->Sub_Mask[3] = 0;
																 
	//加载MAC地址（default 锁死）
	Pnet_param->Phy_Addr[0] = 0x0c;
	Pnet_param->Phy_Addr[1] = 0x29;
	Pnet_param->Phy_Addr[2] = 0xab;
	Pnet_param->Phy_Addr[3] = 0x7c;
	Pnet_param->Phy_Addr[4] = 0x00;
	Pnet_param->Phy_Addr[5] = 0x01;

	//加载源/本机IP地址（default 锁死）
	Pnet_param->IP_Addr[0] = 192;
	Pnet_param->IP_Addr[1] = 168;
	Pnet_param->IP_Addr[2] = 1;
	Pnet_param->IP_Addr[3] = 10;	

	/* Socket 0 配置 */
	{				
		//加载Socket 0的端口号: 7001 （default 锁死）
		Psn_param->Sn_Port = 7001;
	}
	
	/* Socket 1 配置 */	
	{		
		//加载Socket 1的端口号: 7002 （default 锁死）
		(Psn_param+1)->Sn_Port = 7002;

		//UDP(广播)模式需配置目的主机IP地址
		(Psn_param+1)->UDP_DIPR[0] = 192;	
		(Psn_param+1)->UDP_DIPR[1] = 168;
		(Psn_param+1)->UDP_DIPR[2] = 1;
		(Psn_param+1)->UDP_DIPR[3] = 101;

		//UDP(广播)模式需配置目的主机端口号 7002（default 上位机可修改）
		(Psn_param+1)->UDP_DPORT = 7002;
	}
	
	/* Socket 2 配置 */	
	{		
		//加载Socket 2的端口号: 7003 （default 锁死）
		(Psn_param+2)->Sn_Port = 7003;

		//UDP(广播)模式需配置目的主机IP地址
		(Psn_param+2)->UDP_DIPR[0] = 192;	
		(Psn_param+2)->UDP_DIPR[1] = 168;
		(Psn_param+2)->UDP_DIPR[2] = 1;
		(Psn_param+2)->UDP_DIPR[3] = 101;

		//UDP(广播)模式需配置目的主机端口号 7003（default 上位机可修改）
		(Psn_param+2)->UDP_DPORT = 7003;
	}
	
}

/*******************************************************************************
* 函数名  : Detect_Gateway
*
* 描述    : 检查网关服务器 （使用Socket 0测试）
*
* 输入    : 无
*
* 返回值  : @TRUE/0xff 	- 正常
*						@FALSE/0x00 - 异常
*
* 说明    : 无
*******************************************************************************/
static uint8_t Detect_Gateway(void)
{
	uint8_t ip_adde[4];
	
	// 服务器模式下，构造目的IP地址
	ip_adde[0]=Pnet_param->IP_Addr[0]+1;
	ip_adde[1]=Pnet_param->IP_Addr[1]+1;
	ip_adde[2]=Pnet_param->IP_Addr[2]+1;
	ip_adde[3]=Pnet_param->IP_Addr[3]+1;

	//检查网关及获取网关的物理地址
	setSn_DIPR(0,ip_adde);//向目的IP地址寄存器写入构造的目的IP地址
	setSn_MR(0,Sn_MR_TCP);//设置Socket 0为TCP模式
	setSn_CR(0,Sn_CR_OPEN);//打开Socket 0
	HAL_Delay(5);//延时5ms 	
	
	//打开socket
	if(getSn_SR(0) != SOCK_INIT)  
	{
		setSn_CR(0,Sn_CR_CLOSE);//打开失败，则关闭Socket 0
		return FALSE;
	}

	//建立TCP连接
	setSn_CR(0,Sn_CR_CONNECT);//设置Socket0为Connect模式						
	
	do
	{
		uint8_t flag=0;
		
		flag=getSn_IR(0);//读取Socket0中断标志寄存器
		
		if(flag!=0)
			setSn_IR(0,flag); //重置中断寄存器
		
		HAL_Delay(5);//延时5ms 
		
		if((flag & Sn_IR_TIMEOUT) == Sn_IR_TIMEOUT) //超时
		{
			return FALSE;	
		}
		else if( WIZCHIP_READ(Sn_DHAR(0)) != 0xff)  
		{
			//Socket 0目的MAC地址寄存器非默认值，认为读取成功，已建立连接
			setSn_CR(0,Sn_CR_CLOSE); //关闭Socket 0
			return TRUE;							
		}
	}while(1);
}

/*******************************************************************************
* 函数名  : W5500_Socket_Init
*
* 描述    : 指定Socket n(0~7)初始化
*
* 输入    : @sn: Socket寄存器编号，e.g. Socket 1 即 sn=1
*
* 返回值  : 无
*
* 说明    : 无
*******************************************************************************/
static void W5500_Socket_Init(uint8_t sn)
{
	
	/* Socket 寄存器区设置 */

	//设置指定Socket
	switch(sn)
	{
		case 0:
			/* Socket 0 */	
			//设置分片长度
			setSn_MSSR(0, 0x5b4);//最大分片字节数=1460(0x5b4)
	
			// 屏蔽所有的Socket中断，采用轮询的方式 
			setSn_IMR(0,0xE0);	
		
			//设置工作模式 TCP server，打开Socket 0并绑定端口
			while(socket(0,Sn_MR_TCP,Psn_param->Sn_Port,0) != 0);
			//设置心跳包自动发送间隔，单位时间为5s	
			//setSn_KPALVTR(0,2);    	

		break;
		
		case 1:
			/* Socket 1 */			
			//设置分片长度
			setSn_MSSR(1, 0x5b4);//最大分片字节数=1460(0x5b4)			
		
			setSn_IMR(1,0);//屏蔽所有中断
		
		  //设置工作模式 UDP，打开Socket 1并绑定端口
			while(socket(1,Sn_MR_UDP,(Psn_param+1)->Sn_Port,0) != 1);	
			
		break;
		
		case 2:
			/* Socket 2 */			
			//设置分片长度
			setSn_MSSR(2, 0x5b4);//最大分片字节数=1460(0x5b4)		
		
			//socket中断设置
			setSIMR(S2_INT);//启用socket 1中断
			setSn_IMR(2,IMR_RECV);//UDP端口接收中断
		
		  //设置工作模式 UDP，打开Socket 2并绑定端口
			while(socket(2,Sn_MR_UDP,(Psn_param+2)->Sn_Port,0) != 2);	
		
			break;
		case 3:
			break;
		case 4:
			break;
		case 5:
			break;
		case 6:
			break;
		case 7:
			break;
		default:
			break;
	}
}

/*******************************************************************************
* 函数名  : TCPServer_Service
*
* 描述    : TCP服务器服务: 采取轮询的方式获取Socket n状态，完成TCP请求
*
* 输入    : @sn: Socket寄存器编号，e.g. Socket 1 即 sn=1
*						@Procesflag：TCP数据处理完成标志位(TCP帧协议服务处理完成)
*
* 返回值  : @Sn_OPEN - 端口已打开
*						@Sn_LISTEN - 端口正在监听
*						@Sn_CLOSE - 端口已关闭
*						@TCP_PROCESS - TCP端口正在处理数据
*						@TCP_COMPLETE - TCP端口数据处理完毕
*
* 说明    : 调用本函数前确保socket已打开- @ref W5500_Socket_Init
*******************************************************************************/
uint8_t TCPServer_Service(uint8_t sn , uint16_t Procesflag)
{
	uint16_t recvsize=0,sentsize = 0; // 用于回环测试
	uint8_t TCPserv_status; //!< TCP服务状态
	
	switch(getSn_SR(sn)) //检查该socket的状态
	{
		/* Socket n 关闭 */
		case SOCK_CLOSED:
			socket(sn,Sn_MR_TCP,(Psn_param+sn)->Sn_Port,0); //打开Socket绑定TCP默认端口
			//setSn_KPALVTR(sn,2);    //设置心跳包自动发送间隔，单位时间为5s
			
			TCPserv_status = Sn_OPEN;
		break;
		
		/* Socket n 已完成初始化 */
		case SOCK_INIT:
			listen(sn); //开始监听
		
			TCPserv_status = Sn_LISTEN;
		break;
		
		/* Socket n 处于连接状态 */
		case SOCK_ESTABLISHED:
		
			if(getSn_IR(sn) & Sn_IR_CON)
			{
				setSn_IR(sn,Sn_IR_CON); //清除socket标志位
			}	

			if((recvsize = getSn_RX_RSR(sn))>0) //接收目的主机发来的TCP数据			
			{
				DMA_recv(sn,TCP_Rx_Buff,recvsize);  //从接收缓冲区全部读取
				
				TCPserv_status = TCP_RECV;
			}			
			/* 若无TCP帧服务，需手动添加以处理接收的TCP数据 */
			
			if( Procesflag & TCP_PROCESSCLP_EVT ) // 帧服务完成
			{			
				send(sn, TCP_Tx_Buff, TCP_Tx_Buff[1]+3); //TCP回复目的主机
				memset(TCP_Rx_Buff,0xff,sizeof(TCP_Rx_Buff));//清除TCP接收缓冲区
				
				TCPserv_status = TCP_SEND;
			}
		
		break;
			
		/* Socket n 断开请求 */
		case SOCK_CLOSE_WAIT:
			close(sn);
			
			TCPserv_status = Sn_CLOSED; 
		break;
	}
	
	return( TCPserv_status );
}

/*******************************************************************************
* 函数名  : UDP_Service
*
* 描述    : UDP服务: 采取轮询的方式获取Socket n状态，完成UDP收发
*
* 输入    : @sn: Socket寄存器编号，e.g. Socket 1 即 sn=1
*						@Procesflag ：UDP帧协议服务处理完成标志位
*
* 返回值  : @UDP_RECV - UDP接收一帧
*						@UDP_SEND	-	UDP发送完毕
*
* 说明    : 调用本函数前确保socket已打开- @ref W5500_Socket_Init
*******************************************************************************/
uint8_t UDP_Service(uint8_t sn, uint16_t Procesflag)
{
	uint16_t recvsize=0;
	uint8_t UDPserv_status,tmp; //!< UDP服务状态
	
	switch(getSn_SR(sn))
	{
		
		/* Socket n 关闭 */
		case SOCK_CLOSED:
			socket(sn,Sn_MR_UDP,(Psn_param+sn)->Sn_DPort,0); //打开Socket绑定UDP默认端口
		break;
		
		/* Socket n 已完成初始化 */
		case SOCK_UDP:
			
			switch(sn)
				
			/* Socket 1 - UDP数据通道 */
				case 1:
				{
					if( Procesflag & UDP_DTPROCESSCLP_EVT ) // UDP数据帧协议服务处理完成，UDP端口准备发数
					{			
						DMA_sendto(sn, UDP_DTx_Buff,    				\
											sizeof(UDP_DTx_Buff), 				\
											(Psn_param+sn)->UDP_DIPR,  	\
											(Psn_param+sn)->UDP_DPORT);  //一次发送UDP发送缓冲区所有数据
					 
						UDPserv_status = UDP_SEND;			 	
					}
					break;					
				}
				
			/* Socket 2 - UDP标签通道 */
				case 2:
				{
					if( Procesflag&TRIGGER_EVT )
					{				
						if((recvsize = getSn_RX_RSR(sn)) > 0)
						{
							/* 读取UDP端口数据 */
							recvfrom(	sn,UDP_TrgRx_Buff,             \
												recvsize,										\
												(Psn_param+sn)->UDP_DIPR,  	\
												&(Psn_param+sn)->UDP_DPORT); 												 					
						}	
									
							/* 清除中断标志位 */
							tmp=getSIR();
							setSIR(tmp);			
							tmp = getSn_IR(sn);			
							setSn_IR(sn, tmp);
								
						UDPserv_status= UDP_RECV;					
					}
					else if( Procesflag&UDP_TRGPROCESSCLP_EVT )
					{
							/* UDP标签帧服务处理完毕，回复标签+本机时间戳信息 */		
						sendto(sn, UDP_TrgTx_Buff,  \
										6, 		\
										(Psn_param+sn)->UDP_DIPR,  	\
										(Psn_param+sn)->UDP_DPORT);  //一次发送UDP发送缓冲区所有数据
						
						UDPserv_status = UDP_SEND;						
					}
				
				 break;
				}

		break;
	}
	
	 return ( UDPserv_status );		
}
