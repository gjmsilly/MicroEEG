
/*******************************************************************************
 * INCLUDES
 */
#include <string.h>

#include "stm32f4xx_hal.h"
#include "main.h"
#include "W5500_App.h"
#include "W5500.h"
#include "wizchip_conf.h"
#include "socket.h"

/*******************************************************************************
 * GLOBAL VARIABLES
 */
NETWORKParam_t net_param,*Pnet_param;    		//网络参数配置
SOCKETnParam_t s0_param,*Ps0_param;   			//Socket 0参数配置
SOCKETnState_t s0_state,*Ps0_state;   			//Socket 0状态

uint8_t Rx_Buffer[2048];										//接收数据缓冲区 
uint8_t Tx_Buffer[2048];										//发送数据缓冲区 

uint8_t W5500_Interrupt;										//W5500中断标志 0-无，1-有中断

 /*******************************************************************************
 * EXTERNAL VARIABLES
 */

extern uint8_t DMA_TX_Transfer_flag;		//1 - DMA正在传输发送区数据，0 - 传输完成
extern uint8_t DMA_RX_Transfer_flag;		//1 - DMA正在传输接收区数据，0 - 传输完成

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
* 说明    : 首先完成W5500硬件复位，之后最先调用本函数
*******************************************************************************/
void W5500_Init(void)
{
	Pnet_param =&net_param;
	Ps0_param =&s0_param;
	Ps0_state =&s0_state;
	
	W5500_Config();						//初始化W5500通用寄存器区
	Detect_Gateway();					//检查网关服务器 
	W5500_Socket_Init(0);			//Socket配置（默认使用Socket 0）
	W5500_Socket_State(0);		//Socket状态配置
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
* 说明    : 
*******************************************************************************/
void W5500_Config(void)
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
	//每一单位数值为100微秒,初始化时值设为2000(0x07D0),即200ms
	setRTR(0x07d0);

	//设置重试次数，默认为8次 
	//如果重发的次数超过设定值,则产生超时中断(相关的端口中断寄存器中的Sn_IR 超时位(TIMEOUT)置“1”)
	setRCR(8);

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
*						@Gateway_IP		网关IP地址 					32bit				192.168.1.1
*						@Sub_Mask			子网掩码 						32bit       255.255.255.0
*						@Phy_Addr			MAC地址							48bit       0c-29-ab-7c-00-01
*						@IP_Addr			源IP地址						32bit       192.168.1.10
*						@Sn_Port			Socket 0端口号（源端口号）			5000 (default)
*						@Sn_DIP				TCP模式目的IP地址		32bit				192.128.1.101(client)
*						@UDP_DPORT		UDP模式目的端口号								6000 (defualt)
*						@UDP_DIPR     UDP模式目的IP地址		32bit				192.168.1.101
*******************************************************************************/
void W5500_Load_Net_Parameters(void)
{
	//加载网关参数		
	net_param.Gateway_IP[0] = 192;
	net_param.Gateway_IP[1] = 168;
	net_param.Gateway_IP[2] = 1;
	net_param.Gateway_IP[3] = 1;
	
	//加载子网掩码
	net_param.Sub_Mask[0] = 255;
	net_param.Sub_Mask[1] = 255;
	net_param.Sub_Mask[2] = 255;
	net_param.Sub_Mask[3] = 0;
	
	//加载MAC地址
	net_param.Phy_Addr[0] = 0x0c;
	net_param.Phy_Addr[1] = 0x29;
	net_param.Phy_Addr[2] = 0xab;
	net_param.Phy_Addr[3] = 0x7c;
	net_param.Phy_Addr[4] = 0x00;
	net_param.Phy_Addr[5] = 0x01;
	
	//加载源/本机IP地址
	net_param.IP_Addr[0] = 192;
	net_param.IP_Addr[1] = 168;
	net_param.IP_Addr[2] = 1;
	net_param.IP_Addr[3] = 10;
	
	//加载Socket 0的端口号/源端口号 5000(0x1388) （default）
	s0_param.Sn_Port = 5000;

	//UDP(广播)模式,目的主机IP地址
	s0_param.UDP_DIPR[0] = 192;	
	s0_param.UDP_DIPR[1] = 168;
	s0_param.UDP_DIPR[2] = 1;
	s0_param.UDP_DIPR[3] = 101;
	
	//UDP(广播)模式,目的主机端口号 6000（default）
	s0_param.UDP_DPORT = 6000;	
	
	// 加载Socket 0的工作模式：服务器（default）
	// @ref UDP_MODE/TCP_CLIENT/TCP_SERVER
	s0_state.Sn_Mode=TCP_SERVER;	
	
	//TCP客户端模式：需要设置目的端口号和IP地址
	if(s0_state.Sn_Mode==TCP_CLIENT)
	{
		//加载Socket 0的目的IP地址
		s0_param.Sn_DIP[0]=192;
  	s0_param.Sn_DIP[1]=168;
	  s0_param.Sn_DIP[2]=1;
	  s0_param.Sn_DIP[3]=101;
		
		//加载Socket 0的目的端口号 6000（default）
	  s0_param.Sn_DPort = 6000;
	}
}

/*******************************************************************************
* 函数名  : Detect_Gateway
*
* 描述    : 检查网关服务器 （使用Socket 0）
*
* 输入    : 无
*
* 返回值  : @TRUE/0xff 	- 正常
*						@FALSE/0x00 - 异常
*
* 说明    : 无
*******************************************************************************/
uint8_t Detect_Gateway(void)
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
	setSIMR(1);//启用Socket 0中断(对@ref SIMR bit1置1即使能Socket 0中断)
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
void W5500_Socket_Init(uint8_t sn)
{
	/* Socket 寄存器区设置 */
	
	//设置发送缓冲区和接收缓冲区的大小并清空发送缓冲区和接收缓冲区，默认2K
	for(uint8_t i=0;i<8;i++)
	{
		setSn_RXBUF_SIZE(i, 0x02);//Socket Rx memory size=2k
		setSn_TXBUF_SIZE(i, 0x02);//Socket Tx mempry size=2k
	}
	
	//设置指定Socket
	switch(sn)
	{
		case 0:
			/* Socket 0 */
		
			//设置分片长度
			setSn_MSSR(0, 0x5b4);//最大分片字节数=1460(0x5b4)
			
			//设置中断
			setIMR(IM_IR7 | IM_IR6);//启用IP冲突中断，UDP目的地址不能抵达中断；关闭PPPoE，Magic Packet中断	
			setSIMR(1);//启用Socket 0中断(对@ref SIMR bit1置1即使能Socket 0中断)
			setSn_IMR(0, IMR_SENDOK | IMR_TIMEOUT | IMR_RECV | IMR_DISCON | IMR_CON);
			
			//设置工作模式并打开Socket 0
			if(Ps0_state->Sn_Mode == UDP_MODE)
			{
				socket(0,Sn_MR_UDP,Ps0_param->Sn_Port,0);//设置Socket0，UDP模式，源端口号5000
			}
			else
			{
				socket(0,Sn_MR_TCP,Ps0_param->Sn_Port,0);//设置Socket0，TCP模式，源端口号5000
			}
	  
			// Socket 0为客户端时需设置目的端口、目的IP地址 
			if(Ps0_state->Sn_Mode==TCP_CLIENT)
	    {
		 	  //设置目的端口号
			  setSn_DPORT(0, Ps0_param->Sn_DPort);
			  //设置目的IP地址
		  	setSn_DIPR(0, Ps0_param->Sn_DIP);		
	    }			
			
			break;
		case 1:
			break;
		case 2:
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
* 函数名  : W5500_Socket_State
*
* 描述    : Socket连接状态初始化
*
* 输入    : @sn: Socket寄存器编号，e.g. Socket 1 即 sn=1
*
* 返回值  : 无
*
* 说明    : 本函数必须在- @ref W5500_Socket_Init后调用以确保socket打开
*******************************************************************************/
void W5500_Socket_State(uint8_t sn)
{
 	switch(sn)
	{
		case 0:
		if(Ps0_state->Sn_Mode == TCP_SERVER) //TCP服务器模式 
		{
			if(listen(0)==SOCK_OK)  //socket n连接初始化成功
			{
				Ps0_state->Sn_State = S_INIT;
			}
			else
			{
				Ps0_state->Sn_State = 0;
			}
		}
		else if(Ps0_state->Sn_Mode == TCP_CLIENT)// TCP客户端模式 
		{
			if(connect(0, Ps0_param->Sn_DIP, Ps0_param->Sn_DPort)==SOCK_OK) //socket n连接初始化成功
				Ps0_state->Sn_State = S_INIT;
			else
				Ps0_state->Sn_State = 0;
		}
		else //UDP模式，无需连接
		{
				Ps0_state->Sn_State = S_INIT|S_CONN;
		}
			break;
		case 1:
			break;
		case 2:
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
* 函数名  : W5500_Interrupt_Process
*
* 描述    : W5500中断处理程序
*
* 输入    : 无
*
* 返回值  : 无
*
* 说明    : 无
*******************************************************************************/
void W5500_Interrupt_Process(void)
{
	uint8_t flag_c,flag_s;

	IntDispose:
	W5500_Interrupt=0;//清除中断标志
	flag_c = getIR();//读取中断标志寄存器
	setIR(flag_c);	 //清除中断标志
	
	flag_s=getSIR(); //读取Socket中断标志寄存器	
	
	if((flag_c & S0_INT) == S0_INT) //Socket n中断事件处理 
	{
		flag_s=getSn_IR(0);//读取Socket n中断标志寄存器
		setSn_IR(0,flag_s);//Socket0清除中断标志
		
		if(flag_s & IR_CON)	//在TCP模式下，Socket n成功连接 
		{
			Ps0_state->Sn_State|=S_CONN; //更新状态为连接成功，可以传输数据
		}
		
		if(flag_s & IR_DISCON)//在TCP模式下，Socket n断开连接
		{
			//TCP SERVER 模式
			if(Ps0_state->Sn_Mode==TCP_SERVER)
			{
				Ps0_state->Sn_State=0;//更新状态为连接失败
						
				socket(0,Sn_MR_TCP,Ps0_param->Sn_Port,0);//Socket n端口初始化，TCP模式，端口号为5000
			
				if(listen(0)==SOCK_OK)//再次侦听成功
				{
					Ps0_state->Sn_State=S_INIT;//Socket n连接初始化成功
					
					DMA_TX_Transfer_flag=0;//重置标志位
					DMA_RX_Transfer_flag=0;//重置标志位
				}
	  	}
			//TCP CLIENT 模式
			else if(Ps0_state->Sn_Mode==TCP_CLIENT)
			{
				Ps0_state->Sn_State=0;//更新状态为连接失败
				
				socket(0,Sn_MR_TCP,Ps0_param->Sn_Port,0);//Socket n初始化，TCP模式，端口号为5000
				
				while(connect(0, Ps0_param->Sn_DIP, Ps0_param->Sn_DPort))//再次侦听
				{
					Ps0_state->Sn_State=S_INIT;//Socket n连接初始化成功
					
					DMA_TX_Transfer_flag=0;//重置标志位
					DMA_RX_Transfer_flag=0;//重置标志位
				}
	  	}
			//UDP 模式，无需连接，Socket n打开成功即可准备传输数据
			else	
			{
			  socket(0,Sn_MR_UDP,Ps0_param->Sn_Port,0);//Socket n初始化，UDP模式，端口号为5000
				Ps0_state->Sn_State=S_INIT|S_CONN;
				
				DMA_TX_Transfer_flag=0;//重置标志位
				DMA_RX_Transfer_flag=0;//重置标志位
			}
		}
		
		if(flag_s&IR_SEND_OK)//Socket n数据发送完成,可以再次启动函数发送数据 
		{
			Ps0_state->Sn_Data |=S_TRANSMITOK;//更新状态为发送完成
		}
		
		if(flag_s&IR_RECV)//Socket接收到数据,可以启动接收函数 
		{
			Ps0_state->Sn_Data = S_RECEIVE;//更新状态为接收到数据
		}
		
		if(flag_s&IR_TIMEOUT)//Socket连接或数据传输超时处理 
		{
			setSn_CR(0,Sn_CR_CLOSE);// 关闭Socket n,等待重新打开连接 
			Ps0_state->Sn_State=0;//更新状态为连接失败
		}
	}

	if(getSIR() != 0) 
		goto IntDispose;
}


/*******************************************************************************
* 函数名  : W5500_RST
*
* 描述    : W5500_RST引脚初始化配置(PA3)
*
* 输入    : 无
*
* 返回值  : 无
*
* 说明    : 低电平有效，低电平至少保持500us以上
*******************************************************************************/
void W5500_RST(void)
{
	LL_GPIO_ResetOutputPin(W5500_nRST_GPIO_Port,W5500_nRST_Pin);//复位引脚拉低
	HAL_Delay(50);
	LL_GPIO_SetOutputPin(W5500_nRST_GPIO_Port,W5500_nRST_Pin);//复位引脚拉高
	HAL_Delay(100);
	while((WIZCHIP_READ(PHYCFGR)&PHYCFGR_LNK_ON)==0);//等待以太网连接完成


}

