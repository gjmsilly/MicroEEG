/***************************************************************************************
 * 描述    ：W5500的端口0工作在UDP模式,端口通信不需要建立连接即可接收其它UDP模式端口发来的数据,
 *			 数据的前4个字节为对方端口IP地址,第5、6个字节为端口号，获得对方IP和端口号后W5500端口0
 *			 将定时给对应端口发送字符串"\r\nWelcome To YiXinElec!\r\n",同时将接收
 *			 到的数据回发给对应端口。
 * 实验平台：用户STM32开发板 + YIXIN_W5500以太网(TCP/IP)模块
 * 硬件连接：  PC5 -> W5500_RST     
 *             PA4 -> W5500_SCS      
 *             PA5 -> W5500_SCK    
 *             PA6 -> W5500_MISO    
 *             PA7 -> W5500_MOSI    
 * 库版本  ：

***************************************************************************************/
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
NETWORKParam_t net_param,*Pnet_param;    		//网络配置
SOCKETnParam_t s0_param,*Ps0_param;   			//Socket0配置
SOCKETnState_t s0_state,*Ps0_state;   			//Socket0状态

uint8_t Rx_Buffer[2048];										//接收数据缓冲区 
uint8_t Tx_Buffer[2048];										//发送数据缓冲区 

uint8_t W5500_Interrupt;										//W5500中断标志 0-无，1-有中断

 /*******************************************************************************
 * EXTERNAL VARIABLES
 */

extern int32_t Transferstatus;
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
* 说明    : 无
*******************************************************************************/
void W5500_Init(void)
{
	Pnet_param =&net_param;
	Ps0_param =&s0_param;
	Ps0_state =&s0_state;
	
	W5500_Config();			//初始化W5500寄存器
	Detect_Gateway();		//检查网关服务器 
	
	if(Ps0_state->Sn_Mode == UDP_MODE)
	{
		socket(0,Sn_MR_UDP,5000,0);//设置Socket0，UDP模式，端口号5000
	}
	else
	{
	  socket(0,Sn_MR_TCP,5000,0);//设置Socket0，TCP模式，端口号5000
	}
	
	W5500_Socket_Set();		//W5500初始化配置
}

/*******************************************************************************
* 函数名  : W5500_Config
*
* 描述    : 初始化W5500寄存器
*
* 输入    : 无
*
* 返回值  : 无
*
* 说明    : W5500硬件复位后最先调用本函数
*******************************************************************************/
void W5500_Config(void)
{
	uint8_t i=0;

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
	
	//设置发送缓冲区和接收缓冲区的大小并清空发送缓冲区和接收缓冲区，默认2K
	for(i=0;i<8;i++)
	{
		setSn_RXBUF_SIZE(i, 0x02);//Socket Rx memory size=2k
		setSn_TXBUF_SIZE(i, 0x02);//Socket Tx mempry size=2k
	}

	//设置重试时间，默认2000(200ms) 
	//每一单位数值为100微秒,初始化时值设为2000(0x07D0),即200ms
	setRTR(0x07d0);

	//设置重试次数，默认为8次 
	//如果重发的次数超过设定值,则产生超时中断(相关的端口中断寄存器中的Sn_IR 超时位(TIMEOUT)置“1”)
	setRCR(8);
	
	//设置分片长度
	setSn_MSSR(0, 0x5b4);//最大分片字节数=1460(0x5b4)
	
	//设置中断
	setIMR(IM_IR7 | IM_IR6);//启用IP冲突中断，UDP目的地址不能抵达中断；关闭PPPoE，Magic Packet中断	
	setSIMR(1);//启用Socket 0中断(对@ref SIMR bit1置1即使能Socket 0中断)
	setSn_IMR(0, IMR_SENDOK | IMR_TIMEOUT | IMR_RECV | IMR_DISCON | IMR_CON);
}


/*******************************************************************************
* 函数名  : Load_Net_Parameters
*
* 描述    : 配置网络参数
*
* 输入    : 无
*
* 返回值  : 无
*
* 说明    : 在W5500 初始化前调用本函数以预先装载寄存器配置值
*						@Gateway_IP		网关IP地址 					32bit				192.168.1.1
*						@Sub_Mask			子网掩码 						32bit       255.255.255.0
*						@Phy_Addr			MAC地址							48bit       0c-29-ab-7c-00-01
*						@IP_Addr			源IP地址						32bit       192.168.1.10
*						@Sn_Port			Socket 0端口号（源端口号）			5000 (default)
*						@Sn_DIP				TCP模式目的IP地址		32bit				192.128.1.101(client)
*						@UDP_DPORT		UDP模式目的端口号								6000 (defualt)
*						@UDP_DIPR     UDP模式目的IP地址		32bit				192.128.1.101
*******************************************************************************/

void Load_Net_Parameters(void)
{
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
	
	//加载MAC地址
	Pnet_param->Phy_Addr[0] = 0x0c;
	Pnet_param->Phy_Addr[1] = 0x29;
	Pnet_param->Phy_Addr[2] = 0xab;
	Pnet_param->Phy_Addr[3] = 0x7c;
	Pnet_param->Phy_Addr[4] = 0x00;
	Pnet_param->Phy_Addr[5] = 0x01;
	
	//加载源/本机IP地址
	Pnet_param->IP_Addr[0] = 192;
	Pnet_param->IP_Addr[1] = 168;
	Pnet_param->IP_Addr[2] = 1;
	Pnet_param->IP_Addr[3] = 10;
	
	//加载Socket 0的端口号/源端口号 5000(0x1388) （default）
	Ps0_param->Sn_Port = 5000;

	//UDP(广播)模式,目的主机IP地址
	Ps0_param->UDP_DIPR[0] = 192;	
	Ps0_param->UDP_DIPR[1] = 168;
	Ps0_param->UDP_DIPR[2] = 1;
	Ps0_param->UDP_DIPR[3] = 101;
	
	//UDP(广播)模式,目的主机端口号 6000（default）
	Ps0_param->UDP_DPORT = 6000;	
	
	// 加载Socket 0的工作模式：服务器（default）
	// @ref UDP_MODE/TCP_CLIENT/TCP_SERVER
	Ps0_state->Sn_Mode=TCP_SERVER;	
	
	//TCP客户端模式：需要设置目的端口号和IP地址
	if(Ps0_state->Sn_Mode==TCP_CLIENT)
	{
		//加载Socket 0的目的IP地址
		Ps0_param->Sn_DIP[0]=192;
  	Ps0_param->Sn_DIP[1]=168;
	  Ps0_param->Sn_DIP[2]=1;
	  Ps0_param->Sn_DIP[3]=101;
		
		//加载Socket 0的目的端口号 6000（default）
	  Ps0_param->Sn_DPort = 6000;
	}
}

/*******************************************************************************
* 函数名  : Detect_Gateway
*
* 描述    : 检查网关服务器
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
* 函数名  : Socket_Init
*
* 描述    : 指定Socket n(0~7)初始化
*
* 输入    : 无
*
* 返回值  : 无
*
* 说明    : 无
*******************************************************************************/
void Socket_Init(uint8_t s)
{
	//设置分片长度	
	setSn_MSSR(0, 0x5b4); // 最大分片字节数=1460(0x5b4)
	
	//设置指定Socket
	switch(s)
	{
		case 0:
			
		// 设置Socket 0的端口号
			setSn_PORT(0, Ps0_param->Sn_Port);
	  
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
* 函数名  : W5500_Socket_Set
*
* 描述    : W5500端口初始化配置
*
* 输入    : 无
*
* 返回值  : 无
*
* 说明    : 无
*******************************************************************************/
void W5500_Socket_Set(void)
{
	if(S0_State==0)//端口0初始化配置
	{
		if(S0_Mode==TCP_SERVER)//TCP服务器模式 
		{
			if(listen(0)==SOCK_OK)
			{
				S0_State=S_INIT;
			}
			else
			{
				S0_State=0;
			}
		}
		else if(S0_Mode==TCP_CLIENT)//TCP客户端模式 
		{
			if(connect(0, S0_DIP, S0_DPort)==SOCK_OK) //socket0建立连接，目的ip地址，目的端口号
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else//UDP模式 
		{
			//if(Socket_UDP(0)==TRUE)
				S0_State=S_INIT|S_CONN;
			//else
			//	S0_State=0;
		}
	}
}

/*******************************************************************************
* 函数名  : Socket_Connect
* 描述    : 设置指定Socket(0~7)为客户端与远程服务器连接
* 输入    : s:待设定的端口
* 输出    : 无
* 返回值  : 成功返回TRUE(0xFF),失败返回FALSE(0x00)
* 说明    : 当本机Socket工作在客户端模式时,引用该程序,与远程服务器建立连接
*			如果启动连接后出现超时中断，则与服务器连接失败,需要重新调用该程序连接
*			该程序每调用一次,就与服务器产生一次连接
*******************************************************************************/
uint8_t Socket_Connect(uint8_t s)
{
	setSn_MR(s,Sn_MR_TCP);//设置socket为TCP模式
	setSn_CR(s,Sn_CR_OPEN);//打开Socket
	HAL_Delay(5);//延时5ms
	if(getSn_CR(s)!=SOCK_INIT)//如果socket打开失败
	{
		setSn_CR(s,Sn_CR_CLOSE);//打开不成功,关闭Socket
		return FALSE;//返回FALSE(0x00)
	}
	setSn_CR(s,Sn_CR_CONNECT);//设置Socket为Connect模式
	return TRUE;//返回TRUE,设置成功
}

/*******************************************************************************
* 函数名  : Socket_Listen
* 描述    : 设置指定Socket(0~7)作为服务器等待远程主机的连接
* 输入    : s:待设定的端口
* 输出    : 无
* 返回值  : 成功返回TRUE(0xFF),失败返回FALSE(0x00)
* 说明    : 当本机Socket工作在服务器模式时,引用该程序,等等远程主机的连接
*			该程序只调用一次,就使W5500设置为服务器模式
*******************************************************************************/
uint8_t Socket_Listen(uint8_t s)
{
	setSn_MR(s,Sn_MR_TCP);//设置socket为TCP模式 
	setSn_CR(s,Sn_CR_OPEN);//打开Socket	
	HAL_Delay(5);//延时5ms
	if(getSn_SR(s)!=SOCK_INIT)//如果socket打开失败
	{
		setSn_CR(s,Sn_CR_CLOSE);//打开不成功,关闭Socket
		return FALSE;//返回FALSE(0x00)
	}	
	setSn_CR(s,Sn_CR_LISTEN);//设置Socket为侦听模式	
	HAL_Delay(5);//延时5ms
	if(getSn_SR(s)!=SOCK_LISTEN)//如果socket设置失败
	{
		setSn_CR(s,Sn_CR_CLOSE);//设置不成功,关闭Socket
		return FALSE;//返回FALSE(0x00)
	}

	return TRUE;

	//至此完成了Socket的打开和设置侦听工作,至于远程客户端是否与它建立连接,则需要等待Socket中断，
	//以判断Socket的连接是否成功。参考W5500数据手册的Socket中断状态
	//在服务器侦听模式不需要设置目的IP和目的端口号
}

/*******************************************************************************
* 函数名  : Socket_UDP
* 描述    : 设置指定Socket(0~7)为UDP模式
* 输入    : s:待设定的端口
* 输出    : 无
* 返回值  : 成功返回TRUE(0xFF),失败返回FALSE(0x00)
* 说明    : 如果Socket工作在UDP模式,引用该程序,在UDP模式下,Socket通信不需要建立连接
*			该程序只调用一次，就使W5500设置为UDP模式
*******************************************************************************/
uint8_t Socket_UDP(uint8_t s)
{
	setSn_MR(s,Sn_MR_UDP);//设置Socket为UDP模式*/
	setSn_CR(s,Sn_CR_OPEN);//打开Socket*/
	HAL_Delay(5);//延时5ms
	if(getSn_SR(s)!=SOCK_UDP)//如果Socket打开失败
	{
		setSn_CR(s,Sn_CR_CLOSE);//打开不成功,关闭Socket
		return FALSE;//返回FALSE(0x00)
	}
	else
		return TRUE;

	//至此完成了Socket的打开和UDP模式设置,在这种模式下它不需要与远程主机建立连接
	//因为Socket不需要建立连接,所以在发送数据前都可以设置目的主机IP和目的Socket的端口号
	//如果目的主机IP和目的Socket的端口号是固定的,在运行过程中没有改变,那么也可以在这里设置
}

/*******************************************************************************
* 函数名  : W5500_Interrupt_Process
* 描述    : W5500中断处理程序框架
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void W5500_Interrupt_Process(void)
{
	uint8_t i,j;

IntDispose:
	W5500_Interrupt=0;//清零中断标志
	i = getIR();//读取中断标志寄存器
	setIR(i);//清除中断标志
	
	/*if((i & IR_CONFLICT) == IR_CONFLICT)//IP地址冲突异常处理
	{
		 //自己添加代码
	}
	if((i & IR_UNREACH) == IR_UNREACH)//UDP模式下地址无法到达异常处理
	{
		//自己添加代码
	}*/
	
	i=getSIR();//读取端口中断标志寄存器	
	if((i & S0_INT) == S0_INT)//Socket0事件处理 
	{
		j=getSn_IR(0);//读取Socket0中断标志寄存器
		setSn_IR(0,j);//Socket0清除中断标志
		if(j&IR_CON)//在TCP模式下,Socket0成功连接 
		{
			S0_State|=S_CONN;//网络连接状态0x02,端口完成连接，可以正常传输数据
		}
		if(j&IR_DISCON)//在TCP模式下Socket断开连接处理
		{
			if(S0_Mode==TCP_SERVER)
			{
				socket(0,Sn_MR_TCP,5000,0);//Socket0端口初始化，TCP模式，端口号为5000
				S0_State=0;//网络连接状态0x00,端口连接失败
				if(listen(0)==SOCK_OK)//再次侦听
				{
					S0_State=S_INIT;//端口初始化成功
					DMA_TX_Transfer_flag=0;//重置标志位
					DMA_RX_Transfer_flag=0;//重置标志位
				}
	  	}
			else if(S0_Mode==TCP_CLIENT)
			{
				socket(0,Sn_MR_TCP,5000,0);//Socket0端口初始化，TCP模式，端口号为5000
				S0_State=0;//网络连接状态0x00,端口连接失败
				while(connect(0, S0_DIP, S0_DPort))//再次侦听
				{
					S0_State=S_INIT;//端口初始化成功
					DMA_TX_Transfer_flag=0;//重置标志位
					DMA_RX_Transfer_flag=0;//重置标志位
				}
	  	}
			else
			{
			  socket(0,Sn_MR_UDP,5000,0);//Socket0端口初始化，UDP模式，端口号为5000
				S0_State=S_INIT|S_CONN;//端口初始化成功
				DMA_TX_Transfer_flag=0;//重置标志位
				DMA_RX_Transfer_flag=0;//重置标志位
			}
		}
		if(j&IR_SEND_OK)//Socket0数据发送完成,可以再次启动函数发送数据 
		{
			S0_Data|=S_TRANSMITOK;//端口发送一个数据包完成 
		}
		if(j&IR_RECV)//Socket接收到数据,可以启动接收函数 
		{
			S0_Data = S_RECEIVE;//端口接收到一个数据包
		}
		if(j&IR_TIMEOUT)//Socket连接或数据传输超时处理 
		{
			setSn_CR(0,Sn_CR_CLOSE);// 关闭端口,等待重新打开连接 
			S0_State=0;//网络连接状态0x00,端口连接失败
		}
	}

	if(getSIR() != 0) 
		goto IntDispose;
}
/*void W5500_Interrupt_Process(void)
{
	uint8_t i,j;

IntDispose:

	i=getSIR();//读取端口中断标志寄存器	
	if((i & S0_INT) == S0_INT)//Socket0事件处理 
	{
		j=getSn_IR(0);//读取Socket0中断标志寄存器
		setSn_IR(0,j);
		dsf=j;
		if(j&IR_CON)//在TCP模式下,Socket0成功连接 
		{
			S0_State|=S_CONN;//网络连接状态0x02,端口完成连接，可以正常传输数据
		}
		if(j&IR_DISCON)//在TCP模式下Socket断开连接处理
		{
			setSn_CR(0,Sn_CR_CLOSE);//关闭端口,等待重新打开连接 
			Socket_Init(0);		//指定Socket(0~7)初始化,初始化端口0
			S0_State=0;//网络连接状态0x00,端口连接失败
		}
		if(j&IR_SEND_OK)//Socket0数据发送完成,可以再次启动S_tx_process()函数发送数据 
		{
			S0_Data|=S_TRANSMITOK;//端口发送一个数据包完成 
		}
		if(j&IR_RECV)//Socket接收到数据,可以启动S_rx_process()函数 
		{
			S0_Data|=S_RECEIVE;//端口接收到一个数据包
		}
		if(j&IR_TIMEOUT)//Socket连接或数据传输超时处理 
		{
			setSn_CR(0,Sn_CR_CLOSE);// 关闭端口,等待重新打开连接 			
			S0_State=0;//网络连接状态0x00,端口连接失败
		}
	}

	if(getSIR() != 0) 
		goto IntDispose;
}*/

/*******************************************************************************
* 函数名  : Process_Socket_Data
* 描述    : W5500接收并发送接收到的数据
* 输入    : s:端口号
* 输出    : 无
* 返回值  : 无
* 说明    : 本过程先调用S_rx_process()从W5500的端口接收数据缓冲区读取数据,
*			然后将读取的数据从Rx_Buffer拷贝到Temp_Buffer缓冲区进行处理。
*			处理完毕，将数据从Temp_Buffer拷贝到Tx_Buffer缓冲区。
*******************************************************************************/
void Process_Socket_Data(uint8_t s)
{
	int32_t size;
	if(S0_Mode==UDP_MODE)
	{
		size=recv(s, Rx_Buffer, 1460);
		//size=Read_SOCK_Data_Buffer(s, Rx_Buffer);
		UDP_DIPR[0] = Rx_Buffer[0];
		UDP_DIPR[1] = Rx_Buffer[1];
		UDP_DIPR[2] = Rx_Buffer[2];
		UDP_DIPR[3] = Rx_Buffer[3];

		UDP_DPORT = (Rx_Buffer[4]<<8)+Rx_Buffer[5];

		memcpy(Tx_Buffer, Rx_Buffer+8, size-8);			
		//Write_SOCK_Data_Buffer(s, Tx_Buffer, size);
		send(0, Tx_Buffer, size);
	}
	else 
	{
		size=recv(s, Rx_Buffer, 1460);
		//size=Read_SOCK_Data_Buffer(s, Rx_Buffer);
		memcpy(Tx_Buffer, Rx_Buffer, size);			
		send(0, Tx_Buffer, size);
		//Write_SOCK_Data_Buffer(s, Tx_Buffer, size);
	}
}

/*******************************************************************************
* 函数名  : W5500_RST
* 描述    : W5500_RST引脚初始化配置(PA3)
* 输入    : 无
* 输出    : 无
* 返回值  : 无
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

