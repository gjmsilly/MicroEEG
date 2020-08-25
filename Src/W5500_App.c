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
#include <string.h>

#include "stm32f4xx_hal.h"
#include "main.h"
#include "W5500_App.h"
#include "W5500.h"
#include "wizchip_conf.h"
#include "socket.h"
/***************基本参数变量定义（便于阅读）***************/
#define TRUE	0xff
#define FALSE	0x00
/***************寄存器每位定义（便于使用）***************/
	#define IR_SEND_OK		0x10
	#define IR_TIMEOUT		0x08
	#define IR_RECV			0x04
	#define IR_DISCON		0x02
	#define IR_CON			0x01
/***************----- 网络参数变量定义 -----***************/
uint8_t Gateway_IP[4];//网关IP地址 
uint8_t Sub_Mask[4];	//子网掩码 
uint8_t Phy_Addr[6];	//物理地址(MAC) 
uint8_t IP_Addr[4];	//本机IP地址 

uint16_t S0_Port;	//端口0的端口号(5000) 
uint8_t S0_DIP[4];	//端口0目的IP地址 
uint16_t S0_DPort;	//端口0目的端口号(6000) 

uint8_t UDP_DIPR[4];	//UDP(广播)模式,目的主机IP地址
uint16_t UDP_DPORT;	//UDP(广播)模式,目的主机端口号
uint16_t UDP_DPORT_[2];	
/***************----- 端口的运行模式 -----***************/
uint8_t S0_Mode =3;	//端口0的运行模式,0:TCP服务器模式,1:TCP客户端模式,2:UDP(广播)模式
#define TCP_SERVER	0x00	//TCP服务器模式
#define TCP_CLIENT	0x01	//TCP客户端模式 
#define UDP_MODE	0x02	//UDP(广播)模式 

/***************----- 端口的运行状态 -----***************/
uint8_t S0_State =0;	//端口0状态记录,1:端口完成初始化,2端口完成连接(可以正常传输数据) 
#define S_INIT		0x01	//端口完成初始化 
#define S_CONN		0x02	//端口完成连接,可以正常传输数据 

/***************----- 端口收发数据的状态 -----***************/
uint8_t S0_Data;		//端口0接收和发送数据的状态,1:端口接收到数据,2:端口发送数据完成 
#define S_RECEIVE	 0x01	//端口接收到一个数据包 
#define S_TRANSMITOK 0x02	//端口发送一个数据包完成 

/***************----- 端口数据缓冲区 -----***************/
uint8_t Rx_Buffer[2048];	//端口接收数据缓冲区 
uint8_t Tx_Buffer[2048];	//端口发送数据缓冲区 

uint8_t W5500_Interrupt;	//W5500中断标志(0:无中断,1:有中断)
extern int32_t sendstatus;
extern uint8_t DMA_is_Sending;//1:DMA正在发送数据，0：DMA完成发送
extern uint8_t DMA_is_ReSending;//1:DMA正在发送接收的数据，0：DMA完成接收发送
/*******************************************************************************
* 函数名  : W5500_Initialization
* 描述    : W5500初始货配置
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void W5500_Initialization(void)
{
	W5500_Init();		//初始化W5500寄存器函数
	Detect_Gateway();	//检查网关服务器 
	//Socket_Init(0);		
	if(S0_Mode==UDP_MODE)
	{
		socket(0,Sn_MR_UDP,5000,0);//Socket0端口初始化，UDP模式，端口号为5000
	}
	else
	{
	  socket(0,Sn_MR_TCP,5000,0);//Socket0端口初始化，TCP模式，端口号为5000
	}
	
	W5500_Socket_Set();//W5500端口初始化配置
}

/*******************************************************************************
* 函数名  : W5500_Init
* 描述    : 初始化W5500寄存器函数
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 在使用W5500之前，先对W5500初始化
*******************************************************************************/
void W5500_Init(void)
{
	uint8_t i=0;

  setMR(MR_RST);//软件复位W5500,置1有效,复位后自动清0
	HAL_Delay(10);//延时10ms,自己定义该函数

	//设置网关(Gateway)的IP地址,Gateway_IP为4字节unsigned char数组,自己定义 
	//使用网关可以使通信突破子网的局限，通过网关可以访问到其它子网或进入Internet
	setGAR(Gateway_IP);

	//设置子网掩码(MASK)值,SUB_MASK为4字节unsigned char数组,自己定义
	//子网掩码用于子网运算
	setSUBR(Sub_Mask);		
	
	//设置物理地址,PHY_ADDR为6字节unsigned char数组,自己定义,用于唯一标识网络设备的物理地址值
	//该地址值需要到IEEE申请，按照OUI的规定，前3个字节为厂商代码，后三个字节为产品序号
	//如果自己定义物理地址，注意第一个字节必须为偶数
	setSHAR(Phy_Addr);		

	//设置本机的IP地址,IP_ADDR为4字节unsigned char数组,自己定义
	//注意，网关IP必须与本机IP属于同一个子网，否则本机将无法找到网关
	setSIPR(IP_Addr);		
	
	//设置发送缓冲区和接收缓冲区的大小并清空发送缓冲区和接收缓冲区，参考W5500数据手册
	for(i=0;i<8;i++)
	{
		setSn_RXBUF_SIZE(i, 0x02);//Socket Rx memory size=2k
		setSn_TXBUF_SIZE(i, 0x02);//Socket Tx mempry size=2k
	}

	//设置重试时间，默认为2000(200ms) 
	//每一单位数值为100微秒,初始化时值设为2000(0x07D0),等于200毫秒
	setRTR(0x07d0);

	//设置重试次数，默认为8次 
	//如果重发的次数超过设定值,则产生超时中断(相关的端口中断寄存器中的Sn_IR 超时位(TIMEOUT)置“1”)
	setRCR(8);
	
	//设置分片长度，参考W5500数据手册，该值可以不修改	
	setSn_MSSR(0, 0x5b4);//最大分片字节数=1460(0x5b4)
	
	//启动中断，参考W5500数据手册确定自己需要的中断类型
	//其它是Socket事件中断，根据需要添加
	setIMR(IM_IR7 | IM_IR6);//启用IP冲突中断，UDP目的地址不能抵达；关闭PPPoE，Magic Packet中断	
	setSIMR(1);//启用Socket 0中断
	setSn_IMR(0, IMR_SENDOK | IMR_TIMEOUT | IMR_RECV | IMR_DISCON | IMR_CON);
}

/*******************************************************************************
* 函数名  : Load_Net_Parameters
* 描述    : 装载网络参数
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 网关、掩码、物理地址、本机IP地址、端口号、目的IP地址、目的端口号、端口工作模式
*******************************************************************************/
void Load_Net_Parameters(void)
{
	Gateway_IP[0] = 192;//加载网关参数0xC0 0A8
	Gateway_IP[1] = 168;
	Gateway_IP[2] = 1;
	Gateway_IP[3] = 1;

	Sub_Mask[0]=255;//加载子网掩码
	Sub_Mask[1]=255;
	Sub_Mask[2]=255;
	Sub_Mask[3]=0;

	Phy_Addr[0]=0x0c;//加载物理地址
	Phy_Addr[1]=0x29;
	Phy_Addr[2]=0xab;
	Phy_Addr[3]=0x7c;
	Phy_Addr[4]=0x00;
	Phy_Addr[5]=0x01;

	IP_Addr[0]=192;//加载本机IP地址
	IP_Addr[1]=168;
	IP_Addr[2]=1;
	IP_Addr[3]=10;

	S0_Port = 5000;//加载端口0的端口号5000(0x1388) 

	UDP_DIPR[0] = 192;	//UDP(广播)模式,目的主机IP地址
	UDP_DIPR[1] = 168;
	UDP_DIPR[2] = 1;
	UDP_DIPR[3] = 101;

	UDP_DPORT = 6000;	//UDP(广播)模式,目的主机端口号

	//S0_Mode=UDP_MODE;//加载端口0的工作模式,UDP模式
	S0_Mode=TCP_SERVER;//加载端口0的工作模式,TCP服务端模式
	//S0_Mode=TCP_CLIENT;//加载端口0的工作模式,TCP客户端模式
	
	if(S0_Mode==TCP_CLIENT)//TCP客户端模式需要目的端口号和IP地址
	{
		S0_DIP[0]=192;//加载端口0的目的IP地址
  	S0_DIP[1]=168;
	  S0_DIP[2]=1;
	  S0_DIP[3]=101;
	
	  S0_DPort = 6000;//加载端口0的目的端口号6000
	}
}

/*******************************************************************************
* 函数名  : Detect_Gateway
* 描述    : 检查网关服务器
* 输入    : 无
* 输出    : 无
* 返回值  : 成功返回TRUE(0xFF),失败返回FALSE(0x00)
* 说明    : 无
*******************************************************************************/
uint8_t Detect_Gateway(void)
{
	uint8_t ip_adde[4];
	ip_adde[0]=IP_Addr[0]+1;
	ip_adde[1]=IP_Addr[1]+1;
	ip_adde[2]=IP_Addr[2]+1;
	ip_adde[3]=IP_Addr[3]+1;

	//检查网关及获取网关的物理地址
	setSn_DIPR(0,ip_adde);//向目的地址寄存器写入与本机IP不同的IP值
	setSn_MR(0,Sn_MR_TCP);//设置socket0为TCP模式
	setSn_CR(0,Sn_CR_OPEN);//打开Socket0
	HAL_Delay(5);//延时5ms 	
	
	if(getSn_SR(0) != SOCK_INIT)//如果socket打开失败
	{
		setSn_CR(0,Sn_CR_CLOSE);//打开不成功,关闭Socket0
		return FALSE;//返回FALSE(0x00)
	}

	setSn_CR(0,Sn_CR_CONNECT);//设置Socket0为Connect模式						

	do
	{
		uint8_t j=0;
		j=getSn_IR(0);//读取Socket0中断标志寄存器
		if(j!=0)
		setSn_IR(0,j);
		HAL_Delay(5);//延时5ms 
		if((j&Sn_IR_TIMEOUT) == Sn_IR_TIMEOUT)
		{
			return FALSE;	
		}
		else if(WIZCHIP_READ(Sn_DHAR(0)) != 0xff)
		{
			setSn_CR(0,Sn_CR_CLOSE);//关闭Socket
			return TRUE;							
		}
	}while(1);
}


/*******************************************************************************
* 函数名  : Socket_Init
* 描述    : 指定Socket(0~7)初始化
* 输入    : s:待初始化的端口
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void Socket_Init(uint8_t s)
{
	//设置分片长度，参考W5500数据手册，该值可以不修改	
	setSn_MSSR(0, 0x5b4);//最大分片字节数=1460(0x5b4)
	//设置指定端口
	switch(s)
	{
		case 0:
			//设置端口0的端口号
			setSn_PORT(0, S0_Port);
	    if(S0_Mode==TCP_CLIENT)//只有TCP客户端模式需要设置目的端口号和IP地址
	    {
		 	  //设置端口0目的(远程)端口号
			  setSn_DPORT(0, S0_DPort);
			  //设置端口0目的(远程)IP地址
		  	setSn_DIPR(0, S0_DIP);		
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
* 描述    : W5500端口初始化配置
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 分别设置4个端口,根据端口工作模式,将端口置于TCP服务器、TCP客户端或UDP模式.
*			从端口状态字节Socket_State可以判断端口的工作情况
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
					DMA_is_Sending=0;//重置标志位
					DMA_is_ReSending=0;//重置标志位
				}
	  	}
			else if(S0_Mode==TCP_CLIENT)
			{
				socket(0,Sn_MR_TCP,5000,0);//Socket0端口初始化，TCP模式，端口号为5000
				S0_State=0;//网络连接状态0x00,端口连接失败
				while(connect(0, S0_DIP, S0_DPort))//再次侦听
				{
					S0_State=S_INIT;//端口初始化成功
					DMA_is_Sending=0;//重置标志位
					DMA_is_ReSending=0;//重置标志位
				}
	  	}
			else
			{
			  socket(0,Sn_MR_UDP,5000,0);//Socket0端口初始化，UDP模式，端口号为5000
				S0_State=S_INIT|S_CONN;//端口初始化成功
				DMA_is_Sending=0;//重置标志位
				DMA_is_ReSending=0;//重置标志位
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

