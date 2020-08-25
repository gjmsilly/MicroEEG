/***************************************************************************************
 * ����    ��W5500�Ķ˿�0������UDPģʽ,�˿�ͨ�Ų���Ҫ�������Ӽ��ɽ�������UDPģʽ�˿ڷ���������,
 *			 ���ݵ�ǰ4���ֽ�Ϊ�Է��˿�IP��ַ,��5��6���ֽ�Ϊ�˿ںţ���öԷ�IP�Ͷ˿ںź�W5500�˿�0
 *			 ����ʱ����Ӧ�˿ڷ����ַ���"\r\nWelcome To YiXinElec!\r\n",ͬʱ������
 *			 �������ݻط�����Ӧ�˿ڡ�
 * ʵ��ƽ̨���û�STM32������ + YIXIN_W5500��̫��(TCP/IP)ģ��
 * Ӳ�����ӣ�  PC5 -> W5500_RST     
 *             PA4 -> W5500_SCS      
 *             PA5 -> W5500_SCK    
 *             PA6 -> W5500_MISO    
 *             PA7 -> W5500_MOSI    
 * ��汾  ��

***************************************************************************************/
#include <string.h>

#include "stm32f4xx_hal.h"
#include "main.h"
#include "W5500_App.h"
#include "W5500.h"
#include "wizchip_conf.h"
#include "socket.h"
/***************���������������壨�����Ķ���***************/
#define TRUE	0xff
#define FALSE	0x00
/***************�Ĵ���ÿλ���壨����ʹ�ã�***************/
	#define IR_SEND_OK		0x10
	#define IR_TIMEOUT		0x08
	#define IR_RECV			0x04
	#define IR_DISCON		0x02
	#define IR_CON			0x01
/***************----- ��������������� -----***************/
uint8_t Gateway_IP[4];//����IP��ַ 
uint8_t Sub_Mask[4];	//�������� 
uint8_t Phy_Addr[6];	//�����ַ(MAC) 
uint8_t IP_Addr[4];	//����IP��ַ 

uint16_t S0_Port;	//�˿�0�Ķ˿ں�(5000) 
uint8_t S0_DIP[4];	//�˿�0Ŀ��IP��ַ 
uint16_t S0_DPort;	//�˿�0Ŀ�Ķ˿ں�(6000) 

uint8_t UDP_DIPR[4];	//UDP(�㲥)ģʽ,Ŀ������IP��ַ
uint16_t UDP_DPORT;	//UDP(�㲥)ģʽ,Ŀ�������˿ں�
uint16_t UDP_DPORT_[2];	
/***************----- �˿ڵ�����ģʽ -----***************/
uint8_t S0_Mode =3;	//�˿�0������ģʽ,0:TCP������ģʽ,1:TCP�ͻ���ģʽ,2:UDP(�㲥)ģʽ
#define TCP_SERVER	0x00	//TCP������ģʽ
#define TCP_CLIENT	0x01	//TCP�ͻ���ģʽ 
#define UDP_MODE	0x02	//UDP(�㲥)ģʽ 

/***************----- �˿ڵ�����״̬ -----***************/
uint8_t S0_State =0;	//�˿�0״̬��¼,1:�˿���ɳ�ʼ��,2�˿��������(����������������) 
#define S_INIT		0x01	//�˿���ɳ�ʼ�� 
#define S_CONN		0x02	//�˿��������,���������������� 

/***************----- �˿��շ����ݵ�״̬ -----***************/
uint8_t S0_Data;		//�˿�0���պͷ������ݵ�״̬,1:�˿ڽ��յ�����,2:�˿ڷ���������� 
#define S_RECEIVE	 0x01	//�˿ڽ��յ�һ�����ݰ� 
#define S_TRANSMITOK 0x02	//�˿ڷ���һ�����ݰ���� 

/***************----- �˿����ݻ����� -----***************/
uint8_t Rx_Buffer[2048];	//�˿ڽ������ݻ����� 
uint8_t Tx_Buffer[2048];	//�˿ڷ������ݻ����� 

uint8_t W5500_Interrupt;	//W5500�жϱ�־(0:���ж�,1:���ж�)
extern int32_t sendstatus;
extern uint8_t DMA_is_Sending;//1:DMA���ڷ������ݣ�0��DMA��ɷ���
extern uint8_t DMA_is_ReSending;//1:DMA���ڷ��ͽ��յ����ݣ�0��DMA��ɽ��շ���
/*******************************************************************************
* ������  : W5500_Initialization
* ����    : W5500��ʼ������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void W5500_Initialization(void)
{
	W5500_Init();		//��ʼ��W5500�Ĵ�������
	Detect_Gateway();	//������ط����� 
	//Socket_Init(0);		
	if(S0_Mode==UDP_MODE)
	{
		socket(0,Sn_MR_UDP,5000,0);//Socket0�˿ڳ�ʼ����UDPģʽ���˿ں�Ϊ5000
	}
	else
	{
	  socket(0,Sn_MR_TCP,5000,0);//Socket0�˿ڳ�ʼ����TCPģʽ���˿ں�Ϊ5000
	}
	
	W5500_Socket_Set();//W5500�˿ڳ�ʼ������
}

/*******************************************************************************
* ������  : W5500_Init
* ����    : ��ʼ��W5500�Ĵ�������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��ʹ��W5500֮ǰ���ȶ�W5500��ʼ��
*******************************************************************************/
void W5500_Init(void)
{
	uint8_t i=0;

  setMR(MR_RST);//�����λW5500,��1��Ч,��λ���Զ���0
	HAL_Delay(10);//��ʱ10ms,�Լ�����ú���

	//��������(Gateway)��IP��ַ,Gateway_IPΪ4�ֽ�unsigned char����,�Լ����� 
	//ʹ�����ؿ���ʹͨ��ͻ�������ľ��ޣ�ͨ�����ؿ��Է��ʵ��������������Internet
	setGAR(Gateway_IP);

	//������������(MASK)ֵ,SUB_MASKΪ4�ֽ�unsigned char����,�Լ�����
	//��������������������
	setSUBR(Sub_Mask);		
	
	//���������ַ,PHY_ADDRΪ6�ֽ�unsigned char����,�Լ�����,����Ψһ��ʶ�����豸�������ֵַ
	//�õ�ֵַ��Ҫ��IEEE���룬����OUI�Ĺ涨��ǰ3���ֽ�Ϊ���̴��룬�������ֽ�Ϊ��Ʒ���
	//����Լ����������ַ��ע���һ���ֽڱ���Ϊż��
	setSHAR(Phy_Addr);		

	//���ñ�����IP��ַ,IP_ADDRΪ4�ֽ�unsigned char����,�Լ�����
	//ע�⣬����IP�����뱾��IP����ͬһ�����������򱾻����޷��ҵ�����
	setSIPR(IP_Addr);		
	
	//���÷��ͻ������ͽ��ջ������Ĵ�С����շ��ͻ������ͽ��ջ��������ο�W5500�����ֲ�
	for(i=0;i<8;i++)
	{
		setSn_RXBUF_SIZE(i, 0x02);//Socket Rx memory size=2k
		setSn_TXBUF_SIZE(i, 0x02);//Socket Tx mempry size=2k
	}

	//��������ʱ�䣬Ĭ��Ϊ2000(200ms) 
	//ÿһ��λ��ֵΪ100΢��,��ʼ��ʱֵ��Ϊ2000(0x07D0),����200����
	setRTR(0x07d0);

	//�������Դ�����Ĭ��Ϊ8�� 
	//����ط��Ĵ��������趨ֵ,�������ʱ�ж�(��صĶ˿��жϼĴ����е�Sn_IR ��ʱλ(TIMEOUT)�á�1��)
	setRCR(8);
	
	//���÷�Ƭ���ȣ��ο�W5500�����ֲᣬ��ֵ���Բ��޸�	
	setSn_MSSR(0, 0x5b4);//����Ƭ�ֽ���=1460(0x5b4)
	
	//�����жϣ��ο�W5500�����ֲ�ȷ���Լ���Ҫ���ж�����
	//������Socket�¼��жϣ�������Ҫ���
	setIMR(IM_IR7 | IM_IR6);//����IP��ͻ�жϣ�UDPĿ�ĵ�ַ���ִܵ�ر�PPPoE��Magic Packet�ж�	
	setSIMR(1);//����Socket 0�ж�
	setSn_IMR(0, IMR_SENDOK | IMR_TIMEOUT | IMR_RECV | IMR_DISCON | IMR_CON);
}

/*******************************************************************************
* ������  : Load_Net_Parameters
* ����    : װ���������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ���ء����롢�����ַ������IP��ַ���˿ںš�Ŀ��IP��ַ��Ŀ�Ķ˿ںš��˿ڹ���ģʽ
*******************************************************************************/
void Load_Net_Parameters(void)
{
	Gateway_IP[0] = 192;//�������ز���0xC0 0A8
	Gateway_IP[1] = 168;
	Gateway_IP[2] = 1;
	Gateway_IP[3] = 1;

	Sub_Mask[0]=255;//������������
	Sub_Mask[1]=255;
	Sub_Mask[2]=255;
	Sub_Mask[3]=0;

	Phy_Addr[0]=0x0c;//���������ַ
	Phy_Addr[1]=0x29;
	Phy_Addr[2]=0xab;
	Phy_Addr[3]=0x7c;
	Phy_Addr[4]=0x00;
	Phy_Addr[5]=0x01;

	IP_Addr[0]=192;//���ر���IP��ַ
	IP_Addr[1]=168;
	IP_Addr[2]=1;
	IP_Addr[3]=10;

	S0_Port = 5000;//���ض˿�0�Ķ˿ں�5000(0x1388) 

	UDP_DIPR[0] = 192;	//UDP(�㲥)ģʽ,Ŀ������IP��ַ
	UDP_DIPR[1] = 168;
	UDP_DIPR[2] = 1;
	UDP_DIPR[3] = 101;

	UDP_DPORT = 6000;	//UDP(�㲥)ģʽ,Ŀ�������˿ں�

	//S0_Mode=UDP_MODE;//���ض˿�0�Ĺ���ģʽ,UDPģʽ
	S0_Mode=TCP_SERVER;//���ض˿�0�Ĺ���ģʽ,TCP�����ģʽ
	//S0_Mode=TCP_CLIENT;//���ض˿�0�Ĺ���ģʽ,TCP�ͻ���ģʽ
	
	if(S0_Mode==TCP_CLIENT)//TCP�ͻ���ģʽ��ҪĿ�Ķ˿ںź�IP��ַ
	{
		S0_DIP[0]=192;//���ض˿�0��Ŀ��IP��ַ
  	S0_DIP[1]=168;
	  S0_DIP[2]=1;
	  S0_DIP[3]=101;
	
	  S0_DPort = 6000;//���ض˿�0��Ŀ�Ķ˿ں�6000
	}
}

/*******************************************************************************
* ������  : Detect_Gateway
* ����    : ������ط�����
* ����    : ��
* ���    : ��
* ����ֵ  : �ɹ�����TRUE(0xFF),ʧ�ܷ���FALSE(0x00)
* ˵��    : ��
*******************************************************************************/
uint8_t Detect_Gateway(void)
{
	uint8_t ip_adde[4];
	ip_adde[0]=IP_Addr[0]+1;
	ip_adde[1]=IP_Addr[1]+1;
	ip_adde[2]=IP_Addr[2]+1;
	ip_adde[3]=IP_Addr[3]+1;

	//������ؼ���ȡ���ص������ַ
	setSn_DIPR(0,ip_adde);//��Ŀ�ĵ�ַ�Ĵ���д���뱾��IP��ͬ��IPֵ
	setSn_MR(0,Sn_MR_TCP);//����socket0ΪTCPģʽ
	setSn_CR(0,Sn_CR_OPEN);//��Socket0
	HAL_Delay(5);//��ʱ5ms 	
	
	if(getSn_SR(0) != SOCK_INIT)//���socket��ʧ��
	{
		setSn_CR(0,Sn_CR_CLOSE);//�򿪲��ɹ�,�ر�Socket0
		return FALSE;//����FALSE(0x00)
	}

	setSn_CR(0,Sn_CR_CONNECT);//����Socket0ΪConnectģʽ						

	do
	{
		uint8_t j=0;
		j=getSn_IR(0);//��ȡSocket0�жϱ�־�Ĵ���
		if(j!=0)
		setSn_IR(0,j);
		HAL_Delay(5);//��ʱ5ms 
		if((j&Sn_IR_TIMEOUT) == Sn_IR_TIMEOUT)
		{
			return FALSE;	
		}
		else if(WIZCHIP_READ(Sn_DHAR(0)) != 0xff)
		{
			setSn_CR(0,Sn_CR_CLOSE);//�ر�Socket
			return TRUE;							
		}
	}while(1);
}


/*******************************************************************************
* ������  : Socket_Init
* ����    : ָ��Socket(0~7)��ʼ��
* ����    : s:����ʼ���Ķ˿�
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void Socket_Init(uint8_t s)
{
	//���÷�Ƭ���ȣ��ο�W5500�����ֲᣬ��ֵ���Բ��޸�	
	setSn_MSSR(0, 0x5b4);//����Ƭ�ֽ���=1460(0x5b4)
	//����ָ���˿�
	switch(s)
	{
		case 0:
			//���ö˿�0�Ķ˿ں�
			setSn_PORT(0, S0_Port);
	    if(S0_Mode==TCP_CLIENT)//ֻ��TCP�ͻ���ģʽ��Ҫ����Ŀ�Ķ˿ںź�IP��ַ
	    {
		 	  //���ö˿�0Ŀ��(Զ��)�˿ں�
			  setSn_DPORT(0, S0_DPort);
			  //���ö˿�0Ŀ��(Զ��)IP��ַ
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
* ������  : W5500_Socket_Set
* ����    : W5500�˿ڳ�ʼ������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : �ֱ�����4���˿�,���ݶ˿ڹ���ģʽ,���˿�����TCP��������TCP�ͻ��˻�UDPģʽ.
*			�Ӷ˿�״̬�ֽ�Socket_State�����ж϶˿ڵĹ������
*******************************************************************************/
void W5500_Socket_Set(void)
{
	if(S0_State==0)//�˿�0��ʼ������
	{
		if(S0_Mode==TCP_SERVER)//TCP������ģʽ 
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
		else if(S0_Mode==TCP_CLIENT)//TCP�ͻ���ģʽ 
		{
			if(connect(0, S0_DIP, S0_DPort)==SOCK_OK) //socket0�������ӣ�Ŀ��ip��ַ��Ŀ�Ķ˿ں�
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else//UDPģʽ 
		{
			//if(Socket_UDP(0)==TRUE)
				S0_State=S_INIT|S_CONN;
			//else
			//	S0_State=0;
		}
	}
}

/*******************************************************************************
* ������  : Socket_Connect
* ����    : ����ָ��Socket(0~7)Ϊ�ͻ�����Զ�̷���������
* ����    : s:���趨�Ķ˿�
* ���    : ��
* ����ֵ  : �ɹ�����TRUE(0xFF),ʧ�ܷ���FALSE(0x00)
* ˵��    : ������Socket�����ڿͻ���ģʽʱ,���øó���,��Զ�̷�������������
*			����������Ӻ���ֳ�ʱ�жϣ��������������ʧ��,��Ҫ���µ��øó�������
*			�ó���ÿ����һ��,�������������һ������
*******************************************************************************/
uint8_t Socket_Connect(uint8_t s)
{
	setSn_MR(s,Sn_MR_TCP);//����socketΪTCPģʽ
	setSn_CR(s,Sn_CR_OPEN);//��Socket
	HAL_Delay(5);//��ʱ5ms
	if(getSn_CR(s)!=SOCK_INIT)//���socket��ʧ��
	{
		setSn_CR(s,Sn_CR_CLOSE);//�򿪲��ɹ�,�ر�Socket
		return FALSE;//����FALSE(0x00)
	}
	setSn_CR(s,Sn_CR_CONNECT);//����SocketΪConnectģʽ
	return TRUE;//����TRUE,���óɹ�
}

/*******************************************************************************
* ������  : Socket_Listen
* ����    : ����ָ��Socket(0~7)��Ϊ�������ȴ�Զ������������
* ����    : s:���趨�Ķ˿�
* ���    : ��
* ����ֵ  : �ɹ�����TRUE(0xFF),ʧ�ܷ���FALSE(0x00)
* ˵��    : ������Socket�����ڷ�����ģʽʱ,���øó���,�ȵ�Զ������������
*			�ó���ֻ����һ��,��ʹW5500����Ϊ������ģʽ
*******************************************************************************/
uint8_t Socket_Listen(uint8_t s)
{
	setSn_MR(s,Sn_MR_TCP);//����socketΪTCPģʽ 
	setSn_CR(s,Sn_CR_OPEN);//��Socket	
	HAL_Delay(5);//��ʱ5ms
	if(getSn_SR(s)!=SOCK_INIT)//���socket��ʧ��
	{
		setSn_CR(s,Sn_CR_CLOSE);//�򿪲��ɹ�,�ر�Socket
		return FALSE;//����FALSE(0x00)
	}	
	setSn_CR(s,Sn_CR_LISTEN);//����SocketΪ����ģʽ	
	HAL_Delay(5);//��ʱ5ms
	if(getSn_SR(s)!=SOCK_LISTEN)//���socket����ʧ��
	{
		setSn_CR(s,Sn_CR_CLOSE);//���ò��ɹ�,�ر�Socket
		return FALSE;//����FALSE(0x00)
	}

	return TRUE;

	//���������Socket�Ĵ򿪺�������������,����Զ�̿ͻ����Ƿ�������������,����Ҫ�ȴ�Socket�жϣ�
	//���ж�Socket�������Ƿ�ɹ����ο�W5500�����ֲ��Socket�ж�״̬
	//�ڷ���������ģʽ����Ҫ����Ŀ��IP��Ŀ�Ķ˿ں�
}

/*******************************************************************************
* ������  : Socket_UDP
* ����    : ����ָ��Socket(0~7)ΪUDPģʽ
* ����    : s:���趨�Ķ˿�
* ���    : ��
* ����ֵ  : �ɹ�����TRUE(0xFF),ʧ�ܷ���FALSE(0x00)
* ˵��    : ���Socket������UDPģʽ,���øó���,��UDPģʽ��,Socketͨ�Ų���Ҫ��������
*			�ó���ֻ����һ�Σ���ʹW5500����ΪUDPģʽ
*******************************************************************************/
uint8_t Socket_UDP(uint8_t s)
{
	setSn_MR(s,Sn_MR_UDP);//����SocketΪUDPģʽ*/
	setSn_CR(s,Sn_CR_OPEN);//��Socket*/
	HAL_Delay(5);//��ʱ5ms
	if(getSn_SR(s)!=SOCK_UDP)//���Socket��ʧ��
	{
		setSn_CR(s,Sn_CR_CLOSE);//�򿪲��ɹ�,�ر�Socket
		return FALSE;//����FALSE(0x00)
	}
	else
		return TRUE;

	//���������Socket�Ĵ򿪺�UDPģʽ����,������ģʽ��������Ҫ��Զ��������������
	//��ΪSocket����Ҫ��������,�����ڷ�������ǰ����������Ŀ������IP��Ŀ��Socket�Ķ˿ں�
	//���Ŀ������IP��Ŀ��Socket�Ķ˿ں��ǹ̶���,�����й�����û�иı�,��ôҲ��������������
}

/*******************************************************************************
* ������  : W5500_Interrupt_Process
* ����    : W5500�жϴ��������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void W5500_Interrupt_Process(void)
{
	uint8_t i,j;

IntDispose:
	W5500_Interrupt=0;//�����жϱ�־
	i = getIR();//��ȡ�жϱ�־�Ĵ���
	setIR(i);//����жϱ�־
	
	/*if((i & IR_CONFLICT) == IR_CONFLICT)//IP��ַ��ͻ�쳣����
	{
		 //�Լ���Ӵ���
	}
	if((i & IR_UNREACH) == IR_UNREACH)//UDPģʽ�µ�ַ�޷������쳣����
	{
		//�Լ���Ӵ���
	}*/
	
	i=getSIR();//��ȡ�˿��жϱ�־�Ĵ���	
	if((i & S0_INT) == S0_INT)//Socket0�¼����� 
	{
		j=getSn_IR(0);//��ȡSocket0�жϱ�־�Ĵ���
		setSn_IR(0,j);//Socket0����жϱ�־
		if(j&IR_CON)//��TCPģʽ��,Socket0�ɹ����� 
		{
			S0_State|=S_CONN;//��������״̬0x02,�˿�������ӣ�����������������
		}
		if(j&IR_DISCON)//��TCPģʽ��Socket�Ͽ����Ӵ���
		{
			if(S0_Mode==TCP_SERVER)
			{
				socket(0,Sn_MR_TCP,5000,0);//Socket0�˿ڳ�ʼ����TCPģʽ���˿ں�Ϊ5000
				S0_State=0;//��������״̬0x00,�˿�����ʧ��
				if(listen(0)==SOCK_OK)//�ٴ�����
				{
					S0_State=S_INIT;//�˿ڳ�ʼ���ɹ�
					DMA_is_Sending=0;//���ñ�־λ
					DMA_is_ReSending=0;//���ñ�־λ
				}
	  	}
			else if(S0_Mode==TCP_CLIENT)
			{
				socket(0,Sn_MR_TCP,5000,0);//Socket0�˿ڳ�ʼ����TCPģʽ���˿ں�Ϊ5000
				S0_State=0;//��������״̬0x00,�˿�����ʧ��
				while(connect(0, S0_DIP, S0_DPort))//�ٴ�����
				{
					S0_State=S_INIT;//�˿ڳ�ʼ���ɹ�
					DMA_is_Sending=0;//���ñ�־λ
					DMA_is_ReSending=0;//���ñ�־λ
				}
	  	}
			else
			{
			  socket(0,Sn_MR_UDP,5000,0);//Socket0�˿ڳ�ʼ����UDPģʽ���˿ں�Ϊ5000
				S0_State=S_INIT|S_CONN;//�˿ڳ�ʼ���ɹ�
				DMA_is_Sending=0;//���ñ�־λ
				DMA_is_ReSending=0;//���ñ�־λ
			}
		}
		if(j&IR_SEND_OK)//Socket0���ݷ������,�����ٴ����������������� 
		{
			S0_Data|=S_TRANSMITOK;//�˿ڷ���һ�����ݰ���� 
		}
		if(j&IR_RECV)//Socket���յ�����,�����������պ��� 
		{
			S0_Data = S_RECEIVE;//�˿ڽ��յ�һ�����ݰ�
		}
		if(j&IR_TIMEOUT)//Socket���ӻ����ݴ��䳬ʱ���� 
		{
			setSn_CR(0,Sn_CR_CLOSE);// �رն˿�,�ȴ����´����� 
			S0_State=0;//��������״̬0x00,�˿�����ʧ��
		}
	}

	if(getSIR() != 0) 
		goto IntDispose;
}
/*void W5500_Interrupt_Process(void)
{
	uint8_t i,j;

IntDispose:

	i=getSIR();//��ȡ�˿��жϱ�־�Ĵ���	
	if((i & S0_INT) == S0_INT)//Socket0�¼����� 
	{
		j=getSn_IR(0);//��ȡSocket0�жϱ�־�Ĵ���
		setSn_IR(0,j);
		dsf=j;
		if(j&IR_CON)//��TCPģʽ��,Socket0�ɹ����� 
		{
			S0_State|=S_CONN;//��������״̬0x02,�˿�������ӣ�����������������
		}
		if(j&IR_DISCON)//��TCPģʽ��Socket�Ͽ����Ӵ���
		{
			setSn_CR(0,Sn_CR_CLOSE);//�رն˿�,�ȴ����´����� 
			Socket_Init(0);		//ָ��Socket(0~7)��ʼ��,��ʼ���˿�0
			S0_State=0;//��������״̬0x00,�˿�����ʧ��
		}
		if(j&IR_SEND_OK)//Socket0���ݷ������,�����ٴ�����S_tx_process()������������ 
		{
			S0_Data|=S_TRANSMITOK;//�˿ڷ���һ�����ݰ���� 
		}
		if(j&IR_RECV)//Socket���յ�����,��������S_rx_process()���� 
		{
			S0_Data|=S_RECEIVE;//�˿ڽ��յ�һ�����ݰ�
		}
		if(j&IR_TIMEOUT)//Socket���ӻ����ݴ��䳬ʱ���� 
		{
			setSn_CR(0,Sn_CR_CLOSE);// �رն˿�,�ȴ����´����� 			
			S0_State=0;//��������״̬0x00,�˿�����ʧ��
		}
	}

	if(getSIR() != 0) 
		goto IntDispose;
}*/

/*******************************************************************************
* ������  : Process_Socket_Data
* ����    : W5500���ղ����ͽ��յ�������
* ����    : s:�˿ں�
* ���    : ��
* ����ֵ  : ��
* ˵��    : �������ȵ���S_rx_process()��W5500�Ķ˿ڽ������ݻ�������ȡ����,
*			Ȼ�󽫶�ȡ�����ݴ�Rx_Buffer������Temp_Buffer���������д���
*			������ϣ������ݴ�Temp_Buffer������Tx_Buffer��������
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
* ������  : W5500_RST
* ����    : W5500_RST���ų�ʼ������(PA3)
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : �͵�ƽ��Ч���͵�ƽ���ٱ���500us����
*******************************************************************************/
void W5500_RST(void)
{
	LL_GPIO_ResetOutputPin(W5500_nRST_GPIO_Port,W5500_nRST_Pin);//��λ��������
	HAL_Delay(50);
	LL_GPIO_SetOutputPin(W5500_nRST_GPIO_Port,W5500_nRST_Pin);//��λ��������
	HAL_Delay(100);
	while((WIZCHIP_READ(PHYCFGR)&PHYCFGR_LNK_ON)==0);//�ȴ���̫���������
}

