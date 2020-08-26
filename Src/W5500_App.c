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
NETWORKParam_t net_param,*Pnet_param;    		//��������
SOCKETnParam_t s0_param,*Ps0_param;   			//Socket0����
SOCKETnState_t s0_state,*Ps0_state;   			//Socket0״̬

uint8_t Rx_Buffer[2048];										//�������ݻ����� 
uint8_t Tx_Buffer[2048];										//�������ݻ����� 

uint8_t W5500_Interrupt;										//W5500�жϱ�־ 0-�ޣ�1-���ж�

 /*******************************************************************************
 * EXTERNAL VARIABLES
 */

extern int32_t Transferstatus;
extern uint8_t DMA_TX_Transfer_flag;		//1 - DMA���ڴ��䷢�������ݣ�0 - �������
extern uint8_t DMA_RX_Transfer_flag;		//1 - DMA���ڴ�����������ݣ�0 - �������

/*******************************************************************************
 * FUNCTIONS
 */
 
/*******************************************************************************
* ������  : W5500_Init
*
* ����    : W5500��ʼ��
*
* ����    : ��
*
* ����ֵ  : ��
*
* ˵��    : ��
*******************************************************************************/
void W5500_Init(void)
{
	Pnet_param =&net_param;
	Ps0_param =&s0_param;
	Ps0_state =&s0_state;
	
	W5500_Config();			//��ʼ��W5500�Ĵ���
	Detect_Gateway();		//������ط����� 
	
	if(Ps0_state->Sn_Mode == UDP_MODE)
	{
		socket(0,Sn_MR_UDP,5000,0);//����Socket0��UDPģʽ���˿ں�5000
	}
	else
	{
	  socket(0,Sn_MR_TCP,5000,0);//����Socket0��TCPģʽ���˿ں�5000
	}
	
	W5500_Socket_Set();		//W5500��ʼ������
}

/*******************************************************************************
* ������  : W5500_Config
*
* ����    : ��ʼ��W5500�Ĵ���
*
* ����    : ��
*
* ����ֵ  : ��
*
* ˵��    : W5500Ӳ����λ�����ȵ��ñ�����
*******************************************************************************/
void W5500_Config(void)
{
	uint8_t i=0;

  setMR(MR_RST);	//�����λW5500,��1��Ч,��λ���Զ���0
	HAL_Delay(10);	//��ʱ10ms

	//��������(Gateway)��IP��ַ
	setGAR(Pnet_param->Gateway_IP);

	//������������(MASK)����������������������
	setSUBR(Pnet_param->Sub_Mask);		
	
	//����MAC(����)��ַ,����Ψһ��ʶ�����豸�������ֵַ
	//����OUI�Ĺ涨��ǰ3���ֽ�Ϊ���̴��룬�������ֽ�Ϊ��Ʒ���
	//���Լ�����MAC��ַ��ע���һ���ֽڱ���Ϊż��
	setSHAR(Pnet_param->Phy_Addr);		

	//����Դ��������IP��ַ
	//ע�⣺����IP�����뱾��IP����ͬһ�����������򱾻����޷��ҵ�����
	setSIPR(Pnet_param->IP_Addr);		
	
	//���÷��ͻ������ͽ��ջ������Ĵ�С����շ��ͻ������ͽ��ջ�������Ĭ��2K
	for(i=0;i<8;i++)
	{
		setSn_RXBUF_SIZE(i, 0x02);//Socket Rx memory size=2k
		setSn_TXBUF_SIZE(i, 0x02);//Socket Tx mempry size=2k
	}

	//��������ʱ�䣬Ĭ��2000(200ms) 
	//ÿһ��λ��ֵΪ100΢��,��ʼ��ʱֵ��Ϊ2000(0x07D0),��200ms
	setRTR(0x07d0);

	//�������Դ�����Ĭ��Ϊ8�� 
	//����ط��Ĵ��������趨ֵ,�������ʱ�ж�(��صĶ˿��жϼĴ����е�Sn_IR ��ʱλ(TIMEOUT)�á�1��)
	setRCR(8);
	
	//���÷�Ƭ����
	setSn_MSSR(0, 0x5b4);//����Ƭ�ֽ���=1460(0x5b4)
	
	//�����ж�
	setIMR(IM_IR7 | IM_IR6);//����IP��ͻ�жϣ�UDPĿ�ĵ�ַ���ִܵ��жϣ��ر�PPPoE��Magic Packet�ж�	
	setSIMR(1);//����Socket 0�ж�(��@ref SIMR bit1��1��ʹ��Socket 0�ж�)
	setSn_IMR(0, IMR_SENDOK | IMR_TIMEOUT | IMR_RECV | IMR_DISCON | IMR_CON);
}


/*******************************************************************************
* ������  : Load_Net_Parameters
*
* ����    : �����������
*
* ����    : ��
*
* ����ֵ  : ��
*
* ˵��    : ��W5500 ��ʼ��ǰ���ñ�������Ԥ��װ�ؼĴ�������ֵ
*						@Gateway_IP		����IP��ַ 					32bit				192.168.1.1
*						@Sub_Mask			�������� 						32bit       255.255.255.0
*						@Phy_Addr			MAC��ַ							48bit       0c-29-ab-7c-00-01
*						@IP_Addr			ԴIP��ַ						32bit       192.168.1.10
*						@Sn_Port			Socket 0�˿ںţ�Դ�˿ںţ�			5000 (default)
*						@Sn_DIP				TCPģʽĿ��IP��ַ		32bit				192.128.1.101(client)
*						@UDP_DPORT		UDPģʽĿ�Ķ˿ں�								6000 (defualt)
*						@UDP_DIPR     UDPģʽĿ��IP��ַ		32bit				192.128.1.101
*******************************************************************************/

void Load_Net_Parameters(void)
{
	//�������ز���		
	Pnet_param->Gateway_IP[0] = 192;
	Pnet_param->Gateway_IP[1] = 168;
	Pnet_param->Gateway_IP[2] = 1;
	Pnet_param->Gateway_IP[3] = 1;
	
	//������������
	Pnet_param->Sub_Mask[0] = 255;
	Pnet_param->Sub_Mask[1] = 255;
	Pnet_param->Sub_Mask[2] = 255;
	Pnet_param->Sub_Mask[3] = 0;
	
	//����MAC��ַ
	Pnet_param->Phy_Addr[0] = 0x0c;
	Pnet_param->Phy_Addr[1] = 0x29;
	Pnet_param->Phy_Addr[2] = 0xab;
	Pnet_param->Phy_Addr[3] = 0x7c;
	Pnet_param->Phy_Addr[4] = 0x00;
	Pnet_param->Phy_Addr[5] = 0x01;
	
	//����Դ/����IP��ַ
	Pnet_param->IP_Addr[0] = 192;
	Pnet_param->IP_Addr[1] = 168;
	Pnet_param->IP_Addr[2] = 1;
	Pnet_param->IP_Addr[3] = 10;
	
	//����Socket 0�Ķ˿ں�/Դ�˿ں� 5000(0x1388) ��default��
	Ps0_param->Sn_Port = 5000;

	//UDP(�㲥)ģʽ,Ŀ������IP��ַ
	Ps0_param->UDP_DIPR[0] = 192;	
	Ps0_param->UDP_DIPR[1] = 168;
	Ps0_param->UDP_DIPR[2] = 1;
	Ps0_param->UDP_DIPR[3] = 101;
	
	//UDP(�㲥)ģʽ,Ŀ�������˿ں� 6000��default��
	Ps0_param->UDP_DPORT = 6000;	
	
	// ����Socket 0�Ĺ���ģʽ����������default��
	// @ref UDP_MODE/TCP_CLIENT/TCP_SERVER
	Ps0_state->Sn_Mode=TCP_SERVER;	
	
	//TCP�ͻ���ģʽ����Ҫ����Ŀ�Ķ˿ںź�IP��ַ
	if(Ps0_state->Sn_Mode==TCP_CLIENT)
	{
		//����Socket 0��Ŀ��IP��ַ
		Ps0_param->Sn_DIP[0]=192;
  	Ps0_param->Sn_DIP[1]=168;
	  Ps0_param->Sn_DIP[2]=1;
	  Ps0_param->Sn_DIP[3]=101;
		
		//����Socket 0��Ŀ�Ķ˿ں� 6000��default��
	  Ps0_param->Sn_DPort = 6000;
	}
}

/*******************************************************************************
* ������  : Detect_Gateway
*
* ����    : ������ط�����
*
* ����    : ��
*
* ����ֵ  : @TRUE/0xff 	- ����
*						@FALSE/0x00 - �쳣
*
* ˵��    : ��
*******************************************************************************/
uint8_t Detect_Gateway(void)
{
	uint8_t ip_adde[4];
	
	// ������ģʽ�£�����Ŀ��IP��ַ
	ip_adde[0]=Pnet_param->IP_Addr[0]+1;
	ip_adde[1]=Pnet_param->IP_Addr[1]+1;
	ip_adde[2]=Pnet_param->IP_Addr[2]+1;
	ip_adde[3]=Pnet_param->IP_Addr[3]+1;

	//������ؼ���ȡ���ص������ַ
	setSn_DIPR(0,ip_adde);//��Ŀ��IP��ַ�Ĵ���д�빹���Ŀ��IP��ַ
	setSn_MR(0,Sn_MR_TCP);//����Socket 0ΪTCPģʽ
	setSn_CR(0,Sn_CR_OPEN);//��Socket 0
	HAL_Delay(5);//��ʱ5ms 	
	
	//��socket
	if(getSn_SR(0) != SOCK_INIT)  
	{
		setSn_CR(0,Sn_CR_CLOSE);//��ʧ�ܣ���ر�Socket 0
		return FALSE;
	}

	//����TCP����
	setSn_CR(0,Sn_CR_CONNECT);//����Socket0ΪConnectģʽ						
	
	do
	{
		uint8_t flag=0;
		
		flag=getSn_IR(0);//��ȡSocket0�жϱ�־�Ĵ���
		
		if(flag!=0)
			setSn_IR(0,flag); //�����жϼĴ���
		
		HAL_Delay(5);//��ʱ5ms 
		
		if((flag & Sn_IR_TIMEOUT) == Sn_IR_TIMEOUT) //��ʱ
		{
			return FALSE;	
		}
		else if( WIZCHIP_READ(Sn_DHAR(0)) != 0xff)  
		{
			//Socket 0Ŀ��MAC��ַ�Ĵ�����Ĭ��ֵ����Ϊ��ȡ�ɹ����ѽ�������
			setSn_CR(0,Sn_CR_CLOSE); //�ر�Socket 0
			return TRUE;							
		}
	}while(1);
}

/*******************************************************************************
* ������  : Socket_Init
*
* ����    : ָ��Socket n(0~7)��ʼ��
*
* ����    : ��
*
* ����ֵ  : ��
*
* ˵��    : ��
*******************************************************************************/
void Socket_Init(uint8_t s)
{
	//���÷�Ƭ����	
	setSn_MSSR(0, 0x5b4); // ����Ƭ�ֽ���=1460(0x5b4)
	
	//����ָ��Socket
	switch(s)
	{
		case 0:
			
		// ����Socket 0�Ķ˿ں�
			setSn_PORT(0, Ps0_param->Sn_Port);
	  
		// Socket 0Ϊ�ͻ���ʱ������Ŀ�Ķ˿ڡ�Ŀ��IP��ַ 
			if(Ps0_state->Sn_Mode==TCP_CLIENT)
	    {
		 	  //����Ŀ�Ķ˿ں�
			  setSn_DPORT(0, Ps0_param->Sn_DPort);
			  //����Ŀ��IP��ַ
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
* ������  : W5500_Socket_Set
*
* ����    : W5500�˿ڳ�ʼ������
*
* ����    : ��
*
* ����ֵ  : ��
*
* ˵��    : ��
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
					DMA_TX_Transfer_flag=0;//���ñ�־λ
					DMA_RX_Transfer_flag=0;//���ñ�־λ
				}
	  	}
			else if(S0_Mode==TCP_CLIENT)
			{
				socket(0,Sn_MR_TCP,5000,0);//Socket0�˿ڳ�ʼ����TCPģʽ���˿ں�Ϊ5000
				S0_State=0;//��������״̬0x00,�˿�����ʧ��
				while(connect(0, S0_DIP, S0_DPort))//�ٴ�����
				{
					S0_State=S_INIT;//�˿ڳ�ʼ���ɹ�
					DMA_TX_Transfer_flag=0;//���ñ�־λ
					DMA_RX_Transfer_flag=0;//���ñ�־λ
				}
	  	}
			else
			{
			  socket(0,Sn_MR_UDP,5000,0);//Socket0�˿ڳ�ʼ����UDPģʽ���˿ں�Ϊ5000
				S0_State=S_INIT|S_CONN;//�˿ڳ�ʼ���ɹ�
				DMA_TX_Transfer_flag=0;//���ñ�־λ
				DMA_RX_Transfer_flag=0;//���ñ�־λ
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

