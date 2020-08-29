/**
 * @file    W5500_APP.c
 * @author  gjmsilly
 * @brief   W5500Ӧ�ó�����ѯ��ʽ/����Socket 0 - TCP Socket 1 - UDP��
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
#include "main.h"
#include "W5500_App.h"
#include "W5500.h"
#include "wizchip_conf.h"
#include "socket.h"

/*******************************************************************************
 * GLOBAL VARIABLES
 */
NETWORKParam_t net_param,*Pnet_param;    		//�����������
SOCKETnParam_t s0_param,*Ps0_param;   			//Socket 0��������
SOCKETnParam_t s1_param,*Ps1_param;   			//Socket 1��������

uint8_t Rx_Buffer[2048];										//�������ݻ����� 
uint8_t Tx_Buffer[2048];										//�������ݻ�����
uint8_t W5500_Send_flag = 0;                //1- �������ݻ�����������׼��UDP�������ݰ�

#if(_W5500_MODE_ == Interrupt)
SOCKETnState_t s0_state,*Ps0_state;   			//Socket 0״̬
SOCKETnState_t s1_state,*Ps1_state;   			//Socket 1״̬
uint8_t W5500_Interrupt;										//W5500�жϱ�־ 0-�ޣ�1-���ж�
#endif

 /*******************************************************************************
 * EXTERNAL VARIABLES
 */

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
* ˵��    : ���W5500Ӳ����λ֮�����ȵ��ñ����� - @ref W5500_RST
*******************************************************************************/
void W5500_Init(void)
{
	Pnet_param =&net_param;
	Ps0_param =&s0_param;
	Ps1_param =&s1_param;
	
	W5500_Config();						//��ʼ��W5500ͨ�üĴ�����
	Detect_Gateway();					//������ط����� 
	//W5500_Socket_Init(0);			//Socket���ã�Socket 0 - TCP Socket 1 - UDP��
	W5500_Socket_Init(1);

}

/*******************************************************************************
* ������  : W5500_Config
*
* ����    : ��ʼ��W5500ͨ�üĴ���
*
* ����    : ��
*
* ����ֵ  : ��
*
* ˵��    : ���ñ�����ǰ��������������� - @ref W5500_Load_Net_Parameters
*******************************************************************************/
void W5500_Config(void)
{

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

	//��������ʱ�䣬Ĭ��2000(200ms) 
	//ÿһ��λ��ֵΪ100΢��,��ʼ��ʱֵ��Ϊ2000(0x07D0),��200ms
	setRTR(0x07d0);

	//�������Դ�����Ĭ��Ϊ8�� 
	//����ط��Ĵ��������趨ֵ,�������ʱ�ж�(��صĶ˿��жϼĴ����е�Sn_IR ��ʱλ(TIMEOUT)�á�1��)
	setRCR(8);

	//���������ж�
	setIMR(0);//����IP��ͻ�жϣ�UDPĿ�ĵ�ַ���ִܵ��жϣ�PPPoE��Magic Packet�ж�	

}

/*******************************************************************************
* ������  : W5500_Load_Net_Parameters
*
* ����    : �����������
*
* ����    : ��
*
* ����ֵ  : ��
*
* ˵��    : * ��W5500 ��ʼ��ǰ���ñ�������Ԥ��װ�ؼĴ�������ֵ *
*						@Gateway_IP		����IP��ַ 								32bit				192.168.1.1
*						@Sub_Mask			�������� 									32bit       255.255.255.0
*						@Phy_Addr			MAC��ַ										48bit       0c-29-ab-7c-00-01
*						@IP_Addr			Դ/����IP��ַ							32bit       192.168.1.10						
*						@Sn_Port			Դ�˿ں�
*														- Socket 0													5000 (default)
*														- Socket 1													5555 (default)
*						@Sn_Mode			����ģʽ											
*														- Socket 0                     			TCP ������
*														- Socket 1													UDP
*						@Sn_DIP				Ŀ������IP��ַ						32bit			 ��TCP clientʱ���ã� 
*						@UDP_DPORT		Ŀ�������˿ں�												
*														- Socket 1													6000 (defualt)
*						@UDP_DIPR     Ŀ������IP��ַ						
*														- Socket 1              32bit				192.168.1.101
*******************************************************************************/
void W5500_Load_Net_Parameters(void)
{
	//�������ز���		
	net_param.Gateway_IP[0] = 192;
	net_param.Gateway_IP[1] = 168;
	net_param.Gateway_IP[2] = 1;
	net_param.Gateway_IP[3] = 1;
	
	//������������
	net_param.Sub_Mask[0] = 255;
	net_param.Sub_Mask[1] = 255;
	net_param.Sub_Mask[2] = 255;
	net_param.Sub_Mask[3] = 0;
	
	//����MAC��ַ
	net_param.Phy_Addr[0] = 0x0c;
	net_param.Phy_Addr[1] = 0x29;
	net_param.Phy_Addr[2] = 0xab;
	net_param.Phy_Addr[3] = 0x7c;
	net_param.Phy_Addr[4] = 0x00;
	net_param.Phy_Addr[5] = 0x01;

	//����Դ/����IP��ַ
	net_param.IP_Addr[0] = 192;
	net_param.IP_Addr[1] = 168;
	net_param.IP_Addr[2] = 1;
	net_param.IP_Addr[3] = 10;

	/* Socket 0 ���� */
	{				
		//����Socket 0�Ķ˿ں�: 5000��default��
		s0_param.Sn_Port = 5000;
	}
	
	/* Socket 1 ���� */	
	{		
		//����Socket 1�Ķ˿ں�: 5555 ��default��
		s1_param.Sn_Port = 5555;

		//UDP(�㲥)ģʽ������Ŀ������IP��ַ
		s1_param.UDP_DIPR[0] = 192;	
		s1_param.UDP_DIPR[1] = 168;
		s1_param.UDP_DIPR[2] = 1;
		s1_param.UDP_DIPR[3] = 101;

		//UDP(�㲥)ģʽ������Ŀ�������˿ں� 6000��default��
		s1_param.UDP_DPORT = 6000;	
	}

}

/*******************************************************************************
* ������  : Detect_Gateway
*
* ����    : ������ط����� ��ʹ��Socket 0���ԣ�
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
	setSIMR(1);//����Socket 0�ж�(��@ref SIMR bit1��1��ʹ��Socket 0�ж�)
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
* ������  : W5500_Socket_Init
*
* ����    : ָ��Socket n(0~7)��ʼ��
*
* ����    : @sn: Socket�Ĵ�����ţ�e.g. Socket 1 �� sn=1
*
* ����ֵ  : ��
*
* ˵��    : ��
*******************************************************************************/
void W5500_Socket_Init(uint8_t sn)
{
 		uint16_t DPort=6000;
		uint32_t taddr;
	/* Socket �Ĵ��������� */
	
	//���÷��ͻ������ͽ��ջ������Ĵ�С����շ��ͻ������ͽ��ջ�������Ĭ��2K
	for(uint8_t i=0;i<8;i++)
	{
		setSn_RXBUF_SIZE(i, 0x02);//Socket Rx memory size=2k
		setSn_TXBUF_SIZE(i, 0x02);//Socket Tx mempry size=2k
	}
	// �������е�Socket�жϣ�������ѯ�ķ�ʽ @ref DO_TCP_Server
	setSIMR(1);
	
	//����ָ��Socket
	switch(sn)
	{
		case 0:
			/* Socket 0 */			
			//���÷�Ƭ����
			setSn_MSSR(0, 0x5b4);//����Ƭ�ֽ���=1460(0x5b4)			
			//���ù���ģʽ TCP server����Socket 0���󶨶˿�
			while(socket(0,Sn_MR_TCP,Ps0_param->Sn_Port,0) != 0);
			//�����������Զ����ͼ������λʱ��Ϊ5s	
			setSn_KPALVTR(0,2);    	
			break;
		
		case 1:
			/* Socket 1 */			
			//���÷�Ƭ����
			setSn_MSSR(1, 0x5b4);//����Ƭ�ֽ���=1460(0x5b4)			
		  //���ù���ģʽ UDP����Socket 1���󶨶˿�
			while(socket(1,Sn_MR_UDP,Ps1_param->Sn_Port,0) != 1);			
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
* ������  : DO_TCP_Server
*
* ����    : ��ȡ��ѯ�ķ�ʽ��ȡSocket n״̬�����TCP����
*
* ����    : @sn: Socket�Ĵ�����ţ�e.g. Socket 1 �� sn=1
*
* ����ֵ  : ��
*
* ˵��    : ������������- @ref W5500_Socket_Init�������ȷ��socket��
*******************************************************************************/
void DO_TCP_Server(uint8_t sn)
{
	uint16_t len;
	uint16_t recvsize=0,sentsize = 0;
		uint32_t taddr;
	
	switch(getSn_SR(sn))
	{
		/* Socket n �ر� */
		case SOCK_CLOSED:
			socket(sn,Sn_MR_TCP,5000,0); //��Socket��TCPĬ�϶˿�
			setSn_KPALVTR(sn,2);    //�����������Զ����ͼ������λʱ��Ϊ5s
		break;
		
		/* Socket n ����ɳ�ʼ�� */
		case SOCK_INIT:
			listen(sn); //��ʼ����
		break;
		
		/* Socket n ��������״̬ */
		case SOCK_ESTABLISHED:
			getSn_DIPR(0,((uint8_t*)&taddr));
		
			if(getSn_IR(sn) & Sn_IR_CON)
			{
				setSn_IR(sn,Sn_IR_CON); //�����־λ
			}	
				//�ػ�����
				len =getSn_RX_RSR(sn);
				if(len>0)				
				{	
					recvsize = DMA_recv(sn,Rx_Buffer,len);
				}

				 len = DMA_send(sn, Tx_Buffer, recvsize);

			
		break;
			
		/* Socket n �Ͽ����� */
		case SOCK_CLOSE_WAIT:
			close(sn);
		break;
	}		
}

/*******************************************************************************
* ������  : DO_UDP
*
* ����    : ��ȡ��ѯ�ķ�ʽ��ȡSocket n״̬�����UDP����
*
* ����    : @sn: Socket�Ĵ�����ţ�e.g. Socket 1 �� sn=1
*
* ����ֵ  : ��
*
* ˵��    : ������������- @ref W5500_Socket_Init�������ȷ��socket��
*******************************************************************************/
void DO_UDP(uint8_t sn)
{
	switch(getSn_SR(sn))
	{
		
		/* Socket n �ر� */
		case SOCK_CLOSED:
			socket(sn,Sn_MR_UDP,5555,0); //��Socket��UDPĬ�϶˿�
		break;
		
		/* Socket n ����ɳ�ʼ�� */
		case SOCK_UDP:
			
			HAL_Delay(500);
			
			// ׼���������ݰ���Ŀ������
			///if(W5500_Send_flag)
			//{
				sendto(sn,Tx_Buffer,1024,Ps1_param->UDP_DIPR,Ps1_param->UDP_DPORT);  // test:1024byte
			//}		
		break;
		
	}
		
}


#if(_W5500_MODE_ == Interrupt )
/*******************************************************************************
* ������  : W5500_Socket_State
*
* ����    : Socket����״̬��ʼ��
*
* ����    : @sn: Socket�Ĵ�����ţ�e.g. Socket 1 �� sn=1
*
* ����ֵ  : ��
*
* ˵��    : ������������- @ref W5500_Socket_Init�������ȷ��socket��
*******************************************************************************/
void W5500_Socket_State(uint8_t sn)
{
 	switch(sn)
	{
		case 0:
		if(Ps0_state->Sn_Mode == TCP_SERVER) //TCP������ģʽ 
		{
			if(listen(0)==SOCK_OK)  //socket n���ӳ�ʼ���ɹ�
			{
				Ps0_state->Sn_State = S_INIT;
			}
			else
			{
				Ps0_state->Sn_State = 0;
			}
		}
		else if(Ps0_state->Sn_Mode == TCP_CLIENT)// TCP�ͻ���ģʽ 
		{
			if(connect(0, Ps0_param->Sn_DIP, Ps0_param->Sn_DPort)==SOCK_OK) //socket n���ӳ�ʼ���ɹ�
				Ps0_state->Sn_State = S_INIT;
			else
				Ps0_state->Sn_State = 0;
		}
		else //UDPģʽ����������
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
* ������  : W5500_Interrupt_Process
*
* ����    : W5500�жϴ������
*
* ����    : ��
*
* ����ֵ  : ��
*
* ˵��    : ��
*******************************************************************************/
void W5500_Interrupt_Process(void)
{
	uint8_t flag_c,flag_s;

	IntDispose:
	W5500_Interrupt=0;//����жϱ�־
	flag_c = getIR();//��ȡ�жϱ�־�Ĵ���
	setIR(flag_c);	 //����жϱ�־
	
	flag_s=getSIR(); //��ȡSocket�жϱ�־�Ĵ���	
	
	if((flag_c & S0_INT) == S0_INT) //Socket n�ж��¼����� 
	{
		flag_s=getSn_IR(0);//��ȡSocket n�жϱ�־�Ĵ���
		setSn_IR(0,flag_s);//Socket0����жϱ�־
		
		if(flag_s & IR_CON)	//��TCPģʽ�£�Socket n�ɹ����� 
		{
			Ps0_state->Sn_State|=S_CONN; //����״̬Ϊ���ӳɹ������Դ�������
		}
		
		if(flag_s & IR_DISCON)//��TCPģʽ�£�Socket n�Ͽ�����
		{
			//TCP SERVER ģʽ
			if(Ps0_state->Sn_Mode==TCP_SERVER)
			{
				Ps0_state->Sn_State=0;//����״̬Ϊ����ʧ��
						
				socket(0,Sn_MR_TCP,Ps0_param->Sn_Port,0);//Socket n�˿ڳ�ʼ����TCPģʽ���˿ں�Ϊ5000
			
				if(listen(0)==SOCK_OK)//�ٴ������ɹ�
				{
					Ps0_state->Sn_State=S_INIT;//Socket n���ӳ�ʼ���ɹ�
					
					DMA_TX_Transfer_flag=0;//���ñ�־λ
					DMA_RX_Transfer_flag=0;//���ñ�־λ
				}
	  	}
			//TCP CLIENT ģʽ
			else if(Ps0_state->Sn_Mode==TCP_CLIENT)
			{
				Ps0_state->Sn_State=0;//����״̬Ϊ����ʧ��
				
				socket(0,Sn_MR_TCP,Ps0_param->Sn_Port,0);//Socket n��ʼ����TCPģʽ���˿ں�Ϊ5000
				
				while(connect(0, Ps0_param->Sn_DIP, Ps0_param->Sn_DPort))//�ٴ�����
				{
					Ps0_state->Sn_State=S_INIT;//Socket n���ӳ�ʼ���ɹ�
					
					DMA_TX_Transfer_flag=0;//���ñ�־λ
					DMA_RX_Transfer_flag=0;//���ñ�־λ
				}
	  	}
			//UDP ģʽ���������ӣ�Socket n�򿪳ɹ�����׼����������
			else	
			{
			  socket(0,Sn_MR_UDP,Ps0_param->Sn_Port,0);//Socket n��ʼ����UDPģʽ���˿ں�Ϊ5000
				Ps0_state->Sn_State=S_INIT|S_CONN;
				
				DMA_TX_Transfer_flag=0;//���ñ�־λ
				DMA_RX_Transfer_flag=0;//���ñ�־λ
			}
		}
		
		if(flag_s&IR_SEND_OK)//Socket n���ݷ������,�����ٴ����������������� 
		{
			Ps0_state->Sn_Data |=S_TRANSMITOK;//����״̬Ϊ�������
		}
		
		if(flag_s&IR_RECV)//Socket���յ�����,�����������պ��� 
		{
			Ps0_state->Sn_Data = S_RECEIVE;//����״̬Ϊ���յ�����
		}
		
		if(flag_s&IR_TIMEOUT)//Socket���ӻ����ݴ��䳬ʱ���� 
		{
			setSn_CR(0,Sn_CR_CLOSE);// �ر�Socket n,�ȴ����´����� 
			Ps0_state->Sn_State=0;//����״̬Ϊ����ʧ��
		}
	}

	if(getSIR() != 0) 
		goto IntDispose;
}
#endif

/*******************************************************************************
* ������  : W5500_RST
*
* ����    : W5500_RST���ų�ʼ������(PA3)
*
* ����    : ��
*
* ����ֵ  : ��
*
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

