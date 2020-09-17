/**
 * @file    w5500_service.c
 * @author  gjmsilly
 * @brief   W5500���� - ��̫��ͨѶ����
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
#include "w5500_service.h"
#include "main.h"
#include "w5500.h"
#include "wizchip_conf.h"
#include "socket.h"

/******************************************************************************
 * GLOBAL VARIABLES
 */  
uint8_t TCP_Rx_Buff[TCP_Rx_Buff_Size];		//!< TCP�������ݻ����� 
uint8_t TCP_Tx_Buff[TCP_Tx_Buff_Size];		//!< TCP�������ݻ�����
uint8_t UDP_Tx_Buff[UDP_Tx_Buff_Size];		//!< UDP�������ݻ�����
uint8_t *pUDP_Tx_Buff;										//!< UDP��������ָ֡��

 /******************************************************************************
 * LOCAL VARIABLES
 */
static NETWORKParam_t net_param,*Pnet_param;    		//�����������
static SOCKETnParam_t sn_param[2],*Psn_param;   		//Socket n��������(n=0,1)
																																							 
 /******************************************************************************
 * LOCAL FUNCTIONS
 */
static void W5500_Load_Net_Parameters(void);
static void W5500_RST(void);
static void W5500_Config(void);
static uint8_t Detect_Gateway(void);
static void W5500_Socket_Init(uint8_t sn);

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
	W5500_Load_Net_Parameters();	//װ���������	
	W5500_RST();									//Ӳ����λ
	
	W5500_Config();								//��ʼ��W5500ͨ�üĴ�����
	Detect_Gateway();							//������ط����� 
	W5500_Socket_Init(0);					//Socket 0���� - TCP 
	W5500_Socket_Init(1);     		//Socket 1���� - UDP 
	
	pUDP_Tx_Buff = UDP_Tx_Buff;		//UDP���ͻ���������ָ��
}

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
static void W5500_RST(void)
{
	LL_GPIO_ResetOutputPin(W5500_nRST_GPIO_Port,W5500_nRST_Pin);//��λ��������
	HAL_Delay(50);
	LL_GPIO_SetOutputPin(W5500_nRST_GPIO_Port,W5500_nRST_Pin);//��λ��������
	HAL_Delay(100);
	while((WIZCHIP_READ(PHYCFGR)&PHYCFGR_LNK_ON)==0);//�ȴ���̫���������

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
static void W5500_Config(void)
{

  setMR(MR_RST);	//������λW5500,��1��Ч,��λ���Զ���0
	HAL_Delay(10);	//��ʱ10ms


	//��������(Gateway)��IP��ַ
	setGAR(Pnet_param->Gateway_IP);

	//������������(MASK)����������������������
	setSUBR(Pnet_param->Sub_Mask);		
	
	//����MAC(����)��ַ,����Ψһ��ʶ�����豸��������ֵַ
	//����OUI�Ĺ涨��ǰ3���ֽ�Ϊ���̴��룬�������ֽ�Ϊ��Ʒ���
	//���Լ�����MAC��ַ��ע���һ���ֽڱ���Ϊż��
	setSHAR(Pnet_param->Phy_Addr);		

	//����Դ��������IP��ַ
	//ע�⣺����IP�����뱾��IP����ͬһ�����������򱾻����޷��ҵ�����
	setSIPR(Pnet_param->IP_Addr);		

	//��������ʱ�䣬Ĭ��2000(200ms) 
	//ÿһ��λ��ֵΪ100΢��,��ʼ��ʱֵ��Ϊ4000,��400ms
	setRTR(4000);

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
*														- Socket 0													7001 (default)
*														- Socket 1													7002 (default)
*						@Sn_Mode			����ģʽ											
*														- Socket 0                     			TCP ������
*														- Socket 1													UDP
*						@Sn_DIP				Ŀ������IP��ַ						32bit			 ��TCP clientʱ���ã� 
*						@UDP_DPORT		Ŀ�������˿ں�												
*														- Socket 1													7002 (defualt)
*						@UDP_DIPR     Ŀ������IP��ַ						
*														- Socket 1              32bit				192.168.1.101
*******************************************************************************/
static void W5500_Load_Net_Parameters(void)
{
	Pnet_param =&net_param;
	Psn_param = sn_param;
	uint8_t *Plen;
	
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
																 
	//����MAC��ַ��default ������
	Pnet_param->Phy_Addr[0] = 0x0c;
	Pnet_param->Phy_Addr[1] = 0x29;
	Pnet_param->Phy_Addr[2] = 0xab;
	Pnet_param->Phy_Addr[3] = 0x7c;
	Pnet_param->Phy_Addr[4] = 0x00;
	Pnet_param->Phy_Addr[5] = 0x01;

	//����Դ/����IP��ַ��default ������
	Pnet_param->IP_Addr[0] = 192;
	Pnet_param->IP_Addr[1] = 168;
	Pnet_param->IP_Addr[2] = 1;
	Pnet_param->IP_Addr[3] = 10;	

	/* Socket 0 ���� */
	{				
		//����Socket 0�Ķ˿ں�: 7001 ��default ������
		Psn_param->Sn_Port = 7001;
	}
	
	/* Socket 1 ���� */	
	{		
		//����Socket 1�Ķ˿ں�: 7002 ��default ������
		(Psn_param+1)->Sn_Port = 7002;

		//UDP(�㲥)ģʽ������Ŀ������IP��ַ
		(Psn_param+1)->UDP_DIPR[0] = 192;	
		(Psn_param+1)->UDP_DIPR[1] = 168;
		(Psn_param+1)->UDP_DIPR[2] = 1;
		(Psn_param+1)->UDP_DIPR[3] = 101;

		//UDP(�㲥)ģʽ������Ŀ�������˿ں� 7002��default ��λ�����޸ģ�
		(Psn_param+1)->UDP_DPORT = 5555;
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
static uint8_t Detect_Gateway(void)
{
	uint8_t ip_adde[4];
	
	// ������ģʽ�£�����Ŀ��IP��ַ
	ip_adde[0]=Pnet_param->IP_Addr[0]+1;
	ip_adde[1]=Pnet_param->IP_Addr[1]+1;
	ip_adde[2]=Pnet_param->IP_Addr[2]+1;
	ip_adde[3]=Pnet_param->IP_Addr[3]+1;

	//������ؼ���ȡ���ص�������ַ
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
static void W5500_Socket_Init(uint8_t sn)
{
	
	/* Socket �Ĵ��������� */
	
	////���÷��ͻ������ͽ��ջ������Ĵ�С��Ĭ��2K(Ĭ��ֵ����������)
	//	for(uint8_t i=0;i<8;i++)
	//	{
	//		setSn_RXBUF_SIZE(i, 0x02);//Socket Rx memory size=2k
	//		setSn_TXBUF_SIZE(i, 0x02);//Socket Tx mempry size=2k
	//	}
	
	// �������е�Socket�жϣ�������ѯ�ķ�ʽ
	setSIMR(1);
	
	//����ָ��Socket
	switch(sn)
	{
		case 0:
			/* Socket 0 */	
			//���÷�Ƭ����
			setSn_MSSR(0, 0x5b4);//����Ƭ�ֽ���=1460(0x5b4)	
			//���ù���ģʽ TCP server����Socket 0���󶨶˿�
			while(socket(0,Sn_MR_TCP,Psn_param->Sn_Port,0) != 0);
			//�����������Զ����ͼ������λʱ��Ϊ5s	
			setSn_KPALVTR(0,2);    	
			break;
		
		case 1:
			/* Socket 1 */			
			//���÷�Ƭ����
			setSn_MSSR(1, 0x5b4);//����Ƭ�ֽ���=1460(0x5b4)			
		  //���ù���ģʽ UDP����Socket 1���󶨶˿�
			while(socket(1,Sn_MR_UDP,(Psn_param+1)->Sn_Port,0) != 1);			
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
* ������  : TCPServer_Service
*
* ����    : TCP����������: ��ȡ��ѯ�ķ�ʽ��ȡSocket n״̬�����TCP����
*
* ����    : @sn: Socket�Ĵ�����ţ�e.g. Socket 1 �� sn=1
*						@Procesflag��TCP���ݴ�����ɱ�־λ
*
* ����ֵ  : @Sn_OPEN - �˿��Ѵ�
*						@Sn_LISTEN - �˿����ڼ���
*						@Sn_CLOSE - �˿��ѹر�
*						@TCP_PROCESS - TCP�˿����ڴ�������
*						@TCP_COMPLETE - TCP�˿����ݴ������
*
* ˵��    : ���ñ�����ǰȷ��socket�Ѵ�- @ref W5500_Socket_Init
*******************************************************************************/
uint8_t TCPServer_Service(uint8_t sn , uint8_t Procesflag)
{
	uint16_t recvsize=0,sentsize = 0; // ���ڻػ�����
	uint8_t TCPserv_status; //!< TCP����״̬
	
	switch(getSn_SR(sn)) //����socket��״̬
	{
		/* Socket n �ر� */
		case SOCK_CLOSED:
			socket(sn,Sn_MR_TCP,(Psn_param+sn)->Sn_Port,0); //��Socket��TCPĬ�϶˿�
			setSn_KPALVTR(sn,2);    //�����������Զ����ͼ������λʱ��Ϊ5s
			
			TCPserv_status = Sn_OPEN;
		break;
		
		/* Socket n ����ɳ�ʼ�� */
		case SOCK_INIT:
			listen(sn); //��ʼ����
		
			TCPserv_status = Sn_LISTEN;
		break;
		
		/* Socket n ��������״̬ */
		case SOCK_ESTABLISHED:
		
			if(getSn_IR(sn) & Sn_IR_CON)
			{
				setSn_IR(sn,Sn_IR_CON); //���socket��־λ
			}	

			if((recvsize = getSn_RX_RSR(sn))>0) //����Ŀ������������TCP����			
			{
				DMA_recv(sn,TCP_Rx_Buff,recvsize);  //�ӽ��ջ�����ȫ����ȡ
				
				TCPserv_status = TCP_RECV;
			}
			
			/*	����TCP֡�������ֶ������Դ������յ�TCP����
					ͨ���ϲ�Ӧ�ô���Procesflag������TCP�ظ�			 */
			
			if( Procesflag & TCP_PROCESSCLP_EVT ) // ֡Э��������
			{			
				send(sn, TCP_Tx_Buff, TCP_Tx_Buff[1]+3); //TCP�ظ�Ŀ������
				memset(TCP_Rx_Buff,0xff,sizeof(TCP_Rx_Buff));//���TCP���ջ�����
				
				TCPserv_status = TCP_SEND;
			}
		
		break;
			
		/* Socket n �Ͽ����� */
		case SOCK_CLOSE_WAIT:
			close(sn);
			
			TCPserv_status = Sn_CLOSE;
		break;
	}
	
	return( TCPserv_status );
}

/*******************************************************************************
* ������  : UDP_Service
*
* ����    : UDP����: ��ȡ��ѯ�ķ�ʽ��ȡSocket n״̬�����UDP����
*
* ����    : @sn: Socket�Ĵ�����ţ�e.g. Socket 1 �� sn=1
*
* ����ֵ  : ��
*
* ˵��    : ���ñ�����ǰȷ��socket�Ѵ�- @ref W5500_Socket_Init
*******************************************************************************/
uint8_t UDP_Service(uint8_t sn)
{
	switch(getSn_SR(sn))
	{
		
		/* Socket n �ر� */
		case SOCK_CLOSED:
			socket(sn,Sn_MR_UDP,(Psn_param+sn)->Sn_DPort,0); //��Socket��UDPĬ�϶˿�
		break;
		
		/* Socket n ����ɳ�ʼ�� */
		case SOCK_UDP:
			
				sendto(sn, UDP_Tx_Buff,    				  \
							 sizeof(UDP_Tx_Buff), 				\
							 (Psn_param+sn)->UDP_DIPR,  	\
							 (Psn_param+sn)->UDP_DPORT);  // test:1024byte
							 
				 return SUCCESS;
		break;
		
	}
		
}