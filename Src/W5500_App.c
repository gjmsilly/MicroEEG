/**
 * @file    w5500_app.c
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
#include "w5500_app.h"
#include "w5500.h"
#include "wizchip_conf.h"
#include "socket.h"
#include "AttritubeTable.h"

/*******************************************************************************
 * GLOBAL VARIABLES
 */
NETWORKParam_t net_param,*Pnet_param;    		//�����������
SOCKETnParam_t sn_param[2],*Psn_param;   		//Socket n��������(n=0,1)

uint8_t Rx_Buffer[Rx_Buffer_Size];					//�������ݻ����� 
uint8_t Tx_Buffer[Tx_Buffer_Size];					//�������ݻ�����
uint8_t W5500_Send_flag = 0;                //1- ׼����W5500 socket n���ͻ�����д��


 /*******************************************************************************
 * EXTERNAL VARIABLES
 */


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
	Psn_param = sn_param;
	
	W5500_Config();						//��ʼ��W5500ͨ�üĴ�����
	Detect_Gateway();					//������ط����� 
	W5500_Socket_Init(0);			//Socket 0���� - TCP 
	W5500_Socket_Init(1);     //Socket 1���� - UDP 

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
	net_param.Phy_Addr[0] = *attr_tbl.COMM_Param.Dev_MAC.pValue;
	net_param.Phy_Addr[1] = *((uint8_t*)attr_tbl.COMM_Param.Dev_MAC.pValue+1);
	net_param.Phy_Addr[2] = *((uint8_t*)attr_tbl.COMM_Param.Dev_MAC.pValue+2);
	net_param.Phy_Addr[3] = *((uint8_t*)attr_tbl.COMM_Param.Dev_MAC.pValue+3);
	net_param.Phy_Addr[4] = *((uint8_t*)attr_tbl.COMM_Param.Dev_MAC.pValue+4);
	net_param.Phy_Addr[5] = *((uint8_t*)attr_tbl.COMM_Param.Dev_MAC.pValue+5);

	//����Դ/����IP��ַ
	net_param.IP_Addr[0] = *attr_tbl.COMM_Param.Dev_IP.pValue;
	net_param.IP_Addr[1] = *((uint8_t*)attr_tbl.COMM_Param.Dev_IP.pValue+1);
	net_param.IP_Addr[2] = *((uint8_t*)attr_tbl.COMM_Param.Dev_IP.pValue+2);
	net_param.IP_Addr[3] = *((uint8_t*)attr_tbl.COMM_Param.Dev_IP.pValue+3);

	/* Socket 0 ���� */
	{				
		//����Socket 0�Ķ˿ں�: 7001 ��default��
		sn_param[0].Sn_Port = 7001;
	}
	
	/* Socket 1 ���� */	
	{		
		//����Socket 1�Ķ˿ں�: 7002 ��default��
		sn_param[1].Sn_Port = 7002;

		//UDP(�㲥)ģʽ������Ŀ������IP��ַ
		sn_param[1].UDP_DIPR[0] = 192;	
		sn_param[1].UDP_DIPR[1] = 168;
		sn_param[1].UDP_DIPR[2] = 1;
		sn_param[1].UDP_DIPR[3] = 101;

		//UDP(�㲥)ģʽ������Ŀ�������˿ں� 7002��default��
		sn_param[1].UDP_DPORT = *attr_tbl.COMM_Param.Host_Port.pValue;	
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
	
	/* Socket �Ĵ��������� */
	
	////���÷��ͻ������ͽ��ջ������Ĵ�С��Ĭ��2K(Ĭ��ֵ����������)
	//	for(uint8_t i=0;i<8;i++)
	//	{
	//		setSn_RXBUF_SIZE(i, 0x02);//Socket Rx memory size=2k
	//		setSn_TXBUF_SIZE(i, 0x02);//Socket Tx mempry size=2k
	//	}
	
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
*
* ����ֵ  : ��
*
* ˵��    : ���ñ�����ǰȷ��socket�Ѵ�- @ref W5500_Socket_Init
*******************************************************************************/
void TCPServer_Service(uint8_t sn)
{
	uint16_t recvsize=0,sentsize = 0;
	uint32_t taddr;
	
	switch(getSn_SR(sn))
	{
		/* Socket n �ر� */
		case SOCK_CLOSED:
			socket(sn,Sn_MR_TCP,(Psn_param+sn)->Sn_Port,0); //��Socket��TCPĬ�϶˿�
			setSn_KPALVTR(sn,2);    //�����������Զ����ͼ������λʱ��Ϊ5s
		break;
		
		/* Socket n ����ɳ�ʼ�� */
		case SOCK_INIT:
			listen(sn); //��ʼ����
		break;
		
		/* Socket n ��������״̬ */
		case SOCK_ESTABLISHED:
			getSn_DIPR(0,((uint8_t*)&taddr)); //��ȡĿ��IP��ַ - debug
		
			if(getSn_IR(sn) & Sn_IR_CON)
			{
				setSn_IR(sn,Sn_IR_CON); //�����־λ
			}	
				//�ػ�����  - debug
				recvsize = getSn_RX_RSR(sn); //Socket n���ջ��������յ����ֽ���
				if(recvsize>0)				
				{	
					sentsize = recv(sn,Rx_Buffer,recvsize);  //�ӽ��ջ�����ȫ����ȡ
					DMA_send(sn, Rx_Buffer, sentsize); //�ػ�������д��Socket n���ͻ���������
				}
				  
				// ����Ŀ���������͵����� ��1�ֽڣ�
			
		break;
			
		/* Socket n �Ͽ����� */
		case SOCK_CLOSE_WAIT:
			close(sn);
		break;
	}		
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
void UDP_Service(uint8_t sn)
{
	switch(getSn_SR(sn))
	{
		
		/* Socket n �ر� */
		case SOCK_CLOSED:
			socket(sn,Sn_MR_UDP,(Psn_param+sn)->Sn_DPort,0); //��Socket��UDPĬ�϶˿�
		break;
		
		/* Socket n ����ɳ�ʼ�� */
		case SOCK_UDP:
			
			HAL_Delay(500);
			
			// ׼���������ݰ���Ŀ������
			///if(W5500_Send_flag)
			//{
				sendto(sn, Tx_Buffer,    				  \
							 sizeof(Tx_Buffer), 				\
							 (Psn_param+sn)->UDP_DIPR,  \
							 (Psn_param+sn)->UDP_DPORT);  // test:1024byte
			//}		
		break;
		
	}
		
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
void W5500_RST(void)
{
	LL_GPIO_ResetOutputPin(W5500_nRST_GPIO_Port,W5500_nRST_Pin);//��λ��������
	HAL_Delay(50);
	LL_GPIO_SetOutputPin(W5500_nRST_GPIO_Port,W5500_nRST_Pin);//��λ��������
	HAL_Delay(100);
	while((WIZCHIP_READ(PHYCFGR)&PHYCFGR_LNK_ON)==0);//�ȴ���̫���������

}

