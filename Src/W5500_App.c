
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
SOCKETnState_t s0_state,*Ps0_state;   			//Socket 0״̬

uint8_t Rx_Buffer[2048];										//�������ݻ����� 
uint8_t Tx_Buffer[2048];										//�������ݻ����� 

uint8_t W5500_Interrupt;										//W5500�жϱ�־ 0-�ޣ�1-���ж�

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
* ˵��    : �������W5500Ӳ����λ��֮�����ȵ��ñ�����
*******************************************************************************/
void W5500_Init(void)
{
	Pnet_param =&net_param;
	Ps0_param =&s0_param;
	Ps0_state =&s0_state;
	
	W5500_Config();						//��ʼ��W5500ͨ�üĴ�����
	Detect_Gateway();					//������ط����� 
	W5500_Socket_Init(0);			//Socket���ã�Ĭ��ʹ��Socket 0��
	W5500_Socket_State(0);		//Socket״̬����
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
* ˵��    : 
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
*						@Gateway_IP		����IP��ַ 					32bit				192.168.1.1
*						@Sub_Mask			�������� 						32bit       255.255.255.0
*						@Phy_Addr			MAC��ַ							48bit       0c-29-ab-7c-00-01
*						@IP_Addr			ԴIP��ַ						32bit       192.168.1.10
*						@Sn_Port			Socket 0�˿ںţ�Դ�˿ںţ�			5000 (default)
*						@Sn_DIP				TCPģʽĿ��IP��ַ		32bit				192.128.1.101(client)
*						@UDP_DPORT		UDPģʽĿ�Ķ˿ں�								6000 (defualt)
*						@UDP_DIPR     UDPģʽĿ��IP��ַ		32bit				192.168.1.101
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
	
	//����Socket 0�Ķ˿ں�/Դ�˿ں� 5000(0x1388) ��default��
	s0_param.Sn_Port = 5000;

	//UDP(�㲥)ģʽ,Ŀ������IP��ַ
	s0_param.UDP_DIPR[0] = 192;	
	s0_param.UDP_DIPR[1] = 168;
	s0_param.UDP_DIPR[2] = 1;
	s0_param.UDP_DIPR[3] = 101;
	
	//UDP(�㲥)ģʽ,Ŀ�������˿ں� 6000��default��
	s0_param.UDP_DPORT = 6000;	
	
	// ����Socket 0�Ĺ���ģʽ����������default��
	// @ref UDP_MODE/TCP_CLIENT/TCP_SERVER
	s0_state.Sn_Mode=TCP_SERVER;	
	
	//TCP�ͻ���ģʽ����Ҫ����Ŀ�Ķ˿ںź�IP��ַ
	if(s0_state.Sn_Mode==TCP_CLIENT)
	{
		//����Socket 0��Ŀ��IP��ַ
		s0_param.Sn_DIP[0]=192;
  	s0_param.Sn_DIP[1]=168;
	  s0_param.Sn_DIP[2]=1;
	  s0_param.Sn_DIP[3]=101;
		
		//����Socket 0��Ŀ�Ķ˿ں� 6000��default��
	  s0_param.Sn_DPort = 6000;
	}
}

/*******************************************************************************
* ������  : Detect_Gateway
*
* ����    : ������ط����� ��ʹ��Socket 0��
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
	
	//���÷��ͻ������ͽ��ջ������Ĵ�С����շ��ͻ������ͽ��ջ�������Ĭ��2K
	for(uint8_t i=0;i<8;i++)
	{
		setSn_RXBUF_SIZE(i, 0x02);//Socket Rx memory size=2k
		setSn_TXBUF_SIZE(i, 0x02);//Socket Tx mempry size=2k
	}
	
	//����ָ��Socket
	switch(sn)
	{
		case 0:
			/* Socket 0 */
		
			//���÷�Ƭ����
			setSn_MSSR(0, 0x5b4);//����Ƭ�ֽ���=1460(0x5b4)
			
			//�����ж�
			setIMR(IM_IR7 | IM_IR6);//����IP��ͻ�жϣ�UDPĿ�ĵ�ַ���ִܵ��жϣ��ر�PPPoE��Magic Packet�ж�	
			setSIMR(1);//����Socket 0�ж�(��@ref SIMR bit1��1��ʹ��Socket 0�ж�)
			setSn_IMR(0, IMR_SENDOK | IMR_TIMEOUT | IMR_RECV | IMR_DISCON | IMR_CON);
			
			//���ù���ģʽ����Socket 0
			if(Ps0_state->Sn_Mode == UDP_MODE)
			{
				socket(0,Sn_MR_UDP,Ps0_param->Sn_Port,0);//����Socket0��UDPģʽ��Դ�˿ں�5000
			}
			else
			{
				socket(0,Sn_MR_TCP,Ps0_param->Sn_Port,0);//����Socket0��TCPģʽ��Դ�˿ں�5000
			}
	  
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

