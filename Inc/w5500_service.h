#ifndef  _W5500_SERVICE_H_
#define  _W5500_SERVICE_H_

/*********************************************************************
 * INCLUDES
 */
#include "stdint.h"

/*********************************************************************
 * TYPEDEFS
 */
/* �������ò��� */    		//8.26 ��ַ�������� debug
typedef struct
{
	uint8_t Gateway_IP[4];	//����IP��ַ 
	uint8_t Sub_Mask[4];		//�������� 
	uint8_t IP_Addr[4];			//ԴIP��ַ��������IP��ַ
	uint8_t Phy_Addr[6];		//ԴMAC��ַ��������IP��ַ 	

}NETWORKParam_t;

/* Socket n���ò��� */
typedef struct
{
	uint16_t Sn_Port;				//Socket nԴ�˿ں�
		
	//����ΪUDPģʽʱ����
	uint16_t UDP_DPORT;			//Socket nĿ�Ķ˿ں�
	uint8_t  UDP_DIPR[4];		//Socket nĿ��IP��ַ
	uint8_t  _UDP_DIPR_[2]; //�ֽڶ���

}SOCKETnParam_t;

/*********************************************************************
 * Macros
 */
#define TRUE								0x00
#define FALSE								0xff

// �˿�״̬
#define Sn_OPEN							0x00	//!< �˿��Ѵ�
#define Sn_LISTEN						0x01	//!< �˿����ڼ���
#define Sn_CLOSED						0x02	//!< �˿��ѹر�
#define TCP_RECV						0x03	//!< TCP�˿ڽ���һ֡
#define TCP_SEND						0x04	//!< TCP�˿ڷ������
#define UDP_RECV						0x05	//!< UDP�˿ڽ���һ֡
#define UDP_SEND						0x06	//!< UDP�˿ڷ������

// ͨѶ�շ�����������
#define TCP_Rx_Buff_Size			16
#define TCP_Tx_Buff_Size			100
#define	UDP_TrgRx_Buff_Size		16
#define	UDP_TrgTx_Buff_Size		16

#ifdef Dev_Ch32 
#define UDPD_Tx_Buff_Size		1173		//!< ����֡ͷ��23 + ������10 x��������ͷ��7 + (����ͨ��״̬3 + ��ͨ��8 x ÿͨ�������ֽ���3��x ͨ������4)�ֽ�
#endif
#ifdef Dev_Ch24 
#define UDPD_Tx_Buff_Size		903			//!< ����֡ͷ��23 + ������10 x��������ͷ��7 + (����ͨ��״̬3 + ��ͨ��8 x ÿͨ�������ֽ���3��x ͨ������3)�ֽ�
#endif
#ifdef Dev_Ch16 
#define UDPD_Tx_Buff_Size		633			//!< ����֡ͷ��23 + ������10 x��������ͷ��7 + (����ͨ��״̬3 + ��ͨ��8 x ÿͨ�������ֽ���3��x ͨ������2)�ֽ�
#endif
#ifdef Dev_Ch8 
#define UDPD_Tx_Buff_Size		363			//!< ����֡ͷ��23 + ������10 x��������ͷ��7 + (����ͨ��״̬3 + ��ͨ��8 x ÿͨ�������ֽ���3��x ͨ������1)�ֽ�
#endif

/***********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8_t TCP_Rx_Buff[TCP_Rx_Buff_Size];				//TCP���ջ����� 
extern uint8_t TCP_Tx_Buff[TCP_Tx_Buff_Size];				//TCP���ͻ�����
extern uint8_t UDP_DTx_Buff[UDPD_Tx_Buff_Size];			//UDP���ݷ��ͻ�����
extern uint8_t UDP_EvtRx_Buff[UDP_TrgRx_Buff_Size];	//UDP�¼����ͻ�����
extern uint8_t UDP_EvtTx_Buff[UDP_TrgTx_Buff_Size];	//UDP�¼����ջ�����
extern uint8_t *pUDP_DTx_Buff;		
extern NETWORKParam_t net_param,*Pnet_param;
extern SOCKETnParam_t  sn_param[3],*Psn_param;

/***********************************************************************
 * FUNCTIONS
 */
void W5500_Init(void);

uint8_t TCPServer_Service(uint8_t sn , uint16_t Procesflag);
uint8_t UDP_Service(uint8_t sn, uint16_t Procesflag);

#endif   // _W5500_SERVICE_H_
