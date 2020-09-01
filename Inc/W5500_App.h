#ifndef  _W5500_App_H_
#define  _W5500_App_H_

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
	uint16_t Sn_Port;				//Socket n�˿ں�/Դ�˿ں�
	
	//����ΪTCP�ͻ���ģʽʱ����	
	uint16_t Sn_DPort;			//Socket nĿ�Ķ˿ں� 
	uint8_t  Sn_DIP[4];			//Socket nĿ��IP��ַ 
	
	//����ΪUDPģʽʱ����
	uint16_t UDP_DPORT;			//Socket nĿ�Ķ˿ں�
	uint8_t  UDP_DIPR[4];		//Socket nĿ��IP��ַ
	uint8_t  _UDP_DIPR_[2]; //�ֽڶ���
}SOCKETnParam_t;


/*********************************************************************
 * Macros
 */
#define TRUE						0xff
#define FALSE						0x00

#define Rx_Buffer_Size	1024
#define Tx_Buffer_Size	1024

 /*******************************************************************************
 * EXTERNAL VARIABLES
 */

extern uint8_t Rx_Buffer[Rx_Buffer_Size];			//�������ݻ����� 
extern uint8_t Tx_Buffer[Tx_Buffer_Size];			//�������ݻ�����

extern NETWORKParam_t *Pnet_param;  					//�����������
extern SOCKETnParam_t *Psn_param;  					  //Socket n��������(n=0,1) 

/*******************************************************************************
 * FUNCTIONS
 */
void W5500_Init(void);
void W5500_Config(void);
void W5500_RST(void);
void W5500_Load_Net_Parameters(void);
uint8_t Detect_Gateway(void);
void W5500_Socket_Init(uint8_t sn);

void DO_TCP_Server(uint8_t sn);
void DO_UDP(uint8_t sn);


#endif   // _W5500_App_H_
