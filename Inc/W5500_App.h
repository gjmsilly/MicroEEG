#ifndef  _W5500_App_H_
#define  _W5500_App_H_

/*********************************************************************
 * INCLUDES
 */
#include "stdint.h"

/*********************************************************************
 * TYPEDEFS
 */
/* �������ò��� */    //8.26 ��ַ�������� debug����
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
	uint16_t Sn_Port;				//Socket n�˿ں�
	
	//TCPģʽ	
	uint16_t Sn_DPort;			//Socket nĿ�Ķ˿ں� 
	uint8_t  Sn_DIP[4];			//Socket nĿ��IP��ַ 
	
	//UDPģʽ
	uint16_t UDP_DPORT;			//Socket nĿ�Ķ˿ں�
	uint8_t  UDP_DIPR[4];		//Socket nĿ��IP��ַ
	//	uint16_t UDP_DPORT_[2];
}SOCKETnParam_t;

/* Socket n״̬���� */
typedef struct
{
	uint8_t Sn_Mode;				/*Socket n����ģʽ: 		- TCP������ 						0 
																									- TCP�ͻ��� 						1 
																									- UDP										2 */
	uint8_t Sn_State;				/*Socket n����״̬: 		- Socket n��ɳ�ʼ�� 		1
																									- Socket n�������      2 */ 
	uint8_t Sn_Data;				/*Socket n�շ�����״̬: - Socket n���յ�����		1
																									- Socket n�����������	2 */
}SOCKETnState_t;

/*********************************************************************
 * CONSTANTS
 */
#define TRUE						0xff
#define FALSE						0x00

/* Socket n�жϼĴ������� */
#define IR_SEND_OK			0x10				//send��������ж�
#define IR_TIMEOUT			0x08        //��ʱ�ж�
#define IR_RECV					0x04        //���յ������ж�
#define IR_DISCON				0x02        //���յ�FIN��FIN/ACK�ж�
#define IR_CON					0x01        //���ӽ����ж�

/* Socket n����ģʽ */
#define TCP_SERVER			0x00				//TCP������ģʽ
#define TCP_CLIENT			0x01				//TCP�ͻ���ģʽ 
#define UDP_MODE				0x02				//UDP(�㲥)ģʽ 

/* Socket n����״̬ */
#define S_INIT					0x01				//Socket n��ɳ�ʼ�� 
#define S_CONN					0x02				//Socket n�������,���������������� 

/* Socket n�շ�����״̬ */
#define S_RECEIVE	 			0x01				//Socket n���յ�һ�����ݰ� 
#define S_TRANSMITOK 		0x02				//Socket n���һ�����ݰ��ķ��� 

/*******************************************************************************
 * FUNCTIONS
 */
void W5500_Init(void);
void W5500_Config(void);
void W5500_RST(void);
void Load_Net_Parameters(void);
uint8_t Detect_Gateway(void);

void Socket_Init(uint8_t s);
uint8_t Socket_Connect(uint8_t s);
uint8_t Socket_Listen(uint8_t s);
uint8_t Socket_UDP(uint8_t s);
void W5500_Socket_Set(void);
void Process_Socket_Data(uint8_t s);

void W5500_Interrupt_Process(void);

#endif   // _W5500_App_H_
