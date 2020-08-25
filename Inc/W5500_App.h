#ifndef  _W5500_App_H_
#define  _W5500_App_H_

#include "stdint.h"

/***************----- ��������������� -----***************/
extern uint8_t Gateway_IP[4];	//����IP��ַ 
extern uint8_t Sub_Mask[4];	//�������� 
extern uint8_t Phy_Addr[6];	//�����ַ(MAC) 
extern uint8_t IP_Addr[4];	//����IP��ַ 

extern uint16_t S0_Port;	//�˿�0�Ķ˿ں�(5000) 
extern uint8_t S0_DIP[4];		//�˿�0Ŀ��IP��ַ 
extern uint16_t S0_DPort;	//�˿�0Ŀ�Ķ˿ں�(6000) 

extern uint8_t UDP_DIPR[4];	//UDP(�㲥)ģʽ,Ŀ������IP��ַ
extern uint16_t UDP_DPORT;	//UDP(�㲥)ģʽ,Ŀ�������˿ں�
extern uint16_t UDP_DPORT_[2];	//UDP(�㲥)ģʽ,����Ŀ�������˿ں�
/***************----- �˿ڵ�����ģʽ -----***************/
extern uint8_t S0_Mode;	//�˿�0������ģʽ,0:TCP������ģʽ,1:TCP�ͻ���ģʽ,2:UDP(�㲥)ģʽ
#define TCP_SERVER		0x00	//TCP������ģʽ
#define TCP_CLIENT		0x01	//TCP�ͻ���ģʽ 
#define UDP_MODE		0x02	//UDP(�㲥)ģʽ 

/***************----- �˿ڵ�����״̬ -----***************/
extern uint8_t S0_State;	//�˿�0״̬��¼,1:�˿���ɳ�ʼ��,2�˿��������(����������������) 
#define S_INIT			0x01	//�˿���ɳ�ʼ�� 
#define S_CONN			0x02	//�˿��������,���������������� 

/***************----- �˿��շ����ݵ�״̬ -----***************/
extern uint8_t S0_Data;		//�˿�0���պͷ������ݵ�״̬,1:�˿ڽ��յ�����,2:�˿ڷ���������� 
#define S_RECEIVE		0x01		//�˿ڽ��յ�һ�����ݰ� 
#define S_TRANSMITOK	0x02		//�˿ڷ���һ�����ݰ���� 

/***************----- �˿����ݻ����� -----***************/
extern uint8_t Rx_Buffer[2048];	//�˿ڽ������ݻ����� 
extern uint8_t Tx_Buffer[2048];	//�˿ڷ������ݻ����� 

extern uint8_t W5500_Interrupt;	//W5500�жϱ�־(0:���ж�,1:���ж�)

void W5500_Init(void);
void W5500_Initialization(void);
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
