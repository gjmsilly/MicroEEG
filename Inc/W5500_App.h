#ifndef  _W5500_APP_H_
#define  _W5500_APP_H_

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
#define TRUE							0x00
#define FALSE							0xff

// �˿�״̬
#define Sn_OPEN						0x00	//!< �˿��Ѵ�
#define Sn_LISTEN					0x01	//!< �˿����ڼ���
#define Sn_CLOSE					0x02	//!< �˿��ѹر�
#define TCP_PROCESS				0x03	//!< TCP�˿����ڴ�������
#define TCP_COMPLETE			0x04	//!< TCP�˿����ݴ������

/*******************************************************************************
 * FUNCTIONS
 */
void W5500_Init(void);
void W5500_Config(void);
void W5500_RST(void);
void W5500_Load_Net_Parameters(void);
uint8_t Detect_Gateway(void);
void W5500_Socket_Init(uint8_t sn);

uint8_t TCPServer_Service(uint8_t sn);
uint8_t UDP_Service(uint8_t sn);


#endif   // _w5500_app_H_
