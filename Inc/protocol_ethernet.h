#ifndef __PROTOCOL_ETHERNET_H__
#define __PROTOCOL_ETHERNET_H__

/*********************************************************************
 * INCLUDES
 */
#include <stddef.h>
#include <stdint.h>

/*********************************************************************
 * Macros
 */
#define TCP_Rx_Buff_Size		16
#define TCP_Tx_Buff_Size		1024
#define UDP_Tx_Buff_Size		1024

/* ָ���� */
#define	DummyIns						0x00	//!< ��ָ��
#define	CAttr_Read					0x01	//!< ��һ����ͨ����
#define	CAttr_Write					0x10	//!< дһ����ͨ����
#define	ChxAttr_Read				0x02	//!< ��һ��ͨ������
#define	ChxAttr_Write				0x20	//!< дһ��ͨ������

/* ״̬������״̬ */
#define _FSM_CPL_						0x00	//!< ״̬���������
#define _FSM_ON_GOING_			0x01	//!< ״̬����������
#define _FSM_ERR_						0x02	//!< ״̬������

 /*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8_t TCP_Rx_Buff[TCP_Rx_Buff_Size];			//TCP�������ݻ����� 
extern uint8_t TCP_Tx_Buff[TCP_Tx_Buff_Size];			//TCP�������ݻ�����
extern uint8_t UDP_Tx_Buff[UDP_Tx_Buff_Size];			//UDP�������ݻ�����

/**********************************************************************
 * FUNCTIONS
 */
void ProtocolProcessFSMInit(void);
uint8_t ProtocolProcessFSM(void);

#endif  // __PROTOCOL_ETHERNET_H__
