#ifndef __PROTOCOL_ETHERNET_H__
#define __PROTOCOL_ETHERNET_H__

/*********************************************************************
 * INCLUDES
 */
#include <stddef.h>
#include <stdint.h>

/*********************************************************************
 * CONSTANTS
 */

// ����ͨ����TCP�˿ڣ�

// ��λ��->�豸 ָ�����
const uint8_t TCP_Recv_FH = 0xAC;				//<! TCP����֡ͷ
const uint8_t TCP_Recv_FT = 0xCC;				//<! TCP����֡β

const uint8_t	DummyIns = 0x00;					//<! ��ָ��
const uint8_t	CAttr_Read = 0x01;				//<! ��һ����ͨ����
const uint8_t	CAttr_Write = 0x10;				//<! дһ����ͨ����
const uint8_t	ChxAttr_Read = 0x02;			//<! ��һ��ͨ������
const uint8_t	ChxAttr_Write = 0x20;			//<! дһ��ͨ������

// �豸->��λ�� �ظ�
const uint8_t TCP_Send_FH = 0xA2;				//<! TCP����֡ͷ
const uint8_t TCP_Send_FT = 0xC2;				//<! TCP����֡β

const uint8_t	TCP_SUCCESS = 0x00;				//<! ָ���������
const uint8_t	TCPERR_Attr_RO = 0x01;		//<! ָ����󣺶�ֻ������д����
const uint8_t	TCPERR_Attr_LEN = 0x02;		//<! ָ�����дָ�����ݳ��ȴ���

/*********************************************************************
 * FUNCTIONS
 */
void ProtocolProcessFSMInit(void);
void ProtocolProcessFSM(void);

#endif  // __PROTOCOL_ETHERNET_H__
