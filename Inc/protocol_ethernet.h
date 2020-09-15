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

/*******************************************************************
 * TYPEDEFS
 */

/*!
 *  @def    ���� �����Իص�����ԭ��
 *	@param	InsAttrNum - �������Ա��
 *					CHxNum - ͨ����ţ�ͨ������ר�ã�Ĭ�ϲ���	0xFF��
 *					pValue - ����ֵ ��to be returned��
 *					pLen - ����ֵ��С��to be returned�� 
 */
typedef uint8_t (*pfnReadAttrCB_t)( uint8_t InsAttrNum, uint8_t CHxNum, 
																		uint8_t *pValue, uint8_t *pLen );

/*!
 *  @def		���� д���Իص�����ԭ��
 *	@param	InsAttrNum - ��д�����Ա��
 *					CHxNum - ͨ����ţ�ͨ������ר�ã�Ĭ�ϲ���	0xFF��
 *					pValue - ��д�����ݵ�ָ��
 *					pLen - ��д�����ݴ�С	
 */
typedef uint8_t (*pfnWriteAttrCB_t)(	uint8_t InsAttrNum, uint8_t CHxNum,
																			uint8_t *pValue, uint8_t len );

/*!
 *  @def    ���Զ�д�ص����� �ṹ��
 */
typedef struct
{
  pfnReadAttrCB_t 	pfnReadAttrCB;					//!< �����Իص�����ָ��	
  pfnWriteAttrCB_t 	pfnWriteAttrCB;					//!< д���Իص�����ָ��
} AttrCBs_t;

/**********************************************************************
 * FUNCTIONS
 */
void TCP_ProcessFSMInit(void);
uint8_t TCP_ProcessFSM(void);

uint8_t protocol_RegisterAttrCBs(AttrCBs_t *pAttrcallbacks);

#endif  // __PROTOCOL_ETHERNET_H__
