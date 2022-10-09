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

/* TCP֡Э�����״̬������״̬ */
#define _FSM_CPL_						0x00	//!< ״̬���������
#define _FSM_ON_GOING_			0x01	//!< ״̬����������
#define _FSM_ERR_						0x02	//!< ״̬������

/* UDP֡Э���������״̬ */
#define UDP_DATA_CPL				0x00	//!< UDP�����򵥸�����������
#define UDP_HEADER_CPL			0x01	//!< UDP֡ͷ������


/* ����ͨ��������UDP�˿ڣ� */
#define HEAD_SIZE						23		//!< UDP֡ͷ���� - ���ֽ�

#ifdef Dev_Ch32 
#define DATA_SIZE						115		//!< UDP֡������ÿ�������� - ���ֽڣ�������ͷ��7 + (����ͨ��״̬3 + ��ͨ��8 x ÿͨ�������ֽ���3��x ͨ������4)
#endif
#ifdef Dev_Ch24 
#define DATA_SIZE						88		//!< UDP֡������ÿ�������� - ���ֽ�	��������ͷ��7 + (����ͨ��״̬3 + ��ͨ��8 x ÿͨ�������ֽ���3��x ͨ������3)	
#endif
#ifdef Dev_Ch16 
#define DATA_SIZE						61		//!< UDP֡������ÿ�������� - ���ֽ�	��������ͷ��7 + (����ͨ��״̬3 + ��ͨ��8 x ÿͨ�������ֽ���3��x ͨ������2)
#endif
#ifdef Dev_Ch8 
#define DATA_SIZE						34		//!< UDP֡������ÿ�������� - ���ֽ�	��������ͷ��7 + (����ͨ��״̬3 + ��ͨ��8 x ÿͨ�������ֽ���3��x ͨ������1)
#endif

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

/*!
 *  @def    UDP֡ͷ���� �ṹ��
 *
 *	@brief	���ṹ����������ͨ�����Ա��ȡ
 */
 typedef struct
{
	uint8_t	UDP_DevID[4];				//!< UDP����Դ
	uint8_t	UDP_SampleNum[2];		//!< UDP��������
	uint8_t	UDP_ChannelNum;			//!< UDP����Чͨ������
	uint8_t AttrLen;						//!< ���Գ��� - ���ֽ� ��for debug��
} UDPHeader_t;

/**********************************************************************
 * FUNCTIONS
 */
void TCP_ProcessFSMInit(void);
uint8_t TCP_ProcessFSM(void);

uint8_t UDP_DataProcess(uint8_t SampleNum ,uint32_t Procesflag);
uint8_t UDP_EvtProcess();

uint8_t protocol_RegisterAttrCBs(AttrCBs_t *pAttrcallbacks);

#endif  // __PROTOCOL_ETHERNET_H__
