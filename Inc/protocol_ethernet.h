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

/* 指令码 */
#define	DummyIns						0x00	//!< 空指令
#define	CAttr_Read					0x01	//!< 读一个普通属性
#define	CAttr_Write					0x10	//!< 写一个普通属性
#define	ChxAttr_Read				0x02	//!< 读一个通道属性
#define	ChxAttr_Write				0x20	//!< 写一个通道属性

/* 状态机运行状态 */
#define _FSM_CPL_						0x00	//!< 状态机运行完成
#define _FSM_ON_GOING_			0x01	//!< 状态机正在运行
#define _FSM_ERR_						0x02	//!< 状态机错误

/* 数据通道参数（UDP端口） */
#define HEAD_SIZE						23		//!< UDP帧头长度 - 按字节
#define DATA_SIZE						33		//!< UDP帧数据域每样本长度 - 按字节

/*******************************************************************
 * TYPEDEFS
 */

/*!
 *  @def    声明 读属性回调函数原型
 *	@param	InsAttrNum - 待读属性编号
 *					CHxNum - 通道编号（通道属性专用，默认不用	0xFF）
 *					pValue - 属性值 （to be returned）
 *					pLen - 属性值大小（to be returned） 
 */
typedef uint8_t (*pfnReadAttrCB_t)( uint8_t InsAttrNum, uint8_t CHxNum, 
																		uint8_t *pValue, uint8_t *pLen );

/*!
 *  @def		声明 写属性回调函数原型
 *	@param	InsAttrNum - 待写入属性编号
 *					CHxNum - 通道编号（通道属性专用，默认不用	0xFF）
 *					pValue - 待写入数据的指针
 *					pLen - 待写入数据大小	
 */
typedef uint8_t (*pfnWriteAttrCB_t)(	uint8_t InsAttrNum, uint8_t CHxNum,
																			uint8_t *pValue, uint8_t len );

/*!
 *  @def    属性读写回调函数 结构体
 */
typedef struct
{
  pfnReadAttrCB_t 	pfnReadAttrCB;					//!< 读属性回调函数指针	
  pfnWriteAttrCB_t 	pfnWriteAttrCB;					//!< 写属性回调函数指针
} AttrCBs_t;

/*!
 *  @def    UDP帧头数据 结构体
 *
 *	@brief	本结构体内数据需通过属性表获取
 */
 typedef struct
{
	uint8_t	UDP_DevID[4];				//!< UDP数据源
	uint8_t	UDP_SampleNum[2];		//!< UDP包总样数
	uint8_t	UDP_ChannelNum;			//!< UDP包有效通道总数
	uint8_t AttrLen;						//!< 属性长度 - 按字节 （for debug）
} UDPHeader_t;

/**********************************************************************
 * FUNCTIONS
 */
void TCP_ProcessFSMInit(void);
uint8_t TCP_ProcessFSM(void);
uint8_t UDP_Process(uint8_t SampleNum ,uint8_t Procesflag);

uint8_t protocol_RegisterAttrCBs(AttrCBs_t *pAttrcallbacks);

#endif  // __PROTOCOL_ETHERNET_H__
