/**
 * @file    AttritubeTable.c
 * @author  gjmsilly
 * @brief   MicroEEG_M1 属性表
 * @version 0.1
 * @date    2020-09-01
 *
 * @copyright (c) 2020 gjmsilly
 *
 */
/***********************************************************************
 * INCLUDES
 */ 
#include "AttritubeTable.h"
#include "protocol_ethernet.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
/***********************************************************************
 * LOCAL VARIABLES
 */
//!< 属性总表
static uint32_t* pattr_offset[ATTR_NUM];	//!< 属性偏移地址
static uint8_t* pattr;										//!< 属性表首地址

//!< 工作状态与控制组 属性
static bool 		sampling;
static int8_t	impmeas_mode;
static int8_t	impmeas_fxn;

//!< 通信参数组 属性
const uint8_t  dev_mac[6] = {0x0c,0x29,0xab,0x7c,0x00,0x01};
const uint8_t  dev_ip[4] = {192,168,1,10}; 
enum  Dev_PortStat_t dev_portstat; 
const uint16_t host_port = 7002;
const uint16_t samplenum = SAMPLENUM;

/************************************************************************
 *  Attributes  Table
 */
const Attr_Tbl_t attr_tbl = {
	
	/*
	 *  ===================== 工作状态与控制组 ===========================
	 */
		
		//!< 采样控制 0-停止采样 1-开始采样 
		.Sampling			= {	ATTR_RS,							/* permissions */
											1,										/* datasize */
											(uint32_t*)&sampling  /* pAttrValue */
										},
		
		//!< 阻抗测量模式	0- 无阻抗测量 
		.IMPMeas_Mode	= { ATTR_RS,
											1,
											(uint32_t*)&impmeas_mode
										},
											
		//!< 阻抗测量方案	0- 正弦波测AC电阻 1- 测DC电阻 2- 交流激励测阻抗
		.IMPMeas_fxn	= { ATTR_RS,
											1,
										 (uint32_t*)&impmeas_fxn
										},			 

	/*
	 *  ======================== 通信参数组 ==============================
	 */
			 
		//!< 仪器网口MAC地址  0c-29-ab-7c-00-01 (default) 
		.Dev_MAC				= { ATTR_RO,
												6,
												(uint32_t*)dev_mac
											},
		
		//!< 仪器当前IP地址 192.168.1.10 (default) 
		.Dev_IP					=	{ ATTR_NV,
												4,
												(uint32_t*)dev_ip
											},
			
		//!< 仪器网口状态
		.Dev_PortStat		= { ATTR_RA,
												1,
												(uint32_t*)&dev_portstat
											},
		
		//!< 目的主机UDP端口号 - 7002 (default)
		.Host_Port			= { ATTR_NV,
												2,
												(uint32_t*)&host_port
											},
		
		//!< 以太网每包含ad样本数 - 10 (default)
		.SampleNum			= { ATTR_RS,
												1,
												(uint32_t*)&samplenum
											},   

};


/************************************************************************
 *  Callbacks
 */
/* Attribute table callbacks */
static uint8_t ReadAttrCB(	uint8_t InsAttrNum,
														uint8_t CHxNum,
														uint8_t *pValue,
														uint8_t *pLen );

static uint8_t WriteAttrCB(	uint8_t InsAttrNum,
														uint8_t CHxNum,
														uint8_t *pValue,
														uint8_t len );

static AttrCBs_t attr_CBs =
{
	.pfnReadAttrCB = ReadAttrCB,					//!< 读属性回调函数指针 
	.pfnWriteAttrCB = WriteAttrCB					//!< 写属性回调函数指针
};

//////////////////////////////////////////////////////////////////////////

static pfnAttrChangeCB_t pAppCallbacks; //!< 应用程序回调函数指针 

/*!
 *  @fn	应用程序注册回调函数的接口
 *
 *	@param 应用程序的属性值变化回调函数
 *
 *	@return SUCCESS - 回调函数注册成功
 *					FAILURE - 回调函数注册失败
 */
uint8_t Attr_Tbl_RegisterAppCBs(void *appcallbacks)
{
	if ( appcallbacks )
  {
		pAppCallbacks = appcallbacks;
    return ( SUCCESS );
	}
	else
	{
	  return ( ERROR );
	}
}

/*!
 *  @fn	    读属性回调函数
 *
 *	@param	InsAttrNum - 待读属性编号
 *					CHxNum - 通道编号（通道属性专用，默认不用	0xFF）
 *					pValue - 属性值 （to be returned）
 *					pLen - 属性值大小（to be returned）
 *
 *	@return SUCCESS 读取属性值成功
 *					ATTR_NOT_FOUND 属性不存在
 */
static uint8_t ReadAttrCB(	uint8_t InsAttrNum,uint8_t CHxNum, 
														uint8_t *pValue, uint8_t *pLen )
{
	uint8_t status = SUCCESS;
	uint8_t *pAttrValue;	//!< 属性值地址

	if( (InsAttrNum > ATTR_NUM ) && ( CHxNum == 0xFF ))
	{
		status = ATTR_NOT_FOUND; //!< 属性不存在
	}
	
	//!< 读属性值
	if(status == SUCCESS)
	{
		pAttrValue = (uint8_t*)pattr_offset[InsAttrNum];//!< 属性值地址传递
		*pLen = *(pattr+InsAttrNum*6+1);	//!< 属性值大小传递（值传递!地址不变 9.13）
		memcpy(pValue,pAttrValue,*pLen); //!< 属性值读取
	}
	
	return status;
	
}

/*!
 *  @fn			写属性回调函数
 *
 *	@param	InsAttrNum - 待写入属性编号
 *					CHxNum - 通道编号（通道属性专用，默认不用	0xFF）
 *					pValue - 待写入数据的指针
 *					pLen - 待写入数据大小
 *
 *	@return SUCCESS 读取属性值成功
 *					ATTR_NOT_FOUND 属性不存在
 */
static uint8_t WriteAttrCB( uint8_t InsAttrNum,uint8_t CHxNum, 
														uint8_t *pValue, uint8_t len )
{
	uint8_t status;
	uint8_t notifyApp=0xFF;	//!< 标志位 - 通知上层应用程序属性值变化
	uint8_t AttrPermission;	//!< 属性读写权限	
	uint8_t AttrLen;				//!< 属性值大小
	uint8_t *pAttrValue;		//!< 属性值地址	
	
	AttrPermission = *(pattr+InsAttrNum*6); 
	AttrLen = *(pattr+InsAttrNum*6+1); 
	
	if( (InsAttrNum > ATTR_NUM ) && ( CHxNum == 0xFF ))
	{
		status = ATTR_NOT_FOUND; //!< 属性不存在
	}
	else if(( AttrPermission == ATTR_RO )||( AttrPermission == ATTR_NV ))
	{
		status = ATTR_ERR_RO; //!< 属性不允许写操作
	}
	else if( len!= AttrLen)
	{
		status = ATTR_ERR_SIZE; //!< 待写数据长度与属性值长度不符
	}
	else status = SUCCESS;
	
	//!< 写属性值并通知上层应用程序（AttrChange_Process）
	if(status == SUCCESS)
	{
		pAttrValue = (uint8_t*)pattr_offset[InsAttrNum];//!< 属性值地址传递
		
		memcpy(pAttrValue,pValue,len); //!< 属性值写入
		notifyApp=InsAttrNum;
	}
	
	if( (notifyApp!=0xFF) && pAppCallbacks )
	{
		(*pAppCallbacks)(notifyApp);
	}
	
	return status;

}
 /************************************************************************
 * FUNCTIONS
 */
/*!
 *  @fn	属性表初始化
 *	@brief 初始化属性表接口函数（读写回调）
 */
void Attr_Tbl_Init()
{
	/* 向以太网帧协议服务注册读写回调函数 */
	protocol_RegisterAttrCBs(&attr_CBs);
	
	/* 建立地址映射 */		
	pattr = (uint8_t*)&attr_tbl; //!< 属性表首地址
	
	//!< 属性地址偏移映射关系   
	//!< pattr_offset[n]即属性的物理地址 上位机通过下标进行偏移访问
	pattr_offset[SAMPLING] = (uint32_t*)attr_tbl.Sampling.pAttrValue;	
	pattr_offset[IMPMEAS_MODE] = (uint32_t*)attr_tbl.IMPMeas_Mode.pAttrValue;
	pattr_offset[IMPMEAS_FXN] = (uint32_t*)attr_tbl.IMPMeas_fxn.pAttrValue;
	pattr_offset[DEV_MAC] = (uint32_t*)attr_tbl.Dev_MAC.pAttrValue;
	pattr_offset[DEV_IP] = (uint32_t*)attr_tbl.Dev_IP.pAttrValue;
	pattr_offset[DEV_PORTSTAT] = (uint32_t*)attr_tbl.Dev_PortStat.pAttrValue;
	pattr_offset[HOST_PORT] = (uint32_t*)attr_tbl.Host_Port.pAttrValue;
	pattr_offset[SAMPLE_NUM] = (uint32_t*)attr_tbl.SampleNum.pAttrValue;

}

/*!
 *  @fn			读属性函数 （供上层应用获取属性）
 *
 *	@param	InsAttrNum - 待写入属性编号
 *					pValue - 待写入数据的指针
 *
 *	@return SUCCESS 读取属性值成功
 *					ATTR_NOT_FOUND 属性不存在
 */
uint8_t App_GetAttr(uint8_t InsAttrNum, uint8_t *pValue)
{
	uint8_t ret = SUCCESS;
	
	switch(InsAttrNum)
	{
		case SAMPLING:
			memcpy(pValue,&sampling,1);
			break;		
		
	}
  return ( ret );	
}