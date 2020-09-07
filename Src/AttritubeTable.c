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
/*************************************************************************
 * INCLUDES
 */ 
#include "AttritubeTable.h" 
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*************************************************************************
 * GLOBAL VARIABLES
 */
//!< 属性总表
uint32_t* pattr_offset[ATTR_NUM];	//!< 属性偏移地址
uint8_t* pattr;	//!< 属性表首地址

//!< 工作状态与控制组
bool 		sampling;
int8_t	impmeas_mode;
int8_t	impmeas_fxn;

//!< 通信参数组
const uint8_t  dev_mac[6] = {0x0c,0x29,0xab,0x7c,0x00,0x01};
const uint8_t  dev_ip[4] = {192,168,1,10}; 
enum  Dev_PortStat_t dev_portstat; 
const uint16_t host_port = 7002;
const uint16_t samplenum = 10;

/**************************************************************************
 *  Attributes  Table
 */
const Attr_Tbl_t attr_tbl = {
	
	/*
	 *  ===================== 工作状态与控制组 ===========================
	 */
		
		//!< 采样控制 0-停止采样 1-开始采样 
		.Sampling			= {	ATTR_RS,							/* permissions */
											1,										/* datasize */
											(uint32_t*)&sampling  /* pValue */
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
												1,
												(uint32_t*)&host_port
											},
		
		//!< 以太网每包含ad样本数 - 10 (default)
		.SampleNum			= { ATTR_RS,
												1,
												(uint32_t*)&samplenum
											},   

};

/*********************************************************************
 * FUNCTIONS
 */
void Attr_Tbl_Init()
{
	//!< 属性表首地址
	pattr = (uint8_t*)&attr_tbl;
	
	//!< 属性地址偏移映射关系   
	//!< pattr_offset[n]即属性的物理地址 上位机通过下标进行偏移访问
	pattr_offset[0] = (uint32_t*)attr_tbl.Sampling.pValue;	
	pattr_offset[1] = (uint32_t*)attr_tbl.IMPMeas_Mode.pValue;
	pattr_offset[2] = (uint32_t*)attr_tbl.IMPMeas_fxn.pValue;
	pattr_offset[3] = (uint32_t*)attr_tbl.Dev_MAC.pValue;
	pattr_offset[4] = (uint32_t*)attr_tbl.Dev_IP.pValue;
	pattr_offset[5] = (uint32_t*)attr_tbl.Dev_PortStat.pValue;
	pattr_offset[6] = (uint32_t*)attr_tbl.Host_Port.pValue;
	pattr_offset[7] = (uint32_t*)attr_tbl.SampleNum.pValue;

}