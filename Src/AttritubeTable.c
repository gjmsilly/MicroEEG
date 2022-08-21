/**
 * @file    AttrTbl.c
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
#include "w5500_service.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/***********************************************************************
 * LOCAL VARIABLES
 */
//!< 属性总表
static uint8_t* pattr_offset[ATTR_NUM];		//!< 属性偏移地址 
static uint8_t* pchxattr_offset[CHANNEL_NUM][10];		//!< 通道属性偏移地址 

//!< 基本信息组 属性
const	uint16_t channelnum = CHANNEL_NUM;

//!< 工作状态与控制组 属性
static bool 	sampling;
static bool		impmeas_en;
static uint8_t	impmeas_mode;

//!< 通信参数组 属性
Dev_PortStat_t dev_portstat; 
uint16_t samplenum = SAMPLENUM;

//!< 采样参数组 属性
static uint32_t cursamprate = SPS_1K;
static uint32_t samprate_tbl[]={SPS_250,SPS_500,SPS_1K,SPS_2K,SPS_4K};
static uint32_t curgain = GAIN_X24;
static uint32_t gain_tbl[]={GAIN_X1,GAIN_X2,GAIN_X4,GAIN_X6,GAIN_X8,GAIN_X12,GAIN_X24};

//!< 通道属性相关
static bool chx_en[CHANNEL_NUM];
//static uint32_t chx_gain[CHANNEL_NUM]; //本版本均不支持单独修改，保持和全局一致
//static uint32_t chx_sampletare[CHANNEL_NUM];
static bool chx_lead_off_en[CHANNEL_NUM];
static bool chx_lead_off_stat[CHANNEL_NUM];
static uint32_t chx_lead_off_th[CHANNEL_NUM];
static bool chx_gnd[CHANNEL_NUM];
static bool chx_biasout[CHANNEL_NUM];
static uint32_t chx_imp[CHANNEL_NUM]; 

/************************************************************************
 *  Attributes  Table
 */
const Attr_Tbl_t attr_tbl = {

	/*
	 *  ======================== 基本信息组 ==============================
	 */
		//!< 仪器UID  
		.Dev_UID			= {	ATTR_RO,									/* permissions */
											4,												/* datasize */
											(uint32_t*)CPU_UUID_ADDR  /* pAttrValue */
										},
		
		//!< 仪器总通道数
		.Dev_ChNum		= { ATTR_RO,
											2,
											(uint32_t*)&channelnum
										},
															
	
	/*
	 *  ===================== 工作状态与控制组 ===========================
	 */
		
		//!< 采样开关 0-停止采样 1-开始采样 
		.Sampling			= {	ATTR_RS,							/* permissions */
											1,										/* datasize */
											(uint32_t*)&sampling  /* pAttrValue */
										},
		
		//!< 阻抗测量开关 0-采样模式 1-阻抗检测模式
		.IMPMeas_En		= { ATTR_RS,
											1,
											(uint32_t*)&impmeas_en
										},
											
		//!< 阻抗测量方案	0- 正弦波测AC电阻 1- 测DC电阻 2- 交流激励测阻抗
		.IMPMeas_Mode	= { ATTR_RS,
											1,
										 (uint32_t*)&impmeas_mode
										},			 
										
		//!< 阻抗测量值（总表） 也可选择从通道属性-本通道阻抗值 读取
		.IMPMeas_Val	= { ATTR_RA,
											4*CHANNEL_NUM,
										 (uint32_t*)&chx_imp
										},

	/*
	 *  ======================== 通信参数组 ==============================
	 */
			 
		//!< 仪器网口MAC地址  0c-29-ab-7c-00-01 (default) 
		.Dev_MAC				= { ATTR_RO,
												6,
												(uint32_t*)net_param.Phy_Addr
											},
		
		//!< 仪器当前IP地址 192.168.1.10 (default) 
		.Dev_IP					=	{ ATTR_NV,
												4,
												(uint32_t*)net_param.IP_Addr
											},
			
		//!< 仪器网口状态
		.Dev_PortStat		= { ATTR_RA,
												1,
												(uint32_t*)&dev_portstat
											},
		
		//!< 目的主机UDP端口号 - 7002 (default)
		.Host_Port			= { ATTR_NV,
												2,
												(uint32_t*)&sn_param[1].UDP_DPORT
											},
		
		//!< 以太网每包含ad样本数 - 10 (default)
		.SampleNum			= { ATTR_RS,
												1,
												(uint32_t*)&samplenum
											},   

	/*
	 *  ======================== 通信参数组 ==============================
	 */
			 
		//!< 支持的采样率挡位		
		.Samprate_tbl		= { ATTR_RO,
												sizeof(samprate_tbl),
												(uint32_t*)&samprate_tbl
											},
		
		//!< 当前全局采样率 1ksps (default) 
		.CurSamprate		=	{ ATTR_RS,
												4,
												(uint32_t*)&cursamprate
											},
		
		//!< 支持的增益挡位
		.Gain_tbl				= { ATTR_RO,
												sizeof(gain_tbl),
												(uint32_t*)&gain_tbl
											},
		
		//!< 当前全局增益 x24 (default) 
		.CurGain				=	{ ATTR_RS,
												4,
												(uint32_t*)&curgain
											},		
};

/* 通道属性表 */
static CHx_Param_t CHx_Param[CHANNEL_NUM];

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

	if ((InsAttrNum > CHANNEL_NUM ) && ( CHxNum != CHX_NONE ) )
	{
		status = ATTR_NOT_FOUND; //!< 通道属性不存在	
	}
	else if( (InsAttrNum > ATTR_NUM ) && ( CHxNum == CHX_NONE ))
	{
		status = ATTR_NOT_FOUND; //!< 属性不存在
	}
	
	//!< 读属性值
	if(status == SUCCESS)
	{
		if( CHxNum == 0xFF ) //通用属性读操作
		{
			pAttrValue = (uint8_t*)*(uint32_t*)(pattr_offset[InsAttrNum]+2);//!< 属性值地址传递
			*pLen = *(pattr_offset[InsAttrNum]+1); //!< 属性值大小传递（值传递!地址不变 9.13）
		}else //通道属性读操作
		{
			pAttrValue = (uint8_t*)*(uint32_t*)(pchxattr_offset[CHxNum][InsAttrNum]+2);//!< 属性值地址传递
			*pLen = *(pchxattr_offset[CHxNum][InsAttrNum]+1); //!< 属性值大小传递（值传递!地址不变 9.13）
		}
		
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
	
	if(CHxNum == 0xFF)
	{
		AttrPermission = *(pattr_offset[InsAttrNum]); 
		AttrLen = *(pattr_offset[InsAttrNum]+1); 
	}else{
		AttrPermission = *(pchxattr_offset[CHxNum][InsAttrNum]); 
		AttrLen = *(pchxattr_offset[CHxNum][InsAttrNum]+1); 
	}

	
	if ((InsAttrNum > CHANNEL_NUM ) && ( CHxNum != CHX_NONE ) )
	{
		status = ATTR_NOT_FOUND; //!< 通道属性不存在	
	}
	else if( (InsAttrNum > ATTR_NUM ) && ( CHxNum == CHX_NONE ) )
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
	if ( status == SUCCESS )
	{
		if( CHxNum == CHX_NONE ) //通用属性写操作
		{
			pAttrValue = (uint8_t*)*(uint32_t*)(pattr_offset[InsAttrNum]+2);//!< 属性值地址传递
		} else //通道属性写操作
		{
			pAttrValue = (uint8_t*)*(uint32_t*)(pchxattr_offset[CHxNum][InsAttrNum]+2);//!< 属性值地址传递 
		}
		
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
 * LOCAL FUNCTIONS
 */
static void CHx_Tbl_Init()
{
	int i;
	
	for( i=0; i<CHANNEL_NUM; i++ ){
		
		/* 本通道选件增益 - 全局增益 */
		CHx_Param[i].CHx_ConGain.permissions = ATTR_RO;
		CHx_Param[i].CHx_ConGain.Attrsize = 4;
		CHx_Param[i].CHx_ConGain.pAttrValue = (uint32_t*)&curgain;
	
		/* 本通道允许 */
		CHx_Param[i].CHx_En.permissions = ATTR_RS;
		CHx_Param[i].CHx_En.Attrsize = 1;
		CHx_Param[i].CHx_En.pAttrValue = (uint32_t*)&chx_en[i];

		/* 本通道增益 */
		CHx_Param[i].CHx_Gain.permissions = ATTR_RO;
		CHx_Param[i].CHx_Gain.Attrsize = 4;
		CHx_Param[i].CHx_Gain.pAttrValue = (uint32_t*)&curgain;		
		
		/* 本通道采样率 */
		CHx_Param[i].CHx_Samprate.permissions = ATTR_RO;
		CHx_Param[i].CHx_Samprate.Attrsize = 4;
		CHx_Param[i].CHx_Samprate.pAttrValue = (uint32_t*)&cursamprate;	
		
		/* 本通道电极脱落检测允许 */
		CHx_Param[i].CHx_Lead_off_En.permissions = ATTR_RS;
		CHx_Param[i].CHx_Lead_off_En.Attrsize = 1;
		CHx_Param[i].CHx_Lead_off_En.pAttrValue = (uint32_t*)&chx_lead_off_en[i];	

		/* 本通道电极脱落状态 */
		CHx_Param[i].CHx_Lead_off_stat.permissions = ATTR_RA;
		CHx_Param[i].CHx_Lead_off_stat.Attrsize = 1;
		CHx_Param[i].CHx_Lead_off_stat.pAttrValue = (uint32_t*)&chx_lead_off_stat[i];		
		
		/* 本通道电极脱落阈值 */
		CHx_Param[i].CHx_Lead_off_TH.permissions = ATTR_RS;
		CHx_Param[i].CHx_Lead_off_TH.Attrsize = 4;
		CHx_Param[i].CHx_Lead_off_TH.pAttrValue = (uint32_t*)&chx_lead_off_th[i];	
		
		/* 本通道是否接地 */
		CHx_Param[i].CHx_GND.permissions = ATTR_RS;
		CHx_Param[i].CHx_GND.Attrsize = 1;
		CHx_Param[i].CHx_GND.pAttrValue = (uint32_t*)&chx_gnd[i];	

		/* 本通道是否加入共模 */
		CHx_Param[i].CHx_BIASOUT.permissions = ATTR_RS;
		CHx_Param[i].CHx_BIASOUT.Attrsize = 1;
		CHx_Param[i].CHx_BIASOUT.pAttrValue = (uint32_t*)&chx_biasout[i];

		/* 本通道阻抗值 */
		CHx_Param[i].CHx_IMP.permissions = ATTR_RA;
		CHx_Param[i].CHx_IMP.Attrsize = 4;
		CHx_Param[i].CHx_IMP.pAttrValue = (uint32_t*)&chx_imp[i];
		
		/* 通道属性值设置默认值 */
		chx_en[i]=1;
		//chx_gain[i]=curgain;
		//chx_sampletare[i]=cursamprate;
		chx_lead_off_en[i]=0;
		chx_lead_off_stat[i]=0;
		chx_lead_off_th[i]=0;
		chx_gnd[i]=0;
		chx_biasout[i]=0;
		chx_imp[i]=1000;	//1MΩ by default
		
		/* 建立地址映射 */
		pchxattr_offset[i][CHX_CONGAIN]= (uint8_t*)&CHx_Param[i].CHx_ConGain.permissions;
		pchxattr_offset[i][CHX_EN]= (uint8_t*)&CHx_Param[i].CHx_En.permissions;
		pchxattr_offset[i][CHX_GAIN]= (uint8_t*)&CHx_Param[i].CHx_Gain.permissions;
		pchxattr_offset[i][CHX_SAMPLERATE]= (uint8_t*)&CHx_Param[i].CHx_Samprate.permissions;
		pchxattr_offset[i][CHX_LEAD_OFF_EN]= (uint8_t*)&CHx_Param[i].CHx_Lead_off_En.permissions;
		pchxattr_offset[i][CHX_LEAD_OFF_STAT]= (uint8_t*)&CHx_Param[i].CHx_Lead_off_stat.permissions;
		pchxattr_offset[i][CHX_LEAD_OFF_TH]= (uint8_t*)&CHx_Param[i].CHx_Lead_off_TH.permissions;
		pchxattr_offset[i][CHX_GND]= (uint8_t*)&CHx_Param[i].CHx_GND.permissions;
		pchxattr_offset[i][CHX_BIASOUT]= (uint8_t*)&CHx_Param[i].CHx_BIASOUT.permissions;
		pchxattr_offset[i][CHX_IMP]= (uint8_t*)&CHx_Param[i].CHx_IMP.permissions;
	}
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
	
	/* 初始化通道属性表 */
	CHx_Tbl_Init();
	
	/* 建立通用属性的地址映射 */		
	
	//!< 属性地址偏移映射关系   
	//!< pattr_offset[n]即该属性的物理首地址 上位机通过下标进行偏移访问
	pattr_offset[DEV_UID] = (uint8_t*)&attr_tbl.Dev_UID.permissions;
	pattr_offset[DEV_CHANNEL_NUM] = (uint8_t*)&attr_tbl.Dev_ChNum.permissions;
	pattr_offset[SAMPLING] = (uint8_t*)&attr_tbl.Sampling.permissions;	
	pattr_offset[DEV_MAC] = (uint8_t*)&attr_tbl.Dev_MAC.permissions;
	pattr_offset[DEV_IP] = (uint8_t*)&attr_tbl.Dev_IP.permissions;
	pattr_offset[SAMPLE_NUM] = (uint8_t*)&attr_tbl.SampleNum.permissions;
	pattr_offset[SAMPLERATE_TBL] = (uint8_t*)&attr_tbl.Samprate_tbl.permissions;
	pattr_offset[CURSAMPLERATE] = (uint8_t*)&attr_tbl.CurSamprate.permissions;
	pattr_offset[GAIN_TBL] = (uint8_t*)&attr_tbl.Gain_tbl.permissions;
	pattr_offset[CURGAIN] = (uint8_t*)&attr_tbl.CurGain.permissions;	
	pattr_offset[IMP_MEAS_EN] = (uint8_t*)&attr_tbl.IMPMeas_En.permissions;
	pattr_offset[CHX_IMP_VAL] = (uint8_t*)&attr_tbl.IMPMeas_Val.permissions;
	
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
uint8_t App_GetAttr(uint8_t InsAttrNum, uint8_t CHxNum, uint32_t *pValue)
{
	uint8_t ret = SUCCESS;
	
	if(CHxNum == CHX_NONE) //应用层通用属性读操作
	{
		switch(InsAttrNum)
		{
			case SAMPLING:
				memcpy(pValue,&sampling,1);
				break;
			
			case CURSAMPLERATE:
				memcpy(pValue,&cursamprate,4);
				break;
			
			case CURGAIN:
				memcpy(pValue,&curgain,4);
				break;		
				
			case IMP_MEAS_EN:
				memcpy(pValue,&impmeas_en,1);
				break;
				
			case CHX_IMP_VAL:
				memcpy(pValue,&chx_imp,4*CHANNEL_NUM);
				break;
		}
	} 
	//else //通道属性读操作
	//{
	//	switch(InsAttrNum)
	//	{
	//		
	//	}
	//
	//}
	
  return ( ret );	
}

/*!
 *  @fn			写属性函数 （供上层应用修改属性值）
 *
 *	@param	InsAttrNum - 待写入属性编号
 *					Value - 待写入数据
 *
 *	@return SUCCESS 写属性值成功
 *					ATTR_NOT_FOUND 属性不存在
 */
uint8_t App_WriteAttr(uint8_t InsAttrNum, uint8_t CHxNum, uint32_t Value)
{
	uint8_t ret = SUCCESS;
	if(CHxNum == CHX_NONE) //应用层通用属性读操作
	{	
		switch(InsAttrNum)
		{
			case SAMPLING:		//!< 应用层修改正在采样属性
				sampling = (uint8_t)Value;
				break;	
			
			case CURGAIN:			//!< 应用层修改全局增益属性 
				curgain = (uint32_t)Value;
				break;
			
			case CURSAMPLERATE: //!< 应用层修改全局采样率属性
				cursamprate = (uint32_t)Value;
			break;
		}
	}
	else
	{
		switch(InsAttrNum)
		{
			case CHX_EN:		//!< 应用层修改通道开关
				chx_en[CHxNum] = (uint8_t)Value; 
			break;
			
			case CHX_IMP:
				chx_imp[CHxNum] = (uint32_t)Value;
			break;
		}
	}
  return ( ret );	
}