/**
 * @file    AttritubeTable.h
 * @author  gjmsilly
 * @brief   MicroEEG_M1 属性表
 * @version 0.1
 * @date    2020-09-01
 *
 * @copyright (c) 2020 gjmsilly
 *
 */
 
#ifndef __ATTRITUBETABLE_H
#define __ATTRITUBETABLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************
 * CONSTANTS
 */
#define ATTR_NUM						40		//!< 属性表支持的属性数量
#define CHANNEL_MODE				8			//!< 通道数量

/* 属性权限 */
#define	ATTR_RO							0x00	//!< 只读属性 
#define	ATTR_RS							0x01	//!< 读写属性，可读可写
#define	ATTR_RA							0x02	//!< 读写属性，实时更新
#define	ATTR_NV							0x03	//!< 非易失属性

/* 属性值偏移地址表 */
#define SAMPLING						pattr_offset
#define IMPMEAS_MODE				pattr_offset[1]
#define IMPMEAS_FXN					pattr_offset[2]
#define	DEV_MAC							pattr_offset[3]
#define DEV_IP							pattr_offset[4]
#define DEV_PORTSTAT				pattr_offset[5]
#define HOST_PORT						pattr_offset[6]
#define SAMPLE_NUM					pattr_offset[7]

/*******************************************************************
 * TYPEDEFS
 */
/*!
 *  @def    Attr_t
 *  @brief  属性格式
 */
#pragma pack(push)
#pragma pack(1)
typedef struct 
{
	uint8_t					permissions;		//!< 属性权限 - 读写允许
  uint8_t					datasize;				//!< 属性长度 - 按字节
	uint32_t* const	pValue;					//!< 属性地址
} Attr_t;

/*!
 *  @def    CHx_Param_t
 *  @brief  逐通道参数 属性结构体
 */
typedef struct
{
	/* 逐通道参数组 */
	Attr_t	CHx_ConGain;			//!< 本通道选件增益							<RO>
	Attr_t	CHx_en;						//!< 本通道允许									<RS>
	Attr_t	CHx_Gain;					//!< 本通道增益									<RO>
	Attr_t	CHx_Samprate;			//!< 本通道采样率								<RO>
	Attr_t	CHx_ELD_enable;		//!< 本通道电极脱落检测允许			<RS>
	Attr_t	CHx_ELS;					//!< 本通道电极脱落状态					<RA>
	Attr_t	CHx_ELT;					//!< 本通道电极脱落阈值					<RO>
	Attr_t	CHx_CurELT;				//!< 本通道当前电极脱落阈值			<RS>
	Attr_t	CHx_GND;					//!< 本通道选件增益							<RS>
	Attr_t	CHx_BIASOUT;			//!< 本通道选件增益							<RS>

}CHx_Param_t;
/*!
 *  @def    Attr_Tbl_t
 *  @brief  属性表结构体（紧凑结构确保连续内存，供上位机地址偏移访问）
 */

#pragma pack(push)
#pragma pack(1)
typedef struct
{

//	/* 基本信息组 */
//	Attr_t 	Dev_UID;					//!< 仪器UID										<RO>
//	Attr_t 	Dev_ChNum;				//!< 仪器总通道数								<RO>
//	Attr_t 	Dev_ChxType;			//!< 仪器逐通道类型							<RO>
//	Attr_t	Dev_ConType;			//!< 仪器选件类型								<RO>

	/* 工作状态与控制组 */
	Attr_t	Sampling;					//!< 正在采样										<RS>
	Attr_t	IMPMeas_Mode;			//!< 阻抗测量模式								<RS>
	Attr_t	IMPMeas_fxn;			//!< 阻抗测量方案								<RS>

	/* 通信参数组 */
	Attr_t 	Dev_MAC;					//!< 仪器网口MAC地址						<RO>
	Attr_t 	Dev_IP;						//!< 仪器当前IP地址							<NV>
	Attr_t	Dev_PortStat;			//!< 仪器网口状态								<RA>
	Attr_t	Host_Port;				//!< 目的主机UDP端口号					<NV>
	Attr_t	SampleNum;				//!< 以太网每包含ad样本数				<RO>

//	/* 采样参数组 */
//	Attr_t	Samprate;					//!< 支持的采样率								<RO>
//	Attr_t	CurSamprate;			//!< 当前全局采样率							<RS>
//	Attr_t	Samprate_tbl;			//!< 支持的分档采样率表					<RO>
//	
//	Attr_t	Gain;							//!< 支持的增益									<RO>
//	Attr_t	CurGain;					//!< 当前全局增益								<RS>
//	Attr_t	Gain_tbl;					//!< 支持的分档增益表						<RO>
	
//	/* 逐通道参数组 */
//	CHx_Param_t	CHx_Param[CHANNEL_MODE];

//	/* 外触发组 */
//	Attr_t	Trig_en;					//!< 外触发允许									<RS>
//	Attr_t	Trig_POL;					//!< 外触发极性									<RS>
//	Attr_t	Trig_event;				//!< 外触发事件辅助标记通道			<RS>
//	Attr_t	Trig_delay;				//!< 外触发信号延迟时间					<RS>
//	Attr_t	Trig_refdelay;		//!< 外触发事件辅助标记参考延迟	<RS>

//	/* 偏置驱动组 */
//	Attr_t	BIAS_DLD;					//!< 偏执驱动脱落检测						<RS>
//	Attr_t	BIAS_DLS;					//!< 偏执驱动脱落状态						<RA>

//	/* 杂项组 */
//	Attr_t	Dev_BAT_vol;			//!< 仪器电池电压								<RA>
//	Attr_t	Dev_charg;				//!< 仪器充电状态								<RA>
//	Attr_t	Dev_BAT_pct;			//!< 仪器电池电量百分比 				<RA>
//	Attr_t	Dev_CPU_temp;			//!< 仪器CPU温度        				<RA>
//	Attr_t	Dev_AC_temp;			//!< 仪器模拟电路温度   				<RA>
//	Attr_t	Dev_LPC;					//!< 仪器光污染控制 	  				<RA>		

}Attr_Tbl_t;


/*!
 *  @brief		仪器网口状态
 */
enum Dev_PortStat_t
{
	PORT_10M = 0,
	PORT_100M,
	PORT_1000M 
};

///*!
// *  @brief	支持的分档采样率表
// */
//enum Samprate_tbl
//{
//	SPS_250 = 250,
//	SPS_500 = 500,
//	SPS_1K = 1000
//};

///*!
// *  @brief	支持的分档增益表
// */
//enum Gain_tbl
//{
//	GAIN_X1 = 1,
//	GAIN_X2 = 2,
//	GAIN_X4 = 4,
//	GAIN_X6 = 6,
//	GAIN_X8 = 8,
//	GAIN_X24 = 24
//};

///*!
// *  @brief	仪器逐通道类型
// *  @note		检查枚举数值是否正确 （9.1）
// */
//enum Dev_ChxType
//{
//	 Ref_Ch1 = 1,Ref_Ch2,Ref_Ch3,Ref_Ch4,Ref_Ch5,Ref_Ch6,Ref_Ch7,Ref_Ch8,  \
//	 Ref_diffCh= -1,                                                       \
//	 Ref_specCh= -2
//};


 /********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8_t* pattr;
extern uint32_t* pattr_offset[ATTR_NUM];	//!< 属性偏移地址

/*********************************************************************
 * FUNCTIONS
 */
void Attr_Tbl_Init();

#endif
