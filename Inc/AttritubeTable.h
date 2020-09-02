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
 * TYPEDEFS
 */ 
/*
 *  ======================== 基本信息组 ============================
 */
/*!
 *  @def    Basic_Info_t
 *  @brief  基本信息 属性结构体
 */
typedef struct
{
	uint32_t 	Dev_UID;					//!< 仪器UID							<RO>
	uint16_t 	Dev_ChNum;				//!< 仪器总通道数					<RO>
	uint32_t* PDev_ChxType;			//!< 仪器逐通道类型				<RO>
	uint32_t*	PDev_ConType;			//!< 仪器选件类型					<RO>
}Basic_Info_t;

/*!
 *  @brief	仪器逐通道类型
 *  @note		检查枚举数值是否正确 （9.1）
 */
enum Dev_ChxType
{
	 Ref_Ch1 = 1,Ref_Ch2,Ref_Ch3,Ref_Ch4,Ref_Ch5,Ref_Ch6,Ref_Ch7,Ref_Ch8,  \
	 Ref_diffCh= -1,                                                       \
	 Ref_specCh= -2
};

/*!
 *  @brief	仪器选件类型
 */
//enum Dev_ConType
//{
//	NULL
//};

/*
 *  ===================== 工作状态与控制组 =========================
 */
/*!
 *  @def    State_Control_t
 *  @brief  工作状态与控制 属性结构体
 */

//#pragma pack(push)
//#pragma pack(1)
typedef struct 
{
	bool 			Sampling;					//!< 正在采样							<RS>
	int8_t 		IMPMeas_Mode;			//!< 阻抗测量模式					<RS>
	int8_t 		IMPMeas_fxn;			//!< 阻抗测量方案					<RS>
}State_Control_t;


/*
 *  ======================== 通信参数组 ============================
 */
/*!
 *  @def    COMM_Param_t
 *  @brief  通信参数 属性结构体
 */

//#pragma pack(push)
//#pragma pack(1)
typedef struct 
{
	uint8_t 	Dev_MAC[6];				//!< 仪器网口MAC地址				<RO>
	uint8_t 	Dev_IP[4];				//!< 仪器当前IP地址					<NV>
	uint32_t*	PDev_PortStat;		//!< 仪器网口状态						<RA>
	uint16_t	Host_Port;				//!< 目的主机UDP端口号			<NV>
	uint16_t	SampleNum;				//!< 以太网每包含ad样本数		<RO>
}COMM_Param_t;

/*!
 *  @brief		仪器网口状态
 */
enum Host_Port
{
	PORT_10M = 0,
	PORT_100M,
	PORT_1000M 
};

/*
 *  ======================== 采样参数组 ============================
 */
/*!
 *  @def		Sample_Param_t
 *  @brief	采样参数 属性结构体
 */
typedef struct
{
	int8_t		Samprate;				//!< 支持的采样率					<RO>
	int32_t		CurSamprate;		//!< 当前全局采样率				<RS>
	uint32_t*	PSamprate_tbl;	//!< 支持的分档采样率表		<RO>
	
	int8_t		Gain;						//!< 支持的增益						<RO>
	int32_t		CurGain;				//!< 当前全局增益					<RS>
	uint32_t*	PGain_tbl;			//!< 支持的分档增益表			<RO>
}Sample_Param_t;

/*!
 *  @brief	支持的分档采样率表
 */
enum Samprate_tbl
{
	SPS_250 = 250,
	SPS_500 = 500,
	SPS_1K = 1000
};

/*!
 *  @brief	支持的分档增益表
 */
enum Gain_tbl
{
	GAIN_X1 = 1,
	GAIN_X2 = 2,
	GAIN_X4 = 4,
	GAIN_X6 = 6,
	GAIN_X8 = 8,
	GAIN_X24 = 24
};

/*
 *  ======================= 逐通道参数组 ===========================
 */
/*!
 *  @def    CHx_Param_t
 *  @brief  逐通道参数 属性结构体
 */
typedef struct
{
	uint16_t	CHx_ConGain;			//!< 本通道选件增益					<RO>
	bool			CHx_en;						//!< 本通道允许							<RS>
	uint32_t*	CHx_Gain;					//!< 本通道增益							<RO>
	uint32_t*	CHx_Samprate;			//!< 本通道采样率						<RO>
	bool			CHx_ELD_enable;		//!< 本通道电极脱落检测允许	<RS>
	bool			CHx_ELS;					//!< 本通道电极脱落状态			<RA>
	uint32_t*	CHx_ELT;					//!< 本通道电极脱落阈值			<RO>
	uint32_t*	CHx_CurELT;				//!< 本通道当前电极脱落阈值	<RS>
	bool			CHx_GND;					//!< 本通道选件增益					<RS>
	bool			CHx_BIASOUT;			//!< 本通道选件增益					<RS>
}CHx_Param_t;

/*
 *  ====================== 外触发组 ================================
 */
/*!
 *  @def    Ex_Trigger_t
 *  @brief  外触发 属性结构体
 */
typedef struct
{
	bool			Trig_en;					//!< 外触发允许									<RS>
	bool		 	Trig_POL;					//!< 外触发极性									<RS>
	int8_t	 	Trig_event;				//!< 外触发事件辅助标记通道			<RS>
	int32_t		Trig_delay;				//!< 外触发信号延迟时间					<RS>
	int32_t		Trig_refdelay;		//!< 外触发事件辅助标记参考延迟	<RS>
}Ex_Trigger_t;

/*
 *  ====================== 偏置驱动组 ==============================
 */
/*!
 *  @def    BIAS_Driver_t
 *  @brief  偏置驱动 属性结构体
 */
typedef struct 
{
	bool			BIAS_DLD;					//!< 偏执驱动脱落检测				<RS>
	bool		 	BIAS_DLS;					//!< 偏执驱动脱落状态				<RA>
}BIAS_Driver_t;

/*
 *  ======================= 杂项组 ==================================
 */
/*!
 *  @def    Misc_t
 *  @brief  杂项 属性结构体
 */
typedef struct
{
	int16_t		Dev_BAT_vol;			//!< 仪器电池电压						<RA>
	bool		 	Dev_charg;				//!< 仪器充电状态						<RA>
	int8_t		Dev_BAT_pct;			//!< 仪器电池电量百分比 		<RA>
	int16_t		Dev_CPU_temp;			//!< 仪器CPU温度        		<RA>
	int16_t		Dev_AC_temp;			//!< 仪器模拟电路温度   		<RA>
	int16_t		Dev_LPC;					//!< 仪器光污染控制 	  		<RA>		
}Misc_t;

/*
 *  ====================== 错误信息组 ===============================
 */
/*!
 *  @def    ERR_Info_t
 *  @brief  错误信息 属性结构体
 */
//typedef struct __packed
//{

//}ERR_Info_t;

/*
 *  ======================== 属性总表 ===============================
 */
/*!
 *  @def    Attr_Tbl_t
 *  @brief  属性表结构体（紧凑结构确保连续内存，供上位机地址偏移访问）
 */

//#pragma pack(push)
//#pragma pack(1)
typedef struct
{
//	Basic_Info_t				Basic_Info;
	State_Control_t			State_Control;
	COMM_Param_t				COMM_Param;
//	Sample_Param_t			Sample_Param;
//	CHx_Param_t					CH1_Param[32];
//	Ex_Trigger_t				Ex_Trigger;
//	BIAS_Driver_t				BIAS_Driver;
//	Misc_t							Misc;
//	ERR_Info_t					ERR_Info;
}Attr_Tbl_t;

 /********************************************************************
 * EXTERNAL VARIABLES
 */
extern State_Control_t state_control;
extern COMM_Param_t COMM_param;
extern Attr_Tbl_t attr_tbl;

/*********************************************************************
 * FUNCTIONS
 */
void Attr_Tbl_Init();

#endif
