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
 
#include "AttritubeTable.h" 
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 *  ===================== 工作状态与控制组 ===========================
 */
const State_Control state_control ={
	 
	.Sampling = 0,      		//!< 采样控制 0-停止采样 1-开始采样
	.IMPMeas_Mode = 0,      //!< 阻抗测量模式 0- 无阻抗测量 
													//!<							1- 阻抗测量（仪器出裸数）
													//!<							1- 阻抗测量（仪器出结果）	
	.IMPMeas_fxn = 0,        //!< 阻抗测量方案 0- 正弦波测AC电阻 
													//!<							1- 测DC电阻
													//!<							1- 交流激励测阻抗	
	
};

/*
 *  ======================== 通信参数组 ============================
 */
enum Host_Port  host_port;   //!< 仪器网口状态

const COMM_Param COMM_param = {
		 
		/* 仪器网口MAC地址 
			 0c-29-ab-7c-00-01 (default) */ 
		.Dev_MAC[0]			= 0x0c,
		.Dev_MAC[1]			= 0x29,
		.Dev_MAC[2]			= 0xab,
		.Dev_MAC[3]			= 0x7c,
		.Dev_MAC[4]			= 0x00,
		.Dev_MAC[5]			= 0x01,
		
		/* 仪器当前IP地址 
			 192.168.1.10 (default) */ 
		.Dev_IP[0]			= 192,
		.Dev_IP[1]			= 168,	
		.Dev_IP[2]			= 1,
		.Dev_IP[3]			= 10,

		.PDev_PortStat 	= (uint32_t*)&host_port,
		.Host_Port			= 7002,  //!< 目的主机UDP端口号 - 7002 (default)
		.SampleNum			= 10,    //!< 以太网每包含ad样本数 - 10 (default)
};
