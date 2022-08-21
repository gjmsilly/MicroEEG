/**
 * @file    imp_meas.c
 * @author  gjmsilly
 * @brief   MicroEEG 阻抗检测服务
 * @version 0.1
 * @date    2022-08-21
 * @copyright (c) 2022 gjmsilly
 *
 */
#include "ads1299.h"
#include "AttritubeTable.h"
#include "imp_meas.h"
#include "main.h"
 
 /************************************************************************
 * GLOBAL VARIABLES
 */
static uint8_t chx_imp_val[CHANNEL_NUM*3+CHANNEL_NUM/8*3]; //TODO 存储采样值

 /************************************************************************
 * LOCAL FUNCTIONS
 */
/*!
 *  @fn     imp_config
 *
 *  @brief  阻抗检测ADS1299寄存器配置
 *
 *  每次使能一个通道，对该通道添加电流源并使能一次采样。
 */
static void imp_config(uint8_t chx_en)
{
	int i;
	//关闭所有通道
	for( i=0; i<8; i++ ){
		ADS1299_Channel_Control(0,i,0);
	}
	//单独使能
	ADS1299_Channel_Control(0,chx_en,1);
}


 /************************************************************************
 * FUNCTIONS
 */
uint32_t imp_control(uint8_t chx_process)
{

		if( SYS_Event&CHX_IMP_START )
		{	
			imp_config(chx_process);//通道寄存器配置
			ADS1299_IMPMeas_Control(1);//开始一次阻抗测量
			SYS_Event &= ~CHX_IMP_START; //!< 清除前序事件
			
			return CHX_IMPING;
		}
		
		else if ( SYS_Event&CHX_IMP_REDY )
		{ 
			/* 读取阻抗采样值，转换为物理量 */
			ADS1299_ReadResult((uint8_t*)chx_imp_val); //读取阻抗采样值
			ADS1299_IMPMeas_Control(0); //停止采样
			//TODO转换为物理量
			App_WriteAttr(CHX_IMP,chx_process,chx_imp_val[chx_process]);//写入通道属性
			SYS_Event &= ~CHX_IMP_REDY; //!< 清除前序事件
			
			return CHX_IMP_CPL;
		}
		
		return CHX_IMPING;
}
 
 