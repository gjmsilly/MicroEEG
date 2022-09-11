/**
 * @file    imp_meas.c
 * @author  gjmsilly
 * @brief   MicroEEG 阻抗检测服务
 * @version 0.1
 * @date    2022-08-21
 * @copyright (c) 2022 gjmsilly
 *
 */
#include <math.h> 
#include <string.h>   
#include "ads1299.h"
#include "AttritubeTable.h"
#include "imp_meas.h"
#include "main.h"
 /************************************************************************
 * LOCAL VARIABLES
 */
//static uint8_t imp_sample[CHANNEL_NUM*3+CHANNEL_NUM/8*3]; //暂存所有通道阻抗检测原始采样值
uint32_t chx_imp_phyval[CHANNEL_NUM/8][8]; //存储转换后的物理量

 /************************************************************************
 * GLOBAL VARIABLES
 */
uint8_t imp_sample[CHANNEL_NUM*3+CHANNEL_NUM/8*3]; //暂存所有通道阻抗检测原始采样值
uint32_t chx_imp_sample[CHANNEL_NUM/8][8]; //存储提取后的通道采样值

uint8_t* pimp_sample = imp_sample;
uint32_t* pchx_imp_sample = chx_imp_sample[0]; 

 /************************************************************************
 * LOCAL FUNCTIONS
 */
/*!
 *  @fn     imp_config
 *
 *  @brief  阻抗检测ADS1299寄存器配置
 *  				每次使能一个通道，对该通道添加电流源。
 *	
  *	@param	chx_en	使能通道编号(0~7)
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
	
	//通道测试信号配置
	//[3:2]=00(6nA),01(24nA),**10(6uA),11(24uA)
	//[1:0]=00(DC),**01(7.8Hz),10(31.2Hz)
	ADS1299_WriteREG(0,ADS1299_REG_LOFF,0x09);				//[3:2]=00(6nA),01(24nA),10(6uA),11(24uA); [1:0]=01(7.8Hz),10(31.2Hz)
	ADS1299_WriteREG(0,ADS1299_REG_LOFFSENSN,1<<chx_en);
	ADS1299_WriteREG(0,ADS1299_REG_LOFFSENSP,1<<chx_en);
	ADS1299_WriteREG(0,ADS1299_REG_BIASSENSN,0x00);
	ADS1299_WriteREG(0,ADS1299_REG_BIASSENSP,0x00);
	ADS1299_WriteREG(0,ADS1299_REG_CONFIG3,0xec);
	ADS1299_WriteREG(0,ADS1299_REG_CONFIG4,0x02);
	ADS1299_WriteREG(0,ADS1299_REG_MISC1,0x20); // SRB1闭合
	
}

/*!
 *  @fn     convert_to_phy_val
 *
 *  @brief  阻抗采样值转换为物理量 单位kΩ
 *	
  *	@param	chx	通道编号(0~7)
 */
static void convert_to_phy_val(uint8_t chx)
{
	int chip;
	uint32_t sample_val;
	
	for( chip=0; chip<CHANNEL_NUM/8; chip++ )
	{
		sample_val = chx_imp_sample[chip][chx];

		//sample_val = 4.5*sample_val/8388608;//计算真实电压
		//chx_imp_phyval[chip][chx]= sample_val/6*1000; //KΩ
		
		chx_imp_phyval[chip][chx] = sample_val/11;//单位Ω 速算
	}

}

 /************************************************************************
 * FUNCTIONS
 */
uint32_t imp_control(uint8_t chx_process)
{
	int chip;

	if( SYS_Event&CHX_IMP_START )
	{	
		imp_config(chx_process);//通道寄存器配置
		ADS1299_Sampling_Control(1);//开始采样
		SYS_Event &= ~CHX_IMP_START; //!< 清除前序事件
		return CHX_IMPING;
	}
	
	else if ( SYS_Event&CHX_IMP_DONE )
	{ 
		
		ADS1299_Sampling_Control(0); //停止采样
		convert_to_phy_val(chx_process);//转换为物理量
		
		for( chip=0; chip<CHANNEL_NUM/8; chip++ )
		{
			int chx_Real_Num = chx_process+8*chip; //换算成实际通道属性编号
			App_WriteAttr( CHX_IMP, chx_Real_Num, chx_imp_phyval[chip][chx_process] );//写入通道属性
		}

		SYS_Event &= ~CHX_IMP_DONE; //!< 清除前序事件
		
		return CHX_IMP_CPL;
	}
	return CHX_IMPING;
}
 
 