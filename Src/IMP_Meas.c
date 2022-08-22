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
static uint8_t chx_imp_sample[CHANNEL_NUM*3+CHANNEL_NUM/8*3]; //存储采样值
static uint32_t chx_imp_phyval[CHANNEL_NUM/8][8]; //存储转换后的物理量

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
	int chip,sample_val;
	
	for(chip=0; chip<CHANNEL_NUM/8; chip++){
		
		sample_val=chx_imp_sample[3+3*chx+27*chip];//提取通道采样值
		//计算
		chx_imp_phyval[chip][chx]=chip << 8 | chx;
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
		ADS1299_IMPMeas_Control(1);//开始一次阻抗测量
		SYS_Event &= ~CHX_IMP_START; //!< 清除前序事件
		
		return CHX_IMPING;
	}
	
	else if ( SYS_Event&CHX_IMP_REDY )
	{ 
		ADS1299_ReadResult((uint8_t*)chx_imp_sample); //读取阻抗采样值
		ADS1299_IMPMeas_Control(0); //停止采样
		convert_to_phy_val(chx_process);//转换为物理量
		
		for(chip=0; chip<CHANNEL_NUM/8; chip++){
			int chx_Real_Num = chx_process+8*chip; //换算成实际通道属性编号
			App_WriteAttr(CHX_IMP,chx_Real_Num,chx_imp_phyval[chip][chx_process]);//写入通道属性
		}
		
		SYS_Event &= ~CHX_IMP_REDY; //!< 清除前序事件
		
		return CHX_IMP_CPL;
	}
	
	return CHX_IMPING;
}
 
 