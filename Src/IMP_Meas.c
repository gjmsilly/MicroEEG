/**
 * @file    imp_meas.c
 * @author  gjmsilly
 * @brief   MicroEEG �迹������
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
static uint8_t chx_imp_sample[CHANNEL_NUM*3+CHANNEL_NUM/8*3]; //�洢����ֵ
static uint32_t chx_imp_phyval[CHANNEL_NUM/8][8]; //�洢ת�����������

 /************************************************************************
 * LOCAL FUNCTIONS
 */
/*!
 *  @fn     imp_config
 *
 *  @brief  �迹���ADS1299�Ĵ�������
 *  				ÿ��ʹ��һ��ͨ�����Ը�ͨ����ӵ���Դ��
 *	
  *	@param	chx_en	ʹ��ͨ�����(0~7)
 */
static void imp_config(uint8_t chx_en)
{
	int i;
	
	//�ر�����ͨ��
	for( i=0; i<8; i++ ){
		ADS1299_Channel_Control(0,i,0);
	}
	//����ʹ��
	ADS1299_Channel_Control(0,chx_en,1);
}

/*!
 *  @fn     convert_to_phy_val
 *
 *  @brief  �迹����ֵת��Ϊ������ ��λk��
 *	
  *	@param	chx	ͨ�����(0~7)
 */
static void convert_to_phy_val(uint8_t chx)
{
	int chip,sample_val;
	
	for(chip=0; chip<CHANNEL_NUM/8; chip++){
		
		sample_val=chx_imp_sample[3+3*chx+27*chip];//��ȡͨ������ֵ
		//����
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
		imp_config(chx_process);//ͨ���Ĵ�������
		ADS1299_IMPMeas_Control(1);//��ʼһ���迹����
		SYS_Event &= ~CHX_IMP_START; //!< ���ǰ���¼�
		
		return CHX_IMPING;
	}
	
	else if ( SYS_Event&CHX_IMP_REDY )
	{ 
		ADS1299_ReadResult((uint8_t*)chx_imp_sample); //��ȡ�迹����ֵ
		ADS1299_IMPMeas_Control(0); //ֹͣ����
		convert_to_phy_val(chx_process);//ת��Ϊ������
		
		for(chip=0; chip<CHANNEL_NUM/8; chip++){
			int chx_Real_Num = chx_process+8*chip; //�����ʵ��ͨ�����Ա��
			App_WriteAttr(CHX_IMP,chx_Real_Num,chx_imp_phyval[chip][chx_process]);//д��ͨ������
		}
		
		SYS_Event &= ~CHX_IMP_REDY; //!< ���ǰ���¼�
		
		return CHX_IMP_CPL;
	}
	
	return CHX_IMPING;
}
 
 