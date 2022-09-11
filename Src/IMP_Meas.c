/**
 * @file    imp_meas.c
 * @author  gjmsilly
 * @brief   MicroEEG �迹������
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
//static uint8_t imp_sample[CHANNEL_NUM*3+CHANNEL_NUM/8*3]; //�ݴ�����ͨ���迹���ԭʼ����ֵ
uint32_t chx_imp_phyval[CHANNEL_NUM/8][8]; //�洢ת�����������

 /************************************************************************
 * GLOBAL VARIABLES
 */
uint8_t imp_sample[CHANNEL_NUM*3+CHANNEL_NUM/8*3]; //�ݴ�����ͨ���迹���ԭʼ����ֵ
uint32_t chx_imp_sample[CHANNEL_NUM/8][8]; //�洢��ȡ���ͨ������ֵ

uint8_t* pimp_sample = imp_sample;
uint32_t* pchx_imp_sample = chx_imp_sample[0]; 

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
	
	//ͨ�������ź�����
	//[3:2]=00(6nA),01(24nA),**10(6uA),11(24uA)
	//[1:0]=00(DC),**01(7.8Hz),10(31.2Hz)
	ADS1299_WriteREG(0,ADS1299_REG_LOFF,0x09);				//[3:2]=00(6nA),01(24nA),10(6uA),11(24uA); [1:0]=01(7.8Hz),10(31.2Hz)
	ADS1299_WriteREG(0,ADS1299_REG_LOFFSENSN,1<<chx_en);
	ADS1299_WriteREG(0,ADS1299_REG_LOFFSENSP,1<<chx_en);
	ADS1299_WriteREG(0,ADS1299_REG_BIASSENSN,0x00);
	ADS1299_WriteREG(0,ADS1299_REG_BIASSENSP,0x00);
	ADS1299_WriteREG(0,ADS1299_REG_CONFIG3,0xec);
	ADS1299_WriteREG(0,ADS1299_REG_CONFIG4,0x02);
	ADS1299_WriteREG(0,ADS1299_REG_MISC1,0x20); // SRB1�պ�
	
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
	int chip;
	uint32_t sample_val;
	
	for( chip=0; chip<CHANNEL_NUM/8; chip++ )
	{
		sample_val = chx_imp_sample[chip][chx];

		//sample_val = 4.5*sample_val/8388608;//������ʵ��ѹ
		//chx_imp_phyval[chip][chx]= sample_val/6*1000; //K��
		
		chx_imp_phyval[chip][chx] = sample_val/11;//��λ�� ����
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
		ADS1299_Sampling_Control(1);//��ʼ����
		SYS_Event &= ~CHX_IMP_START; //!< ���ǰ���¼�
		return CHX_IMPING;
	}
	
	else if ( SYS_Event&CHX_IMP_DONE )
	{ 
		
		ADS1299_Sampling_Control(0); //ֹͣ����
		convert_to_phy_val(chx_process);//ת��Ϊ������
		
		for( chip=0; chip<CHANNEL_NUM/8; chip++ )
		{
			int chx_Real_Num = chx_process+8*chip; //�����ʵ��ͨ�����Ա��
			App_WriteAttr( CHX_IMP, chx_Real_Num, chx_imp_phyval[chip][chx_process] );//д��ͨ������
		}

		SYS_Event &= ~CHX_IMP_DONE; //!< ���ǰ���¼�
		
		return CHX_IMP_CPL;
	}
	return CHX_IMPING;
}
 
 