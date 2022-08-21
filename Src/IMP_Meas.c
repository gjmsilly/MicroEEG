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
static uint8_t chx_imp_val[CHANNEL_NUM*3+CHANNEL_NUM/8*3]; //TODO �洢����ֵ

 /************************************************************************
 * LOCAL FUNCTIONS
 */
/*!
 *  @fn     imp_config
 *
 *  @brief  �迹���ADS1299�Ĵ�������
 *
 *  ÿ��ʹ��һ��ͨ�����Ը�ͨ����ӵ���Դ��ʹ��һ�β�����
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


 /************************************************************************
 * FUNCTIONS
 */
uint32_t imp_control(uint8_t chx_process)
{

		if( SYS_Event&CHX_IMP_START )
		{	
			imp_config(chx_process);//ͨ���Ĵ�������
			ADS1299_IMPMeas_Control(1);//��ʼһ���迹����
			SYS_Event &= ~CHX_IMP_START; //!< ���ǰ���¼�
			
			return CHX_IMPING;
		}
		
		else if ( SYS_Event&CHX_IMP_REDY )
		{ 
			/* ��ȡ�迹����ֵ��ת��Ϊ������ */
			ADS1299_ReadResult((uint8_t*)chx_imp_val); //��ȡ�迹����ֵ
			ADS1299_IMPMeas_Control(0); //ֹͣ����
			//TODOת��Ϊ������
			App_WriteAttr(CHX_IMP,chx_process,chx_imp_val[chx_process]);//д��ͨ������
			SYS_Event &= ~CHX_IMP_REDY; //!< ���ǰ���¼�
			
			return CHX_IMP_CPL;
		}
		
		return CHX_IMPING;
}
 
 