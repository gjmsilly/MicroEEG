/**
 * @file    AttrTbl.c
 * @author  gjmsilly
 * @brief   MicroEEG_M1 ���Ա�
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
//!< �����ܱ�
static uint8_t* pattr_offset[ATTR_NUM];		//!< ����ƫ�Ƶ�ַ 
static uint8_t* pchxattr_offset[CHANNEL_NUM][10];		//!< ͨ������ƫ�Ƶ�ַ 

//!< ������Ϣ�� ����
const	uint16_t channelnum = CHANNEL_NUM;

//!< ����״̬������� ����
static bool 	sampling;
static bool		impmeas_en;
static uint8_t	impmeas_mode;

//!< ͨ�Ų����� ����
Dev_PortStat_t dev_portstat; 
uint16_t samplenum = SAMPLENUM;

//!< ���������� ����
static uint32_t cursamprate = SPS_1K;
static uint32_t samprate_tbl[]={SPS_250,SPS_500,SPS_1K,SPS_2K,SPS_4K};
static uint32_t curgain = GAIN_X24;
static uint32_t gain_tbl[]={GAIN_X1,GAIN_X2,GAIN_X4,GAIN_X6,GAIN_X8,GAIN_X12,GAIN_X24};

//!< ͨ���������
static bool chx_en[CHANNEL_NUM];
//static uint32_t chx_gain[CHANNEL_NUM]; //���汾����֧�ֵ����޸ģ����ֺ�ȫ��һ��
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
	 *  ======================== ������Ϣ�� ==============================
	 */
		//!< ����UID  
		.Dev_UID			= {	ATTR_RO,									/* permissions */
											4,												/* datasize */
											(uint32_t*)CPU_UUID_ADDR  /* pAttrValue */
										},
		
		//!< ������ͨ����
		.Dev_ChNum		= { ATTR_RO,
											2,
											(uint32_t*)&channelnum
										},
															
	
	/*
	 *  ===================== ����״̬������� ===========================
	 */
		
		//!< �������� 0-ֹͣ���� 1-��ʼ���� 
		.Sampling			= {	ATTR_RS,							/* permissions */
											1,										/* datasize */
											(uint32_t*)&sampling  /* pAttrValue */
										},
		
		//!< �迹�������� 0-����ģʽ 1-�迹���ģʽ
		.IMPMeas_En		= { ATTR_RS,
											1,
											(uint32_t*)&impmeas_en
										},
											
		//!< �迹��������	0- ���Ҳ���AC���� 1- ��DC���� 2- �����������迹
		.IMPMeas_Mode	= { ATTR_RS,
											1,
										 (uint32_t*)&impmeas_mode
										},			 
										
		//!< �迹����ֵ���ܱ� Ҳ��ѡ���ͨ������-��ͨ���迹ֵ ��ȡ
		.IMPMeas_Val	= { ATTR_RA,
											4*CHANNEL_NUM,
										 (uint32_t*)&chx_imp
										},

	/*
	 *  ======================== ͨ�Ų����� ==============================
	 */
			 
		//!< ��������MAC��ַ  0c-29-ab-7c-00-01 (default) 
		.Dev_MAC				= { ATTR_RO,
												6,
												(uint32_t*)net_param.Phy_Addr
											},
		
		//!< ������ǰIP��ַ 192.168.1.10 (default) 
		.Dev_IP					=	{ ATTR_NV,
												4,
												(uint32_t*)net_param.IP_Addr
											},
			
		//!< ��������״̬
		.Dev_PortStat		= { ATTR_RA,
												1,
												(uint32_t*)&dev_portstat
											},
		
		//!< Ŀ������UDP�˿ں� - 7002 (default)
		.Host_Port			= { ATTR_NV,
												2,
												(uint32_t*)&sn_param[1].UDP_DPORT
											},
		
		//!< ��̫��ÿ����ad������ - 10 (default)
		.SampleNum			= { ATTR_RS,
												1,
												(uint32_t*)&samplenum
											},   

	/*
	 *  ======================== ͨ�Ų����� ==============================
	 */
			 
		//!< ֧�ֵĲ����ʵ�λ		
		.Samprate_tbl		= { ATTR_RO,
												sizeof(samprate_tbl),
												(uint32_t*)&samprate_tbl
											},
		
		//!< ��ǰȫ�ֲ����� 1ksps (default) 
		.CurSamprate		=	{ ATTR_RS,
												4,
												(uint32_t*)&cursamprate
											},
		
		//!< ֧�ֵ����浲λ
		.Gain_tbl				= { ATTR_RO,
												sizeof(gain_tbl),
												(uint32_t*)&gain_tbl
											},
		
		//!< ��ǰȫ������ x24 (default) 
		.CurGain				=	{ ATTR_RS,
												4,
												(uint32_t*)&curgain
											},		
};

/* ͨ�����Ա� */
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
	.pfnReadAttrCB = ReadAttrCB,					//!< �����Իص�����ָ�� 
	.pfnWriteAttrCB = WriteAttrCB					//!< д���Իص�����ָ��
};

//////////////////////////////////////////////////////////////////////////

static pfnAttrChangeCB_t pAppCallbacks; //!< Ӧ�ó���ص�����ָ�� 

/*!
 *  @fn	Ӧ�ó���ע��ص������Ľӿ�
 *
 *	@param Ӧ�ó��������ֵ�仯�ص�����
 *
 *	@return SUCCESS - �ص�����ע��ɹ�
 *					FAILURE - �ص�����ע��ʧ��
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
 *  @fn	    �����Իص�����
 *
 *	@param	InsAttrNum - �������Ա��
 *					CHxNum - ͨ����ţ�ͨ������ר�ã�Ĭ�ϲ���	0xFF��
 *					pValue - ����ֵ ��to be returned��
 *					pLen - ����ֵ��С��to be returned��
 *
 *	@return SUCCESS ��ȡ����ֵ�ɹ�
 *					ATTR_NOT_FOUND ���Բ�����
 */
static uint8_t ReadAttrCB(	uint8_t InsAttrNum,uint8_t CHxNum, 
														uint8_t *pValue, uint8_t *pLen )
{
	uint8_t status = SUCCESS;
	uint8_t *pAttrValue;	//!< ����ֵ��ַ

	if ((InsAttrNum > CHANNEL_NUM ) && ( CHxNum != CHX_NONE ) )
	{
		status = ATTR_NOT_FOUND; //!< ͨ�����Բ�����	
	}
	else if( (InsAttrNum > ATTR_NUM ) && ( CHxNum == CHX_NONE ))
	{
		status = ATTR_NOT_FOUND; //!< ���Բ�����
	}
	
	//!< ������ֵ
	if(status == SUCCESS)
	{
		if( CHxNum == 0xFF ) //ͨ�����Զ�����
		{
			pAttrValue = (uint8_t*)*(uint32_t*)(pattr_offset[InsAttrNum]+2);//!< ����ֵ��ַ����
			*pLen = *(pattr_offset[InsAttrNum]+1); //!< ����ֵ��С���ݣ�ֵ����!��ַ���� 9.13��
		}else //ͨ�����Զ�����
		{
			pAttrValue = (uint8_t*)*(uint32_t*)(pchxattr_offset[CHxNum][InsAttrNum]+2);//!< ����ֵ��ַ����
			*pLen = *(pchxattr_offset[CHxNum][InsAttrNum]+1); //!< ����ֵ��С���ݣ�ֵ����!��ַ���� 9.13��
		}
		
		memcpy(pValue,pAttrValue,*pLen); //!< ����ֵ��ȡ
	}
	
	return status;
	
}

/*!
 *  @fn			д���Իص�����
 *
 *	@param	InsAttrNum - ��д�����Ա��
 *					CHxNum - ͨ����ţ�ͨ������ר�ã�Ĭ�ϲ���	0xFF��
 *					pValue - ��д�����ݵ�ָ��
 *					pLen - ��д�����ݴ�С
 *
 *	@return SUCCESS ��ȡ����ֵ�ɹ�
 *					ATTR_NOT_FOUND ���Բ�����
 */
static uint8_t WriteAttrCB( uint8_t InsAttrNum,uint8_t CHxNum, 
														uint8_t *pValue, uint8_t len )
{
	uint8_t status;
	uint8_t notifyApp=0xFF;	//!< ��־λ - ֪ͨ�ϲ�Ӧ�ó�������ֵ�仯
	uint8_t AttrPermission;	//!< ���Զ�дȨ��	
	uint8_t AttrLen;				//!< ����ֵ��С
	uint8_t *pAttrValue;		//!< ����ֵ��ַ	
	
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
		status = ATTR_NOT_FOUND; //!< ͨ�����Բ�����	
	}
	else if( (InsAttrNum > ATTR_NUM ) && ( CHxNum == CHX_NONE ) )
	{
		status = ATTR_NOT_FOUND; //!< ���Բ�����
	}
	else if(( AttrPermission == ATTR_RO )||( AttrPermission == ATTR_NV ))
	{
		status = ATTR_ERR_RO; //!< ���Բ�����д����
	}
	else if( len!= AttrLen)
	{
		status = ATTR_ERR_SIZE; //!< ��д���ݳ���������ֵ���Ȳ���
	}
	else status = SUCCESS;
	
	//!< д����ֵ��֪ͨ�ϲ�Ӧ�ó���AttrChange_Process��
	if ( status == SUCCESS )
	{
		if( CHxNum == CHX_NONE ) //ͨ������д����
		{
			pAttrValue = (uint8_t*)*(uint32_t*)(pattr_offset[InsAttrNum]+2);//!< ����ֵ��ַ����
		} else //ͨ������д����
		{
			pAttrValue = (uint8_t*)*(uint32_t*)(pchxattr_offset[CHxNum][InsAttrNum]+2);//!< ����ֵ��ַ���� 
		}
		
		memcpy(pAttrValue,pValue,len); //!< ����ֵд��
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
		
		/* ��ͨ��ѡ������ - ȫ������ */
		CHx_Param[i].CHx_ConGain.permissions = ATTR_RO;
		CHx_Param[i].CHx_ConGain.Attrsize = 4;
		CHx_Param[i].CHx_ConGain.pAttrValue = (uint32_t*)&curgain;
	
		/* ��ͨ������ */
		CHx_Param[i].CHx_En.permissions = ATTR_RS;
		CHx_Param[i].CHx_En.Attrsize = 1;
		CHx_Param[i].CHx_En.pAttrValue = (uint32_t*)&chx_en[i];

		/* ��ͨ������ */
		CHx_Param[i].CHx_Gain.permissions = ATTR_RO;
		CHx_Param[i].CHx_Gain.Attrsize = 4;
		CHx_Param[i].CHx_Gain.pAttrValue = (uint32_t*)&curgain;		
		
		/* ��ͨ�������� */
		CHx_Param[i].CHx_Samprate.permissions = ATTR_RO;
		CHx_Param[i].CHx_Samprate.Attrsize = 4;
		CHx_Param[i].CHx_Samprate.pAttrValue = (uint32_t*)&cursamprate;	
		
		/* ��ͨ���缫���������� */
		CHx_Param[i].CHx_Lead_off_En.permissions = ATTR_RS;
		CHx_Param[i].CHx_Lead_off_En.Attrsize = 1;
		CHx_Param[i].CHx_Lead_off_En.pAttrValue = (uint32_t*)&chx_lead_off_en[i];	

		/* ��ͨ���缫����״̬ */
		CHx_Param[i].CHx_Lead_off_stat.permissions = ATTR_RA;
		CHx_Param[i].CHx_Lead_off_stat.Attrsize = 1;
		CHx_Param[i].CHx_Lead_off_stat.pAttrValue = (uint32_t*)&chx_lead_off_stat[i];		
		
		/* ��ͨ���缫������ֵ */
		CHx_Param[i].CHx_Lead_off_TH.permissions = ATTR_RS;
		CHx_Param[i].CHx_Lead_off_TH.Attrsize = 4;
		CHx_Param[i].CHx_Lead_off_TH.pAttrValue = (uint32_t*)&chx_lead_off_th[i];	
		
		/* ��ͨ���Ƿ�ӵ� */
		CHx_Param[i].CHx_GND.permissions = ATTR_RS;
		CHx_Param[i].CHx_GND.Attrsize = 1;
		CHx_Param[i].CHx_GND.pAttrValue = (uint32_t*)&chx_gnd[i];	

		/* ��ͨ���Ƿ���빲ģ */
		CHx_Param[i].CHx_BIASOUT.permissions = ATTR_RS;
		CHx_Param[i].CHx_BIASOUT.Attrsize = 1;
		CHx_Param[i].CHx_BIASOUT.pAttrValue = (uint32_t*)&chx_biasout[i];

		/* ��ͨ���迹ֵ */
		CHx_Param[i].CHx_IMP.permissions = ATTR_RA;
		CHx_Param[i].CHx_IMP.Attrsize = 4;
		CHx_Param[i].CHx_IMP.pAttrValue = (uint32_t*)&chx_imp[i];
		
		/* ͨ������ֵ����Ĭ��ֵ */
		chx_en[i]=1;
		//chx_gain[i]=curgain;
		//chx_sampletare[i]=cursamprate;
		chx_lead_off_en[i]=0;
		chx_lead_off_stat[i]=0;
		chx_lead_off_th[i]=0;
		chx_gnd[i]=0;
		chx_biasout[i]=0;
		chx_imp[i]=1000;	//1M�� by default
		
		/* ������ַӳ�� */
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
 *  @fn	���Ա��ʼ��
 *	@brief ��ʼ�����Ա�ӿں�������д�ص���
 */
void Attr_Tbl_Init()
{
	/* ����̫��֡Э�����ע���д�ص����� */
	protocol_RegisterAttrCBs(&attr_CBs);
	
	/* ��ʼ��ͨ�����Ա� */
	CHx_Tbl_Init();
	
	/* ����ͨ�����Եĵ�ַӳ�� */		
	
	//!< ���Ե�ַƫ��ӳ���ϵ   
	//!< pattr_offset[n]�������Ե������׵�ַ ��λ��ͨ���±����ƫ�Ʒ���
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
 *  @fn			�����Ժ��� �����ϲ�Ӧ�û�ȡ���ԣ�
 *
 *	@param	InsAttrNum - ��д�����Ա��
 *					pValue - ��д�����ݵ�ָ��
 *
 *	@return SUCCESS ��ȡ����ֵ�ɹ�
 *					ATTR_NOT_FOUND ���Բ�����
 */
uint8_t App_GetAttr(uint8_t InsAttrNum, uint8_t CHxNum, uint32_t *pValue)
{
	uint8_t ret = SUCCESS;
	
	if(CHxNum == CHX_NONE) //Ӧ�ò�ͨ�����Զ�����
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
	//else //ͨ�����Զ�����
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
 *  @fn			д���Ժ��� �����ϲ�Ӧ���޸�����ֵ��
 *
 *	@param	InsAttrNum - ��д�����Ա��
 *					Value - ��д������
 *
 *	@return SUCCESS д����ֵ�ɹ�
 *					ATTR_NOT_FOUND ���Բ�����
 */
uint8_t App_WriteAttr(uint8_t InsAttrNum, uint8_t CHxNum, uint32_t Value)
{
	uint8_t ret = SUCCESS;
	if(CHxNum == CHX_NONE) //Ӧ�ò�ͨ�����Զ�����
	{	
		switch(InsAttrNum)
		{
			case SAMPLING:		//!< Ӧ�ò��޸����ڲ�������
				sampling = (uint8_t)Value;
				break;	
			
			case CURGAIN:			//!< Ӧ�ò��޸�ȫ���������� 
				curgain = (uint32_t)Value;
				break;
			
			case CURSAMPLERATE: //!< Ӧ�ò��޸�ȫ�ֲ���������
				cursamprate = (uint32_t)Value;
			break;
		}
	}
	else
	{
		switch(InsAttrNum)
		{
			case CHX_EN:		//!< Ӧ�ò��޸�ͨ������
				chx_en[CHxNum] = (uint8_t)Value; 
			break;
			
			case CHX_IMP:
				chx_imp[CHxNum] = (uint32_t)Value;
			break;
		}
	}
  return ( ret );	
}