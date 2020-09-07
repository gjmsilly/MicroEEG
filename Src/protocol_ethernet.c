/**
 * @file    protocol_ethernet.c
 * @author  gjmsilly
 * @brief   ��̫��֡Э�����TCP��������UDP�����
 * @version 1.0
 * @date    2020-09-02
 * @ref			https://www.amobbs.com/forum.php?mod=viewthread&tid=5668532
 * @copyright (c) 2020 gjmsilly
 *
 */
 
/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include "compiler.h"
#include "ooc.h"
#include "simple_fsm.h"
#include "protocol_ethernet.h"
#include "main.h"
#include "microEEG_misc.h"
#include "ads1299.h"
#include "AttritubeTable.h"

/*********************************************************************
 * LOCAL VARIABLES
 */

/* ָ���� */
const uint8_t	DummyIns = 0x00;						//<! ��ָ��
const uint8_t	CAttr_Read = 0x01;					//<! ��һ����ͨ����
const uint8_t	CAttr_Write = 0x10;					//<! дһ����ͨ����
const uint8_t	ChxAttr_Read = 0x02;				//<! ��һ��ͨ������
const uint8_t	ChxAttr_Write = 0x20;				//<! дһ��ͨ������

/* ����ͨ��������TCP�˿ڣ� */
// ��λ��->�豸 ָ�����
static uint8_t	TCP_Recv_FH = 0xAC;				//<! TCP����֡ͷ
static uint8_t	TCP_Recv_FT = 0xCC;				//<! TCP����֡β

// �豸->��λ�� �ظ�
static uint8_t	TCP_Send_FH = 0xA2;				//<! TCP����֡ͷ
static uint8_t	TCP_Send_FT = 0xC2;				//<! TCP����֡β

static uint8_t	TCP_SUCCESS = 0x00;				//<! ָ���������
static uint8_t	TCPERR_Attr_RO = 0x01;		//<! ָ����󣺶�ֻ������д����
static uint8_t	TCPERR_Attr_LEN = 0x02;		//<! ָ�����дָ�����ݳ��ȴ���

/*******************************************************************
 * GLOBAL VARIABLES
 */
  
uint8_t TCP_Rx_Buff[TCP_Rx_Buff_Size];		//!< TCP�������ݻ����� 
uint8_t TCP_Tx_Buff[TCP_Tx_Buff_Size];		//!< TCP�������ݻ�����
uint8_t UDP_Tx_Buff[UDP_Tx_Buff_Size];		//!< UDP�������ݻ�����
uint8_t	DataLength1;// debug
uint8_t*	pDataLength1;// debug
uint8_t FrameHeaderBuff;
uint8_t	TCP_RPY_Size=0xFF;				//!< TCP�ظ�֡�� - ����֪ͨw5500_app
uint8_t	Attr_Change_Num;		//!< ���Ա�ֵ�仯�����Ժ� - ����֪ͨAttrChangeEvt
/*******************************************************************
 * TYPEDEFS
 */
 
/*! @brief	������ʱ1s��״̬�� delay_1s
 *!					ͬʱ�г����в���	wCounter
 */
simple_fsm
( 
		/* ����״̬������ */
		delay_1s,
    
    /* ���屾״̬�����õ������β� */ 
    def_params
		(
        uint32_t wCounter;                  //!< a uint32_t counter
    )
)


/*! @brief	����TCPЭ�����ݴ����״̬�� TCP_Process
 *!       	ͬʱ�г����в���
 *!	@note		��״̬����w5500_app.c -> TCPServer_Service����
 */
simple_fsm
( 
		TCP_Process,
    def_params
		(
				uint8_t FrameLength;				//!< ��Ч֡��
				uint8_t DataLength;					//!< ����֡��
				uint8_t InsNum;							//!< ָ����							
				uint8_t InsAttrNum;					//!< ָ���������Ա��
				uint8_t	AttrPermission;			//!< ���Զ�дȨ��
				uint8_t ChxNum;							//!< ָ������ͨ��
				uint8_t ERR_NUM;						//!< ������
				uint8_t* _OP_;							//!< ����������ָ�� // 9.3 �������������� bool int8 int32 ???
				uint32_t* _RPY_;						//!< ����ֵָ�루��������ֵ��ȡ��TCP�ظ���
				
				fsm(delay_1s) fsmDelay;			//!< sub fsm delay_1s
    )
)


/*********************************************************************
 * FUNCTIONS
 */
/*
 *  ============================ ��ʱ1s ==============================
 */ 
/*! @brief define the fsm initialiser for fsm delay_1s
 *! @param - wCounter  an uint32_t value for the delay count
 */
fsm_initialiser
(
		delay_1s,														 //!< ״̬������
   
	 /*! ��״̬����ʼ���������β��б� */
    args
		(           
        uint32_t wCounter               //!< delay count
    ))
		/*! ��״̬����ʼ������ */
		init_body 
		(
				this.wCounter = wCounter;       //!< initialiser the fsm paramter
		)
/* End of fsm initialization */


/*! @brief Implement the fsm: delay_1s
 *         This fsm only contains one state.
 */
fsm_implementation(delay_1s)
    def_states(DELAY_1S)                //!< list all the states used in the FSM

    /* the body of the FSM: delay_1s */
    body 
		(
        state
				(  
						DELAY_1S,               		//!< state: DELAY_1s
            if (!this.wCounter) 
						{
                fsm_cpl();              //!< FSM is completed
            }
            this.wCounter--;
        )
    )
/* End of fsm implementation */

/*
 *  ======================== TCP֡Э����� ============================
 */ 						
/*! @brief define the fsm initialiser for fsm TCP_Process
 */
fsm_initialiser(TCP_Process)
    init_body ()


/*! @brief Implement the fsm: TCP_Process
 */
fsm_implementation(TCP_Process)

    /*! list all the states used in the FSM */
    def_states(FRAME_SEEKHEAD,FRAME_LENGTH,FRAME_CHK,FRAME_INS,FRAME_ERR,FRAME_RPY)

    body(
				/*! TCP���ղ��- ֡ͷ��ȡ */
        state(FRAME_SEEKHEAD,
		
						FrameHeaderBuff = TCP_Rx_Buff[0];
						if (FrameHeaderBuff == TCP_Recv_FH) 
						{
							this.FrameLength = 0;			//!< ��Ч֡��
							this.DataLength =0;				//!< ����֡��
							this.InsNum = 0;					//!< ָ����							
							this.InsAttrNum = 0x00; 	//!< ָ���������Ա��
							this.ChxNum = 0x00;				//!< ָ������ͨ��
							this.ERR_NUM = 0x00;			//!< ������
							
							printf("S:LENGTH\r\n");
							transfer_to(FRAME_LENGTH);							//!< ��ת֡���Ȼ�ȡ
						}						
       )
						
				/*! ֡���Ȼ�ȡ */												
				state(FRAME_LENGTH,
						
						if ((this.FrameLength = TCP_Rx_Buff[1]) != 0xFF) 
						{
							printf("S:CHK\r\n");
							transfer_to(FRAME_CHK);									//!< ��ת֡���						
						}
       )						
			 
        /*! ֡��� */	
				state(FRAME_CHK,
						
							if (this.FrameLength == 3) //!< �����Բ��� - ��Ч֡����3
							{ 
								this.InsNum = TCP_Rx_Buff[2];
								this.InsAttrNum = TCP_Rx_Buff[3];
								this.ChxNum = TCP_Rx_Buff[4];
							}
							else if(this.FrameLength == 1);	//!< ��ָ�� - ��Ч֡����1
							else //!< д���Բ��� - ��Ч֡>4 ������
							{
								this.InsNum = TCP_Rx_Buff[2];
								this.DataLength = this.FrameLength-3;
								this.InsAttrNum = TCP_Rx_Buff[3];
								this.ChxNum = TCP_Rx_Buff[4];
								this._OP_ = TCP_Rx_Buff+5;
							}
							
							if(TCP_Rx_Buff[this.FrameLength+2]== TCP_Recv_FT)//!< ֡β���
							{								
								printf("S:INS\r\n");								
								transfer_to(FRAME_INS);										//!< ��תָ�����
							}
							else //!< ֡β���ʧ��
							{
								printf("S:ERR-CTFailed\r\n");
								transfer_to(FRAME_SEEKHEAD);							//!< �˰�����
							}																	
       )						
				
				/*! ָ����� */						
        state(FRAME_INS,
            switch (this.InsNum)
            {
            	case CAttr_Read: //!< ����ͨ����
								this._RPY_ = pattr_offset[this.InsAttrNum];	//!< ����ֵ��ַ����
								this.ERR_NUM = TCP_SUCCESS;
								transfer_to(FRAME_ERR);        		
								break;
            	
							case CAttr_Write: //!< д��ͨ����
								//!< ���Զ�дȨ�޼��
								this.AttrPermission = *(pattr+this.InsAttrNum*6);
								if(this.AttrPermission == ATTR_RO ||		\
										this.AttrPermission == ATTR_NV) 
									{
										this.ERR_NUM = TCPERR_Attr_RO; //!< ָ����󣺶�ֻ������д����
										transfer_to(FRAME_ERR);
									}
								//!< ָ�����ݳ��ȼ��
								else if(*(pattr+this.InsAttrNum*6+1)!= this.DataLength)
									{
										this.ERR_NUM = TCPERR_Attr_LEN;//<! ָ�����дָ�����ݳ��ȴ���
										transfer_to(FRAME_ERR);
									}
								else
									{
										this.ERR_NUM = TCP_SUCCESS;
										this._RPY_ = pattr_offset[this.InsAttrNum];	//!< ����ֵ��ַ����
										memcpy((uint8_t*)this._RPY_,(TCP_Rx_Buff+5),this.DataLength); //!< д����
										transfer_to(FRAME_ERR);
									}
									           		
								break;
							
            	default:
								printf("WrongINS\r\n");
								transfer_to(FRAME_SEEKHEAD);							//!< �˰�����
            		break;
            }						
        )
						
				/*! �������� */				
        state(FRAME_ERR,
						
								TCP_Tx_Buff[2]=this.ERR_NUM; 
								
								printf("RPY\r\n");
								transfer_to(FRAME_RPY);										//!< ��ת�ظ�  
        )

				/*! TCP�ظ���� */	
        state(FRAME_RPY,
				
						TCP_Tx_Buff[0] = TCP_Send_FH; //!< ֡ͷ
						
						if(this.ERR_NUM == TCP_SUCCESS) //!< ��ȷ����
						{
							this.DataLength=*(pattr+this.InsAttrNum*6+1); //!< ����֡��
							
							this.FrameLength=this.DataLength+2;  
							TCP_Tx_Buff[1]= this.FrameLength; //!< ��Ч֡��						
							TCP_Tx_Buff[3] = this.InsAttrNum; //!< �ظ����ͣ����Ա�ţ�						
							memcpy((TCP_Tx_Buff+4),(uint8_t*)this._RPY_,this.DataLength); //!< ����֡
						}
						else //!< �������
						{
							this.FrameLength=1; //!< ��Чֻ֡��������
							
						}
											
						TCP_Tx_Buff[this.FrameLength+2]=TCP_Send_FT; //!< ֡β
						
						TCP_RPY_Size = this.FrameLength+3; //!< ֪ͨw5500_app�ظ�Ŀ������
						
						printf("S:SEEK\r\n");
						transfer_to(FRAME_SEEKHEAD);
						           
        )       
    )
/* End of fsm implementation */								
						
/* FSM calling */
static fsm( TCP_Process ) s_fsmProtocolProcess;

void ProtocolProcessFSMInit(void)
{
		if (NULL == init_fsm(TCP_Process, &s_fsmProtocolProcess,
			)) {      //!< String Length
			 /* failed to initialize the FSM, put error handling code here */
	 }
}

void ProtocolProcessFSM(void)
{
		if (fsm_rt_cpl == call_fsm( TCP_Process, &s_fsmProtocolProcess )) 
		{
               /* fsm is complete, do something here */
    }
}
						


/*
void BT_DataFrameBuilder (void)
{
	static unsigned char lChecksumChar;
	unsigned char i;
	
	//header
	BT_USART_BUFF[0] = 0xAB;
	
	//TimeStamp
	BT_USART_BUFF[1] = *pcCurTimeStamp;
	BT_USART_BUFF[2] = *(pcCurTimeStamp+1);
	BT_USART_BUFF[3] = *(pcCurTimeStamp+2);
	BT_USART_BUFF[4] = *(pcCurTimeStamp+3);
	
	//Event Tag
	BT_USART_BUFF[5] = *(pcCurEventTag);
	BT_USART_BUFF[6] = *(pcCurEventTag+1);
	if(BT_USART_BUFF[5]|BT_USART_BUFF[6])
	{
		*(pcCurEventTag) = 0;
		*(pcCurEventTag+1) = 0;
	}

	memcpy(BT_USART_BUFF+7,resultval+4,24);


	
	
	lChecksumChar = 0;
	for(i=1;i<31;i++)
	{
		lChecksumChar += BT_USART_BUFF[i];
	}
	
	BT_USART_BUFF[31] = lChecksumChar;
	BT_USART_BUFF[32] = 0xCB;

}

void ProtocolDataService_Init(void)
{
	
}
void ProtocolDataService_Process(void)
{
	

	
	switch (SYS_Status)
  {
  	case SYS_STATUS_ACQUIRING:
		{	
			if(NewSample)
			{
				NewSample = 0;
				SampleNum++;
				if(SampleNum >= 125)
				{
					SampleNum = 0;
					LL_GPIO_TogglePin(COP_LED_GPIO_Port, COP_LED_Pin);
				}
		
				BT_DataFrameBuilder();
				while(BT_USART_Mutex)  // #### While Mutex
				{}
				BT_USART_DMASend((uint32_t)BT_USART_BUFF,33);
				
			}
			
			
			
  		break;
		}
  	case SYS_STATUS_BTCONNECTIONLOST:
  		break;
		case SYS_STATUS_SPECIALMODE:
			
  		break;
  	default:
  		break;
  }
}

inline void ProtocolDataService_Process_INT(void)
{
	NewSample = 1;
}
						
*/



