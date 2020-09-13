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
#include "SimpleInsQueue.h"
#include "main.h"
#include "microEEG_misc.h"
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

/*******************************************************************
 * GLOBAL VARIABLES
 */
  
uint8_t TCP_Rx_Buff[TCP_Rx_Buff_Size];		//!< TCP�������ݻ����� 
uint8_t TCP_Tx_Buff[TCP_Tx_Buff_Size];		//!< TCP�������ݻ�����
uint8_t UDP_Tx_Buff[UDP_Tx_Buff_Size];		//!< UDP�������ݻ�����

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
				uint8_t	FrameHeader;				//!< ֡ͷ
				uint8_t FrameLength;				//!< ��Ч֡��
				uint8_t DataLength;					//!< ����֡��
				uint8_t InsNum;							//!< ָ����							
				uint8_t InsAttrNum;					//!< ָ���������Ա��
				uint8_t ChxNum;							//!< ָ������ͨ��
				uint8_t ERR_NUM;						//!< ������
				uint8_t *pDataLength;				//!< ����֡��ָ�루���ص��ã�
				uint8_t* _OP_;							//!< ����������ָ�루д�ص��ã�
				
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
    def_states(FRAME_SEEKHEAD,FRAME_LENGTH,FRAME_CHK,FRAME_INS,FRAME_RPY)

    body(
				/*! TCP���ղ��- ֡ͷ��ȡ */
        state(FRAME_SEEKHEAD,
		
						this.FrameHeader = TCP_Rx_Buff[0];
						if (this.FrameHeader == TCP_Recv_FH) 
						{
							/* ��λ */
							this.FrameLength = 0xFF;	//!< ��Ч֡��
							this.DataLength =0xFF;		//!< ����֡��
							this.InsNum = 0xFF;				//!< ָ����							
							this.InsAttrNum = 0xFF; 	//!< ָ���������Ա��
							this.ChxNum = 0xFF;				//!< ָ������ͨ������ͨ������ - 0xFF��
							this.ERR_NUM = 0xFF;			//!< ������
							
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
								memset(TCP_Rx_Buff,0xFF,sizeof(TCP_Rx_Buff_Size));//!< ���TCP���ջ�����
							}																	
       )						
				
				/*! ָ����� */						
        state(FRAME_INS,
            switch (this.InsNum)
            {
            	case CAttr_Read: //!< ����ͨ����
								
								//!< �����Իص�
								this.ERR_NUM = pattr_CBs->pfnReadAttrCB(	this.InsAttrNum,this.ChxNum, \
																													(TCP_Tx_Buff+4),this.pDataLength);
								this.FrameLength = *(this.pDataLength)+2;								
								transfer_to(FRAME_RPY);        		
								break;
            	
							case CAttr_Write: //!< д��ͨ����
								
								//!< д���Իص�
								this.ERR_NUM = pattr_CBs->pfnWriteAttrCB(	this.InsAttrNum,this.ChxNum, \
																													this._OP_,this.DataLength);  										
								
								if( this.ERR_NUM == SUCCESS )
								{
									//!< ����һ�����ڻظ� 
									memcpy((TCP_Tx_Buff+4),this._OP_,this.DataLength);
									this.FrameLength = this.DataLength+2;
								}
								
								transfer_to(FRAME_RPY);									           		
								break;
							
            	default:
								printf("WrongINS\r\n");
								transfer_to(FRAME_SEEKHEAD);							//!< �˰�����
								memset(TCP_Rx_Buff,0xFF,sizeof(TCP_Rx_Buff_Size));//���TCP���ջ�����
            		break;
            }						
        )
						

				/*! TCP�ظ���� */	
        state(FRAME_RPY,
				
						TCP_Tx_Buff[0] = TCP_Send_FH;					//!< ֡ͷ
						if( this.ERR_NUM == SUCCESS )
						{
							TCP_Tx_Buff[1] = this.FrameLength;	//!< ��Ч֡
							TCP_Tx_Buff[2] = this.ERR_NUM;			//!< ������											
							TCP_Tx_Buff[3] = this.InsAttrNum; 	//!< �ظ����ͣ����Ա�ţ�						
						}
						else //!< �������
						{
							this.FrameLength=1; //!< ��Чֻ֡��������
							TCP_Tx_Buff[1] = this.FrameLength;	//!< ��Ч֡							
						}
											
						TCP_Tx_Buff[this.FrameLength+2]=TCP_Send_FT; //!< ֡β
												
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
						