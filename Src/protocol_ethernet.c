/**
 * @file    protocol_ethernet.c
 * @author  gjmsilly
 * @brief   ��̫������Э�飨TCP��������UDP��� ʵ�֣�
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
#include "simpleInsQueue.h"
#include "main.h"
#include "microEEG_misc.h"
#include "ads1299.h"
#include "AttritubeTable.h" 

/*******************************************************************
 * GLOBAL VARIABLES
 */
InsQUEUE TCPInsQueue;   		//!< ���� - TCP���յĿ���ָ��
uint8_t FrameHeaderBuff;    

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
extern_simple_fsm
( 
		TCP_Process,
    def_params
		(
				uint8_t FrameLength;				//!< ��Ч֡��
				uint8_t InsNum;							//!< ָ����							
				uint8_t InsAttrNum;					//!< ָ�����õ�ַ�����Ա�ƫ������
				uint8_t ChxNum;							//!< ָ������ͨ��
				uint8_t ERR_NUM;						//!< ������
				uint8_t	OP[4];							//!< ���������� // 9.3 �������������� bool int8 int32 ???
				uint32_t* RPY;							//!< �ظ�����
				
				bool InsNeedReply;          //!< ָ����Ҫ�ظ�
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
 *  ========================== TCP��� ==============================
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
				/*! ֡ͷ��ȡ */
        state(FRAME_SEEKHEAD,
    						
						FrameHeaderBuff = DeQueue(&TCPInsQueue);   //!< ֡ͷ����
						if (FrameHeaderBuff == TCP_Recv_FH) 
						{
							this.FrameLength = 0;			//!< ��Ч֡��
							this.InsNum = 0;					//!< ָ����							
							this.InsAttrNum = 0x00; 	//!< ָ�����õ�ַ�����Ա�ƫ������
							this.ChxNum = 0x00;				//!< ָ������ͨ��
							this.ERR_NUM = 0x00;			//!< ������
							
							printf("S:LENGTH\r\n");
							transfer_to(FRAME_LENGTH);							//!< ��ת��Ч֡����ȡ
						}						
       )
						
				/*! ֡���Ȼ�ȡ */												
				state(FRAME_LENGTH,
						
						if ((this.FrameLength = DeQueue(&TCPInsQueue)) != 0xFF) 
						{

							printf("S:CHK\r\n");
							transfer_to(FRAME_CHK);									//!< ��ת֡���
							
						}
       )						

        /*! ֡��� */	
				state(FRAME_CHK,
						
						if (GetQueueLength(&TCPInsQueue) >= this.FrameLength +1)   //!< �ȴ���֡��� 
						{
							if (this.FrameLength == 3) //!< �����Բ���
							{
								this.InsNum = DeQueue(&TCPInsQueue);
								this.InsAttrNum = DeQueue(&TCPInsQueue);
								this.ChxNum = DeQueue(&TCPInsQueue);
							}
							else if(this.FrameLength == 0);	//!< ��ָ��
							else //!< д���Բ���
							{
								this.InsNum = DeQueue(&TCPInsQueue);
								this.InsAttrNum = DeQueue(&TCPInsQueue);
								this.ChxNum = DeQueue(&TCPInsQueue);
								
								for (this.i=0;this.i<(this.FrameLength-3);this.i++)
								{
									this.OP[this.i] = DeQueue(&TCPInsQueue);
								}
							}

							if(TCP_Recv_FT == DeQueue(&TCPInsQueue))     //!< ֡β���
							{
								
								printf("S:INS\r\n");
								
								transfer_to(FRAME_INS);										//!< ��תָ�����
							}
							else     //!< ֡β���ʧ��
							{
								//this.ERR_NUM = 0x30;
								printf("S:ERR-CTFailed\r\n");
								transfer_to(FRAME_ERR);
							}							
						}											
       )						
				
				/*! ָ����� */						
        state(FRAME_INS,
            switch (this.InsNum)
            {
            	case 0x01:   																//!< ����ͨ����
								this.RPY = ((uint8_t*)pattr_tbl+2*InsAttrNum+1);						
								transfer_to(FRAME_RPY);										//!< ��ת�ظ�
            		
								break;
            	
							case 0x02:   																//!< д��ͨ����
								
								transfer_to(FRAME_RPY);										//!< ��ת�ظ�
            		
								break;
							
            	default:
//								this.ERR_NUM = 0x11;
								printf("WrongINS\r\n");
								transfer_to(FRAME_ERR);
            		break;
            }						
        )
						
				
        state(FRAME_ERR,
						
						transfer_to(FRAME_SEEKHEAD);
        )


        state(FRAME_RPY,
           
						
						
						switch (this.INS_NUM)
            {
            	case 0x80:   // Start Acq
								
							// preform reply and Starting Acq
//								BT_USART_BUFF[0] = T_FrameHeader;
//								BT_USART_BUFF[1] = 1;
//								BT_USART_BUFF[2] = this.ERR_NUM;
//								BT_USART_BUFF[3] = this.ERR_NUM;
//								BT_USART_BUFF[4] = T_FrameTail;
//								BT_USART_DMASend((uint32_t)BT_USART_BUFF, 5);          		
								break;
            	
							case 0x90:   // Stop Acq
								
//								BT_USART_BUFF[0] = T_FrameHeader;
//								BT_USART_BUFF[1] = 1;
//								BT_USART_BUFF[2] = this.ERR_NUM;
//								BT_USART_BUFF[3] = this.ERR_NUM;
//								BT_USART_BUFF[4] = T_FrameTail;
//								BT_USART_DMASend((uint32_t)BT_USART_BUFF, 5);  
								break;
            	
							case 0x82:   // Start Resistantance Measuring
								
//								BT_USART_BUFF[0] = T_FrameHeader;
//								BT_USART_BUFF[1] = 1;
//								BT_USART_BUFF[2] = this.ERR_NUM;
//								BT_USART_BUFF[3] = this.ERR_NUM;
//								BT_USART_BUFF[4] = T_FrameTail;
//								BT_USART_DMASend((uint32_t)BT_USART_BUFF, 5);
            		break;            	
							
							case 0x92:   // Stop Resistantance Measuring
								
//								BT_USART_BUFF[0] = T_FrameHeader;
//								BT_USART_BUFF[1] = 1;
//								BT_USART_BUFF[2] = this.ERR_NUM;
//								BT_USART_BUFF[3] = this.ERR_NUM;
//								BT_USART_BUFF[4] = T_FrameTail;
//								BT_USART_DMASend((uint32_t)BT_USART_BUFF, 5);
            		break;            	
							
							case 0x31:   // Get Acq Parameter (Sample Rate)
								
								BT_USART_BUFF[0] = T_FrameHeader;
								BT_USART_BUFF[1] = 2;
								BT_USART_BUFF[2] = this.INS_NUM;
								
								BT_USART_BUFF[3] = (uint8_t)gSYS_SampleRate;

								BT_USART_BUFF[4] = BT_USART_BUFF[2]+BT_USART_BUFF[3];
								BT_USART_BUFF[5] = T_FrameTail;
								BT_USART_DMASend((uint32_t)BT_USART_BUFF, 6);
            		break;            	
							
							case 0x41:   // Set Acq Parameter (Sample Rate)
								
								BT_USART_BUFF[0] = T_FrameHeader;
								BT_USART_BUFF[1] = 1;
								BT_USART_BUFF[2] = this.ERR_NUM;
								BT_USART_BUFF[3] = this.ERR_NUM;
								BT_USART_BUFF[4] = T_FrameTail;
								BT_USART_DMASend((uint32_t)BT_USART_BUFF, 5);
            		break;
							
							case 0x32:   // Get Acq Parameter (Gain)
								BT_USART_BUFF[0] = T_FrameHeader;
								BT_USART_BUFF[1] = 2;
								BT_USART_BUFF[2] = this.INS_NUM;
								
								BT_USART_BUFF[3] = (uint8_t)gSYS_Gain;

								BT_USART_BUFF[4] = BT_USART_BUFF[2]+BT_USART_BUFF[3];
								BT_USART_BUFF[5] = T_FrameTail;
								BT_USART_DMASend((uint32_t)BT_USART_BUFF, 6);
            		break;            	
							
							case 0x42:   // Set Acq Parameter (Gain)
								
								BT_USART_BUFF[0] = T_FrameHeader;
								BT_USART_BUFF[1] = 1;
								BT_USART_BUFF[2] = this.ERR_NUM;
								BT_USART_BUFF[3] = this.ERR_NUM;
								BT_USART_BUFF[4] = T_FrameTail;
								BT_USART_DMASend((uint32_t)BT_USART_BUFF, 5);
            		break;
							
							case 0x50:   // Set Special Mode
								
								BT_USART_BUFF[0] = T_FrameHeader;
								BT_USART_BUFF[1] = 1;
								BT_USART_BUFF[2] = this.ERR_NUM;
								BT_USART_BUFF[3] = this.ERR_NUM;
								BT_USART_BUFF[4] = T_FrameTail;
								BT_USART_DMASend((uint32_t)BT_USART_BUFF, 5);
            		break;
							
							case 0x35:   // Get Battary Voltage
								
								
								BT_USART_BUFF[0] = T_FrameHeader;
								BT_USART_BUFF[1] = 3;
								BT_USART_BUFF[2] = this.INS_NUM;
								
								BT_USART_BUFF[3] = gBattVolt&0x00FF;
								BT_USART_BUFF[4] = gBattVolt>>8;
							
								BT_USART_BUFF[5] = BT_USART_BUFF[2]+BT_USART_BUFF[3]+BT_USART_BUFF[4];
								BT_USART_BUFF[6] = T_FrameTail;
								BT_USART_DMASend((uint32_t)BT_USART_BUFF, 7);
            		break;
							
            	default:
								this.ERR_NUM = 0x11;
								printf("WrongINS\r\n");
								transfer_to(FRAME_ERR);
            		break;
            }
						
						printf("S:SEEK\r\n");
						transfer_to(FRAME_SEEKHEAD);
						

            

        )


        
    )
								
						

						
/* FSM calling */
static fsm( ProtocolProcess ) s_fsmProtocolProcess;

void ProtocolProcessFSMInit(void)
{
		if (NULL == init_fsm(ProtocolProcess, &s_fsmProtocolProcess,
			)) {      //!< String Length
			 /* failed to initialize the FSM, put error handling code here */
	 }
}

void ProtocolProcessFSM(void)
{
		if (fsm_rt_cpl == call_fsm( ProtocolProcess, &s_fsmProtocolProcess )) 
		{
               /* fsm is complete, do something here */
    }
}



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
						
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */



