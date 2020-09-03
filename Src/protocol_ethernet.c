/**
 * @file    protocol_ethernet.c
 * @author  gjmsilly
 * @brief   以太网数据协议（TCP封包拆包，UDP封包 实现）
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
InsQUEUE TCPInsQueue;   		//!< 队列 - TCP接收的控制指令
uint8_t FrameHeaderBuff;    

/*******************************************************************
 * TYPEDEFS
 */
 
/*! @brief	定义延时1s的状态机 delay_1s
 *!					同时列出所有参数	wCounter
 */
simple_fsm
( 
		/* 声明状态机名称 */
		delay_1s,
    
    /* 定义本状态机所用的所有形参 */ 
    def_params
		(
        uint32_t wCounter;                  //!< a uint32_t counter
    )
)


/*! @brief	定义TCP协议数据处理的状态机 TCP_Process
 *!       	同时列出所有参数
 *!	@note		本状态机由w5500_app.c -> TCPServer_Service调用
 */
extern_simple_fsm
( 
		TCP_Process,
    def_params
		(
				uint8_t FrameLength;				//!< 有效帧长
				uint8_t InsNum;							//!< 指令码							
				uint8_t InsAttrNum;					//!< 指令作用地址（属性表偏移量）
				uint8_t ChxNum;							//!< 指令作用通道
				uint8_t ERR_NUM;						//!< 错误码
				uint8_t	OP[4];							//!< 操作立即数 // 9.3 操作数数据类型 bool int8 int32 ???
				uint32_t* RPY;							//!< 回复数据
				
				bool InsNeedReply;          //!< 指令需要回复
				fsm(delay_1s) fsmDelay;			//!< sub fsm delay_1s
    )
)


/*********************************************************************
 * FUNCTIONS
 */
/*
 *  ============================ 延时1s ==============================
 */ 
/*! @brief define the fsm initialiser for fsm delay_1s
 *! @param - wCounter  an uint32_t value for the delay count
 */
fsm_initialiser
(
		delay_1s,														 //!< 状态机名称
   
	 /*! 本状态机初始化函数的形参列表 */
    args
		(           
        uint32_t wCounter               //!< delay count
    ))
		/*! 本状态机初始化函数 */
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
 *  ========================== TCP拆包 ==============================
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
				/*! 帧头获取 */
        state(FRAME_SEEKHEAD,
    						
						FrameHeaderBuff = DeQueue(&TCPInsQueue);   //!< 帧头出队
						if (FrameHeaderBuff == TCP_Recv_FH) 
						{
							this.FrameLength = 0;			//!< 有效帧长
							this.InsNum = 0;					//!< 指令码							
							this.InsAttrNum = 0x00; 	//!< 指令作用地址（属性表偏移量）
							this.ChxNum = 0x00;				//!< 指令作用通道
							this.ERR_NUM = 0x00;			//!< 错误码
							
							printf("S:LENGTH\r\n");
							transfer_to(FRAME_LENGTH);							//!< 跳转有效帧长获取
						}						
       )
						
				/*! 帧长度获取 */												
				state(FRAME_LENGTH,
						
						if ((this.FrameLength = DeQueue(&TCPInsQueue)) != 0xFF) 
						{

							printf("S:CHK\r\n");
							transfer_to(FRAME_CHK);									//!< 跳转帧检测
							
						}
       )						

        /*! 帧检测 */	
				state(FRAME_CHK,
						
						if (GetQueueLength(&TCPInsQueue) >= this.FrameLength +1)   //!< 等待整帧入队 
						{
							if (this.FrameLength == 3) //!< 读属性操作
							{
								this.InsNum = DeQueue(&TCPInsQueue);
								this.InsAttrNum = DeQueue(&TCPInsQueue);
								this.ChxNum = DeQueue(&TCPInsQueue);
							}
							else if(this.FrameLength == 0);	//!< 空指令
							else //!< 写属性操作
							{
								this.InsNum = DeQueue(&TCPInsQueue);
								this.InsAttrNum = DeQueue(&TCPInsQueue);
								this.ChxNum = DeQueue(&TCPInsQueue);
								
								for (this.i=0;this.i<(this.FrameLength-3);this.i++)
								{
									this.OP[this.i] = DeQueue(&TCPInsQueue);
								}
							}

							if(TCP_Recv_FT == DeQueue(&TCPInsQueue))     //!< 帧尾检测
							{
								
								printf("S:INS\r\n");
								
								transfer_to(FRAME_INS);										//!< 跳转指令操作
							}
							else     //!< 帧尾检测失败
							{
								//this.ERR_NUM = 0x30;
								printf("S:ERR-CTFailed\r\n");
								transfer_to(FRAME_ERR);
							}							
						}											
       )						
				
				/*! 指令操作 */						
        state(FRAME_INS,
            switch (this.InsNum)
            {
            	case 0x01:   																//!< 读普通属性
								this.RPY = ((uint8_t*)pattr_tbl+2*InsAttrNum+1);						
								transfer_to(FRAME_RPY);										//!< 跳转回复
            		
								break;
            	
							case 0x02:   																//!< 写普通属性
								
								transfer_to(FRAME_RPY);										//!< 跳转回复
            		
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



