/**
 * @file    protocol_ethernet.c
 * @author  gjmsilly
 * @brief   以太网帧协议服务（TCP封包拆包，UDP封包）
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

/* 指令码 */
const uint8_t	DummyIns = 0x00;						//<! 空指令
const uint8_t	CAttr_Read = 0x01;					//<! 读一个普通属性
const uint8_t	CAttr_Write = 0x10;					//<! 写一个普通属性
const uint8_t	ChxAttr_Read = 0x02;				//<! 读一个通道属性
const uint8_t	ChxAttr_Write = 0x20;				//<! 写一个通道属性

/* 控制通道参数（TCP端口） */
// 上位机->设备 指令解析
static uint8_t	TCP_Recv_FH = 0xAC;				//<! TCP接收帧头
static uint8_t	TCP_Recv_FT = 0xCC;				//<! TCP接收帧尾

// 设备->上位机 回复
static uint8_t	TCP_Send_FH = 0xA2;				//<! TCP发送帧头
static uint8_t	TCP_Send_FT = 0xC2;				//<! TCP发送帧尾

static uint8_t	TCP_SUCCESS = 0x00;				//<! 指令解析正常
static uint8_t	TCPERR_Attr_RO = 0x01;		//<! 指令错误：对只读属性写操作
static uint8_t	TCPERR_Attr_LEN = 0x02;		//<! 指令错误：写指令数据长度错误

/*******************************************************************
 * GLOBAL VARIABLES
 */
  
uint8_t TCP_Rx_Buff[TCP_Rx_Buff_Size];		//!< TCP接收数据缓冲区 
uint8_t TCP_Tx_Buff[TCP_Tx_Buff_Size];		//!< TCP发送数据缓冲区
uint8_t UDP_Tx_Buff[UDP_Tx_Buff_Size];		//!< UDP发送数据缓冲区
uint8_t	DataLength1;// debug
uint8_t*	pDataLength1;// debug
uint8_t FrameHeaderBuff;
uint8_t	TCP_RPY_Size=0xFF;				//!< TCP回复帧长 - 用于通知w5500_app
uint8_t	Attr_Change_Num;		//!< 属性表值变化的属性号 - 用于通知AttrChangeEvt
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
simple_fsm
( 
		TCP_Process,
    def_params
		(
				uint8_t FrameLength;				//!< 有效帧长
				uint8_t DataLength;					//!< 数据帧长
				uint8_t InsNum;							//!< 指令码							
				uint8_t InsAttrNum;					//!< 指令作用属性编号
				uint8_t	AttrPermission;			//!< 属性读写权限
				uint8_t ChxNum;							//!< 指令作用通道
				uint8_t ERR_NUM;						//!< 错误码
				uint8_t* _OP_;							//!< 操作立即数指针 // 9.3 操作数数据类型 bool int8 int32 ???
				uint32_t* _RPY_;						//!< 属性值指针（用于属性值读取并TCP回复）
				
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
 *  ======================== TCP帧协议服务 ============================
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
				/*! TCP接收拆包- 帧头获取 */
        state(FRAME_SEEKHEAD,
		
						FrameHeaderBuff = TCP_Rx_Buff[0];
						if (FrameHeaderBuff == TCP_Recv_FH) 
						{
							this.FrameLength = 0;			//!< 有效帧长
							this.DataLength =0;				//!< 数据帧长
							this.InsNum = 0;					//!< 指令码							
							this.InsAttrNum = 0x00; 	//!< 指令作用属性编号
							this.ChxNum = 0x00;				//!< 指令作用通道
							this.ERR_NUM = 0x00;			//!< 错误码
							
							printf("S:LENGTH\r\n");
							transfer_to(FRAME_LENGTH);							//!< 跳转帧长度获取
						}						
       )
						
				/*! 帧长度获取 */												
				state(FRAME_LENGTH,
						
						if ((this.FrameLength = TCP_Rx_Buff[1]) != 0xFF) 
						{
							printf("S:CHK\r\n");
							transfer_to(FRAME_CHK);									//!< 跳转帧检测						
						}
       )						
			 
        /*! 帧检测 */	
				state(FRAME_CHK,
						
							if (this.FrameLength == 3) //!< 读属性操作 - 有效帧定长3
							{ 
								this.InsNum = TCP_Rx_Buff[2];
								this.InsAttrNum = TCP_Rx_Buff[3];
								this.ChxNum = TCP_Rx_Buff[4];
							}
							else if(this.FrameLength == 1);	//!< 空指令 - 有效帧定长1
							else //!< 写属性操作 - 有效帧>4 不定长
							{
								this.InsNum = TCP_Rx_Buff[2];
								this.DataLength = this.FrameLength-3;
								this.InsAttrNum = TCP_Rx_Buff[3];
								this.ChxNum = TCP_Rx_Buff[4];
								this._OP_ = TCP_Rx_Buff+5;
							}
							
							if(TCP_Rx_Buff[this.FrameLength+2]== TCP_Recv_FT)//!< 帧尾检测
							{								
								printf("S:INS\r\n");								
								transfer_to(FRAME_INS);										//!< 跳转指令操作
							}
							else //!< 帧尾检测失败
							{
								printf("S:ERR-CTFailed\r\n");
								transfer_to(FRAME_SEEKHEAD);							//!< 此包丢弃
							}																	
       )						
				
				/*! 指令操作 */						
        state(FRAME_INS,
            switch (this.InsNum)
            {
            	case CAttr_Read: //!< 读普通属性
								this._RPY_ = pattr_offset[this.InsAttrNum];	//!< 属性值地址传递
								this.ERR_NUM = TCP_SUCCESS;
								transfer_to(FRAME_ERR);        		
								break;
            	
							case CAttr_Write: //!< 写普通属性
								//!< 属性读写权限检查
								this.AttrPermission = *(pattr+this.InsAttrNum*6);
								if(this.AttrPermission == ATTR_RO ||		\
										this.AttrPermission == ATTR_NV) 
									{
										this.ERR_NUM = TCPERR_Attr_RO; //!< 指令错误：对只读属性写操作
										transfer_to(FRAME_ERR);
									}
								//!< 指令数据长度检查
								else if(*(pattr+this.InsAttrNum*6+1)!= this.DataLength)
									{
										this.ERR_NUM = TCPERR_Attr_LEN;//<! 指令错误：写指令数据长度错误
										transfer_to(FRAME_ERR);
									}
								else
									{
										this.ERR_NUM = TCP_SUCCESS;
										this._RPY_ = pattr_offset[this.InsAttrNum];	//!< 属性值地址传递
										memcpy((uint8_t*)this._RPY_,(TCP_Rx_Buff+5),this.DataLength); //!< 写操作
										transfer_to(FRAME_ERR);
									}
									           		
								break;
							
            	default:
								printf("WrongINS\r\n");
								transfer_to(FRAME_SEEKHEAD);							//!< 此包丢弃
            		break;
            }						
        )
						
				/*! 错误码标记 */				
        state(FRAME_ERR,
						
								TCP_Tx_Buff[2]=this.ERR_NUM; 
								
								printf("RPY\r\n");
								transfer_to(FRAME_RPY);										//!< 跳转回复  
        )

				/*! TCP回复打包 */	
        state(FRAME_RPY,
				
						TCP_Tx_Buff[0] = TCP_Send_FH; //!< 帧头
						
						if(this.ERR_NUM == TCP_SUCCESS) //!< 正确接收
						{
							this.DataLength=*(pattr+this.InsAttrNum*6+1); //!< 数据帧长
							
							this.FrameLength=this.DataLength+2;  
							TCP_Tx_Buff[1]= this.FrameLength; //!< 有效帧长						
							TCP_Tx_Buff[3] = this.InsAttrNum; //!< 回复类型（属性编号）						
							memcpy((TCP_Tx_Buff+4),(uint8_t*)this._RPY_,this.DataLength); //!< 数据帧
						}
						else //!< 错误接收
						{
							this.FrameLength=1; //!< 有效帧只含错误码
							
						}
											
						TCP_Tx_Buff[this.FrameLength+2]=TCP_Send_FT; //!< 帧尾
						
						TCP_RPY_Size = this.FrameLength+3; //!< 通知w5500_app回复目的主机
						
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



