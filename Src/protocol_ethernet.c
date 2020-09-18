/**
 * @file    protocol_ethernet.c
 * @author  gjmsilly
 * @brief   以太网帧协议服务（TCP帧解析+回复，UDP帧发送）
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
#include "w5500_service.h"
#include "microEEG_misc.h"
#include "ads1299.h"
#include "main.h"											 

/*******************************************************************
 * LOCAL VARIABLES
 */

/* 控制通道变量（TCP端口） */
// 上位机->设备 指令解析
static uint8_t	TCP_Recv_FH = 0xAC;				//!< TCP接收帧头
static uint8_t	TCP_Recv_FT = 0xCC;				//!< TCP接收帧尾

// 设备->上位机 回复
static uint8_t	TCP_Send_FH = 0xA2;				//!< TCP发送帧头
static uint8_t	TCP_Send_FT = 0xC2;				//!< TCP发送帧尾

/* 数据通道变量（UDP端口） */
static uint8_t	UDP_SAMPLE_FH = 0x23;			//!< UDP帧数据域 样起始分隔符
static uint8_t	*DevID;										//!< 设备唯一识别码
static uint32_t	UDPNum;										//!< UDP帧服务执行次数

static uint8_t	fsm_status;								//!< 状态机运行状态


/*******************************************************************
 * GLOBAL VARIABLES
 */


/*******************************************************************
 *  Callbacks
 */

static AttrCBs_t *pattr_CBs = NULL;				//!< 属性表服务回调指针

/*!
 *  @fn	属性表服务注册回调函数的接口
 *
 *	@param 属性表读写回调结构体指针
 *
 *	@return SUCCESS - 回调函数注册成功
 *					ERROR - 回调函数注册失败
 */
uint8_t protocol_RegisterAttrCBs(AttrCBs_t *pAttrcallbacks)
{
	if ( pAttrcallbacks )
  {
		pattr_CBs = pAttrcallbacks;
    
		return SUCCESS;
	}
	else
	{
		 return ERROR;
	}
}

/********************************************************************
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
				uint8_t	FrameHeader;				//!< 帧头
				uint8_t FrameLength;				//!< 有效帧长
				uint8_t DataLength;					//!< 数据帧长
				uint8_t InsNum;							//!< 指令码							
				uint8_t InsAttrNum;					//!< 指令作用属性编号
				uint8_t ChxNum;							//!< 指令作用通道
				uint8_t ERR_NUM;						//!< 错误码
				uint8_t *pDataLength;				//!< 数据帧长指针（读回调用）
				uint8_t* _OP_;							//!< 操作立即数指针（写回调用）
				
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
						
///////////////////////////////////////////////////////////////////////
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
    def_states(FRAME_SEEKHEAD,FRAME_LENGTH,FRAME_CHK,FRAME_INS,FRAME_RPY)

    body(
				/*! TCP接收拆包- 帧头获取 */
        state(FRAME_SEEKHEAD,

						this.FrameHeader = TCP_Rx_Buff[0];
						if (this.FrameHeader == TCP_Recv_FH) 
						{
							/* 复位 */
							this.FrameLength = 0xFF;	//!< 有效帧长
							this.DataLength =0xFF;		//!< 数据帧长
							this.InsNum = 0xFF;				//!< 指令码							
							this.InsAttrNum = 0xFF; 	//!< 指令作用属性编号
							this.ChxNum = 0xFF;				//!< 指令作用通道（无通道操作 - 0xFF）
							this.ERR_NUM = 0xFF;			//!< 错误码
							this.pDataLength = &this.DataLength;	
							
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
								memset(TCP_Rx_Buff,0xFF,sizeof(TCP_Rx_Buff_Size));//!< 清空TCP接收缓冲区
								transfer_to(FRAME_SEEKHEAD);							//!< 此包丢弃								
							}																	
       )						
				
				/*! 指令操作 */						
        state(FRAME_INS,
            switch (this.InsNum)
            {
            	case CAttr_Read: //!< 读普通属性
								
								//!< 读属性回调
								this.ERR_NUM = pattr_CBs->pfnReadAttrCB(	this.InsAttrNum,this.ChxNum, \
																													(TCP_Tx_Buff+4),this.pDataLength);
								this.FrameLength = *(this.pDataLength)+2;
								printf("S:RPY\r\n");								
								transfer_to(FRAME_RPY);        		
								break;
            	
							case CAttr_Write: //!< 写普通属性
								
								//!< 写属性回调
								this.ERR_NUM = pattr_CBs->pfnWriteAttrCB(	this.InsAttrNum,this.ChxNum, \
																													this._OP_,this.DataLength);  										
								
								if( this.ERR_NUM == SUCCESS )
								{
									//!< 拷贝一份用于回复 
									memcpy((TCP_Tx_Buff+4),this._OP_,this.DataLength);
									this.FrameLength = this.DataLength+2;
								}
								printf("S:RPY\r\n");
								transfer_to(FRAME_RPY);									           		
								break;
							
            	default:
								printf("WrongINS\r\n");
								memset(TCP_Rx_Buff,0xFF,sizeof(TCP_Rx_Buff_Size));//清空TCP接收缓冲区
            		transfer_to(FRAME_SEEKHEAD);							//!< 此包丢弃	
								break;
            }						
        )
						

				/*! TCP回复打包 */	
        state(FRAME_RPY,
				
						TCP_Tx_Buff[0] = TCP_Send_FH;					//!< 帧头
						
						if( this.ERR_NUM != SUCCESS )
							this.FrameLength=2; //!< 有效帧只含错误码+回复类型
								
						TCP_Tx_Buff[1] = this.FrameLength;	//!< 有效帧
						TCP_Tx_Buff[2] = this.ERR_NUM;			//!< 错误码											
						TCP_Tx_Buff[3] = this.InsAttrNum; 	//!< 回复类型（属性编号）	
										
						TCP_Tx_Buff[this.FrameLength+2]=TCP_Send_FT; //!< 帧尾
						
						printf("S:END\r\n");
						fsm_cpl(); //!< 状态机完成
						           
        )       
    )
/* End of fsm implementation */								
						
/*!
 *  @fn	fsm_xxx
 *	@brief 状态机初始化 / 对外调用函数
 */
static fsm( TCP_Process ) s_fsmProtocolProcess;

void TCP_ProcessFSMInit(void)
{
		if (NULL == init_fsm(TCP_Process, &s_fsmProtocolProcess,
			)) {      //!< String Length
			 /* failed to initialize the FSM, put error handling code here */
	 }
}

uint8_t TCP_ProcessFSM(void)
{
		//!< 状态机完成
		if (fsm_rt_cpl == call_fsm( TCP_Process, &s_fsmProtocolProcess )) 
		{
			fsm_status=_FSM_CPL_; 
        
			return fsm_status;       
    }
}


///////////////////////////////////////////////////////////////////////
/*
 *  ======================== UDP帧协议服务 ============================
 */ 
/*!
 *  @fn	UDP帧协议服务处理函数
 *
 *	@param	Procesflag - AD数据采集状态标志位
 *					SampleNum - 目前待封包的AD样本序数	- 无需封包则0xFF
 *
 *	@return SUCCESS - UDP帧协议服务打包完成
 *					ERROR - 异常
 */
uint8_t UDP_PROCESS(uint8_t SampleNum ,uint8_t Procesflag)
{

		/* AD数据采集中，对数据域进行封包 */
	 if(Procesflag&EEG_DATA_START_EVT)
	 {
		 if(SampleNum!= 0xFF )
		 {
			UDP_Tx_Buff[HEAD_SIZE+DATA_SIZE*SampleNum] = UDP_SAMPLE_FH;		//!< 样本起始分隔符 
			UDP_Tx_Buff[HEAD_SIZE+DATA_SIZE*SampleNum+1]=SampleNum;			//!< 样本序号 - 显示从0开始的序数
			
			ADS1299_ReadResult((UDP_Tx_Buff+(DATA_SIZE+9)*SampleNum+HEAD_SIZE+6));	//!< 样本每通道量化值 //!< 前三字节覆盖问题需改进 20/9/18			
			
			UDP_Tx_Buff[HEAD_SIZE+DATA_SIZE*SampleNum+3]=*pCurTimeStamp;	//!< 样本时间戳 - 增量型（每样本相对第一样本时间增量）精度10us，注意小端对齐
			UDP_Tx_Buff[HEAD_SIZE+DATA_SIZE*SampleNum+4]=*(pCurTimeStamp+1);
			UDP_Tx_Buff[HEAD_SIZE+DATA_SIZE*SampleNum+5]=*(pCurTimeStamp+2);
			UDP_Tx_Buff[HEAD_SIZE+DATA_SIZE*SampleNum+6]= *(pCurTimeStamp+3);
			
			UDP_Tx_Buff[HEAD_SIZE+DATA_SIZE*SampleNum+7]=0xAA;	//!< 样本事件标记 - 默认0xAAAA
			UDP_Tx_Buff[HEAD_SIZE+DATA_SIZE*SampleNum+8]=0xAA;
			 

		}
		 else
			 return ERROR; 
	}
	 
		/* AD数据采集完毕，对UDP帧头封包 */
	 if(Procesflag&EEG_DATA_READY_EVT)
	 {
		 	uint8_t* len;
			len = (uint8_t*)malloc(2);
		 
			/* 数据源 */
		  DevID = (uint8_t*)0x1FFF7A10;	//!< STM32F4唯一ID起始地址
			memcpy(UDP_Tx_Buff,DevID,4);		 
		 
			/* UDP包累加滚动码 */
			UDP_Tx_Buff[4]=(uint8_t) UDPNum; //!< UDP包累加滚动码,也即UDP帧头封包执行次数，注意小端对齐
			UDP_Tx_Buff[5]=(uint8_t)(UDPNum >> 8);
			UDP_Tx_Buff[6]=(uint8_t)(UDPNum >> 16);
			UDP_Tx_Buff[7]=(uint8_t)(UDPNum >> 24);
			UDPNum++;	
		 
			/* 本UDP包总样数 */
			pattr_CBs->pfnReadAttrCB(	7,0xFF,(UDP_Tx_Buff+8),len);
			
			/* 本UDP包有效通道总数 */
			UDP_Tx_Buff[10]=8; //BUG 先锁定该值
			
			/* 首样时间戳 */		 
		 
			/* 包头保留数 */	
			memset(UDP_Tx_Buff+19,0xff,4);
		 
			free(len);
	
			return SUCCESS; 
	 }		 

	
}
