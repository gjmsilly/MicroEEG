/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <string.h>
#include "compiler.h"
#include "ooc.h"
#include "simple_fsm.h"
#include "SimpleInsQueue.h"
#include "main.h"
#include "MicroEEG_Misc.h"
#include "ads1299.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/




const uint8_t R_FrameHeader = 0xAC;
const uint8_t R_FrameTail   = 0xCC;
const uint8_t T_FrameHeader = 0xA2;
const uint8_t T_FrameTail   = 0xC2;
const uint8_t Tag_FrameHeader = 0xBC;
const uint8_t Tag_FrameTail   = 0xBD;

uint8_t FrameHeaderBuff;
uint8_t TagBuffer[4];


InsQUEUE UARTInsQueue;

extern uint8_t BT_USART_BUFF[];
extern uint16_t gSYS_SampleRate;
extern uint16_t gSYS_Gain;

extern uint8_t BT_USART_Mutex;
extern uint8_t BT_USART_Ready;
const uint8_t testchar2[5]="None ";


extern uint16_t gBattVolt;

extern char* pcCurTimeStamp;
extern char* pcCurEventTag;
extern uint16_t CurEventTag;

unsigned int SampleNum = 0;
uint8_t NewSample = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void BTFrameBuilder (void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/*! /brief define fsm delay_1s
 *!        list all the parameters
 */
simple_fsm
( 
		delay_1s,
    
    /* define all the parameters used in the fsm */ 
    def_params
		(
        uint32_t wCounter;                  //!< a uint32_t counter
    )
)


/*! /brief define fsm ProtocolProcess
 *!        list all the parameters
 */
simple_fsm
( 
		ProtocolProcess,
    def_params
		(
				uint8_t ERR_NUM;
				uint8_t INS_NUM;
				uint8_t PRM_NUM;
				uint8_t FrameLength;
				uint8_t CSBuffer[5];
				uint8_t i,ISUM;
				bool InsNeedReply;
				fsm(delay_1s) fsmDelay;             //!< sub fsm delay_1s
    )
)




/*! /brief define the fsm initialiser for FSML delay_1s
*! /param wCounter  an uint32_t value for the delay count
*/
fsm_initialiser
(
		delay_1s,               //!< define initialiser for fsm: delay_1s
    /*! list all the parameters required by this initialiser */
    args
		(           
        uint32_t wCounter               //!< delay count
    )
)

		/*! the body of this initialiser */
		init_body 
		(
				this.wCounter = wCounter;       //!< initialiser the fsm paramter
		)

fsm_initialiser(ProtocolProcess)
    init_body ()






/*! /brief Implement the fsm: delay_1s
*         This fsm only contains one state.
*/
fsm_implementation(delay_1s)
    def_states(DELAY_1S)                //!< list all the states used in the FSM

    /* the body of the FSM: delay_1s */
    body 
		(
        state
				(  
						DELAY_1S,               //!< state: DELAY_1s
            if (!this.wCounter) 
						{
                fsm_cpl();              //!< FSM is completed
            }
            this.wCounter--;
        )
    )
/* End of fsm implementation */
						
						


/*! /brief Implement the fsm: delay_1s
*         This fsm only contains one state.
*/


fsm_implementation(ProtocolProcess)

    /*! list all the states used in the FSM */
    def_states(FRAME_SEEKHEAD,FRAME_LENGTH,FRAME_CHK,FRAME_INS,FRAME_ERR,FRAME_RPY,FRAME_TAG)

    body(
//! the on_start block are called once and only once on the entry point of a FSM
//        on_start(
//            /* add fsm parameter initialisation code here */
//        )


        state(FRAME_SEEKHEAD,
    
						
						FrameHeaderBuff = DeQueue(&UARTInsQueue);
						// Instruction Frame
						if (FrameHeaderBuff == R_FrameHeader) 
						{
							this.PRM_NUM = 0x00;
							this.ISUM = 0;
							this.FrameLength = 0;
							this.ERR_NUM = 0;
							
							printf("S:LENGTH\r\n");
							transfer_to(FRAME_LENGTH);
						}
						
						//Tag Frame
						if (FrameHeaderBuff == Tag_FrameHeader) 
						{
							if(SYS_Status == SYS_STATUS_ACQUIRING)
							{
								this.PRM_NUM = 0x00;
								this.ISUM = 0;
								this.FrameLength = 0;
								this.ERR_NUM = 0;
							
								transfer_to(FRAME_TAG);
							}
						}
        )
				
						
				state(FRAME_TAG,
    
						
						if(GetQueueLength(&UARTInsQueue) >= 5)   //wait for enough 
						{
							TagBuffer[0] = DeQueue(&UARTInsQueue);
							TagBuffer[1] = DeQueue(&UARTInsQueue);
							TagBuffer[2] = DeQueue(&UARTInsQueue);
							TagBuffer[3] = DeQueue(&UARTInsQueue);
							
							
							if((TagBuffer[0]==TagBuffer[2])&&(TagBuffer[1]==TagBuffer[3]))  //check
							{
								if(Tag_FrameTail == DeQueue(&UARTInsQueue))    //checktail
								{
									*(pcCurEventTag) = TagBuffer[0];
									*(pcCurEventTag+1) = TagBuffer[1];
									
									printf("S:Tagged\r\n");
									
									transfer_to(FRAME_SEEKHEAD);
								}
								else     //checktail failed
								{
									this.ERR_NUM = 0x30;
									printf("S:ERR-CTFailed\r\n");
									transfer_to(FRAME_ERR);
								}
								
							}
							else   //check failed
							{
								this.ERR_NUM = 0x20;
								printf("S:ERR-CSFailed\r\n");								
								transfer_to(FRAME_ERR);
							}
						}
        )	
						
				state(FRAME_LENGTH,
    
						
						if ((this.FrameLength = DeQueue(&UARTInsQueue)) != 0xFF) 
						{

							printf("S:CHK\r\n");
							transfer_to(FRAME_CHK);
							
						}
        )						

        
				state(FRAME_CHK,

						
						if(GetQueueLength(&UARTInsQueue) >= this.FrameLength +1)   //wait for enough 
						{
							
							for (this.i=0;this.i<this.FrameLength;this.i++)
							{
								this.CSBuffer[this.i] = DeQueue(&UARTInsQueue);
								this.ISUM += this.CSBuffer[this.i];
							}
							
							if(this.ISUM == DeQueue(&UARTInsQueue))  //checksum
							{
								if(R_FrameTail == DeQueue(&UARTInsQueue))    //checktail
								{
									
									this.INS_NUM = this.CSBuffer[0];
									printf("S:INS\r\n");
									
									transfer_to(FRAME_INS);
								}
								else     //checktail failed
								{
									this.ERR_NUM = 0x30;
									printf("S:ERR-CTFailed\r\n");
									transfer_to(FRAME_ERR);
								}
								
							}
							else   //checksum failed
							{
								this.ERR_NUM = 0x20;
								printf("S:ERR-CSFailed\r\n");								
								transfer_to(FRAME_ERR);
							}
						}
						
						

        )						
						
        state(FRAME_INS,
            switch (this.INS_NUM)
            {
            	case 0x80:   // Start Acq
								// preform action
								printf("ACQ_start\r\n");
								this.ERR_NUM = 0;
									
									ADS1299_Mode_Config(ADS1299_ParaGroup_ACQ);
									LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_7);
									ADS1299_SendCommand(ADS1299_CMD_RDATAC);		
							
									SYS_Status = SYS_STATUS_ACQUIRING;
							
							
								transfer_to(FRAME_RPY);
            		
								break;
            	
							case 0x90:   // Stop Acq
								// preform action
								printf("ACQ_stop\r\n");
								this.ERR_NUM = 0;
									
									
									LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_7);
									
									Mod_CS_Disable;
									ADS1299_SendCommand(ADS1299_CMD_SDATAC);
							
									SYS_Status = SYS_STATUS_STANDBY;
									
								transfer_to(FRAME_RPY);
            		
								break;
            	
							case 0x82:   // Start Resistantance Measuring
								printf("RES_start\r\n");
									

								this.ERR_NUM = 0;
									
									ADS1299_Mode_Config(ADS1299_ParaGroup_IMP);
									LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_7);
									ADS1299_SendCommand(ADS1299_CMD_RDATAC);		
							
									SYS_Status = SYS_STATUS_ACQUIRING;
								transfer_to(FRAME_RPY);
            		break;            	
							
							case 0x92:   // Stop Resistantance Measuring
								printf("RES_stop\r\n");
								this.ERR_NUM = 0;
									
									
									LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_7);
									
									Mod_CS_Disable;
									ADS1299_SendCommand(ADS1299_CMD_SDATAC);
							
									SYS_Status = SYS_STATUS_STANDBY;
								transfer_to(FRAME_RPY);
            		break;            	
							
							case 0x31:   // Get Acq Parameter (Sample Rate)
								printf("ParaReadSR\r\n");
								transfer_to(FRAME_RPY);
            		break;            	
							
							case 0x41:   // Set Acq Parameter (Sample Rate)
								printf("ParaWriteSR %d\r\n",this.CSBuffer[1]);
								//need check!!
								gSYS_SampleRate = this.CSBuffer[1];
								transfer_to(FRAME_RPY);
            		break;
							
							case 0x32:   // Get Acq Parameter (Gain)
								printf("ParaReadG\r\n");
								transfer_to(FRAME_RPY);
            		break;            	
							
							case 0x42:   // Set Acq Parameter (Gain)
								printf("ParaWriteG %d\r\n",this.CSBuffer[1]);
								//need check!!
								gSYS_Gain = this.CSBuffer[1];
								transfer_to(FRAME_RPY);
            		break;
							
							case 0x50:   // Set Special Mode
								printf("SPMode\r\n");
								transfer_to(FRAME_RPY);
            		break;
							
							case 0x35:   // Get Battary Voltage
								transfer_to(FRAME_RPY);
								printf("ReadBATT_V\r\n");
            		break;
							
            	default:
								this.ERR_NUM = 0x11;
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



