#include "main.h"
#include "stm32f4xx_hal.h"
#include "ads1299.h"
#include "stm32f4xx_ll_gpio.h"

uint8_t ResultByte = 0;
uint8_t DummyByte;


/****************************************************************/
/* ADS1299_Init                                                  */
/** Operation:
 *      - Initial ads1299 chip
 *
 * Parameters:
 *      -  dev:ADS1299 chip number
 *
 * Return value:
 *     - None
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/
void ADS1299_Init(uint8_t dev)
{
	Mod_DRDY_INT_Disable;
	
	// �ϵ縴λ
	ADS1299_PowerOn(dev);
	ADS1299_Reset(dev);	
	
	// �ϵ���
	LL_GPIO_SetOutputPin(Mod_START_GPIO_Port, Mod_START_Pin);
	while(LL_GPIO_IsInputPinSet(Mods_nDRDY_GPIO_Port,Mods_nDRDY_Pin));
	LL_GPIO_ResetOutputPin(Mod_START_GPIO_Port, Mod_START_Pin); 
	
	ADS1299_Reset(0);
	ADS1299_SendCommand(ADS1299_CMD_RESET);
	WaitUs(10);
	ADS1299_SendCommand(ADS1299_CMD_SDATAC);	

}

/****************************************************************/
/* Wait()                 Modified                              */
/** Operation:
 *      - Opens a DSP timer and configures it for the correct
 *        wait time on the first call. Otherwise, just reprograms
 *        the timer period register.
 *      - Waits the number of tosc periods specified in
 *        uiWaitCount
 *
 * Parameters:
 *      - TADS1299 *pADS:Data converter object
 *      - unsigned int uiWaitCount: Number of tosc, which should
 *        be waited
 *
 * Return value:
 *     - TIDC_NO_ERR in case everything was OK
 *     - TIDC_ERR_TIMER if the timer could not be opened
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - One timer
 *
 * Note:
 *     - This routine is a blocking one. That means it waits for
 *       the timer to complete before it returns
 */
/****************************************************************/
void WaitUs(int iWaitUs)
{
    int iPreTickVal = SysTick -> VAL;  
	  int iCounterTargetValue;
	  int iCounterTargetInterval;
	   
	  iCounterTargetInterval = iWaitUs * (HAL_RCC_GetHCLKFreq()/1000000)-32;
	  if(iPreTickVal < iCounterTargetInterval)
	  {
			iCounterTargetValue =  iPreTickVal + SysTick->LOAD - iCounterTargetInterval;
		}
		else
		{
			iCounterTargetValue =  iPreTickVal - iCounterTargetInterval;
		}
		
		while(SysTick -> VAL >= iCounterTargetValue);
	
}




/****************************************************************/
/* ADS1299_Reset                                                  */
/** Operation:
 *      - Reset ADS1299 chip
 *
 * Parameters:
 *      -  dev:ADS1299 chip number
 *
 * Return value:
 *     - None
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/
void ADS1299_Reset(uint8_t dev)
{
	Mod_RESET_L;
	WaitUs(6);			// ��������2tclk
	Mod_RESET_H;
	Mod_CS_Disable;
	WaitUs(200);     // ���ٵȴ�18tclk����ָ��
	
	// wait for 18 tclk then start using device
}

/****************************************************************/
/* ADS1299_Reset                                                  */
/** Operation:
 *      - Reset ADS1299 chip
 *
 * Parameters:
 *      -  dev:ADS1299 chip number
 *
 * Return value:
 *     - None
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/
void ADS1299_PowerOn(uint8_t dev)
{
	Mod_PDWN_H
	Mod_RESET_H
	
	// wait for at least tPOR = 128ms 
	HAL_Delay(200);

}



/****************************************************************/
/* ADS1299_WriteREG()                                                  */
/** Operation:
 *      - Configuring the ADS1299 register
 *
 * Parameters:
 *      - dev:ADS1299 chip number
 *      - address:Destination register address
 *      - value:The value of destination register
 *
 * Return value:
 *     - None
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/
void ADS1299_WriteREG (uint8_t dev, uint8_t address, uint8_t value)
{
	
	address += 0x40;			
	__disable_irq();
	
	Mod_CS_Enable;
	WaitUs(4);	
	
	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
	SPI1->DR = address;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
	address = SPI1->DR;
	WaitUs(4);														// 2us Command decode time 
	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
	SPI1->DR = 0;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
	address = SPI1->DR;
	WaitUs(4);														// 2us Command decode time
	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
	SPI1->DR = value;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
	while(LL_SPI_IsActiveFlag_BSY(SPI1));
	
	WaitUs(5);													// Delay time, final SCLK falling edge to CS high
	Mod_CS_Disable;
	
	__enable_irq();
  
  
}

/****************************************************************/
/* ADS1299_ReadREG()                                                  */
/** Operation:
 *      - Configuring the ADS1299 register
 *
 * Parameters:
 *      - dev:ADS1299 chip number
 *      - address:Destination register address
 *      - value:The value of destination register
 *
 * Return value:
 *     - None
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/
uint8_t ADS1299_ReadREG (uint8_t dev, uint8_t address)
{
	address += 0x20;

	uint8_t DRchar;	
		
	__disable_irq();
	
	Mod_CS_Enable;
	WaitUs(5);	
	DRchar = SPI1->DR;
	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
	SPI1->DR = address;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
	DRchar = SPI1->DR;
	WaitUs(4);														// 2us Command decode time
	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
	SPI1->DR = 0;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
	DRchar = SPI1->DR;
	WaitUs(4);														// 2us Command decode time
	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
	SPI1->DR = 0;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
	while(LL_SPI_IsActiveFlag_BSY(SPI1));
	DRchar = SPI1->DR;
	
	WaitUs(5);													// Delay time, final SCLK falling edge to CS high
	Mod_CS_Disable;
  __enable_irq();

  return DRchar;
}



/****************************************************************/
/* ADS1299_SendCommand()                                        */
/** Operation:
 *      - Send command to the ADS1299 chip
 *
 * Parameters:
 *      - dev:ADS1299 chip number
 *      - command:command to the ADS1299 chip
 *
 * Return value:
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/
void ADS1299_SendCommand(uint8_t command)
{
	__disable_irq();
	
	Mod_CS_Enable
	WaitUs(4);
	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
	SPI1->DR = command;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
	command = SPI1->DR;
	
	while(LL_SPI_IsActiveFlag_BSY(SPI1));
	
	WaitUs(4);													// Delay time, final SCLK falling edge to CS high
	Mod_CS_Disable

  __enable_irq();
	WaitUs(4);													// Pulse duration, CS high
}

/****************************************************************/
/* ADS1299_ReadByte()                                           */
/** Operation:
 *      - Read ADS1299 output byte
 *
 * Parameters:
 *      - None
 *
 * Return result of byte:
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/
inline uint8_t ADS1299_ReadByte(void)
{
	ResultByte = 0;
	//Mod_CS_Enable;
	while(!LL_SPI_IsActiveFlag_TXE(SPI1));
	SPI1->DR = 0x00;
	while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
	ResultByte = SPI1->DR;
	while(LL_SPI_IsActiveFlag_BSY(SPI1));
	//WaitUs(2);
	//Mod_CS_Disable;
	return ResultByte;
}

/****************************************************************/
/* ADS1299_ReadResult()                                         */
/** Operation:
 *      - Read ADS1299 output data
 *
 * Parameters:
 *      - None
 *
 * Return value:
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/
void ADS1299_ReadResult(uint8_t *result)
{
	uint8_t i;
//  Mod_CS_Enable;
//	WaitUs(4); 
	
	//DMA
	#ifdef Dev_Ch32 
	ADS1299_ReadResult_DMA((uint32_t)result, 109); //DMA Read Bug ����1�ֽ�
	#endif
	#ifdef Dev_Ch24 
	ADS1299_ReadResult_DMA((uint32_t)result, 82); //DMA Read Bug ����1�ֽ�
	#endif
	#ifdef Dev_Ch16 
	ADS1299_ReadResult_DMA((uint32_t)result, 55); //DMA Read Bug ����1�ֽ�
	#endif
	#ifdef Dev_Ch8 
	ADS1299_ReadResult_DMA((uint32_t)result, 28); //DMA Read Bug ����1�ֽ�
	#endif
	
	while(!LL_DMA_IsActiveFlag_TC0(DMA2)); // Wait until all data trasferred from SPI1_RX
	
	LL_SPI_DisableDMAReq_TX(SPI1);
	LL_SPI_DisableDMAReq_RX(SPI1);
	
	LL_DMA_ClearFlag_TC3(DMA2);
	LL_DMA_ClearFlag_TC0(DMA2);
	
	//printf("R%d %d %d ",ResultBuffer[3],ResultBuffer[4],ResultBuffer[5]);
	
//	Mod_CS_Disable;
//	WaitUs(4);
}


void ADS1299_ReadResult_DMA(uint32_t DataHeadAddress, uint8_t DataLength)
{
	
	//	Configure the DMA channel for  SPI1_RX 
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, DataLength);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_0, DataHeadAddress);	
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
	LL_DMA_DisableIT_TC(DMA2, LL_DMA_STREAM_0);
	LL_SPI_EnableDMAReq_RX(SPI1);
	
	//	Configure the DMA channel for  SPI1_TX 
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_3, DataLength);
	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_3, (uint32_t)&DummyByte);	
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_3);
	LL_DMA_DisableIT_TC(DMA2, LL_DMA_STREAM_3);
	LL_SPI_EnableDMAReq_TX(SPI1);
	
}


/****************************************************************/
/* ADS1299_Channel_Config()                                                  */
/** Operation:
 *      - Configuring ADS1299 parameters
 *
 * Parameters:
 *      - div:ADS1299 chip number
 *      - gain:The gain of ADS1299
 *      - sample:The sampling rate of ADS1299
 *
 * Return value:
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/
void ADS1299_Channel_Config(uint8_t dev, uint8_t channel, TADS1299CHnSET Para)
{
	ADS1299_WriteREG (0, (ADS1299_REG_CH1SET + channel), Para.value );
}

/****************************************************************/
/* ADS1299_Parameter_Config()                                   */
/** Operation:
 *      - Configuring ADS1299 parameters
 *
 * Parameters:
 *      - div:ADS1299 chip number
 *      - gain:The gain of ADS1299
 *      - sample:The sampling rate of ADS1299
 *
 * Return value:
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/
void ADS1299_Parameter_Config(uint8_t ADS1299_ParaGroup, uint8_t sample,uint8_t gain)
{
	uint8_t i = 0;
	TADS1299CHnSET     ChVal;
	TADS1299CONFIG1    CFG1;
	TADS1299CONFIG2    CFG2;
	TADS1299CONFIG3    CFG3;
	TADS1299CONFIG4    CFG4;
	TADS1299LOFF       LOFF;
	TADS1299BIASSENSP  BIASSP;
	TADS1299MISC1      MISC1;
	ChVal.control_bit.pd = 0;
	ChVal.control_bit.gain = gain;   // Gain = 24x
	

	
//	if(ADS1299_ParaGroup == 0)					//test signal 
//	{
//		ChVal.control_bit.mux = 5;    // Test signal
//	}
//	else if(ADS1299_ParaGroup == 1) 			//EEG
//	{
//		ChVal.control_bit.mux = 5;    // EEG signal
//	}
//	ChVal.control_bit.srb2 = 0;   // SRB2 Open
	
	switch (ADS1299_ParaGroup)
  {
  	case ADS1299_ParaGroup_ACQ:
  	{
			CFG1.control_bit.dr = 6;    //Sample rate 250Hz
			CFG1.control_bit.res7 = 1;
			CFG1.control_bit.rsv4 = 2;
			
			CFG2.value = 0xC0;
			
			CFG3.value = 0x60;
			CFG3.control_bit.pdbrefbuf = 1;
			CFG3.control_bit.pdbbias = 1;
			CFG3.control_bit.biasrefint = 1;
			
			CFG4.value = 0x00;
			
			LOFF.value = 0x00;
			
			
			ChVal.control_bit.gain = 6;
			ChVal.control_bit.pd = 0;
			ChVal.control_bit.mux = 0;
			
			
			BIASSP.value = 0xFF;
			MISC1.control_bit.srb1 = 1;
			
			break;
		}
  	case ADS1299_ParaGroup_IMP:
  		break;
		case ADS1299_ParaGroup_STBY:
  		break;
		case ADS1299_ParaGroup_TSIG:
  	{
			CFG1.control_bit.dr = 6;    //Sample rate 250Hz
			CFG1.control_bit.res7 = 1;
			CFG1.control_bit.rsv4 = 2;
			
			CFG2.value = 0xC0;
			CFG2.control_bit.inttest = 1;
			CFG2.control_bit.testamp = 0;
			CFG2.control_bit.testfreq = 1;
			
			CFG3.value = 0x60;
			CFG3.control_bit.pdbrefbuf = 1;
			CFG3.control_bit.pdbbias = 1;
			CFG3.control_bit.biasrefint = 1;
			
			CFG4.value = 0x00;
			
			LOFF.value = 0x00;
			
			
			ChVal.control_bit.gain = 1;
			ChVal.control_bit.pd = 0;
			ChVal.control_bit.mux = 5;
			
			BIASSP.value = 0xFF;
			MISC1.control_bit.srb1 = 1;
			
			break;
		}
  	default:
  		break;
  }
	
	
	
	switch(sample)
	{
		case 1:		//250Hz
		{
			ADS1299_WriteREG(0,ADS1299_REG_CONFIG1,0xF6);		//250HZ����	
			break;
		}
		case 2:		
			//500Hz
		{
			ADS1299_WriteREG(0,ADS1299_REG_CONFIG1,0xF5);		//500HZ����	
			break;
		}
		case 3:		//1000Hz
		{
			ADS1299_WriteREG(0,ADS1299_REG_CONFIG1,0xF4);		//1000HZ����	
			break;
		}
		break;
	}
	WaitUs(2);
	
	
	ADS1299_WriteREG(0,ADS1299_REG_BIASSENSP,0xFF);	
	WaitUs(2);
	ADS1299_WriteREG(0,ADS1299_REG_BIASSENSN,0xFF);
	WaitUs(2);
	ADS1299_WriteREG(0,ADS1299_REG_MISC1,0x20);		// SRB1ͳһ�ο�
	WaitUs(2);
	ADS1299_WriteREG(0,ADS1299_REG_LOFF,0x00);			
	WaitUs(2);
	ADS1299_WriteREG(0,ADS1299_REG_LOFFSENSP,0x00);	
	WaitUs(2);
	ADS1299_WriteREG(0,ADS1299_REG_LOFFSENSN,0x00);	
	WaitUs(2);
	ADS1299_WriteREG(0,ADS1299_REG_BIASSENSP,0xFF);		
	WaitUs(2);
	ADS1299_WriteREG(0,ADS1299_REG_BIASSENSN,0xFF);
	WaitUs(2);
	
	
	for(i=0;i<8;i++)
	{
		ADS1299_Channel_Config(0,i,ChVal);
		WaitUs(2);
	}

		
}
/****************************************************************/
/* ADS1299_Mode_Config()                               */
/** Operation:
 *      - Configuring ADS1299 Mode Parameters
 *
 * Mode:
 *      - EEG_Acq
 *      - IMP_Meas
 *
 *
 * Return value:
 *      - 0 Configuration Done
 *      - 1 Configuration Failed
 *
 * Globals modified:
 *     - None
 *
 * Resources used:
 *     - None
 */
/****************************************************************/

uint8_t ADS1299_Mode_Config(uint8_t Mode)
{
	uint8_t i,j;
	uint8_t ReadResult;
	
	switch (Mode)
  {
		case 1://EEG_Acq
		{
			ADS1299_WriteREG(0,ADS1299_REG_CONFIG1,0x96);
			ADS1299_WriteREG(0,ADS1299_REG_CONFIG2,0xC0);
			ADS1299_WriteREG(0,ADS1299_REG_CONFIG3,0xEC);
			
			
			ADS1299_WriteREG(0,ADS1299_REG_LOFF,0x00);
			ADS1299_WriteREG(0,ADS1299_REG_LOFFSENSN,0x00);
			ADS1299_WriteREG(0,ADS1299_REG_LOFFSENSP,0x00);
			ADS1299_WriteREG(0,ADS1299_REG_BIASSENSN,0x00);
			ADS1299_WriteREG(0,ADS1299_REG_BIASSENSP,0xFF);
			ADS1299_WriteREG(0,ADS1299_REG_MISC1,0x20);
			
			for(i=0;i<8;i++)
			{
				// Gain = 24
				ADS1299_WriteREG(0,ADS1299_REG_CH1SET+i,0x60);
				HAL_Delay(50);
				
				// �ض�1��
					ReadResult = ADS1299_ReadREG(0,ADS1299_REG_CH1SET+i);
					ReadResult = ADS1299_ReadByte();
					ReadResult = ADS1299_ReadByte();
					ReadResult = ADS1299_ReadByte();
					if(ReadResult!=0x60)	
							ADS1299_WriteREG(0,ADS1299_REG_CH1SET+i,0x60);
		
			}
			
			
			break;
		}
		
		case 2://IMP_Meas
		{
			ADS1299_WriteREG(1,ADS1299_REG_LOFF,0x09);				//[3:2]=00(6nA),01(24nA),10(6uA),11(24uA); [1:0]=01(7.8Hz),10(31.2Hz)
			ADS1299_WriteREG(1,ADS1299_REG_LOFFSENSN,0xFF);
			ADS1299_WriteREG(1,ADS1299_REG_LOFFSENSP,0xFF);
			ADS1299_WriteREG(1,ADS1299_REG_BIASSENSN,0x00);
			ADS1299_WriteREG(1,ADS1299_REG_BIASSENSP,0x00);

			
			for(i=0;i<32;i++)
			{
				// Gain = 24
				ADS1299_WriteREG(0,ADS1299_REG_CH1SET+i,0x00);
				WaitUs(2);
				WaitUs(10);
				//ReadResult = ADS1299_ReadREG(0,ADS1299_REG_CH1SET+i);	
			}
			
			break;
		}

  }
}


