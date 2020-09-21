/****************************************************************/
/* ADS1299 ADCPro Firmware Version 1.0 for the TMS320C5500      */
/* Copyright (C) 2009 - 2010 Texas Instruments Incorporated     */
/* All Rights Reserved                                          */
/* Modified by Liu Miao 2018.5                                  */
/****************************************************************/
/*
Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use, copy,
modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT.  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/
/****************************************************************/

#ifndef __ADS1299_H
#define __ADS1299_H

#include "main.h"
#include "stm32f4xx_hal.h"



/****************************************************************/
/* return types and return codes                                */
/****************************************************************/
#define TTIDCSTATUS         char
#define TIDC_NO_RETVAL      (1)
#define TIDC_NO_ERR         (0)
#define TIDC_ERR_NODEVICE   (-1)
#define TIDC_ERR_MCBSP      (-2)
#define TIDC_ERR_FXNNULL    (-3)
#define TIDC_ERR_NOCHIPRES  (-4)
#define TIDC_ERR_XFERPROG   (-5)
#define TIDC_ERR_BADARGS    (-6)
#define TIDC_ERR_DMA        (-7)
#define TIDC_ERR_TIMER      (-8)
#define TIDC_ERR_REGS       (-9)



/****************************************************************/
/* the following register definitions mirror the version given  */
/* the datasheet dated Jan 2017                                 */
/****************************************************************/
/** Structure definition for configuration register 0 (ID)      */
/****************************************************************/
typedef union
{
   uint8_t value;
   struct
   {
       
     uint8_t revid              :3;
	   uint8_t res4               :1;
     uint8_t devid              :2;
	   uint8_t nuch							  :2;
   } control_bit;
} TADS1299ID;


/****************************************************************/
/** Structure definition for configuration register 1 (CONFIG1) */
/****************************************************************/
typedef union
{
   uint8_t value;
   struct
   {
       
       uint8_t dr                 :3;
       uint8_t rsv4               :2;
       uint8_t clken              :1;
       uint8_t daisyen            :1;
       uint8_t res7               :1;

   } control_bit;
} TADS1299CONFIG1;


/****************************************************************/
/** Structure definition for configuration register 2 (CONFIG2) */
/****************************************************************/
typedef union
{
   uint8_t value;
   struct
   {
       uint8_t testfreq           :2;
       uint8_t testamp            :1;
       uint8_t rsv3               :1;
       uint8_t inttest            :1;
       uint8_t rsv7               :3;
   } control_bit;
} TADS1299CONFIG2;


/****************************************************************/
/** Structure definition for configuration register 3 (CONFIG3) */
/****************************************************************/
typedef union
{
   uint8_t value;
   struct
   {
       
       uint8_t biasstat           :1;
       uint8_t biasloff           :1;
       uint8_t pdbbias            :1;
       uint8_t biasrefint         :1;
       uint8_t biasmeas           :1;
       uint8_t rsv6               :2;
       uint8_t pdbrefbuf          :1;

   } control_bit;
} TADS1299CONFIG3;


/****************************************************************/
/** Structure definition for lead-off control register (LOFF)   */
/****************************************************************/
typedef union
{
   uint8_t value;
   struct
   {
       
       uint8_t fleadoff           :2;
       uint8_t ileadoff           :2;
       uint8_t rsv4               :1;
       uint8_t compth             :3;

   } control_bit;
} TADS1299LOFF;


/****************************************************************/
/** Structure definition for channel settings register (CHSET)  */
/****************************************************************/
typedef union
{
   uint8_t value;
   struct
   {
       uint8_t mux                :3;
       uint8_t srb2               :1;
       uint8_t gain               :3;
       uint8_t pd                 :1;

   } control_bit;
} TADS1299CHnSET;


/****************************************************************/
/** Structure definition for RLD_SENSP register (RLDSENSP)      */
/****************************************************************/
typedef union
{
   uint8_t value;
   struct
   {
       
       uint8_t BIAS1p             :1;
       uint8_t BIAS2p             :1;
       uint8_t BIAS3p             :1;
       uint8_t BIAS4p             :1;
       uint8_t BIAS5p             :1;
       uint8_t BIAS6p             :1;
       uint8_t BIAS7p             :1;
       uint8_t BIAS8p             :1;

   } control_bit;
} TADS1299BIASSENSP;


/****************************************************************/
/** Structure definition for RLD_SENSN register (RLDSENSN)      */
/****************************************************************/
typedef union
{
   uint8_t value;
   struct
   {
       
       uint8_t BIAS1n             :1;
       uint8_t BIAS2n             :1;
       uint8_t BIAS3n             :1;
       uint8_t BIAS4n             :1;
       uint8_t BIAS5n             :1;
       uint8_t BIAS6n             :1;
       uint8_t BIAS7n             :1;
       uint8_t BIAS8n             :1;

   } control_bit;
} TADS1299BIASSENSN;


/****************************************************************/
/** Structure definition for LOFF_SENSP register (LOFFSENSP)    */
/****************************************************************/
typedef union
{
   uint8_t value;
   struct
   {
       
       uint8_t loff1p             :1;
       uint8_t loff2p             :1;
       uint8_t loff3p             :1;
       uint8_t loff4p             :1;
       uint8_t loff5p             :1;
       uint8_t loff6p             :1;
       uint8_t loff7p             :1;
       uint8_t loff8p             :1;

   } control_bit;
} TADS1299LOFFSENSP;


/****************************************************************/
/** Structure definition for LOFF_SENSN register (LOFFSENSN)    */
/****************************************************************/
typedef union
{
   uint8_t value;
   struct
   {
       
       uint8_t loff1n             :1;
       uint8_t loff2n             :1;
       uint8_t loff3n             :1;
       uint8_t loff4n             :1;
       uint8_t loff5n             :1;
       uint8_t loff6n             :1;
       uint8_t loff7n             :1;
       uint8_t loff8n             :1;

   } control_bit;
} TADS1299LOFFSENSN;


/****************************************************************/
/** Structure definition for LOFF_FLIP register (LOFFFLIP)      */
/****************************************************************/
typedef union
{
   uint8_t value;
   struct
   {
       
       uint8_t loffflip1          :1;
       uint8_t loffflip2          :1;
       uint8_t loffflip3          :1;
       uint8_t loffflip4          :1;
       uint8_t loffflip5          :1;
       uint8_t loffflip6          :1;
       uint8_t loffflip7          :1;
       uint8_t loffflip8          :1;

   } control_bit;
} TADS1299LOFFFLIP;


/****************************************************************/
/** Structure definition for lead off status P register
    (LOFFSTATP)                                                 */
/****************************************************************/
typedef union
{
   uint8_t value;
   struct
   {
       
       uint8_t in1poff            :1;
       uint8_t in2poff            :1;
       uint8_t in3poff            :1;
       uint8_t in4poff            :1;
       uint8_t in5poff            :1;
       uint8_t in6poff            :1;
       uint8_t in7poff            :1;
       uint8_t in8poff            :1;

   } control_bit;
} TADS1299LOFFSTATP;


/****************************************************************/
/** Structure definition for lead off status N register
    (LOFFSTATN)                                                 */
/****************************************************************/
typedef union
{
   uint8_t value;
   struct
   {
       
       uint8_t in1noff            :1;
       uint8_t in2noff            :1;
       uint8_t in3noff            :1;
       uint8_t in4noff            :1;
       uint8_t in5noff            :1;
       uint8_t in6noff            :1;
       uint8_t in7noff            :1;
       uint8_t in8noff            :1;

   } control_bit;
} TADS1299LOFFSTATN;


/****************************************************************/
/** Structure definition for GPIO register (GPIO)               */
/****************************************************************/
typedef union
{
   uint8_t value;
   struct
   {
       
       uint8_t gpioc1             :1;
       uint8_t gpioc2             :1;
       uint8_t gpioc3             :1;
       uint8_t gpioc4             :1;
       uint8_t gpiod1             :1;
       uint8_t gpiod2             :1;
       uint8_t gpiod3             :1;
       uint8_t gpiod4             :1;

   } control_bit;
} TADS1299GPIO;


/****************************************************************/
/** Structure definition for PACE detect register (PACE)        */
/****************************************************************/
typedef union
{
   uint8_t value;
   struct
   {
       uint8_t rsv4               :5;
       uint8_t srb1               :1;
       uint8_t rsv7               :2;

  } control_bit;
} TADS1299MISC1;


/****************************************************************/
/** Structure definition for respiration control reg (RESP)     */
/****************************************************************/
typedef union
{
  uint8_t value;
  struct
  {
      
      uint8_t rsv                :8;

 } control_bit;
} TADS1299MISC2;


/****************************************************************/
/** Structure definition for config 4 control reg (CONFIG4)     */
/****************************************************************/
typedef union
{
   uint8_t value;
   struct
   {
       
       uint8_t rsv0               :1;
       uint8_t pdbloffcomp        :1;
       uint8_t rsv2               :1;
       uint8_t singleshot         :1;
       uint8_t rsv7               :4;

  } control_bit;
} TADS1299CONFIG4;

/****************************************************************/
/** Data converter register structure definition.
 *  This structure can be used to access the initialization
 *  values for the ADS1299 from the data converter object.      */
/****************************************************************/
typedef struct
{
    TADS1299ID          devid;
    TADS1299CONFIG1     config1;
    TADS1299CONFIG2     config2;
    TADS1299CONFIG3     config3;
    TADS1299LOFF        loff;
    TADS1299CHnSET      ch1set;
    TADS1299CHnSET      ch2set;
    TADS1299CHnSET      ch3set;
    TADS1299CHnSET      ch4set;
    TADS1299CHnSET      ch5set;
    TADS1299CHnSET      ch6set;
    TADS1299CHnSET      ch7set;
    TADS1299CHnSET      ch8set;
    TADS1299BIASSENSP   biassensp;
    TADS1299BIASSENSN   biassensn;
    TADS1299LOFFSENSP   loffsensp;
    TADS1299LOFFSENSN   loffsensn;
    TADS1299LOFFFLIP    loffflip;
    TADS1299LOFFSTATP   loffstatp;
    TADS1299LOFFSTATN   loffstatn;
    TADS1299GPIO        gpio;
    TADS1299MISC1       misc1;
    TADS1299MISC2       misc2;
    TADS1299CONFIG4     config4;
} TADS1299REGS;

/****************************************************************/
/** Buffer object definition for dual buffering                 */
/****************************************************************/
typedef struct
{
    void             (*ptrCallBack)(void *);/**< ptr to callback*/
    unsigned long    *ulBufPtr;      /**< read buffer pointer   */
    unsigned long    ulBufSize;      /**< read buffer size      */
    volatile unsigned int uiStatus;  /**<  status of the buffer */
} TADS1299BUFOBJ;

/****************************************************************/
/** \struct TADS1299
 *  Definition of the data converter object. This is the main
 *  object to work with.                                        */
/****************************************************************/
typedef struct
{
/** \var  DCP_ SERIAL *serial
 *   Pointer to the McBSP configuration
 *   structure as defined in the tidc_api.h.                    */
/** \var regs
 *  Structure for the registers of the ADS1299. This structure
 *  will be initialized by the code and the values there will
 *  be sent to the device during dc_configure(). Any change
 *  the register values need to be submitted to the device
 *  using the dc_control() function.                            */
/** \var initRegs
 *  This structure is the backup structure for TADS1299REGS regs.
 *  At startup, it is initialized to the same values as regs, but
 *  will not be changed during the program. The function
 *  dc_close() will use these values to re-initialize the
 *  converter with the original register settings.              */
/** \var xferBuffer
 *  Holds the structure of the tye TADS1299BUFOBJ, which is used
 *  to hold information about the submitted buffer              */
/** \var hDmaRcv
 *  Handle to the DMA channel for the data transfer from the
 *  serial port to the memory array. This handle will be created
 *  by the function DMA_open(), which is called during
 *  dc_configure().                                             */
/** \var hDmaXmt
 *  Handle to the DMA channel for the data transfer from the
 *  dummy memory location to the McBSP. This transfer is necessary
 *  to trigger the activity on the SCLK line. The handle will be
 *  created by the function DMA_open(), which is called during
 *  dc_configure().                                             */
/** \var iDmaChanNumber
 *  Number of the DMA channel to be used to collect the data
 *  from the data converter. This channel will be used to create
 *  the hDmRcv handle.                                          */
/** \var uiDrdyIntNum
 *  This variable holds the number of the interrupt line of the
 *  DSP, where the DRDY line is connected to.                   */
/** \var uiMcbspPeriod
 *  Value for the McBSP Period Register. Will be used to set the
 *  speed of the serial port during dc_configure().             */
/** \var iXferInProgress
 *  This value indicates if there is a data transfer ongoing and
 *  can have the following values:
 *      - 0: No transfer is ongoing
 *      - 1: One transfer is ongoing                            */
/** \var uiRcvCcrValue
 *  Backup for the receive DMA Channel Control Register.        */
/** \var uiXmtCcrValue
 *  Backup for the receive DMA Channel Control Register.        */
/****************************************************************/
    TADS1299REGS       regs;
    TADS1299REGS       initRegs;
    TADS1299BUFOBJ     xferBuffer;
    volatile int       iXferInProgress;
    unsigned int       uiRcvCcrValue;
    unsigned int       uiXmtCcrValue;
} TADS1299;



/****************************************************************/
/* Define commands for the dc_control() function                */
/****************************************************************/
/* the defines for the read register and the write register
   commands are shifted by 16 bits, as these are 24-bit commands
   (16-bit for the command and 8-bits for the content).
   All other commands are as described in the data sheet        */
#define ADS1299_CMD_RREG                        (0x2000u)
#define ADS1299_CMD_WREG                        (0x4000u)
#define ADS1299_CMD_WAKEUP                      (0x0002u)
#define ADS1299_CMD_STANDBY                     (0x0004u)
#define ADS1299_CMD_RESET                       (0x0006u)
#define ADS1299_CMD_START                       (0x0008u)
#define ADS1299_CMD_STOP                        (0x000Au)
#define ADS1299_CMD_RDATAC                      (0x0010u)
#define ADS1299_CMD_SDATAC                      (0x0011u)
#define ADS1299_CMD_INITDEVICE                  (0x0100u)
/* offset calibration was removed from the new silicon          */
//#define ADS1299_CMD_OFFCAL                      (0x001Au)


/****************************************************************/
/* Define names for the register addresses                      */
/****************************************************************/
#define ADS1299_REG_DEVID         (0x0000u)
#define ADS1299_REG_CONFIG1       (0x0001u)
#define ADS1299_REG_CONFIG2       (0x0002u)
#define ADS1299_REG_CONFIG3       (0x0003u)
#define ADS1299_REG_LOFF          (0x0004u)
#define ADS1299_REG_CH1SET        (0x0005u)
#define ADS1299_REG_CH2SET        (0x0006u)
#define ADS1299_REG_CH3SET        (0x0007u)
#define ADS1299_REG_CH4SET        (0x0008u)
#define ADS1299_REG_CH5SET        (0x0009u)
#define ADS1299_REG_CH6SET        (0x000Au)
#define ADS1299_REG_CH7SET        (0x000Bu)
#define ADS1299_REG_CH8SET        (0x000Cu)
#define ADS1299_REG_BIASSENSP     (0x000Du)
#define ADS1299_REG_BIASSENSN     (0x000Eu)
#define ADS1299_REG_LOFFSENSP     (0x000Fu)
#define ADS1299_REG_LOFFSENSN     (0x0010u)
#define ADS1299_REG_LOFFFLIP      (0x0011u)
#define ADS1299_REG_LOFFSTATP     (0x0012u)
#define ADS1299_REG_LOFFSTATN     (0x0013u)
#define ADS1299_REG_GPIO          (0x0014u)
#define ADS1299_REG_MISC1         (0x0015u)
#define ADS1299_REG_MISC2         (0x0016u)
#define ADS1299_REG_CONFIG4       (0x0017u)


#define ADS1299_ParaGroup_ACQ   1
#define ADS1299_ParaGroup_IMP   2
#define ADS1299_ParaGroup_STBY  3
#define ADS1299_ParaGroup_TSIG  4


#define Mod_CS_Enable	 LL_GPIO_ResetOutputPin(Mod_nCS_GPIO_Port, Mod_nCS_Pin);
#define Mod_CS_Disable LL_GPIO_SetOutputPin(Mod_nCS_GPIO_Port, Mod_nCS_Pin);
#define Mod_RESET_L    LL_GPIO_ResetOutputPin(Mod_nRESET_GPIO_Port, Mod_nRESET_Pin);
#define Mod_RESET_H    LL_GPIO_SetOutputPin(Mod_nRESET_GPIO_Port, Mod_nRESET_Pin);
#define Mod_PDWN_L     LL_GPIO_ResetOutputPin( Mod_nPWDN_GPIO_Port,  Mod_nPWDN_Pin);
#define Mod_PDWN_H     LL_GPIO_SetOutputPin( Mod_nPWDN_GPIO_Port,  Mod_nPWDN_Pin);

#define Mod_DRDY_INT_Enable    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_7); // Mod1_nDRDY PD7
#define Mod_DRDY_INT_Disable   LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_7);


void WaitUs(int iWaitUs);
void ADS1299_Init(uint8_t dev);
void ADS1299_Reset(uint8_t dev);
void ADS1299_PowerOn(uint8_t dev);
void ADS1299_WriteREG(uint8_t dev, uint8_t address, uint8_t value);
uint8_t ADS1299_ReadREG(uint8_t dev, uint8_t address);
void ADS1299_SendCommand(uint8_t command);

void ADS1299_ReadResult(uint8_t *result);
void ADS1299_Parameter_Config(uint8_t mode,uint8_t sample,uint8_t gain);
void ADS1299_Channel_Config(uint8_t dev, uint8_t channel, TADS1299CHnSET Para);
//void ADS1299_Mode_Config(uint8_t mode);
uint8_t ADS1299_ReadByte(void);
void ADS1299_ReadResult_DMA(uint32_t DataHeadAddress, uint8_t DataLength);


uint8_t ADS1299_Mode_Config(uint8_t);
#endif /* __TADS1299_FN_H */

/****************************************************************/
/* END OF t1299_fn.h                                             */
/****************************************************************/
