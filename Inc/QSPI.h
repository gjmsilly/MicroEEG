#ifndef  QSPI_H_
#define  QSPI_H_

#include "stdint.h"

void QSPI_Send_Control(uint16_t address,uint8_t controlword,uint32_t dummyCycles,uint32_t dataMode);
void QSPI_Send_Control_IT(uint16_t address,uint8_t controlword,uint32_t dummyCycles,uint32_t dataMode);
uint8_t QSPI_Receive(uint8_t* buf,uint32_t datalen);
uint8_t QSPI_Transmit(uint8_t* buf,uint32_t datalen);
uint8_t QSPI_Transmit_IT(uint8_t* buf,uint32_t datalen);

#endif   //QSPI_H_
