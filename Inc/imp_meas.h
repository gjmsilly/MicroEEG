/**
 * @file    imp_meas.h
 * @author  gjmsilly
 * @brief   MicroEEG �迹������
 * @version 0.1
 * @date    2022-08-21
 * @copyright (c) 2022 gjmsilly
 *
 */
#ifndef __IMP_MEAS_H__
#define __IMP_MEAS_H__

#include <stdint.h>

/*******************************************************************
 * MACROS
 */

#define CHX_IMPING		0	//һͨ���迹��⴦����
#define	CHX_IMP_CPL		1	//һͨ���迹������


/*********************************************************************
 * FUNCTIONS
 */
uint32_t imp_control( uint8_t chx_process);

#endif