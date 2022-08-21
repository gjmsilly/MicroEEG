/**
 * @file    imp_meas.h
 * @author  gjmsilly
 * @brief   MicroEEG 阻抗检测服务
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

#define CHX_IMPING		0	//一通道阻抗检测处理中
#define	CHX_IMP_CPL		1	//一通道阻抗检测完毕


/*********************************************************************
 * FUNCTIONS
 */
uint32_t imp_control( uint8_t chx_process);

#endif