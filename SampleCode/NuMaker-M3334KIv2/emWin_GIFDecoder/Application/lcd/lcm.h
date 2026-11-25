/*******************************************************************************
 *                                                                             *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved. *
 *                                                                             *
 *******************************************************************************/

#ifndef __LCM_H__
#define __LCM_H__

#ifdef  __cplusplus
extern "C"
{
#endif

//unsigned char _Read1(void);
//void _ReadM1(unsigned char * pData, int NumItems);
void _Write0(unsigned char Cmd);
void _Write1(unsigned char Data);
void _WriteM1(unsigned char * pData, int NumItems);
void _InitController(void);
void NVT_FillRect(void);

#ifdef  __cplusplus
}
#endif

#endif  // __LCM_H__
