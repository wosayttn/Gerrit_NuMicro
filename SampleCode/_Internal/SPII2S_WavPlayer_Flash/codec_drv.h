/*******************************************************************************
 * @file        codec_drv.h
 * @brief       Codec driver
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __CODEC_DRV_H__
#define __CODEC_DRV_H__

/*---------------------------------------------------------------------------*/
/* Functions                                                                 */
/*---------------------------------------------------------------------------*/
void NAU8822_Setup(uint32_t u32SampleRate, uint32_t u32AudioFormat);

#endif /* #ifndef __CODEC_DRV_H__ */

#ifdef __cplusplus
}

#endif
