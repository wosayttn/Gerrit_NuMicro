/**************************************************************************//**
 * @file     scaling.h
 * @version  V1.00
 * @brief    Image scaling test code related header
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef  __SCALING_H__
#define  __SCALING_H__
#include <stdlib.h>
#include <inttypes.h>
#include <arm_mve.h>

/****************************************************************************
 * Macros Declaration
 ****************************************************************************/
#define CHECK_888RESULT
#define SW_SCALEUP_TEST
#define SRC_IMG_WIDTH    320
#define SRC_IMG_HEIGHT   240
#define SCALED_IMG_WIDTH    128
#define SCALED_IMG_HEIGHT   128
#define RGB_BYTES      3

/****************************************************************************
 * Data Structure
 ****************************************************************************/


/****************************************************************************
 * Global Varibles
 ****************************************************************************/
extern uint8_t image_src[SRC_IMG_WIDTH * SRC_IMG_HEIGHT * RGB_BYTES];
extern uint8_t image_dst[SCALED_IMG_WIDTH * SCALED_IMG_HEIGHT * RGB_BYTES];
/****************************************************************************
 * Function Declaration
 ****************************************************************************/
#ifdef  __cplusplus
extern  "C" {
#endif
int32_t scaling_nonvectorize(void);
#ifdef  __cplusplus
}
#endif
#endif

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
