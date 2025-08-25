/**************************************************************************//**
 * @file     dpm.h
 * @version  V1.00
 * @brief    Debug Protection Mechanism driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __DPM_H__
#define __DPM_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
    @{
*/

/** @addtogroup DPM_Driver DPM Driver
    @{
*/

/** @addtogroup DPM_EXPORTED_CONSTANTS DPM Exported Constants
    @{
*/

typedef enum
{
    eDPM_STS_DEFAULT  = 0,
    eDPM_STS_CLOSE    = 1,
    eDPM_STS_LOCKED   = 2,
    eDPM_STS_OPEN     = 5
} E_DPM_STS;

/*---------------------------------------------------------------------------------------------------------*/
/* DPM Control Register Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define SECURE_DPM          0   /*!< Secure DPM module */
#define NONSECURE_DPM       1   /*!< Non-secure DPM module */

#define DPM_MISC_WVCODE     (0x5AUL << DPM_MISC_RWVCODE_Pos)        /*!< Secure DPM MISC register write verify code */
#define DPM_MISC_RVCODE     (0xA5UL << DPM_MISC_RWVCODE_Pos)        /*!< Secure DPM MISC register read verify code */

/*---------------------------------------------------------------------------------------------------------*/
/* DPM Time-out Handler Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define DPM_TIMEOUT         (SystemCoreClock)   /*!< 1 second time-out */

/*---------------------------------------------------------------------------------------------------------*/
/* DPM Error Code Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define DPM_TIMEOUT_ERR     (-1L)               /*!< DPM time-out error value */

/** @}*/ /* end of group WDT_EXPORTED_CONSTANTS */

/** @} end of group DPM_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __DPM_H__ */
