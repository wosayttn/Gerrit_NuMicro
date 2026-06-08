/**************************************************************************//**
 * @file     dfmc.h
 * @version  V1.00
 * @brief    M2003J Series Data Flash Memory Controller Driver Header File
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __DFMC_H__
#define __DFMC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup DFMC_Driver DFMC Driver
  @{
*/


/** @addtogroup DFMC_EXPORTED_CONSTANTS DFMC Exported Constants
  @{
*/


/*---------------------------------------------------------------------------------------------------------*/
/* Define Base Address                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define DFMC_FLASH_BASE          0x0F400000UL    /*!< APROM base address          \hideinitializer */
#define DFMC_FLASH_END           0x0F402000UL    /*!< APROM end address           \hideinitializer */

#define DFMC_FLASH_PAGE_SIZE     0x200UL        /*!< Flash Page Size (4K bytes)   \hideinitializer */
#define DFMC_PAGE_ADDR_MASK      0xFFFFF000UL    /*!< Flash page address mask     \hideinitializer */

#define DFMC_FLASH_SIZE         (DFMC_FLASH_END - DFMC_FLASH_BASE)/*!< APROM Size \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  ISPCMD constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define DFMC_ISPCMD_READ         0x00UL          /*!< ISP Command: Read flash word         \hideinitializer */
#define DFMC_ISPCMD_READ_UID     0x04UL          /*!< ISP Command: Read Unique ID          \hideinitializer */
#define DFMC_ISPCMD_READ_ALL1    0x08UL          /*!< ISP Command: Read all-one result     \hideinitializer */
#define DFMC_ISPCMD_READ_CID     0x0BUL          /*!< ISP Command: Read Company ID         \hideinitializer */
#define DFMC_ISPCMD_READ_DID     0x0CUL          /*!< ISP Command: Read Device ID          \hideinitializer */
#define DFMC_ISPCMD_READ_CKS     0x0DUL          /*!< ISP Command: Read checksum           \hideinitializer */
#define DFMC_ISPCMD_PROGRAM      0x21UL          /*!< ISP Command: Write flash word        \hideinitializer */
#define DFMC_ISPCMD_PAGE_ERASE   0x22UL          /*!< ISP Command: Page Erase Flash        \hideinitializer */
#define DFMC_ISPCMD_RUN_ALL1     0x28UL          /*!< ISP Command: Run all-one verification \hideinitializer */
#define DFMC_ISPCMD_RUN_CKS      0x2DUL          /*!< ISP Command: Run checksum calculation \hideinitializer */

#define DREAD_ALLONE_YES         0xA11FFFFFUL    /*!< Check-all-one result is all one.     \hideinitializer */
#define DREAD_ALLONE_NOT         0xA1100000UL    /*!< Check-all-one result is not all one. \hideinitializer */
#define DREAD_ALLONE_CMD_FAIL    0xFFFFFFFFUL    /*!< Check-all-one command failed.        \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* DFMC Time-out Handler Constant Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define DFMC_TIMEOUT_READ        (SystemCoreClock>>3)    /*!< Read command time-out 125 ms        \hideinitializer */
#define DFMC_TIMEOUT_WRITE       (SystemCoreClock>>3)    /*!< Write command time-out 125 ms       \hideinitializer */
#define DFMC_TIMEOUT_ERASE       (SystemCoreClock>>2)    /*!< Erase command time-out 250 ms       \hideinitializer */
#define DFMC_TIMEOUT_CHKSUM      (SystemCoreClock<<1)    /*!< Get checksum command time-out 2 s   \hideinitializer */
#define DFMC_TIMEOUT_CHKALLONE   (SystemCoreClock<<1)    /*!< Check-all-one command time-out 2 s  \hideinitializer */
#define DFMC_OK                  ( 0L)                   /*!< FMC operation OK                         \hideinitializer */
#define DFMC_ERR_FAIL            (-1L)                   /*!< FMC operation failed                     \hideinitializer */
#define DFMC_ERR_TIMEOUT         (-2L)                   /*!< FMC operation abort due to timeout error \hideinitializer */

/*@}*/ /* end of group DFMC_EXPORTED_CONSTANTS */


/** @addtogroup DFMC_EXPORTED_MACROS FMC Exported Macros
  @{
*/


/*---------------------------------------------------------------------------------------------------------*/
/*  Macros                                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/


/**
 * @brief      Enable Data Flash Update Function
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function will set DATAEN bit of DFMC_ISPCTL control register to enable Data Flash update function.
 *             User needs to set DATAEN bit before they can update LDROM.
 *
 */
#define DFMC_ENABLE_DF_UPDATE()      (DFMC->ISPCTL |=  DFMC_ISPCTL_DATAEN_Msk)      /*!< Enable Data Flash update        \hideinitializer */

/**
 * @brief      Disable Data Flash Update Function
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function will set DATAEN bit of DFMC_ISPCTL control register to disable Data Flash update function.
 *
 */
#define DFMC_DISABLE_DF_UPDATE()     (DFMC->ISPCTL &= ~DFMC_ISPCTL_DATAEN_Msk)      /*!< Disable Data Flash update       \hideinitializer */

/**
 * @brief      Enable ISP Function
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function will set ISPEN bit of DFMC_ISPCTL control register to enable ISP function.
 *
 */
#define DFMC_ENABLE_ISP()            (DFMC->ISPCTL |=  DFMC_ISPCTL_ISPEN_Msk)                    /*!< Enable ISP function        \hideinitializer */

/**
 * @brief      Disable ISP Function
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function will clear ISPEN bit of DFMC_ISPCTL control register to disable ISP function.
 *
 */
#define DFMC_DISABLE_ISP()           (DFMC->ISPCTL &= ~DFMC_ISPCTL_ISPEN_Msk)                    /*!< Disable ISP function       \hideinitializer */

/**
 * @brief      Get ISP Fail Flag
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function is used to get ISP fail flag when do ISP action.
 *
 */
#define DFMC_GET_FAIL_FLAG()         ((DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk) ? 1UL : 0UL)  /*!< Get ISP fail flag  \hideinitializer */

/**
 * @brief      Clear ISP Fail Flag
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function is used to clear ISP fail flag when ISP fail flag set.
 *
 */
#define DFMC_CLR_FAIL_FLAG()         (DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk)       /*!< Clear ISP fail flag        \hideinitializer */

/**
 * @brief      Enable ISP Interrupt
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function will enable ISP action interrupt.
 *
 */
#define DFMC_ENABLE_ISP_INT()     (DFMC->ISPCTL |=  DFMC_ISPCTL_ISPIFEN_Msk) /*!< Enable ISP interrupt */

/**
 * @brief      Disable ISP Interrupt
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function will disable ISP action interrupt.
 *
 */
#define DFMC_DISABLE_ISP_INT()     (DFMC->ISPCTL &= ~DFMC_ISPCTL_ISPIFEN_Msk) /*!< Disable ISP interrupt */

/**
 * @brief      Get ISP Interrupt Flag
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function will get ISP action interrupt status
 *
 */
#define DFMC_GET_ISP_INT_FLAG()     ((DFMC->ISPSTS & DFMC_ISPSTS_ISPIF_Msk) ? 1UL : 0UL) /*!< Get ISP interrupt flag Status */

/**
 * @brief      Clear ISP Interrupt Flag
 *
 * @param      None
 *
 * @return     None
 *
 * @details    This function will clear ISP interrupt flag
 *
 */
#define DFMC_CLEAR_ISP_INT_FLAG()     (DFMC->ISPSTS = DFMC_ISPSTS_ISPIF_Msk) /*!< Clear ISP interrupt flag */


/*@}*/ /* end of group DFMC_EXPORTED_MACROS */

/*---------------------------------------------------------------------------------------------------------*/
/*  Global variables                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
extern int32_t  g_DFMC_i32ErrCode;

/** @addtogroup DFMC_EXPORTED_FUNCTIONS DFMC Exported Functions
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* inline functions                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

__STATIC_INLINE uint32_t DFMC_ReadCID(void);
__STATIC_INLINE uint32_t DFMC_ReadUID(uint8_t u8Index);
__STATIC_INLINE uint32_t DFMC_ReadUCID(uint32_t u32Index);


/**
  * @brief    Read company ID
  * @param    None
  * @return   The company ID (32-bit). 0xFFFFFFFF means read failed.
  * @details  The company ID of Nuvoton is fixed to be 0x530000DA
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           DFMC_ERR_TIMEOUT  Read time-out
  */
__STATIC_INLINE uint32_t DFMC_ReadCID(void)
{
    int32_t i32TimeOutCnt = DFMC_TIMEOUT_READ;

    g_DFMC_i32ErrCode = 0;

    DFMC->ISPCMD = DFMC_ISPCMD_READ_CID;           /* Set ISP Command Code */
    DFMC->ISPADDR = 0x0u;                          /* Must keep 0x0 when read CID */
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;          /* Trigger to start ISP procedure */
    while(DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk)    /* Waiting for ISP Done */
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_DFMC_i32ErrCode = DFMC_ERR_TIMEOUT;
            return 0xFFFFFFFF;
        }
    }

    return DFMC->ISPDAT;
}


/**
 * @brief       Read Unique ID
 * @param[in]   u8Index  UID index. 0 = UID[31:0], 1 = UID[63:32], 2 = UID[95:64]
 * @return      The 32-bit unique ID data of specified UID index. 0xFFFFFFFF means read failed.
 * @details     To read out 96-bit Unique ID.
 *
 * @note        Global error code g_FMC_i32ErrCode
 *              DFMC_ERR_TIMEOUT  Read time-out
 */
__STATIC_INLINE uint32_t DFMC_ReadUID(uint8_t u8Index)
{
    int32_t i32TimeOutCnt = DFMC_TIMEOUT_READ;

    g_DFMC_i32ErrCode = 0;

    DFMC->ISPCMD = DFMC_ISPCMD_READ_UID;
    DFMC->ISPADDR = ((uint32_t)u8Index << 2u);
    DFMC->ISPDAT = 0u;
    DFMC->ISPTRG = 0x1u;
    while(DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk)   /* Waiting for ISP Done */
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_DFMC_i32ErrCode = DFMC_ERR_TIMEOUT;
            return 0xFFFFFFFF;
        }
    }

    return DFMC->ISPDAT;
}


/**
  * @brief      To read UCID
  * @param[in]  u32Index    Index of the UCID to read. u32Index must be 0, 1, 2, or 3.
  * @return     The UCID of specified index
  * @details    This function is used to read unique chip ID (UCID). 0xFFFFFFFF means read failed.
  *
  * @note       Global error code g_FMC_i32ErrCode
  *             DFMC_ERR_TIMEOUT  Read time-out
  */
__STATIC_INLINE uint32_t DFMC_ReadUCID(uint32_t u32Index)
{
    int32_t i32TimeOutCnt = DFMC_TIMEOUT_READ;

    g_DFMC_i32ErrCode = 0;

    DFMC->ISPCMD = DFMC_ISPCMD_READ_UID;            /* Set ISP Command Code */
    DFMC->ISPADDR = (0x04u * u32Index) + 0x10u;     /* The UCID is at offset 0x10 with word alignment. */
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;           /* Trigger to start ISP procedure */
    while(DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk)     /* Waiting for ISP Done */
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_DFMC_i32ErrCode = DFMC_ERR_TIMEOUT;
            return 0xFFFFFFFF;
        }
    }

    return DFMC->ISPDAT;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Functions                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/

extern void     DFMC_Close(void);
extern int32_t  DFMC_Erase(uint32_t u32PageAddr);
extern void     DFMC_Open(void);
extern uint32_t DFMC_Read(uint32_t u32Addr);
extern int32_t  DFMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern uint32_t DFMC_GetChkSum(uint32_t u32addr, uint32_t u32count);
extern uint32_t DFMC_CheckAllOne(uint32_t u32addr, uint32_t u32count);

/*@}*/ /* end of group DFMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group DFMC_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif   /* __DFMC_H__ */

