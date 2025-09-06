/******************************************************************************
 * @file     dfmc.h
 * @version  V1.00
 * @brief    DFMC driver header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
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

#define NOT_RELEASED
/*---------------------------------------------------------------------------------------------------------*/
/*  ISPCMD Constant Definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define DFMC_ISPCMD_READ         0x00UL         /*!< ISP Command: Read flash word           \hideinitializer */
#define DFMC_ISPCMD_READ_UID     0x04UL         /*!< ISP Command: Read Unique ID            \hideinitializer */
#define DFMC_ISPCMD_READ_ALL1    0x08UL         /*!< ISP Command: Read all-one result       \hideinitializer */
#define DFMC_ISPCMD_READ_CID     0x0BUL         /*!< ISP Command: Read Company ID           \hideinitializer */
#define DFMC_ISPCMD_READ_DID     0x0CUL         /*!< ISP Command: Read Device ID            \hideinitializer */
#define DFMC_ISPCMD_READ_CKS     0x0DUL         /*!< ISP Command: Read checksum             \hideinitializer */
#define DFMC_ISPCMD_PROGRAM      0x21UL         /*!< ISP Command: Write flash word          \hideinitializer */
#define DFMC_ISPCMD_PAGE_ERASE   0x22UL         /*!< ISP Command: Page Erase Flash          \hideinitializer */
#define DFMC_ISPCMD_MASS_ERASE   0x26UL         /*!< ISP Command: Erase whole Flash         \hideinitializer */
#define DFMC_ISPCMD_CFG_ERASE    0x24UL         /*!< ISP Command: Erase config word         \hideinitializer */
#define DFMC_ISPCMD_RUN_ALL1     0x28UL         /*!< ISP Command: Run all-one verification  \hideinitializer */
#define DFMC_ISPCMD_RUN_CKS      0x2DUL         /*!< ISP Command: Run checksum calculation  \hideinitializer */
#define DFMC_ISPCMD_PROGRAM_ECC  0x41UL         /*!< ISP Command: Write flash word with ECC \hideinitializer */
#define DFMC_ISPCMD_READ_ECC     0x60UL         /*!< ISP Command: Read flash word with ECC  \hideinitializer */

#define DFMC_ISPCMD_EEPROM_INIT         0x33
#define DFMC_ISPCMD_EEPROM_READ         0x30
#define DFMC_ISPCMD_EEPROM_WRITE_8Bit   0x31
#define DFMC_ISPCMD_EEPROM_WRITE_32Bit  0x32

#ifdef NOT_RELEASED
#define DFMC_ISPCMD_READ_E       0x0AUL          /*!< ISP Command: Read (Erase Verify)      \hideinitializer */
#define DFMC_ISPCMD_READ_P       0x09UL          /*!< ISP Command: Read (Program Verify)    \hideinitializer */
#endif

#define READ_ALLONE_YES         0xA11FFFFFUL    /*!< Check-all-one result is all one.       \hideinitializer */
#define READ_ALLONE_NOT         0xA1100000UL    /*!< Check-all-one result is not all one.   \hideinitializer */
#define READ_ALLONE_CMD_FAIL    0xFFFFFFFFUL    /*!< Check-all-one command failed.          \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* DFMC Time-out Handler Constant Definitions                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define DFMC_TIMEOUT_READ        (SystemCoreClock >> 3)      /*!< Read command time-out 125 ms               \hideinitializer */
#define DFMC_TIMEOUT_WRITE       (SystemCoreClock >> 3)      /*!< Write command time-out 125 ms              \hideinitializer */
#define DFMC_TIMEOUT_ERASE       (SystemCoreClock >> 2)      /*!< Erase command time-out 250 ms              \hideinitializer */
#define DFMC_TIMEOUT_CHKSUM      (SystemCoreClock << 1)      /*!< Get checksum command time-out 2 s          \hideinitializer */
#define DFMC_TIMEOUT_CHKALLONE   (SystemCoreClock << 1)      /*!< Check-all-one command time-out 2 s         \hideinitializer */
#define DFMC_TIMEOUT_EEPROM_INIT (SystemCoreClock << 1)
#define DFMC_OK                  ( 0L)                       /*!< DFMC operation OK                           \hideinitializer */
#define DFMC_ERR_FAIL            (-1L)                       /*!< DFMC operation failed                       \hideinitializer */
#define DFMC_ERR_TIMEOUT         (-2L)                       /*!< DFMC operation abort due to timeout error   \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* Define Base Address                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define DFMC_DATA_FLASH_BASE        0x0F210000UL                            /*!< Data Flash base address            \hideinitializer */
#define DFMC_DATA_FLASH_SIZE        0x10000UL                               /*!< Data Flash Size                    \hideinitializer */
#define DFMC_DATA_FLASH_END         (DFMC_DATA_FLASH_BASE + DFMC_DATA_FLASH_SIZE)  /*!< Data Flash end address      \hideinitializer */

#define DFMC_EEPROM_BASE       0x0F21E000
#define DFMC_EEPROM_SIZE       0x100
#define DFMC_EEPROM_END        (DFMC_EEPROM_BASE + DFMC_EEPROM_SIZE)

#define DFMC_FLASH_PAGE_SIZE        0x1000UL                                /*!< Flash Page Size (4K bytes)         \hideinitializer */
#define DFMC_FLASH_PAGE_ADDR_MASK   0xFFFFE000UL                            /*!< Flash page address mask            \hideinitializer */

/*@}*/ /* end of group FMC_EXPORTED_CONSTANTS */

/** @addtogroup DFMC_EXPORTED_MACROS DFMC Exported Macros
    @{
*/
/*---------------------------------------------------------------------------------------------------------*/
/*  Macros                                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
/**
* @brief      Get ISP Fail Flag
* @param      None
* @return     None
* @details    This function is used to get ISP fail flag when do ISP action.
*/
#define DFMC_GET_FAIL_FLAG()         ((DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk) ? 1UL : 0UL)          /*!< Get ISP fail flag          \hideinitializer */

/**
* @brief      Clear ISP Fail Flag
* @param      None
* @return     None
* @details    This function is used to clear ISP fail flag when ISP fail flag set.
*/
#define DFMC_CLR_FAIL_FLAG()         (DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk)                       /*!< Clear ISP fail flag        \hideinitializer */

#define DFMC_ENABLE_DF_UPDATE()      (DFMC->ISPCTL |= DFMC_ISPCTL_DATAEN_Msk)
#define DFMC_ENABLE_DF_UPDATE()      (DFMC->ISPCTL |= DFMC_ISPCTL_DATAEN_Msk)
/*@}*/ /* end of group DFMC_EXPORTED_MACROS */

/*---------------------------------------------------------------------------------------------------------*/
/*  Global variables                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
extern int32_t  g_i32DFMC_ErrCode;

/** @addtogroup DFMC_EXPORTED_FUNCTIONS DFMC Exported Functions
    @{
*/

/**
  * @brief    Read company ID
  * @param    None
  * @return   The company ID (32-bit)
  * @details  The company ID of Nuvoton is fixed to be 0xDA
  */
__STATIC_INLINE uint32_t DFMC_ReadCID(void)
{
    DFMC->ISPCMD = DFMC_ISPCMD_READ_CID;           /* Set ISP Command Code */
    DFMC->ISPADDR = 0x0u;                          /* Must keep 0x0 when read CID */
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;          /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                             /* To make sure ISP/CPU be Synchronized */
    while(DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk) {} /* Waiting for ISP Done */

    return DFMC->ISPDAT;
}

  /**
    * @brief    Read product ID
    * @param    None
    * @return   The product ID (32-bit)
    * @details  This function is used to read product ID.
    */
__STATIC_INLINE uint32_t DFMC_ReadDID(void)
{
    DFMC->ISPCMD = DFMC_ISPCMD_READ_DID;          /* Set ISP Command Code */
    DFMC->ISPADDR = 0x04u;                        /* Must keep 0x4 when read PID */
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;         /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                          /* To make sure ISP/CPU be Synchronized */
    while(DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk) {} /* Waiting for ISP Done */

    return DFMC->ISPDAT;
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Functions                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void     DFMC_AllReadCommand(uint8_t u8Enable);
void     DFMC_Open(void);
void     DFMC_Close(void);
int32_t  DFMC_Erase(uint32_t u32PageAddr);
int32_t  DFMC_MassErase(uint32_t u32PageAddr);
uint32_t DFMC_Read(uint32_t u32Addr);
uint32_t DFMC_Read_P(uint32_t u32Addr);
int32_t  DFMC_Write(uint32_t u32Addr, uint32_t u32Data);
uint32_t DFMC_GetChkSum(uint32_t u32Addr, uint32_t u32Count);
uint32_t DFMC_CheckAllOne(uint32_t u32Addr, uint32_t u32Count);
uint32_t DFMC_Read_ECC(uint32_t u32Addr, uint32_t *pu32ECC);
int32_t  DFMC_Write_ECC(uint32_t u32Addr, uint32_t u32Data, uint32_t u32ECC);

/** @} end of group DFMC_EXPORTED_FUNCTIONS */
/** @} end of group DFMC_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif   // __DFMC_H__