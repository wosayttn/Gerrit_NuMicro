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
#define DFMC_ISPCMD_READ_ALL1    0x08UL         /*!< ISP Command: Read all-one result       \hideinitializer */
#define DFMC_ISPCMD_READ_CID     0x0BUL         /*!< ISP Command: Read Company ID           \hideinitializer */
#define DFMC_ISPCMD_READ_DID     0x0CUL         /*!< ISP Command: Read Device ID            \hideinitializer */
#define DFMC_ISPCMD_READ_CKS     0x0DUL         /*!< ISP Command: Read checksum             \hideinitializer */
#define DFMC_ISPCMD_PROGRAM      0x21UL         /*!< ISP Command: Write flash word          \hideinitializer */
#define DFMC_ISPCMD_PAGE_ERASE   0x22UL         /*!< ISP Command: Page Erase Flash          \hideinitializer */
#define DFMC_ISPCMD_MASS_ERASE   0x26UL         /*!< ISP Command: Erase whole Flash         \hideinitializer */
#define DFMC_ISPCMD_RUN_ALL1     0x28UL         /*!< ISP Command: Run all-one verification  \hideinitializer */
#define DFMC_ISPCMD_RUN_CKS      0x2DUL         /*!< ISP Command: Run checksum calculation  \hideinitializer */

#define DFMC_ISPCMD_EEPROM_INIT         0x33
#define DFMC_ISPCMD_EEPROM_READ         0x30
#define DFMC_ISPCMD_EEPROM_WRITE_BYTE   0x31
#define DFMC_ISPCMD_EEPROM_WRITE_WORD   0x32

#define DFMC_READ_ALLONE_YES            0xA11FFFFFUL    /*!< Check-all-one result is all one.       \hideinitializer */
#define DFMC_READ_ALLONE_NOT            0xA1100000UL    /*!< Check-all-one result is not all one.   \hideinitializer */
#define DFMC_READ_ALLONE_CMD_FAIL       0xAFFFFFFFUL    /*!< Check-all-one command failed.          \hideinitializer */
#define DREAD_ALLONE_YES                DFMC_READ_ALLONE_YES        /*!< Check-all-one result is all one.      \hideinitializer */
#define DREAD_ALLONE_NOT                DFMC_READ_ALLONE_NOT        /*!< Check-all-one result is not all one.  \hideinitializer */
#define DREAD_ALLONE_CMD_FAIL           DFMC_READ_ALLONE_CMD_FAIL   /*!< Check-all-one command failed.         \hideinitializer */

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
#define DFMC_ERR_NON_ALIGN       (-3L)                       /*!< DFMC operation abort due to non-aligned parameters   \hideinitializer */
#define DFMC_ERR_ECC             (-4L)                       /*!< DFMC read has ecc error                     \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* Define Base Address                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define DFMC_DATA_FLASH_BASE        0x0F210000UL                            /*!< Data Flash base address            \hideinitializer */
#define DFMC_DATA_FLASH_SIZE        0x10000UL                               /*!< Data Flash Size                    \hideinitializer */
#define DFMC_DATA_FLASH_END         (DFMC_DATA_FLASH_BASE + DFMC_DATA_FLASH_SIZE)  /*!< Data Flash end address      \hideinitializer */

#define DFMC_EEPROM_BASE            0x0F21E000UL
#define DFMC_EEPROM_SIZE            0x100UL
#define DFMC_EEPROM_END             (DFMC_EEPROM_BASE + DFMC_EEPROM_SIZE)

#define DFMC_FLASH_PAGE_SIZE        0x1000UL                                /*!< Flash Page Size (4K bytes)         \hideinitializer */
#define DFMC_FLASH_PAGE_ADDR_MASK   (DFMC_FLASH_PAGE_SIZE - 1)              /*!< Flash page address mask            \hideinitializer */

#define DFMC_DFLASH_BASE            DFMC_DATA_FLASH_BASE
#define DFMC_DFLASH_SIZE            DFMC_DATA_FLASH_SIZE
#define DFMC_DFLASH_END             DFMC_DATA_FLASH_END
#define DFMC_DFLASH_PAGE_SIZE       DFMC_FLASH_PAGE_SIZE

/*@}*/ /* end of group FMC_EXPORTED_CONSTANTS */

/** @addtogroup DFMC_EXPORTED_MACROS DFMC Exported Macros
    @{
*/
/*---------------------------------------------------------------------------------------------------------*/
/*  Macros                                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define DFMC_DISABLE_ISP()          (DFMC->ISPCTL &= ~DFMC_ISPCTL_ISPEN_Msk)                    /*!< Disable ISP function           \hideinitializer */
#define DFMC_ENABLE_ISP()           (DFMC->ISPCTL |=  DFMC_ISPCTL_ISPEN_Msk)                    /*!< Enable ISP function            \hideinitializer */
/**
* @brief      Get ISP Fail Flag
* @param      None
* @return     0: No ISP fail flag, 1: ISP fail flag is set
* @details    This function is used to check ISP fail flag after ISP command finished.
*/
#define DFMC_GET_FAIL_FLAG()        ((DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk) ? 1UL : 0UL)        /*!< Get ISP fail flag              \hideinitializer */

/**
* @brief      Clear ISP Fail Flag
* @param      None
* @return     None
* @details    This function is used to clear ISP fail flag when ISP fail flag set.
*/
#define DFMC_CLR_FAIL_FLAG()        (DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk)                     /*!< Clear ISP fail flag            \hideinitializer */

/**
* @brief      Enable Data Flash Update
* @param      None
* @return     None
*/
#define DFMC_ENABLE_UPDATE()        (DFMC->ISPCTL |= DFMC_ISPCTL_DATAEN_Msk)
/**
* @brief      Disable Data Flash Update
* @param      None
* @return     None
*/
#define DFMC_DISABLE_UPDATE()       (DFMC->ISPCTL &= ~DFMC_ISPCTL_DATAEN_Msk)

/**
* @brief      Enable DFMC ISP Interrupt
* @param      None
* @return     None
*/
#define DFMC_ENABLE_ISP_INT()       (DFMC->ISPCTL |=  DFMC_ISPCTL_ISPIFEN_Msk)                  /*!< Enable ISP interrupt           \hideinitializer */

/**
* @brief      Disable DFMC ISP Interrupt
* @param      None
* @return     None
*/
#define DFMC_DISABLE_ISP_INT()      (DFMC->ISPCTL &= ~DFMC_ISPCTL_ISPIFEN_Msk)                  /*!< Disable ISP interrupt          \hideinitializer */

/**
* @brief      Get ISP Interrupt Flag
* @param      None
* @return     0: No ISP interrupt, 1: ISP interrupt flag is raised
*/
#define DFMC_GET_ISP_INT_FLAG()     ((DFMC->ISPSTS & DFMC_ISPSTS_ISPIF_Msk) ? 1UL : 0UL)        /*!< Get ISP interrupt flag Status  \hideinitializer */

/**
* @brief      Clear ISP Interrupt Flag
* @param      None
* @return     None
*/
#define DFMC_CLEAR_ISP_INT_FLAG()   (DFMC->ISPSTS = DFMC_ISPSTS_ISPIF_Msk)                      /*!< Clear ISP interrupt flag       \hideinitializer */

/**
* @brief      Enable DFMC ECC Single Error Interrupt
* @param      None
* @return     None
*/
#define DFMC_ENABLE_SINGLE_ERR_INT()       (DFMC->ECCCTL |=  DFMC_ECCCTL_SEBDINTEN_Msk)                  /*!< Enable Single Error Interrupt           \hideinitializer */

/**
* @brief      Disable DFMC ECC Single Error Interrupt
* @param      None
* @return     None
*/
#define DFMC_DISABLE_SINGLE_ERR_INT()      (DFMC->ECCCTL &= ~DFMC_ECCCTL_SEBDINTEN_Msk)                  /*!< Disable Single Error Interrupt          \hideinitializer */

/**
* @brief      Get DFMC ECC Single Error Flag
* @param      None
* @return     0: No ECC single error, 1: ECC single error flag is raised
*/
#define DFMC_GET_SINGLE_ERR_FLAG()         ((DFMC->ECCSTS & DFMC_ECCSTS_ECCSEBCF_Msk) ? 1UL : 0UL)       /*!< Get ECC Single Error Flag              \hideinitializer */

/**
* @brief      Clear DFMC ECC Single Error Flag
* @param      None
* @return     None
*/
#define DFMC_CLR_SINGLE_ERR_FLAG()         (DFMC->ECCSTS = DFMC_ECCSTS_ECCSEBCF_Msk)                     /*!< Clear ECC Single Error Flag              \hideinitializer */

/**
* @brief      Get DFMC ECC Single Error Address
* @param      None
* @return     ECC single error address
*/
#define DFMC_GET_SINGLE_ERR_ADDR()         (DFMC->ECCSEFAR)       /*!< Get ECC Single Error Address              \hideinitializer */

/**
* @brief      Enable DFMC ECC Double Error Interrupt
* @param      None
* @return     None
*/
#define DFMC_ENABLE_DOUBLE_ERR_INT()       (DFMC->ECCCTL |=  DFMC_ECCCTL_DEBDINTEN_Msk)                  /*!< Enable Double Error Interrupt           \hideinitializer */

/**
* @brief      Disable DFMC ECC Double Error Interrupt
* @param      None
* @return     None
*/
#define DFMC_DISABLE_DOUBLE_ERR_INT()      (DFMC->ECCCTL &= ~DFMC_ECCCTL_DEBDINTEN_Msk)                  /*!< Disable Double Error Interrupt          \hideinitializer */

/**
* @brief      Get DFMC ECC Double Error Flag
* @param      None
* @return     0: No ECC double error, 1: ECC double error flag is raised
*/
#define DFMC_GET_DOUBLE_ERR_FLAG()         ((DFMC->ECCSTS & DFMC_ECCSTS_ECCDEBDF_Msk) ? 1UL : 0UL)       /*!< Get ECC Double Error Flag              \hideinitializer */

/**
* @brief      Clear DFMC ECC Double Error Flag
* @param      None
* @return     None
*/
#define DFMC_CLR_DOUBLE_ERR_FLAG()         (DFMC->ECCSTS = DFMC_ECCSTS_ECCDEBDF_Msk)                     /*!< Clear ECC Double Error Flag              \hideinitializer */

/**
* @brief      Get DFMC ECC Double Error Address
* @param      None
* @return     ECC double error address
*/
#define DFMC_GET_DOUBLE_ERR_ADDR()          (DFMC->ECCDEFAR)       /*!< Get ECC Double Error Address              \hideinitializer */

/**
* @brief      Get DFMC EEPROM Init Status
* @param      None
* @return     0: EEPROM is in the initial process, 1: EEPROM is not in the initial process
*/
#define DFMC_EEPROM_IS_INIT_DONE()          ((DFMC->EEP_STS & DFMC_EEP_STS_EEP_INIT_Msk) == 0)

/**
* @brief      Get DFMC EEPROM Page Full Status
* @param      None
* @return     0: EEPROM is not in the page full process, 1: EEPROM is in the page full process
*/
#define DFMC_EEPROM_IS_PAGE_FULL_PROC()     (DFMC->EEP_STS & DFMC_EEP_STS_EEP_PGFULL_Msk)

/**
* @brief      Get DFMC EEPROM Current Page Write Pointer
* @param      None
* @return     EEPROM current page write pointer
*/
#define DFMC_EEPROM_GET_WRITE_PTR()         ((DFMC->EEP_STS & DFMC_EEP_STS_EEPWPTR_Msk) >> DFMC_EEP_STS_EEPWPTR_Pos)

/**
* @brief      Get DFMC EEPROM Page Erase Times
* @param      None
* @return     EEPROM page erase times
*/
#define DFMC_EEPROM_GET_PAGE_ERASE_CNT()    ((DFMC->EEP_STS & DFMC_EEP_STS_PETIMES_Msk) >> DFMC_EEP_STS_PETIMES_Pos)

/*@}*/ /* end of group DFMC_EXPORTED_MACROS */

/*---------------------------------------------------------------------------------------------------------*/
/*  Global variables                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
extern int32_t  g_DFMC_i32ErrCode;

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
__STATIC_INLINE uint32_t DFMC_ReadPID(void)
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
void     DFMC_Open(void);
void     DFMC_Close(void);
int32_t  DFMC_Erase(uint32_t u32PageAddr);
void     DFMC_Erase_NonBlocking(uint32_t u32PageAddr);

int32_t  DFMC_MassErase(uint32_t u32PageAddr);
void     DFMC_Mass_Erase_NonBlocking(void);
uint32_t DFMC_Read(uint32_t u32Addr);
uint32_t DFMC_Read_ECC(uint32_t u32Addr, int32_t *pi32ErrBit);

int32_t  DFMC_Write(uint32_t u32Addr, uint32_t u32Data);
void     DFMC_Write_NonBlocking(uint32_t u32Addr, uint32_t u32Data);

uint32_t DFMC_GetChkSum(uint32_t u32Addr, uint32_t u32Count);
uint32_t DFMC_CheckAllOne(uint32_t u32Addr, uint32_t u32Count);

int32_t  DFMC_EEPROM_Init(void);
uint32_t DFMC_EEPROM_Read(uint32_t u32Addr);
int32_t  DFMC_EEPROM_Write(uint32_t u32Addr, uint32_t u32Data);
int32_t  DFMC_EEPROM_WriteByte(uint32_t u32Addr, uint8_t u8Data);

/** @} end of group DFMC_EXPORTED_FUNCTIONS */
/** @} end of group DFMC_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif   // __DFMC_H__