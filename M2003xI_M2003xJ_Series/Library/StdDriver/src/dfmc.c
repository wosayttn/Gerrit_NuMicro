/**************************************************************************//**
 * @file     dfmc.c
 * @version  V1.00
 * @brief    M2003J series DFMC driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>

#include "NuMicro.h"


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup DFMC_Driver DFMC Driver
  @{
*/


/** @addtogroup DFMC_EXPORTED_FUNCTIONS DFMC Exported Functions
  @{
*/

int32_t g_DFMC_i32ErrCode = 0; /*!< FMC global error code */


/**
  * @brief    Enable DFMC ISP function
  *
  * @param    None
  *
  * @return   None
  *
  * @details  ISPEN bit of DFMC_ISPCTL must be set before we can use ISP commands.
  *           Therefore, To use all DFMC function APIs, user needs to call DFMC_Open() first to enable ISP functions.
  *
  * @note     ISP functions are write-protected. user also needs to unlock it by calling SYS_UnlockReg() before using all ISP functions.
  *
  */
void DFMC_Open(void)
{
    DFMC->ISPCTL |=  DFMC_ISPCTL_ISPEN_Msk;
}



/**
  * @brief    Disable ISP Functions
  *
  * @param    None
  *
  * @return   None
  *
  * @details  This function will clear ISPEN bit of DFMC_ISPCTL to disable ISP function.
  *
  */
void DFMC_Close(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;
}


/**
  * @brief Execute DFMC_ISPCMD_PAGE_ERASE command to erase a flash page. The page size is 512 bytes.
  * @param[in]  u32PageAddr Address of the flash page to be erased.
  *             It must be a 512 bytes aligned address.
  * @return ISP page erase success or not.
  * @retval   0  Success
  * @retval   -1  Erase failed
  *
  * @note     Global error code g_DFMC_i32ErrCode
  *           -1  Erase failed or erase time-out
  */
int32_t DFMC_Erase(uint32_t u32PageAddr)
{
    int32_t  ret = 0;
    int32_t i32TimeOutCnt;

    g_DFMC_i32ErrCode = 0;

    DFMC->ISPCMD = DFMC_ISPCMD_PAGE_ERASE;
    DFMC->ISPADDR = u32PageAddr;
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = DFMC_TIMEOUT_ERASE;
    while(DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk)
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_DFMC_i32ErrCode = -1;
            ret = -1;
            break;
        }
    }

    if(DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk)
    {
        DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk;
        g_DFMC_i32ErrCode = -1;
        ret = -1;
    }

    return ret;
}



/**
  * @brief Execute DFMC_ISPCMD_READ command to read a word from flash.
  * @param[in]  u32Addr Address of the flash location to be read.
  *             It must be a word aligned address.
  * @return The word data read from specified flash address.
  *         Return 0xFFFFFFFF if read failed.
  * @note   Global error code g_DFMC_i32ErrCode
  *         -1  Read time-out
  */
uint32_t DFMC_Read(uint32_t u32Addr)
{
    int32_t i32TimeOutCnt;

    g_DFMC_i32ErrCode = 0;
    DFMC->ISPCMD = DFMC_ISPCMD_READ;
    DFMC->ISPADDR = u32Addr;
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = DFMC_TIMEOUT_READ;
    while(DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk)
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_DFMC_i32ErrCode = -1;
            return 0xFFFFFFFF;
        }
    }

    return DFMC->ISPDAT;
}

/**
  * @brief Execute ISP DFMC_ISPCMD_PROGRAM to program a word to flash.
  * @param[in]  u32Addr Address of the flash location to be programmed.
  *             It must be a word aligned address.
  * @param[in]  u32Data The word data to be programmed.
  * @return   0   Success
  * @return   -1  Failed
  *
  * @note     Global error code g_DFMC_i32ErrCode
  *           -1  Program failed or time-out
  */
int32_t DFMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    int32_t i32TimeOutCnt;

    g_DFMC_i32ErrCode = 0;
    DFMC->ISPCMD = DFMC_ISPCMD_PROGRAM;
    DFMC->ISPADDR = u32Addr;
    DFMC->ISPDAT = u32Data;
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = DFMC_TIMEOUT_WRITE;
    while(DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk)
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_DFMC_i32ErrCode = -1;
            return -1;
        }
    }

    return 0;
}

/**
  * @brief Run CRC32 checksum calculation and get result.
  * @param[in] u32addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of page size.
  * @retval   0           Success.
  * @retval   0xFFFFFFFF  Invalid parameter or command failed.
  *
  * @note     Global error code g_DFMC_i32ErrCode
  *           -1  Run/Read check sum time-out failed
  *           -2  u32addr or u32count must be aligned with page size
  */
uint32_t  DFMC_GetChkSum(uint32_t u32addr, uint32_t u32count)
{
    uint32_t   ret;
    int32_t i32TimeOutCnt;

    g_DFMC_i32ErrCode = 0;

    if((u32addr % DFMC_FLASH_PAGE_SIZE) || (u32count % DFMC_FLASH_PAGE_SIZE))
    {
        g_DFMC_i32ErrCode = -2;
        ret = 0xFFFFFFFF;
    }
    else
    {
        DFMC->ISPCMD  = DFMC_ISPCMD_RUN_CKS;
        DFMC->ISPADDR = u32addr;
        DFMC->ISPDAT  = u32count;
        DFMC->ISPTRG  = DFMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = DFMC_TIMEOUT_CHKSUM;
        while(DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk)
        {
            if(i32TimeOutCnt-- <= 0)
            {
                g_DFMC_i32ErrCode = -1;
                return 0xFFFFFFFF;
            }
        }

        DFMC->ISPCMD = DFMC_ISPCMD_READ_CKS;
        DFMC->ISPADDR = u32addr;
        DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = DFMC_TIMEOUT_CHKSUM;
        while(DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk)
        {
            if(i32TimeOutCnt-- <= 0)
            {
                g_DFMC_i32ErrCode = -1;
                return 0xFFFFFFFF;
            }
        }

        ret = DFMC->ISPDAT;
    }

    return ret;
}


/**
  * @brief Run flash all one verification and get result.
  * @param[in] u32addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of page size.
  * @retval   DREAD_ALLONE_YES       The contents of verified flash area are 0xFFFFFFFF.
  * @retval   DREAD_ALLONE_NOT       Some contents of verified flash area are not 0xFFFFFFFF.
  * @retval   DREAD_ALLONE_CMD_FAIL  Unexpected error occurred.
  *
  * @note     Global error code g_DFMC_i32ErrCode
  *           -1  RUN_ALL_ONE or CHECK_ALL_ONE commands time-out
  */
uint32_t  DFMC_CheckAllOne(uint32_t u32addr, uint32_t u32count)
{
    uint32_t  ret = READ_ALLONE_CMD_FAIL;
    int32_t i32TimeOutCnt0, i32TimeOutCnt1;

    g_DFMC_i32ErrCode = 0;

    DFMC->ISPSTS = DFMC_ISPSTS_ALLONE_Msk;   /* clear check all one bit */

    DFMC->ISPCMD   = DFMC_ISPCMD_RUN_ALL1;
    DFMC->ISPADDR  = u32addr;
    DFMC->ISPDAT   = u32count;
    DFMC->ISPTRG   = DFMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt0 = DFMC_TIMEOUT_CHKALLONE;
    while(DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk)
    {
        if(i32TimeOutCnt0-- <= 0)
        {
            g_DFMC_i32ErrCode = -1;
            break;
        }
    }

    if(g_DFMC_i32ErrCode == 0)
    {
        i32TimeOutCnt1 = DFMC_TIMEOUT_CHKALLONE;
        do
        {
            DFMC->ISPCMD = DFMC_ISPCMD_READ_ALL1;
            DFMC->ISPADDR = u32addr;
            DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;

            i32TimeOutCnt0 = DFMC_TIMEOUT_CHKALLONE;
            while(DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk)
            {
                if(i32TimeOutCnt0-- <= 0)
                {
                    g_DFMC_i32ErrCode = -1;
                    break;
                }
            }

            if(i32TimeOutCnt1-- <= 0)
            {
                g_DFMC_i32ErrCode = -1;
            }
        }
        while((DFMC->ISPDAT == 0UL) && (g_DFMC_i32ErrCode == 0));
    }

    if(g_DFMC_i32ErrCode == 0)
    {
        if(DFMC->ISPDAT == DREAD_ALLONE_YES)
            ret = DREAD_ALLONE_YES;
        else if(DFMC->ISPDAT == DREAD_ALLONE_NOT)
            ret = DREAD_ALLONE_NOT;
        else
            g_DFMC_i32ErrCode = -1;
    }

    return ret;
}



/*@}*/ /* end of group DFMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group DFMC_Driver */

/*@}*/ /* end of group Standard_Driver */



