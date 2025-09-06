/******************************************************************************
 * @file     dfmc.c
 * @version  V1.00
 * @brief    DFMC driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
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

#define DBG_PRINTF  printf
int32_t g_i32DFMC_ErrCode = DFMC_OK;    /*!< DFMC global error code */

static uint32_t s_bDFMC_TestAllReadCmd = FALSE;
static uint32_t s_bDoISPFFTest = FALSE;

void DFMC_AllReadCommand(uint8_t u8Enable)
{
    s_bDFMC_TestAllReadCmd = TRUE;
}

void DFMC_ISPFF_Test(uint32_t bEnable)
{
	s_bDoISPFFTest = bEnable;
}

/**
  * @brief    Enable DFMC ISP function
  * @param    None
  * @return   None
  * @details  ISPEN bit of DFMC_ISPCTL must be set before we can use ISP commands.
  *           Therefore, To use all DFMC function APIs, user needs to call DFMC_Open() first to enable ISP functions.
  *
  * @note     ISP functions are write-protected. user also needs to unlock it by calling SYS_UnlockReg() before using all ISP functions.
  */
void DFMC_Open(void)
{
    DFMC->CYCCTL = FMC->CYCCTL;
    DFMC->ISPCTL |=  DFMC_ISPCTL_ISPEN_Msk;
}

  /**
    * @brief    Disable DFMC ISP Functions
    * @param    None
    * @return   None
    * @details  This function will clear ISPEN bit of DFMC_ISPCTL to disable ISP function.
    */
void DFMC_Close(void)
{
    DFMC->ISPCTL &= ~DFMC_ISPCTL_ISPEN_Msk;
}

/**
  * @brief Execute DFMC_ISPCMD_PAGE_ERASE command to erase a flash page. The page size is 4096 bytes.
  * @param[in]  u32PageAddr Address of the flash page to be erased.
  *             It must be a 4096 bytes aligned address.
  * @return ISP page erase success or not.
  * @retval   0  Success
  * @retval   -1  Erase failed
  *
  * @note     Global error code g_i32DFMC_ErrCode
  *           -1  Erase failed
  *           -2  Erase time-out
  */
int32_t DFMC_Erase(uint32_t u32PageAddr)
{
    int32_t i32Ret = 0;
    int32_t i32TimeOutCnt;

    g_i32DFMC_ErrCode = 0;

    DFMC->ISPCMD = DFMC_ISPCMD_PAGE_ERASE;
    DFMC->ISPADDR = u32PageAddr;
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = DFMC_TIMEOUT_ERASE;
    while(DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk)
    {
        if(i32TimeOutCnt-- <= 0)
        {
            printf("timeout\n");
            g_i32DFMC_ErrCode = -2;
            i32Ret = -2;
            break;
        }
    }

    if(DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk)
    {
        //printf("ispff, DFMC->ISPCTL: 0x%08X\n", DFMC->ISPCTL);
        DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk;
        g_i32DFMC_ErrCode = -1;
        i32Ret = -1;
    }

    return i32Ret;
}

int32_t DFMC_MassErase(uint32_t u32PageAddr)
{
    int32_t i32Ret = 0;
    int32_t i32TimeOutCnt;

    DFMC->ISPCMD  = DFMC_ISPCMD_MASS_ERASE;
    DFMC->ISPADDR = u32PageAddr;
    DFMC->ISPTRG  = DFMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = DFMC_TIMEOUT_ERASE;
    while(DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk)
    {
        if(i32TimeOutCnt-- <= 0)
        {
            printf("timeout\n");
            g_i32DFMC_ErrCode = -1;
            i32Ret = -1;
            break;
        }
    }

    if(DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk)
    {
        //printf("ispff, DFMC->ISPCTL: 0x%08X\n", DFMC->ISPCTL);
        DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk;
        g_i32DFMC_ErrCode = -1;
        i32Ret = -1;
    }

    return i32Ret;
}

/**
  * @brief Execute DFMC_ISPCMD_READ command to read a word from flash.
  * @param[in]  u32Addr Address of the flash location to be read.
  *             It must be a word aligned address.
  * @return The word data read from specified flash address.
  *         Return 0xFFFFFFFF if read failed.
  * @note   Global error code g_i32DFMC_ErrCode
  *         -1  Read time-out
  */
uint32_t DFMC_Read(uint32_t u32Addr)
{
    int32_t i32TimeOutCnt = DFMC_TIMEOUT_READ;

    g_i32DFMC_ErrCode = DFMC_OK;

    if (s_bDFMC_TestAllReadCmd)
    {
        uint32_t  n_read, e_read, p_read;

        DFMC->ISPCMD = DFMC_ISPCMD_READ;
        DFMC->ISPADDR = u32Addr;
        DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;
        while (DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk) { }
        n_read = DFMC->ISPDAT;

        DFMC->ISPCMD = DFMC_ISPCMD_READ_E;
        DFMC->ISPADDR = u32Addr;
        DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;
        while (DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk) { }
        e_read = DFMC->ISPDAT;


        DFMC->ISPCMD = DFMC_ISPCMD_READ_P;
        DFMC->ISPADDR = u32Addr;
        DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;
        while (DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk) { }
        p_read = DFMC->ISPDAT;
        if ((n_read != e_read) || (n_read != p_read))
        {
            printf("Read command mismatch!!\n");
            getchar();
        }
    }
    else
    {
        DFMC->ISPCMD  = DFMC_ISPCMD_READ;
        DFMC->ISPADDR = u32Addr;
        DFMC->ISPTRG  = DFMC_ISPTRG_ISPGO_Msk;
        
        while (DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk)
        {
            if( i32TimeOutCnt-- <= 0)
            {
                g_i32DFMC_ErrCode = -1;
                break;
            }
        }
        
        if(s_bDoISPFFTest)
        {
            if (DFMC->ISPSTS & DFMC_ISPSTS_ISPFF_Msk)
            {
                DBG_PRINTF("DFMC read 0x%08X ISPFF\n", u32Addr);
                DFMC->ISPSTS = DFMC_ISPSTS_ISPFF_Msk;
                g_i32DFMC_ErrCode = -2;
            }
        }
    }
    return DFMC->ISPDAT;
}

/**
  * @brief Execute DFMC_ISPCMD_READ_ECC command to read a word from flash.
  * @param[in]  u32Addr Address of the flash location to be read.
  *             It must be a word aligned address.
  * @return The word data read from specified flash address.
  *         Return 0xFFFFFFFF if read failed.
  * @note   Global error code g_i32DFMC_ErrCode
  *         -1  Read time-out
  */
uint32_t DFMC_Read_ECC(uint32_t u32Addr, uint32_t *pu32ECC)
{
    int32_t i32TimeOutCnt;

    g_i32DFMC_ErrCode = DFMC_OK;
    *pu32ECC = 0;

    DFMC->ISPCMD  = DFMC_ISPCMD_READ;
    DFMC->ISPADDR = u32Addr;
    DFMC->ISPTRG  = DFMC_ISPTRG_ISPGO_Msk;
    while (DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk) { }
    
    i32TimeOutCnt = DFMC_TIMEOUT_READ;
    while (DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk)
    {
        if( i32TimeOutCnt-- <= 0)
        {
            g_i32DFMC_ErrCode = -1;
            break;
        }
    }
    
    if(s_bDoISPFFTest)
    {
        if (DFMC->ISPSTS & DFMC_ISPSTS_ISPFF_Msk)
        {
            DBG_PRINTF("DFMC read 0x%08X ISPFF\n", u32Addr);
            DFMC->ISPSTS = DFMC_ISPSTS_ISPFF_Msk;
            g_i32DFMC_ErrCode = -2;
        }
    }
    
    if (g_i32DFMC_ErrCode != DFMC_OK)
        return g_i32DFMC_ErrCode;
    else
    {
        *pu32ECC = DFMC->MPDAT2;
    
        return DFMC->ISPDAT;
    }
}

  /**
    * @brief Execute DFMC_ISPCMD_READ_P command to read a word from flash.
    * @param[in]  u32Addr Address of the flash location to be read.
    *             It must be a word aligned address.
    * @return The word data read from specified flash address.
    */
uint32_t DFMC_Read_P(uint32_t u32Addr)
{
    DFMC->ISPCMD = DFMC_ISPCMD_READ_P;
    DFMC->ISPADDR = u32Addr;
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;
    while (DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk) { }

    return DFMC->ISPDAT;
}

/**
  * @brief Execute ISP DFMC_ISPCMD_PROGRAM to program a word to flash.
  * @param[in]  u32Addr Address of the flash location to be programmed.
  *             It must be a word aligned address.
  * @param[in]  u32Data The word data to be programmed.
  * @return   DFMC_OK   Success
  * @return   -1  Failed
  *
  * @note     Global error code g_i32DFMC_ErrCode
  *           -1  Program failed or time-out
  */
int32_t DFMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    int32_t i32TimeOutCnt;

    g_i32DFMC_ErrCode = DFMC_OK;
    DFMC->ISPCMD = DFMC_ISPCMD_PROGRAM;
    DFMC->ISPADDR = u32Addr;
    DFMC->ISPDAT = u32Data;
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = DFMC_TIMEOUT_WRITE;
    while(DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk)
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_i32DFMC_ErrCode = -1;
            return g_i32DFMC_ErrCode;
        }
    }
    
    if(s_bDoISPFFTest)
    {
        if (DFMC->ISPSTS & DFMC_ISPSTS_ISPFF_Msk)
        {
            DBG_PRINTF("DFMC write 0x%08X ISPFF\n", u32Addr);
            DFMC->ISPSTS = DFMC_ISPSTS_ISPFF_Msk;
            g_i32DFMC_ErrCode = -2;
            return g_i32DFMC_ErrCode;
        }
    }

    return DFMC_OK;
}

/**
  * @brief Execute ISP DFMC_ISPCMD_PROGRAM to program a word to flash.
  * @param[in]  u32Addr Address of the flash location to be programmed.
  *             It must be a word aligned address.
  * @param[in]  u32Data The word data to be programmed.
  * @return   DFMC_OK   Success
  * @return   -1  Failed
  *
  * @note     Global error code g_i32DFMC_ErrCode
  *           -1  Program failed or time-out
  */
int32_t DFMC_Write_ECC(uint32_t u32Addr, uint32_t u32Data, uint32_t u32ECC)
{
    int32_t i32TimeOutCnt;

    g_i32DFMC_ErrCode = DFMC_OK;
    DFMC->ISPCMD  = DFMC_ISPCMD_PROGRAM_ECC;
    DFMC->ISPADDR = u32Addr;
    DFMC->ISPDAT  = u32Data;
    DFMC->MPDAT2  = u32ECC;
    DFMC->ISPTRG  = DFMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = DFMC_TIMEOUT_WRITE;
    while(DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk)
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_i32DFMC_ErrCode = -1;
            return g_i32DFMC_ErrCode;
        }
    }
    
    if(s_bDoISPFFTest)
    {
        if (DFMC->ISPSTS & DFMC_ISPSTS_ISPFF_Msk)
        {
            DBG_PRINTF("DFMC write 0x%08X ISPFF\n", u32Addr);
            DFMC->ISPSTS = DFMC_ISPSTS_ISPFF_Msk;
            g_i32DFMC_ErrCode = -2;
            return g_i32DFMC_ErrCode;
        }
    }

    return DFMC_OK;
}

/**
  * @brief Run CRC32 checksum calculation and get result.
  * @param[in] u32Addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32Count  Byte count of flash to be calculated. It must be multiple of page size.
  * @retval   0           Success.
  * @retval   0xFFFFFFFF  Invalid parameter or command failed.
  *
  * @note     Global error code g_i32DFMC_ErrCode
  *           -1  Run/Read check sum time-out failed
  *           -2  u32Addr or u32Count must be aligned with page size
  */
uint32_t  DFMC_GetChkSum(uint32_t u32Addr, uint32_t u32Count)
{
    uint32_t u32Ret;
    int32_t  i32TimeOutCnt;

    g_i32DFMC_ErrCode = DFMC_OK;

    if((u32Addr % DFMC_FLASH_PAGE_SIZE) || (u32Count % DFMC_FLASH_PAGE_SIZE))
    {
        g_i32DFMC_ErrCode = -2;
        u32Ret = 0xFFFFFFFF;
    }
    else
    {
        DFMC->ISPCMD  = DFMC_ISPCMD_RUN_CKS;
        DFMC->ISPADDR = u32Addr;
        DFMC->ISPDAT  = u32Count;
        DFMC->ISPTRG  = DFMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = DFMC_TIMEOUT_CHKSUM;
        while(DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk)
        {
            if(i32TimeOutCnt-- <= 0)
            {
                g_i32DFMC_ErrCode = -1;
                return 0xFFFFFFFF;
            }
        }

        DFMC->ISPCMD = DFMC_ISPCMD_READ_CKS;
        DFMC->ISPADDR = u32Addr;
        DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = DFMC_TIMEOUT_CHKSUM;
        while(DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk)
        {
            if(i32TimeOutCnt-- <= 0)
            {
                g_i32DFMC_ErrCode = -1;
                return 0xFFFFFFFF;
            }
        }

        u32Ret = DFMC->ISPDAT;
    }

    return u32Ret;
}

/**
  * @brief Run flash all one verification and get result.
  * @param[in] u32Addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32Count  Byte count of flash to be calculated. It must be multiple of page size.
  * @retval   READ_ALLONE_YES       The contents of verified flash area are 0xFFFFFFFF.
  * @retval   READ_ALLONE_NOT       Some contents of verified flash area are not 0xFFFFFFFF.
  * @retval   READ_ALLONE_CMD_FAIL  Unexpected error occurred.
  *
  * @note     Global error code g_i32DFMC_ErrCode
  *           -1  RUN_ALL_ONE or CHECK_ALL_ONE commands time-out
  */
uint32_t  DFMC_CheckAllOne(uint32_t u32Addr, uint32_t u32Count)
{
    uint32_t u32Ret = READ_ALLONE_CMD_FAIL;
    int32_t  i32TimeOutCnt0, i32TimeOutCnt1;

    g_i32DFMC_ErrCode = DFMC_OK;

    DFMC->ISPSTS = DFMC_ISPSTS_ALLONE_Msk;   /* clear check all one bit */

    DFMC->ISPCMD   = DFMC_ISPCMD_RUN_ALL1;
    DFMC->ISPADDR  = u32Addr;
    DFMC->ISPDAT   = u32Count;
    DFMC->ISPTRG   = DFMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt0 = DFMC_TIMEOUT_CHKALLONE;
    while(DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk)
    {
        if(i32TimeOutCnt0-- <= 0)
        {
            g_i32DFMC_ErrCode = -1;
            break;
        }
    }

    if(g_i32DFMC_ErrCode == DFMC_OK)
    {
        i32TimeOutCnt1 = DFMC_TIMEOUT_CHKALLONE;
        do
        {
            DFMC->ISPCMD = DFMC_ISPCMD_READ_ALL1;
            DFMC->ISPADDR = u32Addr;
            DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;

            i32TimeOutCnt0 = DFMC_TIMEOUT_CHKALLONE;
            while(DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk)
            {
                if(i32TimeOutCnt0-- <= 0)
                {
                    g_i32DFMC_ErrCode = -1;
                    break;
                }
            }

            if(i32TimeOutCnt1-- <= 0)
            {
                g_i32DFMC_ErrCode = -1;
            }
        }
        while((DFMC->ISPDAT == 0UL) && (g_i32DFMC_ErrCode == 0));
    }

    if(g_i32DFMC_ErrCode == DFMC_OK)
    {
        if(DFMC->ISPDAT == READ_ALLONE_YES)
            u32Ret = READ_ALLONE_YES;
        else if(DFMC->ISPDAT == READ_ALLONE_NOT)
            u32Ret = READ_ALLONE_NOT;
        else
            g_i32DFMC_ErrCode = -1;
    }

    return u32Ret;
}