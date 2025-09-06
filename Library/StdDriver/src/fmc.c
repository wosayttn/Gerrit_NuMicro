/******************************************************************************
 * @file     fmc.c
 * @version  V1.00
 * @brief    FMC driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"


/** @addtogroup Standard_Driver Standard Driver
    @{
*/

/** @addtogroup FMC_Driver FMC Driver
    @{
*/

/** @addtogroup FMC_EXPORTED_FUNCTIONS FMC Exported Functions
    @{
*/

int32_t g_FMC_i32ErrCode = FMC_OK; /*!< FMC global error code */
static uint32_t  g_bDoISPFFTest = FALSE;

void FMC_ISPFF_Test(uint32_t bTRUE)
{
	g_bDoISPFFTest = bTRUE;
}

/**
  * @brief Get the current boot source.
  * @return The current boot source.
  * @retval   0  Is boot from APROM.
  * @retval   1  Is boot from LDROM.
  * @retval   2  Is boot from Boot Loader.
  */
int32_t FMC_GetBootSource (void)
{
    if (FMC->ISPCTL & FMC_ISPCTL_BL_Msk)
    {
        return 2;
    }
    if (FMC->ISPCTL & FMC_ISPCTL_BS_Msk)
    {
        return 1;
    }
    return 0;
}

/**
  * @brief    Enable FMC ISP function
  * @param    None
  * @return   None
  * @details  ISPEN bit of FMC_ISPCTL must be set before we can use ISP commands.
  *           Therefore, To use all FMC function APIs, user needs to call FMC_Open() first to enable ISP functions.
  *
  * @note     ISP functions are write-protected. user also needs to unlock it by calling SYS_UnlockReg() before using all ISP functions.
  */
void FMC_Open(void)
{
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;
}

/**
  * @brief    Disable FMC ISP Functions
  * @param    None
  * @return   None
  * @details  This function will clear ISPEN bit of FMC_ISPCTL to disable ISP function.
  */
void FMC_Close(void)
{
    FMC->ISPCTL &= ~FMC_ISPCTL_ISPEN_Msk;
}

/**
  * @brief Execute FMC_ISPCMD_PAGE_ERASE command to erase a flash page. The page size is 8192 bytes.
  * @param[in]  u32PageAddr Address of the flash page to be erased.
  *             It must be a 8192 bytes aligned address.
  * @return ISP page erase success or not.
  * @retval   0  Success
  * @retval   -1  Erase failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Erase failed or erase time-out
  */
int32_t FMC_Erase(uint32_t u32PageAddr)
{
    int32_t i32Ret = 0;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = u32PageAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_ERASE;
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = -1;
            i32Ret = -1;
            break;
        }
    }

    if(FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        i32Ret = -1;
    }

    return i32Ret;
}


/**
  * @brief Execute FMC_ISPCMD_BANK_ERASE command to erase a flash bank.
  * @param[in]  u32BankAddr Base address of the flash bank to be erased.
  * @return ISP page erase success or not.
  * @retval   0  Success
  * @retval   -1  Erase failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Erase failed or erase time-out
  */
int32_t FMC_Erase_Bank(uint32_t u32BankAddr)
{
    int32_t i32Ret = 0;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = 0;

    FMC->ISPCMD = FMC_ISPCMD_BANK_ERASE;
    FMC->ISPADDR = u32BankAddr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_ERASE;
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = -1;
            i32Ret = -1;
            break;
        }
    }

    if(FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        i32Ret = -1;
    }
    return i32Ret;
}

/**
  * @brief  Execute Erase XOM Region
  *
  * @param[in]  u32XomNum  The XOMRn(n=0~3)
  *
  * @return   XOM erase success or not.
  * @retval    0  Success
  * @retval   -1  Erase failed
  * @retval   -2  Invalid XOM number.
  *
  * @details Execute FMC_ISPCMD_PAGE_ERASE command to erase XOM.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Program failed or program time-out
  *           -2  Invalid XOM number.
  */
int32_t FMC_EraseXOM(uint32_t u32XomNum)
{
    uint32_t u32Addr;
    int32_t i32Active, i32Ret = FMC_OK;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;

    if(u32XomNum >= XOMR_CNT)
    {
        i32Ret = -2;
    }
    else
    {
        i32Active = FMC_GetXOMState(u32XomNum);

        if(i32Active)
        {
            switch(u32XomNum)
            {
                case 0u:
                    u32Addr = (FMC->XOMR0STS & 0xFFFFFF00u) >> 8u;
                    break;
                case 1u:
                    u32Addr = (FMC->XOMR1STS & 0xFFFFFF00u) >> 8u;
                    break;
                default:
                    /* Should not be here */
                    i32Ret = -2;
                    goto lexit;
            }
            FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
            FMC->ISPADDR = u32Addr;
            FMC->ISPDAT = 0x55aa03u;
            FMC->ISPTRG = 0x1u;
#if ISBEN
            __ISB();
#endif
            i32TimeOutCnt = FMC_TIMEOUT_ERASE;
            while(FMC->ISPTRG)
            {
                if(i32TimeOutCnt-- <= 0)
                {
                    i32Ret = -1;
                    break;
                }
            }

            /* Check ISPFF flag to know whether erase OK or fail. */
            if(FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
            {
                FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
                i32Ret = -1;
            }
        }
        else
        {
            i32Ret = -1;
        }
    }

lexit:
    g_FMC_i32ErrCode = i32Ret;
    return i32Ret;
}

/**
  * @brief  Check the XOM is actived or not.
  *
  * @param[in] u32XomNum    The xom number(0~3).
  *
  * @retval   1   XOM is actived.
  * @retval   0   XOM is not actived.
  * @retval   -2  Invalid XOM number.
  *
  * @details To get specify XOMRn(n=0~3) active status
  */
int32_t FMC_GetXOMState(uint32_t u32XomNum)
{
    int32_t  i32Ret = 0;

    if(u32XomNum >= XOMR_CNT)
    {
        i32Ret = -2;
    }
    else
    {
        i32Ret = (int32_t)(((FMC->XOMSTS & 0xFUL) & (1UL << u32XomNum)) >> u32XomNum);
    }

    return i32Ret;
}

/**
  * @brief     Config XOM Region
  * @param[in] u32XomNum    The XOM number (0~1)
  * @param[in] u32XomBase   The XOM region base address.
  * @param[in] u8XomPage    The XOM page number of region size.
  *
  * @retval   0   Success
  * @retval   1   XOM is has already actived.
  * @retval   -1  Program failed.
  * @retval   -2  Invalid XOM number.
  *
  * @details  Program XOM base address and XOM size(page).
  *           The CONFIG update enable bit CFGUEN(FMC_ISPCTL[4]) should be enabled before using this function.
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Program failed or program time-out
  *           -2  Invalid XOM number.
  */
int32_t FMC_ConfigXOM(uint32_t u32XomNum, uint32_t u32XomBase, uint8_t u8XomPage)
{
    int32_t i32Ret = 0;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = 0;

    if(u32XomNum >= XOMR_CNT)
    {
        g_FMC_i32ErrCode = -2;
        i32Ret = -2;
        return i32Ret;
    }


    if (FMC_GetXOMState(u32XomNum) == XOMR_ACTIVE)
    {
        return XOMR_ACTIVE;
    }

    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = FMC_XOM_BASE + (u32XomNum * 0x10u);
    FMC->ISPDAT = u32XomBase;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    i32TimeOutCnt = FMC_TIMEOUT_WRITE;
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = -1;
            i32Ret = -1;
            break;
        }
    }

    if(FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        i32Ret = -1;
    }

    if(i32Ret == 0)
    {
        FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = FMC_XOM_BASE + (u32XomNum * 0x10u + 0x04u);
        FMC->ISPDAT = u8XomPage;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        i32TimeOutCnt = FMC_TIMEOUT_WRITE;
        while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
        {
            if(i32TimeOutCnt-- <= 0)
            {
                g_FMC_i32ErrCode = -1;
                i32Ret = -1;
                break;
            }
        }

        if(FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
            g_FMC_i32ErrCode = -1;
            i32Ret = -1;
        }
    }

    if(i32Ret == 0)
    {
        FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
        FMC->ISPADDR = FMC_XOM_BASE + (u32XomNum * 0x10u + 0x08u);
        FMC->ISPDAT = 0u;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        i32TimeOutCnt = FMC_TIMEOUT_WRITE;
        while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
        {
            if(i32TimeOutCnt-- <= 0)
            {
                g_FMC_i32ErrCode = -1;
                i32Ret = -1;
                break;
            }
        }

        if(FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
            g_FMC_i32ErrCode = -1;
            i32Ret = -1;
        }
    }

    return i32Ret;
}

/**
  * @brief Execute FMC_ISPCMD_READ command to read a word from flash.
  * @param[in]  u32Addr Address of the flash location to be read.
  *             It must be a word aligned address.
  * @return The word data read from specified flash address.
  *         Return 0xFFFFFFFF if read failed.
  * @note   Global error code g_FMC_i32ErrCode
  *         -1  Read time-out
  */
uint32_t FMC_Read(uint32_t u32Addr)
{
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = u32Addr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_READ;
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = -1;
            return 0xFFFFFFFF;
        }
    }
    
    if(g_bDoISPFFTest)
    {
        if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            //printf("Read 0x%08X ISPFF\n", u32Addr);
            FMC->ISPSTS = FMC_ISPSTS_ISPFF_Msk;
            g_FMC_i32ErrCode = -2;
            return 0xFFFFFFFF;
        }
    }

    return FMC->ISPDAT;
}


/**
  * @brief Execute FMC_ISPCMD_READ_64 command to read a double-word from flash.
  * @param[in]  u32Addr   Address of the flash location to be read.
  *             It must be a double-word aligned address.
  * @param[out] pu32Data0  Place holder of word read from flash address u32Addr.
  * @param[out] pu32Data1  Place holder of word read from flash address u32Addr+4.
  * @return   FMC_OK   Success
  * @return   -1  Failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  */
int32_t FMC_Read_64(uint32_t u32Addr, uint32_t * pu32Data0, uint32_t * pu32Data1)
{
    int32_t  i32Ret = FMC_OK;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;
    FMC->ISPCMD = FMC_ISPCMD_READ_64;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = 0x0UL;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_READ;
    while(FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = -1;
            i32Ret = -1;
            break;
        }
    }

    if(FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        i32Ret = -1;
    }
    else
    {
        *pu32Data0 = FMC->MPDAT0;
        *pu32Data1 = FMC->MPDAT1;
    }
    return i32Ret;
}

/**
  * @brief Execute FMC_ISPCMD_READ_64 command to read a double-word from flash.
  * @param[in]  u32Addr   Address of the flash location to be read.
  *             It must be a double-word aligned address.
  * @param[out] u32data0  Place holder of word 0 read from flash address u32Addr.
  * @param[out] u32data1  Place holder of word 0 read from flash address u32Addr+4.
  * @return   0   Success
  * @return   -1  Failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read time-out
  */
int32_t FMC_Read_64_ECC(uint32_t u32Addr, uint32_t au32Data[])
{
    int32_t  ret = 0;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = 0;
    FMC->ISPCMD = FMC_ISPCMD_READ_64;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = 0x0UL;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_READ;
    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
    {
        if( i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = -1;
            ret = -1;
            break;
        }
    }

    if(g_bDoISPFFTest)
    {
        if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS = FMC_ISPSTS_ISPFF_Msk;
            g_FMC_i32ErrCode = -2;
            ret = -1;
        }
        else
        {
            au32Data[0] = FMC->MPDAT0;
            au32Data[1] = FMC->MPDAT1;
            au32Data[2] = FMC->MPDATE;
        }
    }
    else
    {
        au32Data[0] = FMC->MPDAT0;
        au32Data[1] = FMC->MPDAT1;
        au32Data[2] = FMC->MPDATE;
    }

    return ret;
}

int32_t FMC_Read_64_NoECC(uint32_t u32Addr, uint32_t au32Data[])
{
    int32_t ret = 0;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = 0;
    FMC->ISPCMD = FMC_ISPCMD_READ_NO_ECC;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = 0x0UL;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_READ;
    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
    {
        if( i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = -1;
            ret = -1;
            break;
        }
    }

    if(g_bDoISPFFTest)
    {
        if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
            g_FMC_i32ErrCode = -2;
            ret = -1;
        }
        else
        {
            au32Data[0] = FMC->MPDAT0;
            au32Data[1] = FMC->MPDAT1;
            au32Data[2] = FMC->MPDATE;
        }
    }
    else
    {
        au32Data[0] = FMC->MPDAT0;
        au32Data[1] = FMC->MPDAT1;
        au32Data[2] = FMC->MPDATE;
    }

    return ret;
}

/**
  * @brief Execute ISP FMC_ISPCMD_PROGRAM to program a word to flash.
  * @param[in]  u32Addr Address of the flash location to be programmed.
  *             It must be a word aligned address.
  * @param[in]  u32Data The word data to be programmed.
  * @return   FMC_OK   Success
  * @return   -1  Failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Program failed or time-out
  */
int32_t FMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;
    FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = u32Addr;
    FMC->ISPDAT = u32Data;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_WRITE;
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = -1;
            return g_FMC_i32ErrCode;
        }
    }

    if(g_bDoISPFFTest)
    {
        if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS = FMC_ISPSTS_ISPFF_Msk;
            g_FMC_i32ErrCode = -2;
            //printf("     * ISPFF is set !\n");
            return g_FMC_i32ErrCode;
        }
    }

    return FMC_OK;
}

/**
  * @brief Execute ISP FMC_ISPCMD_PROGRAM_64 to program a double-word to flash.
  * @param[in]  u32Addr Address of the flash location to be programmed.
  *             It must be a double-word aligned address.
  * @param[in]  u32Data0   The word data to be programmed to flash address u32Addr.
  * @param[in]  u32Data1   The word data to be programmed to flash address u32Addr+4.
  * @return   FMC_OK   Success
  * @return   -1  Failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Program failed or time-out
  */
int32_t FMC_Write8Bytes(uint32_t u32Addr, uint32_t u32Data0, uint32_t u32Data1)
{
    int32_t  i32Ret = FMC_OK;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;
    FMC->ISPCMD  = FMC_ISPCMD_PROGRAM_64;
    FMC->ISPADDR = u32Addr;
    FMC->MPDAT0  = u32Data0;
    FMC->MPDAT1  = u32Data1;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_WRITE;
    while(FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = -1;
            i32Ret = -1;
        }
    }

    if(FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        i32Ret = -1;
    }
    return i32Ret;
}

int32_t FMC_Write8Bytes_ECC(uint32_t u32Addr, uint32_t u32data0, uint32_t u32data1, uint32_t u32Syndrome)
{
    int32_t  ret = 0;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = 0;
    FMC->ISPCMD  = FMC_ISPCMD_PROGRAM_ECC;
    FMC->ISPADDR = u32Addr;
    FMC->MPDAT0  = u32data0;
    FMC->MPDAT1  = u32data1;
    FMC->MPDATE  = u32Syndrome;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_WRITE;
    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
    {
        if( i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = -1;
            ret = -1;
        }
    }

    if(g_bDoISPFFTest)
    {
        if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
        {
            FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
            g_FMC_i32ErrCode = -2;
            ret = -1;
        }
    }

    return ret;
}

/**
  * @brief   Program Multi-Word data into specified address of flash.
  * @param[in]  u32Addr    Start flash address in APROM where the data chunk to be programmed into.
  *                        This address must be 8-bytes aligned to flash address.
  * @param[in]  pu32Buf    Buffer that carry the data chunk.
  * @param[in]  u32Len     Length of the data chunk in bytes.
  * @retval   >=0  Number of data bytes were programmed.
  * @retval   -1   Program failed.
  * @retval   -2   Invalid address.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Program failed or time-out
  *           -2  Invalid address
  */
#if 1
int32_t FMC_WriteMultiple(uint32_t u32Addr, uint32_t pu32Buf[], uint32_t u32ByteLen)
{
    int32_t i32TimeOutCnt;
    int32_t i32Idx = 0, i32RetVal = 0;

    g_FMC_i32ErrCode = FMC_OK;

    /* u32Addr and u32ByteLen must be multiple of 16. */
    if ((u32Addr % 16) != 0 || (u32ByteLen % 16) != 0)
    {
        g_FMC_i32ErrCode = -2;
        return -2;
    }

    do
    {
        FMC->ISPADDR = u32Addr;
        FMC->MPDAT0  = pu32Buf[i32Idx++];
        FMC->MPDAT1  = pu32Buf[i32Idx++];
        FMC->MPDAT2  = pu32Buf[i32Idx++];
        FMC->MPDAT3  = pu32Buf[i32Idx++];
        FMC->ISPCMD  = FMC_ISPCMD_PROGRAM_MUL;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        do
        {
            i32TimeOutCnt = FMC_TIMEOUT_WRITE;

            while (FMC->MPSTS & (FMC_MPSTS_D0_Msk | FMC_MPSTS_D1_Msk))
            {
                if (i32TimeOutCnt-- <= 0)
                {
                    g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
                    return FMC_ERR_TIMEOUT;
                }
            }

            i32RetVal += 8;
            u32ByteLen -= 8;

            if (u32ByteLen < 8)
            {
                break;
            }

            if ((FMC->MPSTS & (FMC_MPSTS_MPBUSY_Msk | FMC_MPSTS_ISPFF_Msk)) != FMC_MPSTS_MPBUSY_Msk)
            {
                /* Busy flag is cleared after D0D1 cleared ! */
                break;
            }

            FMC->MPDAT0 = pu32Buf[i32Idx++];
            FMC->MPDAT1 = pu32Buf[i32Idx++];

            i32TimeOutCnt = FMC_TIMEOUT_WRITE;

            while (FMC->MPSTS & (FMC_MPSTS_D2_Msk | FMC_MPSTS_D3_Msk))
            {
                if (i32TimeOutCnt-- <= 0)
                {
                    g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
                    return FMC_ERR_TIMEOUT;
                }
            }

            i32RetVal += 8;
            u32ByteLen -= 8;

            if (u32ByteLen < 8)
            {
                break;
            }

            if ((FMC->MPSTS & (FMC_MPSTS_MPBUSY_Msk | FMC_MPSTS_ISPFF_Msk)) != FMC_MPSTS_MPBUSY_Msk)
            {
                /* Busy flag is cleared after D2D3 cleared ! */
                break;
            }

            FMC->MPDAT2 = pu32Buf[i32Idx++];
            FMC->MPDAT3 = pu32Buf[i32Idx++];
        } while ((i32Idx * 4) % FMC_MULTI_WORD_PROG_LEN);

        i32TimeOutCnt = FMC_TIMEOUT_WRITE;

        while (FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk)
        {
            if (i32TimeOutCnt-- <= 0)
            {
                g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
                return FMC_ERR_TIMEOUT;
            }
        }

        if (((i32Idx * 4) % FMC_MULTI_WORD_PROG_LEN) == 0)
        {
            if (!(FMC->MPSTS & FMC_MPSTS_ISPFF_Msk))
            {
                i32RetVal += 16;
                u32ByteLen -= 16;
                u32Addr += FMC_MULTI_WORD_PROG_LEN;
            }
        }
    } while (u32ByteLen >= 8);

    return i32RetVal;
}
#else
int32_t FMC_WriteMultiple(uint32_t u32Addr, uint32_t pu32Buf[], uint32_t u32Len)
{
    int   i, idx, retval = 0;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = 0;

    if ((u32Addr >= FMC_APROM_END) || ((u32Addr % 8) != 0))
    {
        g_FMC_i32ErrCode = -2;
        return -2;
    }

    u32Len = u32Len - (u32Len % 8);         /* u32Len must be multiple of 8. */

    idx = 0;

    while (u32Len >= 8)
    {
        FMC->ISPADDR = u32Addr;
        FMC->MPDAT0  = pu32Buf[idx++];
        FMC->MPDAT1  = pu32Buf[idx++];
        FMC->MPDAT2  = pu32Buf[idx++];
        FMC->MPDAT3  = pu32Buf[idx++];
        FMC->ISPCMD  = FMC_ISPCMD_PROGRAM_MUL;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        for (i = 16; i < FMC_MULTI_WORD_PROG_LEN; )
        {
            i32TimeOutCnt = FMC_TIMEOUT_WRITE;
            while (FMC->MPSTS & (FMC_MPSTS_D0_Msk | FMC_MPSTS_D1_Msk))
            {
                if( i32TimeOutCnt-- <= 0)
                {
                    g_FMC_i32ErrCode = -1;
                    return -1;
                }
            }

            retval += 8;
            u32Len -= 8;
            if (u32Len < 8)
            {
                return retval;
            }

            if (!(FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk))
            {
                /* printf("    [WARNING] busy cleared after D0D1 cleared!\n"); */
                i += 8;
                break;
            }

            FMC->MPDAT0 = pu32Buf[idx++];
            FMC->MPDAT1 = pu32Buf[idx++];

            if (i == FMC_MULTI_WORD_PROG_LEN/4)
                break;           // done

            i32TimeOutCnt = FMC_TIMEOUT_WRITE;
            while (FMC->MPSTS & (FMC_MPSTS_D2_Msk | FMC_MPSTS_D3_Msk))
            {
                if( i32TimeOutCnt-- <= 0)
                {
                    g_FMC_i32ErrCode = -1;
                    return -1;
                }
            }

            retval += 8;
            u32Len -= 8;
            if (u32Len < 8)
            {
                return retval;
            }

            if (!(FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk))
            {
                /* printf("    [WARNING] busy cleared after D2D3 cleared!\n"); */
                i += 8;
                break;
            }

            FMC->MPDAT2 = pu32Buf[idx++];
            FMC->MPDAT3 = pu32Buf[idx++];
        }

        if (i != FMC_MULTI_WORD_PROG_LEN)
        {
            /* printf("    [WARNING] Multi-word program interrupted at 0x%x !!\n", i); */
            return retval;
        }

        i32TimeOutCnt = FMC_TIMEOUT_WRITE;
        while (FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk)
        {
            if( i32TimeOutCnt-- <= 0)
            {
                g_FMC_i32ErrCode = -1;
                return -1;
            }
        }

        u32Addr += FMC_MULTI_WORD_PROG_LEN;
    }
    return retval;
}
#endif

/**
  * @brief Execute FMC_ISPCMD_READ command to read User Configuration.
  * @param[out]  u32Config A word array.
  *              u32Config[0] holds CONFIG0, while u32Config[1] holds CONFIG1.
  * @param[in] u32Count Available word count in u32Config.

  * @return Success or not.
  * @retval   FMC_OK  Success.
  * @retval   -1  Read failed
  * @retval   -2  Invalid parameter.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Read failed
  *           -2  Invalid parameter
  */
int32_t FMC_ReadConfig(uint32_t u32Config[], uint32_t u32Count)
{
    int32_t  i32ret = FMC_OK;
    uint32_t i;

    g_FMC_i32ErrCode = FMC_OK;

    if(u32Count > FMC_USER_CONFIG_CNT)
    {
        g_FMC_i32ErrCode = -2;
    }
    else
    {
        for (i = 0UL; i < u32Count; i++)
        {
            if (i < 16)
                u32Config[i] = FMC_Read(FMC_CONFIG_BASE + (i << 2));
            else
                u32Config[i] = FMC_Read(FMC_USER_CONFIG_16 + ((i - 16) << 2));

            if (g_FMC_i32ErrCode != 0)
                break;
        }
    }

    i32ret = g_FMC_i32ErrCode;
    return i32ret;
}


/**
  * @brief Execute ISP commands to erase then write User Configuration.
  * @param[in] u32ConfigAddr   Address of config word
  * @param[in] u32ConfigVal    New config value to be written.
  * @retval    FMC_OK  Success
  * @retval   -1  Erase/program/read/verify failed
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           < 0  Errors caused by erase/program/read failed or time-out
  */
int32_t FMC_WriteConfig(uint32_t u32ConfigAddr, uint32_t u32ConfigVal)
{
    uint32_t bIsCFGUEN = (FMC->ISPCTL & FMC_ISPCTL_CFGUEN_Msk) >> FMC_ISPCTL_CFGUEN_Pos;
    
    if (!bIsCFGUEN)
        FMC_ENABLE_CFG_UPDATE();

    if(FMC_EraseConfig(u32ConfigAddr) != 0)
    {
        return -1;
    }

    if(FMC_Read(u32ConfigAddr) != 0xFFFFFFFF)
    {
        if (!bIsCFGUEN)
            FMC_DISABLE_CFG_UPDATE();
        return -1;
    }

    if(g_FMC_i32ErrCode != FMC_OK)
    {
        if (!bIsCFGUEN)
            FMC_DISABLE_CFG_UPDATE();
        return -1;
    }

    if(FMC_Write(u32ConfigAddr, u32ConfigVal) != FMC_OK)
    {
        if (!bIsCFGUEN)
            FMC_DISABLE_CFG_UPDATE();
        return -1;
    }

    if(FMC_Read(u32ConfigAddr) != u32ConfigVal)
    {
        if (!bIsCFGUEN)
            FMC_DISABLE_CFG_UPDATE();
        return -1;
    }

    if(g_FMC_i32ErrCode != FMC_OK)
    {
        if (!bIsCFGUEN)
            FMC_DISABLE_CFG_UPDATE();
        return -1;
    }

    if (!bIsCFGUEN)
            FMC_DISABLE_CFG_UPDATE();
    return FMC_OK;
}

/**
  * @brief      Execute FMC_ISPCMD_CFG_ERASE command to erase a config word.
  * @param[in]  u32ConfigAddr Address of config word to be erased.
  * @return     ISP config erase success or not.
  * @retval     FMC_OK           Success
  * @retval     FMC_ERR_FAIL     Erase failed
  * @retval     FMC_ERR_TIMEOUT  Erase time-out
  *
  * @note       Global error code g_FMC_i32ErrCode
  *             FMC_ERR_FAIL     Erase failed
  *             FMC_ERR_TIMEOUT  Erase time-out
  */
int32_t FMC_EraseConfig(uint32_t u32ConfigAddr)
{
    int32_t i32RetCode = FMC_OK;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;

    FMC->ISPCMD  = FMC_ISPCMD_CFG_ERASE;
    FMC->ISPADDR = u32ConfigAddr;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_ERASE;

    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = FMC_ERR_TIMEOUT;
            i32RetCode = FMC_ERR_TIMEOUT;
            break;
        }
    }

    if(FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = FMC_ERR_FAIL;
        i32RetCode = FMC_ERR_FAIL;
    }

    return i32RetCode;
}


/**
  * @brief Run CRC32 checksum calculation and get result.
  * @param[in] u32Addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of page size.
  * @retval   0           Success.
  * @retval   0xFFFFFFFF  Invalid parameter or command failed.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Run/Read check sum time-out failed
  *           -2  u32Addr or u32count must be aligned with page size
  */
uint32_t  FMC_GetChkSum(uint32_t u32Addr, uint32_t u32count)
{
    uint32_t ret;
    int32_t  i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;

    if((u32Addr % FMC_FLASH_PAGE_SIZE) || (u32count % FMC_FLASH_PAGE_SIZE))
    {
       g_FMC_i32ErrCode = -2;
        ret = 0xFFFFFFFF;
    }
    else
    {
        FMC->ISPCMD  = FMC_ISPCMD_RUN_CKS;
        FMC->ISPADDR = u32Addr;
        FMC->ISPDAT  = u32count;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = FMC_TIMEOUT_CHKSUM;
        while(FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
        {
            if(i32TimeOutCnt-- <= 0)
            {
                g_FMC_i32ErrCode = -1;
                return 0xFFFFFFFF;
            }
        }

        FMC->ISPCMD = FMC_ISPCMD_READ_CKS;
        FMC->ISPADDR = u32Addr;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

        i32TimeOutCnt = FMC_TIMEOUT_CHKSUM;
        while(FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
        {
            if(i32TimeOutCnt-- <= 0)
            {
                g_FMC_i32ErrCode = -1;
                return 0xFFFFFFFF;
            }
        }

        ret = FMC->ISPDAT;
    }

    return ret;
}


/**
  * @brief Run flash all one verification and get result.
  * @param[in] u32Addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of page size.
  * @retval   READ_ALLONE_YES       The contents of verified flash area are 0xFFFFFFFF.
  * @retval   READ_ALLONE_NOT       Some contents of verified flash area are not 0xFFFFFFFF.
  * @retval   READ_ALLONE_CMD_FAIL  Unexpected error occurred.
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  RUN_ALL_ONE or CHECK_ALL_ONE commands time-out
  */
uint32_t  FMC_CheckAllOne(uint32_t u32Addr, uint32_t u32count)
{
    uint32_t  ret = READ_ALLONE_CMD_FAIL;
    int32_t i32TimeOutCnt0, i32TimeOutCnt1;

    g_FMC_i32ErrCode = FMC_OK;

    FMC->ISPSTS = FMC_ISPSTS_ALLONE_Msk;   /* Clear check all one bit */

    FMC->ISPCMD   = FMC_ISPCMD_RUN_ALL1;
    FMC->ISPADDR  = u32Addr;
    FMC->ISPDAT   = u32count;
    FMC->ISPTRG   = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt0 = FMC_TIMEOUT_CHKALLONE;
    while(FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
    {
        if(i32TimeOutCnt0-- <= 0)
        {
            g_FMC_i32ErrCode = -1;
            break;
        }
    }

    if(g_FMC_i32ErrCode == FMC_OK)
    {
        i32TimeOutCnt1 = FMC_TIMEOUT_CHKALLONE;
        do
        {
            FMC->ISPCMD = FMC_ISPCMD_READ_ALL1;
            FMC->ISPADDR = u32Addr;
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

            i32TimeOutCnt0 = FMC_TIMEOUT_CHKALLONE;
            while(FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
            {
                if(i32TimeOutCnt0-- <= 0)
                {
                    g_FMC_i32ErrCode = -1;
                    break;
                }
            }

            if(i32TimeOutCnt1-- <= 0)
            {
                g_FMC_i32ErrCode = -1;
            }
        }
        while((FMC->ISPDAT == 0UL) && (g_FMC_i32ErrCode == 0));
    }

    if(g_FMC_i32ErrCode == FMC_OK)
    {
        if(FMC->ISPDAT == READ_ALLONE_YES)
            ret = READ_ALLONE_YES;
        else if(FMC->ISPDAT == READ_ALLONE_NOT)
            ret = READ_ALLONE_NOT;
        else
            g_FMC_i32ErrCode = -1;
    }

    return ret;
}

/**
  * @brief     Remap Bank0/Bank1 or Segment0/Segment1
  *
  * @param[in] u32Bank    Bank/Segment number which will remap to.
  *
  * @retval    FMC_OK   Success
  * @retval    -1  Program failed.
  *
  * @details  Remap Bank0/Bank1 or Segment0/Segment1
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Program failed or time-out
  */
int32_t FMC_RemapBank(uint32_t u32Bank)
{
    int32_t i32Ret = FMC_OK;
    int32_t i32TimeOutCnt;

    g_FMC_i32ErrCode = FMC_OK;

    FMC->ISPCMD = FMC_ISPCMD_BANK_REMAP;
    FMC->ISPADDR = u32Bank;
    FMC->ISPDAT = 0x5AA55AA5UL;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_WRITE;
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = -1;
            i32Ret = -1;
            break;
        }
    }

    if(FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
    {
        FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
        g_FMC_i32ErrCode = -1;
        i32Ret = -1;
    }
    return i32Ret;
}


/*@}*/ /* end of group FMC_EXPORTED_FUNCTIONS */
/*@}*/ /* end of group FMC_Driver */
/*@}*/ /* end of group Standard_Driver */
