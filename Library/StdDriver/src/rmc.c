/**************************************************************************//**
 * @file     RMC.c
 * @version  V1.00
 * @brief    M2L31 series RRAM Memory Controller driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>

#include "NuMicro.h"


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup RMC_Driver RMC Driver
  @{
*/


/** @addtogroup RMC_EXPORTED_FUNCTIONS RMC Exported Functions
  @{
*/

int32_t  g_RMC_i32ErrCode = 0;

/* Wait until ISPTRG.ISPGO is cleared or timeout occurs. */
static int32_t RMC_WaitISPDone(uint32_t u32Timeout)
{
    uint32_t u32Remain;
    u32Remain = u32Timeout;

    while ((u32Remain > 0UL) && ((RMC->ISPTRG & RMC_ISPTRG_ISPGO_Msk) != 0UL))
    {
        u32Remain--;
    }

    return (u32Remain == 0UL) ? -1L : 0L;
}


/* Clear data buffer and check ISPSTS fail flag. */
static int32_t RMC_ClearDataBuffer(void)
{
    int32_t i32Ret;

    RMC->ISPCMD  = RMC_ISPCMD_CLEAR_DATA_BUFFER;
    RMC->ISPADDR = 0x00000000UL;
    RMC->ISPTRG  = RMC_ISPTRG_ISPGO_Msk;

    i32Ret = RMC_WaitISPDone(RMC_TIMEOUT_WRITE);
    if (i32Ret != 0L)
    {
        return -1L;
    }

    if ((RMC->ISPSTS & RMC_ISPSTS_ISPFF_Msk) != 0UL)
    {
        RMC->ISPSTS |= RMC_ISPSTS_ISPFF_Msk;
        return -1L;
    }

    return 0L;
}

/**
  * @brief    Disable ISP Functions
  *
  * @param    None
  *
  * @return   None
  *
  * @details  This function will clear ISPEN bit of ISPCON to disable ISP function
  *
  */
void RMC_Close(void)
{
    RMC->ISPCTL &= ~RMC_ISPCTL_ISPEN_Msk;
}

/**
  * @brief     Config XOM Region
  * @param[in] u32XomNum    The XOM number(0~3)
  * @param[in] u32XomBase   The XOM region base address.
  * @param[in] u8XomPage    The XOM page number of region size.
  *
  * @retval    0  Success
  * @retval    1  XOM is has already actived.
  * @retval   -1  Program failed.
  * @retval   -2  Invalid XOM number.
  *
  * @details  Program XOM base address and XOM size(page)
  */
int32_t RMC_ConfigXOM(uint32_t u32XomNum, uint32_t u32XomBase, uint8_t u8XomPage)
{
    int32_t  ret = 0L;

    /* Workaround solution: Check ISPADDR to know if wakeup from power-down mode.
       If Magic Number exists, call Read CID command to avoid issue 2.5 (Please refer to Errata Sheet)
     */
    if(RMC_CHECK_MAGICNUM() != 0UL)
    {
        (void)RMC_DummyReadCID();
    }
    g_RMC_i32ErrCode = 0L;

    if(u32XomNum >= 4UL)
    {
        ret = -2L;
    }

    if(ret == 0L)
    {
        ret = RMC_GetXOMState(u32XomNum);
    }


    if (ret == 0L)
    {
        uint32_t u32XomAddr;

        u32XomAddr = RMC_XOM_BASE + (u32XomNum * 0x10UL);

        g_RMC_i32ErrCode = RMC_Write(u32XomAddr, u32XomBase);
        ret = g_RMC_i32ErrCode;

        if (ret == 0L)
        {
            g_RMC_i32ErrCode = RMC_Write((u32XomAddr + 0x04UL), (uint32_t)u8XomPage);
            ret = g_RMC_i32ErrCode;
        }

        if (ret == 0L)
        {
            g_RMC_i32ErrCode = RMC_Write((u32XomAddr + 0x08UL), 0UL);
            ret = g_RMC_i32ErrCode;
        }
    }

    return ret;
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
  * @details Execute RMC_ISPCMD_PAGE_ERASE command to erase XOM.
  */
int32_t RMC_EraseXOM(uint32_t u32XomNum)
{
    uint32_t u32Addr = 0UL;
    uint32_t u32EraseData = 0x55AA03UL;
    int32_t  i32Active;
    int32_t  err = 0L;

    /* Workaround solution: Check ISPADDR to know if wakeup from power-down mode.
       If Magic Number exists, call Read CID command to avoid issue 2.5 (Please refer to Errata Sheet)
     */
    if (RMC_CHECK_MAGICNUM() != 0UL)
    {
        (void)RMC_DummyReadCID();
    }

    g_RMC_i32ErrCode = 0L;

    if(u32XomNum >= 5UL)
    {
        err = -2L;
    }

    if (err == 0L)
    {
        i32Active = RMC_GetXOMState(u32XomNum);
        RMC->ISPCTL &= ~RMC_ISPCTL_MPEN_Msk;

        if (i32Active != 0L)
        {
            switch(u32XomNum)
            {
            case 0UL:
                u32Addr = (RMC->XOMR0STS & 0xFFFFFF00UL) >> 8UL;
                break;

            case 1UL:
                u32Addr = (RMC->XOMR1STS & 0xFFFFFF00UL) >> 8UL;
                break;

            case 2UL:
                u32Addr = (RMC->XOMR2STS & 0xFFFFFF00UL) >> 8UL;
                break;

            case 3UL:
                u32Addr = (RMC->XOMR3STS & 0xFFFFFF00UL) >> 8UL;
                break;

            case 4UL:
                u32Addr = (RMC->XOMR0STS & 0xFFFFFF00UL) >> 8UL;
                u32EraseData = 0x0UL; /*  preserve original special case */
                break;

            default:
                err = -2L;
                break;
            }

            if (err == 0L)
            {
                RMC->ISPCMD  = RMC_ISPCMD_PAGE_ERASE;
                RMC->ISPADDR = u32Addr;
                RMC->ISPDAT  = u32EraseData;
                RMC->ISPTRG  = 0x1UL;

#if defined(ISBEN) && (ISBEN != 0)
                __ISB();
#endif

                if (RMC_WaitISPDone(RMC_TIMEOUT_ERASE) != 0L)
                {
                    err = -1L;
                }
                /* Check ISPFF flag to know whether erase OK or fail. */
                if ((err == 0L) && ((RMC->ISPCTL & RMC_ISPCTL_ISPFF_Msk) != 0UL))
                {
                    RMC->ISPCTL |= RMC_ISPCTL_ISPFF_Msk;

                    if (RMC_ClearDataBuffer() != 0L)
                    {
                        /* keep err as failure */
                    }

                    if ((RMC->ISPSTS & RMC_ISPSTS_ISPFF_Msk) != 0UL)
                    {
                        RMC->ISPSTS |= RMC_ISPSTS_ISPFF_Msk;
                    }

                    err = -1L;
                }
            }
        }
        else
        {
            err = -1L;
        }
    }

    return err;
}

/**
  * @brief  Check the XOM is actived or not.
  *
  * @param[in] u32XomNum    The xom number(0~3).
  *
  * @retval    1   XOM is actived.
  * @retval    0   XOM is not actived.
  * @retval   -2   Invalid XOM number.
  *
  * @details To get specify XOMRn(n = 0 ~ 3) active status
  */
int32_t RMC_GetXOMState(uint32_t u32XomNum)
{
    int32_t  ret = 0L;

    if(u32XomNum >= 4UL)
    {
        ret = -2L;
    }

    if(ret >= 0L)
    {
        uint32_t u32act;
        u32act = (((RMC->XOMSTS) & 0xfUL) & (1UL << u32XomNum)) >> u32XomNum;
        ret = (int32_t)u32act;
    }
    return ret;
}

/**
  * @brief Enable RMC ISP function
  * @return None
  */
void RMC_Open(void)
{
    RMC->ISPCTL |=  RMC_ISPCTL_ISPEN_Msk;
}


/**
  * @brief Execute RMC_ISPCMD_READ command to read a word from flash.
  * @param[in]  u32Addr Address of the flash location to be read.
  *             It must be a word aligned address.
  * @return The word data read from specified flash address.
  */
uint32_t RMC_Read(uint32_t u32Addr)
{
    uint32_t  tout;

    /* Workaround solution: Check ISPADDR to know if wakeup from power-down mode.
       If Magic Number exists, call Read CID command to avoid issue 2.5 (Please refer to Errata Sheet)
     */
    if(RMC_CHECK_MAGICNUM() != 0UL)
    {
        (void)RMC_DummyReadCID();
    }
    g_RMC_i32ErrCode = 0L;
    RMC->ISPCTL = RMC->ISPCTL & ~RMC_ISPCTL_MPEN_Msk;
    RMC->ISPCMD = RMC_ISPCMD_READ;
    RMC->ISPADDR = u32Addr;
    RMC->ISPTRG = RMC_ISPTRG_ISPGO_Msk;
    tout = RMC_TIMEOUT_READ;

    while ((--tout > 0UL) && (RMC->ISPTRG & RMC_ISPTRG_ISPGO_Msk)) {}

    if (tout == 0UL)
    {
        g_RMC_i32ErrCode = -1L;
        return 0xFFFFFFFFUL;
    }
    if(RMC->ISPCTL & RMC_ISPCTL_ISPFF_Msk)
    {
        RMC->ISPCTL |= RMC_ISPCTL_ISPFF_Msk;
        g_RMC_i32ErrCode = -1L;
        return 0xFFFFFFFFUL;
    }
    return RMC->ISPDAT;
}

/**
  * @brief    Get the base address of Data Flash if enabled.
  * @retval   The base address of Data Flash
  */
uint32_t RMC_ReadDataFlashBaseAddr(void)
{
    return RMC->DFBA;
}

/**
  * @brief      Set boot source from LDROM or APROM after next software reset
  * @param[in]  i32BootSrc
  *                1: Boot from LDROM
  *                0: Boot from APROM
  * @return    None
  * @details   This function is used to switch APROM boot or LDROM boot. User need to call
  *            RMC_SetBootSource to select boot source first, then use CPU reset or
  *            System Reset Request to reset system.
  */
void RMC_SetBootSource(int32_t i32BootSrc)
{
    if(i32BootSrc)
    {
        RMC->ISPCTL |= RMC_ISPCTL_BS_Msk; /* Boot from LDROM */
    }
    else
    {
        RMC->ISPCTL &= ~RMC_ISPCTL_BS_Msk;/* Boot from APROM */
    }
}

/**
  * @brief Get the current boot source.
  * @return The current boot source.
  * @retval   0  Is boot from APROM.
  * @retval   1  Is boot from LDROM.
  */
int32_t RMC_GetBootSource (void)
{
    int32_t  ret = 0L;

    if (RMC->ISPCTL & RMC_ISPCTL_BS_Msk)
    {
        ret = 1L;
    }

    return ret;
}

/**
  * @brief Execute ISP RRAM program flow to program a word to flash.
  * @param[in]  u32Addr Address of the flash location to be programmed.
  *             It must be a word aligned address.
  * @param[in]  u32Data The word data to be programmed.
  * @return None
  */
int32_t RMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    int32_t i32Ret = 0L;

    /* Workaround solution: Check ISPADDR to know if wakeup from power-down mode.
       If Magic Number exists, call Read CID command to avoid issue 2.5 (Please refer to Errata Sheet)
     */
    if (RMC_CHECK_MAGICNUM() != 0UL)
    {
        (void)RMC_DummyReadCID();
    }

    g_RMC_i32ErrCode = 0L;
    RMC->ISPCTL &= ~RMC_ISPCTL_MPEN_Msk;

    /* Clear data buffer */
    if (RMC_ClearDataBuffer() != 0L)
    {
        i32Ret = -1L;
    }

    /* Load data buffer */
    if (i32Ret == 0L)
    {
        RMC->ISPCMD  = RMC_ISPCMD_LOAD_DATA_BUFFER;
        RMC->ISPADDR = u32Addr;
        RMC->ISPDAT  = u32Data;
        RMC->ISPTRG  = RMC_ISPTRG_ISPGO_Msk;

        if (RMC_WaitISPDone(RMC_TIMEOUT_WRITE) != 0L)
        {
            i32Ret = -1L;
        }
        else if ((RMC->ISPSTS & RMC_ISPSTS_ISPFF_Msk) != 0UL)
        {
            RMC->ISPSTS |= RMC_ISPSTS_ISPFF_Msk;
            i32Ret = -1L;
        }
        else
        {

        }
    }

    /* Program */
    if (i32Ret == 0L)
    {
        RMC->ISPCMD  = RMC_ISPCMD_PROGRAM;
        RMC->ISPADDR = u32Addr;
        RMC->ISPDAT  = u32Data;
        RMC->ISPTRG  = RMC_ISPTRG_ISPGO_Msk;

        if (RMC_WaitISPDone(RMC_TIMEOUT_WRITE) != 0L)
        {
            i32Ret = -1L;
        }
        else if ((RMC->ISPSTS & RMC_ISPSTS_ISPFF_Msk) != 0UL)
        {
            RMC->ISPSTS |= RMC_ISPSTS_ISPFF_Msk;
            i32Ret = -1L;
        }
        else
        {

        }
    }

    if (i32Ret != 0L)
    {
        g_RMC_i32ErrCode = -1L;

        /* Best-effort cleanup of data buffer after failure. */
        (void)RMC_ClearDataBuffer();

        if ((RMC->ISPSTS & RMC_ISPSTS_ISPFF_Msk) != 0UL)
        {
            RMC->ISPSTS |= RMC_ISPSTS_ISPFF_Msk;
        }

        return -1L;
    }

    return 0L;
}

/**
  * @brief Execute erase operation to erase a flash page (Use Word Line Program). The page size is 4096 bytes.
  * @param[in]  u32PageAddr Address of the flash page to be erased.
  *             It must be a 512 bytes aligned address.
  * @return ISP page erase success or not.
  * @retval    0  Success
  * @retval   -1  Erase failed
  *           -2  Invalid parameter
  */
int32_t RMC_Erase(uint32_t u32PageAddr)
{
    uint32_t u32Addr;
    uint32_t u32EndAddr;
    int32_t  i32Err = 0L;

    /* Workaround solution: Check ISPADDR to know if wakeup from power-down mode.
       If Magic Number exists, call Read CID command to avoid issue 2.5 (Please refer to Errata Sheet)
     */
    if (RMC_CHECK_MAGICNUM() != 0UL)
    {
        (void)RMC_DummyReadCID();
    }

    g_RMC_i32ErrCode = 0L;
    u32Addr = u32PageAddr;
    u32EndAddr = u32PageAddr + RMC_FLASH_PAGE_SIZE;

    if ((u32Addr % 256UL) != 0UL)
    {
        return -2L;
    }

    if (u32Addr < RMC_APROM_END)
    {
        if ((u32Addr + RMC_FLASH_PAGE_SIZE) > RMC_APROM_END)
        {
            return -2L;
        }
    }
    else if ((u32Addr >= RMC_LDROM_BASE) && (u32Addr < RMC_LDROM_END))
    {
        if ((u32Addr + RMC_FLASH_PAGE_SIZE) > RMC_LDROM_END)
        {
            return -2L;
        }
    }
    else
    {
        return -2L;
    }

    while ((u32Addr < u32EndAddr) && (i32Err == 0L))
    {
        uint32_t u32Len;
        uint32_t u32Idx;
        u32Len = RMC_MULTI_WORD_PROG_MAX_LEN;
        RMC->ISPCTL |= RMC_ISPCTL_MPEN_Msk;

        if (RMC_ClearDataBuffer() != 0L)
        {
            i32Err = -1L;
        }

        u32Idx = 0UL;
        while ((u32Len > 0UL) && (i32Err == 0L))
        {
            uint32_t u32ProgAddr;

            u32ProgAddr = u32Addr + (u32Idx * 4UL);

            RMC->ISPCMD  = RMC_ISPCMD_LOAD_DATA_BUFFER;
            RMC->ISPADDR = u32ProgAddr;
            RMC->ISPDAT  = 0xFFFFFFFFUL;
            RMC->MPDAT1  = 0xFFFFFFFFUL;
            RMC->ISPTRG  = RMC_ISPTRG_ISPGO_Msk;

            if (RMC_WaitISPDone(RMC_TIMEOUT_WRITE) != 0L)
            {
                i32Err = -1L;
            }
            else if ((RMC->ISPSTS & RMC_ISPSTS_ISPFF_Msk) != 0UL)
            {
                RMC->ISPSTS |= RMC_ISPSTS_ISPFF_Msk;
                i32Err = -1L;
            }
            else
            {

            }

            u32Idx += 2UL;
            u32Len -= 8UL;
        }

        if (i32Err == 0L)
        {
            RMC->ISPCMD  = RMC_ISPCMD_PROGRAM;
            RMC->ISPADDR = u32Addr;
            RMC->ISPDAT  = 0xFFFFFFFFUL;
            RMC->ISPTRG  = RMC_ISPTRG_ISPGO_Msk;

            if (RMC_WaitISPDone(RMC_TIMEOUT_WRITE) != 0L)
            {
                i32Err = -1L;
            }
            else if ((RMC->ISPSTS & RMC_ISPSTS_ISPFF_Msk) != 0UL)
            {
                RMC->ISPSTS |= RMC_ISPSTS_ISPFF_Msk;
                i32Err = -1L;
            }
            else
            {

            }
        }

        u32Addr += RMC_MULTI_WORD_PROG_MAX_LEN;
    }

    RMC->ISPCTL &= ~RMC_ISPCTL_MPEN_Msk;

    if (i32Err != 0L)
    {
        g_RMC_i32ErrCode = -1L;
        (void)RMC_ClearDataBuffer();

        if ((RMC->ISPSTS & RMC_ISPSTS_ISPFF_Msk) != 0UL)
        {
            RMC->ISPSTS |= RMC_ISPSTS_ISPFF_Msk;
        }

        return -1L;
    }

    return 0L;
}

/**
  * @brief Execute RMC_ISPCMD_READ command to read User Configuration.
  * @param[out]  u32Config A two-word array.
  *              u32Config[0] holds CONFIG0, while u32Config[1] holds CONFIG1.
  * @param[in] u32Count Available word count in u32Config.
  * @return Success or not.
  * @retval    0  Success.
  * @retval   -1  Invalid parameter.
  */
int32_t RMC_ReadConfig(uint32_t u32Config[], uint32_t u32Count)
{
    int32_t   ret = 0L;

    /* Workaround solution: Check ISPADDR to know if wakeup from power-down mode.
       If Magic Number exists, call Read CID command to avoid issue 2.5 (Please refer to Errata Sheet)
     */
    if(RMC_CHECK_MAGICNUM() != 0UL)
    {
        (void)RMC_DummyReadCID();
    }
    u32Config[0] = RMC_Read(RMC_CONFIG_BASE);

    if (u32Count < 2UL)
    {
        ret = -1L;
    }
    else
    {
        u32Config[1] = RMC_Read(RMC_CONFIG_BASE+4UL);
    }
    return ret;
}


/**
  * @brief Execute ISP commands to erase then write User Configuration.
  * @param[in] u32Config    A two-word array.
  *            u32Config[0] holds CONFIG0, while u32Config[1] holds CONFIG1.
  * @param[in] u32Count     The number of User Configuration words to be written.
  * @return Success or not.
  * @retval    0  Success
  * @retval   -1  Failed
  */
int32_t RMC_WriteConfig(const uint32_t u32Config[], uint32_t u32Count)
{
    uint32_t u32Idx;

    /* Defensive check */
    if (u32Config == (const uint32_t *)NULL)
    {
        return -1L;
    }

    /* Workaround solution: Check ISPADDR to know if wakeup from power-down mode.
       If Magic Number exists, call Read CID command to avoid issue 2.5 (Please refer to Errata Sheet)
     */
    if (RMC_CHECK_MAGICNUM() != 0UL)
    {
        (void)RMC_DummyReadCID();
    }

    RMC_ENABLE_CFG_UPDATE();
    RMC->ISPCTL &= ~RMC_ISPCTL_MPEN_Msk;

    for (u32Idx = 0UL; u32Idx < u32Count; u32Idx++)
    {
        uint32_t u32CfgAddr;
        uint32_t u32ReadBack;
        int32_t  i32WriteRet;

        u32CfgAddr = RMC_CONFIG_BASE + (u32Idx * 4UL);

        i32WriteRet = RMC_Write(u32CfgAddr, u32Config[u32Idx]);
        if (i32WriteRet != 0L)
        {
            RMC_DISABLE_CFG_UPDATE();
            return -1L;
        }

        u32ReadBack = RMC_Read(u32CfgAddr);
        if (u32ReadBack != u32Config[u32Idx])
        {
            RMC_DISABLE_CFG_UPDATE();
            return -1L;
        }

        if (g_RMC_i32ErrCode != 0L)
        {
            RMC_DISABLE_CFG_UPDATE();
            return -1L;
        }
    }

    RMC_DISABLE_CFG_UPDATE();
    return 0L;
}

/**
 * @brief      Write Multi-Word bytes to flash
 *
 * @param[in]  u32Addr    Start flash address in APROM where the data chunk to be programmed into.
 *                        This address must be 8-bytes aligned to flash address.
 * @param[in]  pu32Buf    Buffer that carry the data chunk.
 * @param[in]  u32Len     Length of the data chunk in bytes.
 *
 * @retval   >=0  Number of data bytes were programmed.
 * @return   -1   Program failed.
 * @return   -2   Invalid address or length
 *
 * @detail     Program Multi-Word data into specified address of flash.
 * @note     Global error code g_RMC_i32ErrCode
 *           -1  Program failed or time-out
 *           -2  Invalid address or length
 */
int32_t  RMC_WriteMultiple(uint32_t u32Addr, const uint32_t pu32Buf[], uint32_t u32Len)
{
    uint32_t u32Idx;
    uint32_t u32Remain;
    int32_t  i32Err = 0L;
    uint32_t u32WrittenBytes;
    /* Defensive checks */
    if (pu32Buf == (const uint32_t *)NULL)
    {
        return -2L;
    }

    /* Workaround solution: Check ISPADDR to know if wakeup from power-down mode.
       If Magic Number exists, call Read CID command to avoid issue 2.5 (Please refer to Errata Sheet)
     */
    if (RMC_CHECK_MAGICNUM() != 0UL)
    {
        (void)RMC_DummyReadCID();
    }

    g_RMC_i32ErrCode = 0L;

    if (((u32Addr % 256UL) != 0UL) || ((u32Len % 8UL) != 0UL) || (u32Len > RMC_MULTI_WORD_PROG_MAX_LEN))
    {
        return -2L;
    }

    if (u32Addr < RMC_APROM_END)
    {
        if((u32Addr + u32Len) > RMC_APROM_END)
        {
            return -2L;
        }
    }
    else if ((u32Addr >= RMC_LDROM_BASE) && (u32Addr < RMC_LDROM_END))
    {
        if ((u32Addr + u32Len) > RMC_LDROM_END)
        {
            return -2L;
        }
    }
    else
    {
        return -2L; /* original code fell through */
    }

    RMC->ISPCTL |= RMC_ISPCTL_MPEN_Msk;

    if (RMC_ClearDataBuffer() != 0L)
    {
        i32Err = -1L;
    }

    u32Idx = 0UL;
    u32Remain = u32Len; /* local copy, do not modify function parameter */

    while ((u32Remain > 0UL) && (i32Err == 0L))
    {
        uint32_t u32ProgAddr;

        u32ProgAddr = u32Addr + (u32Idx * 4UL);

        RMC->ISPCMD  = RMC_ISPCMD_LOAD_DATA_BUFFER;
        RMC->ISPADDR = u32ProgAddr;
        RMC->ISPDAT  = pu32Buf[u32Idx];
        u32Idx++;
        RMC->MPDAT1  = pu32Buf[u32Idx];
        u32Idx++;
        RMC->ISPTRG  = RMC_ISPTRG_ISPGO_Msk;

        if (RMC_WaitISPDone(RMC_TIMEOUT_WRITE) != 0L)
        {
            i32Err = -1L;
        }
        else if ((RMC->ISPSTS & RMC_ISPSTS_ISPFF_Msk) != 0UL)
        {
            RMC->ISPSTS |= RMC_ISPSTS_ISPFF_Msk;
            i32Err = -1L;
        }
        else
        {

        }

        if (i32Err == 0L)
        {
            u32Remain -= 8UL;
        }
    }

    if (i32Err == 0L)
    {
        RMC->ISPCMD  = RMC_ISPCMD_PROGRAM;
        RMC->ISPADDR = u32Addr;
        RMC->ISPDAT  = pu32Buf[0];
        RMC->ISPTRG  = RMC_ISPTRG_ISPGO_Msk;

        if (RMC_WaitISPDone(RMC_TIMEOUT_WRITE) != 0L)
        {
            i32Err = -1L;
        }
        else if ((RMC->ISPSTS & RMC_ISPSTS_ISPFF_Msk) != 0UL)
        {
            RMC->ISPSTS |= RMC_ISPSTS_ISPFF_Msk;
            i32Err = -1L;
        }
        else
        {

        }
    }

    RMC->ISPCTL &= ~RMC_ISPCTL_MPEN_Msk;

    if (i32Err != 0L)
    {
        g_RMC_i32ErrCode = -1L;
        (void)RMC_ClearDataBuffer();

        if ((RMC->ISPSTS & RMC_ISPSTS_ISPFF_Msk) != 0UL)
        {
            RMC->ISPSTS |= RMC_ISPSTS_ISPFF_Msk;
        }

        return -1L;
    }

    u32WrittenBytes = (u32Idx * 4UL);

    if (u32WrittenBytes > 0x7FFFFFFFUL) /* prevent narrowing overflow */
    {
        return -1L; /* fail-safe */
    }

    return (int32_t)u32WrittenBytes; /* cast single object only */
}

/**
  * @brief Run CRC32 checksum calculation and get result.
  * @param[in] u32addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of 512 bytes.
  * @return Success or not.
  * @retval   0           Success.
  * @retval   0xFFFFFFFF  Invalid parameter.
  */
uint32_t  RMC_GetChkSum(uint32_t u32addr, uint32_t u32count)
{
    uint32_t   ret;

    /* Workaround solution: Check ISPADDR to know if wakeup from power-down mode.
       If Magic Number exists, call Read CID command to avoid issue 2.5 (Please refer to Errata Sheet)
     */
    if(RMC_CHECK_MAGICNUM() != 0UL)
    {
        (void)RMC_DummyReadCID();
    }
    g_RMC_i32ErrCode = 0L;

    if ((u32addr % 512UL) || (u32count % 512UL))
    {
        ret = 0xFFFFFFFFUL;
    }
    else
    {
        uint32_t   tout;
        RMC->ISPCTL = RMC->ISPCTL & ~RMC_ISPCTL_MPEN_Msk;
        RMC->ISPCMD  = RMC_ISPCMD_RUN_CKS;
        RMC->ISPADDR = u32addr;
        RMC->ISPDAT  = u32count;
        RMC->ISPTRG  = RMC_ISPTRG_ISPGO_Msk;
        tout = RMC_TIMEOUT_CHKSUM;

        while ((--tout > 0UL) && (RMC->ISPSTS & RMC_ISPSTS_ISPBUSY_Msk)) {}

        if (tout == 0UL)
        {
            g_RMC_i32ErrCode = -1L;
            return 0xFFFFFFFFUL;
        }

        if (RMC->ISPSTS & RMC_ISPSTS_ISPFF_Msk)
        {
            RMC->ISPSTS |= RMC_ISPSTS_ISPFF_Msk;
            g_RMC_i32ErrCode = -1L;
            return 0xFFFFFFFFUL;
        }
        RMC->ISPCMD = RMC_ISPCMD_READ_CKS;
        RMC->ISPADDR    = u32addr;
        RMC->ISPTRG = RMC_ISPTRG_ISPGO_Msk;

        tout = RMC_TIMEOUT_CHKSUM;
        while ((--tout > 0UL) && (RMC->ISPSTS & RMC_ISPSTS_ISPBUSY_Msk)) {}
        if (tout == 0UL)
        {
            g_RMC_i32ErrCode = -1L;
            return 0xFFFFFFFFUL;
        }

        if (RMC->ISPSTS & RMC_ISPSTS_ISPFF_Msk)
        {
            RMC->ISPSTS |= RMC_ISPSTS_ISPFF_Msk;
            g_RMC_i32ErrCode = -1L;
            return 0xFFFFFFFFUL;
        }
        ret = RMC->ISPDAT;
    }

    return ret;
}


/**
  * @brief Run flash all one verification and get result.
  * @param[in] u32addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of 512 bytes.
  * @retval   READ_ALLONE_YES       The contents of verified flash area are 0xFFFFFFFF.
  * @retval   READ_ALLONE_NOT       Some contents of verified flash area are not 0xFFFFFFFF.
  * @retval   READ_ALLONE_CMD_FAIL  Unexpected error occurred.
  * @note     Global error code g_RMC_i32ErrCode
  *           -1  RUN_ALL_ONE or CHECK_ALL_ONE commands time-out
  */
uint32_t  RMC_CheckAllOne(uint32_t u32addr, uint32_t u32count)
{
    uint32_t  ret = READ_ALLONE_CMD_FAIL;
    int32_t   i32TimeOutCnt0;

    /* Workaround solution: Check ISPADDR to know if wakeup from power-down mode.
       If Magic Number exists, call Read CID command to avoid issue 2.5 (Please refer to Errata Sheet)
     */
    if(RMC_CHECK_MAGICNUM() != 0UL)
    {
        (void)RMC_DummyReadCID();
    }
    g_RMC_i32ErrCode = 0L;

    RMC->ISPSTS = 0x80UL;   /* clear check all one bit */

    RMC->ISPCMD   = RMC_ISPCMD_RUN_ALL1;
    RMC->ISPADDR  = u32addr;
    RMC->ISPDAT   = u32count;
    RMC->ISPTRG   = RMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt0 = RMC_TIMEOUT_CHKALLONE;
    while(RMC->ISPSTS & RMC_ISPSTS_ISPBUSY_Msk)
    {
        if( i32TimeOutCnt0-- <= 0L)
        {
            g_RMC_i32ErrCode = -1L;
            break;
        }
    }

    if(g_RMC_i32ErrCode == 0L)
    {
        int32_t   i32TimeOutCnt1 = RMC_TIMEOUT_CHKALLONE;
        do
        {
            RMC->ISPCMD = RMC_ISPCMD_READ_ALL1;
            RMC->ISPADDR = u32addr;
            RMC->ISPTRG = RMC_ISPTRG_ISPGO_Msk;

            i32TimeOutCnt0 = RMC_TIMEOUT_CHKALLONE;
            while(RMC->ISPSTS & RMC_ISPSTS_ISPBUSY_Msk)
            {
                if( i32TimeOutCnt0-- <= 0L)
                {
                    g_RMC_i32ErrCode = -1L;
                    break;
                }
            }

            if( i32TimeOutCnt1-- <= 0L)
            {
                g_RMC_i32ErrCode = -1L;
            }
        }
        while( (RMC->ISPDAT == 0UL) && (g_RMC_i32ErrCode == 0L) );
    }

    if( g_RMC_i32ErrCode == 0L )
    {
        if(RMC->ISPDAT == READ_ALLONE_YES)
        {
            ret = READ_ALLONE_YES;
        }
        else if(RMC->ISPDAT == READ_ALLONE_NOT)
        {
            ret = READ_ALLONE_NOT;
        }
        else
        {
            g_RMC_i32ErrCode = -1L;
            ret = READ_ALLONE_CMD_FAIL;
        }
    }

    return ret;
}

/**
  * @brief     Remap Bank0/Bank1
  *
  * @param[in] u32Bank    Bank Num which will remap to.
  *
  * @retval    0   Success
  * @retval    -1  Program failed.
  *
  * @details  Remap Bank0/Bank1
  *
  * @note     Global error code g_FMC_i32ErrCode
  *           -1  Program failed or time-out
  */
int32_t RMC_RemapBank(uint32_t u32BankAddr)
{
    int32_t  ret = 0;
    uint32_t  tout;
    /* Workaround solution: Check ISPADDR to know if wakeup from power-down mode.
       If Magic Number exists, call Read CID command to avoid issue 2.5 (Please refer to Errata Sheet)
     */
    if(RMC_CHECK_MAGICNUM() != 0UL)
    {
        (void)RMC_DummyReadCID();
    }
    g_RMC_i32ErrCode = 0L;
    RMC->ISPCMD = RMC_ISPCMD_BANK_REMAP;
    RMC->ISPADDR = u32BankAddr;
    RMC->ISPDAT = 0x5AA55AA5UL;
    RMC->ISPTRG = RMC_ISPTRG_ISPGO_Msk;
    tout = RMC_TIMEOUT_WRITE;

    while ((--tout > 0UL) && (RMC->ISPTRG & RMC_ISPTRG_ISPGO_Msk)) {}

    if (tout == 0UL)
    {
        g_RMC_i32ErrCode = -1L;
        return -1L;
    }

    if (RMC->ISPSTS & RMC_ISPSTS_ISPFF_Msk)
    {
        RMC->ISPSTS |= RMC_ISPSTS_ISPFF_Msk;
        g_RMC_i32ErrCode = -1L;
        return -1L;
    }

    return ret;
}

/**
  * @brief  Read the 64-bits data from the specified OTP.
  * @param[in] otp_num    The OTP number.
  * @param[in] low_word   Low word of the 64-bits data.
  * @param[in] high_word   Low word of the 64-bits data.
  * @retval   0   Success
  * @retval   -1  Read failed.
  * @retval   -2  Invalid OTP number.
  */
int32_t RMC_ReadOTP(uint32_t otp_num, uint32_t *low_word, uint32_t *high_word)
{
    int32_t  ret = 0L;

    /* Workaround solution: Check ISPADDR to know if wakeup from power-down mode.
       If Magic Number exists, call Read CID command to avoid issue 2.5 (Please refer to Errata Sheet)
     */
    if(RMC_CHECK_MAGICNUM() != 0UL)
    {
        (void)RMC_DummyReadCID();
    }
    if (otp_num > 255UL)
    {
        ret = -2L;
    }

    if (ret == 0L)
    {
        *low_word  = RMC_Read(RMC_OTP_BASE + (otp_num * 8UL));
        if(g_RMC_i32ErrCode == 0L)
        {
            *high_word = RMC_Read(RMC_OTP_BASE + (otp_num * 8UL) + 4UL);
        }
    }
    return ret;
}

/**
  * @brief  Lock the specified OTP.
  * @param[in] otp_num    The OTP number.
  * @retval   0   Success
  * @retval   -1  Failed to write OTP lock bits.
  * @retval   -2  Invalid OTP number.
  */
int32_t RMC_LockOTP(uint32_t otp_num)
{
    int32_t ret = 0L;

    /* Workaround solution: Check ISPADDR to know if wakeup from power-down mode */
    if (RMC_CHECK_MAGICNUM() != 0UL)
    {
        (void)RMC_DummyReadCID();
    }

    if (otp_num > 255UL)
    {
        ret = -2L;
    }

    if (ret == 0L)
    {
        int32_t i32WriteRet;
        i32WriteRet = RMC_Write((RMC_OTP_BASE + 0x800UL + (otp_num * 4UL)), 0UL);
        if (i32WriteRet != 0L)
        {
            ret = -1L;
        }
    }
    return ret;
}

/**
  * @brief  Check the OTP is locked or not.
  * @param[in] otp_num    The OTP number.
  * @retval   1   OTP is locked.
  * @retval   0   OTP is not locked.
  * @retval   -1  Failed to read OTP lock bits.
  * @retval   -2  Invalid OTP number.
  */
int32_t RMC_IsOTPLocked(uint32_t otp_num)
{
    int32_t  ret = 0L;

    /* Workaround solution: Check ISPADDR to know if wakeup from power-down mode.
       If Magic Number exists, call Read CID command to avoid issue 2.5 (Please refer to Errata Sheet)
     */
    if(RMC_CHECK_MAGICNUM() != 0UL)
    {
        (void)RMC_DummyReadCID();
    }
    if (otp_num > 255UL)
    {
        ret = -2L;
    }

    if (ret == 0L)
    {
        uint32_t  u32data = RMC_Read(RMC_OTP_BASE + 0x800UL + (otp_num * 4UL));
        if (u32data != 0xFFFFFFFFUL)
        {
            ret = 1L;   /* Lock work was progrmmed. OTP was locked. */
        }
    }
    return ret;
}

/**
  * @brief Program a 64-bits data to the specified OTP.
  * @param[in] otp_num    The OTP number.
  * @param[in] low_word   Low word of the 64-bits data.
  * @param[in] high_word   Low word of the 64-bits data.
  * @retval   0   Success
  * @retval   -1  Program failed.
  * @retval   -2  Invalid OTP number.
  */
int32_t RMC_WriteOTP(uint32_t otp_num, uint32_t low_word, uint32_t high_word)
{
    int32_t ret = 0L;
    int32_t i32WriteRet;

    /* Workaround solution: Check ISPADDR to know if wakeup from power-down mode.
       If Magic Number exists, call Read CID command to avoid issue 2.5 (Please refer to Errata Sheet)
     */
    if (RMC_CHECK_MAGICNUM() != 0UL)
    {
        (void)RMC_DummyReadCID();
    }

    if (otp_num > 255UL)
    {
        ret = -2L;
    }

    if (ret == 0L)
    {
        i32WriteRet = RMC_Write((RMC_OTP_BASE + (otp_num * 8UL)), low_word);
        if (i32WriteRet != 0L)
        {
            ret = -1L;
        }
    }

    if ((ret == 0L) && (g_RMC_i32ErrCode == 0L))
    {
        i32WriteRet = RMC_Write((RMC_OTP_BASE + (otp_num * 8UL) + 4UL), high_word);
        if (i32WriteRet != 0L)
        {
            ret = -1L;
        }
    }

    return ret;
}


/*@}*/ /* end of group RMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group RMC_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/


