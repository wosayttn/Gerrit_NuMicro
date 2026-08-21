/**************************************************************************//**
 * @file     dfmc.c
 * @version  V1.00
 * @brief    M471 series DFMC driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
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

int32_t  g_DFMC_i32ErrCode;

/**
  * @brief Disable DFMC ISP function.
  * @return None
  */
void DFMC_Close(void)
{
    DFMC->ISPCTL &= ~DFMC_ISPCTL_ISPEN_Msk;
}

/**
  * @brief Execute DFMC_ISPCMD_PAGE_ERASE command to erase a flash page. The page size is 256 bytes.
  * @param[in]  u32PageAddr Address of the flash page to be erased.
  *             It must be a 256 bytes aligned address.
  * @return ISP page erase success or not.
  * @retval    0  Success
  * @retval   -1  Erase failed
  *
  * @note     Global error code g_DFMC_i32ErrCode
  *           -1  Erase failed or erase time-out  
  */
int32_t DFMC_Erase(uint32_t u32PageAddr)
{
    uint32_t  tout;
    
    g_DFMC_i32ErrCode = 0;
	
    DFMC->ISPCMD = DFMC_ISPCMD_PAGE_ERASE;
    DFMC->ISPADDR = u32PageAddr;
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;

    tout = DFMC_TIMEOUT_ERASE;    
	
    while ((--tout > 0) && (DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk)) { }
    
    if (tout == 0)
    {
        g_DFMC_i32ErrCode = -1;
        return -1;
    }
    
    if (DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk)
    {
        DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk;
        g_DFMC_i32ErrCode = -1;
        return -1;
    }
    return 0;
}

/**
  * @brief Execute DFMC_ISPCMD_PAGE_ERASE command to erase a flash page with non-blocking mode. The page size is 256 bytes.
  * @param[in]  u32PageAddr Address of the flash page to be erased.
  *             It must be a 256 bytes aligned address.
  * @return ISP page erase success or not.
  * @retval    0  Success
  * @retval   -1  Erase failed
  */
void DFMC_Erase_NonBlocking(uint32_t u32PageAddr)
{
    DFMC->ISPCMD = DFMC_ISPCMD_PAGE_ERASE;
    DFMC->ISPADDR = u32PageAddr;
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;
}


/**
  * @brief Execute DFMC_ISPCMD_MASS_ERASE command to erase entire data flash.
  * @return ISP mass erase success or not.
  * @retval    0  Success
  * @retval   -1  Erase failed
  *
  * @note     Global error code g_DFMC_i32ErrCode
  *           -1  Erase failed or erase time-out
  */
int32_t DFMC_Mass_Erase(void)
{
    uint32_t  tout;
	
    g_DFMC_i32ErrCode = 0;
    
    DFMC->ISPCMD = DFMC_ISPCMD_MASS_ERASE;
    DFMC->ISPADDR = DFMC_DFLASH_BASE;
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;

    tout = DFMC_TIMEOUT_MERASE;    
	
    while ((--tout > 0) && (DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk)) { }
    if (tout == 0)
    {
        g_DFMC_i32ErrCode = -1;
        return -1;
    }

    if (DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk)
    {
        DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk;
        g_DFMC_i32ErrCode = -1;
        return -1;
    }
    return 0;
}

/**
  * @brief Execute DFMC_ISPCMD_MASS_ERASE command to erase a entire data flash with non-blocking mode.
  * @return None
  */
void DFMC_Mass_Erase_NonBlocking(void)
{
    DFMC->ISPCMD = DFMC_ISPCMD_MASS_ERASE;
    DFMC->ISPADDR = DFMC_DFLASH_BASE;
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;
}

/**
  * @brief Enable DFMC ISP function
  * @return None
  */
void DFMC_Open(void)
{
    DFMC->ISPCTL |=  DFMC_ISPCTL_ISPEN_Msk;
}


/**
  * @brief Execute DFMC_ISPCMD_READ command to read a word from flash.
  * @param[in]  u32Addr Address of the flash location to be read.
  *             It must be a word aligned address.
  * @return The word data read from specified flash address.
  *          Return 0xFFFFFFFF if read failed.
  *
  * @note     Global error code g_DFMC_i32ErrCode
  *           -1  Read time-out
  */
uint32_t DFMC_Read(uint32_t u32Addr)
{
    uint32_t  tout;

    g_DFMC_i32ErrCode = 0;
    DFMC->ISPCMD = DFMC_ISPCMD_READ;
    DFMC->ISPADDR = u32Addr;
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;
	
    tout = DFMC_TIMEOUT_READ;    
	
    while ((--tout > 0) && (DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk)) { }
    if (tout == 0)
    {
        g_DFMC_i32ErrCode = -1;
        return 0xFFFFFFFF;
    }
    return DFMC->ISPDAT;
}

/**
  * @brief Execute ISP DFMC_ISPCMD_PROGRAM to program a word to flash.
  * @param[in]  u32Addr Address of the flash location to be programmed.
  *             It must be a word aligned address.
  * @param[in]  u32Data The word data to be programmed.
  * @return   0   Success
  * @return   -1  Program Failed
  *
  * @note     Global error code g_DFMC_i32ErrCode
  *           -1  Program failed or time-out  
  */
int32_t DFMC_Write(uint32_t u32Addr, uint32_t u32Data)
{
    uint32_t  tout;

    g_DFMC_i32ErrCode = 0;
    DFMC->ISPCMD = DFMC_ISPCMD_PROGRAM;
    DFMC->ISPADDR = u32Addr;
    DFMC->ISPDAT = u32Data;
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;
	
    tout = DFMC_TIMEOUT_WRITE;    
	
    while ((--tout > 0) && (DFMC->ISPTRG & DFMC_ISPTRG_ISPGO_Msk)) { }
		
    if (tout == 0)
    {
        g_DFMC_i32ErrCode = -1;
        return -1;
    }
    if (DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk)
    {
        DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk;
        g_DFMC_i32ErrCode = -1;
        return -1;
    }
    return 0;
}
    
/**
  * @brief Execute ISP DFMC_ISPCMD_PROGRAM to program a word to flash in non-blocking mode.
  * @param[in]  u32Addr Address of the flash location to be programmed.
  *             It must be a word aligned address.
  * @param[in]  u32Data The word data to be programmed.
  * @return None
  */
void DFMC_Write_NonBlocking(uint32_t u32Addr, uint32_t u32Data)
{
    DFMC->ISPCMD = DFMC_ISPCMD_PROGRAM;
    DFMC->ISPADDR = u32Addr;
    DFMC->ISPDAT = u32Data;
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;
}

/**
  * @brief Run CRC32 checksum calculation and get result.
  * @param[in] u32addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of 256 bytes.
  * @return Success or not.
  * @retval   0           Success.
  * @retval   0xFFFFFFFF  Invalid parameter or command failed.
  *
  * @note     Global error code g_DFMC_i32ErrCode
  *           -1  Run/Read check sum time-out failed
  *           -2  u32addr or u32count must be aligned with 512
  */
int32_t  DFMC_GetChkSum(uint32_t u32addr, uint32_t u32count)
{
    uint32_t  tout;

    if ((u32addr % 256UL) || (u32count % 256UL))
    {
        g_DFMC_i32ErrCode = -2;
        return 0xFFFFFFFF;
    }
    DFMC->ISPCMD  = DFMC_ISPCMD_RUN_CKS;
    DFMC->ISPADDR = u32addr;
    DFMC->ISPDAT  = u32count;
    DFMC->ISPTRG  = DFMC_ISPTRG_ISPGO_Msk;

    tout = DFMC_TIMEOUT_CHKSUM;
		
    while ((--tout > 0) && (DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk)) { }
		
    if (tout == 0)
    {
        g_DFMC_i32ErrCode = -1;
        return 0xFFFFFFFF;
    }

    if (DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk)
    {
        DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk;
        g_DFMC_i32ErrCode = -2;
        return 0xFFFFFFFF;
    }

    DFMC->ISPCMD = DFMC_ISPCMD_READ_CKS;
    DFMC->ISPADDR    = u32addr;
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_CHKSUM;
		
    while ((--tout > 0) && (DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk)) { }
		
    if (tout == 0)
    {
        g_DFMC_i32ErrCode = -1;
        return 0xFFFFFFFF;
    }
    if (DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk)
    {
        DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk;
        g_DFMC_i32ErrCode = -2;
        return 0xFFFFFFFF;
    }
    return DFMC->ISPDAT;
}


/**
  * @brief Run flash all one verification and get result.
  * @param[in] u32addr   Starting flash address. It must be a page aligned address.
  * @param[in] u32count  Byte count of flash to be calculated. It must be multiple of 256 bytes.
  * @retval   DREAD_ALLONE_YES       The contents of verified flash area are 0xFFFFFFFF.
  * @retval   DREAD_ALLONE_NOT       Some contents of verified flash area are not 0xFFFFFFFF.
  * @retval   DREAD_ALLONE_CMD_FAIL  Unexpected error occurred.
  *
  * @note     Global error code g_DFMC_i32ErrCode
  *           -1  RUN_ALL_ONE or CHECK_ALL_ONE commands time-out  
  */
int32_t  DFMC_CheckAllOne(uint32_t u32addr, uint32_t u32count)
{
    uint32_t  tout;

    g_DFMC_i32ErrCode = READ_ALLONE_CMD_FAIL;
	
    DFMC->ISPSTS = DFMC_ISPSTS_ALLONE_Msk;   /* clear check all one bit */

    DFMC->ISPCMD   = DFMC_ISPCMD_RUN_ALL1;
    DFMC->ISPADDR  = u32addr;
    DFMC->ISPDAT   = u32count;
    DFMC->ISPTRG   = DFMC_ISPTRG_ISPGO_Msk;

    tout = FMC_TIMEOUT_CHKALLONE;
	
    while ((--tout > 0) && (DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk)) { }

    if (tout == 0)
        return -1;

    if (DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk)
    {
        DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk;
        return -1;
    }

    tout = DFMC_TIMEOUT_CHKALLONE;
    do
    {
        DFMC->ISPCMD = DFMC_ISPCMD_READ_ALL1;
        DFMC->ISPADDR    = u32addr;
        DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;
			
        while ((--tout > 0) && (DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk)) { }
				
        if (tout == 0)
            return -1;
    }
    while (DFMC->ISPDAT == 0UL);

    if (DFMC->ISPCTL & DFMC_ISPCTL_ISPFF_Msk)
    {
        DFMC->ISPCTL |= DFMC_ISPCTL_ISPFF_Msk;
        return -1;
    }

    if ((DFMC->ISPDAT == DREAD_ALLONE_YES) || (DFMC->ISPDAT == DREAD_ALLONE_NOT))
    {
			  g_DFMC_i32ErrCode = 0;
        return DFMC->ISPDAT;
    }
    else
    {
        g_DFMC_i32ErrCode = READ_ALLONE_CMD_FAIL;
        return -1;
    }
}

/*@}*/ /* end of group DFMC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group DFMC_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/


