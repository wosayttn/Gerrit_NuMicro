/**************************************************************************//**
 * @file     sys.c
 * @version  V0.10
 * @brief    M2003J series System Manager (SYS) driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SYS_Driver SYS Driver
  @{
*/


/** @addtogroup SYS_EXPORTED_FUNCTIONS SYS Exported Functions
  @{
*/

/**
  * @brief      Clear reset source
  * @param[in]  u32Src is system reset source. Including :
  *             - \ref SYS_RSTSTS_CPULKRF_Msk
  *             - \ref SYS_RSTSTS_CPURF_Msk
  *             - \ref SYS_RSTSTS_SYSRF_Msk
  *             - \ref SYS_RSTSTS_BODRF_Msk
  *             - \ref SYS_RSTSTS_LVRF_Msk
  *             - \ref SYS_RSTSTS_WDTRF_Msk
  *             - \ref SYS_RSTSTS_PINRF_Msk
  *             - \ref SYS_RSTSTS_PORF_Msk
  * @return     None
  * @details    This function clear the selected system reset source.
  */
void SYS_ClearResetSrc(uint32_t u32Src)
{
    SYS->RSTSTS = u32Src;
}

/**
  * @brief      Get Brown-out detector output status
  * @param      None
  * @retval     0 System voltage is higher than BODVL setting or BODEN is 0.
  * @retval     1 System voltage is lower than BODVL setting.
  * @details    This function get Brown-out detector output status.
  */
uint32_t SYS_GetBODStatus(void)
{
    return ((SYS->BODSTS & SYS_BODSTS_BODOUT_Msk) >> SYS_BODSTS_BODOUT_Pos);
}

/**
  * @brief      Get reset status register value
  * @param      None
  * @return     Reset source
  * @details    This function get the system reset status register value.
  */
uint32_t SYS_GetResetSrc(void)
{
    return (SYS->RSTSTS);
}

/**
  * @brief      Check if register is locked or not
  * @param      None
  * @retval     0 Write-protection function is disabled.
  *             1 Write-protection function is enabled.
  * @details    This function check register write-protection bit setting.
  */
uint32_t SYS_IsRegLocked(void)
{
    return (SYS->REGLCTL & BIT0 ? 0UL : 1UL);
}

/**
  * @brief      Get product ID
  * @param      None
  * @return     Product ID
  * @details    This function get product ID.
  */
uint32_t  SYS_ReadPDID(void)
{
    return (SYS->PDID);
}

/**
  * @brief      Reset chip with chip reset
  * @param      None
  * @return     None
  * @details    This function reset chip with chip reset.
  *             The register write-protection function should be disabled before using this function.
  */
void SYS_ResetChip(void)
{
    SYS->RSTCTL |= SYS_RSTCTL_CHIPRST_Msk;
}

/**
  * @brief      Reset chip with CPU reset
  * @param      None
  * @return     None
  * @details    This function reset CPU with CPU reset.
  *             The register write-protection function should be disabled before using this function.
  */
void SYS_ResetCPU(void)
{
    SYS->RSTCTL |= SYS_RSTCTL_CPURST_Msk;
}

/**
  * @brief      Reset selected module
  * @param[in]  u32ModuleIndex is module index. Including :
  *             - \ref ADC0_RST
  *             - \ref BPWM0_RST
  *             - \ref BPWM1_RST
  *             - \ref CRC0_RST
  *             - \ref FMC0_RST
  *             - \ref DFMC0_RST
  *             - \ref GPIO0_RST
  *             - \ref I2C0_RST
  *             - \ref I2C1_RST
  *             - \ref I2C2_RST
  *             - \ref PDMA0_RST
  *             - \ref RTC0_RST
  *             - \ref TMR0_RST
  *             - \ref TMR1_RST
  *             - \ref TMR2_RST
  *             - \ref TMR3_RST
  *             - \ref TMR4_RST
  *             - \ref TMR5_RST
  *             - \ref TMR6_RST
  *             - \ref TMR7_RST
  *             - \ref TMR8_RST
  *             - \ref UART0_RST
  *             - \ref UART1_RST
  *             - \ref UART2_RST
  *             - \ref UART3_RST
  *             - \ref UART4_RST
  *             - \ref USCI0_RST
  *             - \ref USCI1_RST
  *             - \ref USCI2_RST
  *             - \ref USCI3_RST
  *             - \ref USCI4_RST
  *             - \ref WWDT0_RST
  * @return     None
  * @details    This function reset selected module.
  */
void SYS_ResetModule(uint32_t u32ModuleIndex)
{
    /* Generate reset signal to the corresponding module */
    *(volatile uint32_t *)((uint32_t)SYS_BASE + (u32ModuleIndex >> 20UL))  |=  (1UL << (u32ModuleIndex & 0x000FFFFFUL));

    /* Release corresponding module from reset state */
    *(volatile uint32_t *)((uint32_t)SYS_BASE + (u32ModuleIndex >> 20UL))  &= ~(1UL << (u32ModuleIndex & 0x000FFFFFUL));
}

/**
  * @brief      Enable and configure Brown-out detector function
  * @param[in]  i32Mode is reset or interrupt mode. Including :
  *             - \ref SYS_BODCTL_BOD_RST_EN
  *             - \ref SYS_BODCTL_BOD_INTERRUPT_EN
  * @param[in]  u32BODLevel is Brown-out voltage level. Including :
  *             - \ref SYS_BODCTL_BODVL_2_4V
  *             - \ref SYS_BODCTL_BODVL_2_7V
  *             - \ref SYS_BODCTL_BODVL_3_7V
  *             - \ref SYS_BODCTL_BODVL_4_4V
  * @retval     SYS_OK          SYS operation OK.
  * @retval     SYS_ERR_TIMEOUT SYS operation abort due to timeout error.
  * @details    This function configure Brown-out detector reset or interrupt mode, enable Brown-out function and set Brown-out voltage level.
  *             The register write-protection function should be disabled before using this function.
  */
int32_t SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel)
{
    SYS_WAIT_BODCTL_WRBUSY();

    if ((SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk) == 0)
    {
        /* Enable Brown-out Detector function */
        /* Enable Brown-out interrupt or reset function */
        /* Select Brown-out Detector threshold voltage */
        SYS->BODCTL = (SYS->BODCTL & ~(SYS_BODCTL_BODRSTEN_Msk | SYS_BODCTL_BODVL_Msk)) |
                      ((uint32_t)i32Mode) | (u32BODLevel) | (SYS_BODCTL_BODEN_Msk);
    }
    else
    {
        return SYS_ERR_TIMEOUT;
    }

    return SYS_OK;
}

/**
  * @brief      Disable Brown-out detector function
  * @param      None
  * @retval     SYS_OK          SYS operation OK.
  * @retval     SYS_ERR_TIMEOUT SYS operation abort due to timeout error.
  * @details    This function disable Brown-out detector function.
  *             The register write-protection function should be disabled before using this function.
  */
int32_t SYS_DisableBOD(void)
{
    SYS_WAIT_BODCTL_WRBUSY();

    if ((SYS->BODCTL & SYS_BODCTL_WRBUSY_Msk) == 0)
    {
        SYS->BODCTL &= ~SYS_BODCTL_BODEN_Msk;
    }
    else
    {
        return SYS_ERR_TIMEOUT;
    }

    return SYS_OK;
}

/**
  * @brief      Enable SRAM ECC function
  * @param[in]  u32Bank is the index of SRAM bank.
  * @retval     None
  * @details    This function enable SRAM ECC function.
  *             The register write-protection function should be disabled before using this function.
  */
void SYS_SRAM_ECC_Enable(uint32_t u32Bank)
{
    if (u32Bank == 1)
    {
        SYS->SRAM1ICTL |= SYS_SRAM1ICTL_ECCEN_Msk;
    }
    else
    {
        SYS->SRAM0ICTL |= SYS_SRAM0ICTL_ECCEN_Msk;
    }
}

/**
  * @brief      Disable SRAM ECC function
  * @param[in]  u32Bank is the index of SRAM bank.
  * @retval     None
  * @details    This function disable SRAM ECC function.
  *             The register write-protection function should be disabled before using this function.
  */
void SYS_SRAM_ECC_Disable(uint32_t u32Bank)
{
    if (u32Bank == 1)
    {
        SYS->SRAM1ICTL &= ~SYS_SRAM1ICTL_ECCEN_Msk;
    }
    else
    {
        SYS->SRAM0ICTL &= ~SYS_SRAM0ICTL_ECCEN_Msk;
    }
}

/**
  * @brief      Enable SRAM ECC interrupt function
  * @param[in]  u32Bank is the index of SRAM bank.
  * @param[in]  u32Mask is the bit mask of SRAM ECC interrupt enable bit. Including :
  *                 - \ref SYS_SARM_ECC_SB_IEN
  *                 - \ref SYS_SARM_ECC_MB_IEN
  * @retval     None
  * @details    This function enable the SRAM ECC interrupt function.
  */
void SYS_SRAM_ECC_EnableInt(uint32_t u32Bank, uint32_t u32Mask)
{
    if (u32Bank == 1)
    {
        SYS->SRAM1ICTL |= u32Mask;
    }
    else
    {
        SYS->SRAM0ICTL |= u32Mask;
    }
}

/**
  * @brief      Disable SRAM ECC interrupt function
  * @param[in]  u32Bank is the index of SRAM bank.
  * @param[in]  u32Mask is the bit mask of SRAM ECC interrupt enable bit. Including :
  *                 - \ref SYS_SARM_ECC_SB_IEN
  *                 - \ref SYS_SARM_ECC_MB_IEN
  * @retval     None
  * @details    This function disable the SRAM ECC interrupt function.
  */
void SYS_SRAM_ECC_DisableInt(uint32_t u32Bank, uint32_t u32Mask)
{
    if (u32Bank == 1)
    {
        SYS->SRAM1ICTL &= ~(u32Mask);
    }
    else
    {
        SYS->SRAM0ICTL &= ~(u32Mask);
    }
}

/**
  * @brief      Clear SRAM ECC interrupt flag
  * @param[in]  u32Bank is the index of SRAM bank.
  * @param[in]  u32Mask is the bit mask of SRAM ECC status flag. Including :
  *                 - \ref SYS_SARM_ECC_SB_IF
  *                 - \ref SYS_SARM_ECC_MB_IF
  *                 - \ref SYS_SARM_ECC_B0_IF
  *                 - \ref SYS_SARM_ECC_B1_IF
  *                 - \ref SYS_SARM_ECC_B2_IF
  *                 - \ref SYS_SARM_ECC_B3_IF
  * @retval     None
  * @details    This function clear the SRAM ECC interrupt flag.
  */
void SYS_SRAM_ECC_ClearIntFlag(uint32_t u32Bank, uint32_t u32Mask)
{
    if (u32Bank == 1)
    {
        SYS->SRAM1STS |= u32Mask;
    }
    else
    {
        SYS->SRAM0STS |= u32Mask;
    }
}

/**@}*/ /* end of group SYS_EXPORTED_FUNCTIONS */
/**@}*/ /* end of group SYS_Driver */
/**@}*/ /* end of group Standard_Driver */
