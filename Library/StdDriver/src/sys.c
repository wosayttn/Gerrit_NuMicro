/**************************************************************************//**
 * @file     sys.c
 * @version  V1.00
 * @brief    M2U51 series SYS driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NuMicro.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
    @{
*/

/** @addtogroup SYS_Driver SYS Driver
    @{
*/

int32_t g_SYS_i32ErrCode = 0;   /*!< SYS global error code */

/** @addtogroup SYS_EXPORTED_FUNCTIONS SYS Exported Functions
    @{
*/

/**
  * @brief      Clear reset source
  *
  * @param[in]  u32Src is system reset source. Including :
  *             - \ref SYS_RSTSTS_CPULKRF_Msk
  *             - \ref SYS_RSTSTS_CPURF_Msk
  *             - \ref SYS_RSTSTS_SYSRF_Msk
  *             - \ref SYS_RSTSTS_BODRF_Msk
  *             - \ref SYS_RSTSTS_LVRF_Msk
  *             - \ref SYS_RSTSTS_WDTRF_Msk
  *             - \ref SYS_RSTSTS_PINRF_Msk
  *             - \ref SYS_RSTSTS_PORF_Msk
  *
  * @return     None
  *
  * @details    This function clear the selected system reset source.
  */
void SYS_ClearResetSrc(uint32_t u32Src)
{
    SYS->RSTSTS |= u32Src;
}

/**
  * @brief      Get Brown-out detector output status
  *
  * @param      None
  *
  * @retval     0 System voltage is higher than BODVL setting or BODEN is 0.
  * @retval     1 System voltage is lower than BODVL setting.
  *
  * @details    This function get Brown-out detector output status.
  */
uint32_t SYS_GetBODStatus(void)
{
    return ((SYS->BODCTL & SYS_BODCTL_BODOUT_Msk) >> SYS_BODCTL_BODOUT_Pos);
}

/**
  * @brief      Get reset status register value
  *
  * @param      None
  *
  * @return     Reset source
  *
  * @details    This function get the system reset status register value.
  */
uint32_t SYS_GetResetSrc(void)
{
    return (SYS->RSTSTS);
}

/**
  * @brief      Check if register is locked nor not
  *
  * @param      None
  *
  * @retval     0 Write-protection function is disabled.
  *             1 Write-protection function is enabled.
  *
  * @details    This function check register write-protection bit setting.
  */
uint32_t SYS_IsRegLocked(void)
{
    return ((SYS->REGLCTL & BIT0) ? 0UL : 1UL);
}

/**
  * @brief      Get product ID
  *
  * @param      None
  *
  * @return     Product ID
  *
  * @details    This function get product ID.
  */
uint32_t  SYS_ReadPDID(void)
{
    return (SYS->PDID);
}

/**
  * @brief      Reset chip with chip reset
  *
  * @param      None
  *
  * @return     None
  *
  * @details    This function reset chip with chip reset.
  *             The register write-protection function should be disabled before using this function.
  */
void SYS_ResetChip(void)
{
    SYS->IPRST0 |= SYS_IPRST0_CHIPRST_Msk;
}

/**
  * @brief      Reset chip with CPU reset
  *
  * @param      None
  *
  * @return     None
  *
  * @details    This function reset CPU with CPU reset.
  *             The register write-protection function should be disabled before using this function.
  */
void SYS_ResetCPU(void)
{
    SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
}

/**
  * @brief      Reset selected module
  *
  * @param[in]  u32ModuleIndex is module index. Including :
  *             - \ref CHIP_RST
  *             - \ref CPU_RST
  *             - \ref PDMA0_RST
  *             - \ref CRC_RST
  *             - \ref CRPT_RST
  *             - \ref GPIO_RST
  *             - \ref TMR0_RST
  *             - \ref TMR1_RST
  *             - \ref TMR2_RST
  *             - \ref TMR3_RST
  *             - \ref ACMP01_RST
  *             - \ref I2C0_RST
  *             - \ref I2C1_RST
  *             - \ref I2C2_RST
  *             - \ref SPI0_RST
  *             - \ref SPI1_RST
  *             - \ref SPI2_RST
  *             - \ref UART0_RST
  *             - \ref UART1_RST
  *             - \ref UART2_RST
  *             - \ref USCI0_RST
  *             - \ref WWDT_RST
  *             - \ref PWM0_RST
  *             - \ref BPWM0_RST
  *             - \ref LCD_RST
  *             - \ref ADC0_RST
  *
  * @return     None
  *
  * @details    This function reset selected module.
  */
void SYS_ResetModule(uint32_t u32ModuleIndex)
{
    uint32_t u32tmpVal = 0UL, u32tmpAddr = 0UL;

    /* Support the IP RESET register NEAR SYS_IPRST0 */
    u32tmpAddr = (uint32_t)&SYS->IPRST0 + ((u32ModuleIndex >> 24UL));

    /* Generate reset signal to the corresponding module */
    u32tmpVal = (BIT0 << (u32ModuleIndex & 0x00FFFFFFUL));
    *(uint32_t *)u32tmpAddr |= u32tmpVal;

    /* Release corresponding module from reset state */
    u32tmpVal = ~(BIT0 << (u32ModuleIndex & 0x00FFFFFFUL));
    *(uint32_t *)u32tmpAddr &= u32tmpVal;
}

/**
  * @brief      Enable and configure Brown-out detector function
  *
  * @param[in]  i32Mode is reset or interrupt mode. Including :
  *             - \ref SYS_BODCTL_BOD_RST_EN
  *             - \ref SYS_BODCTL_BOD_INTERRUPT_EN
  * @param[in]  u32BODLevel is Brown-out voltage level. Including :
  *             - \ref SYS_BODCTL_BODVL_4_4V
  *             - \ref SYS_BODCTL_BODVL_3_7V
  *             - \ref SYS_BODCTL_BODVL_3_0V
  *             - \ref SYS_BODCTL_BODVL_2_7V
  *             - \ref SYS_BODCTL_BODVL_2_4V
  *             - \ref SYS_BODCTL_BODVL_2_0V
  *             - \ref SYS_BODCTL_BODVL_1_8V
  *
  * @return     None
  *
  * @details    This function configure Brown-out detector reset or interrupt mode, enable Brown-out function and set Brown-out voltage level.
  *             The register write-protection function should be disabled before using this function.
  */
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel)
{
    /* Enable Brown-out Detector function */
    SYS->BODCTL |= SYS_BODCTL_BODEN_Msk;

    /* Enable Brown-out interrupt or reset function */
    SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODRSTEN_Msk) | (uint32_t)i32Mode;

    /* Select Brown-out Detector threshold voltage */
    SYS->BODCTL = (SYS->BODCTL & ~SYS_BODCTL_BODVL_Msk) | u32BODLevel;
}

/**
  * @brief      Disable Brown-out detector function
  *
  * @param      None
  *
  * @return     None
  *
  * @details    This function disable Brown-out detector function.
  *             The register write-protection function should be disabled before using this function.
  */
void SYS_DisableBOD(void)
{
    SYS->BODCTL &= ~(SYS_BODCTL_BODEN_Msk);
}

/**
  * @brief      Set Main Voltage Regulator Type
  *
  * @param[in]  u32PowerRegulator is main voltage regulator type. Including :
  *             - \ref SYS_PLCTL_MVRS_LDO
  *             - \ref SYS_PLCTL_MVRS_DCDC
  *
  * @retval     0  main voltage regulator type setting is not finished
  * @retval     1  main voltage regulator type setting is finished
  *
  * @details    This function set main voltage regulator type.
  *             The main voltage regulator type setting to DCDC cannot finished if the inductor is not detected.
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t SYS_SetPowerRegulator(uint32_t u32PowerRegulator)
{
    int32_t i32TimeOutCnt = 400;
    uint32_t u32Ret = 1UL;
    uint32_t u32PowerRegStatus;
    uint32_t u32BGPowerRegStatus;
    uint32_t u32Delay;

    /* Get Band-gap power mode status */
    u32BGPowerRegStatus = (CLK->PMUCTL & CLK_PMUCTL_NRBGLPEL_Msk);

    /* Must set Band-gap power mode to normal mode before change regulator type */
    CLK->PMUCTL = (CLK->PMUCTL & ~(CLK_PMUCTL_NRBGLPEL_Msk)) | CLK_PMUCTL_NRBGLPEL_NORMAL;
    /* MUST delay 100us between CLK_PMUCTL_NRBGLPEL_NORMAL and CLK_PMUCTL_NRBGLPEL_PL */
    for (u32Delay = SystemCoreClock/10000; u32Delay > 0; u32Delay--);

    /* Get main voltage regulator type status */
    u32PowerRegStatus = (SYS->PLSTS & SYS_PLSTS_CURMVR_Msk);

    /* Set main voltage regulator type */
    if((u32PowerRegulator == SYS_PLCTL_MVRS_DCDC) && (u32PowerRegStatus == SYS_PLSTS_CURMVR_LDO))
    {
        /* Set main voltage regulator type to DCDC if status is LDO */
        SYS->PLCTL = (SYS->PLCTL & (~SYS_PLCTL_MVRS_Msk)) | (SYS_PLCTL_MVRS_DCDC);

        /* Wait inductor detection and main voltage regulator type change ready */
        while((SYS->PLSTS & SYS_PLSTS_MVRCBUSY_Msk) == SYS_PLSTS_MVRCBUSY_Msk)
        {
            if(i32TimeOutCnt-- <= 0)
            {
                u32Ret = 0UL;   /* Main voltage regulator type change time-out */
                break;
            }
        }
    }
    else if((u32PowerRegulator == SYS_PLCTL_MVRS_LDO) && (u32PowerRegStatus == SYS_PLSTS_CURMVR_DCDC))
    {
        /* Set main voltage regulator type to LDO if status is DCDC */
        SYS->PLCTL = (SYS->PLCTL & (~SYS_PLCTL_MVRS_Msk)) | (SYS_PLCTL_MVRS_LDO);

        /* Wait main voltage regulator type change ready */
        while((SYS->PLSTS & SYS_PLSTS_MVRCBUSY_Msk) == SYS_PLSTS_MVRCBUSY_Msk)
        {
            if(i32TimeOutCnt-- <= 0)
            {
                u32Ret = 0UL;   /* Main voltage regulator type change time-out */
                break;
            }
        }
    }

    /* Clear main voltage regulator type change error flag */
    if(SYS->PLSTS & SYS_PLSTS_MVRCERR_Msk)
    {
        SYS->PLSTS = SYS_PLSTS_MVRCERR_Msk;
        u32Ret = 0UL;
    }

    /* Set Band-gap power mode back to original mode */
    CLK->PMUCTL = (CLK->PMUCTL & ~(CLK_PMUCTL_NRBGLPEL_Msk)) | u32BGPowerRegStatus;

    return u32Ret;
}

/**
  * @brief      Set Reference Voltage
  *
  * @param[in]  u32VRefCTL is reference voltage setting. Including :
  *             - \ref SYS_VREFCTL_VREF_1_5V
  *             - \ref SYS_VREFCTL_VREF_2_0V
  *             - \ref SYS_VREFCTL_VREF_2_5V
  *             - \ref SYS_VREFCTL_VREF_3_0V
  *             - \ref SYS_VREFCTL_VREF_4_0V
  *
  * @return     None
  *
  * @details    This function select reference voltage.
  *             The register write-protection function should be disabled before using this function.
  */
void SYS_SetVRef(uint32_t u32VRefCTL)
{
    /* Set reference voltage */
    SYS->VREFCTL = (SYS->VREFCTL & (~SYS_VREFCTL_VREFSEL_Msk)) | (u32VRefCTL) | (SYS_VREFCTL_VREFEN_Msk);
}

/*@}*/ /* end of group SYS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group SYS_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

/*** (C) COPYRIGHT 2024 Nuvoton Technology Corp. ***/
