/**************************************************************************//**
 * @file     clk.c
 * @version  V0.10
 * @brief    M2003J series Clock Controller (CLK) driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup CLK_Driver CLK Driver
  @{
*/

int32_t g_CLK_i32ErrCode = 0;   /*!< CLK global error code */

/** @addtogroup CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/


/**
  * @brief      Disable frequency output function
  * @param      None
  * @return     None
  * @details    This function disable frequency output function.
  */
void CLK_DisableCKO(void)
{
    /* Disable CKO feature */
    CLK->CLKOCTL &= (~CLK_CLKOCTL_CLKOEN_Msk);
}


/**
  * @brief      This function enable frequency divider module clock.
  *             enable frequency divider clock function and configure frequency divider.
  * @param[in]  u32ClkSrc is frequency divider function clock source. Including :
  *             - \ref CLK_CLKOSEL_CLKOSEL_HCLK
  *             - \ref CLK_CLKOSEL_CLKOSEL_PCLK0
  *             - \ref CLK_CLKOSEL_CLKOSEL_PCLK1
  *             - \ref CLK_CLKOSEL_CLKOSEL_LIRC
  *             - \ref CLK_CLKOSEL_CLKOSEL_LXT
  *             - \ref CLK_CLKOSEL_CLKOSEL_HXT
  *             - \ref CLK_CLKOSEL_CLKOSEL_HIRC
  *             - \ref CLK_CLKOSEL_CLKOSEL_PLL
  * @param[in]  u32ClkDiv is divider output frequency selection.
  * @param[in]  u32ClkDivBy1En is frequency divided by one enable.
  * @return     None
  * @details    Output selected clock to CKO. The output clock frequency is divided by u32ClkDiv.
  *             The formula is:
  *                 CKO frequency = (Clock source frequency) / 2^(u32ClkDiv + 1)
  *             This function is just used to set CKO clock.
  *             User must enable I/O for CKO clock output pin by themselves.
  */
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En)
{
    /* Select CKO clock source */
    CLK->CLKOSEL = (CLK->CLKOSEL & ~(CLK_CLKOSEL_CLKOSEL_Msk)) | (u32ClkSrc);

    /* CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKOCTL = CLK_CLKOCTL_CLKOEN_Msk | u32ClkDiv | (u32ClkDivBy1En << CLK_CLKOCTL_DIV1EN_Pos);
}


/**
  * @brief      Enter to Power-down mode
  * @param      None
  * @return     None
  * @details    This function is used to let system enter to Power-down mode. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_PowerDown(void)
{
    volatile uint32_t u32SysTickTICKINT = 0;    /* Backup Systick interrupt enable bit */

    /* Set the processor uses deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Backup systick interrupt setting */
    u32SysTickTICKINT = SysTick->CTRL & SysTick_CTRL_TICKINT_Msk;

    /* Disable systick interrupt */
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

    /* Chip enter Power-down mode after CPU run WFI instruction */
    __WFI();

    /* Restore systick interrupt setting */
    if (u32SysTickTICKINT)
    {
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    }
}


/**
  * @brief      Enter to Idle mode
  * @param      None
  * @return     None
  * @details    This function let system enter to Idle mode. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_Idle(void)
{
    /* Set the processor uses sleep as its low power mode */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* Chip enter idle mode after CPU run WFI instruction */
    __WFI();
}


/**
  * @brief      Get external high speed crystal clock frequency
  * @param      None
  * @return     External high frequency crystal frequency
  * @details    This function get external high frequency crystal frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetHXTFreq(void)
{
    if(CLK->SRCCTL & CLK_SRCCTL_HXTEN_Msk)
        return __HXT;
    else
        return 0;
}


/**
  * @brief      Get external low speed crystal clock frequency
  * @param      None
  * @return     External low speed crystal clock frequency
  * @details    This function get external low frequency crystal frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetLXTFreq(void)
{
    if(CLK->SRCCTL & CLK_SRCCTL_LXTEN_Msk)
        return __LXT;
    else
        return 0;
}


/**
  * @brief      Get HCLK frequency
  * @param      None
  * @return     HCLK frequency
  * @details    This function get HCLK frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetHCLKFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}


/**
  * @brief      Get PCLK0 frequency
  * @param      None
  * @return     PCLK0 frequency
  * @details    This function get PCLK0 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK0Freq(void)
{
    uint32_t PCLK0Div;

    SystemCoreClockUpdate();
    PCLK0Div = (CLK->PCLKDIV & CLK_PCLKDIV_PCLK0DIV_Msk) >> CLK_PCLKDIV_PCLK0DIV_Pos;
    return (SystemCoreClock >> PCLK0Div);
}


/**
  * @brief      Get PCLK1 frequency
  * @param      None
  * @return     PCLK1 frequency
  * @details    This function get PCLK1 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK1Freq(void)
{
    uint32_t PCLK1Div;

    SystemCoreClockUpdate();
    PCLK1Div = (CLK->PCLKDIV & CLK_PCLKDIV_PCLK1DIV_Msk) >> CLK_PCLKDIV_PCLK1DIV_Pos;
    return (SystemCoreClock >> PCLK1Div);
}


/**
  * @brief      Get CPU frequency
  * @param      None
  * @return     CPU frequency
  * @details    This function get CPU frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetCPUFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}


/**
  * @brief      Set HCLK frequency
  * @param[in]  u32Hclk is HCLK frequency.
  *             The range of u32Hclk is 32 MHz ~ 40 MHz.
  * @return     HCLK frequency
  * @details    This function is used to set HCLK frequency by using PLL. \n
  *             Power level is also set according to HCLK frequency. The frequency unit is Hz. \n
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_SetCoreClock(uint32_t u32Hclk)
{
    /* Check HCLK frequency range */
    if(u32Hclk > FREQ_40MHZ)
    {
        /* M2003J core clock up to 40 MHz */
        u32Hclk = FREQ_40MHZ;
    }
    else if(u32Hclk < FREQ_32MHZ)
    {
        /* The lower limit of PLL FOUT is 32 MHz */
        u32Hclk = FREQ_32MHZ;
    }

    /* Configure PLL setting if HXT clock is stable */
    if(CLK->STATUS & CLK_STATUS_HXTSTB_Msk)
    {
        u32Hclk = CLK_EnablePLL(NULL, u32Hclk);
    }
    else
    {
        return 0UL; /* Enable PLL fail since HXT not stable */
    }

    /* Select HCLK clock to PLL/1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_HCLKDIV_HCLK(1UL));

    /* Return actually HCLK frequency is PLL frequency divide 1 */
    return u32Hclk;
}


/**
  * @brief      Set HCLK clock source and HCLK clock divider
  * @param[in]  u32ClkSrc is HCLK clock source. Including :
  *             - \ref CLK_HCLKSEL_HCLKSEL_HIRC
  *             - \ref CLK_HCLKSEL_HCLKSEL_LIRC
  *             - \ref CLK_HCLKSEL_HCLKSEL_LXT
  *             - \ref CLK_HCLKSEL_HCLKSEL_HXT
  *             - \ref CLK_HCLKSEL_HCLKSEL_PLL
  * @param[in]  u32ClkDiv is HCLK clock divider. Including :
  *             - \ref CLK_HCLKDIV_HCLK(x)
  * @return     None
  * @details    This function set HCLK clock source and HCLK clock divider. \n
  *             Power level and flash access cycle are also set according to HCLK operation frequency. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    uint32_t u32HIRCSTB;

    /* Read HIRC clock source stable flag */
    u32HIRCSTB = CLK->STATUS & CLK_STATUS_HIRCSTB_Msk;

    /* Set Flash Access Cycle to maximum value for safe */
    FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (2);

    /* Switch HCLK clock source to HIRC clock for safe */
    CLK->SRCCTL |= CLK_SRCCTL_HIRCEN_Msk;
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK->HCLKSEL = (CLK->HCLKSEL & ~CLK_HCLKSEL_HCLKSEL_Msk) |
                   (CLK_CLKSEL0_HCLKSEL_HIRC);

    /* Apply new Divider */
    CLK->HCLKDIV = (CLK->HCLKDIV & ~CLK_HCLKDIV_HCLKDIV_Msk) | (u32ClkDiv);

    /* Switch HCLK to new HCLK source */
    CLK->HCLKSEL = (CLK->HCLKSEL & ~CLK_HCLKSEL_HCLKSEL_Msk) | (u32ClkSrc);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set Flash Access Cycle */
    if(SystemCoreClock < FREQ_21MHZ)
    {
        FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (1);
    }
    else
    {
        FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (2);
    }

    /* Disable HIRC if HIRC is disabled before switching HCLK source */
    if(u32HIRCSTB == 0UL)
    {
        CLK->SRCCTL &= ~CLK_SRCCTL_HIRCEN_Msk;
    }
}


/**
  * @brief      This function set selected module clock source and module clock divider
  * @param[in]  u32ModuleIdx is module index.
  * @param[in]  u32ClkSrc is module clock source.
  * @param[in]  u32ClkDiv is module clock divider.
  * @return     None
  * @details    Valid parameter combinations listed in following table:
  *
  * |Module index   |Clock source                           |Divider                   |
  * | :-----------  | :------------------------------------ | :----------------------- |
  * |ADC0_MODULE    | x                                     |\ref CLK_ADCDIV_ADC0(x)   |
  * |BPWM0_MODULE   |\ref CLK_BPWMSEL_BPWM0SEL_PCLK0        | x                        |
  * |BPWM0_MODULE   |\ref CLK_BPWMSEL_BPWM0SEL_HCLK         | x                        |
  * |BPWM1_MODULE   |\ref CLK_BPWMSEL_BPWM1SEL_PCLK1        | x                        |
  * |BPWM1_MODULE   |\ref CLK_BPWMSEL_BPWM1SEL_HCLK         | x                        |
  * |CLKO_MODULE    |\ref CLK_CLKOSEL_CLKOSEL_HCLK          | x                        |
  * |CLKO_MODULE    |\ref CLK_CLKOSEL_CLKOSEL_PCLK0         | x                        |
  * |CLKO_MODULE    |\ref CLK_CLKOSEL_CLKOSEL_PCLK1         | x                        |
  * |CLKO_MODULE    |\ref CLK_CLKOSEL_CLKOSEL_LIRC          | x                        |
  * |CLKO_MODULE    |\ref CLK_CLKOSEL_CLKOSEL_LXT           | x                        |
  * |CLKO_MODULE    |\ref CLK_CLKOSEL_CLKOSEL_HXT           | x                        |
  * |CLKO_MODULE    |\ref CLK_CLKOSEL_CLKOSEL_HIRC          | x                        |
  * |CLKO_MODULE    |\ref CLK_CLKOSEL_CLKOSEL_PLL           | x                        |
  * |RTC0_MODULE    |\ref RTC_LXTCTL_RTCCKSEL_LXT           | x                        |
  * |RTC0_MODULE    |\ref RTC_LXTCTL_RTCCKSEL_LIRC          | x                        |
  * |TMR0_MODULE    |\ref CLK_TMRSEL0_TMR0SEL_PCLK0         | x                        |
  * |TMR0_MODULE    |\ref CLK_TMRSEL0_TMR0SEL_LIRC          | x                        |
  * |TMR0_MODULE    |\ref CLK_TMRSEL0_TMR0SEL_LXT           | x                        |
  * |TMR0_MODULE    |\ref CLK_TMRSEL0_TMR0SEL_HXT           | x                        |
  * |TMR0_MODULE    |\ref CLK_TMRSEL0_TMR0SEL_HIRC          | x                        |
  * |TMR0_MODULE    |\ref CLK_TMRSEL0_TMR0SEL_EXT           | x                        |
  * |TMR1_MODULE    |\ref CLK_TMRSEL0_TMR1SEL_PCLK0         | x                        |
  * |TMR1_MODULE    |\ref CLK_TMRSEL0_TMR1SEL_LIRC          | x                        |
  * |TMR1_MODULE    |\ref CLK_TMRSEL0_TMR1SEL_LXT           | x                        |
  * |TMR1_MODULE    |\ref CLK_TMRSEL0_TMR1SEL_HXT           | x                        |
  * |TMR1_MODULE    |\ref CLK_TMRSEL0_TMR1SEL_HIRC          | x                        |
  * |TMR1_MODULE    |\ref CLK_TMRSEL0_TMR1SEL_EXT           | x                        |
  * |TMR2_MODULE    |\ref CLK_TMRSEL0_TMR2SEL_PCLK1         | x                        |
  * |TMR2_MODULE    |\ref CLK_TMRSEL0_TMR2SEL_LIRC          | x                        |
  * |TMR2_MODULE    |\ref CLK_TMRSEL0_TMR2SEL_LXT           | x                        |
  * |TMR2_MODULE    |\ref CLK_TMRSEL0_TMR2SEL_HXT           | x                        |
  * |TMR2_MODULE    |\ref CLK_TMRSEL0_TMR2SEL_HIRC          | x                        |
  * |TMR2_MODULE    |\ref CLK_TMRSEL0_TMR2SEL_EXT           | x                        |
  * |TMR3_MODULE    |\ref CLK_TMRSEL0_TMR3SEL_PCLK1         | x                        |
  * |TMR3_MODULE    |\ref CLK_TMRSEL0_TMR3SEL_LIRC          | x                        |
  * |TMR3_MODULE    |\ref CLK_TMRSEL0_TMR3SEL_LXT           | x                        |
  * |TMR3_MODULE    |\ref CLK_TMRSEL0_TMR3SEL_HXT           | x                        |
  * |TMR3_MODULE    |\ref CLK_TMRSEL0_TMR3SEL_HIRC          | x                        |
  * |TMR3_MODULE    |\ref CLK_TMRSEL0_TMR3SEL_EXT           | x                        |
  * |TMR4_MODULE    |\ref CLK_TMRSEL0_TMR4SEL_PCLK0         | x                        |
  * |TMR4_MODULE    |\ref CLK_TMRSEL0_TMR4SEL_LIRC          | x                        |
  * |TMR4_MODULE    |\ref CLK_TMRSEL0_TMR4SEL_LXT           | x                        |
  * |TMR4_MODULE    |\ref CLK_TMRSEL0_TMR4SEL_HXT           | x                        |
  * |TMR4_MODULE    |\ref CLK_TMRSEL0_TMR4SEL_HIRC          | x                        |
  * |TMR4_MODULE    |\ref CLK_TMRSEL0_TMR4SEL_EXT           | x                        |
  * |TMR5_MODULE    |\ref CLK_TMRSEL0_TMR5SEL_PCLK0         | x                        |
  * |TMR5_MODULE    |\ref CLK_TMRSEL0_TMR5SEL_LIRC          | x                        |
  * |TMR5_MODULE    |\ref CLK_TMRSEL0_TMR5SEL_LXT           | x                        |
  * |TMR5_MODULE    |\ref CLK_TMRSEL0_TMR5SEL_HXT           | x                        |
  * |TMR5_MODULE    |\ref CLK_TMRSEL0_TMR5SEL_HIRC          | x                        |
  * |TMR5_MODULE    |\ref CLK_TMRSEL0_TMR5SEL_EXT           | x                        |
  * |TMR6_MODULE    |\ref CLK_TMRSEL0_TMR6SEL_PCLK1         | x                        |
  * |TMR6_MODULE    |\ref CLK_TMRSEL0_TMR6SEL_LIRC          | x                        |
  * |TMR6_MODULE    |\ref CLK_TMRSEL0_TMR6SEL_LXT           | x                        |
  * |TMR6_MODULE    |\ref CLK_TMRSEL0_TMR6SEL_HXT           | x                        |
  * |TMR6_MODULE    |\ref CLK_TMRSEL0_TMR6SEL_HIRC          | x                        |
  * |TMR6_MODULE    |\ref CLK_TMRSEL0_TMR6SEL_EXT           | x                        |
  * |TMR7_MODULE    |\ref CLK_TMRSEL0_TMR7SEL_PCLK1         | x                        |
  * |TMR7_MODULE    |\ref CLK_TMRSEL0_TMR7SEL_LIRC          | x                        |
  * |TMR7_MODULE    |\ref CLK_TMRSEL0_TMR7SEL_LXT           | x                        |
  * |TMR7_MODULE    |\ref CLK_TMRSEL0_TMR7SEL_HXT           | x                        |
  * |TMR7_MODULE    |\ref CLK_TMRSEL0_TMR7SEL_HIRC          | x                        |
  * |TMR7_MODULE    |\ref CLK_TMRSEL0_TMR7SEL_EXT           | x                        |
  * |TMR8_MODULE    |\ref CLK_TMRSEL0_TMR8SEL_PCLK1         | x                        |
  * |TMR8_MODULE    |\ref CLK_TMRSEL0_TMR8SEL_LIRC          | x                        |
  * |TMR8_MODULE    |\ref CLK_TMRSEL0_TMR8SEL_LXT           | x                        |
  * |TMR8_MODULE    |\ref CLK_TMRSEL0_TMR8SEL_HXT           | x                        |
  * |TMR8_MODULE    |\ref CLK_TMRSEL0_TMR8SEL_HIRC          | x                        |
  * |TMR8_MODULE    |\ref CLK_TMRSEL0_TMR8SEL_EXT           | x                        |
  * |UART0_MODULE   |\ref CLK_UARTSEL_UART0SEL_PCLK0        |\ref CLK_UARTDIV_UART0(X) |
  * |UART0_MODULE   |\ref CLK_UARTSEL_UART0SEL_LIRC         |\ref CLK_UARTDIV_UART0(X) |
  * |UART0_MODULE   |\ref CLK_UARTSEL_UART0SEL_LXT          |\ref CLK_UARTDIV_UART0(X) |
  * |UART0_MODULE   |\ref CLK_UARTSEL_UART0SEL_HXT          |\ref CLK_UARTDIV_UART0(X) |
  * |UART0_MODULE   |\ref CLK_UARTSEL_UART0SEL_HIRC         |\ref CLK_UARTDIV_UART0(X) |
  * |UART1_MODULE   |\ref CLK_UARTSEL_UART1SEL_PCLK1        |\ref CLK_UARTDIV_UART1(X) |
  * |UART1_MODULE   |\ref CLK_UARTSEL_UART1SEL_LIRC         |\ref CLK_UARTDIV_UART1(X) |
  * |UART1_MODULE   |\ref CLK_UARTSEL_UART1SEL_LXT          |\ref CLK_UARTDIV_UART1(X) |
  * |UART1_MODULE   |\ref CLK_UARTSEL_UART1SEL_HXT          |\ref CLK_UARTDIV_UART1(X) |
  * |UART1_MODULE   |\ref CLK_UARTSEL_UART1SEL_HIRC         |\ref CLK_UARTDIV_UART1(X) |
  * |UART2_MODULE   |\ref CLK_UARTSEL_UART2SEL_PCLK0        |\ref CLK_UARTDIV_UART2(X) |
  * |UART2_MODULE   |\ref CLK_UARTSEL_UART2SEL_LIRC         |\ref CLK_UARTDIV_UART2(X) |
  * |UART2_MODULE   |\ref CLK_UARTSEL_UART2SEL_LXT          |\ref CLK_UARTDIV_UART2(X) |
  * |UART2_MODULE   |\ref CLK_UARTSEL_UART2SEL_HXT          |\ref CLK_UARTDIV_UART2(X) |
  * |UART2_MODULE   |\ref CLK_UARTSEL_UART2SEL_HIRC         |\ref CLK_UARTDIV_UART2(X) |
  * |UART3_MODULE   |\ref CLK_UARTSEL_UART3SEL_PCLK1        |\ref CLK_UARTDIV_UART3(X) |
  * |UART3_MODULE   |\ref CLK_UARTSEL_UART3SEL_LIRC         |\ref CLK_UARTDIV_UART3(X) |
  * |UART3_MODULE   |\ref CLK_UARTSEL_UART3SEL_LXT          |\ref CLK_UARTDIV_UART3(X) |
  * |UART3_MODULE   |\ref CLK_UARTSEL_UART3SEL_HXT          |\ref CLK_UARTDIV_UART3(X) |
  * |UART3_MODULE   |\ref CLK_UARTSEL_UART3SEL_HIRC         |\ref CLK_UARTDIV_UART3(X) |
  * |UART4_MODULE   |\ref CLK_UARTSEL_UART4SEL_PCLK0        |\ref CLK_UARTDIV_UART4(X) |
  * |UART4_MODULE   |\ref CLK_UARTSEL_UART4SEL_LIRC         |\ref CLK_UARTDIV_UART4(X) |
  * |UART4_MODULE   |\ref CLK_UARTSEL_UART4SEL_LXT          |\ref CLK_UARTDIV_UART4(X) |
  * |UART4_MODULE   |\ref CLK_UARTSEL_UART4SEL_HXT          |\ref CLK_UARTDIV_UART4(X) |
  * |UART4_MODULE   |\ref CLK_UARTSEL_UART4SEL_HIRC         |\ref CLK_UARTDIV_UART4(X) |
  * |WDT0_MODULE    |\ref CLK_WDTSEL_WDT0SEL_HCLK_DIV2048   | x                        |
  * |WDT0_MODULE    |\ref CLK_WDTSEL_WDT0SEL_LXT            | x                        |
  * |WDT0_MODULE    |\ref CLK_WDTSEL_WDT0SEL_LIRC           | x                        |
  * |WWDT0_MODULE   |\ref CLK_WWDTSEL_WWDT0SEL_HCLK_DIV2048 | x                        |
  * |WWDT0_MODULE   |\ref CLK_WWDTSEL_WWDT0SEL_LIRC         | x                        |
  */
void CLK_SetModuleClock(uint64_t u64ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    uint32_t u32Sel = 0UL, u32Div = 0UL;
    uint32_t u32RTCCKEN;

    /* RTC clock source configuration */
    if (u64ModuleIdx == RTC0_MODULE)
    {
        u32RTCCKEN = CLK->RTCCTL & CLK_RTCCTL_RTC0CKEN_Msk;

        /* Enable RTC clock to get LXTCTL value */
        CLK->RTCCTL |= CLK_RTCCTL_RTC0CKEN_Msk;

        /* Select RTC clock source */
        RTC->LXTCTL = (RTC->LXTCTL & (~RTC_LXTCTL_RTCCKSEL_Msk)) | (u32ClkSrc);

        /* Disable RTC clock if it is disabled before */
        if (u32RTCCKEN == 0UL)
        {
            CLK->RTCCTL &= (~CLK_RTCCTL_RTC0CKEN_Msk);
        }
    }
    else /* Others clock source configuration */
    {
        /* Configure clock source divider */
        if (MODULE_CLKDIV_Msk(u64ModuleIdx) != MODULE_NoMsk)
        {
            /* Get clock divider control register address */
            u32Div = (uint32_t)MODULE_CLKDIV_BASE + ((uint32_t)MODULE_CLKDIV(u64ModuleIdx) << 2);
            /* Apply new divider */
            M32(u32Div) = (M32(u32Div) & (~((uint32_t)MODULE_CLKDIV_Msk(u64ModuleIdx) << (uint32_t)MODULE_CLKDIV_Pos(u64ModuleIdx)))) | u32ClkDiv;
        }

        /* Configure clock source */
        if (MODULE_CLKSEL_Msk(u64ModuleIdx) != MODULE_NoMsk)
        {
            /* Get clock select control register address */
            u32Sel = (uint32_t)MODULE_CLKSEL_BASE + ((uint32_t)MODULE_CLKSEL(u64ModuleIdx) << 2);
            /* Set new clock selection setting */
            M32(u32Sel) = (M32(u32Sel) & (~((uint32_t)MODULE_CLKSEL_Msk(u64ModuleIdx) << (uint32_t)MODULE_CLKSEL_Pos(u64ModuleIdx)))) | u32ClkSrc;
        }
    }
}


/**
  * @brief      Enable clock source
  * @param[in]  u32ClkMask is clock source mask. Including :
  *             - \ref CLK_SRCCTL_LIRCEN_Msk
  *             - \ref CLK_SRCCTL_LXTEN_Msk
  *             - \ref CLK_SRCCTL_HXTEN_Msk
  *             - \ref CLK_SRCCTL_HIRCEN_Msk
  *             - \ref CLK_SRCCTL_PLLEN_Msk
  * @return     None
  * @details    This function enable clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_EnableXtalRC(uint32_t u32ClkMask)
{
    CLK->SRCCTL |= u32ClkMask;
}


/**
  * @brief      Disable clock source
  * @param[in]  u32ClkMask is clock source mask. Including :
  *             - \ref CLK_SRCCTL_LIRCEN_Msk
  *             - \ref CLK_SRCCTL_LXTEN_Msk
  *             - \ref CLK_SRCCTL_HXTEN_Msk
  *             - \ref CLK_SRCCTL_HIRCEN_Msk
  *             - \ref CLK_SRCCTL_PLLEN_Msk
  * @return     None
  * @details    This function disable clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_DisableXtalRC(uint32_t u32ClkMask)
{
    CLK->SRCCTL &= ~u32ClkMask;
}


/**
  * @brief      Enable module clock
  * @param[in]  u32ModuleIdx is module index. Including :
  *             - \ref ADC0_MODULE
  *             - \ref BPWM0_MODULE
  *             - \ref BPWM1_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref CRC0_MODULE
  *             - \ref FMC0_MODULE
  *             - \ref DFMC0_MODULE
  *             - \ref ISP0_MODULE
  *             - \ref GPIOA_MODULE
  *             - \ref GPIOB_MODULE
  *             - \ref GPIOC_MODULE
  *             - \ref GPIOD_MODULE
  *             - \ref GPIOE_MODULE
  *             - \ref GPIOF_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref I2C1_MODULE
  *             - \ref I2C2_MODULE
  *             - \ref PDMA0_MODULE
  *             - \ref RTC0_MODULE
  *             - \ref SRAM0_MODULE
  *             - \ref SRAM1_MODULE
  *             - \ref ST0_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref TMR4_MODULE
  *             - \ref TMR5_MODULE
  *             - \ref TMR6_MODULE
  *             - \ref TMR7_MODULE
  *             - \ref TMR8_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref UART2_MODULE
  *             - \ref UART3_MODULE
  *             - \ref UART4_MODULE
  *             - \ref USCI0_MODULE
  *             - \ref USCI1_MODULE
  *             - \ref USCI2_MODULE
  *             - \ref USCI3_MODULE
  *             - \ref USCI4_MODULE
  *             - \ref WDT0_MODULE
  *             - \ref WWDT0_MODULE
  * @return     None
  * @details    This function is used to enable module clock.
  */
void CLK_EnableModuleClock(uint64_t u64ModuleIdx)
{
    uint32_t u32TmpVal = 0UL, u32TmpAddr = 0UL;

    /* Get enable bit from module index */
    u32TmpVal = (1UL << (uint32_t)MODULE_CLKEN_Pos(u64ModuleIdx));
    /* Get address bit from module index */
    u32TmpAddr = (uint32_t)MODULE_CLKCTL_BASE + (uint32_t)(MODULE_CLKCTL(u64ModuleIdx) << 2);
    /* Enable module clock */
    *(volatile uint32_t *)u32TmpAddr |= u32TmpVal;
}


/**
  * @brief      Disable module clock
  * @param[in]  u32ModuleIdx is module index
  *             - \ref ADC0_MODULE
  *             - \ref BPWM0_MODULE
  *             - \ref BPWM1_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref CRC0_MODULE
  *             - \ref FMC0_MODULE
  *             - \ref DFMC0_MODULE
  *             - \ref ISP0_MODULE
  *             - \ref GPIOA_MODULE
  *             - \ref GPIOB_MODULE
  *             - \ref GPIOC_MODULE
  *             - \ref GPIOD_MODULE
  *             - \ref GPIOE_MODULE
  *             - \ref GPIOF_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref I2C1_MODULE
  *             - \ref I2C2_MODULE
  *             - \ref PDMA0_MODULE
  *             - \ref RTC0_MODULE
  *             - \ref SRAM0_MODULE
  *             - \ref SRAM1_MODULE
  *             - \ref ST0_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref TMR4_MODULE
  *             - \ref TMR5_MODULE
  *             - \ref TMR6_MODULE
  *             - \ref TMR7_MODULE
  *             - \ref TMR8_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref UART2_MODULE
  *             - \ref UART3_MODULE
  *             - \ref UART4_MODULE
  *             - \ref USCI0_MODULE
  *             - \ref USCI1_MODULE
  *             - \ref USCI2_MODULE
  *             - \ref USCI3_MODULE
  *             - \ref USCI4_MODULE
  *             - \ref WDT0_MODULE
  *             - \ref WWDT0_MODULE
  * @return     None
  * @details    This function is used to disable module clock.
  */
void CLK_DisableModuleClock(uint64_t u64ModuleIdx)
{
    uint32_t u32TmpVal = 0UL, u32TmpAddr = 0UL;

    /* Get enable bit from module index */
    u32TmpVal = (1UL << (uint32_t)MODULE_CLKEN_Pos(u64ModuleIdx));
    /* Get address bit from module index */
    u32TmpAddr = (uint32_t)MODULE_CLKCTL_BASE + (uint32_t)(MODULE_CLKCTL(u64ModuleIdx) << 2);
    /* Disable module clock */
    *(volatile uint32_t *)u32TmpAddr &= ~u32TmpVal;
}


/**
  * @brief      Set PLL frequency
  * @param[in]  u32PllClkSrc is PLL clock source. M2003J always use HXT as clcok source.
  * @param[in]  u32PllFreq is PLL frequency. The range of u32PllFreq is 32 MHz ~ 40 MHz.
  * @return     PLL frequency
  * @details    This function is used to configure PLLCTL register to set specified PLL frequency. \n
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq)
{
    uint32_t u32PllSrcClk, u32NR, u32NF, u32NO, u32PllClk;
    uint32_t u32Tmp, u32Tmp2, u32Tmp3, u32Min, u32MinNF, u32MinNR;

    /* Disable PLL first to avoid unstable when setting PLL */
    CLK->SRCCTL &= ~CLK_SRCCTL_PLLEN_Msk;

    /* Config PLL source clock */
    /* M2003J always use HXT as clcok source */
    /* Enable HXT clock */
    CLK->SRCCTL |= CLK_SRCCTL_HXTEN_Msk;

    /* Wait for HXT clock ready */
    if (CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk) == 0)
    {
        return 0;
    }

    /* Select PLL source clock from HXT */
    u32PllSrcClk = __HXT;

    /* Check PLL frequency range */
    /* Constraint 1: 32MHz <= FOUT <= 40MHz */
    if((u32PllFreq <= FREQ_40MHZ) && (u32PllFreq >= FREQ_32MHZ))
    {
        /* M2003J only support NO = 4 */
        u32NO = 4UL;

        /* u32NR start from 3 to avoid calculation overflow if needed */
        u32NR = 1UL;

        /* Find best solution */
        u32Min = (uint32_t) - 1;    /* initial u32Min to max value of uint32_t (0xFFFFFFFF) */
        u32MinNR = 0UL;
        u32MinNF = 0UL;

        for(; u32NR <= 16UL; u32NR++)   /* max NR = 16 since NR = INDIV+1 and INDIV = 0~15 */
        {
            u32Tmp = u32PllSrcClk / u32NR;                      /* FREF = FIN/NR */
            if((u32Tmp >= FREQ_4MHZ) && (u32Tmp <= FREQ_8MHZ))  /* Constraint 2: 4MHz <= FREF <= 8MHz. */
            {
                for(u32NF = 4UL; u32NF <= 10UL; u32NF++)       /* NF should be 1~16 since NF = FBDIV+1 and FBDIV = 0~15 */
                {
                    /* Note: NF min value is 4 and max value is 10 */
                    u32Tmp2 = (u32Tmp * u32NF) << 2;                            /* FVCO = FREF*4*NF */
                    if((u32Tmp2 >= FREQ_128MHZ) && (u32Tmp2 <= FREQ_160MHZ))     /* Constraint 3: 128MHz <= FVCO <= 160MHz */
                    {
                        u32Tmp3 = (u32Tmp2 > u32PllFreq) ? u32Tmp2 - u32PllFreq : u32PllFreq - u32Tmp2;
                        if(u32Tmp3 < u32Min)
                        {
                            u32Min = u32Tmp3;
                            u32MinNR = u32NR;
                            u32MinNF = u32NF;

                            /* Break when get good results */
                            if(u32Min == 0UL)
                            {
                                break;
                            }
                        }
                    }
                }
            }
        }

        /* Enable and apply new PLL setting. */
        CLK->PLLCTL = u32PllClkSrc |
                      (CLK_PLLCTL_STBSEL_CLKSRC) |
                      ((u32MinNR - 1UL) << CLK_PLLCTL_INDIV_Pos) |
                      ((u32MinNF - 1UL) << CLK_PLLCTL_FBDIV_Pos);

        /* Actual PLL output clock frequency. FOUT = (FIN/NR)*4*NF*(1/NO) */
        u32PllClk = u32PllSrcClk / ((u32NO) * u32MinNR) * (u32MinNF << 2);
    }
    else
    {
        /* Apply default PLL setting and return */
        CLK->PLLCTL = CLK_PLLCTL_40MHz_HXT | CLK_PLLCTL_STBSEL_CLKSRC;

        /* Actual PLL output clock frequency */
        u32PllClk = FREQ_40MHZ;
    }

    /* Enable PLL */
    CLK->SRCCTL |= CLK_SRCCTL_PLLEN_Msk;

    /* Wait for PLL clock stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Return actual PLL output clock frequency */
    return u32PllClk;
}


/**
  * @brief      Disable PLL
  * @param      None
  * @return     None
  * @details    This function disable PLL. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_DisablePLL(void)
{
    CLK->SRCCTL &= ~CLK_SRCCTL_PLLEN_Msk;
}


/**
  * @brief      This function check selected clock source status
  * @param[in]  u32ClkMask is selected clock source. Including :
  *             - \ref CLK_STATUS_HXTSTB_Msk
  *             - \ref CLK_STATUS_LXTSTB_Msk
  *             - \ref CLK_STATUS_HIRCSTB_Msk
  *             - \ref CLK_STATUS_LIRCSTB_Msk
  *             - \ref CLK_STATUS_PLLSTB_Msk
  *             - \ref CLK_STATUS_HCLKSWF_Msk
  * @retval     0  clock is not stable
  * @retval     1  clock is stable
  * @details    To wait for clock ready by specified clock source stable flag or timeout (~500ms)
  * @note       This function sets g_CLK_i32ErrCode to CLK_TIMEOUT_ERR if clock source status is not stable.
  */
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask)
{
    /* Set timeout to about 500ms. */
    uint32_t u32TimeOutCnt = (CyclesPerUs * 500000) / 50;

    uint32_t u32Ret = 1U;

    g_CLK_i32ErrCode = 0;
    while((CLK->STATUS & u32ClkMask) != u32ClkMask)
    {
        if(--u32TimeOutCnt == 0)
        {
            u32Ret = 0U;
            break;
        }
    }

    if(u32TimeOutCnt == 0)
        g_CLK_i32ErrCode = CLK_TIMEOUT_ERR;

    return u32Ret;
}


/**
  * @brief      Enable System Tick counter
  * @param[in]  u32ClkSrc is System Tick clock source. Including:
  *             - N/A
  * @param[in]  u32Count is System Tick reload value. It could be 0~0xFFFFFF.
  * @return     None
  * @details    This function set System Tick clock source, reload value, enable System Tick counter and interrupt. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count)
{
    /* Set System Tick counter disabled */
    SysTick->CTRL = 0UL;

    /* Set System Tick reload value */
    SysTick->LOAD = u32Count;

    /* Clear System Tick current value and counter flag */
    SysTick->VAL = 0UL;

    /* Set System Tick interrupt enabled and counter enabled */
    SysTick->CTRL |= (SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
}


/**
  * @brief      Disable System Tick counter
  * @param      None
  * @return     None
  * @details    This function disable System Tick counter.
  */
void CLK_DisableSysTick(void)
{
    /* Set System Tick counter disabled */
    SysTick->CTRL = 0UL;
}


/**
  * @brief      Get PLL clock frequency
  * @param      None
  * @return     PLL frequency
  * @details    This function get PLL frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPLLClockFreq(void)
{
    uint32_t u32PllFreq = 0UL, u32PllReg;
    uint32_t u32FIN, u32NF, u32NR;

    u32PllReg = CLK->PLLCTL;

    u32FIN = __HXT;     /* PLL source clock from HXT */

    /* Calculate PLL frequency */
    if(u32PllReg & CLK_PLLCTL_BP_Msk)
    {
        u32PllFreq = u32FIN;  /* PLL is in bypass mode */
    }
    else
    {
        /* PLL is output enabled in normal work mode */
        u32NF = ((u32PllReg & CLK_PLLCTL_FBDIV_Msk) >> CLK_PLLCTL_FBDIV_Pos) + 1UL;
        u32NR = ((u32PllReg & CLK_PLLCTL_INDIV_Msk) >> CLK_PLLCTL_INDIV_Pos) + 1UL;

        /* u32FIN is shifted 2 bits to avoid overflow */
        u32PllFreq = ( ((u32FIN >> 2) * 4 * u32NF) / (u32NR * 4) ) << 2;
    }

    return u32PllFreq;
}


/**
  * @brief      Get selected module clock source
  * @param[in]  u32ModuleIdx is module index.
  *             - \ref BPWM0_MODULE
  *             - \ref BPWM1_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref RTC0_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref TMR4_MODULE
  *             - \ref TMR5_MODULE
  *             - \ref TMR6_MODULE
  *             - \ref TMR7_MODULE
  *             - \ref TMR8_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref UART2_MODULE
  *             - \ref UART3_MODULE
  *             - \ref UART4_MODULE
  *             - \ref WDT0_MODULE
  *             - \ref WWDT0_MODULE
  * @return     Selected module clock source setting
  * @details    This function get selected module clock source.
  */
uint32_t CLK_GetModuleClockSource(uint64_t u64ModuleIdx)
{
    uint32_t u32TmpVal = 0UL, u32TmpAddr = 0UL;

    /* Get clock select control register address */
    u32TmpAddr = (uint32_t)MODULE_CLKSEL_BASE + (uint32_t)(MODULE_CLKSEL(u64ModuleIdx) << 2);

    /* Get clock source selection setting */
    u32TmpVal = (inpw((uint32_t *)u32TmpAddr) & (uint32_t)(MODULE_CLKSEL_Msk(u64ModuleIdx) << (uint32_t)MODULE_CLKSEL_Pos(u64ModuleIdx))) >> (uint32_t)MODULE_CLKSEL_Pos(u64ModuleIdx);

    return u32TmpVal;
}


/**
  * @brief      Get selected module clock divider number
  * @param[in]  u32ModuleIdx is module index.
  *             - \ref ADC0_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref UART2_MODULE
  *             - \ref UART3_MODULE
  *             - \ref UART4_MODULE
  * @return     Selected module clock divider number setting
  * @details    This function get selected module clock divider number.
  */
uint32_t CLK_GetModuleClockDivider(uint64_t u64ModuleIdx)
{
    uint32_t u32DivVal = 0UL, u32DivAddr = 0UL;

    /* Get clock divider control register address */
    u32DivAddr = (uint32_t)MODULE_CLKDIV_BASE + ((uint32_t)MODULE_CLKDIV(u64ModuleIdx) << 2);

    /* Get clock divider number setting */
    u32DivVal = (inpw((uint32_t *)u32DivAddr) & (uint32_t)(MODULE_CLKDIV_Msk(u64ModuleIdx) << (uint32_t)MODULE_CLKDIV_Pos(u64ModuleIdx))) >> (uint32_t)MODULE_CLKDIV_Pos(u64ModuleIdx);

    return u32DivVal;
}


/**@}*/ /* end of group CLK_EXPORTED_FUNCTIONS */

/**@}*/ /* end of group CLK_Driver */

/**@}*/ /* end of group Standard_Driver */
