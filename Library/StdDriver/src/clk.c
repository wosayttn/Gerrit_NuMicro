/**************************************************************************//**
 * @file     clk.c
 * @version  V1.00
 * @brief    M2U51 series CLK driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
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
  * @brief      Disable clock divider output function
  *
  * @param      None
  *
  * @return     None
  *
  * @details    This function disable clock divider output function.
  */
void CLK_DisableCKO(void)
{
    /* Disable CKO clock source */
    CLK_DisableModuleClock(CLKO_MODULE);
}

/**
  * @brief      This function enable clock divider output module clock,
  *             enable clock divider output function and set frequency selection.
  *
  * @param[in]  u32ClkSrc is frequency divider function clock source. Including :
  *             - \ref CLK_CLKSEL1_CLKOSEL_HCLK
  *             - \ref CLK_CLKSEL1_CLKOSEL_MIRC
  *             - \ref CLK_CLKSEL1_CLKOSEL_HIRC
  *             - \ref CLK_CLKSEL1_CLKOSEL_LIRC
  *             - \ref CLK_CLKSEL1_CLKOSEL_LXT
  * @param[in]  u32ClkDiv is divider output frequency selection. It could be 0~15.
  * @param[in]  u32ClkDivBy1En is clock divided by one enabled.
  *
  * @return     None
  *
  * @details    Output selected clock to CKO. The output clock frequency is divided by u32ClkDiv. \n
  *             The formula is: \n
  *                 CKO frequency = (Clock source frequency) / 2^(u32ClkDiv + 1) \n
  *             This function is just used to set CKO clock.
  *             User must enable I/O for CKO clock output pin by themselves. \n
  */
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En)
{
    /* CKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKOCTL = CLK_CLKOCTL_CLKOEN_Msk | (u32ClkDiv) | (u32ClkDivBy1En << CLK_CLKOCTL_DIV1EN_Pos);

    /* Enable CKO clock source */
    CLK_EnableModuleClock(CLKO_MODULE);

    /* Select CKO clock source */
    CLK_SetModuleClock(CLKO_MODULE, u32ClkSrc, 0UL);
}

/**
  * @brief      Enter to Power-down mode
  *
  * @param      None
  *
  * @return     None
  *
  * @details    This function is used to let system enter to Power-down mode. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_PowerDown(void)
{
    uint32_t u32HIRCTRIMCTL;

    /* Set the processor uses deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled */
    CLK->PWRCTL |= (CLK_PWRCTL_PDEN_Msk);

    /* Store HIRC control register */
    u32HIRCTRIMCTL = SYS->HIRCTCTL;

    /* Disable HIRC auto trim */
    SYS->HIRCTCTL &= (~SYS_HIRCTCTL_AUTRIMEN_Msk);

    /* Set the LDO/DCDC to ULP mode for SPD0 power-down mode to save power consumption. */
    if ((CLK->PMUCTL & CLK_PMUCTL_PDMSEL_Msk) == CLK_PMUCTL_PDMSEL_SPD0)
    {
        CLK->PMUCTL = (CLK->PMUCTL & ~(CLK_PMUCTL_PDLDCSEL_Msk)) | CLK_PMUCTL_PDLDCSEL_ULP;
    }

    /* Chip enter Power-down mode after CPU run WFI instruction */
    __WFI();

    /* Set the LDO/DCDC back to the default LP mode. */
    if ((CLK->PMUCTL & CLK_PMUCTL_PDMSEL_Msk) == CLK_PMUCTL_PDMSEL_SPD0)
    {
        CLK->PMUCTL = (CLK->PMUCTL & ~(CLK_PMUCTL_PDLDCSEL_Msk)) | CLK_PMUCTL_PDLDCSEL_LP;
    }

    /* Restore HIRC control register */
    SYS->HIRCTCTL = u32HIRCTRIMCTL;
}

/**
  * @brief      Enter to Idle mode
  *
  * @param      None
  *
  * @return     None
  *
  * @details    This function let system enter to Idle mode. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_Idle(void)
{
    /* Set the processor uses sleep as its low power mode */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* Set chip in idle mode because of WFI command */
    CLK->PWRCTL &= ~CLK_PWRCTL_PDEN_Msk;

    /* Chip enter idle mode after CPU run WFI instruction */
    __WFI();
}

/**
  * @brief      Get external low speed crystal clock frequency
  *
  * @param      None
  *
  * @return     External low speed crystal clock frequency
  *
  * @details    This function get external low frequency crystal frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetLXTFreq(void)
{
    uint32_t u32Freq;
    if((CLK->PWRCTL & CLK_PWRCTL_LXTEN_Msk) == CLK_PWRCTL_LXTEN_Msk)
    {
        u32Freq = __LXT;
    }
    else
    {
        u32Freq = 0UL;
    }

    return u32Freq;
}

/**
  * @brief      Get PCLK0 frequency
  *
  * @param      None
  *
  * @return     PCLK0 frequency
  *
  * @details    This function get PCLK0 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK0Freq(void)
{
    uint32_t u32Freq;
    SystemCoreClockUpdate();

    if((CLK->PCLKDIV & CLK_PCLKDIV_APB0DIV_Msk) == CLK_PCLKDIV_APB0DIV_DIV1)
    {
        u32Freq = SystemCoreClock;
    }
    else if((CLK->PCLKDIV & CLK_PCLKDIV_APB0DIV_Msk) == CLK_PCLKDIV_APB0DIV_DIV2)
    {
        u32Freq = SystemCoreClock / 2UL;
    }
    else if((CLK->PCLKDIV & CLK_PCLKDIV_APB0DIV_Msk) == CLK_PCLKDIV_APB0DIV_DIV4)
    {
        u32Freq = SystemCoreClock / 4UL;
    }
    else if((CLK->PCLKDIV & CLK_PCLKDIV_APB0DIV_Msk) == CLK_PCLKDIV_APB0DIV_DIV8)
    {
        u32Freq = SystemCoreClock / 8UL;
    }
    else if((CLK->PCLKDIV & CLK_PCLKDIV_APB0DIV_Msk) == CLK_PCLKDIV_APB0DIV_DIV16)
    {
        u32Freq = SystemCoreClock / 16UL;
    }
    else
    {
        u32Freq = SystemCoreClock;
    }

    return u32Freq;
}

/**
  * @brief      Get PCLK1 frequency
  *
  * @param      None
  *
  * @return     PCLK1 frequency
  *
  * @details    This function get PCLK1 frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetPCLK1Freq(void)
{
    uint32_t u32Freq;
    SystemCoreClockUpdate();

    if((CLK->PCLKDIV & CLK_PCLKDIV_APB1DIV_Msk) == CLK_PCLKDIV_APB1DIV_DIV1)
    {
        u32Freq = SystemCoreClock;
    }
    else if((CLK->PCLKDIV & CLK_PCLKDIV_APB1DIV_Msk) == CLK_PCLKDIV_APB1DIV_DIV2)
    {
        u32Freq = SystemCoreClock / 2UL;
    }
    else if((CLK->PCLKDIV & CLK_PCLKDIV_APB1DIV_Msk) == CLK_PCLKDIV_APB1DIV_DIV4)
    {
        u32Freq = SystemCoreClock / 4UL;
    }
    else if((CLK->PCLKDIV & CLK_PCLKDIV_APB1DIV_Msk) == CLK_PCLKDIV_APB1DIV_DIV8)
    {
        u32Freq = SystemCoreClock / 8UL;
    }
    else if((CLK->PCLKDIV & CLK_PCLKDIV_APB1DIV_Msk) == CLK_PCLKDIV_APB1DIV_DIV16)
    {
        u32Freq = SystemCoreClock / 16UL;
    }
    else
    {
        u32Freq = SystemCoreClock;
    }

    return u32Freq;
}

/**
  * @brief      Get HCLK frequency
  *
  * @param      None
  *
  * @return     HCLK frequency
  *
  * @details    This function get HCLK frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetHCLKFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

/**
  * @brief      Get CPU frequency
  *
  * @param      None
  *
  * @return     CPU frequency
  *
  * @details    This function get CPU frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetCPUFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}


/**
  * @brief      Set HCLK frequency
  *
  * @param[in]  u32Hclk is HCLK frequency. The range of u32Hclk is running up to 40MHz.
  *
  * @return     HCLK frequency
  *
  * @details    This function is used to set HCLK frequency. The frequency unit is Hz. \n
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_SetCoreClock(uint32_t u32Hclk)
{
    uint32_t u32HIRCSTB;
    uint32_t u32MIRC;
    uint32_t u32ActuallyHclk;
    uint32_t u32FmcCycle;
    int32_t  i32TimeOutCnt = 2160000;

    /* The range of u32Hclk is running up to 40 MHz */
    if(u32Hclk >= FREQ_40MHZ)
    {
        u32MIRC = CLK_PWRCTL_MIRCFSEL_40M;
        u32ActuallyHclk = FREQ_40MHZ;
        u32FmcCycle = 3UL;
    }
    else if(u32Hclk >= FREQ_32MHZ)
    {
        u32MIRC = CLK_PWRCTL_MIRCFSEL_32M;
        u32ActuallyHclk = FREQ_32MHZ;
        u32FmcCycle = 3UL;
    }
    else if(u32Hclk >= FREQ_24MHZ)
    {
        u32MIRC = CLK_PWRCTL_MIRCFSEL_24M;
        u32ActuallyHclk = FREQ_24MHZ;
        u32FmcCycle = 2UL;
    }
    else if(u32Hclk >= FREQ_16MHZ)
    {
        u32MIRC = CLK_PWRCTL_MIRCFSEL_16M;
        u32ActuallyHclk = FREQ_16MHZ;
        u32FmcCycle = 1UL;
    }
    else if(u32Hclk >= FREQ_12MHZ)
    {
        u32MIRC = CLK_PWRCTL_MIRCFSEL_12M;
        u32ActuallyHclk = FREQ_12MHZ;
        u32FmcCycle = 1UL;
    }
    else if(u32Hclk >= FREQ_8MHZ)
    {
        u32MIRC = CLK_PWRCTL_MIRCFSEL_8M;
        u32ActuallyHclk = FREQ_8MHZ;
        u32FmcCycle = 1UL;
    }
    else if(u32Hclk >= FREQ_4MHZ)
    {
        u32MIRC = CLK_PWRCTL_MIRCFSEL_4M;
        u32ActuallyHclk = FREQ_4MHZ;
        u32FmcCycle = 1UL;
    }
    else if(u32Hclk >= FREQ_2MHZ)
    {
        u32MIRC = CLK_PWRCTL_MIRCFSEL_2M;
        u32ActuallyHclk = FREQ_2MHZ;
        u32FmcCycle = 1UL;
    }
    else
    {
        u32MIRC = CLK_PWRCTL_MIRCFSEL_1M;
        u32ActuallyHclk = FREQ_1MHZ;
        u32FmcCycle = 1UL;
    }

    /* M2U51 series supports high-performance MIRC switching if HCLK is already set to MIRC. */
    if ((CLK->CLKSEL0 & CLK_CLKSEL0_HCLKSEL_Msk) == CLK_CLKSEL0_HCLKSEL_MIRC)
    {
        /* Switch FMC access cycle to maximum value for safe */
        FMC->CYCCTL = 3UL;

        /* Change MIRC frequency and then enable */
        CLK->PWRCTL = (CLK->PWRCTL & ~(CLK_PWRCTL_MIRCFSEL_Msk)) | (u32MIRC);
        CLK->PWRCTL |= CLK_PWRCTL_MIRCEN_Msk;

        /* Wait MIRC stable */
        while((CLK->STATUS & CLK_STATUS_MIRCSTB_Msk) != CLK_STATUS_MIRCSTB_Msk)
        {
            if(i32TimeOutCnt-- <= 0)
            {
                break;
            }
        }

        /* Set HCLK divider to 1 */
        CLK->HCLKDIV = (CLK->HCLKDIV & (~CLK_HCLKDIV_HCLKDIV_Msk)) | 0;

        /* Switch FMC access cycle to suitable value for performance */
        FMC->CYCCTL = u32FmcCycle;

        /* Update System Core Clock */
        SystemCoreClockUpdate();
    }
    else
    {
        /* Read HIRC clock source stable flag */
        u32HIRCSTB = CLK->STATUS & CLK_STATUS_HIRCSTB_Msk;

        /* Switch HCLK clock source to HIRC clock for safe */
        CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
        CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
        CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_HCLKDIV_HCLK(1UL));

        /* Switch HCLK clock source to MIRC clock */
        u32ActuallyHclk = CLK_EnableMIRC(u32MIRC);
        CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_MIRC, CLK_HCLKDIV_HCLK(1UL));

        /* Disable HIRC if HIRC is disabled before setting core clock */
        if(u32HIRCSTB == 0UL)
        {
            CLK_DisableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
        }
    }

    /* Return actually HCLK frequency */
    return u32ActuallyHclk;
}

/**
  * @brief      This function set HCLK clock source and HCLK clock divider
  *
  * @param[in]  u32ClkSrc is HCLK clock source. Including :
  *             - \ref CLK_CLKSEL0_HCLKSEL_MIRC
  *             - \ref CLK_CLKSEL0_HCLKSEL_HIRC
  *             - \ref CLK_CLKSEL0_HCLKSEL_LIRC
  *             - \ref CLK_CLKSEL0_HCLKSEL_LXT
  * @param[in]  u32ClkDiv is HCLK clock divider. Including :
  *             - \ref CLK_CLKDIV0_HCLK(x)
  *
  * @return     None
  *
  * @details    This function set HCLK clock source and HCLK clock divider. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    uint32_t u32HIRCSTB;

    /* Read HIRC clock source stable flag */
    u32HIRCSTB = CLK->STATUS & CLK_STATUS_HIRCSTB_Msk;

    /* Switch FMC access cycle to maximum value for safe */
    FMC->CYCCTL = 3UL;

    /* Switch to HIRC for Safe. Avoid HCLK too high when applying new divider. */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Apply new Divider */
    CLK->HCLKDIV = (CLK->HCLKDIV & (~CLK_HCLKDIV_HCLKDIV_Msk)) | u32ClkDiv;

    /* Switch HCLK to new HCLK source */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | u32ClkSrc;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Switch FMC access cycle to suitable value base on HCLK */
    if (SystemCoreClock >= 32000000UL)
    {
        FMC->CYCCTL = 3UL;
    }
    else if (SystemCoreClock >= 17000000UL)
    {
        FMC->CYCCTL = 2UL;
    }
    else
    {
        FMC->CYCCTL = 1UL;
    }

    /* Disable HIRC if HIRC is disabled before switching HCLK source */
    if(u32HIRCSTB == 0UL)
    {
        CLK_DisableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    }
}

/* Convert mask bit number to mask */
static uint32_t num_to_mask(uint32_t num)
{
    uint32_t u32mask;
    switch(num)
    {
    case 1:
        u32mask = 0x01UL;
        break;
    case 2:
        u32mask = 0x03UL;
        break;
    case 3:
        u32mask = 0x07UL;
        break;
    case 4:
        u32mask = 0x0FUL;
        break;
    case 5:
        u32mask = 0x1FUL;
        break;
    case 6:
        u32mask = 0x3FUL;
        break;
    case 7:
        u32mask = 0x7FUL;
        break;
    case 8:
        u32mask = 0xFFUL;
        break;
    default:
        u32mask = 0x00UL;
        break;
    }
    return u32mask;
}

/**
  * @brief      This function set selected module clock source and module clock divider
  *
  * @param[in]  u32ModuleIdx is module index.
  * @param[in]  u32ClkSrc is module clock source.
  * @param[in]  u32ClkDiv is module clock divider.
  *
  * @return     None
  *
  * @details    Valid parameter combinations listed in following table:
  *
  * |Module index       |Clock source                           |Divider                    |
  * | :---------------- | :-----------------------------------  | :----------------------   |
  * |\ref WDT_MODULE    |\ref CLK_CLKSEL1_WDTSEL_LIRC           | x                         |
  * |\ref WDT_MODULE    |\ref CLK_CLKSEL1_WDTSEL_LXT            | x                         |
  * |\ref WWDT_MODULE   |\ref CLK_CLKSEL1_WWDTSEL_LIRC          | x                         |
  * |\ref WWDT_MODULE   |\ref CLK_CLKSEL1_WWDTSEL_LXT           | x                         |
  * |\ref CLKO_MODULE   |\ref CLK_CLKSEL1_CLKOSEL_HCLK          | x                         |
  * |\ref CLKO_MODULE   |\ref CLK_CLKSEL1_CLKOSEL_HCLK0         | x                         |
  * |\ref CLKO_MODULE   |\ref CLK_CLKSEL1_CLKOSEL_MIRC          | x                         |
  * |\ref CLKO_MODULE   |\ref CLK_CLKSEL1_CLKOSEL_HIRC          | x                         |
  * |\ref CLKO_MODULE   |\ref CLK_CLKSEL1_CLKOSEL_LIRC          | x                         |
  * |\ref CLKO_MODULE   |\ref CLK_CLKSEL1_CLKOSEL_LXT           | x                         |
  * |\ref TMR0_MODULE   |\ref CLK_CLKSEL1_TMR0SEL_PCLK0         | x                         |
  * |\ref TMR0_MODULE   |\ref CLK_CLKSEL1_TMR0SEL_MIRC          | x                         |
  * |\ref TMR0_MODULE   |\ref CLK_CLKSEL1_TMR0SEL_HIRC          | x                         |
  * |\ref TMR0_MODULE   |\ref CLK_CLKSEL1_TMR0SEL_LIRC          | x                         |
  * |\ref TMR0_MODULE   |\ref CLK_CLKSEL1_TMR0SEL_LXT           | x                         |
  * |\ref TMR0_MODULE   |\ref CLK_CLKSEL1_TMR0SEL_EXT           | x                         |
  * |\ref TMR1_MODULE   |\ref CLK_CLKSEL1_TMR1SEL_PCLK0         | x                         |
  * |\ref TMR1_MODULE   |\ref CLK_CLKSEL1_TMR1SEL_MIRC          | x                         |
  * |\ref TMR1_MODULE   |\ref CLK_CLKSEL1_TMR1SEL_HIRC          | x                         |
  * |\ref TMR1_MODULE   |\ref CLK_CLKSEL1_TMR1SEL_LIRC          | x                         |
  * |\ref TMR1_MODULE   |\ref CLK_CLKSEL1_TMR1SEL_LXT           | x                         |
  * |\ref TMR1_MODULE   |\ref CLK_CLKSEL1_TMR1SEL_EXT           | x                         |
  * |\ref TMR2_MODULE   |\ref CLK_CLKSEL1_TMR2SEL_PCLK1         | x                         |
  * |\ref TMR2_MODULE   |\ref CLK_CLKSEL1_TMR2SEL_MIRC          | x                         |
  * |\ref TMR2_MODULE   |\ref CLK_CLKSEL1_TMR2SEL_HIRC          | x                         |
  * |\ref TMR2_MODULE   |\ref CLK_CLKSEL1_TMR2SEL_LIRC          | x                         |
  * |\ref TMR2_MODULE   |\ref CLK_CLKSEL1_TMR2SEL_LXT           | x                         |
  * |\ref TMR2_MODULE   |\ref CLK_CLKSEL1_TMR2SEL_EXT           | x                         |
  * |\ref TMR3_MODULE   |\ref CLK_CLKSEL1_TMR3SEL_PCLK1         | x                         |
  * |\ref TMR3_MODULE   |\ref CLK_CLKSEL1_TMR3SEL_MIRC          | x                         |
  * |\ref TMR3_MODULE   |\ref CLK_CLKSEL1_TMR3SEL_HIRC          | x                         |
  * |\ref TMR3_MODULE   |\ref CLK_CLKSEL1_TMR3SEL_LIRC          | x                         |
  * |\ref TMR3_MODULE   |\ref CLK_CLKSEL1_TMR3SEL_LXT           | x                         |
  * |\ref TMR3_MODULE   |\ref CLK_CLKSEL1_TMR3SEL_EXT           | x                         |
  * |\ref ADC0_MODULE   |\ref CLK_CLKSEL1_ADC0SEL_PCLK1         |\ref CLK_CLKDIV_ADC0(x)    |
  * |\ref ADC0_MODULE   |\ref CLK_CLKSEL1_ADC0SEL_MIRC          |\ref CLK_CLKDIV_ADC0(x)    |
  * |\ref ADC0_MODULE   |\ref CLK_CLKSEL1_ADC0SEL_HIRC          |\ref CLK_CLKDIV_ADC0(x)    |
  * |\ref LCD_MODULE    |\ref CLK_CLKSEL1_LCDSEL_LIRC           | x                         |
  * |\ref LCD_MODULE    |\ref CLK_CLKSEL1_LCDSEL_LXT            | x                         |
  * |\ref LCDCP_MODULE  |\ref CLK_CLKSEL1_LCDCPSEL_DIV32        | x                         |
  * |\ref LCDCP_MODULE  |\ref CLK_CLKSEL1_LCDCPSEL_DIV16        | x                         |
  * |\ref LCDCP_MODULE  |\ref CLK_CLKSEL1_LCDCPSEL_DIV8         | x                         |
  * |\ref SPI0_MODULE   |\ref CLK_CLKSEL2_SPI0SEL_PCLK1         | x                         |
  * |\ref SPI0_MODULE   |\ref CLK_CLKSEL2_SPI0SEL_MIRC          | x                         |
  * |\ref SPI0_MODULE   |\ref CLK_CLKSEL2_SPI0SEL_HIRC          | x                         |
  * |\ref SPI1_MODULE   |\ref CLK_CLKSEL2_SPI1SEL_PCLK0         | x                         |
  * |\ref SPI1_MODULE   |\ref CLK_CLKSEL2_SPI1SEL_MIRC          | x                         |
  * |\ref SPI1_MODULE   |\ref CLK_CLKSEL2_SPI1SEL_HIRC          | x                         |
  * |\ref SPI2_MODULE   |\ref CLK_CLKSEL2_SPI2SEL_PCLK1         | x                         |
  * |\ref SPI2_MODULE   |\ref CLK_CLKSEL2_SPI2SEL_MIRC          | x                         |
  * |\ref SPI2_MODULE   |\ref CLK_CLKSEL2_SPI2SEL_HIRC          | x                         |
  * |\ref UART0_MODULE  |\ref CLK_CLKSEL2_UART0SEL_PCLK0        |\ref CLK_CLKDIV_UART0(x)   |
  * |\ref UART0_MODULE  |\ref CLK_CLKSEL2_UART0SEL_MIRC         |\ref CLK_CLKDIV_UART0(x)   |
  * |\ref UART0_MODULE  |\ref CLK_CLKSEL2_UART0SEL_HIRC         |\ref CLK_CLKDIV_UART0(x)   |
  * |\ref UART0_MODULE  |\ref CLK_CLKSEL2_UART0SEL_LIRC         |\ref CLK_CLKDIV_UART0(x)   |
  * |\ref UART0_MODULE  |\ref CLK_CLKSEL2_UART0SEL_LXT          |\ref CLK_CLKDIV_UART0(x)   |
  * |\ref UART1_MODULE  |\ref CLK_CLKSEL2_UART1SEL_PCLK1        |\ref CLK_CLKDIV_UART1(x)   |
  * |\ref UART1_MODULE  |\ref CLK_CLKSEL2_UART1SEL_MIRC         |\ref CLK_CLKDIV_UART1(x)   |
  * |\ref UART1_MODULE  |\ref CLK_CLKSEL2_UART1SEL_HIRC         |\ref CLK_CLKDIV_UART1(x)   |
  * |\ref UART1_MODULE  |\ref CLK_CLKSEL2_UART1SEL_LIRC         |\ref CLK_CLKDIV_UART1(x)   |
  * |\ref UART1_MODULE  |\ref CLK_CLKSEL2_UART1SEL_LXT          |\ref CLK_CLKDIV_UART1(x)   |
  * |\ref UART2_MODULE  |\ref CLK_CLKSEL2_UART2SEL_PCLK0        |\ref CLK_CLKDIV_UART2(x)   |
  * |\ref UART2_MODULE  |\ref CLK_CLKSEL2_UART2SEL_MIRC         |\ref CLK_CLKDIV_UART2(x)   |
  * |\ref UART2_MODULE  |\ref CLK_CLKSEL2_UART2SEL_HIRC         |\ref CLK_CLKDIV_UART2(x)   |
  * |\ref UART2_MODULE  |\ref CLK_CLKSEL2_UART2SEL_LIRC         |\ref CLK_CLKDIV_UART2(x)   |
  * |\ref UART2_MODULE  |\ref CLK_CLKSEL2_UART2SEL_LXT          |\ref CLK_CLKDIV_UART2(x)   |
  */
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    uint32_t u32sel = 0UL, u32div = 0UL;
    uint32_t u32SelTbl[4] = {0x0UL, 0x4UL, 0x8UL, 0x0UL};   /* CLKSEL offset on MODULE index, 0x0:CLKSEL0, 0x1:CLKSEL1, 0x2:CLKSEL2 */
    uint32_t u32DivTbl[4] = {0x0UL, 0x4UL, 0x8UL, 0x0UL};   /* CLKDIV offset on MODULE index, 0x0:HCLKDIV, 0x1:PCLKDIV, 0x2:CLKDIV */
    uint32_t u32mask;

    if(MODULE_CLKDIV_Msk(u32ModuleIdx) != MODULE_NoMsk)
    {
        /* Get clock divider control register address */
        u32div = (uint32_t)&CLK->HCLKDIV + (u32DivTbl[MODULE_CLKDIV(u32ModuleIdx)]);

        /* Convert mask bit number to mask */
        u32mask = num_to_mask(MODULE_CLKDIV_Msk(u32ModuleIdx));

        /* Apply new divider */
        M32(u32div) = (M32(u32div) & (~(u32mask << MODULE_CLKDIV_Pos(u32ModuleIdx)))) | u32ClkDiv;
    }

    if(MODULE_CLKSEL_Msk(u32ModuleIdx) != MODULE_NoMsk)
    {
        /* Get clock select control register address */
        u32sel = (uint32_t)&CLK->CLKSEL0 + (u32SelTbl[MODULE_CLKSEL(u32ModuleIdx)]);

        /* Convert mask bit number to mask */
        u32mask = num_to_mask(MODULE_CLKSEL_Msk(u32ModuleIdx));

        /* Set new clock selection setting */
        M32(u32sel) = (M32(u32sel) & (~(u32mask << MODULE_CLKSEL_Pos(u32ModuleIdx)))) | u32ClkSrc;
    }
}

/**
  * @brief      Enable clock source
  *
  * @param[in]  u32ClkMask is clock source mask. Including :
  *             - \ref CLK_PWRCTL_LXTEN_Msk
  *             - \ref CLK_PWRCTL_HIRCEN_Msk
  *             - \ref CLK_PWRCTL_LIRCEN_Msk
  *             - \ref CLK_PWRCTL_MIRCEN_Msk
  *
  * @return     None
  *
  * @details    This function enable clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_EnableXtalRC(uint32_t u32ClkMask)
{
    CLK->PWRCTL |= u32ClkMask;
}

/**
  * @brief      Disable clock source
  *
  * @param[in]  u32ClkMask is clock source mask. Including :
  *             - \ref CLK_PWRCTL_LXTEN_Msk
  *             - \ref CLK_PWRCTL_HIRCEN_Msk
  *             - \ref CLK_PWRCTL_LIRCEN_Msk
  *             - \ref CLK_PWRCTL_MIRCEN_Msk
  *
  * @return     None
  *
  * @details    This function disable clock source. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_DisableXtalRC(uint32_t u32ClkMask)
{
    CLK->PWRCTL &= ~(u32ClkMask);
}

/**
  * @brief      Enable module clock
  *
  * @param[in]  u32ModuleIdx is module index. Including :
  *             - \ref PDMA0_MODULE
  *             - \ref ISP_MODULE
  *             - \ref CRC_MODULE
  *             - \ref CRPT_MODULE
  *             - \ref GPA_MODULE
  *             - \ref GPB_MODULE
  *             - \ref GPC_MODULE
  *             - \ref GPD_MODULE
  *             - \ref GPE_MODULE
  *             - \ref GPF_MODULE
  *             - \ref GPG_MODULE
  *             - \ref GPH_MODULE
  *             - \ref WDT_MODULE
  *             - \ref RTC_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref ACMP01_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref I2C1_MODULE
  *             - \ref I2C2_MODULE
  *             - \ref SPI0_MODULE
  *             - \ref SPI1_MODULE
  *             - \ref SPI2_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref UART2_MODULE
  *             - \ref USCI0_MODULE
  *             - \ref WWDT_MODULE
  *             - \ref PWM0_MODULE
  *             - \ref BPWM0_MODULE
  *             - \ref LCD_MODULE
  *             - \ref LCDCP_MODULE
  *             - \ref ADC0_MODULE
  *
  * @return     None
  *
  * @details    This function is used to enable module clock.
  */
void CLK_EnableModuleClock(uint32_t u32ModuleIdx)
{
    uint32_t u32ClkTbl[2] = {0x0UL, 0x4UL}; /* AHBCLK/APBCLK offset on MODULE index, 0x0:AHBCLK0, 0x1:APBCLK0 */

    *(volatile uint32_t *)((uint32_t)&CLK->AHBCLK0 + (u32ClkTbl[MODULE_APBCLK(u32ModuleIdx)]))  |= (1UL << MODULE_IP_EN_Pos(u32ModuleIdx));
}

/**
  * @brief      Disable module clock
  *
  * @param[in]  u32ModuleIdx is module index. Including :
  *             - \ref PDMA0_MODULE
  *             - \ref ISP_MODULE
  *             - \ref CRC_MODULE
  *             - \ref CRPT_MODULE
  *             - \ref GPA_MODULE
  *             - \ref GPB_MODULE
  *             - \ref GPC_MODULE
  *             - \ref GPD_MODULE
  *             - \ref GPE_MODULE
  *             - \ref GPF_MODULE
  *             - \ref GPG_MODULE
  *             - \ref GPH_MODULE
  *             - \ref WDT_MODULE
  *             - \ref RTC_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref ACMP01_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref I2C1_MODULE
  *             - \ref I2C2_MODULE
  *             - \ref SPI0_MODULE
  *             - \ref SPI1_MODULE
  *             - \ref SPI2_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref UART2_MODULE
  *             - \ref USCI0_MODULE
  *             - \ref WWDT_MODULE
  *             - \ref PWM0_MODULE
  *             - \ref BPWM0_MODULE
  *             - \ref LCD_MODULE
  *             - \ref LCDCP_MODULE
  *             - \ref ADC0_MODULE
  *
  * @return     None
  *
  * @details    This function is used to disable module clock.
  */
void CLK_DisableModuleClock(uint32_t u32ModuleIdx)
{
    uint32_t u32ClkTbl[2] = {0x0UL, 0x4UL}; /* AHBCLK/APBCLK offset on MODULE index, 0x0:AHBCLK0, 0x1:APBCLK0 */

    *(volatile uint32_t *)((uint32_t)&CLK->AHBCLK0 + (u32ClkTbl[MODULE_APBCLK(u32ModuleIdx)]))  &= ~(1 << MODULE_IP_EN_Pos(u32ModuleIdx));
}

/**
  * @brief      This function check selected clock source status
  *
  * @param[in]  u32ClkMask is selected clock source. Including :
  *             - \ref CLK_STATUS_LXTSTB_Msk
  *             - \ref CLK_STATUS_LIRCSTB_Msk
  *             - \ref CLK_STATUS_HIRCSTB_Msk
  *             - \ref CLK_STATUS_MIRCSTB_Msk
  *
  * @retval     0  clock is not stable
  * @retval     1  clock is stable
  *
  * @details    To wait for clock ready by specified clock source stable flag or timeout.
  */
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask)
{
    int32_t i32TimeOutCnt = 2160000;
    uint32_t u32Ret = 1UL;

    while((CLK->STATUS & u32ClkMask) != u32ClkMask)
    {
        if(i32TimeOutCnt-- <= 0)
        {
            u32Ret = 0UL;
            break;
        }
    }
    return u32Ret;
}

/**
  * @brief      Enable System Tick counter
  *
  * @param[in]  u32ClkSrc is System Tick clock source. M2U51 don't support it.
  * @param[in]  u32Count is System Tick reload value. It could be 0~0xFFFFFF.
  *
  * @return     None
  *
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
  *
  * @param      None
  *
  * @return     None
  *
  * @details    This function disable System Tick counter.
  */
void CLK_DisableSysTick(void)
{
    /* Set System Tick counter disabled */
    SysTick->CTRL = 0UL;
}

/**
  * @brief      Power-down mode selected
  *
  * @param[in]  u32PDMode is power down mode index. Including :
  *             - \ref CLK_PMUCTL_PDMSEL_NPD0
  *             - \ref CLK_PMUCTL_PDMSEL_NPD1
  *             - \ref CLK_PMUCTL_PDMSEL_NPD2
  *             - \ref CLK_PMUCTL_PDMSEL_SPD0
  *             - \ref CLK_PMUCTL_PDMSEL_DPD0
  *
  * @return     None
  *
  * @details    This function is used to set power-down mode.
  */
void CLK_SetPowerDownMode(uint32_t u32PDMode)
{
    CLK->PMUCTL = (CLK->PMUCTL & ~(CLK_PMUCTL_PDMSEL_Msk)) | u32PDMode;
}


/**
  * @brief       Set Wake-up pin trigger type at Deep Power down mode
  *
  * @param[in]   u32Pin  The pin of specified GPIO.
  *              - \ref CLK_DPDWKPIN_0   : PC0
  *              - \ref CLK_DPDWKPIN_1   : PB0
  *              - \ref CLK_DPDWKPIN_2   : PB2
  *              - \ref CLK_DPDWKPIN_3   : PB12
  *              - \ref CLK_DPDWKPIN_4   : PF6
  * @param[in]   u32TriggerType
  *              - \ref CLK_DPDWKPIN_DISABLE
  *              - \ref CLK_DPDWKPIN_RISING
  *              - \ref CLK_DPDWKPIN_FALLING
  *              - \ref CLK_DPDWKPIN_BOTHEDGE
  *
  * @return      None
  *
  * @details     This function is used to enable Wake-up pin trigger type.
  */
void CLK_EnableDPDWKPin(uint32_t u32Pin, uint32_t u32TriggerType)
{
    switch (u32Pin)
    {
    case CLK_DPDWKPIN_0:
        CLK->PMUWKCTL = (CLK->PMUWKCTL & ~CLK_PMUWKCTL_WKPINEN0_Msk) | (u32TriggerType << CLK_PMUWKCTL_WKPINEN0_Pos);
        break;
    case CLK_DPDWKPIN_1:
        CLK->PMUWKCTL = (CLK->PMUWKCTL & ~CLK_PMUWKCTL_WKPINEN1_Msk) | (u32TriggerType << CLK_PMUWKCTL_WKPINEN1_Pos);
        break;
    case CLK_DPDWKPIN_2:
        CLK->PMUWKCTL = (CLK->PMUWKCTL & ~CLK_PMUWKCTL_WKPINEN2_Msk) | (u32TriggerType << CLK_PMUWKCTL_WKPINEN2_Pos);
        break;
    case CLK_DPDWKPIN_3:
        CLK->PMUWKCTL = (CLK->PMUWKCTL & ~CLK_PMUWKCTL_WKPINEN3_Msk) | (u32TriggerType << CLK_PMUWKCTL_WKPINEN3_Pos);
        break;
    case CLK_DPDWKPIN_4:
        CLK->PMUWKCTL = (CLK->PMUWKCTL & ~CLK_PMUWKCTL_WKPINEN4_Msk) | (u32TriggerType << CLK_PMUWKCTL_WKPINEN4_Pos);
        break;
    }
}

/**
  * @brief      Get power manager wake up source
  *
  * @param[in]   None
  *
  * @return      None
  *
  * @details     This function get power manager wake up source.
  */

uint32_t CLK_GetPMUWKSrc(void)
{
    return (CLK->PMUSTS);
}

/**
  * @brief       Set specified GPIO as wake up source at Stand-by Power down mode
  *
  * @param[in]   u32Port GPIO port. It could be 0~3.
  * @param[in]   u32Pin  The pin of specified GPIO port. It could be 0 ~ 15.
  * @param[in]   u32TriggerType
  *              - \ref CLK_SPDWKPIN_RISING
  *              - \ref CLK_SPDWKPIN_FALLING
  * @param[in]   u32DebounceEn
  *              - \ref CLK_SPDWKPIN_DEBOUNCEEN
  *              - \ref CLK_SPDWKPIN_DEBOUNCEDIS
  *
  * @return      None
  *
  * @details     This function is used to set specified GPIO as wake up source
  *              at Stand-by Power down mode.
  */
void CLK_EnableSPDWKPin(uint32_t u32Port, uint32_t u32Pin, uint32_t u32TriggerType, uint32_t u32DebounceEn)
{
    uint32_t u32tmpAddr = 0UL;
    uint32_t u32tmpVal = 0UL;

    /* GPx Stand-by Power-down Wake-up Pin Select */
    u32tmpAddr = (uint32_t)&CLK->PAPWCTL;
    u32tmpAddr += (0x4UL * u32Port);

    u32tmpVal = inpw((uint32_t *)u32tmpAddr);
    u32tmpVal = (u32tmpVal & ~(CLK_PAPWCTL_WKPSEL0_Msk | CLK_PAPWCTL_PRWKEN0_Msk | CLK_PAPWCTL_PFWKEN0_Msk | CLK_PAPWCTL_DBEN0_Msk | CLK_PAPWCTL_WKEN0_Msk)) |
                (u32Pin << CLK_PAPWCTL_WKPSEL0_Pos) | u32TriggerType | u32DebounceEn | CLK_SPDWKPIN_ENABLE;
    outpw((uint32_t *)u32tmpAddr, u32tmpVal);
}

/**
  * @brief      Get selected module clock source
  *
  * @param[in]  u32ModuleIdx is module index.
  *             - \ref PDMA0_MODULE
  *             - \ref ISP_MODULE
  *             - \ref CRC_MODULE
  *             - \ref CRPT_MODULE
  *             - \ref GPA_MODULE
  *             - \ref GPB_MODULE
  *             - \ref GPC_MODULE
  *             - \ref GPD_MODULE
  *             - \ref GPE_MODULE
  *             - \ref GPF_MODULE
  *             - \ref GPG_MODULE
  *             - \ref GPH_MODULE
  *             - \ref WDT_MODULE
  *             - \ref RTC_MODULE
  *             - \ref TMR0_MODULE
  *             - \ref TMR1_MODULE
  *             - \ref TMR2_MODULE
  *             - \ref TMR3_MODULE
  *             - \ref CLKO_MODULE
  *             - \ref ACMP01_MODULE
  *             - \ref I2C0_MODULE
  *             - \ref I2C1_MODULE
  *             - \ref I2C2_MODULE
  *             - \ref SPI0_MODULE
  *             - \ref SPI1_MODULE
  *             - \ref SPI2_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref UART2_MODULE
  *             - \ref USCI0_MODULE
  *             - \ref WWDT_MODULE
  *             - \ref PWM0_MODULE
  *             - \ref BPWM0_MODULE
  *             - \ref LCD_MODULE
  *             - \ref LCDCP_MODULE
  *             - \ref ADC0_MODULE
  *
  * @return     Selected module clock source setting
  *
  * @details    This function get selected module clock source.
  */
uint32_t CLK_GetModuleClockSource(uint32_t u32ModuleIdx)
{
    uint32_t u32sel = 0UL;
    uint32_t u32SelTbl[3] = {0x0UL, 0x04UL, 0x08UL};    /* CLKSEL offset on MODULE index, 0x0:CLKSEL0, 0x1:CLKSEL1, 0x2:CLKSEL2 */
    uint32_t u32mask;

    /* Get clock source selection setting */
    if(MODULE_CLKSEL_Msk(u32ModuleIdx) != MODULE_NoMsk)
    {
        /* Get clock select control register address */
        u32sel = (uint32_t)&CLK->CLKSEL0 + (u32SelTbl[MODULE_CLKSEL(u32ModuleIdx)]);

        /* Convert mask bit number to mask */
        u32mask = num_to_mask(MODULE_CLKSEL_Msk(u32ModuleIdx));

        /* Get clock source selection setting */
        return ((M32(u32sel) & (u32mask << MODULE_CLKSEL_Pos(u32ModuleIdx))) >> MODULE_CLKSEL_Pos(u32ModuleIdx));
    }
    else
        return 0;
}

/**
  * @brief      Get selected module clock divider number
  *
  * @param[in]  u32ModuleIdx is module index.
  *             - \ref ADC0_MODULE
  *             - \ref UART0_MODULE
  *             - \ref UART1_MODULE
  *             - \ref UART2_MODULE
  *
  * @return     Selected module clock divider number setting
  *
  * @details    This function get selected module clock divider number.
  */
uint32_t CLK_GetModuleClockDivider(uint32_t u32ModuleIdx)
{
    uint32_t u32div = 0UL;
    uint32_t u32DivTbl[3] = {0x0UL, 0x04UL, 0x08UL}; /* CLKDIV offset on MODULE index, 0x0:HCLKDIV, 0x1:PCLKDIV, 0x2:CLKDIV */
    uint32_t u32mask;

    if(MODULE_CLKDIV_Msk(u32ModuleIdx) != MODULE_NoMsk)
    {
        /* Get clock divider control register address */
        u32div = (uint32_t)&CLK->HCLKDIV + (u32DivTbl[MODULE_CLKDIV(u32ModuleIdx)]);

        /* Convert mask bit number to mask */
        u32mask = num_to_mask(MODULE_CLKDIV_Msk(u32ModuleIdx));

        /* Get clock divider number setting */
        return ((M32(u32div) & (u32mask << MODULE_CLKDIV_Pos(u32ModuleIdx))) >> MODULE_CLKDIV_Pos(u32ModuleIdx));
    }
    else
        return 0;
}

/**
  * @brief      Get internal multiple speed RC oscillator (MIRC) clock frequency
  *
  * @param      None
  *
  * @return     Internal multiple speed RC oscillator (MIRC) clock frequency
  *
  * @details    This function get internal multiple speed RC oscillator (MIRC) clock frequency. The frequency unit is Hz.
  */
uint32_t CLK_GetMIRCFreq(void)
{
    uint32_t u32Freq = 0UL;

    if((CLK->PWRCTL & CLK_PWRCTL_MIRCEN_Msk) == CLK_PWRCTL_MIRCEN_Msk)
    {
        switch (CLK->PWRCTL & CLK_PWRCTL_MIRCFSEL_Msk)
        {
        case CLK_PWRCTL_MIRCFSEL_1M:
            u32Freq = FREQ_1MHZ;
            break;
        case CLK_PWRCTL_MIRCFSEL_2M:
            u32Freq = FREQ_2MHZ;
            break;
        case CLK_PWRCTL_MIRCFSEL_4M:
            u32Freq = FREQ_4MHZ;
            break;
        case CLK_PWRCTL_MIRCFSEL_8M:
            u32Freq = FREQ_8MHZ;
            break;
        case CLK_PWRCTL_MIRCFSEL_12M:
            u32Freq = FREQ_12MHZ;
            break;
        case CLK_PWRCTL_MIRCFSEL_16M:
            u32Freq = FREQ_16MHZ;
            break;
        case CLK_PWRCTL_MIRCFSEL_24M:
            u32Freq = FREQ_24MHZ;
            break;
        case CLK_PWRCTL_MIRCFSEL_32M:
            u32Freq = FREQ_32MHZ;
            break;
        case CLK_PWRCTL_MIRCFSEL_40M:
            u32Freq = FREQ_40MHZ;
            break;
        default:
            u32Freq = __MIRC;
            break;
        }
    }
    else
    {
        u32Freq = 0UL;
    }

    return u32Freq;
}

/**
  * @brief      Disable MIRC
  *
  * @param      None
  *
  * @return     None
  *
  * @details    This function disable MIRC clock. \n
  *             The register write-protection function should be disabled before using this function.
  */
void CLK_DisableMIRC(void)
{
    CLK->PWRCTL &= ~(CLK_PWRCTL_MIRCEN_Msk);
}

static void MIRC_delay(uint32_t mirc_hclk)
{
    uint32_t nop_count, i;

    nop_count = mirc_hclk / FREQ_1MHZ;
    if (nop_count == 0)
    {
        __NOP();
    }
    else
    {
        nop_count = 5 * nop_count;
        for(i=0; i<nop_count; i++)
        {
            __NOP();
        }
    }
}

/**
  * @brief      Set MIRC frequency
  *
  * @param[in]  u32MircFreq is MIRC clock frequency. Including :
  *             - \ref CLK_PWRCTL_MIRCFSEL_1M
  *             - \ref CLK_PWRCTL_MIRCFSEL_2M
  *             - \ref CLK_PWRCTL_MIRCFSEL_4M
  *             - \ref CLK_PWRCTL_MIRCFSEL_8M
  *             - \ref CLK_PWRCTL_MIRCFSEL_12M
  *             - \ref CLK_PWRCTL_MIRCFSEL_16M
  *             - \ref CLK_PWRCTL_MIRCFSEL_24M
  *             - \ref CLK_PWRCTL_MIRCFSEL_32M
  *             - \ref CLK_PWRCTL_MIRCFSEL_40M
  *
  * @return     MIRC frequency
  *
  * @details    This function is used to configure PWRCTL register to set specified MIRC frequency. \n
  *             The register write-protection function should be disabled before using this function.
  */
uint32_t CLK_EnableMIRC(uint32_t u32MircFreq)
{
    uint32_t mirc_status, mirc_hclk;

    mirc_status = (CLK->PWRCTL & CLK_PWRCTL_MIRCEN_Msk);
    mirc_hclk   = CLK_GetCPUFreq();

    /* M2U51 series supports MIRC switching without needing to disable MIRC first. */
    /* Other series could need disable MIRC first to avoid unstable when setting MIRC. */
    /* CLK_DisableMIRC(); */

    /* Change MIRC frequency and then enable */
    CLK->PWRCTL = (CLK->PWRCTL & ~(CLK_PWRCTL_MIRCFSEL_Msk)) | (u32MircFreq);
    CLK->PWRCTL |= CLK_PWRCTL_MIRCEN_Msk;

    /* M2U51 series requires a delay before checking the MIRC stable flag. */
    if (mirc_status == CLK_PWRCTL_MIRCEN_Msk)
    {
        MIRC_delay(mirc_hclk);
    }
    else
    {
        MIRC_delay(mirc_hclk);
        CLK_WaitClockReady(CLK_STATUS_MIRCSTB_Msk);
        MIRC_delay(mirc_hclk);
    }

    /* Wait for MIRC clock stable */
    CLK_WaitClockReady(CLK_STATUS_MIRCSTB_Msk);

    /* Return actual MIRC output clock frequency */
    return CLK_GetMIRCFreq();
}

/*@}*/ /* end of group CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group CLK_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2024 Nuvoton Technology Corp. ***/
