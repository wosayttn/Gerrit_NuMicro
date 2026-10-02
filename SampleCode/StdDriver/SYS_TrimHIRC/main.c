/****************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to use LXT to trim HIRC
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

/*--------------------------------------------------------------------------------------------------------*/
/*  IRCTrim IRQ Handler                                                                                   */
/*--------------------------------------------------------------------------------------------------------*/
/**
  * @brief      This function get HIRC trim status
  * @param      None
  * @retval     BIT1 Trim Failure Interrupt
  * @retval     BIT2 LXT Clock error Interrupt
  * @details    This macro get HIRC trim interrupt status register.
  */
void IRC_IRQHandler()
{
    if(SYS->IRCTISTS & SYS_IRCTISTS_TFAILIF_Msk)   /* Get Trim Failure Interrupt */
    {
        /* Display HIRC trim status */
        printf("HIRC Trim Failure Interrupt\n");
        /* Clear Trim Failure Interrupt */
        SYS->IRCTISTS = SYS_IRCTISTS_TFAILIF_Msk;
    }
    if(SYS->IRCTISTS & SYS_IRCTISTS_CLKERRIF_Msk)   /* Get LXT Clock Error Interrupt */
    {
        /* Display HIRC trim status */
        printf("LXT Clock Error Interrupt\n");
        /* Clear LXT Clock Error Interrupt */
        SYS->IRCTISTS = SYS_IRCTISTS_CLKERRIF_Msk;
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 96MHz from PLL */
    CLK_SetCoreClock(FREQ_96MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    UART_Open(UART0, 115200); /* Configure UART0 and set UART0 Baud rate */
}

void TrimHIRC()
{
    /*  Enable IRC Trim, set HIRC clock and enable interrupt */
    SYS->IRCTIEN |= (SYS_IRCTIEN_CLKEIEN_Msk | SYS_IRCTIEN_TFAILIEN_Msk);
    SYS->IRCTCTL = (SYS->IRCTCTL & ~SYS_IRCTCTL_FREQSEL_Msk)| 0x1;

    CLK_SysTickDelay(2000); /* Waiting for HIRC Frequency Lock */

    /* Get HIRC Frequency Lock */
    while(1)
    {
        if(SYS->IRCTISTS & SYS_IRCTISTS_FREQLOCK_Msk)
        {
            printf("HIRC Frequency Lock\n");
            SYS->IRCTISTS = SYS_IRCTISTS_FREQLOCK_Msk;     /* Clear Trim Lock */
            break;
        }
    }

    /* Enable CLKO and output frequency = HIRC  */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HIRC, 0, 1);

}

int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O
    After SYS_Init() will issue SYS_LockReg()
    to lock protected register. If user want to write
    protected register, please issue SYS_UnlockReg()
    to unlock protected register if necessary */
    SYS_Init();

    UART0_Init(); /* Init UART0 for printf */

    /* Enable Interrupt */
    NVIC_EnableIRQ(IRC_IRQn);
    /* Trim HIRC to 12MHz */
    TrimHIRC();

    /* Disable IRC Trim */
    SYS->IRCTCTL = 0;
    printf("Disable IRC Trim\n");
    while(1);
}


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
