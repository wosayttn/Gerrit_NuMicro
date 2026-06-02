/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to enter to different Power-down mode and wake-up by Wake-up Timer.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PDMD_FLAG_ADDR      0x20004FFC

static uint32_t s_u32PowerDownMode;
static volatile uint32_t s_u32WKTMR_INT;

void WKTMR_Init(void);
void PowerDownFunction(void);
int32_t CheckPowerSource(void);
void SYS_Init(void);
void UART0_Init(void);

/**
 * @brief       IRQ Handler for ETI Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The ETI_IRQHandler is default IRQ of ETI
 */
void ETI_IRQHandler(void)
{
    uint32_t u32PMUINTS = CLK->PMUINTS;

    /* To check if Wake-up Timer interrupt occurred */
    if (u32PMUINTS & CLK_PMUINTS_WKTMRIF_Msk)
    {
        /* Clear Wake-up Timer interrupt flag */
        CLK->PMUINTS = CLK_PMUINTS_WKTMRIF_Msk;
    }

    s_u32WKTMR_INT++;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for Wake-up Timer setting                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void WKTMR_Init(void)
{
    /* Enable Wake-up Timer */
    CLK_ENABLE_WKTMR();

    /* Select Wake-up Timer time-out interval */
    CLK_SET_WKTMR_INTERVAL(CLK_PMUCTL_WKTMRIS_32768);

    /* Enable Wake-up Timer interrupt */
    CLK->PMUINTC |= CLK_PMUINTC_WKTMRIE_Msk;

    /* Enable Wake-up Timer interrupt NVIC */
    NVIC_EnableIRQ(ETI_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Select Power-down mode */
    CLK_SetPowerDownMode(s_u32PowerDownMode);

    /* Forces a write of all user-space buffered data for the given output */
    fflush(stdout);

    /* To check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(DEBUG_PORT)

    if (--u32TimeOutCnt == 0) break;

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for Check Power Manager Status                                                                  */
/*-----------------------------------------------------------------------------------------------------------*/
int32_t CheckPowerSource(void)
{
    /* Get Power Manager Status */
    uint32_t u32Status = CLK_GetPMUWKSrc();

    /* Clear wake-up status flag */
    CLK->PMUSTS = CLK_PMUSTS_CLRWK_Msk;

    /* Check Power Manager Status is wake-up by Wake-up Timer */
    if (u32Status == 0)
    {
        /* It is the start of sample code */
        printf("\n\nCPU @ %dHz\n", SystemCoreClock);
        printf("+------------------------------------------+\n");
        printf("|    Power-down and Wake-up Sample Code    |\n");
        printf("+------------------------------------------+\n");
    }
    else if (u32Status & CLK_PMUSTS_TMRWK_Msk)
    {
        s_u32PowerDownMode = M32(PDMD_FLAG_ADDR);

        switch (s_u32PowerDownMode)
        {
            case CLK_PMUCTL_PDMSEL_SPD0:

                /* Wake-up from Standby Power-down Mode */
                printf("Wake-up from SPD0!\n");

                /* Next Power-down Mode is Deep Power-down Mode */
                M32(PDMD_FLAG_ADDR) = CLK_PMUCTL_PDMSEL_SPD1;
                break;

            case CLK_PMUCTL_PDMSEL_SPD1:

                /* Wake-up from Deep Power-down Mode */
                printf("Wake-up from SPD1!\n");

                /* End of sample code and clear Power-down Mode flag */
                printf("\nSample code end. Press Reset Button and continue.\n");
                M32(PDMD_FLAG_ADDR) = 0;
                return 1;
        }
    }

    return 0;
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set core clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                */
    /*------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set PC multi-function pin for CLKO(PC.13) */
    SET_CLKO_PC13();

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32TimeOutCnt;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Release I/O hold status after SPD wake-up */
    CLK_RELEASE_GPIO();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Unlock protected registers before setting wake-up source and Power-down mode */
    SYS_UnlockReg();

    /* Enable Clock Output function, output clock is stopped in Power-down mode */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 3, 0);

    /* Get power manager wake-up source */
    if (CheckPowerSource() != 0)
        goto lexit;

    /* Wake-up Timer setting */
    WKTMR_Init();

    /*
        This sample code will enter to different Power-down mode and wake-up by Wake-up Timer:
        1. Normal Power-down mode (NPD1).
        2. Low Leakage Power-down mode (NPD2).
        3. Fast Wake-up Power-down mode (NPD0).
        4. Standby Power-down mode (SPD0).
        5. Low Leakage Standby Power-down mode (SPD1).
    */
    while (1)
    {

        /* Select Power-down mode */
        s_u32PowerDownMode = M32(PDMD_FLAG_ADDR);

        switch (s_u32PowerDownMode)
        {
            case CLK_PMUCTL_PDMSEL_NPD0:
                printf("\nSystem enters to NPD0 power-down mode ... ");
                break;

            case CLK_PMUCTL_PDMSEL_NPD1:
                printf("\nSystem enters to NPD1 power-down mode ... ");
                break;

            case CLK_PMUCTL_PDMSEL_NPD2:
                printf("\nSystem enters to NPD2 power-down mode ... ");
                break;

            case CLK_PMUCTL_PDMSEL_SPD0:
                printf("\nSystem enters to SPD0 power-down mode ... ");
                break;

            case CLK_PMUCTL_PDMSEL_SPD1:
                printf("\nSystem enters to SPD1 power-down mode ... ");
                break;

            default:
                printf("\nInit sample code. Press Reset Button and continue.\n");
                M32(PDMD_FLAG_ADDR) = 0;
                goto lexit;
        }

        /* Enter to Power-down mode */
        s_u32WKTMR_INT = 0;
        PowerDownFunction();
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (s_u32WKTMR_INT == 0)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for Wake-up Timer interrupt time-out!");
                break;
            }
        }

        printf("Wake-up!\n");

        /* Select next Power-down mode */
        switch (s_u32PowerDownMode)
        {
            case CLK_PMUCTL_PDMSEL_NPD1:
                M32(PDMD_FLAG_ADDR) = CLK_PMUCTL_PDMSEL_NPD2;
                break;

            case CLK_PMUCTL_PDMSEL_NPD2:
                M32(PDMD_FLAG_ADDR) = CLK_PMUCTL_PDMSEL_NPD0;
                break;

            case CLK_PMUCTL_PDMSEL_NPD0:
                M32(PDMD_FLAG_ADDR) = CLK_PMUCTL_PDMSEL_SPD0;
                break;

            default:
                printf("\nInit sample code. Press Reset Button and continue.\n");
                M32(PDMD_FLAG_ADDR) = 0;
                goto lexit;
        }
    }

lexit:

    while (1);
}
