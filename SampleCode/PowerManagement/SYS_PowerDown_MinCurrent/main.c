/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to minimize power consumption when entering power down mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/

/*
// <o0> Power-down Mode
//      <0=> NPD1
//      <1=> NPD2
//      <2=> NPD0
//      <4=> SPD0
//      <5=> SPD1
*/
#define SET_PDMSEL    0

/*
// <o0> SPD mode SRAM bnak0 retention
//      <0=> SPD no retention
//      <1=> SPD retention
*/
#define SET_SRETSEL_BANK0   (1 << 0)

/*
// <o0> SPD mode SRAM bnak1 retention
//      <0=> SPD no retention
//      <1=> SPD retention
*/
#define SET_SRETSEL_BANK1   (1 << 1)

/*
// <o0> SPD mode SRAM bnak2 retention
//      <0=> SPD no retention
//      <1=> SPD retention
*/
#define SET_SRETSEL_BANK2   (0 << 2)

/*
// <o0> LVR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LVR       0

/*
// <o0> POR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_POR       0

/*
// <o0> LIRC
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LIRC      0

/*
// <o0> LXT
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LXT       0


#define GPIO_P0_TO_P15      0xFFFF


void PowerDownFunction(void);
void GPB_IRQHandler(void);
int32_t LvrSetting(void);
void PorSetting(void);
int32_t LircSetting(void);
int32_t LxtSetting(void);
void SYS_Init(void);
void UART0_Init(void);


/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(DEBUG_PORT)

    if (--u32TimeOutCnt == 0) break;

    /* Select Power-down mode */
    CLK_SetPowerDownMode(SET_PDMSEL << CLK_PMUCTL_PDMSEL_Pos);

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

/**
 * @brief       GPIO PB IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PB default IRQ
 */
void GPB_IRQHandler(void)
{
    volatile uint32_t u32temp;

    /* To check if PB.2 interrupt occurred */
    if (GPIO_GET_INT_FLAG(PB, BIT2))
    {
        GPIO_CLR_INT_FLAG(PB, BIT2);
        printf("PB2 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        u32temp = PB->INTSRC;
        PB->INTSRC = u32temp;
        printf("Un-expected interrupts.\n");
    }
}

int32_t LvrSetting(void)
{
    uint32_t u32TimeOutCnt;

    if (SET_LVR == 0)
    {
        /* Disable LVR and wait for LVR stable flag is cleared */
        SYS_DISABLE_LVR();
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (SYS->BODCTL & SYS_BODCTL_LVRRDY_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for LVR disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        /* Enable LVR and wait for LVR stable flag is set */
        SYS_ENABLE_LVR();
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while ((SYS->BODCTL & SYS_BODCTL_LVRRDY_Msk) == 0)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for LVR enable time-out!\n");
                return -1;
            }
        }
    }

    return 0;
}

void PorSetting(void)
{
    if (SET_POR == 0)
    {
        /* Disable POR */
        SYS_DISABLE_POR();
    }
    else
    {
        /* Enable POR */
        SYS_ENABLE_POR();
    }
}

int32_t LircSetting(void)
{
    uint32_t u32TimeOutCnt;

    if (SET_LIRC == 0)
    {
        /* Disable LIRC and wait for LIRC stable flag is cleared */
        CLK_DisableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (CLK->STATUS & CLK_STATUS_LIRCSTB_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for LIRC disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        /* Enable LIRC and wait for LIRC stable flag is set */
        CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

        if (CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk) == 0)
        {
            printf("Wait for LIRC enable time-out!\n");
            return -1;
        }
    }

    return 0;
}

int32_t LxtSetting(void)
{
    uint32_t u32TimeOutCnt;

    if (SET_LXT == 0)
    {
        /* Disable LXT and wait for LXT stable flag is cleared */
        CLK_DisableXtalRC(CLK_PWRCTL_LXTEN_Msk);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (CLK->STATUS & CLK_STATUS_LXTSTB_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for LXT disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        /* Enable LXT and wait for LXT stable flag is set */
        CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

        if (CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk) == 0)
        {
            printf("Wait for LXT enable time-out!\n");
            return -1;
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

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32TimeOutCnt, u32PMUSTS;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Clear SPD0/SPD1 mode wake-up status for entering SPD0/SPD1 mode again */
    u32PMUSTS = CLK->PMUSTS;

    if (u32PMUSTS)
    {
        /* Release I/O hold status for SPD mode */
        CLK_RELEASE_GPIO();

        /* Clear SPD0/SPD1 mode wake-up status */
        CLK->PMUSTS = CLK_PMUSTS_CLRWK_Msk;
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (CLK->PMUSTS)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for SPD0/SPD1 mode wake-up status is cleared time-out!\n");
                goto lexit;
            }
        }
    }

    /* Check SPD0/SPD1 mode PB.2 falling-edge wake-up event */
    if (u32PMUSTS & CLK_PMUSTS_GPBWK0_Msk)
    {
        printf("System waken-up done.\n\n");

        while (1);
    }

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------------+\n");
    printf("|  SYS_PowerDown_MinCurrent and Wake-up by PB.2 Sample Code         |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    printf("+-------------------------------------------------------------------+\n");
    printf("| Operating sequence                                                |\n");
    printf("|  1. Remove all continuous load, e.g. LED.                         |\n");
    printf("|  2. Configure all GPIO as Quasi-bidirectional Mode                |\n");
    printf("|  3. Disable analog function, e.g. POR module                      |\n");
    printf("|  4. Disable unused clock, e.g. LIRC                               |\n");
    printf("|  5. Disable SRAM retention for SPD mode                           |\n");
    printf("|  6. Enter to Power-Down                                           |\n");
    printf("|  7. Wait for PB.2 falling-edge interrupt event to wake-up the MCU |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    /* Set function pin to GPIO mode except UART pin to print message */
    SYS->GPA_MFP0 = 0;
    SYS->GPA_MFP1 = 0;
    SYS->GPA_MFP2 = 0;
    SYS->GPA_MFP3 = 0;
    SYS->GPB_MFP0 = 0;
    SYS->GPB_MFP1 = 0;
    SYS->GPB_MFP2 = 0;
    SYS->GPB_MFP3 = SYS_GPB_MFP3_PB13MFP_UART0_TXD | SYS_GPB_MFP3_PB12MFP_UART0_RXD;
    SYS->GPC_MFP0 = 0;
    SYS->GPC_MFP1 = 0;
    SYS->GPC_MFP2 = 0;
    SYS->GPC_MFP3 = 0;
    SYS->GPD_MFP0 = 0;
    SYS->GPD_MFP1 = 0;
    SYS->GPD_MFP2 = 0;
    SYS->GPD_MFP3 = 0;
    SYS->GPE_MFP0 = 0;
    SYS->GPE_MFP1 = 0;
    SYS->GPE_MFP2 = 0;
    SYS->GPE_MFP3 = 0;
    SYS->GPF_MFP0 = SYS_GPF_MFP0_PF1MFP_ICE_CLK | SYS_GPF_MFP0_PF0MFP_ICE_DAT;
    SYS->GPF_MFP1 = 0;
    SYS->GPF_MFP2 = 0;
    SYS->GPF_MFP3 = 0;
    SYS->GPG_MFP0 = 0;
    SYS->GPG_MFP1 = 0;
    SYS->GPG_MFP2 = 0;
    SYS->GPG_MFP3 = 0;
    SYS->GPH_MFP0 = 0;
    SYS->GPH_MFP1 = 0;
    SYS->GPH_MFP2 = 0;

    /*
        Configure all GPIO as Quasi-bidirectional Mode. They are default output high.
    */
    GPIO_SetMode(PA, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PB, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PC, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PD, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PE, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PF, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PG, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PH, GPIO_P0_TO_P15, GPIO_MODE_QUASI);

    /* Unlock protected registers for Power-down and wake-up setting */
    SYS_UnlockReg();

    /* LVR setting */
    if (LvrSetting() < 0) goto lexit;

    /* POR setting */
    PorSetting();

    /* LIRC setting */
    if (LircSetting() < 0) goto lexit;

    /* LXT setting */
    if (LxtSetting() < 0) goto lexit;

    /* Select SPD mode SRAM retention size */
    CLK->PMUCTL = (CLK->PMUCTL & (~CLK_PMUCTL_SRETSEL_Msk)) |
                  ((SET_SRETSEL_BANK0 | SET_SRETSEL_BANK1 | SET_SRETSEL_BANK2) << CLK_PMUCTL_SRETSEL_Pos);

    /* Wake-up source configuration */
    if ((SET_PDMSEL == CLK_PMUCTL_PDMSEL_NPD0) ||
            (SET_PDMSEL == CLK_PMUCTL_PDMSEL_NPD1) ||
            (SET_PDMSEL == CLK_PMUCTL_PDMSEL_NPD2))
    {
        /* Configure PB.2 as Quasi mode and enable interrupt by falling edge trigger */
        GPIO_SetMode(PB, BIT2, GPIO_MODE_QUASI);
        GPIO_EnableInt(PB, 2, GPIO_INT_FALLING);
        NVIC_EnableIRQ(GPB_IRQn);
    }
    else if ((SET_PDMSEL == CLK_PMUCTL_PDMSEL_SPD0) ||
             (SET_PDMSEL == CLK_PMUCTL_PDMSEL_SPD1))
    {
        /* Enable wake-up pin PB.2 falling edge wake-up at SPD mode */
        CLK_EnableSPDWKPin(1, 2, CLK_SPDWKPIN0_FALLING, CLK_SPDWKPIN0_DEBOUNCEDIS);
    }
    else
    {
        printf("Unknown Power-down mode!\n");
        goto lexit;
    }

    printf("Press any key to enter power-down mode.\n");
    (void)getchar();

    /* Enter to Power-down mode */
    if (SET_PDMSEL == CLK_PMUCTL_PDMSEL_NPD0)        printf("Enter to NPD0 Power-Down...\n");
    else if (SET_PDMSEL == CLK_PMUCTL_PDMSEL_NPD1)   printf("Enter to NPD1 Power-Down...\n");
    else if (SET_PDMSEL == CLK_PMUCTL_PDMSEL_NPD2)   printf("Enter to NPD2 Power-Down...\n");
    else if (SET_PDMSEL == CLK_PMUCTL_PDMSEL_SPD0)   printf("Enter to SPD0 Power-Down...\n");
    else if (SET_PDMSEL == CLK_PMUCTL_PDMSEL_SPD1)   printf("Enter to SPD1 Power-Down...\n");

    UART_WAIT_TX_EMPTY(UART0) {}

    PowerDownFunction();

    /* Waiting for PB.2 falling-edge interrupt event */
    printf("System waken-up done.\n\n");

lexit:

    while (1);
}
