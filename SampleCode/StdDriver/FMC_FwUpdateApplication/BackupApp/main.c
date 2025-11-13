/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Bank Remap sample code(Bank/Segment0 App).
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "../common/inc/NuDB_common.h"


static volatile uint32_t s_u32ExecBank;
static volatile uint32_t s_u32GetSum;

void WDT0_IRQHandler(void);
void SYS_Init(void);
int32_t  SelfTest(void);
uint32_t  FuncCrc32(uint32_t u32Start, uint32_t u32Len);


uint32_t  FuncCrc32(uint32_t u32Start, uint32_t u32Len)
{
    uint32_t  u32Idx, u32Data = 0UL;

    for(u32Idx = 0; u32Idx < u32Len; u32Idx += 4)
    {
        u32Data += *(uint32_t *)(u32Start + u32Idx);
    }
    u32Data = 0xFFFFFFFF - u32Data + 1UL;

    return u32Data;
}

void WDT0_IRQHandler(void)
{
    WDT_RESET_COUNTER(WDT0);

    if(WDT_GET_TIMEOUT_INT_FLAG(WDT0) == 1)
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG(WDT0);
    }
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_180MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;

    /* Enable WDT0 module clock */
    CLK_EnableModuleClock(WDT0_MODULE);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
}


int32_t  SelfTest(void)
{
    printf("\n Self test pass!!! \n");
    return 0;
}


int main()
{
    uint32_t i;
    int32_t i32Ret;

    /* Disable register write-protection function */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Set Vector Table Offset Register */
    SCB->VTOR = APP_BASE;

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Enable ISP and APROM update */
    FMC_ENABLE_ISP();
    FMC_ENABLE_AP_UPDATE();

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable WDT interrupt NVIC */
    NVIC_EnableIRQ(WDT0_IRQn);

    /* Configure WDT settings and start WDT counting */
    WDT_Open(WDT0, WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_18CLK, 1, 0);

    /* Enable WDT interrupt function */
    WDT_EnableInt(WDT0);

    do
    {
        printf("\n\n");
        printf("+------------------------+\n");
        printf("|  Boot from 0x%08X  |\n", FMC_GetVECMAP());
        printf("+------------------------+\n");

        /* Check CPU run at Bank/Segment 0 or 1 */
        s_u32ExecBank = (uint32_t)((FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) >> FMC_ISPSTS_FBS_Pos);
        printf("\n Bank/Segment%d APP processing (Backup Firmware)\n", s_u32ExecBank);

        /* Execute firmware self test */
        i32Ret = SelfTest();

        if(i32Ret == 0)
        {
            /* Normal test condition */
            for(i = 0; i < 1000; i++)
            {
                printf(" Firmware processing....  cnt[%d]\r", i);
                s_u32GetSum = FuncCrc32(APP_BASE, APP_SIZE);
            }
        }
        else
        {
            /* Failure test condition, will reset by WDT and start from Bank/Segment 0 loader */
            printf("\n Enter power down...\n");
            CLK_SysTickDelay(2000);
            CLK_PowerDown();
        }

    }
    while(1);

}
