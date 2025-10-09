/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Bank Remap sample code(Bank0 App).
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "NuDB_common.h"

#define PLL_CLOCK    96000000
#define TEST_MODE    1


static volatile uint32_t s_u32ExecBank;
static volatile uint32_t s_u32GetSum;

void ResetCPU(void);
void WDT_IRQHandler(void);
void SYS_Init(void);
int32_t  SelfTest(void);
uint32_t  FuncCrc32(uint32_t u32Start, uint32_t u32Len);


uint32_t  FuncCrc32(uint32_t u32Start, uint32_t u32Len)
{
    uint32_t  u32Idx, u32Data = 0UL;

    /* WDTAT_RVS, CHECKSUM_RVS, CHECKSUM_COM */
    for(u32Idx = 0; u32Idx < u32Len; u32Idx += 4)
    {
        u32Data += *(uint32_t *)(u32Start + u32Idx);
    }
    u32Data = 0xFFFFFFFF - u32Data + 1UL;

    return u32Data;
}

void ResetCPU(void)
{
    SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
    SYS->IPRST0 &= ~SYS_IPRST0_CPURST_Msk;
}

void WDT_IRQHandler(void)
{
    WDT_RESET_COUNTER();

    if(WDT_GET_TIMEOUT_INT_FLAG() == 1)
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();
    }


}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable WDT module clock */
    CLK_EnableModuleClock(WDT_MODULE);
    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins */
    Uart0DefaultMPF();
}


int32_t  SelfTest(void)
{
#if TEST_MODE
    uint8_t u8GetCh;

    printf("\n Self test pass? y/n \n");
    u8GetCh = (uint8_t)getchar();
    printf("\n User select [%c] \n", u8GetCh);

    /* Let user select to test pass or fail condition*/
    if(u8GetCh == 'y')
    {
        printf("\n Self test pass!!! \n\n");
        return 0;
    }
    else
    {
        printf("\n Self test fail!!! \n\n");
        return -1;
    }
#else
    s_u32GetSum = RMC_GetChkSum(APP_BASE, APP_SIZE);
    g_u32KeepSum = RMC_Read(FW_CRC_BASE);
    printf("\n GetSum = 0x%x, Keep Sum = 0x%x \n", s_u32GetSum, g_u32KeepSum);

    if(s_u32GetSum == g_u32KeepSum)
    {
        printf("\n Self test pass!!! \n");
        return 0;
    }
    else
    {
        printf("\n Self test fail!!! \n");
        return -1;
    }
#endif
}


int main()
{
    uint32_t i;
    int32_t i32Ret;

    /* Disable register write-protection function */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Enable ISP and APROM update */
    RMC_ENABLE_ISP();
    RMC_ENABLE_AP_UPDATE();

    /* Unlock protected registers */
    SYS_UnlockReg();

    NVIC_EnableIRQ(WDT_IRQn);

    /* Configure WDT settings and start WDT counting */
    WDT_Open(WDT_TIMEOUT_2POW18, WDT_RESET_DELAY_18CLK, 1, 0);

    /* Enable WDT interrupt function */
    WDT_EnableInt();

    do
    {
        printf("\n\n");
        printf("+------------------------+\n");
        printf("|  Boot from 0x%08X  |\n", RMC_GetVECMAP());
        printf("+------------------------+\n");

        /* Check CPU run at Bank0 or Bank1 */
        s_u32ExecBank = (uint32_t)((RMC->ISPSTS & RMC_ISPSTS_FBS_Msk) >> RMC_ISPSTS_FBS_Pos);
        printf("\n BANK%d APP processing (Active firmware)\n", s_u32ExecBank);

        /* Execute firmware self test */
        i32Ret = SelfTest();

        if(i32Ret == 0)
        {
            /* Normal test condition*/
            for(i = 0; i < 1000; i++)
            {
                printf(" Firmware processing....  cnt[%d]\r", i);
                s_u32GetSum = FuncCrc32(APP_BASE, APP_SIZE);
            }
        }
        else
        {
            /* Failure test condition, will reset by WDT and start from Bank0 loader */
            printf("\n Enter power down...\n");
            CLK_SysTickDelay(2000);
            CLK_PowerDown();
        }

    }
    while(1);


}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
