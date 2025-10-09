/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 16/10/17 2:06p $
 * @brief
 *           Implement a code and execute in SRAM to program embedded Flash.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define APROM_TEST_BASE             0x3000
#define TEST_PATTERN                0x5A5A5A5A

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

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int32_t main(void)
{
    uint32_t u32Data, u32RData;
    uint32_t u32Addr;
    uint32_t u32i;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|      RMC Write/Read code execute in SRAM Sample Code      |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
       This sample code is used to demonstrate how to implement a code to execute in SRAM.
       By setting scatter loading file (scatter.scf),
       RO code is placed to 0x20000000 ~ 0x20001fff with RW is placed to 0x20002000 ~ 0x20003fff.
    */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable RMC ISP function. Before using RMC function, it should unlock system register first. */
    RMC_Open();

    /* Update APROM enabled */
    RMC_ENABLE_AP_UPDATE();

    /* The ROM address for write/read demo */
    u32Addr = 0x4000;

    for(u32i = 0; u32i < 0x100; u32i += 4)
    {

        /* Write Demo */
        u32Data = u32i + 0x12345678;
        RMC_Write(u32Addr + u32i, u32Data);

        if((u32i & 0xf) == 0)
            printf(".");

        /* Read Demo */
        u32RData = RMC_Read(u32Addr + u32i);

        if(u32Data != u32RData)
        {
            printf("[Read/Write FAIL]\n");
            break;
        }
    }

    printf("\nISP function run at SRAM finished\n");

    /* Disable RMC ISP function */
    RMC->ISPCTL &=  ~RMC_ISPCTL_ISPEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nRMC Sample Code Completed.\n");

    while(1);

}
/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/


