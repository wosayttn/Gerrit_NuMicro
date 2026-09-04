/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 23/02/24 4:40p $
 * @brief    Demonstrate how to use RMC Read-All-One ISP command to verify APROM/LDROM pages are all 0xFFFFFFFF or not.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

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
    uint32_t    u32Ret;                   /* return value */
    uint32_t    u32Data;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("+------------------------------------------+\n");
    printf("|        RMC_ReadAllOne Sample Demo        |\n");
    printf("+------------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable RMC ISP function. Before using RMC function, it should unlock system register first. */
    RMC_Open();

    u32Data = RMC_ReadCID();           /* Read company ID. Should be 0xDA. */
    if (g_RMC_i32ErrCode != 0)
    {
        printf("RMC_ReadCID failed!\n");
        goto lexit;
    }
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = RMC_ReadPID();           /* Read product ID. */
    if (g_RMC_i32ErrCode != 0)
    {
        printf("RMC_ReadPID failed!\n");
        goto lexit;
    }
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    /* Read User Configuration CONFIG0 */
    printf("  User Config 0 ......................... [0x%08x]\n", RMC_Read(RMC_CONFIG_BASE));
    if (g_RMC_i32ErrCode != 0)
    {
        printf("RMC_Read(RMC_CONFIG_BASE) failed!\n");
        goto lexit;
    }

    /* Read User Configuration CONFIG1 */
    printf("  User Config 1 ......................... [0x%08x]\n", RMC_Read(RMC_CONFIG_BASE + 4));
    if (g_RMC_i32ErrCode != 0)
    {
        printf("RMC_Read(RMC_CONFIG_BASE+4) failed!\n");
        goto lexit;
    }

    RMC_ENABLE_LD_UPDATE();            /* Enable LDROM update. */

    if (RMC_Erase(RMC_LDROM_BASE) != 0)     /* Erase LDROM page 0. */
    {
        printf("RMC_Erase(RMC_LDROM_BASE) failed!\n");
        goto lexit;
    }

    /* Run and check flash contents are all 0xFFFFFFFF. */
    u32Ret = RMC_CheckAllOne(RMC_LDROM_BASE, RMC_FLASH_PAGE_SIZE);
    if (g_RMC_i32ErrCode != 0)
    {
        printf("RMC_CheckAllOne failed!\n");
        goto lexit;
    }

    if(u32Ret == READ_ALLONE_YES)                   /* return value READ_ALLONE_YES means all flash contents are 0xFFFFFFFF */
        printf("READ_ALLONE_YES success.\n");    /* RMC_CheckAllOne() READ_ALLONE_YES passed on LDROM page 0. */
    else
        printf("READ_ALLONE_YES failed!\n");     /* RMC_CheckAllOne() READ_ALLONE_YES failed on LDROM page 0. */

    if (RMC_Write(RMC_LDROM_BASE, 0) != 0)       /* program a 0 to LDROM to make it not all 0xFFFFFFFF. */
    {
        printf("RMC_Write RMC_LDROM_BASE failed!\n");
        goto lexit;
    }

    /* Run and check flash contents are not all 0xFFFFFFFF. */
    u32Ret = RMC_CheckAllOne(RMC_LDROM_BASE, RMC_FLASH_PAGE_SIZE);
    if (g_RMC_i32ErrCode != 0)
    {
        printf("RMC_CheckAllOne failed!\n");
        goto lexit;
    }
    if(u32Ret == READ_ALLONE_NOT)
        printf("READ_ALLONE_NOT success.\n");   /* RMC_CheckAllOne() READ_ALLONE_NOT passed on LDROM page 0. */
    else
        printf("READ_ALLONE_NOT failed!\n");    /* RMC_CheckAllOne() READ_ALLONE_NOT failed on LDROM page 0. */

    printf("\nRMC Read-All-One test done.\n");

lexit:
    RMC_Close();                       /* Disable RMC ISP function */

    SYS_LockReg();                     /* Lock protected registers */

    while(1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/


