/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A multi-boot system to boot different applications from different address.
 *          This sample code implemented 1 LDROM code and 4 APROM code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

int32_t SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*------------------------------------------------------------------------*/
    /* Init System Clock                                                      */
    /*------------------------------------------------------------------------*/
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 to HCLK/2 */
    CLK_SET_PCLK0DIV(CLK_PCLKDIV_APB0DIV_DIV2);
    /* Set PCLK1 to HCLK/2 */
    CLK_SET_PCLK1DIV(CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                */
    /*------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();

    return 0;
}

int32_t main(void)
{
    uint8_t u8Char;     /* variables */

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    /*
        This sample code shows how to boot with different firmware images in APROM.
        In the code, VECMAP is used to implement multi-boot function. Software set VECMAP
        to remap page of VECMAP to 0x0 ~ 0x3FF.
        NOTE: VECMAP only valid when CBS = 00'b or 10'b.

        To use this sample code, please:
        1. Build all targets and download to device individually. The targets are:
            FMC_MultiBoot_0x0,     RO=0x0
            FMC_MultiBoot_0x60000, RO=0x40000
            FMC_MultiBoot_0x80000, RO=0x80000
            FMC_MultiBoot_0xC0000, RO=0xC0000
            FMC_MultiBoot_LDROM,   RO=0xF100000
        2. Reset MCU to execute FMC_MultiBoot.

    */
    printf("\n\n");
    printf("+----------------------------------------------+\n");
    printf("|      Multi-Boot Sample Code (0x%08X)     |\n", FMC_GetVECMAP());
    printf("+----------------------------------------------+\n");
    SYS_UnlockReg();    /* Unlock protected registers */
    FMC_Open();         /* Enable FMC ISP function */

    printf("VECMAP: 0x%08X\n", FMC_GetVECMAP());
    printf("PC    : 0x%08X\n", __PC());

    printf("Select boot image: \n");
    printf("[0] Boot from 0x40000\n");
    printf("[1] Boot from 0x80000\n");
    printf("[2] Boot from 0xC0000\n");
    printf("[3] Boot from 0xF100000\n");
    printf("[Others] Boot from 0x0\n");
    u8Char = getchar();
    printf("Select %c\n", u8Char);

    switch (u8Char)
    {
        case '0':
            FMC_SetVectorPageAddr(0x40000);
            break;

        case '1':
            FMC_SetVectorPageAddr(0x80000);
            break;

        case '2':
            FMC_SetVectorPageAddr(0xC0000);
            break;

        case '3':
            FMC_SetVectorPageAddr(FMC_LDROM_BASE);
            break;

        default:
            FMC_SetVectorPageAddr(0x0);
            break;
    }

    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_SetVectorPageAddr failed !\n");
        goto lexit;
    }

    /* Use reset CPU only or reset system to reset to new vector page */
#if 0
    printf("SYS_ResetCPU\n");
    UART_WAIT_TX_EMPTY(DEBUG_PORT);
    /* Reset CPU only to reset to new vector page */
    SYS_ResetCPU();
#else
    printf("NVIC_SystemReset\n");
    UART_WAIT_TX_EMPTY(DEBUG_PORT);
    /* Reset System to reset to new vector page. */
    NVIC_SystemReset();
#endif

lexit:

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
