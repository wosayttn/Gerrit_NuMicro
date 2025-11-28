/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to select SRAM power mode in system Power-down mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define __M2L31_40KB_SRM__

#ifdef __M2L31_40KB_SRM__
    #define SRAM_REGION_NUMBER  (3)
#else
    #define SRAM_REGION_NUMBER  (7)
#endif

static volatile uint8_t s_u8IsINTEvent;

/*---------------------------------------------------------------------------------------------------------*/
/*  WDT IRQ Handler                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void WDT_IRQHandler(void)
{
    if(WDT_GET_TIMEOUT_INT_FLAG())
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();
    }

    if(WDT_GET_TIMEOUT_WAKEUP_FLAG())
    {
        /* Clear WDT time-out wake-up flag */
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG();
    }

    s_u8IsINTEvent = 1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* To check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(IsDebugFifoEmpty() == 0)
        if(--u32TimeOutCnt == 0) break;

    /* Enter to Power-down mode */
    CLK_PowerDown();
}


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Release I/O hold status */
    CLK->IOPDCTL = 1;

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

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set module clock */
    CLK_SetModuleClock(WDT_MODULE, LPSCC_CLKSEL0_WDTSEL_LIRC, 0);

    /* Enable module clock */
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_EnableModuleClock(CRC_MODULE);
    CLK_EnableModuleClock(LPSRAM_MODULE);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t au32SRAMCheckSum[SRAM_REGION_NUMBER] = {0};
    uint32_t au32SRAMCheckSum2[SRAM_REGION_NUMBER] = {0};
#if (SRAM_REGION_NUMBER == 7)
    uint32_t au32SRAMSize[SRAM_REGION_NUMBER] = { 8192, 16384, 16384, 32768, 32768, 65536, 8192};
#else
    uint32_t au32SRAMSize[SRAM_REGION_NUMBER] = { 8192, 16384, 16384};
#endif
    uint32_t u32Idx, u32Addr, u32SRAMStartAddr = 0;
    uint32_t u32TimeOutCnt;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+---------------------------------------+\n");
    printf("|       SRAM Power Mode Sample Code     |\n");
    printf("+---------------------------------------+\n\n");

    /*
        SRAM power mode in system Power-down mode can select as normal mode, retention mode,
        and shut down mode.
        If SRAM power mode select as shut down mode, SRAM data will not kept after
        Power-down and wake-up.
        This sample code will set
            SRAM region 0 to normal mode
            SRAM region 1 to normal mode
            SRAM region 2 to retention mode
            SRAM region 3 to retention mode
            SRAM region 4 to shut down mode
            SRAM region 5 to shut down mode
            SRAM region 6 to normal mode
        The SRAM region 0 checksum after wake-up will be different with checksum before entering to Power-down mode
            because SRAM region 0 include SRAM variables.
        The SRAM region 4 and 5 checksum after wake-up will be different with checksum before entering to Power-down mode.
    */

    /* Unlock protected registers before setting SRAM power mode */
    SYS_UnlockReg();

    /* Calculate SRAM checksum before entering to Power-down mode */
    printf("Calculate SRAM checksum before Power-down:\n");

    /* Specify SRAM region start address */
    u32SRAMStartAddr = SRAM_BASE;

    /* Calculate SRAM checksum */
    for(u32Idx = 0; u32Idx < SRAM_REGION_NUMBER; u32Idx++)
    {
        /* For M2L31, SRAM regison 6 is in LPSRAM. */
        if (u32Idx == 6)
        {
            u32SRAMStartAddr = LPSRAM_BASE;
        }

        /* Modify SRAM to non-default value. */
        if (u32Idx > 0)
        {
            outpw(u32SRAMStartAddr, 0x12345678);
        }

        /* Configure CRC controller for CRC-CRC32 mode */
        CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFF, CRC_CPU_WDATA_32);

        /* Start to execute CRC-CRC32 operation */
        for(u32Addr = u32SRAMStartAddr; u32Addr < (u32SRAMStartAddr + au32SRAMSize[u32Idx]); u32Addr += 4)
        {
            CRC_WRITE_DATA(inpw(u32Addr));
        }

        /* Record checksum result */
        au32SRAMCheckSum[u32Idx] = CRC_GetChecksum();

        /* Specify next SRAM region start address */
        u32SRAMStartAddr = u32Addr;
    }

    printf("SRAM Region 0 Checksum [0x%08X]\n", au32SRAMCheckSum[0]);
    printf("SRAM Region 1 Checksum [0x%08X]\n", au32SRAMCheckSum[1]);
    printf("SRAM Region 2 Checksum [0x%08X]\n", au32SRAMCheckSum[2]);
    /* Select SRAM power mode in system Power-down Mode */
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM0PM_Msk, SYS_SRAMPC0_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM1PM_Msk, SYS_SRAMPC0_SRAM_NORMAL);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM2PM_Msk, SYS_SRAMPC0_SRAM_RETENTION);

#if (SRAM_REGION_NUMBER == 7)
    printf("SRAM Region 3 Checksum [0x%08X]\n", au32SRAMCheckSum[3]);
    printf("SRAM Region 4 Checksum [0x%08X]\n", au32SRAMCheckSum[4]);
    printf("SRAM Region 5 Checksum [0x%08X]\n", au32SRAMCheckSum[5]);
    printf("SRAM Region 6 Checksum [0x%08X]\n", au32SRAMCheckSum[6]);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM3PM_Msk, SYS_SRAMPC0_SRAM_RETENTION);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM4PM_Msk, SYS_SRAMPC0_SRAM_SHUT_DOWN);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM5PM_Msk, SYS_SRAMPC0_SRAM_SHUT_DOWN);
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM6PM_Msk, SYS_SRAMPC0_SRAM_NORMAL);
#endif

    /* Enter to Power-down mode and wake-up by WDT interrupt */
    printf("\n");
    printf("Enter to Power-down mode ... ");

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);

    /* Configure WDT settings and start WDT counting */
    WDT_Open(WDT_TIMEOUT_2POW14, (uint32_t)NULL, FALSE, TRUE);

    /* Enable WDT interrupt function */
    WDT_EnableInt();

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enter to Power-down mode */
    PowerDownFunction();

    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM2PM_Msk, SYS_SRAMPC0_SRAM_NORMAL);
#if (SRAM_REGION_NUMBER == 7)
    SYS_SetSSRAMPowerMode(SYS_SRAMPC0_SRAM3PM_Msk, SYS_SRAMPC0_SRAM_NORMAL);
#endif

    /* Check if WDT time-out interrupt and wake-up occurred or not */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(s_u8IsINTEvent == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for WDT interrupt time-out!");
            break;
        }
    }
    printf("wake-up by WDT!\n\n");

    /* Calculate SRAM checksum after wake-up from Power-down mode */
    printf("Calculate SRAM CheckSum after wake-up:\n");

    /* Specify SRAM region start address */
    u32SRAMStartAddr = SRAM_BASE;

    /* Calculate SRAM checksum */
    for(u32Idx = 0; u32Idx < SRAM_REGION_NUMBER; u32Idx++)
    {
        /* For M2L31, SRAM regison 6 is in LPSRAM. */
        if (u32Idx == 6)
        {
            u32SRAMStartAddr = LPSRAM_BASE;
        }

        /* Configure CRC controller for CRC-CRC32 mode */
        CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFF, CRC_CPU_WDATA_32);

        /* Start to execute CRC-CRC32 operation */
        for(u32Addr = u32SRAMStartAddr; u32Addr < u32SRAMStartAddr + au32SRAMSize[u32Idx]; u32Addr += 4)
        {
            CRC_WRITE_DATA(inpw(u32Addr));
        }

        /* Record checksum result */
        au32SRAMCheckSum2[u32Idx] = CRC_GetChecksum();

        /* Specify next SRAM region start address */
        u32SRAMStartAddr = u32Addr;
    }

    printf("SRAM Region 0 Checksum [0x%08X] (%s) (Normal mode but include SRAM variables)\n",   au32SRAMCheckSum2[0],
           (au32SRAMCheckSum[0]==au32SRAMCheckSum2[0]) ? "Same" : "Diff");
    printf("SRAM Region 1 Checksum [0x%08X] (%s) (Normal mode)\n",          au32SRAMCheckSum2[1],
           (au32SRAMCheckSum[1]==au32SRAMCheckSum2[1]) ? "Same" : "Diff");
    printf("SRAM Region 2 Checksum [0x%08X] (%s) (Retention mode)\n",       au32SRAMCheckSum2[2],
           (au32SRAMCheckSum[2]==au32SRAMCheckSum2[2]) ? "Same" : "Diff");
#if (SRAM_REGION_NUMBER == 7)
    printf("SRAM Region 3 Checksum [0x%08X] (%s) (Retention mode)\n",       au32SRAMCheckSum2[3],
           (au32SRAMCheckSum[3]==au32SRAMCheckSum2[3]) ? "Same" : "Diff");
    printf("SRAM Region 4 Checksum [0x%08X] (%s) (Shut down mode)\n",       au32SRAMCheckSum2[4],
           (au32SRAMCheckSum[4]==au32SRAMCheckSum2[4]) ? "Same" : "Diff");
    printf("SRAM Region 5 Checksum [0x%08X] (%s) (Shut down mode)\n",       au32SRAMCheckSum2[5],
           (au32SRAMCheckSum[5]==au32SRAMCheckSum2[5]) ? "Same" : "Diff");
    printf("SRAM Region 6 Checksum [0x%08X] (%s) (Normal mode)\n",          au32SRAMCheckSum2[6],
           (au32SRAMCheckSum[6]==au32SRAMCheckSum2[6]) ? "Same" : "Diff");
#endif

    while(1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
