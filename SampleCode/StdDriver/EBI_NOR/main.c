/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Configure EBI interface to access NOR Flash connected on EBI interface.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
extern uint16_t NOR_MX29LV320T_READ(uint32_t u32Bank, uint32_t u32DstAddr);
extern int32_t NOR_MX29LV320T_WRITE(uint32_t u32Bank, uint32_t u32DstAddr, uint16_t u16Data);
extern void NOR_MX29LV320T_GET_ID(uint32_t u32Bank, uint16_t *pu16IDTable);
extern int32_t NOR_MX29LV320T_EraseChip(uint32_t u32Bank, uint32_t u32IsCheckBlank);

void Configure_EBI_16BIT_Pins(void);
void SYS_Init(void);

void Configure_EBI_16BIT_Pins(void)
{
    /* EBI MCLK, ALE, nRD, nWR */
    SET_EBI_ALE_PA8();
    SET_EBI_MCLK_PA9();
    SET_EBI_nRD_PA11();
    SET_EBI_nWR_PA10();

    /* EBI nWRH and nWRL pins */
    SET_EBI_nWRH_PB6();
    SET_EBI_nWRL_PB7();

    /* EBI nCS0 */
    SET_EBI_nCS0_PD12();
    /* EBI nCS1 */
    SET_EBI_nCS1_PD11();
    /* EBI nCS2 */
    SET_EBI_nCS2_PD10();

    /* AD0 ~ AD7 */
    SET_EBI_AD0_PC0();
    SET_EBI_AD1_PC1();
    SET_EBI_AD2_PC2();
    SET_EBI_AD3_PC3();
    SET_EBI_AD4_PC4();
    SET_EBI_AD5_PC5();
    SET_EBI_AD6_PD8();
    SET_EBI_AD7_PD9();

    /* AD8 ~ AD15 */
    SET_EBI_AD8_PE14();
    SET_EBI_AD9_PE15();
    SET_EBI_AD10_PD3();
    SET_EBI_AD11_PC14();
    SET_EBI_AD12_PB15();
    SET_EBI_AD13_PB14();
    SET_EBI_AD14_PH10();
    SET_EBI_AD15_PH11();

    /* ADR0 ~ ADR15 */
    SET_EBI_ADR0_PH7();
    SET_EBI_ADR1_PH6();
    SET_EBI_ADR2_PH5();
    SET_EBI_ADR3_PH4();
    SET_EBI_ADR4_PC12();
    SET_EBI_ADR5_PC11();
    SET_EBI_ADR6_PC10();
    SET_EBI_ADR7_PC9();
    SET_EBI_ADR8_PB1();
    SET_EBI_ADR9_PB0();
    SET_EBI_ADR10_PC13();
    SET_EBI_ADR11_PG2();
    SET_EBI_ADR12_PG3();
    SET_EBI_ADR13_PG4();
    SET_EBI_ADR14_PF11();
    SET_EBI_ADR15_PE13();
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL and set core clock to 144MHz*/
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /* Enable EBI clock */
    CLK_EnableModuleClock(EBI_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32Addr, u32MaxEBISize;
    uint16_t u16WData, u16RData;
    uint16_t u16IDTable[2];

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------------+\n");
    printf("|    EBI Nor Flash Sample Code on Bank1   |\n");
    printf("+-----------------------------------------+\n\n");

    printf("************************************************************************\n");
    printf("* Please connect MX29LV320T nor flash to EBI bank1 before accessing !! *\n");
    printf("************************************************************************\n\n");

    /* Configure multi-function pins for EBI 16-bit application */
    Configure_EBI_16BIT_Pins();

    /* Initialize EBI bank1 to access external nor */
    EBI_Open(EBI_BANK1, EBI_BUSWIDTH_16BIT, EBI_TIMING_SLOW, EBI_OPMODE_NORMAL, EBI_CS_ACTIVE_LOW);

    /* Step 1, check ID */
    NOR_MX29LV320T_GET_ID(EBI_BANK1, (uint16_t *)u16IDTable);

    printf(">> Manufacture ID: 0x%X, Device ID: 0x%X .... ", u16IDTable[0], u16IDTable[1]);

    if ((u16IDTable[0] != 0xC2) || (u16IDTable[1] != 0x22A8))
    {
        printf("FAIL !!!\n\n");
        goto lexit;
    }
    else
    {
        printf("PASS !!!\n\n");
    }

    /* Step 2, erase chip */
    if (NOR_MX29LV320T_EraseChip(EBI_BANK1, TRUE) < 0)
    {
        printf("FAIL !!!\n\n");
        goto lexit;
    }

    /* Step 3, program flash and compare data */
    printf(">> Run program flash test ......\n");
    u32MaxEBISize = EBI_MAX_SIZE;

    for (u32Addr = 0; u32Addr < u32MaxEBISize; u32Addr += 2)
    {
        u16WData = (0x7657 + u32Addr / 2) & 0xFFFF;

        if (NOR_MX29LV320T_WRITE(EBI_BANK1, u32Addr, u16WData) < 0)
        {
            printf("Program [0x%08X]: [0x%08X] FAIL !!!\n\n", (uint32_t)(EBI_BANK1_BASE_ADDR) + u32Addr, u16WData);
            goto lexit;
        }
        else
        {
            /* Show UART message ...... */
            if ((u32Addr % 256) == 0)
                printf("Program [0x%08X]:[0x%08X] !!!       \r", (uint32_t)(EBI_BANK1_BASE_ADDR) + u32Addr, u16WData);
        }
    }

    for (u32Addr = 0; u32Addr < u32MaxEBISize; u32Addr += 2)
    {
        u16WData = (0x7657 + (u32Addr / 2)) & 0xFFFF;
        u16RData = NOR_MX29LV320T_READ(EBI_BANK1, u32Addr);

        if (u16WData != u16RData)
        {
            printf("Compare [0x%08X] FAIL !!! (W:0x%08X, R:0x%08X)\n\n", (uint32_t)(EBI_BANK1_BASE_ADDR) + u32Addr, u16WData, u16RData);
            goto lexit;
        }
        else
        {
            /* Show UART message ...... */
            if ((u32Addr % 256) == 0)
                printf("Read [0x%08X]: [0x%08X] !!!         \r", (uint32_t)(EBI_BANK1_BASE_ADDR + u32Addr), u16RData);
        }
    }

    printf(">> Program flash OK !!!                             \n\n");

lexit:

    /* Disable EBI function */
    EBI_Close(EBI_BANK1);

    /* Disable EBI clock */
    CLK_DisableModuleClock(EBI_MODULE);

    while (1) {}
}
