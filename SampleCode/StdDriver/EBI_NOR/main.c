/**************************************************************************//**
 * @file     main.c
 * @version  V1.0
 * @brief    Configure EBI interface to access MX29LV320T (NOR Flash) on EBI interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern void NOR_MX29LV320T_RESET(uint32_t u32Bank);
extern int32_t NOR_MX29LV320T_CheckStatus(uint32_t u32DstAddr, uint16_t u16Data, uint32_t u32TimeoutMs);
extern uint16_t NOR_MX29LV320T_READ(uint32_t u32Bank, uint32_t u32DstAddr);
extern int32_t NOR_MX29LV320T_WRITE(uint32_t u32Bank, uint32_t u32DstAddr, uint16_t u16Data);
extern void NOR_MX29LV320T_GET_ID(uint32_t u32Bank, uint16_t *pu16IDTable);
extern int32_t NOR_MX29LV320T_EraseChip(uint32_t u32Bank, uint32_t u32IsCheckBlank);

void Configure_EBI_16BIT_Pins(void)
{
    /* EBI AD[5:0]=PC[5:0] */
    SYS->GPC_MFP0 &= ~(SYS_GPC_MFP0_PC3MFP_Msk | SYS_GPC_MFP0_PC2MFP_Msk
                       | SYS_GPC_MFP0_PC1MFP_Msk | SYS_GPC_MFP0_PC0MFP_Msk);
    SYS->GPC_MFP1 &= ~(SYS_GPC_MFP1_PC4MFP_Msk | SYS_GPC_MFP1_PC5MFP_Msk);
    SYS->GPC_MFP0 |= SYS_GPC_MFP0_PC3MFP_EBI_AD3 | SYS_GPC_MFP0_PC2MFP_EBI_AD2
                     | SYS_GPC_MFP0_PC1MFP_EBI_AD1 | SYS_GPC_MFP0_PC0MFP_EBI_AD0;
    SYS->GPC_MFP1 |= SYS_GPC_MFP1_PC5MFP_EBI_AD5 | SYS_GPC_MFP1_PC4MFP_EBI_AD4;

    /* AD[7:6]=PA[7:6] */
    SYS->GPA_MFP1 &= ~(SYS_GPA_MFP1_PA6MFP_Msk | SYS_GPA_MFP1_PA7MFP_Msk);
    SYS->GPA_MFP1 |= SYS_GPA_MFP1_PA7MFP_EBI_AD7 | SYS_GPA_MFP1_PA6MFP_EBI_AD6;

    /* AD[9:8]=PC[7:6] */
    SYS->GPC_MFP1 &= ~(SYS_GPC_MFP1_PC6MFP_Msk | SYS_GPC_MFP1_PC7MFP_Msk);
    SYS->GPC_MFP1 |= SYS_GPC_MFP1_PC7MFP_EBI_AD9 | SYS_GPC_MFP1_PC6MFP_EBI_AD8;

    /* AD[13:10]=PD[0:3] */
    SYS->GPD_MFP0 &= ~(SYS_GPD_MFP0_PD3MFP_Msk | SYS_GPD_MFP0_PD2MFP_Msk
                       | SYS_GPD_MFP0_PD1MFP_Msk | SYS_GPD_MFP0_PD0MFP_Msk);
    SYS->GPD_MFP0 |= (SYS_GPD_MFP0_PD1MFP_EBI_AD12 | SYS_GPD_MFP0_PD0MFP_EBI_AD13
                      | SYS_GPD_MFP0_PD2MFP_EBI_AD11 | SYS_GPD_MFP0_PD3MFP_EBI_AD10);

    /* AD[15:14]=PB[12:13] */
    SYS->GPB_MFP3 &= ~(SYS_GPB_MFP3_PB12MFP_Msk | SYS_GPB_MFP3_PB13MFP_Msk);
    SYS->GPB_MFP3 |= SYS_GPB_MFP3_PB13MFP_EBI_AD14 | SYS_GPB_MFP3_PB12MFP_EBI_AD15;

    /* ADR[19:16]=PB[8:11] */
    SYS->GPB_MFP2 &= ~(SYS_GPB_MFP2_PB8MFP_Msk | SYS_GPB_MFP2_PB9MFP_Msk
                       | SYS_GPB_MFP2_PB10MFP_Msk | SYS_GPB_MFP2_PB11MFP_Msk);
    SYS->GPB_MFP2 |= (SYS_GPB_MFP2_PB11MFP_EBI_ADR16 | SYS_GPB_MFP2_PB10MFP_EBI_ADR17
                      | SYS_GPB_MFP2_PB9MFP_EBI_ADR18 | SYS_GPB_MFP2_PB8MFP_EBI_ADR19);

    /* #WRL and #WRH pins on PB.7 and PB.6 */
    SYS->GPB_MFP1 &= ~(SYS_GPB_MFP1_PB7MFP_Msk | SYS_GPB_MFP1_PB6MFP_Msk);
    SYS->GPB_MFP1 |= SYS_GPB_MFP1_PB7MFP_EBI_nWRL | SYS_GPB_MFP1_PB6MFP_EBI_nWRH;

    /* #RD and #WR pins on PA.11 and PA.10 */
    SYS->GPA_MFP2 &= ~(SYS_GPA_MFP2_PA11MFP_Msk  | SYS_GPA_MFP2_PA10MFP_Msk);
    SYS->GPA_MFP2 |= (SYS_GPA_MFP2_PA11MFP_EBI_nRD  | SYS_GPA_MFP2_PA10MFP_EBI_nWR);

    /* #CS[0:2]=PD[12:10] */
    SYS->GPD_MFP3 &= ~(SYS_GPD_MFP3_PD12MFP_Msk);
    SYS->GPD_MFP3 |= SYS_GPD_MFP3_PD12MFP_EBI_nCS0;
    SYS->GPD_MFP2 &= ~(SYS_GPD_MFP2_PD11MFP_Msk | SYS_GPD_MFP2_PD10MFP_Msk);
    SYS->GPD_MFP2 |= (SYS_GPD_MFP2_PD11MFP_EBI_nCS1 | SYS_GPD_MFP2_PD10MFP_EBI_nCS2);

    /* ALE and MCLK pins on PA.8 and PA.9 */
    SYS->GPA_MFP2 &= ~(SYS_GPA_MFP2_PA9MFP_Msk | SYS_GPA_MFP2_PA8MFP_Msk);
    SYS->GPA_MFP2 |= (SYS_GPA_MFP2_PA9MFP_EBI_MCLK | SYS_GPA_MFP2_PA8MFP_EBI_ALE);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable EBI peripheral clock */
    CLK_EnableModuleClock(EBI_MODULE);

    /* Enable PDMA peripheral clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD=PA.4 and TXD=PA.5 */
    SYS->GPA_MFP1 &= ~(SYS_GPA_MFP1_PA5MFP_Msk | SYS_GPA_MFP1_PA4MFP_Msk);
    SYS->GPA_MFP1 |= SYS_GPA_MFP1_PA5MFP_UART0_TXD | SYS_GPA_MFP1_PA4MFP_UART0_RXD;
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32Addr, u32MaxEBISize;
    uint16_t u16WData, u16RData;
    uint16_t u16IDTable[2];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------------+\n");
    printf("|    EBI Nor Flash Sample Code on Bank1   |\n");
    printf("+-----------------------------------------+\n\n");

    printf("************************************************************************\n");
    printf("* Please connect MX29LV320T nor flash to EBI bank1 before accessing !! *\n");
    printf("* EBI pins settings:                                                   *\n");
    printf("*                                                                      *\n");
    printf("*   - AD0 ~ AD5     on PC.0 ~ PC.5                                     *\n");
    printf("*   - AD6 ~ AD7     on PA.6 ~ PA.7                                     *\n");
    printf("*   - AD8 ~ AD9     on PC.6 ~ PC.7                                     *\n");
    printf("*   - AD10 ~ AD11   on PD.3 ~ PD.2                                     *\n");
    printf("*   - AD12 ~ AD13   on PD.1 ~ PD.0                                     *\n");
    printf("*   - AD14 ~ AD15   on PB.13 ~ PB.12                                   *\n");
    printf("*   - ADR16 ~ ADR17 on PB.11 ~ PB.10                                   *\n");
    printf("*   - ADR18 ~ ADR19 on PB.9  ~ PB.8                                    *\n");
    printf("*   - nWR           on PA.10                                           *\n");
    printf("*   - nRD           on PA.11                                           *\n");
    printf("*   - nWRL          on PB.7                                            *\n");
    printf("*   - nWRH          on PB.6                                            *\n");
    printf("*   - nCS0          on PD.12                                           *\n");
    printf("*   - nCS1          on PD.11                                           *\n");
    printf("*   - nCS2          on PD.10                                           *\n");
    printf("*   - ALE           on PA.8                                            *\n");
    printf("*   - MCLK          on PA.9                                            *\n");
    printf("*                                                                      *\n");
    printf("**********************************************************************\n\n");

    /* Configure multi-function pins for EBI 16-bit application */
    Configure_EBI_16BIT_Pins();

    /* Initialize EBI bank1 to access external nor */
    EBI_Open(EBI_BANK1, EBI_BUSWIDTH_16BIT, EBI_TIMING_VERYSLOW, 0, EBI_CS_ACTIVE_LOW);

    /* Step 1, check ID */
    NOR_MX29LV320T_GET_ID(EBI_BANK1, (uint16_t *)u16IDTable);
    printf(">> Manufacture ID: 0x%X, Device ID: 0x%X .... ", u16IDTable[0], u16IDTable[1]);

    if ((u16IDTable[0] != 0xC2) || (u16IDTable[1] != 0x22A8))
    {
        printf("FAIL !!!\n\n");

        while (1);
    }
    else
    {
        printf("PASS !!!\n\n");
    }

    /* Step 2, erase chip */
    if (NOR_MX29LV320T_EraseChip(EBI_BANK1, TRUE) < 0)
        while (1);


    /* Step 3, program flash and compare data */
    printf(">> Run program flash test ......\n");
    u32MaxEBISize = EBI_MAX_SIZE;

    for (u32Addr = 0; u32Addr < u32MaxEBISize; u32Addr += 2)
    {
        u16WData = (0x7657 + u32Addr / 2) & 0xFFFF;

        if (NOR_MX29LV320T_WRITE(EBI_BANK1, u32Addr, u16WData) < 0)
        {
            printf("Program [0x%08X]: [0x%08X] FAIL !!!\n\n", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr), u16WData);

            while (1);
        }
        else
        {
            /* Show UART message ...... */
            if ((u32Addr % 256) == 0)
                printf("Program [0x%X]:[0x%X] !!!       \r", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr), u16WData);
        }
    }

    for (u32Addr = 0; u32Addr < u32MaxEBISize; u32Addr += 2)
    {
        u16WData = (0x7657 + u32Addr / 2) & 0xFFFF;
        u16RData = NOR_MX29LV320T_READ(EBI_BANK1, u32Addr);

        if (u16WData != u16RData)
        {
            printf("Compare [0x%08X] FAIL !!! (W:0x%08X, R:0x%08X)\n\n", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr), u16WData, u16RData);

            while (1);
        }
        else
        {
            /* Show UART message ...... */
            if ((u32Addr % 256) == 0)
                printf("Read [0x%08X]: [0x%08X] !!!         \r", (uint32_t)(EBI_BANK0_BASE_ADDR + (0x100000 * EBI_BANK1) + u32Addr), u16RData);
        }
    }

    printf(">> Program flash OK !!!                             \n\n");

    /* Disable EBI function */
    EBI_Close(EBI_BANK1);

    /* Disable EBI clock */
    CLK_DisableModuleClock(EBI_MODULE);

    while (1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
