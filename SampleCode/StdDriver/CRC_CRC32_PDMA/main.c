/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 9 $
 * $Date: 18/07/09 4:26p $
 * @brief    Implement CRC in CRC-32 mode with PDMA transfer.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

uint32_t volatile g_u32IsTestOver = 0;

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch the core clock to 40MHz from the MIRC */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable CRC clock */
    CLK_EnableModuleClock(CRC_MODULE);

    /* Enable PDMA clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

uint32_t GetFMCChecksum(uint32_t u32Address, uint32_t u32Size)
{
    uint32_t u32CHKS;

    FMC_ENABLE_ISP();
    u32CHKS = FMC_GetChkSum(u32Address, u32Size);

    return u32CHKS;
}

/**
 * @brief       PDMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PDMA default IRQ, declared in startup_M2U51.c.
 */
void PDMA0_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA0);

    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        /* Check if channel 1 has abort error */
        if (PDMA_GET_ABORT_STS(PDMA0) & PDMA_ABTSTS_ABTIF1_Msk)
            g_u32IsTestOver = 2;

        /* Clear abort flag of channel 1 */
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF1_Msk);
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        /* Check transmission of channel 1 has been transfer done */
        if (PDMA_GET_TD_STS(PDMA0) & PDMA_TDSTS_TDIF1_Msk)
            g_u32IsTestOver = 1;

        /* Clear transfer done flag of channel 1 */
        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF1_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

uint32_t GetPDMAChecksum(uint32_t u32Address, uint32_t u32Size)
{
    /* Open Channel 1 */
    PDMA_Open(PDMA0,1 << 1);
    /* Transfer count is u32Size / 4, transfer width is 32 bits(one word) */
    PDMA_SetTransferCnt(PDMA0, 1, PDMA_WIDTH_32, u32Size / 4);
    /* Set source address is u32Address, destination address is CRC->DAT, Source increment size is 32 bits(one word), Destination increment size is 0 */
    PDMA_SetTransferAddr(PDMA0, 1, u32Address, PDMA_SAR_INC, (uint32_t)&CRC->DAT, PDMA_DAR_FIX);
    /* Request source is memory to memory */
    PDMA_SetTransferMode(PDMA0, 1, PDMA_MEM, FALSE, 0);
    /* Transfer type is burst transfer and burst size is 4 */
    PDMA_SetBurstType(PDMA0, 1, PDMA_REQ_BURST, PDMA_BURST_4);

    /* Enable interrupt */
    PDMA_EnableInt(PDMA0, 1, PDMA_INT_TRANS_DONE);

    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA0_IRQn);

    g_u32IsTestOver = 0;

    /* Generate a software request to trigger transfer with PDMA channel 1  */
    PDMA_Trigger(PDMA0, 1);

    /* Waiting for transfer done */
    while(g_u32IsTestOver == 0);

    return CRC->CHECKSUM;
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t addr, size, u32FMCChecksum, u32CRC32Checksum, u32PDMAChecksum;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    size = 1024 * 2;

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------------------------+\n");
    printf("|    CRC32 with PDMA Sample Code                      |\n");
    printf("|       - Get APROM first %d bytes CRC result by    |\n", size);
    printf("|          a.) FMC checksum command                   |\n");
    printf("|          b.) CPU write CRC data register directly   |\n");
    printf("|          c.) PDMA write CRC data register           |\n");
    printf("+-----------------------------------------------------+\n\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function. Before using FMC function, it should unlock system register first. */
    FMC_Open();

    /*  Case a. */
    u32FMCChecksum = GetFMCChecksum(0x0, size);

    /* Lock protected registers */
    SYS_LockReg();

    /*  Case b. */
    /* Configure CRC controller for CRC-CRC32 mode */
    CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFF, CRC_CPU_WDATA_32);

    /* Start to execute CRC-CRC32 operation */
    for(addr = 0; addr < size; addr += 4)
    {
        CRC_WRITE_DATA(inpw(addr));
    }
    u32CRC32Checksum = CRC_GetChecksum();

    /*  Case c. */
    /* Configure CRC controller for CRC-CRC32 mode */
    CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFF, CRC_CPU_WDATA_32);

    u32PDMAChecksum = GetPDMAChecksum(0x0, size);

    printf("APROM first %d bytes checksum:\n", size);
    printf("   - by FMC command: 0x%08X\n", u32FMCChecksum);
    printf("   - by CPU write:   0x%08X\n", u32CRC32Checksum);
    printf("   - by PDMA write:  0x%08X\n", u32PDMAChecksum);

    if((u32FMCChecksum == u32CRC32Checksum) && (u32CRC32Checksum == u32PDMAChecksum))
    {
        if((u32FMCChecksum == 0) || (u32FMCChecksum == 0xFFFFFFFF))
        {
            printf("\n[Get checksum ... WRONG]");
        }
        else
        {
            printf("\n[Compare checksum ... PASS]");
        }
    }
    else
    {
        printf("\n[Compare checksum ... WRONG]");
    }

    /* Disable CRC function */
    CLK_DisableModuleClock(CRC_MODULE);

    while(1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
