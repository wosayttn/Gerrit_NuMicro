/******************************************************************************
 * @file    main.c
 * @version V1.00
 * @brief   Show the usage of GPIO EINT trigger PDMA function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

uint32_t PDMA_TEST_LENGTH = 64;
#ifdef __ICCARM__
    #pragma data_alignment=4
    uint8_t au8SrcArray[256];
    uint8_t au8DestArray[256];
#else
    __attribute__((aligned(4))) uint8_t au8SrcArray[256];
    __attribute__((aligned(4))) uint8_t au8DestArray[256];
#endif
uint32_t volatile g_u32IsTestOver = 0;

void EINT1_IRQHandler(void)
{
    /* To check if external interrupt occurred */
    if (GPIO_GET_EINT_FLAG(1))
    {
        GPIO_CLR_EINT_FLAG(1);
        printf("EINT1 occurred.\n");
    }
}

void PDMA0_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA0);

    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        /* Check if channel 2 has abort error */
        if (PDMA_GET_ABORT_STS(PDMA0) & PDMA_ABTSTS_ABTIF2_Msk)
        {
            g_u32IsTestOver = 2;
        }

        /* Clear abort flag of channel 2 */
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        /* Check transmission of channel 2 has been transfer done */
        if (PDMA_GET_TD_STS(PDMA0) & PDMA_TDSTS_TDIF2_Msk)
        {
            g_u32IsTestOver = 1;
        }

        /* Clear transfer done flag of channel 2 */
        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
    }
    else
    {
        printf("unknown interrupt !!\n");
    }
}

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
    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);
    /* Enable GPIO Port A clock */
    CLK_EnableModuleClock(GPA_MODULE);
    /* Set multi-function pin for EINT1(PA.7) */
    SET_INT1_PA7();
    /* Enable Internal low speed RC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
    /* Waiting for Internal low speed RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);
    /* Lock protected registers */
    SYS_LockReg();
    return 0;
}

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------+\n");
    printf("|    GPIO EINT Trigger PDMA Sample Code    |\n");
    printf("+------------------------------------------+\n\n");
    printf("Use EINT1(PA.7) falling edge to start PDMA transfer.\n");
    /*---------------------------------------------------------------------------------------------------------*/
    /* PDMA Configuration                                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Open Channel 2 */
    PDMA_Open(PDMA0, BIT2);
    /* Transfer count is PDMA_TEST_LENGTH, transfer width is 32 bits(one word) */
    PDMA_SetTransferCnt(PDMA0, 2, PDMA_WIDTH_32, PDMA_TEST_LENGTH);
    /* Set source address is au8SrcArray, destination address is au8DestArray, Source/Destination increment size is 32 bits(one word) */
    PDMA_SetTransferAddr(PDMA0, 2, (uint32_t)au8SrcArray, PDMA_SAR_INC, (uint32_t)au8DestArray, PDMA_DAR_INC);
    /* Request source is memory to memory */
    PDMA_SetTransferMode(PDMA0, 2, PDMA_EINT1, FALSE, 0);
    /* Transfer type is burst transfer and burst size is 4 */
    PDMA_SetBurstType(PDMA0, 2, PDMA_REQ_BURST, PDMA_BURST_4);
    /* Enable interrupt */
    PDMA_EnableInt(PDMA0, 2, PDMA_INT_TRANS_DONE);
    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA0_IRQn);
    g_u32IsTestOver = 0;
    /*---------------------------------------------------------------------------------------------------------*/
    /* GPIO Configuration                                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure PA.7 as EINT1 pin and enable interrupt by falling edge trigger */
    GPIO_SetMode(PA, BIT7, GPIO_MODE_INPUT);
    GPIO_SetPullCtl(PA, BIT7, GPIO_PUSEL_PULL_UP);
    GPIO_EnableEINT(1, GPIO_INT_EDETCTL_FALLING);
    NVIC_EnableIRQ(EINT1_IRQn);
    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(PA, GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PA, BIT7);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Waiting for PDMA transfer done                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Waiting for transfer done */
    while (g_u32IsTestOver == 0) {};

    /* Check transfer result */
    if (g_u32IsTestOver == 1)
    {
        printf("test done...\n");
    }
    else if (g_u32IsTestOver == 2)
    {
        printf("target abort...\n");
    }

    /* Close PDMA0 */
    PDMA_Close(PDMA0);

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
