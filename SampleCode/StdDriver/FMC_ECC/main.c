/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrated FMC ECC status and ECC error fault address.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

volatile uint32_t g_u32ECC_Error_Flag = 0U;
volatile uint32_t g_au32Data[3] = {0U};

void ISP_IRQHandler(void)
{
    int32_t i32RetCode;

    if (FMC_GET_FAIL_FLAG())
    {
        /* Clear ISP interrupt flag */
        FMC_CLR_FAIL_FLAG();
    }

    if (FMC_GET_SINGLE_ERR_FLAG())
    {
        /* Print ECC single error fault address */
        printf("[ISP_IRQ] ECC Single Error Fault Address: 0x%08X\n", FMC_GET_SINGLE_ERR_ADDR());

        /* Read ECC data */
        i32RetCode = FMC_Read_64_ECC(FMC_GET_SINGLE_ERR_ADDR(), (uint32_t *)g_au32Data);

        if (i32RetCode == FMC_ERR_ECC)
        {
            printf("ECC Single Error Bit location: 0x%2X\n", g_au32Data[2]);
        }
        else if (i32RetCode != FMC_OK)
        {
            printf("FMC Read Error!\n");
        }

        /* Clear ECC single error bit correction flag */
        FMC_CLR_SINGLE_ERR_FLAG();

        g_u32ECC_Error_Flag |= FMC_ECCSTS_ECCSEBCF_Msk;
    }

    if (FMC_GET_DOUBLE_ERR_FLAG())
    {
        /* Print ECC double error fault address */
        printf("[ISP_IRQ] ECC Double Error Fault Address: 0x%08X\n", FMC_GET_DOUBLE_ERR_ADDR());

        /* Clear ECC double error bits detection flag */
        FMC_CLR_DOUBLE_ERR_FLAG();

        g_u32ECC_Error_Flag |= FMC_ECCSTS_ECCDEBDF_Msk;
    }

    FMC_CLR_ISP_INT_FLAG();
}

void HardFault_Handler(void)
{
    printf("[HardFault] CPU accessed ECC Double Error Fault Address: 0x%08X\n", FMC_GET_DOUBLE_ERR_ADDR());

    while (1)
        ;
}

void FMC_IRQInit(uint32_t u32IntEnable)
{
    if (u32IntEnable)
    {
        /* Enable ISP interrupt */
        NVIC_EnableIRQ(ISP_IRQn);
        FMC_ENABLE_ISP_INT();

        /* Enable ECC single error bit and double error bits detection interrupt */
        FMC->ECCCTL = (FMC_ECCCTL_SEBDINTEN_Msk | FMC_ECCCTL_DEBDINTEN_Msk);
    }
    else
    {
        /* Disable ISP interrupt */
        NVIC_DisableIRQ(ISP_IRQn);
        FMC_DISABLE_ISP_INT();

        /* Disable ECC single error bit and double error bits detection interrupt */
        FMC->ECCCTL = 0;
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

    /* Lock protected registers */
    SYS_LockReg();

    return 0;
}

int32_t main(void)
{
    uint8_t u8IspIntEnable, u8Option;
    int32_t i32RetCode;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("+--------------------------------------------------------------+\n");
    printf("|       FMC Check ECC status and ECC error fault address       |\n");
    printf("|   Terminal will print fault address when ECC error occurs.   |\n");
    printf("+--------------------------------------------------------------+\n");
    printf("  Enable ISP interrupt to check ECC status ?\n");
    printf("  [0] No\n");
    printf("  [1] Yes\n");
    printf("  Select: ");
    u8IspIntEnable = getchar() - '0';
    printf("%d\n\n", u8IspIntEnable);

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable FMC ISP function */
    FMC_Open();

    if (u8IspIntEnable)
    {
        /* Enable ISP interrupt */
        FMC_IRQInit(ENABLE);
    }
    else
    {
        /* Disable ISP interrupt */
        FMC_IRQInit(DISABLE);
    }

    /* Clear ISP fail flag */
    FMC_CLR_FAIL_FLAG();

    /* Clear ECC single error bit correction and double error bits detection flag */
    FMC_CLR_SINGLE_ERR_FLAG();
    FMC_CLR_DOUBLE_ERR_FLAG();

    while (1)
    {
        if (u8IspIntEnable == 0)
        {
            if (FMC_GET_SINGLE_ERR_FLAG())
            {
                /* Print ECC single error fault address */
                printf("ECC Single Error Fault Address: 0x%08X\n", FMC_GET_SINGLE_ERR_ADDR());

                /* Read ECC data */
                i32RetCode = FMC_Read_64_ECC(FMC_GET_SINGLE_ERR_ADDR(), (uint32_t *)g_au32Data);

                if (i32RetCode == FMC_ERR_ECC)
                {
                    printf("ECC Single Error Bit location: 0x%2X\n", g_au32Data[2]);
                }
                else if (i32RetCode != FMC_OK)
                {
                    printf("FMC Read Error!\n");
                }

                /* Clear ECC single error bit correction flag */
                FMC_CLR_SINGLE_ERR_FLAG();

                g_u32ECC_Error_Flag |= FMC_ECCSTS_ECCSEBCF_Msk;
            }

            if (FMC_GET_DOUBLE_ERR_FLAG())
            {
                /* Print ECC double error fault address */
                printf("ECC Double Error Fault Address: 0x%08X\n", FMC_GET_DOUBLE_ERR_ADDR());

                /* Clear ECC double error bits detection flag */
                FMC_CLR_DOUBLE_ERR_FLAG();

                g_u32ECC_Error_Flag |= FMC_ECCSTS_ECCDEBDF_Msk;
            }
        }

        if (g_u32ECC_Error_Flag)
        {
            if (g_u32ECC_Error_Flag & FMC_ECCSTS_ECCDEBDF_Msk)
            {
                printf("ECC double error bits detected!\n");

                while (1)
                    ;
            }

            printf("Press [ESC] to exit or any key to continue...\n");
            u8Option = getchar();

            if (u8Option == 0x1B) /* ESC */
            {
                goto lexit;
            }

            g_u32ECC_Error_Flag = 0U;
        }
    }

lexit:

    FMC_Close();                       /* Disable FMC ISP function */

    FMC_IRQInit(DISABLE);              /* Disable ISP interrupt */

    SYS_LockReg();                     /* Lock protected registers */

    printf("\nFMC Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
