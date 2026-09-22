/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrated FMC ECC status and ECC error fault address.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

uint32_t g_u32ECC_Error_Flag = 0U;
uint32_t g_u32Data[3] = {0U};

int32_t FMC_Read_64_ECC(uint32_t u32addr, uint32_t u32data[]);

void ISP_IRQHandler(void)
{
    if ((FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk) == FMC_ISPSTS_ISPFF_Msk)
    {
        /* Clear ISP interrupt flag */
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
    }

    if (FMC->ECCSTS & FMC_ECCSTS_ECCSEBCF_Msk)
    {
        /* Print ECC single error fault address */
        printf("ECC Single Error Fault Address: 0x%08X\n", FMC->ECCSEFAR);

        /* Read ECC data */
        if (FMC_Read_64_ECC(FMC->ECCSEFAR, g_u32Data) == 0)
        {
            printf("ECC Single Error Bit location: 0x%2X\n", g_u32Data[2]);
        }
        else
        {
            printf("FMC Read Error!\n");
        }

        /* Clear ECC single error bit correction flag */
        FMC->ECCSTS = FMC_ECCSTS_ECCSEBCF_Msk;

        g_u32ECC_Error_Flag |= FMC_ECCSTS_ECCSEBCF_Msk;
    }

    if (FMC->ECCSTS & FMC_ECCSTS_ECCDEBDF_Msk)
    {
        /* Print ECC double error fault address */
        printf("ECC Double Error Fault Address: 0x%08X\n", FMC->ECCDEFAR);

        /* Clear ECC double error bits detection flag */
        FMC->ECCSTS = FMC_ECCSTS_ECCDEBDF_Msk;

        g_u32ECC_Error_Flag |= FMC_ECCSTS_ECCDEBDF_Msk;
    }

    FMC->ISPSTS |= FMC_ISPSTS_INTFLAG_Msk;
}

void HardFault_Handler(void)
{
    uint32_t *sp = (uint32_t *)__get_MSP();
    uint32_t n = 6;

    printf("[HardFault] CPU accessed ECC Double Error Fault Address: 0x%08X\n", FMC->ECCDEFAR);

    sp[n] += 2;

    while (1)
        ;
}

int32_t FMC_Read_64_ECC(uint32_t u32addr, uint32_t u32data[])
{
    int32_t ret = 0;
    int32_t i32TimeOutCnt;

    FMC->ISPCMD = FMC_ISPCMD_READ_64;
    FMC->ISPADDR = u32addr;
    FMC->ISPDAT = 0x0UL;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = FMC_TIMEOUT_READ;
    while (FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk)
    {
        if( i32TimeOutCnt-- <= 0)
        {
            ret = -1;
            break;
        }
    }

    if (FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
    {
        FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
        ret = -2;
    }
    else
    {
        u32data[0] = FMC->MPDAT0;
        u32data[1] = FMC->MPDAT1;
        u32data[2] = FMC->MPDAT2;
    }
    return ret;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_180MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
}

void UART0_Init(void)
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
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

int main()
{
    uint8_t u8IspIntEnable, u8Option;

    /* Unlock register lock protect */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Initialize UART0 */
    UART0_Init();

    printf("+--------------------------------------------------------------+\n");
    printf("|       FMC Check ECC status and ECC error fault address       |\n");
    printf("|   Terminal will print fault address when ECC error occurs.   |\n");
    printf("+--------------------------------------------------------------+\n");
    printf("  Enable ISP interrupt to check ECC status?\n");
    printf("  [0] No\n");
    printf("  [1] Yes\n");
    printf("  Select: ");
    u8IspIntEnable = getchar() - '0';
    printf("%d\n\n", u8IspIntEnable);

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
    FMC->ECCSTS = (FMC_ECCSTS_ECCSEBCF_Msk | FMC_ECCSTS_ECCDEBDF_Msk);

    while (1)
    {
        if (u8IspIntEnable == 0)
        {
            if (FMC->ECCSTS & FMC_ECCSTS_ECCSEBCF_Msk)
            {
                /* Print ECC single error fault address */
                printf("ECC Single Error Fault Address: 0x%08X\n", FMC->ECCSEFAR);

                /* Read ECC data */
                if (FMC_Read_64_ECC(FMC->ECCSEFAR, g_u32Data) == 0)
                {
                    printf("ECC Single Error Bit location: 0x%2X\n", g_u32Data[2]);
                }
                else
                {
                    printf("FMC Read Error!\n");
                }

                /* Clear ECC single error bit correction flag */
                FMC->ECCSTS = FMC_ECCSTS_ECCSEBCF_Msk;

                g_u32ECC_Error_Flag |= FMC_ECCSTS_ECCSEBCF_Msk;
            }

            if (FMC->ECCSTS & FMC_ECCSTS_ECCDEBDF_Msk)
            {
                /* Print ECC double error fault address */
                printf("ECC Double Error Fault Address: 0x%08X\n", FMC->ECCDEFAR);

                /* Clear ECC double error bits detection flag */
                FMC->ECCSTS = FMC_ECCSTS_ECCDEBDF_Msk;

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
