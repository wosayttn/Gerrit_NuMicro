/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate DFMC ECC status and ECC error fault address.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2017-2026 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

uint32_t g_u32ECC_Error_Flag = 0U;
uint32_t g_u32Data[3] = {0U};

int32_t DFMC_Read_ECC(uint32_t u32addr, uint32_t u32data[]);

void DFMC_IRQHandler(void)
{
    if ((DFMC->ISPSTS & DFMC_ISPSTS_ISPFF_Msk) == DFMC_ISPSTS_ISPFF_Msk)
    {
        /* Clear ISP interrupt flag */
        DFMC->ISPSTS |= DFMC_ISPSTS_ISPFF_Msk;
    }

    if (DFMC->ECCSTS & DFMC_ECCSTS_ECCSEBCF_Msk)
    {
        /* Print ECC single error fault address */
        printf("ECC Single Error Fault Address: 0x%08x\n", DFMC->ECCSEFAR);

        /* Read ECC data */
        if (DFMC_Read_ECC(DFMC->ECCSEFAR, g_u32Data) == 0)
        {
            printf("ECC Single Error Bit location: 0x%2X\n", g_u32Data[2]);
        }
        else
        {
            printf("DFMC Read Error!\n");
        }

        /* Clear ECC single error bit correction flag */
        DFMC->ECCSTS = DFMC_ECCSTS_ECCSEBCF_Msk;

        g_u32ECC_Error_Flag |= DFMC_ECCSTS_ECCSEBCF_Msk;
    }

    if (DFMC->ECCSTS & DFMC_ECCSTS_ECCDEBDF_Msk)
    {
        /* Print ECC double error fault address */
        printf("ECC Double Error Fault Address: 0x%08x\n", DFMC->ECCDEFAR);

        /* Clear ECC double error bits detection flag */
        DFMC->ECCSTS = DFMC_ECCSTS_ECCDEBDF_Msk;

        g_u32ECC_Error_Flag |= DFMC_ECCSTS_ECCDEBDF_Msk;
    }

    DFMC->ISPSTS |= DFMC_ISPSTS_ISPIF_Msk;
}

void HardFault_Handler(void)
{
    uint32_t *sp = (uint32_t *)__get_MSP();
    uint32_t n = 6;

    printf("[HardFault] CPU accessed ECC Double Error Fault Address: 0x%08x\n", FMC->ECCDEFAR);

    sp[n] += 2;

    while (1)
        ;
}

int32_t DFMC_Read_ECC(uint32_t u32addr, uint32_t u32data[])
{
    int32_t ret = 0;
    int32_t i32TimeOutCnt;

    DFMC->ISPCMD = DFMC_ISPCMD_READ;
    DFMC->ISPADDR = u32addr;
    DFMC->ISPDAT = 0x0UL;
    DFMC->ISPTRG = DFMC_ISPTRG_ISPGO_Msk;

    i32TimeOutCnt = DFMC_TIMEOUT_READ;
    while (DFMC->ISPSTS & DFMC_ISPSTS_ISPBUSY_Msk)
    {
        if( i32TimeOutCnt-- <= 0)
        {
            ret = -1;
            break;
        }
    }

    if (DFMC->ISPSTS & DFMC_ISPSTS_ISPFF_Msk)
    {
        DFMC->ISPSTS |= DFMC_ISPSTS_ISPFF_Msk;
        ret = -2;
    }
    else
    {
        u32data[0] = DFMC->ISPDAT;
        u32data[2] = DFMC->MPDAT2;
    }
    return ret;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_HCLKDIV_HCLK(1));

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_PCLK0DIV_DIV2 | CLK_PCLKDIV_PCLK1DIV_DIV2);

    /* Set core clock to 40MHz */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Enable DFMC0 module clock */
    CLK_EnableModuleClock(DFMC0_MODULE);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_UARTSEL_UART0SEL_HIRC, CLK_UARTDIV_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void DFMC_IRQInit(uint32_t u32IntEnable)
{
    if (u32IntEnable)
    {
        /* Enable ISP interrupt */
        NVIC_EnableIRQ(DFMC_IRQn);
        DFMC_ENABLE_ISP_INT();

        /* Enable ECC single error bit and double error bits detection interrupt */
        DFMC->ECCCTL = (DFMC_ECCCTL_SEBDINTEN_Msk | DFMC_ECCCTL_DEBDINTEN_Msk);
    }
    else
    {
        /* Disable ISP interrupt */
        NVIC_DisableIRQ(DFMC_IRQn);
        DFMC_DISABLE_ISP_INT();

        /* Disable ECC single error bit and double error bits detection interrupt */
        DFMC->ECCCTL = 0;
    }
}

int main()
{
    uint8_t u8IspIntEnable, u8Option;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Initialize UART0 */
    UART0_Init();

    printf("+--------------------------------------------------------------+\n");
    printf("|       DFMC Check ECC status and ECC error fault address      |\n");
    printf("|   Terminal will print fault address when ECC error occurs.   |\n");
    printf("+--------------------------------------------------------------+\n");
    printf("  Enable ISP interrupt to check ECC status?\n");
    printf("  [0] No\n");
    printf("  [1] Yes\n");
    printf("  Select: ");
    u8IspIntEnable = getchar() - '0';
    printf("%d\n\n", u8IspIntEnable);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable DFMC ISP function. Before using DFMC function, it should unlock system register first. */
    DFMC_Open();

    if (u8IspIntEnable)
    {
        /* Enable ISP interrupt */
        DFMC_IRQInit(ENABLE);
    }
    else
    {
        /* Disable ISP interrupt */
        DFMC_IRQInit(DISABLE);
    }

    /* Clear ISP fail flag */
    DFMC_CLR_FAIL_FLAG();

    /* Clear ECC single error bit correction and double error bits detection flag */
    DFMC->ECCSTS = (DFMC_ECCSTS_ECCSEBCF_Msk | DFMC_ECCSTS_ECCDEBDF_Msk);

    while (1)
    {
        if (u8IspIntEnable == 0)
        {
            if (DFMC->ECCSTS & DFMC_ECCSTS_ECCSEBCF_Msk)
            {
                /* Print ECC single error fault address */
                printf("ECC Single Error Fault Address: 0x%08x\n", DFMC->ECCSEFAR);

                /* Read ECC data */
                if (DFMC_Read_ECC(DFMC->ECCSEFAR, g_u32Data) == 0)
                {
                    printf("ECC Single Error Bit location: 0x%2X\n", g_u32Data[2]);
                }
                else
                {
                    printf("DFMC Read Error!\n");
                }

                /* Clear ECC single error bit correction flag */
                DFMC->ECCSTS = DFMC_ECCSTS_ECCSEBCF_Msk;

                g_u32ECC_Error_Flag |= DFMC_ECCSTS_ECCSEBCF_Msk;
            }

            if (DFMC->ECCSTS & DFMC_ECCSTS_ECCDEBDF_Msk)
            {
                /* Print ECC double error fault address */
                printf("ECC Double Error Fault Address: 0x%08x\n", DFMC->ECCDEFAR);

                /* Clear ECC double error bits detection flag */
                DFMC->ECCSTS = DFMC_ECCSTS_ECCDEBDF_Msk;

                g_u32ECC_Error_Flag |= DFMC_ECCSTS_ECCDEBDF_Msk;
            }
        }

        if (g_u32ECC_Error_Flag)
        {
            if (g_u32ECC_Error_Flag & DFMC_ECCSTS_ECCDEBDF_Msk)
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

    DFMC_Close();                       /* Disable DFMC ISP function */

    DFMC_IRQInit(DISABLE);              /* Disable ISP interrupt */

    SYS_LockReg();                     /* Lock protected registers */

    printf("\nDFMC Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2017-2026 Nuvoton Technology Corp. ***/