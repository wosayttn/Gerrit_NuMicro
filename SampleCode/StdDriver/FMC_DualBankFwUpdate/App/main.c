/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Bank Remap sample code(Bank/Segment0 App).
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "NuDB_common.h"
#include "xmodem.h"


#define FMC_SEGMENT_SIZE (FMC_APROM_SIZE>>1)

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t  s_u32DbLength;             /* Dual bank/segment program remaining length         */
static volatile uint32_t  s_u32DbAddr;               /* Dual bank/segment program current flash address    */

/*---------------------------------------------------------------------------------------------------------*/
/* Global Functions Declaration                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
int main(void);


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

int main(void)
{
    uint32_t u32ch;
    int32_t  i32Err;
    uint32_t u32TimeOutCnt;

    /* CPU executing in which Bank */
    uint32_t  u32ExecBank = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Set Vector Table Offset Register */
    SCB->VTOR = APP_BASE;

    /* Enable ISP and APROM update */
    FMC_ENABLE_ISP();
    FMC_ENABLE_AP_UPDATE();

    do
    {
        printf("\n\n");
        printf("+------------------------+\n");
        printf("|  Boot from 0x%08X  |\n", FMC_GetVECMAP());
        printf("+------------------------+\n");

        u32ExecBank = (uint32_t)((FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) >> FMC_ISPSTS_FBS_Pos);

#ifdef NewApp
        printf("\n Bank/Segment%d APP processing (New Firmware!!!)\n", u32ExecBank);
#else
        printf("\n Bank/Segment%d APP processing \n", u32ExecBank);
#endif

        printf("\n Download new FW?[y/n]\n");
        u32ch = (uint32_t)getchar();
        if(u32ch == 'y')
        {
            printf("\n Bank/Segment%d processing, download data to Bank/Segment%d.\n", u32ExecBank, u32ExecBank ^ 1);

            /* Dual bank background program address */
            s_u32DbAddr   = FMC_SEGMENT_SIZE * (u32ExecBank ^ 1) + APP_BASE;
            /* Dual bank background length */
            s_u32DbLength = APP_SIZE;

            i32Err = Xmodem(s_u32DbAddr);
            if(i32Err < 0)
            {
                printf("Xmodem transfer fail!\n");
                while(1);
            }
            else
            {
                printf("Xomdem transfer done!\n");
                printf("Total transfer size is %d\n", i32Err);
            }

            printf("\n Firmware download completed!!\n");

        }
        else
        {
            printf("\n Reset from Bank/Segment%d Loader \n", u32ExecBank);
            UART_WAIT_TX_EMPTY(DEBUG_PORT);

            /* Forces a write of all user-space buffered data for the given output */
            fflush(stdout);

            /* Remap to Loader */
            FMC->ISPCMD = FMC_ISPCMD_VECMAP;    /* Set ISP Command Code */
            FMC->ISPADDR = 0;                   /* The address of specified page which will be map to address 0x0. It must be page alignment. */
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk; /* Trigger to start ISP procedure */

            u32TimeOutCnt = SystemCoreClock;    /* 1 second time-out */
            while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for FMC operation finished time-out!\n");
                    break;
                }
            }

            if(u32TimeOutCnt != 0)
            {
                SYS_ResetCPU();
            }
        }

    }
    while(1);

}
