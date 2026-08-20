/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 23/02/24 4:40p $
 * @brief    Demonstrate how to program, read, and lock OTP.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/* To enable/disable OTP to be programed (Note: Generally, programing OTP is not allowed)  */
#define PROGRAM_OTP     0

#if PROGRAM_OTP
static volatile uint8_t g_u8ProgAbortFlag;
#endif

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

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

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();

}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int32_t main(void)
{
    uint32_t    u32i, u32OtpHw, u32OtpLw;
#if PROGRAM_OTP
    int32_t     i32Ret, i32GetCh;
#endif

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("+-----------------------------------+\n");
    printf("|        RMC OTP Sample Demo        |\n");
    printf("+-----------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable RMC ISP function. Before using RMC function, it should unlock system register first. */
    RMC_Open();

    for(u32i = 0; u32i < RMC_OTP_ENTRY_CNT; u32i++)
    {
        if(RMC_ReadOTP(u32i, &u32OtpLw, &u32OtpHw) != 0)
        {
            printf("Read OTP%d failed!\n", u32i);
            goto lexit;
        }

        if((u32OtpLw == 0xFFFFFFFF) && (u32OtpHw == 0xFFFFFFFF))
        {
            printf("OTP%d is 0xFFFFFFFF-0xFFFFFFFF. It should be a free entry.\n", u32i);
            break;
        }
    }

    if(u32i == RMC_OTP_ENTRY_CNT)
    {
        printf("All OTP entries are used.\n");
        goto lexit;
    }

#if PROGRAM_OTP
    g_u8ProgAbortFlag = 0;
    printf("\nOTP%d is a free entry?   \n", u32i);
    printf("Program OTP%d?  (y/n) \n", u32i);

    i32GetCh = getchar();

    if(i32GetCh == 'y' || i32GetCh == 'Y')
    {
        printf("\n\nWarnning: \n");
        printf("          If OTP is programed, the original OTP data could be destroyed.\n");
        printf("          If OTP is programed, each bit only can be programed from 1 to 0  \n");
        printf("          If OTP is programed, it could not be erased(even mass erase)     \n");
        printf("\n\nReally program OTP?  (y/n)   \n");
        i32GetCh = getchar();
        if(i32GetCh == 'y' || i32GetCh == 'Y')
        {
            printf("Program OTP%d with 0x%x-0x%x...\n", u32i, 0x5A5A0000 | u32i, 0x00005A5A | u32i);

            if(RMC_WriteOTP(u32i, 0x5A5A0000 | u32i, 0x00005A5A | u32i) != 0)
            {
                printf("Failed to program OTP%d!\n", u32i);
                goto lexit;
            }
        }
        else
        {
            printf("Abort programing OTP\n");
            g_u8ProgAbortFlag = 1;
        }
    }
    else
    {
        printf("Abort programing OTP\n");
        g_u8ProgAbortFlag = 1;
    }
#endif

    if(RMC_ReadOTP(u32i, &u32OtpLw, &u32OtpHw) != 0)
    {
        printf("Read OTP%d failed after programmed!\n", u32i);
        goto lexit;
    }

    printf("Read back OTP%d: 0x%x-0x%x.\n", u32i, u32OtpLw, u32OtpHw);

#if PROGRAM_OTP
    if(g_u8ProgAbortFlag == 0)
    {
        if((u32OtpLw != (0x5A5A0000 | u32i)) || (u32OtpHw != (0x00005A5A | u32i)))
        {
            printf("OTP%d value is not matched with programmed value!\n", u32i);
            goto lexit;
        }
    }

    i32Ret = RMC_IsOTPLocked(u32i);

    if(i32Ret == 0)
    {
        printf("\nOTP%d is still not locked   \n", u32i);
        printf("Lock OTP%d?  (y/n)   \n", u32i);

        i32GetCh = getchar();

        if((i32GetCh == 'y' || i32GetCh == 'Y') && (g_u8ProgAbortFlag == 0))
        {
            printf("\n\nWarnning: \n");
            printf("          If OTP lock is programed, each bit only can be programed from 1 to 0  \n");
            printf("          If OTP is locked, it could not be erased(even mass erase)     \n");
            printf("\n\nReally lock OTP?  (y/n)   \n");
            i32GetCh = getchar();
            if(i32GetCh == 'y' || i32GetCh == 'Y')
            {

                printf("Lock OTP%d...\n", u32i);

                if(RMC_LockOTP(u32i) != 0)
                {
                    printf("Failed to lock OTP%d!\n", u32i);
                    goto lexit;
                }
            }
            else
            {
                printf("Abort OTP lock\n");
                g_u8ProgAbortFlag = 1;
            }
        }
        else
        {
            printf("Abort OTP lock\n");
            g_u8ProgAbortFlag = 1;
        }
    }
    else if(i32Ret == 1)
    {
        printf("\nOTP%d is is already locked   \n", u32i);
    }
    else
    {
        printf("\nRead OTP%d lock failed!\n", u32i);
    }

#endif

    if(RMC_ReadOTP(u32i, &u32OtpLw, &u32OtpHw) != 0)
    {
        printf("Read OTP%d failed after programmed!\n", u32i);
        goto lexit;
    }

    printf("Read OTP%d locked: 0x%x-0x%x.\n", u32i, u32OtpLw, u32OtpHw);

#if PROGRAM_OTP
    if(g_u8ProgAbortFlag == 0)
    {
        if((u32OtpLw != (0x5A5A0000 | u32i)) || (u32OtpHw != (0x00005A5A | u32i)))
        {
            printf("OTP%d value is incorrect after locked!\n", u32i);
            goto lexit;
        }
    }
    g_u8ProgAbortFlag = 0;
#endif

    printf("OTP demo done.\n");

lexit:
    RMC_Close();                       /* Disable RMC ISP function */
    SYS_LockReg();                     /* Lock protected registers */

    while(1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/


