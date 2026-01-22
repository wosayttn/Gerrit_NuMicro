/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Show the usage of EQEI compare function.
 *
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define EQEI0A   PC3
#define EQEI0B   PC4


void EQEI0_IRQHandler(void)
{
    if (EQEI_GET_INT_FLAG(EQEI0, EQEI_STATUS_CMPF_Msk))    /* Compare-match flag */
    {
        printf("Compare-match INT!\n\n");
        EQEI_CLR_INT_FLAG(EQEI0, EQEI_STATUS_CMPF_Msk);
    }

    if (EQEI_GET_INT_FLAG(EQEI0, EQEI_STATUS_OVUNF_Msk))   /* Counter Overflow or underflow flag */
    {
        printf("Overflow INT!\n\n");
        EQEI_CLR_INT_FLAG(EQEI0, EQEI_STATUS_OVUNF_Msk);
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock to 144MHz */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Enable all GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
    CLK_EnableModuleClock(GPE_MODULE);
    CLK_EnableModuleClock(GPF_MODULE);
    CLK_EnableModuleClock(GPG_MODULE);
    CLK_EnableModuleClock(GPH_MODULE);

    /* Enable DebugUART module clock */
    SetDebugUartCLK();

    /* Enable EQEI0 module clock */
    CLK_EnableModuleClock(EQEI0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for DebugUART RXD and TXD */
    SetDebugUartMFP();

    /* Set PA multi-function pins for EQEI0_A, EQEI0_B, EQEI0_INDEX */
    SET_EQEI0_A_PA4();
    SET_EQEI0_B_PA3();
    SET_EQEI0_INDEX_PA5();
}


int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init DebugUART for printf */
    InitDebugUart();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+--------------------------------------+\n");
    printf("|    EQEI Driver Sample Code           |\n");
    printf("+--------------------------------------+\n\n");
    printf("  >> Please connect PC.3 and PA.4 << \n");
    printf("  >> Please connect PC.4 and PA.3 << \n");
    printf("     Press any key to start test\n\n");
    getchar();
    printf("statrting ...\n\n");
    /* Configure PC.3 and PC.4 as output mode */
    GPIO_SetMode(PC, BIT3, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PC, BIT4, GPIO_MODE_OUTPUT);

    EQEI0A = 0;
    EQEI0B = 0;

    /* Set EQEI counting mode as X4 Compare-counting mode,
       set maximum counter value and enable IDX, QEA and QEB input */
    EQEI_Open(EQEI0, EQEI_CTL_X4_COMPARE_COUNTING_MODE, 0x20000);

    /* Set counter compare value */
    EQEI_SET_CNT_CMP(EQEI0, 0x10000);

    /* Enable compare function */
    EQEI_ENABLE_CNT_CMP(EQEI0);

    /* Enable EQEI interrupt */
    EQEI_EnableInt(EQEI0, EQEI_CTL_CMPIEN_Msk | EQEI_CTL_OVUNIEN_Msk);

    /* Enable EQEI IRQ */
    NVIC_EnableIRQ(EQEI0_IRQn);

    /* Start EQEI function */
    EQEI_Start(EQEI0);

    /* Wait compare-match and overflow interrupt happened */
    while (1)
    {
        EQEI0A = 1;
        CLK_SysTickDelay(16);
        EQEI0B = 1;
        CLK_SysTickDelay(16);
        EQEI0A = 0;
        CLK_SysTickDelay(16);
        EQEI0B = 0;
        CLK_SysTickDelay(16);
    }

}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/