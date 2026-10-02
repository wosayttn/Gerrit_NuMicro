/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 20/08/10 1:55p $
 * @brief    Show DFMC read flash IDs, erase, read, and write functions.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

int32_t volatile gDFMC_Busy = 0;

void FMC_IRQHandler(void)
{
    gDFMC_Busy = 0;

    if (DFMC_GET_ISP_INT_FLAG())
    {
        DFMC_CLEAR_ISP_INT_FLAG();
    }

    if (DFMC_GET_FAIL_FLAG())
    {
        DFMC_CLR_FAIL_FLAG(); 
        gDFMC_Busy = -1;
    }
}
int IsDebugFifoEmpty(void);


/**
  * @brief    Erase the entire Flash
  * @param    None
  * @return    0 : Timeout
  *           -1 : ISPFF is set
  *           Others : CPU Count
  * @details  This function is used to read product ID.
  */
int32_t DFMC_MassEraseTest(void)
{
    int32_t count = 0;
    DFMC_CLEAR_ISP_INT_FLAG();
    DFMC_ENABLE_ISP_INT();
    gDFMC_Busy = 1;
    DFMC_Mass_Erase_NonBlocking();

    while (gDFMC_Busy == 1)
    {
        if(count++ == 0xFFFFFF)
        {
            DFMC_DISABLE_ISP_INT();
            return FALSE;
        }
    }

    DFMC_DISABLE_ISP_INT();
    if(gDFMC_Busy == -1)
        return gDFMC_Busy;
    else
        return count;
}

/**
  * @brief    Erase one page
  * @param    None
  * @return    0 : Timeout
  *           -1 : ISPFF is set
  *           Others : CPU Count
  * @details  This function is used to read product ID.
  */
int32_t DFMC_PageEraseTest(uint32_t address)
{
    int32_t count = 0;
    gDFMC_Busy = 1;
    DFMC_CLEAR_ISP_INT_FLAG();
    DFMC_ENABLE_ISP_INT();

    DFMC_Erase_NonBlocking(address);

    while (gDFMC_Busy == 1)
    {
        if(count++ == 0xFFFFFF)
        {
            DFMC_DISABLE_ISP_INT();
            return FALSE;
        }
    }

    DFMC_DISABLE_ISP_INT();
    if(gDFMC_Busy == -1)
        return gDFMC_Busy;
    else
        return count;
}

/**
  * @brief    Write data array to Flash
  * @param[in]  pdata    Data pointer to be programmed.
  * @param[in]  address  Address of the flash location to be programmed.
  * @param[in]  size     Data size to be programmed.
  * @return    0 : Timeout
  *           -1 : ISPFF is set
  *           Others : CPU Count
  * @details  This function is used to read product ID.
  */
int32_t DFMC_WriteTest(uint32_t* pdata, uint32_t address, uint32_t size)
{
    uint32_t i =0, u32Addr;
    int32_t count = 0;
    DFMC_CLEAR_ISP_INT_FLAG();
    DFMC_ENABLE_ISP_INT();

    for (u32Addr = address; i < size; u32Addr += 4)
    {
        gDFMC_Busy = 1;
        DFMC_Write_NonBlocking(u32Addr, pdata[i]);

        while (gDFMC_Busy == 1)
        {
            if(count++ == 0xFFFFFF)
            {
                DFMC_DISABLE_ISP_INT();
                return FALSE;
            }
        }
        i++;
    }
    DFMC_DISABLE_ISP_INT();
    if(gDFMC_Busy == -1)
        return gDFMC_Busy;
    else
        return count;
}

/**
  * @brief    Write data to Flash
  * @param[in]  data     Data to be programmed.
  * @param[in]  address  Address of the flash location to be programmed.
  * @return    0 : Timeout
  *           -1 : ISPFF is set
  *           Others : CPU Count
  * @details  This function is used to read product ID.
  */
uint32_t DFMC_WriteWordTest(uint32_t data, uint32_t address)
{
    int32_t count = 0;
    gDFMC_Busy = 1;
    DFMC_CLEAR_ISP_INT_FLAG();
    DFMC_ENABLE_ISP_INT();
    DFMC_Write_NonBlocking(address, data);

    while (gDFMC_Busy == 1)
    {
        if(count++ == 0xFFFFFF)
        {
            DFMC_DISABLE_ISP_INT();
            return FALSE;
        }
    }

    DFMC_DISABLE_ISP_INT();
    if(gDFMC_Busy == -1)
        return gDFMC_Busy;
    else
        return count;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 120 MHz from PLL */
    CLK_SetCoreClock(FREQ_120MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    /* Select IP clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);
    
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
    /* Lock protected registers */
    SYS_LockReg();
}

int32_t  flash_test(uint32_t u32TestAddr)
{
    uint32_t    time_cnt, i, CpuCounter;
    uint32_t    data[100];
    
    for(i=0;i<100;i++)
        data[i]=i;

    printf("    DataFlash test address: 0x%x    \n", u32TestAddr);
    
    printf("    DataFlash Word Write Test    \n");

    TIMER0->CTL = TIMER_PERIODIC_MODE;
    TIMER0->CMP = 0xFFFFFF;

    TIMER_Start(TIMER0);
    time_cnt = TIMER_GetCounter(TIMER0);
    CpuCounter = DFMC_WriteWordTest(0x5AA55AA5, u32TestAddr);
    time_cnt = TIMER_GetCounter(TIMER0) - time_cnt;
    printf("    Time counter is %7d, period is %f Sec , CPU Count = %6d   \n\n", time_cnt, time_cnt/(float)__HIRC, CpuCounter);
    
    printf("    DataFlash 100 Words Write Test    \n");
    TIMER_ResetCounter(TIMER0);
    time_cnt = TIMER_GetCounter(TIMER0);
    CpuCounter = DFMC_WriteTest(data, u32TestAddr, 100);

    time_cnt = TIMER_GetCounter(TIMER0) - time_cnt;
    printf("    Time counter is %7d, period is %f Sec , CPU Count = %6d   \n\n", time_cnt, time_cnt/(float)__HIRC, CpuCounter);
    
    printf("    DataFlash Page Erase Test    \n");
    TIMER_ResetCounter(TIMER0);
    time_cnt = TIMER_GetCounter(TIMER0);
    CpuCounter = DFMC_PageEraseTest(u32TestAddr);
    time_cnt = TIMER_GetCounter(TIMER0) - time_cnt;
    printf("    Time counter is %7d, period is %f Sec , CPU Count = %6d   \n\n", time_cnt, time_cnt/(float)__HIRC, CpuCounter);
    
    printf("    DataFlash Chip Erase Test    \n");
    TIMER_ResetCounter(TIMER0);
    time_cnt = TIMER_GetCounter(TIMER0);
    CpuCounter = DFMC_MassEraseTest();

    time_cnt = TIMER_GetCounter(TIMER0) - time_cnt;
    printf("    Time counter is %7d, period is %f Sec , CPU Count = %6d   \n\n", time_cnt, time_cnt/(float)__HIRC, CpuCounter);

    return 0;
}

int main()
{
    uint32_t    u32Data;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|    M471 DFMC Non-Blocking Sample Code  |\n");
    printf("+----------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable DFMC ISP function. Before using DFMC function, it should unlock system register first. */
    DFMC_Open();

    u32Data = DFMC_ReadCID();
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = DFMC_ReadPID();
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    NVIC_EnableIRQ(FMC_IRQn);

    printf("\n\nDataflash test =>\n");

    DFMC_ENABLE_UPDATE();

    if (flash_test(DFMC_DFLASH_BASE) < 0)
    {
        printf("\n\nDataflash test failed!\n");
        goto lexit;
    }
    DFMC_DISABLE_UPDATE();

lexit:

    NVIC_DisableIRQ(FMC_IRQn);

    /* Disable DFMC ISP function */
    DFMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nDFMC Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
