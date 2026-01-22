/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Show DFMC erase, read, and write functions in non-blocking mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

#define DEF_TIMER_FREQ  FREQ_1MHZ

enum
{
    eDFMC_STATUS_IDLE = 0,
    eDFMC_STATUS_BUSY = 1,
    eDFMC_STATUS_FAIL = -1
};

volatile int32_t g_u32DFMC_Status = eDFMC_STATUS_IDLE;

void DFMC_IRQHandler(void)
{
    g_u32DFMC_Status = eDFMC_STATUS_IDLE;

    if (DFMC_GET_ISP_INT_FLAG())
    {
        DFMC_CLR_ISP_INT_FLAG();
    }

    if (DFMC_GET_FAIL_FLAG())
    {
        DFMC_CLR_FAIL_FLAG();
        g_u32DFMC_Status = eDFMC_STATUS_FAIL;
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

    /* Enable module clock */
    CLK_EnableModuleClock(DFMC_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    /* Select module clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

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

/**
  * @brief    Erase the entire DFMC data flash
  * @param    None
  * @retval   DFMC_ERR_TIMEOUT: Erase timeout
  * @retval   DFMC_ERR_FAIL   : Erase error
  * @retval   Others          : CPU loop count
  */
int32_t DFMC_MassEraseTest(void)
{
    int32_t i32Count = 0;

    DFMC_CLR_ISP_INT_FLAG();
    DFMC_ENABLE_ISP_INT();
    g_u32DFMC_Status = eDFMC_STATUS_BUSY;
    DFMC_Mass_Erase_NonBlocking();

    while (g_u32DFMC_Status == eDFMC_STATUS_BUSY)
    {
        if (i32Count++ == 0xFFFFFF)
        {
            DFMC_DISABLE_ISP_INT();
            return DFMC_ERR_TIMEOUT;
        }
    }

    DFMC_DISABLE_ISP_INT();

    if (g_u32DFMC_Status == eDFMC_STATUS_FAIL)
        return DFMC_ERR_FAIL;
    else
        return i32Count;
}

/**
  * @brief    Erase specified page of DFMC data flash
  * @param[in]  u32PageAddr Address of the flash page to be erased.
  * @retval   DFMC_ERR_TIMEOUT: Erase timeout
  * @retval   DFMC_ERR_FAIL   : Erase error
  * @retval   Others          : CPU loop count
  */
int32_t DFMC_PageEraseTest(uint32_t u32PageAddr)
{
    int32_t i32Count = 0;

    g_u32DFMC_Status = eDFMC_STATUS_BUSY;
    DFMC_CLR_ISP_INT_FLAG();
    DFMC_ENABLE_ISP_INT();

    DFMC_Erase_NonBlocking(u32PageAddr);

    while (g_u32DFMC_Status == eDFMC_STATUS_BUSY)
    {
        if (i32Count++ == 0xFFFFFF)
        {
            DFMC_DISABLE_ISP_INT();
            return DFMC_ERR_TIMEOUT;
        }
    }

    DFMC_DISABLE_ISP_INT();

    if (g_u32DFMC_Status == eDFMC_STATUS_FAIL)
        return DFMC_ERR_FAIL;
    else
        return i32Count;
}

/**
  * @brief    Write word to DFMC data flash
  * @param[in]  u32Data     Data to be programmed.
  * @param[in]  u32WriteAddr  Address of the flash location to be programmed.
  * @retval   DFMC_ERR_TIMEOUT: Program timeout
  * @retval   DFMC_ERR_FAIL   : Program error
  * @retval   Others          : CPU loop count
  */
int32_t DFMC_WriteWordTest(uint32_t u32WriteAddr, uint32_t u32Data)
{
    int32_t i32Count = 0;

    g_u32DFMC_Status = eDFMC_STATUS_BUSY;
    DFMC_CLR_ISP_INT_FLAG();
    DFMC_ENABLE_ISP_INT();
    DFMC_Write_NonBlocking(u32WriteAddr, u32Data);

    while (g_u32DFMC_Status == eDFMC_STATUS_BUSY)
    {
        if (i32Count++ == 0xFFFFFF)
        {
            DFMC_DISABLE_ISP_INT();
            return DFMC_ERR_TIMEOUT;
        }
    }

    DFMC_DISABLE_ISP_INT();

    if (g_u32DFMC_Status == eDFMC_STATUS_FAIL)
        return DFMC_ERR_FAIL;
    else
        return i32Count;
}

/**
  * @brief    Write data array to DFMC data flash
  * @param[in]  u32WriteAddr  Address of the flash location to be programmed.
  * @param[in]  pu32Data      Data pointer to be programmed.
  * @param[in]  u32WordCnt    Data word count to be programmed.
  * @retval   DFMC_ERR_TIMEOUT: Program timeout
  * @retval   DFMC_ERR_FAIL   : Program error
  * @retval   Others          : CPU loop count
  */
int32_t DFMC_WriteTest(uint32_t u32WriteAddr, uint32_t *pu32Data, uint32_t u32WordCnt)
{
    uint32_t i = 0, u32Addr;
    int32_t i32Count = 0, i32TotalCount = 0;

    DFMC_CLR_ISP_INT_FLAG();
    DFMC_ENABLE_ISP_INT();

    for (u32Addr = u32WriteAddr; i < u32WordCnt; u32Addr += 4)
    {
        i32Count = 0;
        g_u32DFMC_Status = eDFMC_STATUS_BUSY;
        DFMC_Write_NonBlocking(u32Addr, pu32Data[i]);

        while (g_u32DFMC_Status == eDFMC_STATUS_BUSY)
        {
            if (i32Count++ == 0xFFFFFF)
            {
                DFMC_DISABLE_ISP_INT();
                return DFMC_ERR_TIMEOUT;
            }
        }

        if (g_u32DFMC_Status == eDFMC_STATUS_FAIL)
            break;

        i++;
        i32TotalCount += i32Count;
    }

    DFMC_DISABLE_ISP_INT();

    if (g_u32DFMC_Status == eDFMC_STATUS_FAIL)
        return DFMC_ERR_FAIL;
    else
        return i32TotalCount;
}

int32_t  flash_test(uint32_t u32TestAddr)
{
    uint32_t    u32TimerCnt, i, u32CpuCounter;
    uint32_t    au32DataBuf[100];

    for (i = 0; i < (sizeof(au32DataBuf) / sizeof(au32DataBuf[0])); i++)
        au32DataBuf[i] = i;

    printf("    DataFlash test address: 0x%x    \n", u32TestAddr);

    TIMER_SET_PRESCALE_VALUE(TIMER0, ((__HIRC / DEF_TIMER_FREQ) - 1));
    TIMER_SET_OPMODE(TIMER0, TIMER_PERIODIC_MODE);
    TIMER_SET_CMP_VALUE(TIMER0, 0xFFFFFF);

    printf("    DataFlash Word Write Test    \n");
    TIMER_Start(TIMER0);
    u32TimerCnt   = TIMER_GetCounter(TIMER0);
    u32CpuCounter = DFMC_WriteWordTest(u32TestAddr, 0x5AA55AA5);
    u32TimerCnt   = TIMER_GetCounter(TIMER0) - u32TimerCnt;
    printf("    Timer counter is %7d, period is %f Sec , CPU Count = %6d   \n\n", u32TimerCnt, ((double)u32TimerCnt / DEF_TIMER_FREQ), u32CpuCounter);

    printf("    DataFlash 100 Words Write Test    \n");
    TIMER_ResetCounter(TIMER0);
    u32TimerCnt   = TIMER_GetCounter(TIMER0);
    u32CpuCounter = DFMC_WriteTest(u32TestAddr, au32DataBuf, 100);
    u32TimerCnt   = TIMER_GetCounter(TIMER0) - u32TimerCnt;
    printf("    Timer counter is %7d, period is %f Sec , CPU Count = %6d   \n\n", u32TimerCnt, ((double)u32TimerCnt / DEF_TIMER_FREQ), u32CpuCounter);

    printf("    DataFlash Page Erase Test    \n");
    TIMER_ResetCounter(TIMER0);
    u32TimerCnt   = TIMER_GetCounter(TIMER0);
    u32CpuCounter = DFMC_PageEraseTest(u32TestAddr);
    u32TimerCnt   = TIMER_GetCounter(TIMER0) - u32TimerCnt;
    printf("    Timer counter is %7d, period is %f Sec , CPU Count = %6d   \n\n", u32TimerCnt, ((double)u32TimerCnt / DEF_TIMER_FREQ), u32CpuCounter);

    printf("    DataFlash Mass Erase Test    \n");
    TIMER_ResetCounter(TIMER0);
    u32TimerCnt   = TIMER_GetCounter(TIMER0);
    u32CpuCounter = DFMC_MassEraseTest();
    u32TimerCnt   = TIMER_GetCounter(TIMER0) - u32TimerCnt;
    printf("    Timer counter is %7d, period is %f Sec , CPU Count = %6d   \n\n", u32TimerCnt, ((double)u32TimerCnt / DEF_TIMER_FREQ), u32CpuCounter);

    return 0;
}

int32_t main(void)
{
    uint32_t u32Data;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("\n\n");
    printf("+-----------------------------------------+\n");
    printf("|      DFMC Non-Blocking Sample Code      |\n");
    printf("+-----------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable DFMC ISP function. Before using DFMC function, it should unlock system register first. */
    DFMC_Open();

    u32Data = DFMC_ReadCID();
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = DFMC_ReadPID();
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    NVIC_EnableIRQ(DFMC_IRQn);

    printf("\n\nDataflash test =>\n");

    DFMC_ENABLE_UPDATE();

    if (flash_test(DFMC_DFLASH_BASE) < 0)
    {
        printf("\n\nDataflash test failed!\n");
        goto lexit;
    }

    DFMC_DISABLE_UPDATE();

lexit:

    NVIC_DisableIRQ(DFMC_IRQn);

    /* Disable DFMC ISP function */
    DFMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nDFMC Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
