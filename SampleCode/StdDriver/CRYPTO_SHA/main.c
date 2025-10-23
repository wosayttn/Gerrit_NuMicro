/**************************************************************************//**
* @file    main.c
* @version V1.00
* @brief   CRYPTO_SHA code for NuMicro M3351
*
* SPDX-License-Identifier: Apache-2.0
* @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"
#include "vector_parser.h"

#define SHA_TEST_DIGEST_LEN    20
extern void OpenTestVector(void);
extern int  GetNextPattern(void);


static volatile int g_SHA_done;
void CRYPTO_IRQHandler(void);
int  do_compare(uint8_t *output, uint8_t *expect, int cmp_len);
int32_t RunSHA(void);
void SYS_Init(void);
void DEBUG_PORT_Init(void);


void CRYPTO_IRQHandler(void)
{

    if (SHA_GET_INT_FLAG(CRYPTO))
    {
        g_SHA_done = 1;

        if (SHA_GET_INT_FLAG(CRYPTO)&CRYPTO_INTSTS_HMACEIF_Msk)
        {
            printf("SHA error flag is set!!\n");
        }

        SHA_CLR_INT_FLAG(CRYPTO);
    }
}

void SYS_Init(void)
{

    /* Enable Internal RC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
	
	/* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
	
	/* Enable PLL0 144MHz clock */
    CLK_SetCoreClock(FREQ_144MHZ);
	
	/* Update System Core Clock */
    SystemCoreClockUpdate();
	
    /* Enable CRYPTO module clock */
    CLK_EnableModuleClock(CRPT_MODULE);

    /* Enable UART0 module clock */
    SetDebugUartCLK();

    /* Set UART0 MFP */
    SetDebugUartMFP();

}

int  do_compare(uint8_t *output, uint8_t *expect, int cmp_len)
{
    int   i;

    if (memcmp(expect, output, (size_t)cmp_len))
    {
        printf("\nMismatch!! - %d\n", cmp_len);

        for (i = 0; i < cmp_len; i++)
            printf("0x%02x    0x%02x\n", expect[i], output[i]);

        return -1;
    }

    return 0;
}


int32_t RunSHA(void)
{
    uint32_t  au32OutputDigest[8];
    uint32_t u32TimeOutCnt;

    SHA_Open(CRYPTO, SHA_MODE_SHA1, SHA_IN_SWAP, 0);

    SHA_SetDMATransfer(CRYPTO, (uint32_t)&g_au8ShaData[0], (uint32_t)g_i32DataLen / 8);

    printf("Data len= %d bits\n", g_i32DataLen);

    g_SHA_done = 0;
    /* Start SHA calculation */
    SHA_Start(CRYPTO, CRYPTO_DMA_ONE_SHOT);

    /* Waiting for SHA calcuation done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (!g_SHA_done)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for SHA calcuation done time-out!\n");
            return (-1);
        }
    }

    /* Read SHA calculation result */
    SHA_Read(CRYPTO, au32OutputDigest);

    /* Compare calculation result with golden pattern */
    if (do_compare((uint8_t *)&au32OutputDigest[0], &g_au8ShaDigest[0], SHA_TEST_DIGEST_LEN) < 0)
    {
        printf("Compare error!\n");
        return (-1);
    }

    printf("Compare Pass!\n");
    return 0;
}

/*-----------------------------------------------------------------------------*/
int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART for printf */
    InitDebugUart();
    /* Lock protected registers */
    SYS_LockReg();

    printf("+-----------------------------------+\n");
    printf("|       Crypto SHA Sample Demo      |\n");
    printf("+-----------------------------------+\n");

    NVIC_EnableIRQ(CRYPTO_IRQn);
    SHA_ENABLE_INT(CRYPTO);

    /* Load test vector data base */
    OpenTestVector();

    while (1)
    {
        /* Get data from test vector to calcualte and
           compre the result with golden pattern */
        if (GetNextPattern() < 0)
            break;

        if (RunSHA() < 0)
            break;
    }

    printf("SHA test done.\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
