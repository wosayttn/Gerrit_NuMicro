/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   CRYPTO_AES code for NuMicro M3351
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


static uint32_t au32MyAESKey[8] =
{
    0x00010203, 0x04050607, 0x08090a0b, 0x0c0d0e0f,
    0x00010203, 0x04050607, 0x08090a0b, 0x0c0d0e0f
};

static uint32_t au32MyAESIV[4] =
{
    0x00000000, 0x00000000, 0x00000000, 0x00000000
};


#define AES_INPUT_DATA_SIZE         (16 * 1)
#define AES_OUTPUT_DATA_SIZE        (1024)

//------------------------------------------------------------------------------
// Standard buffer alignment when DCache is disabled
__attribute__((aligned(4))) static uint8_t au8InputData[AES_INPUT_DATA_SIZE] =
{
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
    0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff
};

// Standard buffer alignment when DCache is disabled
__attribute__((aligned(4))) static uint8_t au8OutputData[AES_OUTPUT_DATA_SIZE];

static volatile int32_t  g_AES_done;

void CRYPTO_IRQHandler(void);
void DumpBuffHex(uint8_t *pucBuff, int nBytes);
void SYS_Init(void);
void DEBUG_PORT_Init(void);


void CRYPTO_IRQHandler(void)
{
    if (AES_GET_INT_FLAG(CRYPTO))
    {
        g_AES_done = 1;
        AES_CLR_INT_FLAG(CRYPTO);
    }
}


void DumpBuffHex(uint8_t *pucBuff, int nBytes)
{
    int32_t i32Idx, i;

    i32Idx = 0;

    while (nBytes > 0)
    {
        printf("0x%04X  ", i32Idx);

        for (i = 0; i < 16; i++)
            printf("%02x ", pucBuff[i32Idx + i]);

        printf("  ");

        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[i32Idx + i] >= 0x20) && (pucBuff[i32Idx + i] < 127))
                printf("%c", pucBuff[i32Idx + i]);
            else
                printf(".");

            nBytes--;
        }

        i32Idx += 16;
        printf("\n");
    }

    printf("\n");
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


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART for printf */
    InitDebugUart();
    /* Lock protected registers */
    SYS_LockReg();

    printf("+---------------------------------------+\n");
    printf("|     Crypto AES Driver Sample Code     |\n");
    printf("+---------------------------------------+\n");

    NVIC_EnableIRQ(CRYPTO_IRQn);
    AES_ENABLE_INT(CRYPTO);

    /*---------------------------------------
     *  AES-128 ECB mode encrypt
     *---------------------------------------*/
    AES_Open(CRYPTO, 0, 1, AES_MODE_ECB, AES_KEY_SIZE_128, AES_IN_OUT_SWAP);
    /*Load Key and IV to AES register*/
    AES_SetKey(CRYPTO, 0, au32MyAESKey, AES_KEY_SIZE_128);
    AES_SetInitVect(CRYPTO, 0, au32MyAESIV);

    //au8InputData  Dcache is cleaned at startup
    /*Trigger AES DMA*/
    AES_SetDMATransfer(CRYPTO, 0, (uint32_t)au8InputData, (uint32_t)au8OutputData, sizeof(au8InputData));
    g_AES_done = 0;
    /* Start AES Encrypt */
    AES_Start(CRYPTO, 0, CRYPTO_DMA_ONE_SHOT);

    /* Waiting for AES calculation */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (!g_AES_done)
    {
        if (--u32TimeOutCnt == 0)
        {
            //printf("Wait for AES encrypt done time-out!\n");
            return -1;
        }
    }

    printf("AES encrypt done.\n\n");
    DumpBuffHex(au8OutputData, sizeof(au8InputData));

    /*-------------------------------------------------------------
     *  AES-128 ECB mode decrypt, exchange inputdata and outputdata
     *-------------------------------------------------------------*/
    AES_Open(CRYPTO, 0, 0, AES_MODE_ECB, AES_KEY_SIZE_128, AES_IN_OUT_SWAP);
    /*Load Key and IV to AES register*/
    AES_SetKey(CRYPTO, 0, au32MyAESKey, AES_KEY_SIZE_128);
    AES_SetInitVect(CRYPTO, 0, au32MyAESIV);
    AES_SetDMATransfer(CRYPTO, 0, (uint32_t)au8OutputData, (uint32_t)au8InputData, sizeof(au8InputData));

    g_AES_done = 0;
    /* Start AES decrypt */
    AES_Start(CRYPTO, 0, CRYPTO_DMA_ONE_SHOT);

    /* Waiting for AES calculation */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (!g_AES_done)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for AES decrypt done time-out!\n");
            return -1;
        }
    }

    printf("AES decrypt done.\n\n");
    DumpBuffHex(au8InputData, sizeof(au8InputData));

    while (1);
}
