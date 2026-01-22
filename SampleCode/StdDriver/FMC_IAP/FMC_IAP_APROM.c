/******************************************************************************
* @file    main.c
* @version V1.00
* @brief   IAP boot from APROM sample code
*
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

typedef void (*PFN_FUNC_PTR)(void);
extern uint32_t  LDROM_IMAGE_BASE, LDROM_IMAGE_LIMIT;   /* Symbol of LDROM image start and end */

#if defined (__ICCARM__)
    #pragma section = "LoaderImage"
#endif

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
    CLK_EnableModuleClock(ISP_MODULE);

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

static int  LoadImage(uint32_t u32ImageBase, uint32_t u32ImageLimit, uint32_t u32FlashAddr, uint32_t u32MaxSize)
{
    uint32_t   u32i, u32j, u32Data, u32ImageSize, *pu32Loader;

    u32ImageSize = (u32ImageLimit - u32ImageBase);

    if (u32ImageSize > u32MaxSize)
    {
        printf("Image size: %d bytes > Max image: %d bytes\n", u32ImageSize, u32MaxSize);
        return -1;
    }

    printf("Program image of %d bytes to flash address 0x%x...", u32ImageSize, u32FlashAddr);
    pu32Loader = (uint32_t *)u32ImageBase;

    for (u32i = 0; u32i < u32ImageSize; u32i += FMC_FLASH_PAGE_SIZE)
    {
        FMC_Erase(u32FlashAddr + u32i);

        for (u32j = 0; u32j < FMC_FLASH_PAGE_SIZE; u32j += 8)
        {
            FMC_Write8Bytes(u32FlashAddr + u32i + u32j, pu32Loader[(u32i + u32j) / 4], pu32Loader[(u32i + u32j) / 4 + 1]);
        }
    }

    printf("OK.\n");

    printf("Verify ... ");

    /* Verify loader */
    for (u32i = 0; u32i < u32ImageSize; u32i += FMC_FLASH_PAGE_SIZE)
    {
        for (u32j = 0; u32j < FMC_FLASH_PAGE_SIZE; u32j += 4)
        {
            u32Data = FMC_Read(u32FlashAddr + u32i + u32j);

            if (u32Data != pu32Loader[(u32i + u32j) / 4])
            {
                printf("Data mismatch on 0x%x ! [0x%x] != [0x%x]\n", u32FlashAddr + u32i + u32j, u32Data, pu32Loader[(u32i + u32j) / 4]);
                return -1;
            }

            if (u32i + u32j >= u32ImageSize)
                break;
        }
    }

    printf("OK.\n");
    return 0;
}

int main(void)
{
    uint8_t     u8Item;
    uint32_t    u32Data;

    /* Init system clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART for print message */
    InitDebugUart();

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    printf("\n\n");
    printf("+-------------------------------+\n");
    printf("|      FMC IAP Sample Code      |\n");
    printf("|         [APROM code]          |\n");
    printf("+-------------------------------+\n");

    /* Enable FMC ISP function */
    FMC_Open();

    u32Data = FMC_ReadCID();
    printf("  Company ID .............. [0x%08x]\n", u32Data);

    u32Data = FMC_ReadPID();
    printf("  Product ID .............. [0x%08x]\n", u32Data);

    /* Read User Configuration */
    printf("  User Config 0 ........... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    printf("  User Config 1 ........... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE + 4));

    do
    {
        printf("\n\n\n");
        printf("+-------------------------------------+\n");
        printf("| Sample Code Menu                    |\n");
        printf("+-------------------------------------+\n");
        printf("| [0] Load IAP code to LDROM          |\n");
        printf("| [1] Run IAP code in LDROM           |\n");
        printf("+-------------------------------------+\n");
        u8Item = getchar();
        printf("Select [%c]\n", u8Item);

        switch (u8Item)
        {
            case '0':
                FMC_ENABLE_LD_UPDATE();

#if defined (__ICCARM__)

                if (LoadImage((uint32_t)&LDROM_IMAGE_BASE, (uint32_t)__section_end("LoaderImage"),
                              FMC_LDROM_BASE, FMC_LDROM_SIZE) != 0)
#else
                if (LoadImage((uint32_t)&LDROM_IMAGE_BASE, (uint32_t)&LDROM_IMAGE_LIMIT,
                              FMC_LDROM_BASE, FMC_LDROM_SIZE) != 0)
#endif
                {
                    printf("Load image to LDROM failed !\n");
                    goto lexit;
                }

                FMC_DISABLE_LD_UPDATE();
                break;

            case '1':
                printf("\n\nChange VECMAP and branch to LDROM ...\n");
                UART_WAIT_TX_EMPTY(DEBUG_PORT); /* To make sure all message has been print out */

                /* Mask all interrupt before changing VECMAP to avoid wrong interrupt handler fetched */
                __set_PRIMASK(1);

                /* Set VECMAP to LDROM for booting from LDROM */
                FMC_SetVectorPageAddr(FMC_LDROM_BASE);

                /* Software reset to boot to LDROM */
                NVIC_SystemReset();

                break;

            default :
                break;
        }
    } while (1);


lexit:
    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nEnd of FMC IAP Sample Code\n");

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
