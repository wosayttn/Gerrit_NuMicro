/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   MP3 player sample plays MP3 files stored on USBH mass storage.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"
#include "config.h"
#include "diskio.h"
#include "ff.h"
#include "usbh_lib.h"

//------------------------------------------------------------------------------
#ifdef __ICCARM__
    #pragma data_alignment=32
    /* DMA descriptor table, must be aligned to 32 bytes */
    DMA_DESC_T DMA_DESC[2];
#else
    /* DMA descriptor table, must be aligned to 32 bytes */
    DMA_DESC_T DMA_DESC[2] __attribute__((aligned(32)));
#endif

extern signed int aPCMBuffer[2][PCM_BUFFER_SIZE];
extern void MP3_Storage_Init(TCHAR *path);

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

DWORD get_fattime(void)
{
    DWORD tmr;

    tmr = 0x00000;

    return tmr;
}

void SYS_Init(void)
{
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 144MHz */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

#if (USE_I2S == 1)
    /* Enable I2S0 module clock */
    CLK_EnableModuleClock(I2S0_MODULE);

    /* Select source from HIRC(12MHz) */
    CLK_SetModuleClock(I2S0_MODULE, CLK_I2SSEL_I2S0SEL_HIRC, MODULE_NoMsk);

    /* Enable I2C module clock */
    CLK_EnableModuleClock(I2C3_MODULE);

#else
    /* Enable SPI peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Select HIRC as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_HIRC, MODULE_NoMsk);

    /* Enable I2C clock */
    CLK_EnableModuleClock(I2C1_MODULE);

#endif

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable all GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
    CLK_EnableModuleClock(GPE_MODULE);
    CLK_EnableModuleClock(GPF_MODULE);
    CLK_EnableModuleClock(GPG_MODULE);
    CLK_EnableModuleClock(GPH_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

#if (USE_I2S == 1)
    /* Set multi-function pins for I2S0 */
    SET_I2S0_BCLK_PI6();
    SET_I2S0_MCLK_PI7();
    SET_I2S0_DI_PI8();
    SET_I2S0_DO_PI9();
    SET_I2S0_LRCK_PI10();

    /* Enable I2S0 clock pin (PI6) schmitt trigger */
    PI->SMTEN |= GPIO_SMTEN_SMTEN6_Msk;

    /* Set I2C3 multi-function pins */
    SET_I2C3_SDA_PG1();
    SET_I2C3_SCL_PG0();

    /* Enable I2C3 clock pin (PG0) schmitt trigger */
    PG->SMTEN |= GPIO_SMTEN_SMTEN0_Msk;

#else
    /* Setup SPI0 multi-function pins */
    /* PA.3 is SPI0_SS,   PA.2 is SPI0_CLK,
       PA.1 is SPI0_MISO, PA.0 is SPI0_MOSI*/
    SET_SPI0_SS_PA3();
    SET_SPI0_CLK_PA2();
    SET_SPI0_MOSI_PA0();
    SET_SPI0_MISO_PA1();

    /* Enable SPII2S0 clock pin (PA.2) schmitt trigger */
    PA->SMTEN |= (GPIO_SMTEN_SMTEN2_Msk);

    /* PA.4 is SPI0_I2SMCLK */
    SET_SPI0_I2SMCLK_PA4();

    /* Set PB multi-function pins for I2C1 */
    SYS->GPB_MFP0 = SYS_GPB_MFP0_PB1MFP_I2C1_SCL | SYS_GPB_MFP0_PB0MFP_I2C1_SDA;

    /* Enable I2C clock pinschmitt trigger */
    PB->SMTEN |= GPIO_SMTEN_SMTEN1_Msk;
#endif
}

void I2C_Init(void)
{
    /* Open I2C and set clock to 100k */
    I2C_Open(I2C_PORT, 100000);
}

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetTxSGTable(uint8_t id)
{
    DMA_DESC[id].ctl = (DMA_DESC[id].ctl & ~PDMA_DSCT_CTL_TXCNT_Msk) |
                       ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
                       PDMA_OP_SCATTER;
}

/* Configure PDMA to Scatter Gather mode */
void PDMA_Init(void)
{
    uint32_t u32Peripheral = 0;

#if (USE_I2S == 1)
    DMA_DESC[0].ctl = ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_DESC[0].src = (uint32_t)&aPCMBuffer[0][0];
    DMA_DESC[0].dest = (uint32_t)&I2S_PORT->TXFIFO;
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1];

    DMA_DESC[1].ctl = ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_DESC[1].src = (uint32_t)&aPCMBuffer[1][0];
    DMA_DESC[1].dest = (uint32_t)&I2S_PORT->TXFIFO;
    DMA_DESC[1].offset = (uint32_t)&DMA_DESC[0];

    u32Peripheral = PDMA_I2S0_TX;
#else
    DMA_DESC[0].ctl = ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_DESC[0].src = (uint32_t)&aPCMBuffer[0][0];
    DMA_DESC[0].dest = (uint32_t)&SPI_PORT->TX;
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1];

    DMA_DESC[1].ctl = ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_DESC[1].src = (uint32_t)&aPCMBuffer[1][0];
    DMA_DESC[1].dest = (uint32_t)&SPI_PORT->TX;
    DMA_DESC[1].offset = (uint32_t)&DMA_DESC[0];

    u32Peripheral = PDMA_SPI0_TX;
#endif

    PDMA_Open(PDMA_PORT, 1 << I2S_TX_DMA_CH);

    PDMA_SetTransferMode(PDMA_PORT, I2S_TX_DMA_CH, u32Peripheral, 1, (uint32_t)&DMA_DESC[0]);

    PDMA_EnableInt(PDMA_PORT, I2S_TX_DMA_CH, PDMA_INT_TRANS_DONE);

    NVIC_EnableIRQ(PDMA0_IRQn);
}

int32_t main(void)
{
    TCHAR path[] = {'0', ':', 0};

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("+-----------------------------------------------------------------------+\n");
    printf("|                   MP3 Player Sample with audio codec                  |\n");
    printf("+-----------------------------------------------------------------------+\n");
    printf(" Please put MP3 files on storage \n");

    /* Init I2C to access codec */
    I2C_Init();

    MP3_Storage_Init(path);

    // Set default path
    f_chdrive(path);

    /* Lock protected registers */
    SYS_LockReg();

    while (1)
    {
        /* Play mp3 */
#if (USE_SDH == 1)
        if (SD0.IsCardInsert == TRUE)
#elif (USE_USBH == 1)
        if (usbh_pooling_hubs())

#endif
        {
            MP3Player();
        }
    }
}
