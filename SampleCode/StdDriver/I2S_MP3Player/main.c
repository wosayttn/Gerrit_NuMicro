/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   MP3 player sample plays MP3 files stored on SD memory card.
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

//------------------------------------------------------------------------------
#ifdef __ICCARM__
    #pragma data_alignment=32
    #if (NVT_DCACHE_ON == 1)
        /* DMA descriptor table, must be aligned to 32 bytes and placed in DTCM */
        NVT_NONCACHEABLE __ALIGNED(32) DMA_DESC_T DMA_DESC[2];
    #else
        /* DMA descriptor table, must be aligned to 32 bytes */
        DMA_DESC_T DMA_DESC[2];
    #endif
#else
    #if (NVT_DCACHE_ON == 1)
        /* DMA descriptor table, must be aligned to 32 bytes and placed in DTCM */
        NVT_NONCACHEABLE __ALIGNED(32) DMA_DESC_T DMA_DESC[2];
    #else
        /* DMA descriptor table, must be aligned to 32 bytes */
        DMA_DESC_T DMA_DESC[2] __attribute__((aligned(32)));
    #endif
#endif

extern signed int aPCMBuffer[2][PCM_BUFFER_SIZE];
extern uint32_t volatile sd_init_ok;
extern int32_t SDH_Open_Disk(SDH_T *sdh, uint32_t u32CardDetSrc);

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

NVT_ITCM void SDH0_IRQHandler(void)
{
    unsigned int volatile isr;
    unsigned int volatile ier;
    volatile uint32_t u32CDState;

    // FMI data abort interrupt
    if (SDH0->GINTSTS & SDH_GINTSTS_DTAIF_Msk)
    {
        /* ResetAllEngine() */
        SDH0->GCTL |= SDH_GCTL_GCTLRST_Msk;
    }

    //----- SD interrupt status
    isr = SDH0->INTSTS;
    ier = SDH0->INTEN;

    if (isr & SDH_INTSTS_BLKDIF_Msk)
    {
        // block down
        SD0.DataReadyFlag = TRUE;
        SDH0->INTSTS = SDH_INTSTS_BLKDIF_Msk;
        //printf("SD block down\r\n");
    }

    if ((ier & SDH_INTEN_CDIEN_Msk) &&
            (isr & SDH_INTSTS_CDIF_Msk))    // card detect
    {
        //----- SD interrupt status
        // delay 50 us to sync the GPIO and SDH
        {
            (void)SDH0->INTSTS;

            CLK_SysTickDelay(50);

            isr = SDH0->INTSTS;
        }

        u32CDState = (((SDH0->INTEN & SDH_INTEN_CDSRC_Msk) >> SDH_INTEN_CDSRC_Pos) == 0) ?
                     (!(SDH0->INTSTS & SDH_INTSTS_CDSTS_Msk)) : (SDH0->INTSTS & SDH_INTSTS_CDSTS_Msk);

        if (u32CDState)
        {
            printf("\n***** card remove !\n");
            SD0.IsCardInsert = FALSE;   // SDISR_CD_Card = 1 means card remove for GPIO mode
            memset(&SD0, 0, sizeof(SDH_INFO_T));
        }
        else
        {
            printf("***** card insert !\n");
            //SDH_Open(SDH0, CardDetect_From_GPIO);
            //SDH_Probe(SDH0);
        }

        SDH0->INTSTS = SDH_INTSTS_CDIF_Msk;
    }

    // CRC error interrupt
    if (isr & SDH_INTSTS_CRCIF_Msk)
    {
        if (!(isr & SDH_INTSTS_CRC16_Msk))
        {
            //printf("***** ISR sdioIntHandler(): CRC_16 error !\n");
            // handle CRC error
        }
        else if (!(isr & SDH_INTSTS_CRC7_Msk))
        {
            if (!SD0.R3Flag)
            {
                //printf("***** ISR sdioIntHandler(): CRC_7 error !\n");
                // handle CRC error
            }
        }

        SDH0->INTSTS = SDH_INTSTS_CRCIF_Msk;      // clear interrupt flag
    }

    if (isr & SDH_INTSTS_DITOIF_Msk)
    {
        printf("***** ISR: data in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_DITOIF_Msk;
    }

    // Response in timeout interrupt
    if (isr & SDH_INTSTS_RTOIF_Msk)
    {
        printf("***** ISR: response in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_RTOIF_Msk;
    }
}

void SD_Inits(void)
{
    /* Select multi-function pins */
    SET_SD0_DAT0_PE2();
    SET_SD0_DAT1_PE3();
    SET_SD0_DAT2_PE4();
    SET_SD0_DAT3_PE5();
    SET_SD0_CLK_PE6();
    SET_SD0_CMD_PE7();
    SET_SD0_nCD_PD13();

    /* Select IP clock source */
    CLK_SetModuleClock(SDH0_MODULE, CLK_SDHSEL_SDH0SEL_APLL1_DIV2, CLK_SDHDIV_SDH0DIV(5));

    /* Enable IP clock */
    CLK_EnableModuleClock(SDH0_MODULE);
}

void SYS_Init(void)
{
    /* Switch SCLK clock source to APLL0 and Enable APLL0 clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Enable APLL1 clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ, CLK_APLL1_SELECT);

    /* Enable I2S0 module clock */
    CLK_EnableModuleClock(I2S0_MODULE);

    /* Select source from HIRC(12MHz) */
    CLK_SetModuleClock(I2S0_MODULE, CLK_I2SSEL_I2S0SEL_HIRC, MODULE_NoMsk);

    /* Enable I2C3 module clock */
    CLK_EnableModuleClock(I2C3_MODULE);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable all GPIO clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(GPIOF_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

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
}

void I2C_Init(void)
{
    /* Open I2C and set clock to 100k */
    I2C_Open(I2C_PORT, 100000);
}

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetTxSGTable(uint8_t u8Id)
{
    DMA_DESC[u8Id].ctl = (DMA_DESC[u8Id].ctl & ~PDMA_DSCT_CTL_TXCNT_Msk) |
                         ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) |
                         PDMA_OP_SCATTER;

#if (NVT_DCACHE_ON == 1)
    SCB_CleanDCache_by_Addr((void *)&DMA_DESC[u8Id], sizeof(DMA_DESC[u8Id]));
    SCB_CleanDCache_by_Addr((void *)aPCMBuffer[u8Id], PCM_BUFFER_SIZE * sizeof(uint32_t));
#endif
}

/* Configure PDMA to Scatter Gather mode */
void PDMA_Init(void)
{
    DMA_DESC[0].ctl = ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_DESC[0].src = (uint32_t)&aPCMBuffer[0][0];
    DMA_DESC[0].dest = (uint32_t)&I2S_PORT->TXFIFO;
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1];

    DMA_DESC[1].ctl = ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_DESC[1].src = (uint32_t)&aPCMBuffer[1][0];
    DMA_DESC[1].dest = (uint32_t)&I2S_PORT->TXFIFO;
    DMA_DESC[1].offset = (uint32_t)&DMA_DESC[0];

    PDMA_Open(PDMA_PORT, (1 << I2S_TX_DMA_CH));
    PDMA_SetTransferMode(PDMA_PORT, I2S_TX_DMA_CH, PDMA_I2S0_TX, 1, (uint32_t)&DMA_DESC[0]);

    PDMA_EnableInt(PDMA_PORT, I2S_TX_DMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);
}

int32_t main(void)
{
    TCHAR sd_path[] = { '0', ':', 0 };    /* SD drive started from 0 */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init SD */
    SD_Inits();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Init I2C to access codec */
    I2C_Init();

    printf("+-----------------------------------------------------------------------+\n");
    printf("|                   MP3 Player Sample with audio codec                  |\n");
    printf("+-----------------------------------------------------------------------+\n");
    printf(" Please put MP3 files on SD card \n");

    /* Configure FATFS */
    SDH_Open_Disk(SDH0, CardDetect_From_GPIO);
    f_chdrive(sd_path);          /* Set default path */

    /* Lock protected registers */
    SYS_LockReg();

    while (1)
    {
        /* Play mp3 */
        if (SD0.IsCardInsert == TRUE)
            MP3Player();
    }
}
