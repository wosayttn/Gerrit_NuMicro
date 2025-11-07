/**************************************************************************//**
 * @file     main.c
 * @brief    A WAV file player demo using internal audio codec used to
 *           play WAV file which stored in internal flash.
 * @note
 * Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"
#include "config.h"
#include "spiflash_drv.h"

/*---------------------------------------------------------------------------*/
/* Global variables                                                          */
/*---------------------------------------------------------------------------*/
uint8_t aPCMBuffer[2][PCM_BUFFER_SIZE];
volatile uint8_t aPCMBuffer_Full[2] = {0, 0};
DESC_TABLE_T g_asDescTableCodec_TX[2], g_asDescTableCodec_RX[2], g_asDescTableFlash_TX[2], g_asDescTableFlash_RX[2];

/*---------------------------------------------------------------------------*/
/* Functions                                                                 */
/*---------------------------------------------------------------------------*/
void SYS_Init(void)
{
    SYS_UnlockReg();
    /*-----------------------------------------------------------------------*/
    /* Init System Clock                                                     */
    /*-----------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable peripheral clock */
    CLK_EnableModuleClock(PDMA_MODULE);

    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Enable UART0 Module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* UART0 module clock from EXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    /* Reset IP */
    SYS_ResetModule(UART0_RST);
    /* Configure UART0 and set UART0 Baud-rate */
    UART_Open(UART0, 115200);
    /*-------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                 */
    /*-------------------------------------------------------------------------*/
    /* Configure multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF() ;
}

void QSPI_Init(void)
{
    /* Enable Module clock */
    CLK_EnableModuleClock(QSPI0_MODULE);

    /* Select PCLK0 as the clock source of QSPI0 */
    CLK_SetModuleClock(QSPI0_MODULE, CLK_CLKSEL2_QSPI0SEL_PCLK0, MODULE_NoMsk);

    /* Reset IP */
    SYS_ResetModule(QSPI0_RST);
    /*------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                */
    /*------------------------------------------------------------------------*/
    /* Configure multi-function pins for QSPI */
    // GPA[0]  = MOSI0
    // GPA[1]  = MISO1
    // GPA[2]  = CLK
    // GPA[3]  = SS
    // GPA[4]  = MOSI1
    // GPA[5]  = MISO1

    /* Setup SPI0 multi-function pins */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_QSPI0_MOSI0 | SYS_GPA_MFPL_PA1MFP_QSPI0_MISO0 | SYS_GPA_MFPL_PA2MFP_QSPI0_CLK | SYS_GPA_MFPL_PA3MFP_QSPI0_SS;

    /* Enable SPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;
}

void SPII2S_Init(void)
{
    /* Enable I2S Module clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Select PCLK0 as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_HXT, MODULE_NoMsk);

    /* Reset IP */
    SYS_ResetModule(SPI0_RST);
    /*------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                */
    /*------------------------------------------------------------------------*/
    /* Configure multi-function pins for I2S */
    // GPA[0]  = DOUT
    // GPA[1]  = DIN
    // GPA[2]  = BCLK
    // GPA[3]  = LRCLK
    // GPB[0]  = MCLK

    /* Setup SPI0 multi-function pins */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk | SYS_GPD_MFPL_PD2MFP_Msk | SYS_GPD_MFPL_PD3MFP_Msk);
    SYS->GPD_MFPL |= SYS_GPD_MFPL_PD0MFP_SPI0_MOSI | SYS_GPD_MFPL_PD1MFP_SPI0_MISO | SYS_GPD_MFPL_PD2MFP_SPI0_CLK | SYS_GPD_MFPL_PD3MFP_SPI0_SS ;

    /* GPB[11] : SPI0_MCLK */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB11MFP_Msk)) | SYS_GPB_MFPH_PB11MFP_SPI0_I2SMCLK;

    /* Enable SPI0 clock pin (PD2) schmitt trigger */
    PD->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;
}

void I2C0_Init(void)
{
    /* Enable peripheral clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Open I2C1 and set clock to 100k */
    I2C_Open(I2C0, 100000);

    /* Get I2C3 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Set PA multi-function pins for I2C0 SDA and SCL */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC0MFP_I2C0_SDA | SYS_GPC_MFPL_PC1MFP_I2C0_SCL);
}

// Configure PDMA to Scatter Gather mode */
void PDMA_Codec_Init(void)
{
    /* Tx description */
    g_asDescTableCodec_TX[0].CTL = (((PCM_BUFFER_SIZE / 4) - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTableCodec_TX[0].SA = (uint32_t)&aPCMBuffer[0];
    g_asDescTableCodec_TX[0].DA = (uint32_t)&SPI0->TX;
    g_asDescTableCodec_TX[0].NEXT = (uint32_t)&g_asDescTableCodec_TX[1] - (PDMA->SCATBA);

    g_asDescTableCodec_TX[1].CTL = (((PCM_BUFFER_SIZE / 4) - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTableCodec_TX[1].SA = (uint32_t)&aPCMBuffer[1];
    g_asDescTableCodec_TX[1].DA = (uint32_t)&SPI0->TX;
    g_asDescTableCodec_TX[1].NEXT = (uint32_t)&g_asDescTableCodec_TX[0] - (PDMA->SCATBA);   //link to first description

    /* Open PDMA channel for SPI TX*/
    PDMA_Open(PDMA, 1 << PDMA_CODEC_TX_CH);

    /* Configure PDMA transfer mode */
    PDMA_SetTransferMode(PDMA, PDMA_CODEC_TX_CH, PDMA_SPI0_TX, 1, (uint32_t)&g_asDescTableCodec_TX[0]);

    /* Enable PDMA channel interrupt */
    PDMA_EnableInt(PDMA, PDMA_CODEC_TX_CH, 0);

}

void PDMA_Flash_Init(uint8_t *u8DataBuffer)
{
    g_au8PDMAFlashRXFlag = 0;
    uint8_t dummy = 0;

    PDMA_Open(PDMA, ((1 << PDMA_FLASH_TX_CH) | ((1 << PDMA_FLASH_RX_CH))));

    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA, PDMA_FLASH_TX_CH, PDMA_WIDTH_8, PCM_BUFFER_SIZE);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA, PDMA_FLASH_TX_CH, dummy, PDMA_DAR_FIX, (uint32_t)&QSPI0->TX, PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA, PDMA_FLASH_TX_CH, PDMA_QSPI0_TX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA, PDMA_FLASH_TX_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA->DSCT[PDMA_FLASH_TX_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    /*=======================================================================
      SPI master PDMA RX channel configuration:
      -----------------------------------------------------------------------
      =======================================================================*/
    /* Set transfer width (8 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA, PDMA_FLASH_RX_CH, PDMA_WIDTH_8, PCM_BUFFER_SIZE);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA, PDMA_FLASH_RX_CH, (uint32_t)&QSPI0->RX, PDMA_SAR_FIX, (uint32_t)u8DataBuffer, PDMA_DAR_INC);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA, PDMA_FLASH_RX_CH, PDMA_QSPI0_RX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA, PDMA_FLASH_RX_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA->DSCT[PDMA_FLASH_RX_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    /* Enable PDMA channel 1&2 interrupt */
    PDMA_EnableInt(PDMA, PDMA_FLASH_RX_CH, 0);
}

/*---------------------------------------------------------------------------------*/
/*  Main Function                                                                  */
/*---------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART0_Init();

    /* Init QSPI */
    QSPI_Init();

    /* Init I2S */
    SPII2S_Init();

    /* Init I2C */
    I2C0_Init();

    PDMA_Codec_Init();

    NVIC_EnableIRQ(PDMA_IRQn);

    printf("+-----------------------------------------------------------+\n");
    printf("|      Play WAV file from internal Flash Sample Code        |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("  Please store WAV file first in the internal Flash.\n");
    printf("  WAV file storage location cannot exceed flash size\n");
    printf("  NOTE: Need head-phone. \n\n");

    /* Play wav from internal flash at WAVFile_address */
    SpiFlash_Open();
    SpiFlash_WriteMusic();

    ReadFlashPlay(0);

    while (1);
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
