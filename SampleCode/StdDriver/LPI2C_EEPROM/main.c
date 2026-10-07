/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Read/write EEPROM via LPI2C interface
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8TxData[3];
volatile uint8_t g_u8RxData;
volatile uint8_t g_u8DataLen;
volatile uint8_t g_u8EndFlag = 0;

typedef void (*LPI2C_FUNC)(uint32_t u32Status);

volatile static LPI2C_FUNC s_LPI2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C0 IRQ Handler                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void LPI2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = LPI2C_GET_STATUS(LPI2C0);

    if (LPI2C_GET_TIMEOUT_FLAG(LPI2C0))
    {
        /* Clear LPI2C0 Timeout Flag */
        LPI2C_ClearTimeoutFlag(LPI2C0);
    }
    else
    {
        if (s_LPI2C0HandlerFn != NULL)
            s_LPI2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C Rx Callback Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void LPI2C_MasterRx(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted and prepare SLA+W */
    {
        LPI2C_SET_DATA(LPI2C0, (g_u8DeviceAddr << 1)); /* Write SLA+W to Register LPI2CDAT */
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        LPI2C_SET_DATA(LPI2C0, g_au8TxData[g_u8DataLen++]);
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STO | LPI2C_CTL_SI);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8DataLen < 2)
        {
            LPI2C_SET_DATA(LPI2C0, g_au8TxData[g_u8DataLen++]);
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        else
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STA | LPI2C_CTL_SI);
        }
    }
    else if (u32Status == 0x10)                 /* Repeat START has been transmitted and prepare SLA+R */
    {
        LPI2C_SET_DATA(LPI2C0, (g_u8DeviceAddr << 1) | 0x01);  /* Write SLA+R to Register LPI2CDAT */
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x40)                 /* SLA+R has been transmitted and ACK has been received */
    {
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x58)                 /* DATA has been received and NACK has been returned */
    {
        g_u8RxData = LPI2C_GET_DATA(LPI2C0);
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STO | LPI2C_CTL_SI);
        g_u8EndFlag = 1;
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C Tx Callback Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void LPI2C_MasterTx(uint32_t u32Status)
{
    if (u32Status == 0x08)                      /* START has been transmitted */
    {
        LPI2C_SET_DATA(LPI2C0, g_u8DeviceAddr << 1);  /* Write SLA+W to Register LPI2CDAT */
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        LPI2C_SET_DATA(LPI2C0, g_au8TxData[g_u8DataLen++]);
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STA | LPI2C_CTL_STO | LPI2C_CTL_SI);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8DataLen != 3)
        {
            LPI2C_SET_DATA(LPI2C0, g_au8TxData[g_u8DataLen++]);
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        else
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STO | LPI2C_CTL_SI);
            g_u8EndFlag = 1;
        }
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set HCLK1 to HIRC */
    CLK_SetModuleClock(HCLK1_MODULE, CLK_CLKSEL0_HCLK1SEL_HIRC, LPSCC_CLKDIV0_HCLK1(1));

    /* Enable HCLK1 clock */
    CLK_EnableModuleClock(HCLK1_MODULE);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable LPI2C0 clock */
    CLK_EnableModuleClock(LPI2C0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set LPI2C0 multi-function pins */
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(SYS_GPA_MFP1_PA4MFP_Msk | SYS_GPA_MFP1_PA5MFP_Msk)) |
                    (SYS_GPA_MFP1_PA4MFP_LPI2C0_SDA | SYS_GPA_MFP1_PA5MFP_LPI2C0_SCL);

    /* LPI2C pin enable schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

void LPI2C0_Init(void)
{
    /* Open LPI2C0 and set clock to 100k */
    LPI2C_Open(LPI2C0, 100000);

    /* Get LPI2C0 Bus Clock */
    printf("LPI2C clock %d Hz\n", LPI2C_GetBusClockFreq(LPI2C0));

    LPI2C_EnableInt(LPI2C0);
    NVIC_EnableIRQ(LPI2C0_IRQn);
}

int32_t main (void)
{
    uint32_t i;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /*
        This sample code sets LPI2C bus clock to 100kHz. Then, accesses EEPROM 24LC64 with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|        LPI2C Driver Sample Code with EEPROM 24LC64    |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init LPI2C0 to access EEPROM */
    LPI2C0_Init();

    g_u8DeviceAddr = 0x50;

    for (i = 0; i < 2; i++)
    {
        g_au8TxData[0] = (uint8_t)((i & 0xFF00) >> 8);
        g_au8TxData[1] = (uint8_t)(i & 0x00FF);
        g_au8TxData[2] = (uint8_t)(g_au8TxData[1] + 3);

        g_u8DataLen = 0;
        g_u8EndFlag = 0;

        /* LPI2C function to write data to slave */
        s_LPI2C0HandlerFn = (LPI2C_FUNC)LPI2C_MasterTx;

        /* LPI2C as master sends START signal */
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STA);

        /* Wait LPI2C Tx Finish */
        while (g_u8EndFlag == 0);
        g_u8EndFlag = 0;

        /* Make sure LPI2C0 STOP already */
        while(LPI2C0->CTL0 & LPI2C_CTL0_STO_Msk);

        /* Wait write operation complete */
        CLK_SysTickDelay(10000);

        /* LPI2C function to read data from slave */
        s_LPI2C0HandlerFn = (LPI2C_FUNC)LPI2C_MasterRx;

        g_u8DataLen = 0;
        g_u8DeviceAddr = 0x50;

        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STA);

        /* Wait LPI2C Rx Finish */
        while (g_u8EndFlag == 0);

        /* Make sure LPI2C0 STOP already */
        while(LPI2C0->CTL0 & LPI2C_CTL0_STO_Msk);

        /* Compare data */
        if (g_u8RxData != g_au8TxData[2])
        {
            printf("LPI2C Byte Write/Read Failed, Data 0x%x\n", g_u8RxData);
            return -1;
        }
    }
    printf("LPI2C Access EEPROM Test OK\n");

    while(1);
}
