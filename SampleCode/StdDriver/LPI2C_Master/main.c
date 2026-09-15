/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * @brief
 *           Show a Master how to access Slave.
 *           This sample code needs to work with LPI2C_Slave.
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
volatile uint8_t g_au8MstTxData[3];
volatile uint8_t g_u8MstRxData;
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8MstEndFlag = 0;
volatile uint8_t g_u8MstTxAbortFlag = 0;
volatile uint8_t g_u8MstRxAbortFlag = 0;
volatile uint8_t g_u8MstReStartFlag = 0;
volatile uint8_t g_u8TimeoutFlag = 0;

typedef void (*LPI2C_FUNC)(uint32_t u32Status);

static volatile LPI2C_FUNC s_LPI2C0HandlerFn = NULL;

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
        g_u8TimeoutFlag = 1;
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
    uint32_t u32TimeOutCnt;

    if (u32Status == 0x08)                      /* START has been transmitted and prepare SLA+W */
    {
        LPI2C_SET_DATA(LPI2C0, (g_u8DeviceAddr << 1)); /* Write SLA+W to Register LPI2CDAT */
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        LPI2C_SET_DATA(LPI2C0, g_au8MstTxData[g_u8MstDataLen++]);
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        LPI2C_STOP(LPI2C0);
        LPI2C_START(LPI2C0);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8MstDataLen != 2)
        {
            LPI2C_SET_DATA(LPI2C0, g_au8MstTxData[g_u8MstDataLen++]);
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        else
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STA_SI);
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
        g_u8MstRxData = LPI2C_GET_DATA(LPI2C0);
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STO_SI);
        g_u8MstEndFlag = 1;
    }
    else
    {
        /* Error condition process */
        printf("[MasterRx] Status [0x%x] Unexpected abort!! Press any key to re-start\n", u32Status);
        if(u32Status == 0x38)                 /* Master arbitration lost, stop LPI2C and clear SI */
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        else if(u32Status == 0x30)            /* Master transmit data NACK, stop LPI2C and clear SI */
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        else if(u32Status == 0x48)            /* Master receive address NACK, stop LPI2C and clear SI */
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        else if(u32Status == 0x00)            /* Master bus error, stop LPI2C and clear SI */
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        else
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        /*Setting MasterRx abort flag for re-start mechanism*/
        g_u8MstRxAbortFlag = 1;
        getchar();
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        u32TimeOutCnt = SystemCoreClock;
        while(LPI2C0->CTL0 & LPI2C_CTL0_SI_Msk)
            if(--u32TimeOutCnt == 0) break;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C Tx Callback Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void LPI2C_MasterTx(uint32_t u32Status)
{
    uint32_t u32TimeOutCnt;

    if (u32Status == 0x08)                      /* START has been transmitted */
    {
        LPI2C_SET_DATA(LPI2C0, g_u8DeviceAddr << 1);  /* Write SLA+W to Register LPI2CDAT */
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x18)                 /* SLA+W has been transmitted and ACK has been received */
    {
        LPI2C_SET_DATA(LPI2C0, g_au8MstTxData[g_u8MstDataLen++]);
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    else if (u32Status == 0x20)                 /* SLA+W has been transmitted and NACK has been received */
    {
        LPI2C_STOP(LPI2C0);
        LPI2C_START(LPI2C0);
    }
    else if (u32Status == 0x28)                 /* DATA has been transmitted and ACK has been received */
    {
        if (g_u8MstDataLen != 3)
        {
            LPI2C_SET_DATA(LPI2C0, g_au8MstTxData[g_u8MstDataLen++]);
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        else
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STO_SI);
            g_u8MstEndFlag = 1;
        }
    }
    else
    {
        /* Error condition process */
        printf("[MasterTx] Status [0x%x] Unexpected abort!! Press any key to re-start\n", u32Status);

        if(u32Status == 0x38)                   /* Master arbitration lost, stop LPI2C and clear SI */
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        else if(u32Status == 0x00)              /* Master bus error, stop LPI2C and clear SI */
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        else if(u32Status == 0x30)              /* Master transmit data NACK, stop LPI2C and clear SI */
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        else if(u32Status == 0x48)              /* Master receive address NACK, stop LPI2C and clear SI */
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        else if(u32Status == 0x10)              /* Master repeat start, clear SI */
        {
            LPI2C_SET_DATA(LPI2C0, (uint32_t)((g_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        else
        {
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        }
        /*Setting MasterTRx abort flag for re-start mechanism*/
        g_u8MstTxAbortFlag = 1;
        getchar();
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
        u32TimeOutCnt = SystemCoreClock;
        while(LPI2C0->CTL0 & LPI2C_CTL0_SI_Msk)
            if(--u32TimeOutCnt == 0) break;
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

int32_t Read_Write_SLAVE(uint8_t slvaddr)
{
    uint32_t i;

    do
    {
        /* Enable LPI2C timeout */
        LPI2C_EnableTimeout(LPI2C0, 0);
        g_u8MstReStartFlag = 0;
        g_u8DeviceAddr = slvaddr;

        g_u8TimeoutFlag = 0;

        for (i = 0; i < 0x100; i++)
        {
            g_au8MstTxData[0] = (uint8_t)((i & 0xFF00) >> 8);
            g_au8MstTxData[1] = (uint8_t)(i & 0x00FF);
            g_au8MstTxData[2] = (uint8_t)(g_au8MstTxData[1] + 3);

            g_u8MstDataLen = 0;
            g_u8MstEndFlag = 0;

            /* LPI2C function to write data to slave */
            s_LPI2C0HandlerFn = (LPI2C_FUNC)LPI2C_MasterTx;

            /* LPI2C as master sends START signal */
            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STA);

            /* Wait LPI2C Tx Finish or Unexpected Abort*/
            do
            {
                if(g_u8TimeoutFlag)
                {
                    printf(" MasterTx time out!! Press any to reset IP\n");
                    getchar();
                    LPSCC->IPRST0 |= LPSCC_IPRST0_LPI2C0RST_Msk;
                    LPSCC->IPRST0 = 0;
                    LPI2C0_Init();
                    /* Set MasterTx abort flag*/
                    g_u8MstTxAbortFlag = 1;
                }
            }
            while(g_u8MstEndFlag == 0 && g_u8MstTxAbortFlag == 0);

            g_u8MstEndFlag = 0;

            if(g_u8MstTxAbortFlag)
            {
                /* Clear MasterTx abort flag*/
                g_u8MstTxAbortFlag = 0;
                /* Set Master re-start flag*/
                g_u8MstReStartFlag = 1;
                break;
            }

            /* LPI2C function to read data from slave */
            s_LPI2C0HandlerFn = (LPI2C_FUNC)LPI2C_MasterRx;

            g_u8MstDataLen = 0;
            g_u8DeviceAddr = slvaddr;

            LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_STA);

            /* Wait I2C Rx Finish or Unexpected Abort*/
            do
            {
                if(g_u8TimeoutFlag)
                {
                    /* When I2C timeout, reset IP*/
                    printf(" MasterRx time out!! Press any to reset IP\n");
                    getchar();
                    LPSCC->IPRST0 |= LPSCC_IPRST0_LPI2C0RST_Msk;
                    LPSCC->IPRST0 = 0;
                    LPI2C0_Init();
                    /* Set MasterRx abort flag*/
                    g_u8MstRxAbortFlag = 1;
                }
            }
            while(g_u8MstEndFlag == 0 && g_u8MstRxAbortFlag == 0);

            g_u8MstEndFlag = 0;

            if(g_u8MstRxAbortFlag)
            {
                /* Clear MasterRx abort flag*/
                g_u8MstRxAbortFlag = 0;
                /* Set Master re-start flag*/
                g_u8MstReStartFlag = 1;
                break;
            }

            /* Compare data */
            if (g_u8MstRxData != g_au8MstTxData[2])
            {
                /* Disable LPI2C timeout */
                LPI2C_DisableTimeout(LPI2C0);
                printf("LPI2C Byte Write/Read Failed, Data 0x%x\n", g_u8MstRxData);
                return -1;
            }
        }
    }
    while(g_u8MstReStartFlag);   /*If unexpected abort happens, re-start the transmition*/

    /* Disable LPI2C timeout */
    LPI2C_DisableTimeout(LPI2C0);
    printf("Master Access Slave (0x%X) Test OK\n", slvaddr);
    return 0;
}

int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /*
        This sample code sets LPI2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|     LPI2C Driver Sample Code(Master) for access Slave |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init LPI2C0 */
    LPI2C0_Init();

    /* Access Slave with no address mask */
    printf("\n");
    printf(" == No Mask Address ==\n");
    Read_Write_SLAVE(0x15);
    Read_Write_SLAVE(0x35);
    Read_Write_SLAVE(0x55);
    Read_Write_SLAVE(0x75);
    printf("SLAVE Address test OK.\n");

    /* Access Slave with address mask */
    printf("\n");
    printf(" == Mask Address ==\n");
    Read_Write_SLAVE(0x15 & ~0x01);
    Read_Write_SLAVE(0x35 & ~0x04);
    Read_Write_SLAVE(0x55 & ~0x01);
    Read_Write_SLAVE(0x75 & ~0x04);
    printf("SLAVE Address Mask test OK.\n");

    while(1);
}
