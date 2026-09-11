/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Show how to set I2C in Slave mode and receive the data from Master.
 *           This sample code needs to work with I2C_Master.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8SlvDataLen;
uint32_t slave_buff_addr;
uint8_t g_au8SlvData[256];
uint8_t g_au8SlvRxData[3];
volatile uint8_t g_u8SlvTRxAbortFlag = 0;
volatile uint8_t g_u8TimeoutFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

volatile static I2C_FUNC s_I2C0HandlerFn = NULL;


void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if (I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
        g_u8TimeoutFlag = 1;
    }
    else
    {
        if (s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C TRx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
    uint8_t u8Data;

    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        u8Data = (unsigned char) I2C_GET_DATA(I2C0);
        if(g_u8SlvDataLen < 2)
        {
            g_au8SlvRxData[g_u8SlvDataLen++] = u8Data;
            slave_buff_addr = (uint32_t)(g_au8SlvRxData[0] << 8) + g_au8SlvRxData[1];
        }
        else
        {
            g_au8SlvData[slave_buff_addr++] = u8Data;
            if(slave_buff_addr == 256)
            {
                slave_buff_addr = 0;
            }
        }

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
        I2C_SET_DATA(I2C0, g_au8SlvData[slave_buff_addr]);
        slave_buff_addr++;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xB8)                  /* Data byte in I2CDAT has been transmitted ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8SlvData[slave_buff_addr++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else
    {
        printf("[SlaveTRx] Status [0x%x] Unexpected abort!!\n", u32Status);
        if(u32Status == 0x68)               /* Slave receive arbitration lost, clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        }
        else if(u32Status == 0xB0)          /* Address transmit arbitration lost, clear SI  */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        }
        else                                /* Slave bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        g_u8SlvTRxAbortFlag = 1;
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

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable I2C0 clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set I2C0 multi-function pins */
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(SYS_GPA_MFP1_PA4MFP_Msk | SYS_GPA_MFP1_PA5MFP_Msk)) |
                    (SYS_GPA_MFP1_PA4MFP_I2C0_SDA | SYS_GPA_MFP1_PA5MFP_I2C0_SCL);

    /* I2C pin enable schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

void I2C0_Init(void)
{
    /* Open I2C0 and set clock to 100k */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Set I2C0 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C0, 0, 0x15, I2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C0, 1, 0x35, I2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C0, 2, 0x55, I2C_GCMODE_DISABLE);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C0, 3, 0x75, I2C_GCMODE_DISABLE);   /* Slave Address : 0x75 */

    I2C_SetSlaveAddrMask(I2C0, 0, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 1, 0x04);
    I2C_SetSlaveAddrMask(I2C0, 2, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 3, 0x04);

    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}


int32_t main (void)
{
    uint32_t i, u32TimeOutCnt;


    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("|               I2C Driver Sample Code(Slave)           |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init I2C0 */
    I2C0_Init();

    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

    for (i = 0; i < 0x100; i++)
    {
        g_au8SlvData[i] = 0;
    }

    /* I2C function to Slave receive/transmit data */
    s_I2C0HandlerFn = I2C_SlaveTRx;

    printf("\n");
    printf("I2C Slave Mode is Running.\n");

    g_u8TimeoutFlag = 0;

    while(1)
    {
        /* Handle Slave timeout condition */
        if(g_u8TimeoutFlag)
        {
            printf(" SlaveTRx time out, press any key to reset IP\n");
            getchar();
            SYS->IPRST1 |= SYS_IPRST1_I2C0RST_Msk;
            SYS->IPRST1 = 0;
            I2C0_Init();
            g_u8TimeoutFlag = 0;
            g_u8SlvTRxAbortFlag = 1;
        }
        /* When I2C abort, clear SI to enter non-addressed SLV mode*/
        if(g_u8SlvTRxAbortFlag)
        {
            g_u8SlvTRxAbortFlag = 0;

            u32TimeOutCnt = SystemCoreClock;
            while(I2C0->CTL0 & I2C_CTL0_SI_Msk)
                if(--u32TimeOutCnt == 0) break;

            printf("I2C Slave re-start. status[0x%x]\n", I2C0->STATUS0);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
        }
    }
}
