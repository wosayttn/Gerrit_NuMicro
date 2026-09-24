/*************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Show how to wake up MCU from Power-down mode through LPI2C interface.
 *           This sample code needs to work with LPI2C_Master.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/* LPI2C can support NPD0 ~ NDP4 power-down mode */
#define TEST_POWER_DOWN_MODE    CLK_PMUCTL_PDMSEL_NPD4

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t slave_buff_addr;
uint8_t g_au8SlvData[256];
uint8_t g_au8SlvRxData[3];
uint8_t g_u8SlvPWRDNWK, g_u8SlvLPI2CWK;
uint8_t g_u8DeviceAddr;
uint8_t g_u8SlvDataLen;

typedef void (*LPI2C_FUNC)(uint32_t u32Status);

static LPI2C_FUNC s_LPI2C0HandlerFn = NULL;


void LPI2C0_IRQHandler(void)
{
    uint32_t u32Status;

    /* Check LPI2C Wake-up interrupt flag set or not */
    if(LPI2C_GET_WAKEUP_FLAG(LPI2C0))
    {
        /* Waitinn for LPI2C response ACK finish */
        while(!(LPI2C0->WKSTS & LPI2C_WKSTS_WKAKDONE_Msk));

        /* Clear Wakeup done flag, LPI2C will release bus */
        LPI2C0->WKSTS = LPI2C_WKSTS_WKAKDONE_Msk;

        /* Clear LPI2C Wake-up interrupt flag */
        LPI2C_CLEAR_WAKEUP_FLAG(LPI2C0);
        g_u8SlvLPI2CWK = 1;

        return;
    }

    u32Status = LPI2C_GET_STATUS(LPI2C0);

    if(LPI2C_GET_TIMEOUT_FLAG(LPI2C0))
    {
        /* Clear LPI2C0 Timeout Flag */
        LPI2C_ClearTimeoutFlag(LPI2C0);
    }
    else
    {
        if(s_LPI2C0HandlerFn != NULL)
            s_LPI2C0HandlerFn(u32Status);
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Power Wake-up IRQ Handler                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void PWRWU_IRQHandler(void)
{
    /* Check system power down mode wake-up interrupt flag */
    if(((CLK->PWRCTL) & CLK_PWRCTL_PDWKIF_Msk) != 0)
    {
        /* Clear system power down wake-up interrupt flag */
        CLK->PWRCTL |= CLK_PWRCTL_PDWKIF_Msk;
        g_u8SlvPWRDNWK = 1;

    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(DEBUG_PORT);

    /* Set Power-down mode */
    CLK_SetPowerDownMode(TEST_POWER_DOWN_MODE);

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPI2C Slave Transmit/Receive Callback Function                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void LPI2C_SlaveTRx(uint32_t u32Status)
{
    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlvDataLen = 0;
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        g_au8SlvRxData[g_u8SlvDataLen] = (unsigned char)LPI2C_GET_DATA(LPI2C0);
        g_u8SlvDataLen++;

        if(g_u8SlvDataLen == 2)
        {
            slave_buff_addr = (g_au8SlvRxData[0] << 8) + g_au8SlvRxData[1];
        }
        if(g_u8SlvDataLen == 3)
        {
            g_au8SlvData[slave_buff_addr] = g_au8SlvRxData[2];
            g_u8SlvDataLen = 0;
        }
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
        LPI2C_SET_DATA(LPI2C0, g_au8SlvData[slave_buff_addr]);
        slave_buff_addr++;
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if(u32Status == 0xC0)                 /* Data byte or last data in LPI2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8SlvDataLen = 0;
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8SlvDataLen = 0;
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);
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
    /* Open LPI2C module and set bus clock */
    LPI2C_Open(LPI2C0, 100000);

    /* Get LPI2C0 Bus Clock */
    printf("LPI2C clock %d Hz\n", LPI2C_GetBusClockFreq(LPI2C0));

    /* Set LPI2C 4 Slave Addresses */
    LPI2C_SetSlaveAddr(LPI2C0, 0, 0x15, 0);   /* Slave Address : 0x15 */
    LPI2C_SetSlaveAddr(LPI2C0, 1, 0x35, 0);   /* Slave Address : 0x35 */
    LPI2C_SetSlaveAddr(LPI2C0, 2, 0x55, 0);   /* Slave Address : 0x55 */
    LPI2C_SetSlaveAddr(LPI2C0, 3, 0x75, 0);   /* Slave Address : 0x75 */

    /* Set LPI2C 4 Slave Addresses Mask */
    LPI2C_SetSlaveAddrMask(LPI2C0, 0, 0x01);
    LPI2C_SetSlaveAddrMask(LPI2C0, 1, 0x04);
    LPI2C_SetSlaveAddrMask(LPI2C0, 2, 0x01);
    LPI2C_SetSlaveAddrMask(LPI2C0, 3, 0x04);

    /* Enable LPI2C interrupt */
    LPI2C_EnableInt(LPI2C0);
    NVIC_EnableIRQ(LPI2C0_IRQn);
}

void LPI2C0_Close(void)
{
    /* Disable LPI2C0 interrupt and clear corresponding NVIC bit */
    LPI2C_DisableInt(LPI2C0);
    NVIC_DisableIRQ(LPI2C0_IRQn);

    /* Disable LPI2C0 and close LPI2C0 clock */
    LPI2C_Close(LPI2C0);
    CLK_DisableModuleClock(LPI2C0_MODULE);

}


int32_t main(void)
{
    uint32_t i;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code is LPI2C SLAVE mode and it simulates EEPROM function
    */
    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("| LPI2C Driver Sample Code (Slave) for wake-up & access Slave test     |\n");
    printf("|                                                                      |\n");
    printf("| LPI2C Master (LPI2C0) <---> LPI2C Slave(LPI2C0)                      |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("Configure LPI2C0 as a slave.\n");
    printf("The I/O connection for LPI2C0:\n");

    /* Init LPI2C0 */
    LPI2C0_Init();

    for(i = 0; i < 0x100; i++)
    {
        g_au8SlvData[i] = 0;
    }

    /* LPI2C function to Transmit/Receive data as slave */
    s_LPI2C0HandlerFn = LPI2C_SlaveTRx;

    /* Set LPI2C0 enter Not Address SLAVE mode */
    LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI_AA);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable power wake-up interrupt */
    CLK->PWRCTL |= CLK_PWRCTL_PDWKIEN_Msk;
    NVIC_EnableIRQ(PWRWU_IRQn);
    g_u8SlvPWRDNWK = 0;

    /* Enable LPI2C wake-up */
    LPI2C_EnableWakeup(LPI2C0);
    g_u8SlvLPI2CWK = 0;

    printf("\n");
    printf("\n");
    printf("CHIP enter power down status.\n");

    if(((LPI2C0->CTL0)&LPI2C_CTL0_SI_Msk) != 0)
    {
        LPI2C_SET_CONTROL_REG(LPI2C0, LPI2C_CTL_SI);
    }
    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Waiting for system wake-up and LPI2C wake-up finish */
    while((g_u8SlvPWRDNWK & g_u8SlvLPI2CWK) == 0);

    /* Wake-up Interrupt Message */
    printf("Power-down Wake-up INT 0x%x\n", (unsigned int)((CLK->PWRCTL) & CLK_PWRCTL_PDWKIF_Msk));
    printf("LPI2C0 WAKE INT 0x%x\n", LPI2C0->WKSTS);

    /* Disable power wake-up interrupt */
    CLK->PWRCTL &= ~CLK_PWRCTL_PDWKIEN_Msk;
    NVIC_DisableIRQ(PWRWU_IRQn);

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n");
    printf("Slave wake-up from power down status.\n");

    printf("\n");
    printf("Slave Waiting for receiving data.\n");

    while(1);
}
/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

