/******************************************************************************
* @file     main.c
* @brief
*           Show how to wake up MCU from Power-down mode through I2C interface.
*           This sample code needs to work with I2C_Master.
*
* SPDX-License-Identifier: Apache-2.0
* @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_POWER_DOWN_MODE    CLK_PMUCTL_PDMSEL_NPD1

volatile uint32_t g_u32SlvBufAddr;
volatile uint8_t g_au8SlvData[256];
volatile uint8_t g_au8SlvRxData[3];
volatile uint32_t g_u32SlvPWRDNWK, g_u32SlvI2CWK;
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_u8SlvDataLen;

typedef void (*I2C_FUNC)(uint32_t u32Status);

volatile static I2C_FUNC s_I2C0HandlerFn = NULL;

void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    /* Check I2C Wake-up interrupt flag set or not */
    if (I2C_GET_WAKEUP_FLAG(I2C0))
    {
        // Record I2C0->WKSTS when Wake Up Event occurred.
        g_u32SlvI2CWK = I2C0->WKSTS;

        /* Waiting for I2C response ACK finish */
        while (!(I2C0->WKSTS & I2C_WKSTS_WKAKDONE_Msk));

        /* Clear Wakeup done flag, I2C will release bus */
        I2C0->WKSTS = I2C_WKSTS_WKAKDONE_Msk;
        /* Clear I2C Wake-up interrupt flag */
        I2C_CLEAR_WAKEUP_FLAG(I2C0);
        return;
    }

    u32Status = I2C_GET_STATUS(I2C0);

    if (I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    }
    else
    {
        if (s_I2C0HandlerFn != NULL)
        {
            s_I2C0HandlerFn(u32Status);
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Power Wake-up IRQ Handler                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void PWRWU_IRQHandler(void)
{
    uint32_t u32Status;

    /* check power down wakeup flag */
    if (((CLK->PWRCTL) & CLK_PWRCTL_PDWKIF_Msk) != 0)
    {
        // // Record CLK->PWRCTL when Wake Up Event occurred.
        g_u32SlvPWRDNWK = CLK->PWRCTL;
        CLK->PWRCTL |= CLK_PWRCTL_PDWKIF_Msk;
        // CPU read interrupt flag register to wait write(clear) instruction completement.
        u32Status = CLK->PWRCTL;
        (void)u32Status;
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
/*  I2C Slave Transmit/Receive Callback Function                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
    uint8_t u8SlvDataLen = g_u8SlvDataLen;
    uint32_t u32SlvBufAddr = g_u32SlvBufAddr, u32Temp = 0;

    if (u32Status == 0x60)                      /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        g_au8SlvRxData[u8SlvDataLen] = (unsigned char)I2C_GET_DATA(I2C0);
        g_u8SlvDataLen++;

        if (g_u8SlvDataLen == 2)
        {
            u32Temp = (g_au8SlvRxData[0] << 8);
            u32Temp += g_au8SlvRxData[1];
            g_u32SlvBufAddr = u32Temp;
        }

        if (g_u8SlvDataLen == 3)
        {
            u32Temp = g_au8SlvRxData[2];
            g_au8SlvData[u32SlvBufAddr] = u32Temp;
            g_u8SlvDataLen = 0;
        }

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA8)                 /* Own SLA+R has been receive; ACK has been return */
    {
        I2C_SET_DATA(I2C0, g_au8SlvData[g_u32SlvBufAddr]);
        g_u32SlvBufAddr++;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xF8)     /*I2C wave keeps going*/
    {
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

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
    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);
    /* Set core clock to 144MHz */
    CLK_SetCoreClock(FREQ_144MHZ);
    /* Enable UART module clock */
    SetDebugUartCLK();
    /*------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                */
    /*------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Enable I2C0 module clock */
    CLK_EnableModuleClock(I2C0_MODULE);
    /* Set multi-function pins for I2C0 SDA and SCL */
    SET_I2C0_SDA_PA4();
    SET_I2C0_SCL_PA5();
    /* I2C pins enable schmitt trigger */
    CLK_EnableModuleClock(GPA_MODULE);
    GPIO_ENABLE_SCHMITT_TRIGGER(PA, (BIT4 | BIT5));
    /* Lock protected registers */
    SYS_LockReg();
    return 0;
}

void I2C0_Init(void)
{
    /* Open I2C0 and set clock to 100k */
    I2C_Open(I2C0, 100000);
    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));
    /* Set I2C 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C0, 0, 0x15, I2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C0, 1, 0x35, I2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C0, 2, 0x55, I2C_GCMODE_DISABLE);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C0, 3, 0x75, I2C_GCMODE_DISABLE);   /* Slave Address : 0x75 */
    /* Set I2C 4 Slave Addresses Mask */
    I2C_SetSlaveAddrMask(I2C0, 0, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 1, 0x04);
    I2C_SetSlaveAddrMask(I2C0, 2, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 3, 0x04);
    /* Enable I2C interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t i, u32TimeOutCnt;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    /*
        This sample code is I2C SLAVE mode and it simulates EEPROM function
    */
    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code (Slave) for wake-up & access Slave test       |\n");
    printf("|                                                                      |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)                              |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("Configure I2C0 as a slave.\n");
    printf("The I/O connection for I2C0:\n");
    /* Init I2C0 */
    I2C0_Init();

    for (i = 0; i < 0x100; i++)
    {
        g_au8SlvData[i] = 0;
    }

    /* I2C function to Transmit/Receive data as slave */
    s_I2C0HandlerFn = I2C_SlaveTRx;
    /* Set I2C0 enter Not Address SLAVE mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable power wake-up interrupt */
    CLK->PWRCTL |= CLK_PWRCTL_PDWKIEN_Msk;
    NVIC_EnableIRQ(PWRWU_IRQn);
    g_u32SlvPWRDNWK = 0;
    /* Enable I2C wake-up */
    I2C_EnableWakeup(I2C0);
    g_u32SlvI2CWK = 0;
    printf("\n");
    printf("\n");
    printf("Press any key to enter power down status.\n");
    getchar();

    if (((I2C0->CTL0)&I2C_CTL0_SI_Msk) != 0)
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }

    /* Enter to Power-down mode */
    PowerDownFunction();
    u32TimeOutCnt = I2C_TIMEOUT;

    /* Waiting for both system wake-up and I2C wake-up finish */
    while ((g_u32SlvPWRDNWK == 0) || (g_u32SlvI2CWK == 0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for I2C wake-up time-out!\n");

            while (1);
        }
    }

    /* Wake-up Interrupt Message */
    printf("Power-down Wake-up INT 0x%x\n", (uint32_t)(g_u32SlvPWRDNWK & CLK_PWRCTL_PDWKIF_Msk));
    printf("I2C0 WAKE INT 0x%x\n", (uint32_t)(g_u32SlvI2CWK & I2C_WKSTS_WKIF_Msk));
    /* Disable power wake-up interrupt */
    CLK->PWRCTL &= ~CLK_PWRCTL_PDWKIEN_Msk;
    NVIC_DisableIRQ(PWRWU_IRQn);
    /* Lock protected registers */
    SYS_LockReg();
    printf("\n");
    printf("Slave wake-up from power down status.\n");
    printf("\n");
    printf("Slave Waiting for receiving data.\n");

    while (1);
}
