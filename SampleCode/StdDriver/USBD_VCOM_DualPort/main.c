/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 23/02/24 4:40p $
 * @brief    Demonstrate how to implement a USB dual virtual COM port device.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "vcom_serial.h"

#define CRYSTAL_LESS        1
#define TRIM_INIT           (SYS_BASE+0x10C)

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING g_LineCoding0 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
STR_VCOM_LINE_CODING g_LineCoding1 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t g_u16CtrlSignal0 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
uint16_t g_u16CtrlSignal1 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* RX buffer size */

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* UART0 */
static volatile uint8_t s_au8ComRbuf0[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes0 = 0;
volatile uint16_t g_u16ComRhead0 = 0;
volatile uint16_t g_u16ComRtail0 = 0;

static volatile uint8_t s_au8ComTbuf0[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes0 = 0;
volatile uint16_t g_u16ComThead0 = 0;
volatile uint16_t g_u16ComTtail0 = 0;

static uint8_t s_au8RxBuf0[64] = {0};
uint8_t *g_pu8RxBuf0 = 0;
uint32_t g_u32RxSize0 = 0;
uint32_t g_u32TxSize0 = 0;

volatile int8_t g_i8BulkOutReady0 = 0;

/* UART1 */
static volatile uint8_t s_au8ComRbuf1[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes1 = 0;
volatile uint16_t g_u16ComRhead1 = 0;
volatile uint16_t g_u16ComRtail1 = 0;

static volatile uint8_t s_au8ComTbuf1[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes1 = 0;
volatile uint16_t g_u16ComThead1 = 0;
volatile uint16_t g_u16ComTtail1 = 0;

static uint8_t s_au8RxBuf1[64] = {0};
uint8_t *g_pu8RxBuf1 = 0;
uint32_t g_u32RxSize1 = 0;
uint32_t g_u32TxSize1 = 0;

volatile int8_t g_i8BulkOutReady1 = 0;

void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);
void PowerDown(void);

#ifdef VBUS_DIVIDER

#include "utcpd.c"
#include "i2c_controller.c"

int port = 0;
int alert;
uint32_t u32VBUSDetEn, u32VBUSPresent = 0, u32VBUSPrevious = 0, u32VCONNPresent, u32SnkVBUS;

void GPIO_Init(void)
{
    /* Enable PA13~14 (D+ / D-) interrupt for wakeup */
    GPIO_CLR_INT_FLAG(PA, BIT13 | BIT14);
    GPIO_EnableInt(PA, 13, GPIO_INT_BOTH_EDGE);
    GPIO_EnableInt(PA, 14, GPIO_INT_BOTH_EDGE);
    GPIO_ENABLE_DEBOUNCE(PA, BIT13 | BIT14);   /* Enable key debounce */
}
#endif

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

#if (!CRYSTAL_LESS)
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    CLK_EnablePLL(CLK_PLLCTL_PLLSRC_HXT, FREQ_144MHZ);

    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(3));

    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_PLL, CLK_CLKDIV0_USB(3));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
#else
    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable HIRC48 clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRC48MEN_Msk);

    /* Waiting for HIRC48 clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRC48MSTB_Msk);

    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_HIRC48M, CLK_CLKDIV0_USB(1));
#endif

#ifdef VBUS_DIVIDER
    /* Enable UTCPD module clock */
    CLK_EnableModuleClock(UTCPD0_MODULE);

    /* Enable GPA module clock */
    CLK_EnableModuleClock(GPA_MODULE);
#endif
    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable UART1 module clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Select UART1 module clock source as HIRC and UART1 module clock divider as 1 */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL4_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    /* Enable GPB module clock */
    CLK_EnableModuleClock(GPB_MODULE);

    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_USBEN_Msk | SYS_USBPHY_SBO_Msk;

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/

#ifdef VBUS_DIVIDER
    SYS->GPA_MFP3 &= ~(SYS_GPA_MFP3_PA13MFP_Msk | SYS_GPA_MFP3_PA14MFP_Msk | SYS_GPA_MFP3_PA15MFP_Msk);

    /* USBD multi-function pins for D+, D-, and ID pins */
    SYS->GPA_MFP3 |= (SYS_GPA_MFP3_PA13MFP_USB_D_N | SYS_GPA_MFP3_PA14MFP_USB_D_P | SYS_GPA_MFP3_PA15MFP_USB_OTG_ID);
#endif

    /* Set multi-function pins */
    Uart0DefaultMPF();

    /* Set multi-function pins for UART1 RXD and TXD */
    SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~(SYS_GPB_MFP0_PB2MFP_Msk|SYS_GPB_MFP0_PB3MFP_Msk)) | (SYS_GPB_MFP0_PB2MFP_UART1_RXD | SYS_GPB_MFP0_PB3MFP_UART1_TXD);

    /* Lock protected registers */
    SYS_LockReg();

}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART0, 115200);

    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}

void UART1_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS_ResetModule(UART1_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART1, 115200);

    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    uint8_t u8InChar;
    int32_t i32Size;
    uint32_t u32IntStatus;

    u32IntStatus = UART0->INTSTS;

    if((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while(!(UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
        {
            /* Get the character from UART Buffer */
            u8InChar = (uint8_t)UART0->DAT;

            /* Check if buffer full */
            if(g_u16ComRbytes0 < RXBUFSIZE)
            {
                /* Enqueue the character */
                s_au8ComRbuf0[g_u16ComRtail0++] = u8InChar;
                if(g_u16ComRtail0 >= RXBUFSIZE)
                    g_u16ComRtail0 = 0;
                g_u16ComRbytes0++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_INTSTS_THREIF_Msk)
    {

        if(g_u16ComTbytes0 && (UART0->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            i32Size = g_u16ComTbytes0;
            if(i32Size >= UART0_FIFO_SIZE)
            {
                i32Size = UART0_FIFO_SIZE;
            }

            while(i32Size)
            {
                u8InChar = s_au8ComTbuf0[g_u16ComThead0++];
                UART0->DAT = u8InChar;
                if(g_u16ComThead0 >= TXBUFSIZE)
                    g_u16ComThead0 = 0;
                g_u16ComTbytes0--;
                i32Size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART0->INTEN &= ~UART_INTEN_THREIEN_Msk;
        }
    }
}

void UART1_IRQHandler(void)
{
    uint8_t u8InChar;
    int32_t i32Size;
    uint32_t u32IntStatus;

    u32IntStatus = UART1->INTSTS;

    if((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while(!(UART1->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
        {
            /* Get the character from UART Buffer */
            u8InChar = (uint8_t)UART1->DAT;

            /* Check if buffer full */
            if(g_u16ComRbytes1 < RXBUFSIZE)
            {
                /* Enqueue the character */
                s_au8ComRbuf1[g_u16ComRtail1++] = u8InChar;
                if(g_u16ComRtail1 >= RXBUFSIZE)
                    g_u16ComRtail1 = 0;
                g_u16ComRbytes1++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_INTSTS_THREIF_Msk)
    {

        if(g_u16ComTbytes1 && (UART1->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            i32Size = g_u16ComTbytes1;
            if(i32Size >= UART1_FIFO_SIZE)
            {
                i32Size = UART1_FIFO_SIZE;
            }

            while(i32Size)
            {
                u8InChar = s_au8ComTbuf1[g_u16ComThead1++];
                UART1->DAT = u8InChar;
                if(g_u16ComThead1 >= TXBUFSIZE)
                    g_u16ComThead1 = 0;
                g_u16ComTbytes1--;
                i32Size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART1->INTEN &= ~UART_INTEN_THREIEN_Msk;
        }
    }
}

void VCOM_TransferData(void)
{
    uint32_t i, u32Len;

    /* Check whether USB is ready for next packet or not */
    if(g_u32TxSize0 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(g_u16ComRbytes0)
        {
            u32Len = g_u16ComRbytes0;
            if(u32Len > EP2_MAX_PKT_SIZE)
                u32Len = EP2_MAX_PKT_SIZE;

            for(i = 0; i < u32Len; i++)
            {
                s_au8RxBuf0[i] = s_au8ComRbuf0[g_u16ComRhead0++];
                if(g_u16ComRhead0 >= RXBUFSIZE)
                    g_u16ComRhead0 = 0;
            }

            __set_PRIMASK(1);
            g_u16ComRbytes0 -= u32Len;
            __set_PRIMASK(0);

            g_u32TxSize0 = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)s_au8RxBuf0, u32Len);
            USBD_SET_PAYLOAD_LEN(EP2, u32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP2_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            u32Len = USBD_GET_PAYLOAD_LEN(EP2);
            if(u32Len == EP2_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP2, 0);
        }
    }

    if(g_u32TxSize1 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(g_u16ComRbytes1)
        {
            u32Len = g_u16ComRbytes1;
            if(u32Len > EP7_MAX_PKT_SIZE)
                u32Len = EP7_MAX_PKT_SIZE;

            for(i = 0; i < u32Len; i++)
            {
                s_au8RxBuf1[i] = s_au8ComRbuf1[g_u16ComRhead1++];
                if(g_u16ComRhead1 >= RXBUFSIZE)
                    g_u16ComRhead1 = 0;
            }

            __set_PRIMASK(1);
            g_u16ComRbytes1 -= u32Len;
            __set_PRIMASK(0);

            g_u32TxSize1 = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP7)), (uint8_t *)s_au8RxBuf1, u32Len);
            USBD_SET_PAYLOAD_LEN(EP7, u32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP7_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            u32Len = USBD_GET_PAYLOAD_LEN(EP7);
            if(u32Len == EP7_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP7, 0);
        }
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if(g_i8BulkOutReady0 && (g_u32RxSize0 <= TXBUFSIZE - g_u16ComTbytes0))
    {
        for(i = 0; i < g_u32RxSize0; i++)
        {
            s_au8ComTbuf0[g_u16ComTtail0++] = g_pu8RxBuf0[i];
            if(g_u16ComTtail0 >= TXBUFSIZE)
                g_u16ComTtail0 = 0;
        }

        __set_PRIMASK(1);
        g_u16ComTbytes0 += g_u32RxSize0;
        __set_PRIMASK(0);

        g_u32RxSize0 = 0;
        g_i8BulkOutReady0 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }

    if(g_i8BulkOutReady1 && (g_u32RxSize1 <= TXBUFSIZE - g_u16ComTbytes1))
    {
        for(i = 0; i < g_u32RxSize1; i++)
        {
            s_au8ComTbuf1[g_u16ComTtail1++] = g_pu8RxBuf1[i];
            if(g_u16ComTtail1 >= TXBUFSIZE)
                g_u16ComTtail1 = 0;
        }

        __set_PRIMASK(1);
        g_u16ComTbytes1 += g_u32RxSize1;
        __set_PRIMASK(0);

        g_u32RxSize1 = 0;
        g_i8BulkOutReady1 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
    }

    /* Process the software Tx FIFO */
    if(g_u16ComTbytes0)
    {
        /* Check if Tx is working */
        if((UART0->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART0->DAT = s_au8ComTbuf0[g_u16ComThead0++];
            if(g_u16ComThead0 >= TXBUFSIZE)
                g_u16ComThead0 = 0;

            g_u16ComTbytes0--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART0->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }

    if(g_u16ComTbytes1)
    {
        /* Check if Tx is working */
        if((UART1->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART1->DAT = s_au8ComTbuf1[g_u16ComThead1++];
            if(g_u16ComThead1 >= TXBUFSIZE)
                g_u16ComThead1 = 0;

            g_u16ComTbytes1--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART1->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
}
#ifdef VBUS_DIVIDER
void UTCPD_IRQHandler(void)
{
    UTCPD_GetAlertStatus(0, &alert);

    if(alert & UTCPD_ALERT_PWRSCHIS)
    {
        UTCPD_GetPwrSts(port, &u32VBUSDetEn, &u32VBUSPresent, &u32VCONNPresent, &u32SnkVBUS);

        if(u32VBUSPresent != u32VBUSPrevious)
        {
            if(u32VBUSPresent)
            {
                /* USB Plug In */
                USBD_ENABLE_USB();
                printf("Plug\n");
            }
            else
            {
                /* USB Un-plug */
                USBD_DISABLE_USB();
                printf("Un-Plug\n");
            }
            u32VBUSPrevious = u32VBUSPresent;
        }
    }
    UTCPD_ClearAlertStatus(0, alert);
}
#endif

void PowerDown(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WKEN_Msk);

    UART_WAIT_TX_EMPTY(DEBUG_PORT);

#ifdef VBUS_DIVIDER
    /* Change USBD multi-function pins (D+, D-) to GPIO */
    SYS->GPA_MFP3 &= ~(SYS_GPA_MFP3_PA12MFP_Msk | SYS_GPA_MFP3_PA13MFP_Msk | SYS_GPA_MFP3_PA14MFP_Msk);

    GPIO_CLR_INT_FLAG(PA, BIT13 | BIT14);
#endif

    CLK_PowerDown();

#ifdef VBUS_DIVIDER
    GPIO_CLR_INT_FLAG(PA, BIT13 | BIT14);

    /* Change PA13 & PA14 to USBD multi-function pins (D+, D-) */
    SYS->GPA_MFP3 |= (SYS_GPA_MFP3_PA13MFP_USB_D_N | SYS_GPA_MFP3_PA14MFP_USB_D_P);
#else
    /* Change PA13 & PA14 to USBD multi-function pins (D+, D-) */
    SYS->GPA_MFP3 |= (SYS_GPA_MFP3_PA12MFP_USB_VBUS | SYS_GPA_MFP3_PA13MFP_USB_D_N | SYS_GPA_MFP3_PA14MFP_USB_D_P | SYS_GPA_MFP3_PA15MFP_USB_OTG_ID);
#endif

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if(CLK->PWRCTL & CLK_PWRCTL_PDEN_Msk)
        CLK->PWRCTL ^= CLK_PWRCTL_PDEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART */
    UART0_Init();
    UART1_Init();

    printf("\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|        NuMicro USB Virtual COM Dual Port Sample Code        |\n");
    printf("+-------------------------------------------------------------+\n");

#ifdef VBUS_DIVIDER

    GPIO_DISABLE_DIGITAL_PATH(PA, BIT12);

    UTCPD->MUXSEL = (UTCPD->MUXSEL & ~(UTCPD_MUXSEL_ADCSELVB_Msk | UTCPD_MUXSEL_ADCSELVC_Msk)) | ((2 << UTCPD_MUXSEL_ADCSELVB_Pos) | (3 << UTCPD_MUXSEL_ADCSELVC_Pos));

    UTCPD->VBVOL = (UTCPD->VBVOL & ~UTCPD_VBVOL_VBSCALE_Msk) | (2 << UTCPD_VBVOL_VBSCALE_Pos);

    UTCPD_Open(port);

    UTCPD_SetRoleCtrl(port, (uint32_t)NULL, UTCPD_ROLECTL_RPVALUE_1P5A, UTCPD_ROLECTL_CC2_RD, UTCPD_ROLECTL_CC1_RD);

    UTCPD_DisablePowerCtrl (port, UTCPD_PWRCTL_VBMONI_DIS);

    UTCPD_IsssueCmd(port, UTCPD_CMD_ENABLE_VBUS_DETECT);

    UTCPD_EnableAlertMask(0, UTCPD_ALERTM_PWRSCHIE);

    UTCPD_EnablePowerStatusMask(0, UTCPD_PWRSM_VBPSIE);

    NVIC_EnableIRQ(UTCPD_IRQn);

    GPIO_Init();
#else
    SYS->GPA_MFP3 &= ~(SYS_GPA_MFP3_PA12MFP_Msk | SYS_GPA_MFP3_PA13MFP_Msk | SYS_GPA_MFP3_PA14MFP_Msk | SYS_GPA_MFP3_PA15MFP_Msk);

    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    SYS->GPA_MFP3 |= (SYS_GPA_MFP3_PA12MFP_USB_VBUS | SYS_GPA_MFP3_PA13MFP_USB_D_N | SYS_GPA_MFP3_PA14MFP_USB_D_P | SYS_GPA_MFP3_PA15MFP_USB_OTG_ID);
#endif
    USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);

    /* Endpoint configuration */
    VCOM_Init();

    USBD_Start();

    NVIC_EnableIRQ(UART0_IRQn);
    NVIC_EnableIRQ(UART1_IRQn);

    NVIC_EnableIRQ(USBD_IRQn);

#if CRYSTAL_LESS
    /* Backup default trim */
    u32TrimInit = M32(TRIM_INIT);

    /* Waiting for USB bus stable */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

    while((USBD->INTSTS & USBD_INTSTS_SOFIF_Msk) == 0);

    /* Enable USB crystal-less */
    SYS->HIRCTCTL |= (SYS_HIRCTCTL_REFCKSEL_Msk | 0x1);
#endif

    while(1)
    {
#if CRYSTAL_LESS
        /* Start USB trim if it is not enabled. */
        if((SYS->HIRCTCTL & SYS_HIRCTCTL_FREQSEL_Msk) != 1)
        {
            /* Start USB trim only when SOF */
            if(USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

                /* Re-enable crystal-less */
                SYS->HIRCTCTL = 0x01;
                SYS->HIRCTCTL |= SYS_HIRCTCTL_REFCKSEL_Msk | SYS_HIRCTCTL_BOUNDEN_Msk | (8 << SYS_HIRCTCTL_BOUNDARY_Pos);
            }
        }

        /* Disable USB Trim when error */
        if(SYS->HIRCTISTS & (SYS_HIRCTISTS_CLKERRIF_Msk | SYS_HIRCTISTS_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable crystal-less */
            SYS->HIRCTCTL = 0;

            /* Clear error flags */
            SYS->HIRCTISTS = SYS_HIRCTISTS_CLKERRIF_Msk | SYS_HIRCTISTS_TFAILIF_Msk;

            /* Clear SOF */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
        }
#endif

        /* Enter power down when USB suspend */
        if(g_u8Suspend)
            PowerDown();

        VCOM_TransferData();
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/