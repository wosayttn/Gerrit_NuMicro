/******************************************************************************
 * @file     ccid_vcom.c
 * @version  V2.00
 * @brief    M252 USB composite device(CCID smart card reader and VCOM) sample file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*!<Includes */
#include <string.h>
#include "NuMicro.h"
#include "ccid_vcom.h"
#include "ccid_if.h"
#include "sclib.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32OutToggle = 0;

/* UART0 */
volatile uint8_t  g_au8ComRbuf[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes = 0;
volatile uint16_t g_u16ComRhead = 0;
volatile uint16_t g_u16ComRtail = 0;

volatile uint8_t  g_au8ComTbuf[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes = 0;
volatile uint16_t g_u16ComThead = 0;
volatile uint16_t g_u16ComTtail = 0;

uint8_t g_au8RxBuf[64] = {0};
volatile uint8_t *g_pu8RxBuf = 0;
volatile uint32_t g_u32RxSize = 0;
volatile uint32_t g_u32TxSize = 0;

volatile int8_t g_i8BulkOutReady = 0;

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING g_sLineCoding = {115200, 0, 0, 8};   /* Baud rate: 115200     */
/* Stop bit : 1 Stops Bit*/
/* parity   : None       */
/* data bits: 8 Bits     */
uint16_t g_u16CtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* TX buffer size */

#define TX_FIFO_SIZE        64  /* TX Hardware FIFO size */

/*--------------------------------------------------------------------------*/
void USBD_IRQHandler(void)
{
    uint32_t u32IntSts = USBD_GET_INT_FLAG();
    uint32_t u32State = USBD_GET_BUS_STATE();

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_FLDET)
    {
        // Floating detect
        USBD_CLR_INT_FLAG(USBD_INTSTS_FLDET);

        if (USBD_IS_ATTACHED())
        {
            /* USB Plug In */
            USBD_ENABLE_USB();
        }
        else
        {
            /* USB Un-plug */
            USBD_DISABLE_USB();
        }
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_WAKEUP)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_WAKEUP);
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_BUS)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if (u32State & USBD_STATE_USBRST)
        {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();
            g_u32OutToggle = 0;
            g_i8BulkOutReady = 0;
        }

        if (u32State & USBD_STATE_SUSPEND)
        {
            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
        }

        if (u32State & USBD_STATE_RESUME)
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
        }
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_USB)
    {
        // USB event
        if (u32IntSts & USBD_INTSTS_SETUP)
        {
            // Setup packet
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_SETUP);

            /* Clear the data IN/OUT ready flag of control end-points */
            USBD_STOP_TRANSACTION(EP0);
            USBD_STOP_TRANSACTION(EP1);

            USBD_ProcessSetupPacket();
        }

        // EP events
        if (u32IntSts & USBD_INTSTS_EP0)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP0);

            // control IN
            USBD_CtrlIn();
        }

        if (u32IntSts & USBD_INTSTS_EP1)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP1);

            // control OUT
            USBD_CtrlOut();
        }

        if (u32IntSts & USBD_INTSTS_EP2)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP2);
            // Bulk IN
            EP2_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP3)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP3);
            // Bulk Out
            EP3_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP4)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);
        }

        if (u32IntSts & USBD_INTSTS_EP5)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);

            // Bulk IN
            EP5_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP6)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);

            // Bulk Out
            EP6_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP7)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);
        }
    }
}

void EP2_Handler(void)
{
    /* BULK IN transfer */
    if (gu8IsBulkInReady)
    {
        if (gi32UsbdMessageLength >= EP2_MAX_PKT_SIZE)
        {
            gu8IsBulkInReady = 1;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), pUsbMessageBuffer, EP2_MAX_PKT_SIZE);
            USBD_SET_PAYLOAD_LEN(EP2, EP2_MAX_PKT_SIZE);

            pUsbMessageBuffer += EP2_MAX_PKT_SIZE;
            gi32UsbdMessageLength -= EP2_MAX_PKT_SIZE;
        }
        else
        {
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), pUsbMessageBuffer, gi32UsbdMessageLength);
            USBD_SET_PAYLOAD_LEN(EP2, gi32UsbdMessageLength);
            gi32UsbdMessageLength = 0;
            gu8IsBulkInReady = 0;
        }
    }

    if (!gu8IsBulkOutReady)
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
}


void EP3_Handler(void)
{
    /* BULK OUT */
    static int offset = 0;
    uint32_t len;

    if (g_u32OutToggle == (USBD->EPSTS0 & USBD_EPSTS0_EPSTS3_Msk))
    {
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }
    else
    {
        g_u32OutToggle = USBD->EPSTS0 & USBD_EPSTS0_EPSTS3_Msk;
        len = USBD_GET_PAYLOAD_LEN(EP3);

        USBD_MemCopy(&UsbMessageBuffer[offset], (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3)), len);

        if ((len >= 0x0A && len != 0xFF) || offset != 0)
        {
            if (offset == 0)
            {
                /* Calculate number of byte to receive to finish the message  */
                gi32UsbdMessageLength = USB_MESSAGE_HEADER_SIZE + make32(&UsbMessageBuffer[OFFSET_DWLENGTH]);
            }

            gi32UsbdMessageLength -= (int) len;

            /* Prepare next reception if whole message not received */
            if (gi32UsbdMessageLength > 0)
            {
                pUsbMessageBuffer = UsbMessageBuffer + len;
                offset += len;
            }

            if (gi32UsbdMessageLength == 0)
            {
                gu8IsBulkOutReady = 1;
                offset = 0;
            }

            if (gi32UsbdMessageLength < 0)
            {
                UsbMessageBuffer[OFFSET_DWLENGTH] = 0xFF;
                UsbMessageBuffer[OFFSET_DWLENGTH + 1] = 0xFF;
                UsbMessageBuffer[OFFSET_DWLENGTH + 2] = 0xFF;
                UsbMessageBuffer[OFFSET_DWLENGTH + 3] = 0xFF;
                gu8IsBulkOutReady = 1;
            }
        }

        CCID_DispatchMessage();

        /* trigger next out packet */
        if (gi32UsbdMessageLength > 0)
            USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }
}

void EP4_Handler(void)
{
    /* INT IN transfer */
    if (gu8IsDeviceReady)
    {
        RDR_to_PC_NotifySlotChange();
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4)), pu8IntInBuf, 2);
        USBD_SET_PAYLOAD_LEN(EP4, 2);
        gu8IsDeviceReady = 0;
    }

    if (!gu8IsBulkOutReady)
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
}



void EP5_Handler(void)
{
    g_u32TxSize = 0;
}

void EP6_Handler(void)
{
    /* Bulk OUT */
    if (g_u32OutToggle == (USBD->EPSTS0 & USBD_EPSTS0_EPSTS6_Msk))
    {
        USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
    }
    else
    {
        g_u32RxSize = USBD_GET_PAYLOAD_LEN(EP6);
        g_pu8RxBuf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP6));

        g_u32OutToggle = USBD->EPSTS0 & USBD_EPSTS0_EPSTS6_Msk;
        /* Set a flag to indicate bulk out ready */
        g_i8BulkOutReady = 1;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* The interrupt services routine of smartcard port 0                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void SC0_IRQHandler(void)
{
    /* Please don't remove any of the function calls below */
    if (SCLIB_CheckCDEvent(0))
    {
        RDR_to_PC_NotifySlotChange();
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4)), pu8IntInBuf, 2);
        USBD_SET_PAYLOAD_LEN(EP4, 2);
        return; // Card insert/remove event occurred, no need to check other event...
    }

    SCLIB_CheckTimeOutEvent(0);
    SCLIB_CheckTxRxEvent(0);
    SCLIB_CheckErrorEvent(0);

    return;
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    uint32_t u32IntStatus;
    uint8_t bInChar;
    int32_t size;

    u32IntStatus = UART0->INTSTS;

    if ((u32IntStatus & 0x1 /* RDAIF */) || (u32IntStatus & 0x10 /* TOUT_IF */))
    {
        /* Receiver FIFO threashold level is reached or RX time out */

        /* Get all the input characters */
        while ((UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            /* Get the character from UART Buffer */
            bInChar = UART0->DAT;

            /* Check if buffer full */
            if (g_u16ComRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                g_au8ComRbuf[g_u16ComRtail++] = bInChar;

                if (g_u16ComRtail >= RXBUFSIZE)
                    g_u16ComRtail = 0;

                g_u16ComRbytes++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if (u32IntStatus & 0x2 /* THRE_IF */)
    {

        if (g_u16ComTbytes)
        {
            /* Fill the TX FIFO */
            size = g_u16ComTbytes;

            if (size >= TX_FIFO_SIZE)
            {
                size = TX_FIFO_SIZE;
            }

            while (size)
            {
                bInChar = g_au8ComTbuf[g_u16ComThead++];
                UART0->DAT = bInChar;

                if (g_u16ComThead >= TXBUFSIZE)
                    g_u16ComThead = 0;

                g_u16ComTbytes--;
                size--;
            }
        }
        else
        {
            /* No more data, just stop TX (Stop work) */
            UART0->INTEN &= (~UART_INTEN_THREIEN_Msk);
        }
    }

}

void VCOM_TransferData(void)
{
    uint32_t i, u32Len;

    /* Check wether USB is ready for next packet or not*/
    if (g_u32TxSize == 0)
    {
        /* Check wether we have new COM Rx data to send to USB or not */
        if (g_u16ComRbytes)
        {
            u32Len = g_u16ComRbytes;

            if (u32Len > EP5_MAX_PKT_SIZE)
                u32Len = EP5_MAX_PKT_SIZE;

            for (i = 0; i < u32Len; i++)
            {
                g_au8RxBuf[i] = g_au8ComRbuf[g_u16ComRhead++];

                if (g_u16ComRhead >= RXBUFSIZE)
                    g_u16ComRhead = 0;
            }

            __set_PRIMASK(1);
            g_u16ComRbytes -= u32Len;
            __set_PRIMASK(0);

            g_u32TxSize = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP5)), (uint8_t *)g_au8RxBuf, u32Len);
            USBD_SET_PAYLOAD_LEN(EP5, u32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP5_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            u32Len = USBD_GET_PAYLOAD_LEN(EP5);

            if (u32Len == EP5_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP5, 0);
        }
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if (g_i8BulkOutReady && (g_u32RxSize <= TXBUFSIZE - g_u16ComTbytes))
    {
        for (i = 0; i < g_u32RxSize; i++)
        {
            g_au8ComTbuf[g_u16ComTtail++] = g_pu8RxBuf[i];

            if (g_u16ComTtail >= TXBUFSIZE)
                g_u16ComTtail = 0;
        }

        __set_PRIMASK(1);
        g_u16ComTbytes += g_u32RxSize;
        __set_PRIMASK(0);

        g_u32RxSize = 0;
        g_i8BulkOutReady = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
    }

    /* Process the software TX FIFO */
    if (g_u16ComTbytes)
    {
        /* Check if TX is working */
        if ((UART0->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART0->DAT = g_au8ComTbuf[g_u16ComThead++];

            if (g_u16ComThead >= TXBUFSIZE)
                g_u16ComThead = 0;

            __set_PRIMASK(1);
            g_u16ComTbytes--;
            __set_PRIMASK(0);

            /* Enable TX Empty Interrupt. (Trigger first one) */
            UART0->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
}

void VCOM_LineCoding(uint8_t port)
{
    uint32_t u32Reg;
    uint32_t u32Baud_Div;

    if (port == 0)
    {
        NVIC_DisableIRQ(UART0_IRQn);
        // Reset software FIFO
        g_u16ComRbytes = 0;
        g_u16ComRhead = 0;
        g_u16ComRtail = 0;

        g_u16ComTbytes = 0;
        g_u16ComThead = 0;
        g_u16ComTtail = 0;

        // Reset hardware FIFO
        UART0->FIFO = UART_FIFO_RXRST_Msk | UART_FIFO_TXRST_Msk;

        // Set baudrate
        u32Baud_Div = UART_BAUD_MODE2_DIVIDER(__HIRC, g_sLineCoding.u32DTERate);

        if (u32Baud_Div > 0xFFFF)
            UART0->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(__HIRC, g_sLineCoding.u32DTERate));
        else
            UART0->BAUD = (UART_BAUD_MODE2 | u32Baud_Div);

        // Set parity
        if (g_sLineCoding.u8ParityType == 0)
            u32Reg = 0; // none parity
        else if (g_sLineCoding.u8ParityType == 1)
            u32Reg = 0x08; // odd parity
        else if (g_sLineCoding.u8ParityType == 2)
            u32Reg = 0x18; // even parity
        else
            u32Reg = 0;

        // bit width
        switch (g_sLineCoding.u8DataBits)
        {
            case 5:
                u32Reg |= 0;
                break;

            case 6:
                u32Reg |= 1;
                break;

            case 7:
                u32Reg |= 2;
                break;

            case 8:
                u32Reg |= 3;
                break;

            default:
                break;
        }

        // stop bit
        if (g_sLineCoding.u8CharFormat > 0)
            u32Reg |= 0x4; // 2 or 1.5 bits

        UART0->LINE = u32Reg;

        // Re-enable UART interrupt
        NVIC_EnableIRQ(UART0_IRQn);
    }
}

/*--------------------------------------------------------------------------*/
/**
  * @brief  USBD Endpoint Config.
  * @param  None.
  * @retval None.
  */
void CCID_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer for setup packet -> [0 ~ 0x7] */
    USBD->STBUFSEG = SETUP_BUF_BASE;

    /*****************************************************/
    /* EP0 ==> control IN endpoint, address 0 */
    USBD_CONFIG_EP(EP0, USBD_CFG_CSTALL | USBD_CFG_EPMODE_IN | 0);
    /* Buffer range for EP0 */
    USBD_SET_EP_BUF_ADDR(EP0, EP0_BUF_BASE);

    /* EP1 ==> control OUT endpoint, address 0 */
    USBD_CONFIG_EP(EP1, USBD_CFG_CSTALL | USBD_CFG_EPMODE_OUT | 0);
    /* Buffer range for EP1 */
    USBD_SET_EP_BUF_ADDR(EP1, EP1_BUF_BASE);

    /*****************************************************/
    /* EP2 ==> Bulk IN endpoint, address 2 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM);
    /* Buffer offset for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /* EP3 ==> Bulk Out endpoint, address 2 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM);
    /* Buffer offset for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

    /* EP4 ==> Interrupt IN endpoint, address 3 */
    USBD_CONFIG_EP(EP4, USBD_CFG_EPMODE_IN | INT_IN_EP_NUM);
    /* Buffer offset for EP4 ->  */
    USBD_SET_EP_BUF_ADDR(EP4, EP4_BUF_BASE);

    /* check card state */
    gu8IsDeviceReady = 1;
    pu8IntInBuf = &UsbIntMessageBuffer[0];
    pUsbMessageBuffer = &UsbMessageBuffer[0];
    RDR_to_PC_NotifySlotChange();
    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4)), pu8IntInBuf, 2);
    USBD_SET_PAYLOAD_LEN(EP4, 2);


    /*****************************************************/
    /* EP5 ==> Bulk IN endpoint, address 4 */
    USBD_CONFIG_EP(EP5, USBD_CFG_EPMODE_IN | VCOM_BULK_IN_EP_NUM);
    /* Buffer offset for EP5 */
    USBD_SET_EP_BUF_ADDR(EP5, EP5_BUF_BASE);

    /* EP6 ==> Bulk Out endpoint, address 4 */
    USBD_CONFIG_EP(EP6, USBD_CFG_EPMODE_OUT | VCOM_BULK_OUT_EP_NUM);
    /* Buffer offset for EP6 */
    USBD_SET_EP_BUF_ADDR(EP6, EP6_BUF_BASE);
    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);

    /* EP7 ==> Interrupt IN endpoint, address 5 */
    USBD_CONFIG_EP(EP7, USBD_CFG_EPMODE_IN | VCOM_INT_IN_EP_NUM);
    /* Buffer offset for EP7 ->  */
    USBD_SET_EP_BUF_ADDR(EP7, EP7_BUF_BASE);

}


void CCID_ClassRequest(void)
{
    uint8_t buf[8];

    USBD_GetSetupPacket(buf);

    if (buf[0] & 0x80)   /* request data transfer direction */
    {
        // Device to host
        switch (buf[1])
        {
            case CCID_GET_CLOCK_FREQUENCIES:
            case CCID_GET_DATA_RATES:
            {
                uint8_t pData[1] = {0};
                USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), pData, sizeof(pData));
                /* Data stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, sizeof(pData));
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }

            case GET_LINE_CODE:
            {
                if (buf[4] == 1)   /* VCOM-1 */
                {
                    USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), (uint8_t *)&g_sLineCoding, 7);
                }

                /* Data stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 7);
                /* Status stage */
                USBD_PrepareCtrlOut(0, 0);
                break;
            }

            default:
            {
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
            }
        }
    }
    else
    {
        // Host to device
        switch (buf[1])
        {
            case CCID_ABORT:
            {
                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }

            case SET_CONTROL_LINE_STATE:
            {
                if (buf[4] == 0)   /* VCOM-1 */
                {
                    g_u16CtrlSignal = buf[3];
                    g_u16CtrlSignal = (g_u16CtrlSignal << 8) | buf[2];
                    //printf("RTS=%d  DTR=%d\n", (gCtrlSignal0 >> 1) & 1, gCtrlSignal0 & 1);
                }

                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }

            case SET_LINE_CODE:
            {
                if (buf[4] == 1) /* VCOM-1 */
                    USBD_PrepareCtrlOut((uint8_t *)&g_sLineCoding, 7);

                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);

                /* UART setting */
                if (buf[4] == 1) /* VCOM-1 */
                    VCOM_LineCoding(0);

                break;
            }

            default:
            {
                // Stall
                /* Setup error, stall the device */
                USBD_SetStall(0);
                break;
            }
        }
    }
}

void CCID_BulkInMessage(void)
{
    gi32UsbdMessageLength = USB_MESSAGE_HEADER_SIZE + make32(&UsbMessageBuffer[OFFSET_DWLENGTH]);

    pUsbMessageBuffer = UsbMessageBuffer;

    if (gu8IsBulkInReady)
    {
        if (gi32UsbdMessageLength >= EP2_MAX_PKT_SIZE)
        {
            gu8IsBulkInReady = 1;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), pUsbMessageBuffer, EP2_MAX_PKT_SIZE);
            USBD_SET_PAYLOAD_LEN(EP2, EP2_MAX_PKT_SIZE);

            pUsbMessageBuffer += EP2_MAX_PKT_SIZE;
            gi32UsbdMessageLength -= EP2_MAX_PKT_SIZE;
        }
        else
        {
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), pUsbMessageBuffer, gi32UsbdMessageLength);
            USBD_SET_PAYLOAD_LEN(EP2, gi32UsbdMessageLength);
            gi32UsbdMessageLength = 0;
            gu8IsBulkInReady = 0;
        }
    }
}


void CCID_DispatchMessage(void)
{
    uint8_t ErrorCode;

    if (gu8IsBulkOutReady)
    {
        switch (UsbMessageBuffer[OFFSET_BMESSAGETYPE])
        {
            case PC_TO_RDR_ICCPOWERON:
                ErrorCode = PC_to_RDR_IccPowerOn();
                RDR_to_PC_DataBlock(ErrorCode);
                break;

            case PC_TO_RDR_ICCPOWEROFF:
                ErrorCode = PC_to_RDR_IccPowerOff();
                RDR_to_PC_SlotStatus(ErrorCode);
                break;

            case PC_TO_RDR_GETSLOTSTATUS:
                ErrorCode = PC_to_RDR_GetSlotStatus();
                RDR_to_PC_SlotStatus(ErrorCode);
                break;

            case PC_TO_RDR_XFRBLOCK:
                ErrorCode = PC_to_RDR_XfrBlock();
                RDR_to_PC_DataBlock(ErrorCode);
                break;

            case PC_TO_RDR_GETPARAMETERS:
                ErrorCode = PC_to_RDR_GetParameters();
                RDR_to_PC_Parameters(ErrorCode);
                break;

            case PC_TO_RDR_RESETPARAMETERS:
                ErrorCode = PC_to_RDR_ResetParameters();
                RDR_to_PC_Parameters(ErrorCode);
                break;

            case PC_TO_RDR_SETPARAMETERS:
                ErrorCode = PC_to_RDR_SetParameters();
                RDR_to_PC_Parameters(ErrorCode);
                break;

            case PC_TO_RDR_ESCAPE:
                ErrorCode = PC_to_RDR_Escape();
                RDR_to_PC_Escape(ErrorCode);
                break;

            case PC_TO_RDR_ICCCLOCK:
                ErrorCode = PC_to_RDR_IccClock();
                RDR_to_PC_SlotStatus(ErrorCode);
                break;

            case PC_TO_RDR_ABORT:
                ErrorCode = PC_to_RDR_Abort();
                RDR_to_PC_SlotStatus(ErrorCode);
                break;

            case PC_TO_RDR_SETDATARATEANDCLOCKFREQUENCY:
            case PC_TO_RDR_SECURE:
            case PC_TO_RDR_T0APDU:
            case PC_TO_RDR_MECHANICAL:
            default:
                CmdNotSupported();
                break;
        }

        CCID_BulkInMessage();
        gu8IsBulkOutReady = 0;
    }
}

