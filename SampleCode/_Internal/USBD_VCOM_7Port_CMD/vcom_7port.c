/****************************************************************************//**
 * @file     vcom_7port.c
 * @version  V0.10
 * @brief    M253 series USBD VCOM sample file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "NuMicro.h"
#include "vcom_7port.h"

S_USBD_REQ_T g_setup_t;

const VCOM_CONTROL_CONST_t sVCOM_CONST[VCOM_CNT] =
{
#if (VCOM_CNT>=1)
    {
        .UART_BASE          = UART0,
        .eIRQn              = UART0_IRQn,
        .u32TX_FIFO_SIZE    = TX_FIFO_SIZE_0,
    },
#endif
#if (VCOM_CNT>=2)
    {
        .UART_BASE          = UART1,
        .eIRQn              = UART1_IRQn,
        .u32TX_FIFO_SIZE    = TX_FIFO_SIZE_1,
    },
#endif
#if (VCOM_CNT>=3)
    {
        .UART_BASE          = UART2,
        .eIRQn              = UART2_IRQn,
        .u32TX_FIFO_SIZE    = TX_FIFO_SIZE_2,
    },
#endif
#if (VCOM_CNT>=4)
    {
        .UART_BASE          = UART3,
        .eIRQn              = UART3_IRQn,
        .u32TX_FIFO_SIZE    = TX_FIFO_SIZE_3,
    },
#endif
#if (VCOM_CNT>=5)
    {
        .UART_BASE          = UART4,
        .eIRQn              = UART4_IRQn,
        .u32TX_FIFO_SIZE    = TX_FIFO_SIZE_4,
    },
#endif
#if (VCOM_CNT>=6)
    {
        .UART_BASE          = UUART0,
        .eIRQn              = USCI0_IRQn,
        .u32TX_FIFO_SIZE    = TX_FIFO_SIZE_5,
    },
#endif
#if (VCOM_CNT>=7)
    {   /* VCOM 6 loopback without peripheral */
        .UART_BASE  = NULL,
    },
#endif

};

VCOM_CONTROL_VAR_t sVCOM_var[VCOM_CNT] =
{
#if (VCOM_CNT>=1)
    {
        //.au8ComRbuf[RXBUFSIZE],
        .u16ComRbytes            = 0,
        .u16ComRhead             = 0,
        .u16ComRtail             = 0,
        //.au8ComTbuf[TXBUFSIZE];
        .u16ComTbytes            = 0,
        .u16ComThead             = 0,
        .u16ComTtail             = 0,
        .u32OutToggle            = 0,
        .sLineCoding             = {115200, 0, 0, 8}, /* Baud rate: 38400, Stop bit, parity, data bits    */
        .u16CtrlSignal           = 0, /* BIT0                     : DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
        //.tVCOM[IDX_VCOM0].psCtrlVar.au8RxBuf[64] = {0};
        .pu8RxBuf                = NULL,
        .u32RxSize               = 0,
        .u32TxSize               = 0,
        .i8BulkOutReady          = 0,

    },
#endif
#if (VCOM_CNT>=2)
    {
        //.au8ComRbuf[RXBUFSIZE],
        .u16ComRbytes            = 0,
        .u16ComRhead             = 0,
        .u16ComRtail             = 0,
        //.au8ComTbuf[TXBUFSIZE];
        .u16ComTbytes            = 0,
        .u16ComThead             = 0,
        .u16ComTtail             = 0,
        .u32OutToggle            = 0,
        .sLineCoding             = {115200, 0, 0, 8}, /* Baud rate: 38400, Stop bit, parity, data bits    */
        .u16CtrlSignal           = 0, /* BIT0                     : DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
        //.tVCOM[IDX_VCOM1].au8RxBuf[64] = {0};
        .pu8RxBuf                = NULL,
        .u32RxSize               = 0,
        .u32TxSize               = 0,
        .i8BulkOutReady          = 0,
    },
#endif
#if (VCOM_CNT>=3)
    {
        //.au8ComRbuf[RXBUFSIZE],
        .u16ComRbytes            = 0,
        .u16ComRhead             = 0,
        .u16ComRtail             = 0,
        //.au8ComTbuf[TXBUFSIZE];
        .u16ComTbytes            = 0,
        .u16ComThead             = 0,
        .u16ComTtail             = 0,
        .u32OutToggle            = 0,
        .sLineCoding             = {115200, 0, 0, 8}, /* Baud rate: 38400, Stop bit, parity, data bits    */
        .u16CtrlSignal           = 0, /* BIT0                     : DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
        //.tVCOM[IDX_VCOM2].au8RxBuf[64] = {0};
        .pu8RxBuf                = NULL,
        .u32RxSize               = 0,
        .u32TxSize               = 0,
        .i8BulkOutReady          = 0,
    },
#endif
#if (VCOM_CNT>=4)
    {
        //.au8ComRbuf[RXBUFSIZE],
        .u16ComRbytes            = 0,
        .u16ComRhead             = 0,
        .u16ComRtail             = 0,
        //.au8ComTbuf[TXBUFSIZE];
        .u16ComTbytes            = 0,
        .u16ComThead             = 0,
        .u16ComTtail             = 0,
        .u32OutToggle            = 0,
        .sLineCoding             = {115200, 0, 0, 8}, /* Baud rate: 38400, Stop bit, parity, data bits    */
        .u16CtrlSignal           = 0, /* BIT0                     : DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
        //.tVCOM[IDX_VCOM2].au8RxBuf[64] = {0};
        .pu8RxBuf                = NULL,
        .u32RxSize               = 0,
        .u32TxSize               = 0,
        .i8BulkOutReady          = 0,
    },
#endif
#if (VCOM_CNT>=5)
    {
        //.au8ComRbuf[RXBUFSIZE],
        .u16ComRbytes            = 0,
        .u16ComRhead             = 0,
        .u16ComRtail             = 0,
        //.au8ComTbuf[TXBUFSIZE];
        .u16ComTbytes            = 0,
        .u16ComThead             = 0,
        .u16ComTtail             = 0,
        .u32OutToggle            = 0,
        .sLineCoding             = {115200, 0, 0, 8}, /* Baud rate: 38400, Stop bit, parity, data bits    */
        .u16CtrlSignal           = 0, /* BIT0                     : DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
        //.tVCOM[IDX_VCOM2].au8RxBuf[64] = {0};
        .pu8RxBuf                = NULL,
        .u32RxSize               = 0,
        .u32TxSize               = 0,
        .i8BulkOutReady          = 0,
    },
#endif
#if (VCOM_CNT>=6)
    {
        //.au8ComRbuf[RXBUFSIZE],
        .u16ComRbytes            = 0,
        .u16ComRhead             = 0,
        .u16ComRtail             = 0,
        //.au8ComTbuf[TXBUFSIZE];
        .u16ComTbytes            = 0,
        .u16ComThead             = 0,
        .u16ComTtail             = 0,
        .u32OutToggle            = 0,
        .sLineCoding             = {115200, 0, 0, 8}, /* Baud rate: 38400, Stop bit, parity, data bits    */
        .u16CtrlSignal           = 0, /* BIT0                     : DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
        //.tVCOM[IDX_VCOM2].au8RxBuf[64] = {0};
        .pu8RxBuf                = NULL,
        .u32RxSize               = 0,
        .u32TxSize               = 0,
        .i8BulkOutReady          = 0,
    },
#endif
#if (VCOM_CNT>=7)
    {
        //.au8ComRbuf[RXBUFSIZE],
        .u16ComRbytes            = 0,
        .u16ComRhead             = 0,
        .u16ComRtail             = 0,
        //.au8ComTbuf[TXBUFSIZE];
        .u16ComTbytes            = 0,
        .u16ComThead             = 0,
        .u16ComTtail             = 0,
        .u32OutToggle            = 0,
        .sLineCoding             = {115200, 0, 0, 8}, /* Baud rate: 38400, Stop bit, parity, data bits    */
        .u16CtrlSignal           = 0, /* BIT0                     : DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
        //.tVCOM[IDX_VCOM2].au8RxBuf[64] = {0};
        .pu8RxBuf                = NULL,
        .u32RxSize               = 0,
        .u32TxSize               = 0,
        .i8BulkOutReady          = 0,
    },
#endif

};


VCOM_CONTROL_BLOCK_t tVCOM[VCOM_CNT] =
{
#if (VCOM_CNT>=1)
    {
        .psCtrlVar           = &sVCOM_var[IDX_VCOM0],
        .psPeripheralInfo    = (VCOM_CONTROL_CONST_t *) &sVCOM_CONST[IDX_VCOM0],
    },
#endif
#if (VCOM_CNT>=2)
    {
        .psCtrlVar           = &sVCOM_var[IDX_VCOM1],
        .psPeripheralInfo    = (VCOM_CONTROL_CONST_t *) &sVCOM_CONST[IDX_VCOM1],
    },
#endif
#if (VCOM_CNT>=3)
    {
        .psCtrlVar           = &sVCOM_var[IDX_VCOM2],
        .psPeripheralInfo    = (VCOM_CONTROL_CONST_t *) &sVCOM_CONST[IDX_VCOM2],
    },
#endif
#if (VCOM_CNT>=4)
    {
        .psCtrlVar           = &sVCOM_var[IDX_VCOM3],
        .psPeripheralInfo    = (VCOM_CONTROL_CONST_t *) &sVCOM_CONST[IDX_VCOM3],
    },
#endif
#if (VCOM_CNT>=5)
    {
        .psCtrlVar           = &sVCOM_var[IDX_VCOM4],
        .psPeripheralInfo    = (VCOM_CONTROL_CONST_t *) &sVCOM_CONST[IDX_VCOM4],
    },
#endif
#if (VCOM_CNT>=6)
    {
        .psCtrlVar           = &sVCOM_var[IDX_VCOM5],
        .psPeripheralInfo    = (VCOM_CONTROL_CONST_t *) &sVCOM_CONST[IDX_VCOM5],
    },
#endif
#if (VCOM_CNT>=7)
    {
        .psCtrlVar           = &sVCOM_var[IDX_VCOM6],
        .psPeripheralInfo    = (VCOM_CONTROL_CONST_t *) &sVCOM_CONST[IDX_VCOM6],
    },
#endif

};

uint8_t volatile g_u8Suspend = 0;

uint8_t g_u8Idle = 0, g_u8Protocol = 0;

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
    if (u32IntSts & USBD_INTSTS_BUS)
    {
        /* Clear event flag */
        USBD_CLR_INT_FLAG(USBD_INTSTS_BUS);

        if (u32State & USBD_STATE_USBRST)
        {
            /* Bus reset */
            USBD_ENABLE_USB();
            USBD_SwReset();
#if (VCOM_CNT>=1)
            tVCOM[IDX_VCOM0].psCtrlVar->u32OutToggle = 0;
#endif
#if (VCOM_CNT>=2)
            tVCOM[IDX_VCOM1].psCtrlVar->u32OutToggle = 0;
#endif
#if (VCOM_CNT>=3)
            tVCOM[IDX_VCOM2].psCtrlVar->u32OutToggle = 0;
#endif
#if (VCOM_CNT>=4)
            tVCOM[IDX_VCOM3].psCtrlVar->u32OutToggle = 0;
#endif
#if (VCOM_CNT>=5)
            tVCOM[IDX_VCOM4].psCtrlVar->u32OutToggle = 0;
#endif
#if (VCOM_CNT >= 6)
            tVCOM[IDX_VCOM5].psCtrlVar->u32OutToggle = 0;
#endif
#if (VCOM_CNT >= 7)
            tVCOM[IDX_VCOM6].psCtrlVar->u32OutToggle = 0;
#endif
            g_u8Suspend = 0;
        }

        if (u32State & USBD_STATE_SUSPEND)
        {
            /* Enter power down to wait USB attached */
            g_u8Suspend = 1;

            /* Enable USB but disable PHY */
            USBD_DISABLE_PHY();
        }

        if (u32State & USBD_STATE_RESUME)
        {
            /* Enable USB and enable PHY */
            USBD_ENABLE_USB();
            g_u8Suspend = 0;
        }

#ifdef SUPPORT_LPM

        if (u32State & USBD_STATE_L1SUSPEND)
        {
            /*
               TODO: Implement LPM SUSPEND flag here.
                     Recommend implementing the power-saving function in main loop.
            */
        }

        if (u32State & USBD_STATE_L1RESUME)
        {
            /*
               TODO: Implement LPM RESUME flag here.
            */
        }

#endif

    }

    if (u32IntSts & USBD_INTSTS_NEVWKIF_Msk)
    {
        /*Clear no-event wake up interrupt */
        USBD_CLR_INT_FLAG(USBD_INTSTS_NEVWKIF_Msk);
        /*
           TODO: Implement the function that will be executed when device is woken by non-USB event.
        */
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

            /* UART setting */
            if (g_setup_t.setup.bRequest == SET_LINE_CODE)
            {
                VCOM_LineCoding(g_setup_t.setup.wIndex / 2);
            }
        }

#if (VCOM_CNT>=1)

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

#endif
#if (VCOM_CNT>=2)

        if (u32IntSts & USBD_INTSTS_EP4)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP4);

            //Bulk IN
            EP4_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP5)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP5);

            // Bulk Out
            EP5_Handler();
        }

#endif

#if (VCOM_CNT>=3)

        if (u32IntSts & USBD_INTSTS_EP6)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP6);

            //Bulk IN
            EP6_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP7)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP7);

            // Bulk Out
            EP7_Handler();
        }

#endif
#if (VCOM_CNT>=4)

        if (u32IntSts & USBD_INTSTS_EP8)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP8);

            //Bulk IN
            EP8_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP9)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP9);

            // Bulk Out
            EP9_Handler();
        }

#endif
#if (VCOM_CNT>=5)

        if (u32IntSts & USBD_INTSTS_EP10)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP10);

            //Bulk IN
            EP10_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP11)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP11);

            // Bulk Out
            EP11_Handler();
        }

#endif
#if (VCOM_CNT>=6)

        if (u32IntSts & USBD_INTSTS_EP12)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP12);

            //Bulk IN
            EP12_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP13)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP13);

            // Bulk Out
            EP13_Handler();
        }

#endif
#if (VCOM_CNT>=7)

        if (u32IntSts & USBD_INTSTS_EP14)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP14);

            //Bulk IN
            EP14_Handler();
        }

        if (u32IntSts & USBD_INTSTS_EP15)
        {
            /* Clear event flag */
            USBD_CLR_INT_FLAG(USBD_INTSTS_EP15);

            // Bulk Out
            EP15_Handler();
        }

#endif
    }
}

#if (VCOM_CNT>=1)
void EP2_Handler(void)
{
    tVCOM[IDX_VCOM0].psCtrlVar->u32TxSize = 0;
}

void EP3_Handler(void)
{
    /* Bulk OUT */
    if (tVCOM[IDX_VCOM0].psCtrlVar->u32OutToggle == (USBD->EPSTS0 & USBD_EPSTS0_EPSTS3_Msk))
    {
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }
    else
    {
        tVCOM[IDX_VCOM0].psCtrlVar->u32RxSize = USBD_GET_PAYLOAD_LEN(EP3);
        tVCOM[IDX_VCOM0].psCtrlVar->pu8RxBuf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP3));

        tVCOM[IDX_VCOM0].psCtrlVar->u32OutToggle = USBD->EPSTS0 & USBD_EPSTS0_EPSTS3_Msk;
        /* Set a flag to indicate bulk out ready */
        tVCOM[IDX_VCOM0].psCtrlVar->i8BulkOutReady = 1;
    }
}

#endif
#if (VCOM_CNT>=2)
void EP4_Handler(void)
{
    tVCOM[IDX_VCOM1].psCtrlVar->u32TxSize = 0;
}

void EP5_Handler(void)
{
    /* Bulk OUT */
    if (tVCOM[IDX_VCOM1].psCtrlVar->u32OutToggle == (USBD->EPSTS0 & USBD_EPSTS0_EPSTS5_Msk))
    {
        USBD_SET_PAYLOAD_LEN(EP5, EP5_MAX_PKT_SIZE);
    }
    else
    {
        tVCOM[IDX_VCOM1].psCtrlVar->u32RxSize = USBD_GET_PAYLOAD_LEN(EP5);
        tVCOM[IDX_VCOM1].psCtrlVar->pu8RxBuf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP5));

        tVCOM[IDX_VCOM1].psCtrlVar->u32OutToggle = USBD->EPSTS0 & USBD_EPSTS0_EPSTS5_Msk;
        /* Set a flag to indicate bulk out ready */
        tVCOM[IDX_VCOM1].psCtrlVar->i8BulkOutReady = 1;
    }
}

#endif
#if (VCOM_CNT>=3)
void EP6_Handler(void)
{
    tVCOM[IDX_VCOM2].psCtrlVar->u32TxSize = 0;
}

void EP7_Handler(void)
{
    /* Bulk OUT */
    if (tVCOM[IDX_VCOM2].psCtrlVar->u32OutToggle == (USBD->EPSTS0 & USBD_EPSTS0_EPSTS7_Msk))
    {
        USBD_SET_PAYLOAD_LEN(EP7, EP7_MAX_PKT_SIZE);
    }
    else
    {
        tVCOM[IDX_VCOM2].psCtrlVar->u32RxSize = USBD_GET_PAYLOAD_LEN(EP7);
        tVCOM[IDX_VCOM2].psCtrlVar->pu8RxBuf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP7));

        tVCOM[IDX_VCOM2].psCtrlVar->u32OutToggle = USBD->EPSTS0 & USBD_EPSTS0_EPSTS7_Msk;
        /* Set a flag to indicate bulk out ready */
        tVCOM[IDX_VCOM2].psCtrlVar->i8BulkOutReady = 1;
    }
}

#endif
#if (VCOM_CNT>=4)
void EP8_Handler(void)
{
    tVCOM[IDX_VCOM3].psCtrlVar->u32TxSize = 0;
}

void EP9_Handler(void)
{
    /* Bulk OUT */
    if (tVCOM[IDX_VCOM3].psCtrlVar->u32OutToggle == (USBD->EPSTS1 & USBD_EPSTS1_EPSTS9_Msk))
    {
        USBD_SET_PAYLOAD_LEN(EP9, EP9_MAX_PKT_SIZE);
    }
    else
    {
        tVCOM[IDX_VCOM3].psCtrlVar->u32RxSize = USBD_GET_PAYLOAD_LEN(EP9);
        tVCOM[IDX_VCOM3].psCtrlVar->pu8RxBuf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP9));

        tVCOM[IDX_VCOM3].psCtrlVar->u32OutToggle = USBD->EPSTS1 & USBD_EPSTS1_EPSTS9_Msk;
        /* Set a flag to indicate bulk out ready */
        tVCOM[IDX_VCOM3].psCtrlVar->i8BulkOutReady = 1;
    }
}

#endif
#if (VCOM_CNT>=5)
void EP10_Handler(void)
{
    tVCOM[IDX_VCOM4].psCtrlVar->u32TxSize = 0;
}

void EP11_Handler(void)
{
    /* Bulk OUT */
    if (tVCOM[IDX_VCOM4].psCtrlVar->u32OutToggle == (USBD->EPSTS1 & USBD_EPSTS1_EPSTS11_Msk))
    {
        USBD_SET_PAYLOAD_LEN(EP11, EP11_MAX_PKT_SIZE);
    }
    else
    {
        tVCOM[IDX_VCOM4].psCtrlVar->u32RxSize = USBD_GET_PAYLOAD_LEN(EP11);
        tVCOM[IDX_VCOM4].psCtrlVar->pu8RxBuf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP11));

        tVCOM[IDX_VCOM4].psCtrlVar->u32OutToggle = USBD->EPSTS1 & USBD_EPSTS1_EPSTS11_Msk;
        /* Set a flag to indicate bulk out ready */
        tVCOM[IDX_VCOM4].psCtrlVar->i8BulkOutReady = 1;
    }
}

#endif
#if (VCOM_CNT>=6)
void EP12_Handler(void)
{
    tVCOM[IDX_VCOM5].psCtrlVar->u32TxSize = 0;
}

void EP13_Handler(void)
{
    /* Bulk OUT */
    if (tVCOM[IDX_VCOM5].psCtrlVar->u32OutToggle == (USBD->EPSTS1 & USBD_EPSTS1_EPSTS13_Msk))
    {
        USBD_SET_PAYLOAD_LEN(EP13, EP13_MAX_PKT_SIZE);
    }
    else
    {
        tVCOM[IDX_VCOM5].psCtrlVar->u32RxSize = USBD_GET_PAYLOAD_LEN(EP13);
        tVCOM[IDX_VCOM5].psCtrlVar->pu8RxBuf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP13));

        tVCOM[IDX_VCOM5].psCtrlVar->u32OutToggle = USBD->EPSTS1 & USBD_EPSTS1_EPSTS13_Msk;
        /* Set a flag to indicate bulk out ready */
        tVCOM[IDX_VCOM5].psCtrlVar->i8BulkOutReady = 1;
    }
}

#endif
#if (VCOM_CNT>=7)
void EP14_Handler(void)
{
    tVCOM[IDX_VCOM6].psCtrlVar->u32TxSize = 0;
}

void EP15_Handler(void)
{
    /* Bulk OUT */
    if (tVCOM[IDX_VCOM6].psCtrlVar->u32OutToggle == (USBD->EPSTS1 & USBD_EPSTS1_EPSTS15_Msk))
    {
        USBD_SET_PAYLOAD_LEN(EP15, EP15_MAX_PKT_SIZE);
    }
    else
    {
        tVCOM[IDX_VCOM6].psCtrlVar->u32RxSize = USBD_GET_PAYLOAD_LEN(EP15);
        tVCOM[IDX_VCOM6].psCtrlVar->pu8RxBuf = (uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP15));

        tVCOM[IDX_VCOM6].psCtrlVar->u32OutToggle = USBD->EPSTS1 & USBD_EPSTS1_EPSTS15_Msk;
        /* Set a flag to indicate bulk out ready */
        tVCOM[IDX_VCOM6].psCtrlVar->i8BulkOutReady = 1;
    }
}

#endif


/*--------------------------------------------------------------------------*/
/**
  * @brief  USBD Endpoint Config.
  * @param  None.
  * @retval None.
  */
void VCOM_Init(void)
{
    /* Init setup packet buffer */
    /* Buffer for setup packet */
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


#if (VCOM_CNT>=1)
    /*****************************************************/
    /* EP2 ==> Bulk IN endpoint, address 1 */
    USBD_CONFIG_EP(EP2, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM_0);

    /* Buffer offset for EP2 */
    USBD_SET_EP_BUF_ADDR(EP2, EP2_BUF_BASE);

    /* EP3 ==> Bulk OUT endpoint, address 1 */
    USBD_CONFIG_EP(EP3, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM_0);

    /* Buffer offset for EP3 */
    USBD_SET_EP_BUF_ADDR(EP3, EP3_BUF_BASE);

    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);

#endif
#if (VCOM_CNT>=2)
    /*****************************************************/
    /* EP4 ==> Bulk IN endpoint, address 2 */
    USBD_CONFIG_EP(EP4, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM_1);

    /* Buffer offset for EP4 */
    USBD_SET_EP_BUF_ADDR(EP4, EP4_BUF_BASE);

    /* EP5 ==> Bulk OUT endpoint, address 2 */
    USBD_CONFIG_EP(EP5, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM_1);

    /* Buffer offset for EP5 */
    USBD_SET_EP_BUF_ADDR(EP5, EP5_BUF_BASE);

    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP5, EP5_MAX_PKT_SIZE);
#endif
#if (VCOM_CNT>=3)
    /*****************************************************/
    /* EP6 ==> Bulk IN endpoint, address 3 */
    USBD_CONFIG_EP(EP6, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM_2);

    /* Buffer offset for EP6 */
    USBD_SET_EP_BUF_ADDR(EP6, EP6_BUF_BASE);

    /* EP7 ==> Bulk OUT endpoint, address 3 */
    USBD_CONFIG_EP(EP7, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM_2);

    /* Buffer offset for EP7 */
    USBD_SET_EP_BUF_ADDR(EP7, EP7_BUF_BASE);

    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP7, EP7_MAX_PKT_SIZE);
#endif
#if (VCOM_CNT>=4)
    /*****************************************************/
    /* EP8 ==> Bulk IN endpoint, address 4 */
    USBD_CONFIG_EP(EP8, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM_3);

    /* Buffer offset for EP8 */
    USBD_SET_EP_BUF_ADDR(EP8, EP8_BUF_BASE);

    /* EP9 ==> Bulk OUT endpoint, address 4 */
    USBD_CONFIG_EP(EP9, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM_3);

    /* Buffer offset for EP9 */
    USBD_SET_EP_BUF_ADDR(EP9, EP9_BUF_BASE);

    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP9, EP9_MAX_PKT_SIZE);
#endif
#if (VCOM_CNT>=5)
    /*****************************************************/
    /* EP10 ==> Bulk IN endpoint, address 5 */
    USBD_CONFIG_EP(EP10, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM_4);

    /* Buffer offset for EP10 */
    USBD_SET_EP_BUF_ADDR(EP10, EP10_BUF_BASE);

    /* EP11 ==> Bulk OUT endpoint, address 5 */
    USBD_CONFIG_EP(EP11, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM_4);

    /* Buffer offset for EP11 */
    USBD_SET_EP_BUF_ADDR(EP11, EP11_BUF_BASE);

    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP11, EP11_MAX_PKT_SIZE);
#endif
#if (VCOM_CNT>=6)
    /*****************************************************/
    /* EP12 ==> Bulk IN endpoint, address 6 */
    USBD_CONFIG_EP(EP12, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM_5);

    /* Buffer offset for EP12 */
    USBD_SET_EP_BUF_ADDR(EP12, EP12_BUF_BASE);

    /* EP13 ==> Bulk OUT endpoint, address 6 */
    USBD_CONFIG_EP(EP13, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM_5);

    /* Buffer offset for EP13 */
    USBD_SET_EP_BUF_ADDR(EP13, EP13_BUF_BASE);

    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP13, EP13_MAX_PKT_SIZE);
#endif
#if (VCOM_CNT>=7)
    /*****************************************************/
    /* EP14 ==> Bulk IN endpoint, address 7 */
    USBD_CONFIG_EP(EP14, USBD_CFG_EPMODE_IN | BULK_IN_EP_NUM_6);

    /* Buffer offset for EP14 */
    USBD_SET_EP_BUF_ADDR(EP14, EP14_BUF_BASE);

    /* EP15 ==> Bulk OUT endpoint, address 7 */
    USBD_CONFIG_EP(EP15, USBD_CFG_EPMODE_OUT | BULK_OUT_EP_NUM_6);

    /* Buffer offset for EP15 */
    USBD_SET_EP_BUF_ADDR(EP15, EP15_BUF_BASE);

    /* trigger receive OUT data */
    USBD_SET_PAYLOAD_LEN(EP15, EP15_MAX_PKT_SIZE);
#endif

#if ENABLE_COMMON_INTERRUPT_EP
    /* EP16 ==> Interrupt IN endpoint, address COMMON_INT_EP_NUM */
    USBD_CONFIG_EP(EP16, USBD_CFG_EPMODE_IN | COMMON_INT_EP_NUM);
    /* Buffer offset for EP16 */
    USBD_SET_EP_BUF_ADDR(EP16, COMMON_INT_EP_BASE);
#endif

}


void VCOM_ClassRequest(void)
{

    USBD_GetSetupPacket(g_setup_t.au8setup);

    if (g_setup_t.setup.bmRequestType & 0x80)   /* request data transfer direction */
    {
        // Device to host
        switch (g_setup_t.setup.bRequest)
        {
            case GET_LINE_CODE:
            {
                USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP0)), (uint8_t *)&tVCOM[g_setup_t.setup.wIndex / 2].psCtrlVar->sLineCoding, 7);

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
                USBD_SetStall(EP0);
                USBD_SetStall(EP1);
                break;
            }
        }
    }
    else
    {
        // Host to device
        switch (g_setup_t.setup.bRequest)
        {
            case SET_CONTROL_LINE_STATE:
            {
                tVCOM[g_setup_t.setup.wIndex / 2].psCtrlVar->u16CtrlSignal = (tVCOM[g_setup_t.setup.wIndex / 2].psCtrlVar->u16CtrlSignal << 8) | g_setup_t.setup.wValue;

                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);
                break;
            }

            case SET_LINE_CODE:
            {
                USBD_PrepareCtrlOut((uint8_t *)&tVCOM[g_setup_t.setup.wIndex / 2].psCtrlVar->sLineCoding, 7);

                /* Status stage */
                USBD_SET_DATA1(EP0);
                USBD_SET_PAYLOAD_LEN(EP0, 0);

                break;
            }

            default:
            {
                // Stall
                /* Setup error, stall the device */
                USBD_SetStall(EP0);
                USBD_SetStall(EP1);
                break;
            }
        }
    }
}

void UUART_SetLine_Config_Wapper(UUART_T *psUUART, uint32_t u32Baudrate, uint32_t u32DataWidth, uint32_t u32Parity, uint32_t u32StopBits)
{
    UUART_SetLine_Config(psUUART, u32Baudrate, u32DataWidth, u32Parity, u32StopBits);
}

void VCOM_LineCoding(uint8_t port)
{
    uint32_t u32DataWidth;
    uint32_t u32Parity;
    uint32_t u32StopBits;
    SetlineCoding_t pfSetLineCoding;
    UART_TYPE_T eUARTtype;

    if (tVCOM[port].psPeripheralInfo->UART_BASE == NULL)
    {
        return;
    }

#if (VCOM_CNT >= 6)

    if (IDX_VCOM5 == port)
    {
        pfSetLineCoding = (SetlineCoding_t)UUART_SetLine_Config_Wapper;
        // Reset hardware FIFO
        ((UUART_T *)tVCOM[port].psPeripheralInfo->UART_BASE)->BUFCTL = UUART_BUFCTL_RXRST_Msk | UUART_BUFCTL_TXRST_Msk;
        eUARTtype = eIsUUART;
    }
    else
#endif

    {
        pfSetLineCoding = (SetlineCoding_t)UART_SetLine_Config;
        // Reset hardware FIFO
        ((UART_T *)tVCOM[port].psPeripheralInfo->UART_BASE)->FIFO = UART_FIFO_RXRST_Msk | UART_FIFO_TXRST_Msk;
        eUARTtype = eIsUART;
    }

    NVIC_DisableIRQ(tVCOM[port].psPeripheralInfo->eIRQn);

    // Reset software FIFO
    tVCOM[port].psCtrlVar->u16ComRbytes = 0;
    tVCOM[port].psCtrlVar->u16ComRhead = 0;
    tVCOM[port].psCtrlVar->u16ComRtail = 0;

    tVCOM[port].psCtrlVar->u16ComTbytes = 0;
    tVCOM[port].psCtrlVar->u16ComThead = 0;
    tVCOM[port].psCtrlVar->u16ComTtail = 0;


    switch (tVCOM[port].psCtrlVar->sLineCoding.u8DataBits)
    {
        case 5:
            u32DataWidth = (eUARTtype == eIsUART) ? UART_WORD_LEN_5 : UUART_WORD_LEN_8; //UUART not support 5-bit length
            break;

        case 6:
            u32DataWidth = (eUARTtype == eIsUART) ? UART_WORD_LEN_6 : UUART_WORD_LEN_6;
            break;

        case 7:
            u32DataWidth = (eUARTtype == eIsUART) ? UART_WORD_LEN_7 : UUART_WORD_LEN_7;
            break;

        case 8:
            u32DataWidth = (eUARTtype == eIsUART) ? UART_WORD_LEN_8 : UUART_WORD_LEN_8;
            break;

        default:
            u32DataWidth = (eUARTtype == eIsUART) ? UART_WORD_LEN_8 : UUART_WORD_LEN_8;
            break;
    }

    switch (tVCOM[port].psCtrlVar->sLineCoding.u8CharFormat)
    {
        case 0:
            u32StopBits = (eUARTtype == eIsUART) ? UART_STOP_BIT_1 : UUART_STOP_BIT_1; // 1 stop bits
            break;

        case 1:
            u32StopBits = (eUARTtype == eIsUART) ? UART_STOP_BIT_1_5 : UUART_STOP_BIT_2; //1.5 bits
            break;

        case 2:
            u32StopBits = (eUARTtype == eIsUART) ? UART_STOP_BIT_2 : UUART_STOP_BIT_2; // 2 bits
            break;

        default:
            u32StopBits = (eUARTtype == eIsUART) ? UART_STOP_BIT_1 : UUART_STOP_BIT_1; // 1 stop bits
    }

    switch (tVCOM[port].psCtrlVar->sLineCoding.u8ParityType)
    {
        case 0:
            u32Parity = (eUARTtype == eIsUART) ? UART_PARITY_NONE : UUART_PARITY_NONE; // none parity
            break;

        case 1:
            u32Parity = (eUARTtype == eIsUART) ? UART_PARITY_ODD : UUART_PARITY_ODD; // odd parity
            break;

        case 2:
            u32Parity = (eUARTtype == eIsUART) ? UART_PARITY_EVEN : UUART_PARITY_EVEN; // even parity
            break;

        default:
            u32Parity = (eUARTtype == eIsUART) ? UART_PARITY_NONE : UUART_PARITY_NONE; // none parity
    }

    pfSetLineCoding(tVCOM[port].psPeripheralInfo->UART_BASE,
                    tVCOM[port].psCtrlVar->sLineCoding.u32DTERate,
                    u32DataWidth,
                    u32Parity,
                    u32StopBits
                   );

    // Re-enable UART interrupt
    NVIC_EnableIRQ(tVCOM[port].psPeripheralInfo->eIRQn);

}







