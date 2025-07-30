/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use an I3C Target device to reveive and transmit the data from a Controller.
 *           This sample code can be used with the I3C_ControllerWR sample code or other Controller devices.
 *           Users can set the transfer mode or enable DMA function for data transfer.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/
#define DA_ASSIGNED_MODE        (0)

/*
// <o> Select Target Transfer Mode
//      <0=> SDR Transfer ... byte length from 1 to 64.
//      <1=> HDR-DDR Transfer ... byte length aligned to byte-pairs (2-bytes).
//      <2=> HDR-BT Transfer ... byte length aligned to byte-pairs (2-bytes).
// <i> Select Target transfer mode for Controller Read Operation.
*/
#define TARGET_TRANSFER_MODE    (0)

// <c1> Enable PDMA Transfer
// <i> Enable PDMA mode to transfer Target data.
#define DEVICE_DMA_ENABLED
//</c>

// <c1> Enable Interrupt Debug Log
// <i> Show detail interrupt status on UART.
#define DGBINT          printf
//</c>

#ifndef DGBINT
#define DGBINT(...)
#endif
// *** <<< end of configuration section >>>    ***


/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART_Init(void);
void I3C0_IRQHandler(void);
void I3C_Init(I3C_DEVICE_T *dev);
void I3C_IRQFunc(I3C_DEVICE_T *dev);

#define I3C0_SA             (0x68)
#define I3C0_MID            (0x8123UL)
#define I3C0_PID            (0xA13573C0UL)
#define I3C0_VT1_SA         (0x78)
#define I3C0_VT1_MID        (I3C0_MID + 1)
#define I3C0_VT1_PID        (I3C0_PID + 1)

volatile uint32_t   g_I3CDevRx[I3C_DEVICE_RX_BUF_CNT], g_I3CDevTx[I3C_DEVICE_TX_BUF_CNT];


I3C_DEVICE_T g_I3CDev = 
{
    .port = (I3C_T *)I3C0,
    
    .device_role = I3C_TARGET,    
    .main_target_sa = I3C0_SA,

    .target_index = 0,
    .target_count = 2,
    
    .cmd_response = I3C_CTRRESP_INITIAL_VALUE,
    
    .is_HDR_cmd  = FALSE,
    .is_last_cmd = TRUE,
    
    .is_DB = FALSE, 
    
    .tx_id  = I3C_TX_TID,
    .tx_len = 0,
    .tx_buf = (uint8_t *)g_I3CDevTx,
    
    .rx_len = 0,
    .rx_buf = (uint8_t *)g_I3CDevRx,
            
    .tgtRespQ[0].ErrSts = (uint8_t)I3C_TGTRESP_INITIAL_VALUE,
    
#if defined(DEVICE_DMA_ENABLED)
    .is_DMA = TRUE,
    .RxDMACh = 0,
    .TxDMACh = 1,
#endif    
};

void I3C_Init(I3C_DEVICE_T *dev)
{
    uint32_t i3c_engclk;

    /* Set multi-function pins for I3C pin */
    GPIO_ENABLE_SCHMITT_TRIGGER(PA, (BIT0 | BIT1));
    SET_I3C0_SDA_PA0();
    SET_I3C0_SCL_PA1();
    
    /* Enable I3C module clock */
    CLK_EnableModuleClock(I3C0_MODULE);
    
    /* Select I3C module clock source */
    CLK_SetModuleClock(I3C0_MODULE, CLK_CLKSEL3_I3C0SEL_PCLK0, 0);
    i3c_engclk = CLK_GetPCLK0Freq();
    
    SYS_ResetModule(I3C0_RST);
    
    /* Enable DMA before setting the I3C ENABLE bit */
    if(dev->is_DMA)
    {
        /* Enable PDMA0 module clock */
        CLK_EnableModuleClock(PDMA0_MODULE);
        
        /* Enable PDMA channel for I3C Rx function */
        I3C_ConfigRxDMA(dev, (uint32_t)(&dev->port->TXRXDAT), (uint32_t)(dev->rx_buf), (I3C_DEVICE_RX_BUF_CNT * 4));
        
        /* Enable I3C DMA function */
        I3C_EnableDMA(dev->port);
    }

    dev->engclk = i3c_engclk;
    dev->irq_enable = TRUE;
    if(DA_ASSIGNED_MODE == 1)
        dev->target_daa_mode = I3C_SUPPORT_IMMEDIATE_HJ;
    else if(DA_ASSIGNED_MODE == 2)
        dev->target_daa_mode = I3C_SUPPORT_ADAPTIVE_HJ;
    else
        dev->target_daa_mode = I3C_SUPPORT_ENTDAA;
    if(I3C_DeviceInit(dev) != I3C_STS_NO_ERR)
    {
        printf("\n# ERROR - I3C_DeviceInit\n\n"); while(1) {}
    }
    
    /* Configure MID and PID for Main Target */
    dev->port->SLVMID = I3C0_MID;
    dev->port->SLVPID = I3C0_PID;
    
    /* Enable Virtual Target 1 and confogure MID and PID */
    dev->port->VTGTCFG[0].ADDR = (I3C_VTGTADDR_ENABLE_Msk | (I3C_VTGTADDR_SAVALID_Msk | (I3C0_VT1_SA << I3C_VTGTADDR_SADDR_Pos)));
    dev->port->VTGTCFG[0].MID = I3C0_VT1_MID;
    dev->port->VTGTCFG[0].PID = I3C0_VT1_PID;
    
    dev->intsts = dev->port->INTSTS;
    if(dev->irq_enable)
        NVIC_EnableIRQ(I3C0_IRQn);
    else
        NVIC_DisableIRQ(I3C0_IRQn);       

    /* Enable device */
    dev->port->DEVCTL |= I3C_DEVCTL_ENABLE_Msk;
}

void I3C_IRQFunc(I3C_DEVICE_T *dev)
{
    volatile uint32_t int_sts;
    
    int_sts = dev->port->INTSTS;
            
    if(int_sts & I3C_INTSTS_RESPQ_READY)
    {
        /* Read-only */
        /* Read RESPQUE to clear I3C_INTSTS_RESPQ_READY */        
        DGBINT("[ INT ] RESPQ_READY\n");
        
        if(dev->device_role == I3C_CONTROLLER)
        {
            dev->cmd_response = dev->port->RESPQUE;
        }
        else
        {
            /* Get Target's response by Controller Write operation */
            I3C_TgtRecv(dev);
        }
    }

    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /* For Controller role */
    if(int_sts & I3C_INTSTS_IBI_RECEIVED)
    {
        /* Read IBISTS to clear I3C_INTSTS_IBI_RECEIVED */
        dev->ibi_status = dev->port->IBISTS;
        dev->intsts     = I3C_INTSTS_IBI_RECEIVED;
        DGBINT("[ INT ] IBI_RECEIVED\n");
        
        /* Parse IBI queue for MR, In-Band interrupt or Hot-Join event */
        I3C_CtrGetIBI(dev);
    }
    if(int_sts & I3C_INTSTS_TRANSFER_ABORT)
    {
        dev->port->INTSTS = I3C_INTSTS_TRANSFER_ABORT;
        DGBINT("[ INT ] TRANSFER_ABORT\n");
    }
    if(int_sts & I3C_INTSTS_BUSRST_DONE)
    {
        dev->port->INTSTS = I3C_INTSTS_BUSRST_DONE;
        DGBINT("[ INT ] BUSRST_DONE\n");
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
    
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /* For Target role */
    if(int_sts & I3C_INTSTS_CCC_UPDATED)
    {
        dev->port->INTSTS = I3C_INTSTS_CCC_UPDATED;
        dev->intsts       = I3C_INTSTS_CCC_UPDATED;
        DGBINT("[ INT ] CCC_UPDATED\n");
        
        if( !((int_sts & I3C_INTSTS_DA_ASSIGNED) || (int_sts & I3C_INTSTS_GRPDA_ASSIGNED)) )
        {
            /* Process CCC updated events */
            I3C_TgtHandleIntSts(dev);
        }
    }
    if(int_sts & I3C_INTSTS_DA_ASSIGNED)
    {
        dev->port->INTSTS = I3C_INTSTS_DA_ASSIGNED;
        dev->intsts       = I3C_INTSTS_DA_ASSIGNED;
        DGBINT("[ INT ] DA_ASSIGNED\n");
        
        /* Dump Target's Main and Vitrual address */
        I3C_TgtHandleIntSts(dev);
    }
    if(int_sts & I3C_INTSTS_GRPDA_ASSIGNED)
    {
        dev->port->INTSTS = I3C_INTSTS_GRPDA_ASSIGNED;
        dev->intsts       = I3C_INTSTS_GRPDA_ASSIGNED;
        DGBINT("[ INT ] GRPDA_ASSIGNED\n");
        
        /* Dump Target's Group address */
        I3C_TgtHandleIntSts(dev);
    }
    if(int_sts & I3C_INTSTS_DEFTGTS)
    {
        dev->port->INTSTS = I3C_INTSTS_DEFTGTS;
        DGBINT("[ INT ] DEFTGTS\n");
    }
    if(int_sts & I3C_INTSTS_READ_REQUEST)
    {
        dev->port->INTSTS = I3C_INTSTS_READ_REQUEST;
        DGBINT("[ INT ] READ_REQUEST\n");
    }
    if(int_sts & I3C_INTSTS_IBI_UPDATED)
    {
        dev->port->INTSTS = I3C_INTSTS_IBI_UPDATED;
        dev->intsts       = I3C_INTSTS_IBI_UPDATED;
        DGBINT("[ INT ] IBI_UPDATED\n");
    }
    if(int_sts & I3C_INTSTS_START_DETECTED)
    {
        dev->port->INTSTS = I3C_INTSTS_START_DETECTED;
        //DGBINT("[ INT ] START_DETECTED\n");
    }
    if(int_sts & I3C_INTSTS_TGTRST_DETECTED)
    {
        dev->port->INTSTS = I3C_INTSTS_TGTRST_DETECTED;
        DGBINT("[ INT ] TGTRST_DETECTED\n");
    }
    if(int_sts & I3C_INTSTS_SDA_RELEASED)
    {
        dev->port->INTSTS = I3C_INTSTS_SDA_RELEASED;
        DGBINT("[ INT ] SDA_RELEASED\n");
    }
    if(int_sts & I3C_INTSTS_EXTCMD_FINISHED)
    {
        /* Read-only */
        /* Read EXTCMDVLD to clear I3C_INTSTS_EXTCMD_FINISHED */
        DGBINT("[ INT ] EXTCMD_FINISHED\n");        
        
        /* Get Target Extended Transfer Command result by Controller Read operation*/
        I3C_TgtGetSendResult(dev);
    }
    if(int_sts & I3C_INTSTS_EXTCMD_TX_EMPTY_THLD)
    {
        /* Read-only */
        //DGBINT("[ INT ] EXTCMD_TX_EMPTY_THLD\n");        
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
    
    if(int_sts & I3C_INTSTS_TX_EMPTY_THLD)
    {
        /* Read-only */
        //DGBINT("[ INT ] TX_EMPTY_THLD\n");
    }
    if(int_sts & I3C_INTSTS_RX_THLD)
    {
        /* Read-only */
        //DGBINT("[ INT ] RX_THLD\n");
    }
    if(int_sts & I3C_INTSTS_CMDQ_EMPTY_THLD)
    {
        /* Read-only */
        //DGBINT("[ INT ] CMDQ_EMPTY_THLD\n");
    }
    if(int_sts & I3C_INTSTS_BUSOWNER_UPDATED)
    {
        dev->port->INTSTS = I3C_INTSTS_BUSOWNER_UPDATED;
        DGBINT("[ INT ] BUSOWNER_UPDATED\n");
       
        dev->port->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
        while((dev->port->DEVCTL & I3C_DEVCTL_RESUME_Msk) == I3C_DEVCTL_RESUME_Msk) {}
            
        if(I3C_IS_MASTER(dev->port))
            DGBINT("\t[ INT ] Device changes to Controller Role\n");       
        else    
            DGBINT("\t[ INT ] Device changes to Target Role\n");       
    }
    if(int_sts & I3C_INTSTS_TRANSFER_ERR)
    {
        dev->port->INTSTS = I3C_INTSTS_TRANSFER_ERR;
        DGBINT("[ INT ] TRANSFER_ERR\n");
       
        /* Parse error code and recovery to idle state */
        if(dev->device_role == I3C_CONTROLLER)
            I3C_CtrHandleTransErr(dev);
        else
            I3C_TgtHandleTransErr(dev);
    }
               
    DGBINT("[ INT ] status: 0x%08x -> 0x%08x\n\n", int_sts, dev->port->INTSTS);
       
    if(int_sts & I3C_INTSTS_RESPQ_READY)
    {
        if(dev->device_role == I3C_CONTROLLER)
        {
            if((dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk) == I3C_CTRRESP_NO_ERR)
            {
                dev->tx_id  = ((dev->cmd_response&I3C_CTRRESP_TID_Msk) >> I3C_CTRRESP_TID_Pos);
                dev->rx_len = ((dev->cmd_response&I3C_CTRRESP_DATLEN_Msk) >> I3C_CTRRESP_DATLEN_Pos);
                DGBINT("[ INT ] #CTR Resp:\n\tTID %d, Len %d\n\n", dev->tx_id, dev->rx_len);  
            }
        }
    }          
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    volatile uint32_t i;
    uint8_t RespQSts, TID, RxLen, IsCCCWrite, CmdSize;
    uint8_t *pu8Data;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART_Init();
    
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------------------+\n");
    printf("|    I3C Target Read/Write Sample Code          |\n");
    printf("|    # Users can set the transfer mode or       |\n");
    printf("|      enable DMA function for data transfer    |\n");
    printf("+-----------------------------------------------+\n");
#if defined(DEVICE_DMA_ENABLED)
    printf("\t[ PDMA enabled ]\n");
#endif    
    if(TARGET_TRANSFER_MODE == 0)
        printf("\t[ Use SDR transfer ]\n");
    if(TARGET_TRANSFER_MODE == 1)
        printf("\t[ Use HDR-DDR transfer ]\n");
    if(TARGET_TRANSFER_MODE == 2)
        printf("\t[ Use HDR-BT transfer ]\n");
        
    printf("\n");
    if(DA_ASSIGNED_MODE == 1)
        printf("[ Initiate a Hot-Join request immediately after I3C Target enabled ]\n\n");
    else if(DA_ASSIGNED_MODE == 2)
        printf("[ Initiate a Hot-Join request when a 7'h7E header on the bus ]\n\n");
    else
        printf("[ Wait Controller to send ENTDAA CCC ]\n\n");

    /* Initializes the I3C device */
    I3C_Init(&g_I3CDev);
    
    printf("# I3C Target settings:\n");
    printf("    - SDA on PA.0\n");
    printf("    - SCL on PA.1\n");
    printf("    - I2C Static Address 0x%02x\n", g_I3CDev.main_target_sa);
    printf("    - The first operation of the I3C enable bit requires at least bus SCLx4 to become active\n");
    printf("# An I3C Controller can write N-bytes data to Target,\n");
    printf("  then perform a read request to receive the N-bytes data from Target.\n");
    printf("    - The write data should be equal to the received data\n");
    printf("\n");

    while(1)
    {
        if(g_I3CDev.intsts != 0)
        {
            if(g_I3CDev.intsts & I3C_INTSTS_DA_ASSIGNED)
            {
                if(g_I3CDev.target_da[0] != 0x0)
                    printf("Target-0 DA: 0x%02x (Main Target)\n", g_I3CDev.target_da[0]);
                else
                    printf("Target-0 SA: 0x%02x (Main Target)\n", g_I3CDev.target_sa[0]);
                if(g_I3CDev.target_da[1] != 0x0)
                    printf("Target-1 DA: 0x%02x (Virtual Target)\n", g_I3CDev.target_da[1]);
                else
                    printf("Target-1 SA: 0x%02x (Virtual Target)\n", g_I3CDev.target_sa[1]);
                printf("\n");
            }            
        }
        g_I3CDev.intsts = 0;
        
        RespQSts = g_I3CDev.tgtRespQ[0].ErrSts; /* Parse from I3C_TgtRecv(...) */
        
        if(RespQSts != (uint8_t)I3C_TGTRESP_INITIAL_VALUE)
        {
            g_I3CDev.tgtRespQ[0].ErrSts = (uint8_t)I3C_TGTRESP_INITIAL_VALUE;
            
            TID        = g_I3CDev.tgtRespQ[0].TargetID;
            RxLen      = g_I3CDev.tgtRespQ[0].RxBufLen;
            IsCCCWrite = g_I3CDev.tgtRespQ[0].IsCCCWrite;
            CmdSize    = g_I3CDev.tgtRespQ[0].CmdSize;
            pu8Data    = (uint8_t *)(g_I3CDev.tgtRespQ[0].RxBufAddr);
            if(RespQSts == I3C_STS_NO_ERR)
            {
                printf("*** [ Resp for %s Write - ", (IsCCCWrite == 1)? "CCC":"Private");
                for(i=0; i<8; i++)
                {
                    if(TID == (1<<i))
                    {
                        TID = i; 
                        if(TID == 0)
                            printf("Main Target ] ***\n");
                        else 
                            printf("Virtual Target %d ] ***\n", i);
                        break;
                    }
                }
                if(i == 8)
                {
                    printf("BroadCast ] *** (%d)\n", TID);
                }
                                    
                if(IsCCCWrite == 1)
                {
                    if(CmdSize == 1)
                        printf("Read CCC/CMD code: 0x%x.\n", g_I3CDev.tgtRespQ[0].CmdWord);

                    if(CmdSize == 2)
                        printf("Read CCC/CMD code: 0x%x and Defining byte: 0x%x.\n", g_I3CDev.tgtRespQ[0].CmdWord, g_I3CDev.rx_buf[1]);
                }
                
                if(RxLen == 0)
                {
                    if(g_I3CDev.is_DMA)
                    {                
                        /* Enable I3C PDMA receive function */
                        I3C_ConfigRxDMA(&g_I3CDev, (uint32_t)(&g_I3CDev.port->TXRXDAT), (uint32_t)(g_I3CDev.rx_buf), (I3C_DEVICE_RX_BUF_CNT * 4));
                    }
                    printf("\n");
                    continue;
                }
                
                printf("Read %d-bytes: \n\t", RxLen);
                pu8Data = (uint8_t *)(g_I3CDev.tgtRespQ[0].RxBufAddr);                   
                for(i=0; i<RxLen; i++)
                {
                    printf("%02x ", pu8Data[i]);
                }
                printf("\n\n");
                
                /* Prepare Target transfer data for Controller Read operation */
                memcpy((uint8_t *)g_I3CDev.tx_buf, pu8Data, RxLen);
                g_I3CDev.target_index  = TID;  // Target index, valid: 0~4
                g_I3CDev.target_extcmd = TID;   // Ext cmd index, valid: 0~7   
                if(TARGET_TRANSFER_MODE == 0) // SDR transfer
                {
                    g_I3CDev.tx_len   = RxLen; // byte count
                    g_I3CDev.ccc_code = 0;
                }
                if(TARGET_TRANSFER_MODE == 1) // HDR-DDR transfer
                {
                    RxLen = (((RxLen+1) / 2) * 2);
                    g_I3CDev.tx_len     = RxLen;
                    g_I3CDev.ccc_code   = I3C_CCC_ENTHDR(0);
                    g_I3CDev.is_HDR_cmd = TRUE;
                    g_I3CDev.HDR_cmd    = 0x80; // HDR0 Command code 0x80~0xFF(NOT CCC code)
                }
                if(TARGET_TRANSFER_MODE == 2) // HDR-BT transfer
                {
                    RxLen = (((RxLen+1) / 2) * 2);
                    g_I3CDev.tx_len       = RxLen;
                    g_I3CDev.ccc_code     = I3C_CCC_ENTHDR(3);
                    g_I3CDev.is_HDRBT_cmd = TRUE;
                    g_I3CDev.HDRBT_cmd    = 0x01010101;
               }
                
                /* Push data to Extend Command Data Buffer */
                I3C_TgtSend(&g_I3CDev);
            }
            else
            {
                printf("\n*** [ Resp error status  0x%x ] ***\n\n", RespQSts);
                
                I3C_TgtHandleTransErr(&g_I3CDev);
            }
            
            if(g_I3CDev.is_DMA)
            {                
                /* Enable I3C PDMA receive function */
                I3C_ConfigRxDMA(&g_I3CDev, (uint32_t)(&g_I3CDev.port->TXRXDAT), (uint32_t)(g_I3CDev.rx_buf), (I3C_DEVICE_RX_BUF_CNT * 4));
            }
        }
    }
}

void SYS_Init(void)
{
     /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 160MHz */
    CLK_SetCoreClock(160000000);
    
    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    
   /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(DEBUG_PORT, 115200);
}

/**
  * @brief  The I3C0 default IRQ, declared in startup_m3331.S.
  */
void I3C0_IRQHandler(void)
{    
    I3C_IRQFunc(&g_I3CDev);
}
