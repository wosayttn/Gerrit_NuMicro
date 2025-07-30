/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use an I3C Controller to transmit and receive the data from a Target.
 *           This sample code can be used with the I3C_TargetRW sample code or other Target devices.
 *           This I3C Controller also supports In-Band Interrupt, Controller Request and Hot-Join Request From I3C Target.
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
void ParseUartInputChar(I3C_DEVICE_T *dev);
uint8_t Process_ADDRCCC(I3C_DEVICE_T *dev, uint32_t ccc);
void    Process_IBIReceived(I3C_DEVICE_T *dev);

#define I3C0_DA             (0x4A)
#define I3C0_MID            (0x8123UL)
#define I3C0_PID            (0xA13573C0UL)

const uint8_t       g_TgtDA[] = {0x18, 0x28, 0x38, 0x48, 0x58, 0x68, 0x78};
volatile uint32_t   g_I3CDevRx[I3C_DEVICE_RX_BUF_CNT], g_I3CDevTx[I3C_DEVICE_TX_BUF_CNT];


I3C_DEVICE_T g_I3CDev = 
{
    .port = (I3C_T *)I3C0,
    
    .device_role = I3C_CONTROLLER,    
    .main_controller_da = I3C0_DA,

    .speed_mode       = I3C_DEVI3C_SPEED_SDR0,
    .i2c_fm_freq      = (400 * 1000),
    .i2c_fm_plus_freq = (1000 * 1000),
    .i3c_sdr_freq     = 1200000, //12500000,

    .target_index = 0,
    .target_count = 7,
    
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
    volatile uint32_t i;
    uint32_t i3c_engclk;
   
    /* Set multi-function pins for I3C pin */
    GPIO_ENABLE_SCHMITT_TRIGGER(PA, (BIT0 | BIT1));
    SET_I3C0_SDA_PA0();
    SET_I3C0_SCL_PA1();
    
    /* Enable I3C module clock */
    CLK_EnableModuleClock(I3C0_MODULE);
    
    /* Select I3C module clock source */
    CLK_SetModuleClock(I3C0_MODULE, CLK_CLKSEL3_I3C0SEL_PCLK0, NULL);
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
    
    dev->intsts = dev->port->INTSTS;
    if(dev->irq_enable)
        NVIC_EnableIRQ(I3C0_IRQn);
    else
        NVIC_DisableIRQ(I3C0_IRQn);       

    /* Enable device */
    dev->port->DEVCTL |= I3C_DEVCTL_ENABLE_Msk;
    
    if(dev->device_role == I3C_CONTROLLER)
    {                
        /* Allocate "Device Address Table for Target" */
        for(i=0; i<7; i++)
        {
            I3C_SetDeviceAddr(dev->port, i, I3C_DEVTYPE_I3C, g_TgtDA[i], 0);
        }
    }    
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
               
    DGBINT("[ INT ] status: 0x%08x -> 0x%08x\n\n", (uint32_t)int_sts, (uint32_t)dev->port->INTSTS);
       
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

uint8_t Process_ADDRCCC(I3C_DEVICE_T *dev, uint32_t ccc)
{
    volatile uint32_t i;
    static uint8_t sTotalAddrCnt = 0xFF;
    
    do{
        if(ccc == I3C_CCC_RSTDAA)
        {
            /* RSTDAA CCC */
            dev->tx_len       = 0;
            dev->is_last_cmd  = TRUE;
            dev->ccc_code     = I3C_CCC_RSTDAA;
            I3C_CtrCCCSet(dev);

            // Waiting for IRQ
            while(dev->cmd_response == I3C_CTRRESP_INITIAL_VALUE) {}
            if((dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk) == I3C_CTRRESP_NO_ERR)
            {
                //printf("\n[ RSTDAA PASS ]\n");
                
                /* Reset valid target count to 0 */
                dev->target_count = 0;
                
                sTotalAddrCnt = 0xFF;
                for(i=0; i<7; i++)
                    dev->target_da[i] = 0x0;
            }
            else
            {
                //printf("\n[ RSTDAA, error code %d ]\n", ((uint32_t)(dev->cmd_response&I3C_CTRRESP_ERRSTS_Msk) >> I3C_CTRRESP_ERRSTS_Pos));
                dev->target_count = (sTotalAddrCnt==0xFF)? 0:sTotalAddrCnt;
            }
            printf("Total I3C Target x%d after RSTDAA CCC\n", dev->target_count);
            printf("\n");
        }
        
        if(ccc == I3C_CCC_ENTDAA)
        {
            if(dev->target_count >= sTotalAddrCnt)
                sTotalAddrCnt = dev->target_count;
            
            /* ENTDAA CCC */           
            dev->target_index = 0;
            dev->target_count = 7; 
            dev->ccc_code     = I3C_CCC_ENTDAA;
            I3C_CtrDAA(dev);
            
            // Waiting for IRQ
            while(dev->cmd_response == I3C_CTRRESP_INITIAL_VALUE) {}
            if((dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk) == I3C_CTRRESP_NO_ERR)
            {
                sTotalAddrCnt = dev->target_count;
                //printf("\n[ ENTDAA no Error ]\n");
            }
            else
            {
                //printf("\n[ ENTDAA, error code %d ]\n", ((uint32_t)(dev->cmd_response&I3C_CTRRESP_ERRSTS_Msk) >> I3C_CTRRESP_ERRSTS_Pos));
                if(dev->target_count != 0)
                    sTotalAddrCnt = dev->target_count;
                else
                    dev->target_count = (sTotalAddrCnt==0xFF)? 0:sTotalAddrCnt;
            }
                            
            printf("Total I3C Target x%d after ENTDAA CCC\n", dev->target_count);

            if(dev->target_count == 0)
            {
                for(i=0; i<7; i++)
                    dev->target_da[i] = 0x0;
                printf("\tNo valid I3C Target on the bus\n");
            }
            else
            {
                for(i=0; i<dev->target_count; i++)
                {
                    printf("\tTarget #%d:\n", i);
                    dev->target_da[i] = ((dev->port->TGTCHAR[i].DADDR & I3C_TGTCHAR4_DADDR_Msk) >> I3C_TGTCHAR4_DADDR_Pos);
                    printf("\t - Provisional ID = 0x%08x%02x \n", dev->port->TGTCHAR[i].PIDMSB, dev->port->TGTCHAR[i].PIDLSB);
                    printf("\t - BCR, DCR       = 0x%08x \n", dev->port->TGTCHAR[i].BCRDCR);
                    printf("\t - DADDR          = 0x%02x \n", dev->target_da[i]);
                }
            }
            
            printf("\n");
        }
    } while(0);
    
    return sTotalAddrCnt;
}

void Process_IBIReceived(I3C_DEVICE_T *dev)
{
    volatile uint32_t i;
    uint8_t *pu8Buf;

    if(dev->ibi_status & (uint32_t)(I3C_IBIQSTS_NACK))
    {
        printf("NACK In-Band Interrupt.\n");
    }
    else
    {
        if(dev->ibi_type == I3C_IBI_TYPE_TIR)
        {
            printf("Process In-Band Interrupt by Target-0x%02x and get %d-bytes.\n\t", dev->ibi_id , dev->ibi_len);
            pu8Buf = (uint8_t *)&dev->ibi_payload;
            for(i=0; i<dev->ibi_len; i++)
                printf("0x%02x ", pu8Buf[i]);
        }
        
        if(dev->ibi_type == I3C_IBI_TYPE_CR)
        {
            printf("Process Controller Request by Target-0x%02x, and Controller active in %s role.\n", 
                dev->ibi_id , I3C_IS_MASTER(dev->port)?"Controller":"Target");
        }
        
        if(dev->ibi_type == I3C_IBI_TYPE_HJ)
        {
            printf("Process Hot-Join request ... %s.\n", (dev->ibi_id==0x2)? "PASS":"FAIL");
            printf("Total I3C Target x%d after Hot-Join reguest:\n", dev->target_count);
            for(i=0; i<dev->target_count; i++)
            {
                printf("\tTarget #%d:\n", i);
                dev->target_da[i] = ((dev->port->TGTCHAR[i].DADDR & I3C_TGTCHAR4_DADDR_Msk) >> I3C_TGTCHAR4_DADDR_Pos);
                printf("\t - Provisional ID = 0x%08x%02x \n", dev->port->TGTCHAR[i].PIDMSB, dev->port->TGTCHAR[i].PIDLSB);
                printf("\t - BCR, DCR       = 0x%08x \n", dev->port->TGTCHAR[i].BCRDCR);
                printf("\t - DADDR          = 0x%02x \n", dev->target_da[i]);
            }
        }
    }
    
    printf("\n");
}

/**
  * @brief  Polling a specified char for Controller operation.
  */
void ParseUartInputChar(I3C_DEVICE_T *dev)
{
    char mode;
    volatile uint32_t i;
    uint32_t idx, len, *pu32Buf, buf_cnt;      
    
    if((DEBUG_PORT->FIFOSTS&UART_FIFOSTS_RXEMPTY_Msk) == 0U)
    {
        mode = (char)DEBUG_PORT->DAT;
        do {
            if(mode == 'i')
            {
                printf("\n");
                printf("[R] RSTDAA CCC\n");
                printf("[E] ENTDAA CCC\n");
                printf("[w] Write 16-bytes data to Target-0 by SDR mode\n");
                printf("[r] Read 16-bytes data from Target-0 by SDR mode\n");
                printf("[I] Dump all Target's info\n");
                printf("\n");
                break;
            }
            
            if(mode == 'R')
            {
                Process_ADDRCCC(dev, I3C_CCC_RSTDAA);
            }

            if(mode == 'E')
            {
                Process_ADDRCCC(dev, I3C_CCC_ENTDAA);
            }
            
            if(mode == 'w')
            {
                static uint8_t sVal = 0;
                
                idx = 0;  // Target-0
                len = 16; // TX lenght 16-bytes
                
                // prepare TX data
                for(i=0; i<len; i++)
                    dev->tx_buf[i] = ((0x10 + i) + sVal);
                
                dev->target_index = idx;
                dev->tx_len       = len;
                dev->is_last_cmd  = TRUE;
                I3C_CtrWrite(dev);
                if(dev->irq_enable)
                {
                    // Process in IRQ
                    while(dev->cmd_response == I3C_CTRRESP_INITIAL_VALUE) {}
                    if((dev->cmd_response&I3C_CTRRESP_ERRSTS_Msk) != I3C_CTRRESP_NO_ERR)
                    {
                        printf("\n[ Write error occurred, error code %d ]\n", ((uint32_t)(dev->cmd_response&I3C_CTRRESP_ERRSTS_Msk) >> I3C_CTRRESP_ERRSTS_Pos)); 
                    }
                    else
                    {
                        printf("\n[ Write PASS ]\n\t");
                        
                        for(i=0; i<len; i++)
                        {
                            printf("%02x ", dev->tx_buf[i]);
                            if((i%8) == 7)
                                printf("\n\t");
                        }
                        printf("\n");
                        
                        sVal++;
                    }
                }
                else             
                {
                    while((dev->port->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0) {}
                    printf("\n[ TODO: Users need to manually parse the RespQ status 0x%08x ]\n", dev->port->RESPQUE);
                }
            }        
            
            if(mode == 'r')
            {                
                idx = 0;  // Target-0
                len = 16; // RX lenght 16-bytes
                
                // claer RX data
                for(i=0; i<len; i++)
                    dev->rx_buf[i] = 0x0;
                
                dev->target_index = idx;
                dev->rx_len       = len;
                dev->is_last_cmd  = TRUE;
                I3C_CtrRead(dev);
                if(dev->irq_enable)
                {
                    // Process in IRQ
                    while(dev->cmd_response == I3C_CTRRESP_INITIAL_VALUE) {}
                    if((dev->cmd_response&I3C_CTRRESP_ERRSTS_Msk) != I3C_CTRRESP_NO_ERR)
                    {
                        printf("\n[ Read error occurred, error code %d ]\n", ((uint32_t)(dev->cmd_response&I3C_CTRRESP_ERRSTS_Msk) >> I3C_CTRRESP_ERRSTS_Pos)); 
                    }
                    else
                    {
                        printf("\n[ Read PASS ]\n\t");
                        
                        len = dev->rx_len;
                        buf_cnt = (len+ 3) / 4;

                        pu32Buf = (uint32_t *)&dev->rx_buf[0];
                        if(dev->is_DMA)
                        {
                            /* Get data by Rx PDAM */
                        }
                        else
                        {
                            for(i=0; i<buf_cnt; i++)
                                pu32Buf[i] = dev->port->TXRXDAT;
                        }
                        
                        for(i=0; i<len; i++)
                        {
                            printf("%02x ", dev->rx_buf[i]);
                            if((i%8) == 7)
                                printf("\n\t");
                        }
                        printf("\n");
                    }
                    
                    if(dev->is_DMA)
                    {                        
                        /* Enable PDMA channel for I3C Rx function */
                        I3C_ConfigRxDMA(dev, (uint32_t)(&dev->port->TXRXDAT), (uint32_t)(dev->rx_buf), (I3C_DEVICE_RX_BUF_CNT * 4));
                    }
                }
                else             
                {
                    while((dev->port->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0) {}
                    printf("\n[ TODO: Users need to manually parse the RespQ status 0x%08x ]\n", dev->port->RESPQUE);
                }
            }        
            
            if(mode == 'I')
            {
                printf("\nAll Target's info:\n");
                if(dev->target_da[0] == 0x0)
                {
                    printf("\tNo valid I3C Target on the bus\n\n");
                    break;
                }
                
                for(i=0; i<dev->target_count; i++)
                {
                    printf("\tTarget #%d:\n", i);
                    dev->target_da[i] = ((dev->port->TGTCHAR[i].DADDR & I3C_TGTCHAR4_DADDR_Msk) >> I3C_TGTCHAR4_DADDR_Pos);
                    printf("\t - Provisional ID = 0x%08x%02x \n", dev->port->TGTCHAR[i].PIDMSB, dev->port->TGTCHAR[i].PIDLSB);
                    printf("\t - BCR, DCR       = 0x%08x \n", dev->port->TGTCHAR[i].BCRDCR);
                    printf("\t - DADDR          = 0x%02x \n", dev->target_da[i]);
                }
                printf("\n");
            }
            
            if(mode == 'M')
            {
                if( !I3C_IS_MASTER(dev->port) )
                {
                    printf("Issue Controller Request to become Controller role again ...\n");

                    dev->target_index = 0; // Use main Target
                    dev->ibi_type     = I3C_IBI_TYPE_CR;
                    if(I3C_TgtIssueIBI(dev) != I3C_STS_NO_ERR)
                        printf("\tError in I3C_TgtIssueIBI\n");
                }
                printf("\n");
            }
        }while(0);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    volatile uint32_t i;
    
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART_Init();
    
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------------------+\n");
    printf("|    I3C Controller Write/Read Sample Code      |\n");
    printf("|    # Supports:                                |\n");
    printf("|      - Process In-Band Interrupt              |\n");
    printf("|      - Process Hot-Join Request               |\n");
    printf("|      - Process Controller Request             |\n");
    printf("+-----------------------------------------------+\n");
#if defined(DEVICE_DMA_ENABLED)
    printf("\t[ PDMA enabled ]\n");
#endif    
        
    printf("\n");

    /* Initializes the I3C device */
    I3C_Init(&g_I3CDev);
    
    printf("# I3C Controller settings:\n");
    printf("    - SDA on PA.0\n");
    printf("    - SCL on PA.1\n");
    printf("        - SDR frequency: %d Hz\n", g_I3CDev.i3c_sdr_freq);
    printf("# User can hit [i] to display the function menu.\n");
    printf("\n");

    while(1)
    {
        if(g_I3CDev.intsts != 0)
        {
            if(g_I3CDev.intsts & I3C_INTSTS_IBI_RECEIVED)
                Process_IBIReceived(&g_I3CDev);
            
            g_I3CDev.intsts = 0;
        }
        
        /* Polling a specified char for Controller operation */
        ParseUartInputChar(&g_I3CDev);
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
