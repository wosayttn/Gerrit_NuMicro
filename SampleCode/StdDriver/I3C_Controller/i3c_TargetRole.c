/**************************************************************************//**
 * @file     i3c_TargetRole.c
 * @version  V3.00
 * @brief    Functions for I3C Target Role.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "i3c_DeviceFunc.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static int32_t  ParseTGTReceiveData(I3C_DEVICE_T *dev);
static void     ExecTGTOperation(I3C_DEVICE_T *dev);


/**
  * @brief  Polling a specified char for Target operation.
  */
static void ExecTGTOperation(I3C_DEVICE_T *dev)
{
    char mode;
    volatile uint32_t i;
    static uint8_t sInitVal = 0;
    int32_t iRet = I3C_STS_NO_ERR;
    uint16_t len;
    uint8_t g_DevTxData[I3C_DEVICE_TX_BUF_CNT * 4], g_DevRxData[I3C_DEVICE_TX_BUF_CNT * 4];
    
    if((DEBUG_PORT->FIFOSTS&UART_FIFOSTS_RXEMPTY_Msk) == 0U)
    {
        mode = (char)DEBUG_PORT->DAT;
        switch(mode)
        {
            case 's':
                if(dev->device_role == I3C_TARGET)
                {
                    printf("Device at Target Role, no action.\n\n");
                    break;
                }
            
                // Write operation to Target-0
                len = 16;
                for(i=0; i<len; i++)
                    g_DevTxData[i] = ((0xA0 + i) + sInitVal);
            
                iRet = I3C_FuncSDRWrite(dev, 0, (uint8_t *)g_DevTxData, len);
                if(iRet != I3C_STS_NO_ERR)
                    break;
                printf("\n[ Write PASS ]\n\t");
                
                for(i=0; i<len; i++)
                {
                    printf("%02x ", dev->tx_buf[i]);
                    if((i%8) == 7)
                        printf("\n\t");
                }
                sInitVal++;
                printf("\n");
                
                CLK_SysTickDelay(5000);
                
                // Read operation from Target-0
                len = 16;
                iRet = I3C_FuncSDRRead(dev, 0, (uint8_t *)g_DevRxData, &len);
                if(iRet != I3C_STS_NO_ERR)
                    break;
                printf("\n[ Read PASS ]\n\t");
                                    
                for(i=0; i<len; i++)
                {
                    printf("%02x ", g_DevRxData[i]);
                    if((i%8) == 7)
                        printf("\n\t");
                }
                printf("\n");
                break;
                                                            
            case '#':
                I3C_FuncCRRequest(dev);
                break;
                    
            default:
                printf("\n");
                printf("[s] SDR Write 16-bytes to Target-0, then read to compare \n");
                printf("[#] Switch device to Controller Role \n");
                printf("\n");
                break;
        }        
    }
}

static int32_t ParseTGTReceiveData(I3C_DEVICE_T *dev)
{
    volatile uint32_t i;
    uint8_t RespQSts, TID, RxLen, IsCCCWrite, IsDEFTGTS, CmdSize;
    uint8_t *pu8Data;

    RespQSts = dev->tgtRespQ[0].ErrSts; /* Parse from I3C_TgtRecv(...) */
    
    if(RespQSts != (uint8_t)I3C_TGTRESP_INITIAL_VALUE)
    {
        dev->tgtRespQ[0].ErrSts = (uint8_t)I3C_TGTRESP_INITIAL_VALUE;
        
        TID        = dev->tgtRespQ[0].TargetID;
        RxLen      = dev->tgtRespQ[0].RxBufLen;
        IsCCCWrite = dev->tgtRespQ[0].IsCCCWrite;
        IsDEFTGTS  = dev->tgtRespQ[0].IsDEFTGTS;
        CmdSize    = dev->tgtRespQ[0].CmdSize;
        pu8Data    = (uint8_t *)(dev->tgtRespQ[0].RxBufAddr);
        if(RespQSts == I3C_STS_NO_ERR)
        {
            if(IsDEFTGTS == 1)
            {
                /* Add delay loop */
                I3C_DelayLoop(SystemCoreClock/500);
                I3C_FuncGetDEFTGTS(dev);
                return 0;
            }
            
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
                    printf("Read CCC/CMD code: 0x%x.\n", dev->tgtRespQ[0].CmdWord);

                if(CmdSize == 2)
                    printf("Read CCC/CMD code: 0x%x and Defining byte: 0x%x.\n", dev->tgtRespQ[0].CmdWord, dev->rx_buf[1]);
            }
                            
            if(RxLen == 0)
            {
                if(dev->is_DMA)
                {                
                    /* Enable I3C PDMA receive function */
                    I3C_ConfigRxDMA(dev, (uint32_t)(&dev->port->TXRXDAT), (uint32_t)(dev->rx_buf), (I3C_DEVICE_RX_BUF_CNT * 4));
                }
                printf("\n");
                return 0;
            }
            
            printf("Read %d-bytes: \n\t", RxLen);
            pu8Data = (uint8_t *)(dev->tgtRespQ[0].RxBufAddr);                   
            for(i=0; i<RxLen; i++)
            {
                printf("%02x ", pu8Data[i]);
            }
            printf("\n\n");
            
            /* Prepare Target transfer data for Controller Read operation */
            memcpy((uint8_t *)dev->tx_buf, pu8Data, RxLen);
            dev->target_index  = TID;  // Target index, valid: 0~4
            dev->target_extcmd = TID;   // Ext cmd index, valid: 0~7   
            dev->tx_len        = RxLen; // byte count
            dev->ccc_code      = 0;
            
            /* Push data to Extend Command Data Buffer */
            I3C_TgtSend(dev);
        }
        else
        {
            printf("\n*** [ Resp error status  0x%x ] ***\n\n", RespQSts);
            
            I3C_TgtHandleTransErr(dev);
        }
        
        if(dev->is_DMA)
        {                
            /* Enable I3C PDMA receive function */
            I3C_ConfigRxDMA(dev, (uint32_t)(&dev->port->TXRXDAT), (uint32_t)(dev->rx_buf), (I3C_DEVICE_RX_BUF_CNT * 4));
        }
    }
    
    return 0;
}

/**
  * @brief  Run in I3C Target Role.
  */
void I3C_TargetRole(I3C_DEVICE_T *dev, uint32_t u32IsInit)
{    
    volatile uint32_t i;
       
    if(u32IsInit == 1)
    {
        /* Initialize the device as I3C Target Role */
                    
        /* Initializes the I3C device */
        I3C_Init(dev);
    }
    else
    {        
        /* Device has switched to I3C Target Role */
        
        printf("\n*** Switched to Target Role ***\n\n");
        dev->device_role = I3C_TARGET;
        
        if(dev->is_DMA)
        {                
            /* Enable I3C PDMA receive function */
            I3C_ConfigRxDMA(dev, (uint32_t)(&dev->port->TXRXDAT), (uint32_t)(dev->rx_buf), (I3C_DEVICE_RX_BUF_CNT * 4));
        }
    }
            
    while(1)
    {
        if(dev->intsts != 0)
        {
            if(dev->intsts & I3C_INTSTS_DA_ASSIGNED)
            {
                /* Add delay loop */
                I3C_DelayLoop(SystemCoreClock/500);
                
                dev->intsts &= ~I3C_INTSTS_DA_ASSIGNED;
                
                for(i=0; i<5; i++)
                {
                    if(dev->target_da[i] != 0x0)
                        printf("# Target-%d DA: 0x%02x (%s Target)\n", i, dev->target_da[i], (i==0)?"Main":"Virtual");
                    else
                        printf("# Target-%d SA: 0x%02x (%s Target)\n", i, dev->target_sa[i], (i==0)?"Main":"Virtual");
                }                
                printf("\n");
            }     
            
            if(dev->intsts & I3C_INTSTS_IBI_UPDATED)
            {                
                dev->intsts &= ~I3C_INTSTS_IBI_UPDATED;
                printf("# IBI UPDATED completed.\n\n");
            }
            
            if(dev->intsts & I3C_INTSTS_BUSOWNER_UPDATED)
            {
                /* Switch to I3C Controller Role */
                dev->port->INTSTS = I3C_INTSTS_BUSOWNER_UPDATED;
                //dev->intsts &= ~I3C_INTSTS_BUSOWNER_UPDATED;
                dev->intsts = 0;
                printf("# BUSOWNER UPDATED, change to %s.\n\n", I3C_IS_MASTER(dev->port)?"Controller Role":"Target Role");
                break;
            }
        }
        //dev->intsts = 0;
        
        /* Parsing Target receiving data */
        ParseTGTReceiveData(dev);
        
        /* Polling a specified char for Target operation */
        ExecTGTOperation(dev);
    }
    
    /* Jump to execute in i3c_ControllerRole.c */    
}
