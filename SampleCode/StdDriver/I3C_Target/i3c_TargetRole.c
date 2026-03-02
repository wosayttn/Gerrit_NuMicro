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
static void     DumpCurrentTargetInfo(I3C_DEVICE_T *dev);
static int32_t  ParseTGTReceiveData(I3C_DEVICE_T *dev);
static void     ExecTGTOperation(I3C_DEVICE_T *dev);


/**
  * @brief  Dump current Target's Info.
  */
static void DumpCurrentTargetInfo(I3C_DEVICE_T *dev)
{
    volatile uint32_t i;
            
    printf("\nCurrent Target works at %s mode.\n", ((uint32_t)I3C_IS_DA_VALID(dev->port)? "I3C":"I2C"));
    
    if(I3C_IS_DA_VALID(dev->port))
    {
        printf("\t - Main DADDR     = 0x%02x \n", (uint8_t)I3C_GET_I3C_DA(dev->port));
        
        /* Virtual Target 0~3 Address  */
        for(i=0; i<4; i++)
        {
            if(dev->port->VTGTCFG[i].ADDR & I3C_VTGTADDR_ENABLE_Msk)
            {
                if(dev->port->VTGTCFG[i].ADDR & I3C_VTGTADDR_DAVALID_Msk)
                    printf("\t - VT-%d DADDR     = 0x%02x \n", i, ((uint32_t)(dev->port->VTGTCFG[i].ADDR & I3C_VTGTADDR_DADDR_Msk) >> I3C_VTGTADDR_DADDR_Pos));
            }                        
        }        
        
        /* Group Address 0~3 */
        for(i=0; i<4; i++)
        {
            if((dev->port->GRPASTS[i] & (0x1F<<I3C_GRPASTS_GRPAVLD0_Pos)) != 0 )
                printf("\t - GP-%d DADDR     = 0x%02x \n", i, (uint8_t)(dev->port->GRPASTS[i] & I3C_GRPASTS_GRPADDR_Msk));
        }
    }
    else
    {
        printf("\t - Main SADDR     = 0x%02x \n", (uint8_t)I3C_GET_I2C_SA(dev->port));
        
        /* Virtual Target 0~3 Address  */
        for(i=0; i<4; i++)
        {
            if(dev->port->VTGTCFG[i].ADDR & I3C_VTGTADDR_ENABLE_Msk)
            {
                if(dev->port->VTGTCFG[i].ADDR & I3C_VTGTADDR_SAVALID_Msk)
                    printf("\t - VT-%d SADDR     = 0x%02x \n", i, ((uint32_t)(dev->port->VTGTCFG[i].ADDR & I3C_VTGTADDR_SADDR_Msk) >> I3C_VTGTADDR_SADDR_Pos));
            }                        
        }
    }
    printf("\n");
}

/**
  * @brief  Prepare CCC response data.
  */
int32_t I3C_FuncSetCCCData(I3C_DEVICE_T *dev, uint32_t ccc, uint8_t tgt)
{    
    if((dev->port->DEVCTL & I3C_DEVCTL_ENABLE_Msk) == 0)
        return -1;
    
    switch(ccc)
    {
        case I3C_CCC_GETXTIME:
            /* GETXTIME CCC response data at EXTCMD 6 */
            dev->target_index  = tgt;
            dev->tx_len        = 4;
            dev->tx_buf[0]     = 0x3; // Supported Modes Byte
            dev->tx_buf[1]     = 0x3; // State Byte 
            dev->tx_buf[2]     = 20;  // Frequency Byte. E.g, 0.5 MHz x20=10 MHz
            dev->tx_buf[3]     = 25;  // Inaccuracy Byte. E.g, represents a maximum frequency variation of 2.5%
            dev->target_extcmd = 6;  
            dev->ccc_code      = I3C_CCC_GETXTIME;    
            /* Push data to Extend Command Data Buffer */
            I3C_TgtSend(dev);
            break;
        
        case I3C_CCC_RSTACT(0):
            /* RSTACT CCC response data at EXTCMD 7 */
            dev->target_index  = tgt;
            dev->tx_len        = 1;
            dev->tx_buf[0]     = 0xa0;
            dev->target_extcmd = 7;  
            dev->ccc_code      = I3C_CCC_RSTACT(0);    
            dev->is_DB         = TRUE;
            dev->DB            = 0x80;
            /* Push data to Extend Command Data Buffer */
            I3C_TgtSend(dev);
            break;
        
        default:
            break;
    }
        
    return 0;
}

/**
  * @brief  Polling a specified char for Target operation.
  */
static void ExecTGTOperation(I3C_DEVICE_T *dev)
{
    char mode;
    
    if((DEBUG_PORT->FIFOSTS&UART_FIFOSTS_RXEMPTY_Msk) == 0U)
    {
        mode = (char)DEBUG_PORT->DAT;
        switch(mode)
        {
            case 'I':
                if(dev->device_role == I3C_CONTROLLER)
                {
                    printf("Device at Controller Role, no action.\n\n");
                    break;
                }
                
#if (DEVICE_CONTROLLER_ROLE == 1)
                I3C_FuncIBIRequest(dev, 0xA0, 0xA4A3A2A1, 4);
#else
                I3C_FuncIBIRequest(dev, 0xB, 0xB4B3B2B1, 4);
#endif                
                break;
                                                            
            case '#':
                if(dev->device_role == I3C_CONTROLLER)
                {
                    printf("Device at Controller Role, no action.\n\n");
                    break;
                }
                
                I3C_FuncCRRequest(dev);
                break;

            default:
                printf("\n");
                printf("[I] Perform In-Band Interrupt Request \n");
                printf("[#] Perform Controller Request (Target will switch to Controller Role) \n");
            
                /* Dump Target Info. */
                DumpCurrentTargetInfo(dev);
                break;
        }        
    }
}

static int32_t ParseTGTReceiveData(I3C_DEVICE_T *dev)
{
    volatile uint32_t i;
    uint8_t RespQSts, TID, RxLen, IsCCCWrite, IsDEFTGTS, CmdSize, CCCCode;
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
        CCCCode    = dev->tgtRespQ[0].CmdCCC;
        pu8Data    = (uint8_t *)(dev->tgtRespQ[0].RxBufAddr);
        if(RespQSts == I3C_STS_NO_ERR)
        {
            if(IsDEFTGTS == 1)
            {
                /* Add delay loop */
                I3C_DelayLoop(SystemCoreClock/500);
                I3C_FuncDEFTGTSResp(dev);
                return 0;
            }
            
            //printf("[ CCC/CMD word: 0x%x ]\n", dev->tgtRespQ[0].CmdWord);
            printf("*** [ Resp for %s Write - ", 
                (IsCCCWrite == 1)? "CCC":(CmdSize == 0)? "Private":\
                (CCCCode == I3C_CCC_HDRDDR)? "HDR-DDR":\
                (CCCCode == I3C_CCC_HDRBT)? "HDR-BT":"UNDEF");
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
                        
            /* TODO: check if IsCCCWrite command */
            if(IsCCCWrite == 1)
                return 0;
            
                
            /* Prepare Target transfer data for Controller Read operation */
            memcpy((uint8_t *)dev->tx_buf, pu8Data, RxLen);
            dev->target_index  = TID;   // Target index, valid: 0~4
            dev->target_extcmd = TID;   // Ext cmd index, valid: 0~7   
            dev->tx_len        = RxLen; // byte count
            dev->ccc_code      = CCCCode;
            if(dev->ccc_code == I3C_CCC_HDRDDR)
                dev->is_HDR_cmd = TRUE;
            if(dev->ccc_code == I3C_CCC_HDRBT)
                dev->is_HDRBT_cmd = TRUE;
            
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
        
        DumpCurrentTargetInfo(dev);
        
        /* Prepare CCC response data */
        I3C_FuncSetCCCData(dev, I3C_CCC_GETXTIME, 0);
        I3C_FuncSetCCCData(dev, I3C_CCC_RSTACT(0), 0);
    }
    
    while(1)
    {
        if(dev->intsts != 0)
        {
            if((dev->intsts & (I3C_INTSTS_DA_ASSIGNED | I3C_INTSTS_GRPDA_ASSIGNED | I3C_INTSTS_CCC_UPDATED)) == I3C_INTSTS_CCC_UPDATED)
            {
                dev->intsts &= ~I3C_INTSTS_CCC_UPDATED;
                printf("# CCC Updated Info.\n");                
                printf("\t- MWL to 0x%04x\n", 
                    (uint32_t)((dev->port->SLVMXLEN & I3C_SLVMXLEN_MWL_Msk) >> I3C_SLVMXLEN_MWL_Pos));
                printf("\t- MRL to 0x%04x. IBI Payload Size %d.\n", 
                    (uint32_t)((dev->port->SLVMXLEN & I3C_SLVMXLEN_MRL_Msk) >> I3C_SLVMXLEN_MRL_Pos),
                    (uint32_t)((dev->port->DEVCTL & I3C_DEVCTL_IBIPSIZE_Msk) >> I3C_DEVCTL_IBIPSIZE_Pos));
                printf("\t- ENTAS %d\n", (uint32_t)((dev->port->SLVEVNTS & I3C_SLVEVNTS_ACTSTSTS_Msk) >> I3C_SLVEVNTS_ACTSTSTS_Pos));
                printf("\t- HJEN  %d\n", (uint32_t)((dev->port->SLVEVNTS & I3C_SLVEVNTS_HJEN_Msk) >> I3C_SLVEVNTS_HJEN_Pos));
                printf("\t- SIREN %d\n", (uint32_t)((dev->port->SLVEVNTS & I3C_SLVEVNTS_SIREN_Msk) >> I3C_SLVEVNTS_SIREN_Pos));
                printf("\n");
                
                /* Prepare CCC response data */
                I3C_FuncSetCCCData(dev, I3C_CCC_GETXTIME, 0);
                I3C_FuncSetCCCData(dev, I3C_CCC_RSTACT(0), 0);
            }

            if(dev->intsts & I3C_INTSTS_GRPDA_ASSIGNED)
            {
                dev->intsts &= ~I3C_INTSTS_GRPDA_ASSIGNED;
                
                /* Dump Target Info. */
                DumpCurrentTargetInfo(dev);
            }

            if(dev->intsts & I3C_INTSTS_DA_ASSIGNED)
            {
                /* Add delay loop */
                I3C_DelayLoop(SystemCoreClock/500);
                
                dev->intsts &= ~I3C_INTSTS_DA_ASSIGNED;
                
                /* Dump Target Info. */
                DumpCurrentTargetInfo(dev);
                
                /* Prepare CCC response data */
                I3C_FuncSetCCCData(dev, I3C_CCC_GETXTIME, 0);
                I3C_FuncSetCCCData(dev, I3C_CCC_RSTACT(0), 0);
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
                dev->intsts = 0;
                
                printf("# BUSOWNER UPDATED, change to %s.\n\n", I3C_IS_CONTROLLER(dev->port)?"Controller Role":"Target Role");
                break;
            }
            
            if(dev->intsts & I3C_INTSTS_EXTCMD_FINISHED)
            {
                dev->port->INTSTS = I3C_INTSTS_EXTCMD_FINISHED; 
                
                for(i=0; i<8; i++)
                {
                    /* Check if valid finished cmd */
                    if(dev->extcmdsts & (1<<i))
                    {
                        uint8_t tgt, ccc;
                        
                        tgt = ((dev->port->EXTCMD[i].WORD1 & I3C_TGTCMDW1_ADDRIDX_Msk) >> I3C_TGTCMDW1_ADDRIDX_Pos);
                        ccc = ((dev->port->EXTCMD[i].WORD2 & I3C_TGTCMDW2_CCCHEAD_Msk) >> I3C_TGTCMDW2_CCCHEAD_Pos);
                        
                        /* Check if CCC Direct Read Command finished */ 
                        if(dev->port->EXTCMD[i].WORD1 & I3C_TGTCMDW1_CCC_Msk)
                        {
                            printf("# Target-%d transmit CCC-0x%02x finished.\n\n", tgt, ccc);
                            
                            /* Prepare CCC response data */
                            if((dev->port->EXTCMD[i].WORD2 & I3C_TGTCMDW2_CCCHEAD_Msk))
                                I3C_FuncSetCCCData(dev, ccc, tgt);
                        }
                        else
                        {
                            printf("# Target-%d transmit data finished.\n\n", tgt);
                        }
                    }                        
                }
            }
            
            dev->intsts = 0;
        }
        
        /* Parsing Target receiving data */
        ParseTGTReceiveData(dev);
        
        /* Polling a specified char for Target operation */
        ExecTGTOperation(dev);
    }
    
    /* Jump to execute in i3c_ControllerRole.c */    
}
