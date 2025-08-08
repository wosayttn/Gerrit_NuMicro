/**************************************************************************//**
 * @file     i3c_ControllerRole.c
 * @version  V3.00
 * @brief    Functions for I3C Controller Role.
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
static void ExecCTROperation(I3C_DEVICE_T *dev);


/**
  * @brief  Polling a specified char for Controller operation.
  */
static void ExecCTROperation(I3C_DEVICE_T *dev)
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
                // Write operation to Target-0
                len = 16;
                for(i=0; i<len; i++)
                    g_DevTxData[i] = ((0x10 + i) + sInitVal);
            
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
                                        
#if (DEVICE_CONTROLLER_ROLE == 1)
            case 'R':
                I3C_FuncAddrCCC(dev, I3C_CCC_RSTDAA);
                break;
                    
            case 'E':
                I3C_FuncAddrCCC(dev, I3C_CCC_ENTDAA);
                break;
                    
            case 'i':
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
                break;
#endif

            default:
                printf("\n");
                printf("[s] SDR Write 16-bytes to Target-0, then read to compare \n");
#if (DEVICE_CONTROLLER_ROLE == 1)
                printf("[E] ENTDAA CCC to set all Targets in I3C mode \n");
                printf("[R] RSTDAA CCC to set all Targets in I2C mode \n");
                printf("[i] Dump all Target's info \n");
#endif            
                printf("\n");
                break;
        }        
    }
}

/**
  * @brief  Run in I3C Controller Role.
  */
void I3C_ControllerRole(I3C_DEVICE_T *dev, uint32_t u32IsInit)
{
    volatile uint32_t i;

    if(u32IsInit == 1)
    {    
        /* Initialize the device as I3C Controller Role */
        
        /* Initializes the I3C device */
        I3C_Init(dev);
    }
    else
    {
        /* Device has switched to I3C Controller Role */
        
        printf("\n*** Switched to Controller Role ***\n");
        dev->device_role = I3C_CONTROLLER;
        
#if (DEVICE_CONTROLLER_ROLE == 0)
        /* The I3C Secondary Controller need to allocate Device Address Table for all Targets */
        for(i=0; i<7; i++)
        {
            if(i >= dev->target_count)
                dev->target_da[i] = 0;
            
            I3C_SetDeviceAddr(dev->port, i, I3C_DEVTYPE_I3C, dev->target_da[i], 0x0);
        }
#endif        
    
        printf("\nValid Target's DA: \n\t");    
        for(i=0; i<dev->target_count; i++)
        {        
            printf("0x%02x, ", ((dev->port->TGTCFG[i] >> I3C_TGTCFG_DADDR_Pos) & 0x7F));
        }
        printf("\n\n");    
    }
    
    while(1)
    {
        if(dev->intsts != 0)
        {
            if(dev->intsts & I3C_INTSTS_IBI_RECEIVED)
            {
                dev->intsts &= ~I3C_INTSTS_IBI_RECEIVED;
                I3C_FuncIBIReceived(dev);            
            }
            
            if(dev->intsts & I3C_INTSTS_BUSOWNER_UPDATED)
            {
                /* Switch to I3C Target Role */
                dev->intsts = 0;
                break;
            }
        }
        //dev->intsts = 0;
                
        /* Polling a specified char for Controller operation */
        ExecCTROperation(dev);
    }
    
    /* Jump to execute in i3c_TargetRole.c */
}
