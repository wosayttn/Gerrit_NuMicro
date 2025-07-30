/**************************************************************************//**
 * @file     i3c.c
 * @version  V1.00
 * @brief    I3C driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "NuMicro.h"


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup I3C_Driver I3C Driver
  @{
*/

/** @addtogroup I3C_EXPORTED_FUNCTIONS I3C Exported Functions
  @{
*/

/**
  * @brief      Open I3C with Static Address and Initial Mode
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  * @param[in]  u32MasterSlave  Decides the I3C module is operating in master mode or in slave mode. Valid values are:
  *                                 - \ref I3C_SLAVE
  *                                 - \ref I3C_MASTER
  * @param[in]  u8StaticAddr    7-bit slave address for I2C operation.
  * @param[in]  u32ModeSel      Initial mode selection to support ENTDAA CCC or Hot-Join generation. Valid values are:
  *                                 - \ref I3C_SUPPORT_ENTDAA
  *                                 - \ref I3C_SUPPORT_ADAPTIVE_HJ
  *                                 - \ref I3C_SUPPORT_IMMEDIATE_HJ
  *
  * @details    This API is used to configure I3C controller can receive ENTDAA CCC or generate a Hot-Join request.
  *             The 7-bit slave address is used for related I2C operations until I3C controller has a valid Dynamic Address.
  * @note       This API also enables all interrupt status and configures TX, RX FIFO and Response, Command Queue threshold to 0.
  */
void I3C_Open(I3C_T *i3c,
              uint32_t u32MasterSlave,
              uint8_t u8StaticAddr,
              uint32_t u32ModeSel)
{
    i3c->DEVCTLE = u32MasterSlave; // [0] 0: master, 1: slave
    /* Enable all interrupt status */
    i3c->INTSTSEN = 0xFFFFFFFF;
    /* Set Tx/Rx/CmdQ/ReapQ threshold to 0 */
    i3c->DBTHCTL = 0x0;
    i3c->QUETHCTL = 0x0;

    if (u32MasterSlave == I3C_MASTER)
    {
        // In Master mode, self-assigns its dynamic address.
        i3c->DEVADDR = ((u8StaticAddr << I3C_DEVADDR_SA_Pos) | I3C_DEVADDR_SAVALID_Msk)
                       | ((u8StaticAddr << I3C_DEVADDR_DA_Pos) | I3C_DEVADDR_DAVALID_Msk);
        return;
    }
    else if (u8StaticAddr == 0)
    {
        i3c->DEVADDR &= ~I3C_DEVADDR_SAVALID_Msk;
    }
    else
    {
        i3c->DEVADDR &= ~I3C_DEVADDR_SA_Msk;
        i3c->DEVADDR |= (I3C_DEVADDR_SAVALID_Msk | u8StaticAddr);
    }

    /* Select the slave ability for enter I3C mode */
    if (u32ModeSel == I3C_SUPPORT_ENTDAA)
    {
        /* HJEN disabled: Slave supports ENTDAA CCC */
        i3c->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
        i3c->SLVEVNTS &= ~I3C_SLVEVNTS_HJEN_Msk;
    }
    else if (u32ModeSel == I3C_SUPPORT_ADAPTIVE_HJ)
    {
        /* Both ADAPTIVE and HJEN enabled: Slave generates a Hot-Join request until receive I3C header 7'h7E on the bus */
        i3c->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
        i3c->SLVEVNTS |= I3C_SLVEVNTS_HJEN_Msk;
    }
    else if (u32ModeSel == I3C_SUPPORT_IMMEDIATE_HJ)
    {
        /* Only HJEN enabled: Slave generates a Hot-Join request immediately */
        i3c->DEVCTL &= ~I3C_DEVCTL_ADAPTIVE_Msk;
        i3c->SLVEVNTS |= I3C_SLVEVNTS_HJEN_Msk;
    }
    else
    {
        /* HJEN disabled: Slave supports ENTDAA CCC */
        i3c->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
        i3c->SLVEVNTS &= ~I3C_SLVEVNTS_HJEN_Msk;
    }
}

/**
  * @brief      Reset and Resume I3C Controller
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  * @param[in]  u32ResetMask    Software reset operation of I3C module. Valid values are:
  *                                 - \ref I3C_RESET_CMD_QUEUE
  *                                 - \ref I3C_RESET_RESP_QUEUE
  *                                 - \ref I3C_RESET_TX_BUF
  *                                 - \ref I3C_RESET_RX_BUF
  *                                 - \ref I3C_RESET_ALL_QUEUE_AND_BUF
  * @param[in]  u32EnableResume Enable resume I3C Slave controller. Valid values are TRUE and FALSE.
  *
  * @retval     I3C_STS_NO_ERR      No error
  * @retval     I3C_TIMEOUT_ERR     Function time-out
  *
  * @details    This API is used to reset specified FIFO and Queue or resume the I3C Slave controller from the halt state.
  * @note       THe application can reset Queues and FIFO only when the Slave controller is disabled or
  *             when the Slave controller is in Halt state(I3C_CCCDEVS[9] SLVBUSY = 1) after any error occurred from the I3C Master request.
  */
int32_t I3C_ResetAndResume(I3C_T *i3c, uint32_t u32ResetMask, uint32_t u32EnableResume)
{
    uint8_t u8InHaltState = 0;
    volatile uint32_t u32Timeout;

    if (I3C_IS_SLAVE_BUSY(i3c))
    {
        u8InHaltState = 1;
    }

    if (u32ResetMask)
    {
        if (u8InHaltState == 0)
        {
            /* Disable I3C controller for reset buffer and queue */
            if (I3C_Disable(i3c) != I3C_STS_NO_ERR)
            {
                return I3C_TIMEOUT_ERR;
            }
        }

        /* Reset specify source */
        i3c->RSTCTL = u32ResetMask;
        u32Timeout = (SystemCoreClock / 1000);

        while ((i3c->RSTCTL != 0) && (--u32Timeout)) {}

        if (u32Timeout == 0)
        {
            return I3C_TIMEOUT_ERR;
        }

        if (u8InHaltState == 0)
        {
            /* Enable I3C controller again */
            if (I3C_Enable(i3c) != I3C_STS_NO_ERR)
            {
                return I3C_TIMEOUT_ERR;
            }
        }
    }

    if (u32EnableResume || u8InHaltState)
    {
        /* The application has to take necessary action to handle the error condition and
            then set RESUME bit to resume the controller. */
        /* Slave will receive GETSTATUS CCC to clear specify status in I3C_CCCDEVS register. */
        i3c->DEVCTL |= I3C_DEVCTL_RESUME_Msk;

        while ((i3c->DEVCTL & I3C_DEVCTL_RESUME_Msk) == I3C_DEVCTL_RESUME_Msk) {}

        /* RESUME bit is auto-cleared once the controller is ready to accept new transfers. */
    }

    return I3C_STS_NO_ERR;
}

// move to emulation code
///**
//  * @brief      Get Resopnse Status and Receive Data
//  *
//  * @param[in]  i3c             The pointer of the specified I3C module.
//  * @param[out] pu32RespQ       The response data structure to get the response data.
//  *
//  * @retval     I3C_STS_NO_ERR          No error
//  * @retval     I3C_STS_INVALID_INPUT   Invalid input parameter
//  * @retval     I3C_STS_RESPQ_EMPTY     No Response Queue data
//  * @return     Response error status
//  *
//  * @details    This API is used to get the response data and the received data on Master write operation.
//  */
//int32_t I3C_ParseRespQueue(I3C_T *i3c, uint32_t *pu32RespQ)
//{
//    uint8_t qn, u8RespQCnt;

//    /* Check if RespQ buffer is empty */
//    if (pu32RespQ == NULL)
//    {
//        return I3C_STS_INVALID_INPUT;
//    }

//    /* Check if RespQ is empty */
//    if (I3C_IS_RESPQ_EMPTY(i3c))
//    {
//        return I3C_STS_RESPQ_EMPTY;
//    }

//    u8RespQCnt = I3C_GET_RESPQ_THLD(i3c) + 1;
//    qn = 0; // Queue number

//    do
//    {
//        pu32RespQ[qn] = I3C_GET_RESP_DATA(i3c);
//#if defined(__DBG)        
//    printf("RESPQ :0x%08x\n", pu32RespQ[qn]);
//#endif        

//        if (!I3C_IS_RESP_NO_ERR(pu32RespQ[qn]))
//        {
//            return (pu32RespQ[qn] & I3C_RESPQUE_ERRSTS_Msk);
//        }

//        qn++;
//        u8RespQCnt--;
//    } while (u8RespQCnt);

//    return I3C_STS_NO_ERR;
//}

/**
  * @brief      Set Command Queue and Transmit Data for Master Read
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  * @param[in]  u8TID           Specified Transmit Transaction ID in Command Queue.
  * @param[in]  pu32TxBuf       The buffer to send the data to transmit FIFO.
  * @param[in]  u16WriteBytes   The byte number of TX data.
  *
  * @retval     I3C_STS_NO_ERR          No error
  * @retval     I3C_STS_INVALID_INPUT   Invalid input parameter
  * @retval     I3C_STS_CMDQ_FULL       Command Queue is full
  * @retval     I3C_STS_TX_FULL         TX FIFO is full
  *
  * @details    This API is used to prepare a Command Queue and TX response data for Master read operation.
  */
int32_t I3C_SetCmdQueueAndData(I3C_T *i3c, uint8_t u8TID, uint32_t *pu32TxBuf, uint16_t u16WriteBytes)
{
    uint32_t i;

    /* Check if write bytes is exceeded */
    if (u16WriteBytes > (I3C_DEVICE_TX_BUF_CNT * 4))
    {
        return I3C_STS_INVALID_INPUT;
    }

    /* Check if CmdQ is full */
    if (I3C_IS_CMDQ_FULL(i3c))
    {
        return I3C_STS_CMDQ_FULL;
    }

    if (pu32TxBuf != NULL)
    {
        for (i = 0; i < ((u16WriteBytes + 3) / 4); i++)
        {
            /* Check if TX buffer is full */
            if (I3C_IS_TX_FULL(i3c))
            {
                return I3C_STS_TX_FULL;
            }

            i3c->TXRXDAT = pu32TxBuf[i];
        }
    }

    i3c->CMDQUE = ((u8TID << I3C_CMDQUE_TID_Pos) | (u16WriteBytes << I3C_CMDQUE_DATLEN_Pos));
    return I3C_STS_NO_ERR;
}

///**
//  * @brief      Generate IBI Request
//  *
//  * @param[in]  i3c             The pointer of the specified I3C module.
//  * @param[in]  u8MandatoryData The mandatory data byte.
//  * @param[in]  u32PayloadData  The payload data.
//  * @param[in]  u8PayloadLen    The byte number of payload data. The maximum length is 4 bytes.
//  *
//  * @retval     I3C_STS_NO_ERR              No error, IBI request accepted by the Master
//  * @retval     I3C_STS_INVALID_STATE       Invalid state
//  * @retval     I3C_STS_INVALID_INPUT       Invalid input parameter
//  *
//  * @details    This API is used to generate an IBI request on the bus.
//  */
//int32_t I3C_SendIBIRequest(I3C_T *i3c, uint8_t u8MandatoryData, uint32_t u32PayloadData, uint8_t u8PayloadLen)
//{
//    /* Check if Controller is in busy state */
//    if (I3C_IS_SLAVE_BUSY(i3c))
//    {
//        return I3C_STS_INVALID_STATE;
//    }

//    /* Check if SIR function enabled */
//    if (!(i3c->SLVEVNTS & I3C_SLVEVNTS_SIREN_Msk))
//    {
//        return I3C_STS_INVALID_STATE;
//    }

//    /* Check if payload length > 4-bytes */
//    if (u8PayloadLen > 4)
//    {
//        return I3C_STS_INVALID_INPUT;
//    }

//    /* Program IBI payload data, payload length and MDB */
//    i3c->SIR = (u8PayloadLen << I3C_SIR_DATLEN_Pos) | (u8MandatoryData << I3C_SIR_MDB_Pos) | (0 << I3C_SIR_CTL_Pos);
//    i3c->SIRDAT = u32PayloadData;
//    /* Trigger IBI request */
//    /* SIR_EN bit be cleared automatically after the Master accepts the IBI request or Slave unable to issue the IBI request */
//    i3c->SIR |= I3C_SIR_SR_Msk;
//    return I3C_STS_NO_ERR;
//}

/**
  * @brief      Generate MR Request
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  *
  * @retval     I3C_STS_NO_ERR              No error, IBI request accepted by the Master
  * @retval     I3C_STS_INVALID_STATE       Invalid state
  *
  * @details    This API is used to generate an IBI request on the bus.
  */
int32_t I3C_SendMRRequest(I3C_T *i3c)
{
    /* Check if Controller is in busy state */
    if (I3C_IS_SLAVE_BUSY(i3c))
    {
        return I3C_STS_INVALID_STATE;
    }

    /* Check if MR function enabled */
    if (!(i3c->SLVEVNTS & I3C_SLVEVNTS_MREN_Msk))
    {
        return I3C_STS_INVALID_STATE;
    }

    /* Trigger MR request */
    i3c->SIR |= I3C_SIR_MR_Msk;
    return I3C_STS_NO_ERR;
}

/**
  * @brief      Enable HJ Generation
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  * @param[in]  u32ModeSel      Select the Hot-Join generation method. Valid values are:
  *                                 - \ref I3C_SUPPORT_ADAPTIVE_HJ
  *                                 - \ref I3C_SUPPORT_IMMEDIATE_HJ
  *
  * @retval     I3C_STS_NO_ERR          No error
  * @retval     I3C_TIMEOUT_ERR         Enable/Disable I3C time-out
  * @retval     I3C_STS_INVALID_INPUT   Invalid input parameter
  *
  * @details    This API is used to enable the specified HJ generation after enable I3C controller again.
  * @note       I3C Slave controller does not recognize the ENTDAA CCC after enabling the HJ generation.
  */
int32_t I3C_EnableHJRequest(I3C_T *i3c, uint32_t u32ModeSel)
{
    volatile uint32_t u32Timeout;
    
    I3C_ResetAndResume(i3c, I3C_RESET_ALL_QUEUE_AND_BUF, FALSE);

    /* Disable I3C controller */
    if (I3C_Disable(i3c) != I3C_STS_NO_ERR)
    {
        return I3C_TIMEOUT_ERR;
    }

    /* Select the slave ability for enter I3C mode */
    if (u32ModeSel == I3C_SUPPORT_ADAPTIVE_HJ)
    {
        /* Both ADAPTIVE and HJEN enabled: Slave generates a Hot-Join request until receive I3C header 7'h7E on the bus */
        i3c->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
        i3c->SLVEVNTS |= I3C_SLVEVNTS_HJEN_Msk;
    }
    else if (u32ModeSel == I3C_SUPPORT_IMMEDIATE_HJ)
    {
        /* Only HJEN enabled: Slave generates a Hot-Join request immediately */
        i3c->DEVCTL &= ~I3C_DEVCTL_ADAPTIVE_Msk;
        i3c->SLVEVNTS |= I3C_SLVEVNTS_HJEN_Msk;
    }
    else
    {
        /* HJEN disabled: Slave supports ENTDAA CCC */
        i3c->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
        i3c->SLVEVNTS &= ~I3C_SLVEVNTS_HJEN_Msk;

        /* Enable I3C controller */
        if (I3C_Enable(i3c) != I3C_STS_NO_ERR)
        {
            return I3C_TIMEOUT_ERR;
        }

        return I3C_STS_INVALID_INPUT;
    }

    /* Enable I3C controller */
    if (I3C_Enable(i3c) != I3C_STS_NO_ERR)
    {
        return I3C_TIMEOUT_ERR;
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief      Disable HJ Generation
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  *
  * @retval     I3C_STS_NO_ERR      No error
  * @retval     I3C_TIMEOUT_ERR     Enable/Disable I3C time-out
  *
  * @details    This API is used to disable the HJ generation.
  * @note       I3C Slave controller can recognize the ENTDAA CCC after disabling the HJ generation.
  */
int32_t I3C_DisableHJRequest(I3C_T *i3c)
{
    /* HJEN disabled: Slave supports ENTDAA CCC */
    i3c->SLVEVNTS &= ~I3C_SLVEVNTS_HJEN_Msk;
    i3c->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
    return I3C_STS_NO_ERR;
}

/**
  * @brief      I3C Response Error Recovery
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  * @param[in]  u32RespStatus   Response error status from the response queue.
  *
  * @return     I3C_STS_NO_ERR
  *
  * @details    This API is used to perform error recovery then the I3C Slave controller can leave Halt(Busy) state.
  * @note       The RESUME operation is completed after a GETSTATUS CCC is received or the specified Master operation is successfully.
  */
int32_t I3C_RespErrorRecovery(I3C_T *i3c, uint32_t u32RespStatus)
{
    if (u32RespStatus != I3C_STS_NO_ERR)
    {
        if (I3C_IS_SLAVE_BUSY(i3c))
        {
            switch (u32RespStatus)
            {
                case I3C_TGTRESP_CRC_ERR:
                case I3C_TGTRESP_PARITY_ERR:
                case I3C_TGTRESP_FRAME_ERR:
                case I3C_TGTRESP_FLOW_ERR:
                    /* Reset RX FIFO -> apply resume */
                    I3C_ResetAndResume(i3c, I3C_RESET_RX_BUF, TRUE);
                    break;

                case I3C_TGTRESP_MASTER_TERMINATE_ERR:
                    /* Reset TX FIFO and CMDQ Queue -> apply resume */
                    I3C_ResetAndResume(i3c, (I3C_RESET_TX_BUF | I3C_RESET_CMD_QUEUE), TRUE);
                    break;

                default:
                    /* Reset all FIFO and Queue */
                    I3C_ResetAndResume(i3c, I3C_RESET_ALL_QUEUE_AND_BUF, FALSE);
                    break;
            }
        }
    }

    return I3C_STS_NO_ERR;
}


/**
  * @brief      Setup I2C / I3C Device Address Table
  *
  * @param[in]  *i3c            Point to I3C peripheral
  * @param[in]  u8DevIndex      the offset of Device Address Table.
  *                             It could be 0 ~ 6 for DEV1ADR to DEV7ADR.
  * @param[in]  u8DevType       the slave device type. It could be I3C_DEVTYPE_I3C for I3C device and I3C_DEVTYPE_I2C for I2C device
  * @param[in]  u8DAddr         7Bits Device Synamic Address
  * @param[in]  u8SAddr         7Bits Device Static Address
  *
  * @retval     I3C_STS_NO_ERR          No error
  * @retval     I3C_STS_INVALID_INPUT   Invalid input parameter
  *
  * @details    The function is used for I3C Master to setup Device Address Table.
  * @note       Device Address Table must be set before communication with Slave Devices.
  *
  */
int32_t I3C_SetDeviceAddr(I3C_T *i3c, uint8_t u8DevIndex, uint8_t u8DevType, uint8_t u8DAddr, uint8_t u8SAddr)
{
    uint32_t i, count, u32Device = 0;

    if ((u8DAddr & 0x80) || (u8SAddr & 0x80))
    {
        return I3C_STS_INVALID_INPUT;
    }

    // I3C Device Dynamic Address with Odd Parity
    for (i = 0, count = 0; i < 8; i++)
    {
        if ((u8DAddr >> i) & 0x1)
        {
            count ++;
        }
    }

    if ((count % 2) == 0)
    {
        u32Device = ((u8DAddr | 0x80) << I3C_TGTCFG_DADDR_Pos);
    }
    else
    {
        u32Device = (u8DAddr << I3C_TGTCFG_DADDR_Pos);
    }

    // I2C Device Static Address
    u32Device |= (u8SAddr << I3C_TGTCFG_SADDR_Pos);
    
    // Support IBI with one or more Mandatory Bytes
    u32Device |= (I3C_TGTCFG_IBIWMDB_Msk);

    // Device Type
    if (u8DevType != I3C_DEVTYPE_I3C)
    {
        u32Device |= I3C_TGTCFG_DEVTYPE_Msk;
    }

    switch (u8DevIndex)
    {
        case 0ul:
        case 1ul:
        case 2ul:
        case 3ul:
        case 4ul:
        case 5ul:
        case 6ul:
            /* Configure device address on specify table location */
            i3c->TGTCFG[u8DevIndex] = u32Device;
            break;
        
        default:
            return I3C_STS_INVALID_INPUT;
            break;
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief      Write data to Slave
  *
  * @param[in]  *i3c            Point to I3C peripheral
  * @param[in]  u8DevIndex      the offset of Device Address Table.
  *                             It could be 0 ~ 6 for DEV1ADR to DEV7ADR.
  * @param[in]  u32Speed        the speed in which the transfer should be driven. It could be
  *                                 \ref I3C_DEVI3C_SPEED_SDR0
  *                                 \ref I3C_DEVI3C_SPEED_SDR1
  *                                 \ref I3C_DEVI3C_SPEED_SDR2
  *                                 \ref I3C_DEVI3C_SPEED_SDR3
  *                                 \ref I3C_DEVI3C_SPEED_SDR4
  *                                 \ref I3C_DEVI3C_SPEED_HDRDDR
  *                                 \ref I3C_DEVI3C_SPEED_I2CFM
  *                                 \ref I3C_DEVI2C_SPEED_I2CFM
  *                                 \ref I3C_DEVI2C_SPEED_I2CFMPLUS
  * @param[in]  *pu32TxBuf      Pointer to array to write data to Slave
  * @param[in]  u16WriteBytes   How many bytes need to write to Slave
  *
  * @retval     I3C_STS_NO_ERR          No error
  * @retval     I3C_STS_INVALID_INPUT   Invalid input parameter
  * @retval     I3C_STS_CMDQ_FULL       Command Queue is full
  * @retval     I3C_STS_TX_FULL         TX FIFO is full
  *
  * @details    The function is used for I3C Master write data to Slave.
  *
  * @note       Device Address Table must be set before using this function.
  *
  */
int32_t I3C_Write(I3C_T *i3c, uint8_t u8DevIndex, uint32_t u32Speed, uint32_t *pu32TxBuf, uint16_t u16WriteBytes)
{
    uint32_t i;
    uint32_t u32TimeOutCount = 0u;
    uint32_t response;

    if ((u16WriteBytes == 0) || (pu32TxBuf == NULL))
    {
        return I3C_STS_INVALID_INPUT;
    }

    /* Check if CmdQ is full */
    if (I3C_IS_CMDQ_FULL(i3c))
    {
        return I3C_STS_CMDQ_FULL;
    }

    if (u16WriteBytes <= 3)
    {
        i3c->CMDQUE = (((pu32TxBuf[0] & 0x00FFFFFF) << I3C_CMDQUE_DATBYTE0_Pos)
                       | ((((1 << u16WriteBytes) - 1) & 0x07) << I3C_CMDQUE_BYTESTRB_Pos)
                       | I3C_CMDATTR_SHORT_DATA_ARG);

        /* Check if CmdQ is full */
        if (I3C_IS_CMDQ_FULL(i3c))
        {
            return I3C_STS_CMDQ_FULL;
        }

        i3c->CMDQUE = (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_SDAP_Msk | I3C_CMDQUE_ROC_Msk
                       | (u32Speed & I3C_CMDQUE_SPEED_Msk)
                       | ((u8DevIndex & 0x1F) << I3C_CMDQUE_DEVINDX_Pos)
                       | (I3C_TX_TID << I3C_CMDQUE_TID_Pos)
                       | I3C_CMDATTR_TRANSFER_CMD);
    }
    else
    {
        i3c->CMDQUE = ((u16WriteBytes << I3C_CMDQUE_DATLEN_Pos) | I3C_CMDATTR_TRANSFER_ARG);

        /* Check if CmdQ is full */
        if (I3C_IS_CMDQ_FULL(i3c))
        {
            return I3C_STS_CMDQ_FULL;
        }

        i3c->CMDQUE = (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk
                       | (u32Speed & I3C_CMDQUE_SPEED_Msk)
                       | ((u8DevIndex & 0x1F) << I3C_CMDQUE_DEVINDX_Pos)
                       | (I3C_TX_TID << I3C_CMDQUE_TID_Pos)
                       | I3C_CMDATTR_TRANSFER_CMD);

        for (i = 0; i < ((u16WriteBytes + 3) / 4); i++)
        {
            u32TimeOutCount = SystemCoreClock;

            /* Check if TX buffer is full */
            while (I3C_IS_TX_FULL(i3c))
            {
                if (--u32TimeOutCount == 0)
                {
                    return I3C_STS_TX_FULL;
                }
            }

            i3c->TXRXDAT = pu32TxBuf[i];
        }
    }

    while ((i3c->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0);

    response = i3c->RESPQUE;

    if (response & I3C_CTRRESP_ERRSTS_Msk)
    {
        i3c->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
        return I3C_STS_INVALID_STATE;
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief      Read data from Slave
  *
  * @param[in]  *i3c            Point to I3C peripheral
  * @param[in]  u8DevIndex      the offset of Device Address Table.
  *                             It could be 0 ~ 6 for DEV1ADR to DEV7ADR.
  * @param[in]  u32Speed        the speed in which the transfer should be driven. It could be
  *                                 \ref I3C_DEVI3C_SPEED_SDR0
  *                                 \ref I3C_DEVI3C_SPEED_SDR1
  *                                 \ref I3C_DEVI3C_SPEED_SDR2
  *                                 \ref I3C_DEVI3C_SPEED_SDR3
  *                                 \ref I3C_DEVI3C_SPEED_SDR4
  *                                 \ref I3C_DEVI3C_SPEED_HDRDDR
  *                                 \ref I3C_DEVI3C_SPEED_I2CFM
  *                                 \ref I3C_DEVI2C_SPEED_I2CFM
  *                                 \ref I3C_DEVI2C_SPEED_I2CFMPLUS
  * @param[in]  *pu32RxBuf      Pointer to array to read data from Slave
  * @param[in]  u16ReadBytes    How many bytes need to read from Slave
  *
  * @retval     I3C_STS_NO_ERR          No error
  * @retval     I3C_STS_INVALID_INPUT   Invalid input parameter
  * @retval     I3C_STS_CMDQ_FULL       Command Queue is full
  *
  * @details    The function is used for I3C Master write data to Slave.
  *
  * @note       Device Address Table must be set before using this function.
  * @note       if u16ReadBytes is not
  *
  */
int32_t I3C_Read(I3C_T *i3c, uint8_t u8DevIndex, uint32_t u32Speed, uint32_t *pu32RxBuf, uint16_t u16ReadBytes)
{
    uint32_t i;
    uint32_t u32TimeOutCount = (SystemCoreClock / 1000);
    uint32_t response;

    if ((u16ReadBytes == 0) || (pu32RxBuf == NULL))
    {
        return I3C_STS_INVALID_INPUT;
    }

    /* Check if CmdQ is full */
    if (I3C_IS_CMDQ_FULL(i3c))
    {
        return I3C_STS_CMDQ_FULL;
    }

    i3c->CMDQUE = ((u16ReadBytes << I3C_CMDQUE_DATLEN_Pos) | I3C_CMDATTR_TRANSFER_ARG);

    /* Check if CmdQ is full */
    if (I3C_IS_CMDQ_FULL(i3c))
    {
        return I3C_STS_CMDQ_FULL;
    }

    i3c->CMDQUE = (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_RNW_Msk | I3C_CMDQUE_ROC_Msk
                   | (u32Speed & I3C_CMDQUE_SPEED_Msk)
                   | ((u8DevIndex & 0x1F) << I3C_CMDQUE_DEVINDX_Pos)
                   | (I3C_RX_TID << I3C_CMDQUE_TID_Pos)
                   | I3C_CMDATTR_TRANSFER_CMD);

    while ((i3c->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0)
    {
        if (--u32TimeOutCount == 0)
        {
            return I3C_TIMEOUT_ERR;
        }
    };

    response = i3c->RESPQUE;

    if ((response & I3C_CTRRESP_ERRSTS_Msk) != I3C_CTRRESP_NO_ERR)
    {
        return I3C_STS_INVALID_STATE;
    }

    response = ((response  & I3C_CTRRESP_DATLEN_Msk) + 3) / 4;

    if (response != 0)
    {
        for (i = 0; i < response; i++)
        {
            pu32RxBuf[i] = i3c->TXRXDAT ;
        }
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief      Broadcast RSTDAA command
  *
  * @param[in]  *i3c            Point to I3C peripheral
  *
  * @retval     I3C_STS_NO_ERR              No error
  * @retval     I3C_STS_INVALID_STATE       Invalid state
  * @details    The function is used for I3C Master.
  *
  */

int32_t I3C_BroadcastRSTDAA(I3C_T *i3c)
{
    uint32_t response;
    i3c->CMDQUE = ((0 << I3C_CMDQUE_DATLEN_Pos) | I3C_CMDATTR_TRANSFER_ARG);
    i3c->CMDQUE = (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk
                   | I3C_CMDQUE_CP_Msk
                   | ((I3C_CCC_RSTDAA <<  I3C_CMDQUE_CMD_Pos) & I3C_CMDQUE_CMD_Msk)
                   | I3C_CMDATTR_TRANSFER_CMD);

    while ((i3c->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0);

    response = i3c->RESPQUE;

    if (response & I3C_CTRRESP_ERRSTS_Msk)
    {
        i3c->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
        return I3C_STS_INVALID_STATE;
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief      Broadcast ENTDAA command
  *
  * @param[in]  *i3c            Point to I3C peripheral
  * @param[in]  u8DevCount      Indicating the number of devices to be assigned with the ENTDAA command
  *
  * @retval     positive integer            number of devices assigned, this value should be less or equal to u8DevCount
  * @retval     I3C_STS_INVALID_STATE       Invalid state
  * @details    The function is used for I3C Master.
  *
  */

int32_t I3C_BroadcastENTDAA(I3C_T *i3c, uint8_t u8DevCount)
{
    uint32_t response, error;
    i3c->CMDQUE = (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk
                   | (0 << I3C_CMDQUE_DEVINDX_Pos)
                   | ((u8DevCount << I3C_CMDQUE_DEVCOUNT_Pos) & I3C_CMDQUE_DEVCOUNT_Msk)
                   | (I3C_CCC_ENTDAA <<  I3C_CMDQUE_CMD_Pos)
                   | I3C_CMDATTR_ADDR_ASSGN_CMD);

    while ((i3c->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0);

    response = i3c->RESPQUE;
    I3C_DrvMsg("[CMD val: 0x%08x] I3C_BroadcastENTDAA\n", response);
    error = (response & I3C_CTRRESP_ERRSTS_Msk);

    if (i3c->INTSTS & I3C_INTSTS_TFRERR_Msk)
    {
        i3c->INTSTS |= I3C_INTSTS_TFRERR_Msk;
    }

    if (error == I3C_CTRRESP_NO_ERR)
    {
        return u8DevCount;
    }
    else if (error == I3C_CTRRESP_BRD_ADDR_NACK_ERR)
    {
        i3c->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
        /*
            The controller writes the transfer complete status into the Command Response queue. The Data
            Length Field of Response Data Structure indicates remaining device count in case if the transfer is
            terminated abruptly due to NACK response from the target.
        */
        return (u8DevCount - (response & I3C_CTRRESP_DATLEN_Msk));
    }
    else
    {
        i3c->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
        return I3C_STS_INVALID_STATE;
    }
}

/**
  * @brief      Unicast SETDASA command
  *
  * @param[in]  *i3c            Point to I3C peripheral
  * @param[in]  u8DevIndex      the offset of Device Address Table.
  *                             It could be 0 ~ 6 for DEV1ADR to DEV7ADR.
  *
  * @retval     I3C_STS_NO_ERR              No error
  * @retval     I3C_STS_INVALID_STATE       Invalid state
  * @details    The function is used for I3C Master.
  * @note       Device Address Table must be set before using this function.
  *
  */

int32_t I3C_UnicastSETDASA(I3C_T *i3c, uint8_t u8DevIndex)
{
    uint32_t response;
    i3c->CMDQUE = (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk
                   | (1 << I3C_CMDQUE_DEVCOUNT_Pos)
                   | ((u8DevIndex & 0x1F) << I3C_CMDQUE_DEVINDX_Pos)
                   | (I3C_CCC_SETDASA <<  I3C_CMDQUE_CMD_Pos)
                   | I3C_CMDATTR_ADDR_ASSGN_CMD);

    while ((i3c->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0);

    response = i3c->RESPQUE;

    if (response & I3C_CTRRESP_ERRSTS_Msk)
    {
        i3c->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
        return I3C_STS_INVALID_STATE;
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief      Unicast I3C_CCC_GETACCMST command
  *
  * @param[in]  *i3c            Point to I3C peripheral
  * @param[in]  u8DevIndex      the offset of Device Address Table.
  *                             It could be 0 ~ 6 for DEV1ADR to DEV7ADR.
  * @param[in]  *pu32RxBuf      Pointer to array to read data from Slave
  *
  * @retval     I3C_STS_NO_ERR              No error
  * @retval     I3C_STS_INVALID_STATE       Invalid state
  * @details    The function is used for I3C Master.
  * @note       Device Address Table must be set before using this function.
  *
  */

int32_t I3C_UnicastGETACCMST(I3C_T *i3c, uint8_t u8DevIndex, uint32_t *pu32RxBuf)
{
    uint32_t i, cmd, response, data;
    i3c->CMDQUE = ((1 << I3C_CMDQUE_DATLEN_Pos) | I3C_CMDATTR_TRANSFER_ARG);
    //
    cmd = (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk
           | ((u8DevIndex & 0x1F) << I3C_CMDQUE_DEVINDX_Pos)
           | I3C_CMDQUE_RNW_Msk // 0: Write, 1: Read
           | I3C_CMDQUE_CP_Msk | (I3C_CCC_GETACCMST <<  I3C_CMDQUE_CMD_Pos)
           | I3C_CMDATTR_TRANSFER_CMD);
    i3c->CMDQUE = cmd;

    while ((I3C0->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0);

    response = I3C0->RESPQUE;

    if (response & I3C_CTRRESP_ERRSTS_Msk)
    {
        I3C0->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
    }

    response = ((response  & I3C_CTRRESP_DATLEN_Msk) + 3) / 4;

    if (response != 0)
    {
        for (i = 0; i < response; i++)
        {
            data = I3C0->TXRXDAT;

            if ((pu32RxBuf != NULL))
            {
                pu32RxBuf[0] = data;
            }
        }
    }

    return I3C_STS_NO_ERR;
}



/**
  * @brief  Initial I3C Device Type
  */
int32_t I3C_DeviceInit(I3C_DEVICE_T *dev)
{
    /* Response buffer threshold */
    dev->port->QUETHCTL  = (0 << I3C_QUETHCTL_RESPTH_Pos);
    /* Command buffer empty threshold */
    dev->port->QUETHCTL |= (1 << I3C_QUETHCTL_CMDETH_Pos);
    /* IBI data segment size to 2-words(8-bytes) */
    dev->port->QUETHCTL |= (2 << I3C_QUETHCTL_IBIDATTH_Pos);
    
    /* Rx receive and Tx empty threshold */
    dev->port->DBTHCTL  = ((0 << I3C_DBTHCTL_TXTH_Pos) | (0 << I3C_DBTHCTL_RXTH_Pos));
    /* Rx and Tx start threshold */
    dev->port->DBTHCTL |= ((0 << I3C_DBTHCTL_TXSTATH_Pos) | (0 << I3C_DBTHCTL_RXSTATH_Pos));
    
    /* Clear current interrupt status */
    dev->port->INTSTS |= dev->port->INTSTS;
    
    /* Enable all interrupt status */
    dev->port->INTSTSEN = ~0ul;
    /* Enable specified interrupt signal */
    dev->port->INTEN = (I3C_INTEN_RESPRDY_Msk  | I3C_INTEN_CCCUPD_Msk   | I3C_INTEN_DAA_Msk | 
                        I3C_INTEN_TFRERR_Msk   | I3C_INTEN_READREQ_Msk  | I3C_INTEN_TFRABORT_Msk | 
                        I3C_INTEN_IBIUPD_Msk   | I3C_INTEN_DEFTGTS_Msk  | I3C_INTEN_BUSOWNER_Msk | 
                        I3C_INTEN_BUSRSTDN_Msk | I3C_INTEN_RSTPTDET_Msk | I3C_INTEN_GRPADDRA_Msk | 
                        I3C_INTEN_SDARES_Msk   | I3C_INTEN_EXTFINS_Msk  | I3C_INTEN_IBITH_Msk);
    
    /* Configure related bus timings, SCL clock */
    if(dev->engclk == 0)
        return I3C_STS_INVALID_INPUT;
        
    I3C_BusClkConfig(dev);       
    
    if(dev->device_role == I3C_CONTROLLER)
    {
        /* Program Dynamic Address for Controller */
        dev->port->DEVADDR = (I3C_DEVADDR_DAVALID_Msk | (dev->main_controller_da << I3C_DEVADDR_DA_Pos));
        
        /* Set as Controller role */
        dev->port->DEVCTLE = dev->device_role;
        
        /* Set Conroller includes I3C Broadcast Address */
        dev->port->DEVCTL |= (I3C_DEVCTL_ENABLE_Msk | I3C_DEVCTL_IBAINCL_Msk);
        
        /* Enable device */
        dev->port->DEVCTL |= I3C_DEVCTL_ENABLE_Msk;
    }
    else if(dev->device_role == I3C_TARGET)
    {
        /* Set as Target role */
        dev->port->DEVCTLE = dev->device_role;
        
        if(dev->main_target_sa != 0)
        {
            /* Program Statis Address for Target */
            dev->port->DEVADDR |= (I3C_DEVADDR_SAVALID_Msk | (dev->main_target_sa << I3C_DEVADDR_SA_Pos));
            dev->target_sa[0] = dev->main_target_sa;
        }         
        
        /* Select the Target ability for enter I3C mode */
        if(dev->target_daa_mode == I3C_SUPPORT_ENTDAA)
        {
            /* HJEN disabled: Target supports ENTDAA CCC */
            dev->port->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
            dev->port->SLVEVNTS &= ~I3C_SLVEVNTS_HJEN_Msk;
        }
        else if(dev->target_daa_mode == I3C_SUPPORT_ADAPTIVE_HJ)
        {
            /* Both ADAPTIVE and HJEN enabled: Slave generates a Hot-Join request until receive I3C header 7'h7E on the bus */
            dev->port->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
            dev->port->SLVEVNTS |= I3C_SLVEVNTS_HJEN_Msk;
        }
        else if(dev->target_daa_mode == I3C_SUPPORT_IMMEDIATE_HJ)
        {
            /* Only HJEN enabled: Target generates a Hot-Join request immediately */
            dev->port->DEVCTL &= ~I3C_DEVCTL_ADAPTIVE_Msk;
            dev->port->SLVEVNTS |= I3C_SLVEVNTS_HJEN_Msk;
        }
        else
        {
            /* HJEN disabled: Target supports ENTDAA CCC */
            dev->port->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
            dev->port->SLVEVNTS &= ~I3C_SLVEVNTS_HJEN_Msk;
        }
    }
    else
    {
        return I3C_STS_INVALID_INPUT;
    }
    
    if(dev->device_role == I3C_CONTROLLER)
    {
        if(!(dev->port->DEVCTL & I3C_DEVCTL_ENABLE_Msk))
            return (~I3C_STS_NO_ERR);
    }
    
    return I3C_STS_NO_ERR;
}

/**
  * @brief  Configure Bus Timing
  */
void I3C_BusClkConfig(I3C_DEVICE_T *dev)
{
    volatile uint32_t count;
    
    if(dev->i2c_fm_freq != 0)
    {
        /* SCL freq for I2C FM mode */
        if(dev->i2c_fm_freq > dev->engclk)
            dev->i2c_fm_freq = dev->engclk;
        count = ((dev->engclk/dev->i2c_fm_freq) / 2); if(count<5) count = 5;    
        dev->port->SCLFM = ((count << I3C_SCLFM_FMHCNT_Pos) | (count << I3C_SCLFM_FMLCNT_Pos));   
    }
    
    if(dev->i2c_fm_plus_freq != 0)
    {
        /* SCL freq for I2C FM+ mode */
        if(dev->i2c_fm_plus_freq > dev->engclk)
            dev->i2c_fm_plus_freq = dev->engclk;
        count = ((dev->engclk/dev->i2c_fm_plus_freq) / 2); if(count<5) count = 5;    
        dev->port->SCLFMP = ((count << I3C_SCLFMP_FMPHCNT_Pos) | (count << I3C_SCLFMP_FMPLCNT_Pos));   
    }
    
    if(dev->i3c_sdr_freq != 0)
    {
        /* Set OD mode SCL freq 100kHz */
        count = ((dev->engclk/(100*1000)) / 2); if(count<5) count = 5;    
        dev->port->SCLOD = ((count << I3C_SCLOD_ODHCNT_Pos) | (count << I3C_SCLOD_ODLCNT_Pos));
        
        /* Set PP mode SCL freq for SDR0 */
        if(dev->i3c_sdr_freq > dev->engclk)
            dev->i3c_sdr_freq = dev->engclk;
        count = ((dev->engclk/dev->i3c_sdr_freq) / 2); if(count<5) count = 5;   
        dev->port->SCLPP = ((count << I3C_SCLPP_PPHCNT_Pos) | (count << I3C_SCLPP_PPLCNT_Pos));
        
        /* Set PP mode SCL freq for SDR1, 2, 3, 4 */        
        dev->port->SCLEXTLO = (((count*5) << 24) | ((count*4) << 16) | ((count*3) << 8) | ((count*2) << 0));
    }
    
    /*  Bus Idle Timing ~ 200us */
    count = (uint32_t)(((float)200 * (float)dev->engclk) / (float)1000000); if(count == 0) count = 0x50;
    dev->port->BUSIDLET = count;
    
    /* Bus Available(1.0us) Timing ~ 1.0us */
    count = (uint32_t)(((float)1 * (float)dev->engclk) / (float)1000000); if(count == 0) count = 0x50;
    dev->port->BUSFAT = (count << I3C_BUSFAT_AVAILTC_Pos);
    /* Bus Free Timing ~ 38.4ns/0.5us/1.3us */
    count = (uint32_t)(((float)1300 * (float)dev->engclk) / (float)1000000000); if(count == 0) count = 0x50;
    dev->port->BUSFAT |= (count << I3C_BUSFAT_FREETC_Pos);
}

/**
  * @brief  Show Present State Information
  */
void I3C_PresentStateInfo(I3C_DEVICE_T *dev)
{
    volatile uint32_t role, sts, reg_val[2];

    I3C_DrvMsgOn("\n");
    
    role = ((dev->port->DEVCTLE & I3C_DEVCTLE_OPERMODE_Msk) >> I3C_DEVCTLE_OPERMODE_Pos);
    
    reg_val[0] = dev->port->PRESENTS;
    reg_val[1] = dev->port->CCCDEVS;
    I3C_DrvMsgOn("Present state info: 0x%08x (%s mode)\n", reg_val[0], (role==0)? "Controller":"Target");
    
    sts = ((reg_val[0] & I3C_PRESENTS_TFRTYPE_Msk) >> I3C_PRESENTS_TFRTYPE_Pos);
    if(sts == 0)
    {
        I3C_DrvMsgOn("\tDevice in IDLE state\n");
    }
    else
    {
        if(role == 0)
        {
            if(reg_val[0] & I3C_PRESENTS_CTRIDLES_Msk)
                I3C_DrvMsgOn("\tController is in IDLE State\n");
            else
                I3C_DrvMsgOn("\tController is NOT in IDLE State\n");
            
            if(sts == 0xF)
                I3C_DrvMsgOn("\tController in Halt State, waiting for resume\n");
            else 
                I3C_DrvMsgOn("\tController error code: 0x%x\n", sts);
        }
        else
        {
            if(sts == 1)
                I3C_DrvMsgOn("\tHot-Join Transfer State\n");
            else if(sts == 2)
                I3C_DrvMsgOn("\nIBI Transfer State\n");
            else if(sts == 3)
                I3C_DrvMsgOn("\nController Write Transfer Ongoing\n");
            //else if(sts == 4) Not support in M3331
            //    I3C_DrvMsgOn("\nRead Data Prefetch State\n");
            else if(sts == 5)
                I3C_DrvMsgOn("\nController Read Transfer Ongoing\n");
            else if(sts == 6)
                I3C_DrvMsgOn("\nTarget in Halt State, waiting for resume\n");
            else 
                I3C_DrvMsgOn("\nTarget error code: 0x%x\n", sts);
        }                
    }
    
    I3C_DrvMsgOn("\n");
}

/**
  * @brief  Enable I3C Rx DMA function on PDMA ch-n.
  */
int32_t I3C_ConfigRxDMA(I3C_DEVICE_T *dev, uint32_t u32Src, uint32_t u32Dest, uint32_t u32ByteCnts)
{        
    uint8_t ch = dev->RxDMACh;
    
    PDMA_RESET(PDMA0, ch);
            
    /* PDMA-ch for I3C Rx */
    PDMA0->DSCT[ch].CTL = 
            PDMA_OP_BASIC | PDMA_REQ_SINGLE |
            PDMA_SAR_FIX  | PDMA_DAR_INC |
            PDMA_WIDTH_32 | 
            ((((u32ByteCnts+3)/4)-1) << PDMA_DSCT_CTL_TXCNT_Pos);

    PDMA0->DSCT[ch].SA = u32Src;
    PDMA0->DSCT[ch].DA = u32Dest;

    PDMA0->REQSEL0_3 &= ~PDMA_REQSEL0_3_REQSRC0_Msk;
    PDMA0->REQSEL0_3 |= (PDMA_I3C0_RX << PDMA_REQSEL0_3_REQSRC0_Pos);
    
    PDMA0->CHCTL |= (1 << ch);
        
    return 0;
}

/**
  * @brief  Enable I3C Tx DMA function on PDMA ch-n.
  */
int32_t I3C_ConfigTxDMA(I3C_DEVICE_T *dev, uint32_t u32Src, uint32_t u32Dest, uint32_t u32ByteCnts)
{            
    uint8_t ch = dev->TxDMACh;
    
    PDMA_RESET(PDMA0, ch);
    
    /* PDMA-ch for I3C Tx */
    PDMA0->DSCT[ch].CTL = 
            PDMA_OP_BASIC | PDMA_REQ_SINGLE |
            PDMA_SAR_INC  | PDMA_DAR_FIX |
            PDMA_WIDTH_32 | 
            ((((u32ByteCnts+3)/4)-1) << PDMA_DSCT_CTL_TXCNT_Pos);

    PDMA0->DSCT[ch].SA = u32Src;
    PDMA0->DSCT[ch].DA = u32Dest;
    
    PDMA0->REQSEL0_3 &= ~PDMA_REQSEL0_3_REQSRC1_Msk;
    PDMA0->REQSEL0_3 |= (PDMA_I3C0_TX << PDMA_REQSEL0_3_REQSRC1_Pos);
    
    PDMA0->CHCTL |= (1 << ch);
    
    /* Set DMAEN bit 0 then 1 to load Tx data by I3C DMA */
    I3C_EnableDMA(dev->port);
    
    return 0;
}    

/**
  * @brief  Perform Dynamic Address Assignment by ENTDAA and SETDASA CCC
  */
int32_t I3C_CtrDAA(I3C_DEVICE_T *dev)
{
	uint32_t val, count, index;
    
	/* Initialize command response value */
    dev->cmd_response = I3C_CTRRESP_INITIAL_VALUE;

    if( !((dev->ccc_code == I3C_CCC_ENTDAA) || (dev->ccc_code == I3C_CCC_SETDASA)) )
        return I3C_STS_INVALID_INPUT;
            
    if(dev->ccc_code == I3C_CCC_SETDASA)
    {
        if(dev->target_index > 6)
            return I3C_STS_INVALID_INPUT;
        
        index = dev->target_index;
        count = 1;
    }
    else
    {
        if(dev->target_count > 7)
            return I3C_STS_INVALID_INPUT;
        
        index = dev->target_index;
        count = dev->target_count;
    }
        
	/* Program Address Assignment Command Data Structure */
    val = 0;
	val |= (  (count << I3C_CMDQUE_DEVCOUNT_Pos) 
            | (index << I3C_CMDQUE_DEVINDX_Pos)
            | (dev->tx_id << I3C_CMDQUE_TID_Pos)
            | (dev->ccc_code << I3C_CMDQUE_CMD_Pos) 
            | I3C_CMDATTR_ADDR_ASSGN_CMD );
    
    /* Stop and Response Status on Transfer Completion */
    val |= (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk);
    
    I3C_DrvMsg("[CMD val: 0x%08x] - I3C_CtrDAA\n", val);
    dev->port->CMDQUE = val;
    
	return I3C_STS_NO_ERR;
}

/**
  * @brief  Perform Write CCC Operation
  */
int32_t I3C_CtrCCCSet(I3C_DEVICE_T *dev)
{
	volatile uint32_t i;
	uint32_t val, *p32Buf;

	/* Initialize command response value */
    dev->cmd_response = I3C_CTRRESP_INITIAL_VALUE;

    val = 0;
        
	/* Try to use SDAP */
	if((dev->tx_len > 0) && (dev->tx_len <= I3C_SDAP_MAX_SIZE)) 
    {
		/* Set transfer argument params */
		switch (dev->tx_len) 
        {
            case 3:
                val |= (1 << (I3C_CMDQUE_BYTESTRB_Pos + 2));
                val |= (dev->tx_buf[2] << I3C_CMDQUE_DATBYTE2_Pos);
            case 2:
                val |= (1 << (I3C_CMDQUE_BYTESTRB_Pos + 1));
                val |= (dev->tx_buf[1] << I3C_CMDQUE_DATBYTE1_Pos);
            case 1:
                val |= (1 << (I3C_CMDQUE_BYTESTRB_Pos + 0));
                val |= (dev->tx_buf[0] << I3C_CMDQUE_DATBYTE0_Pos);
            default:
                break;
		}
		val |= I3C_CMDATTR_SHORT_DATA_ARG;
        
        I3C_DrvMsg("[SDAP val: 0x%08x] - I3C_CtrCCCSet\n", val);
        dev->port->CMDQUE = val;

		/* Set transfer command params */
		val = I3C_CMDQUE_SDAP_Msk;
	}
	else if(dev->tx_len > I3C_SDAP_MAX_SIZE) 
    {        
        p32Buf = (uint32_t *)dev->tx_buf;
        
		/* Write bytes to tx port */
        for(i=0; i<((dev->tx_len + 3) / 4); i++)
        {
            dev->port->TXRXDAT = p32Buf[i];
		}
        
		/* Program transfer argument */
		val = ((dev->tx_len << I3C_CMDQUE_DATLEN_Pos) | I3C_CMDATTR_TRANSFER_ARG);
        
        I3C_DrvMsg("[TRANS val: 0x%08x] - I3C_CtrCCCSet\n", val);
        dev->port->CMDQUE = val;
        
		val = 0;
	}
    
	/* Program transfer command */
	val |= (  I3C_CMDQUE_CP_Msk
            | (dev->target_index << I3C_CMDQUE_DEVINDX_Pos)
            | (dev->tx_id << I3C_CMDQUE_TID_Pos)
            | (dev->ccc_code << I3C_CMDQUE_CMD_Pos)
            | I3C_CMDATTR_TRANSFER_CMD );
           
	if(dev->is_last_cmd)
    {
        /* Stop and Response Status on Transfer Completion */
		val |= (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk);
    }
    
    I3C_DrvMsg("[CMD val: 0x%08x] - I3C_CtrCCCSet\n", val);
    dev->port->CMDQUE = val;
            
    /* Clear parameters */
    dev->is_last_cmd = TRUE;
    dev->is_HDR_cmd  = FALSE;
    dev->is_DB       = FALSE;
    
	return I3C_STS_NO_ERR;
}

/**
  * @brief  Perform Read CCC Operation
  */
int32_t I3C_CtrCCCGet(I3C_DEVICE_T *dev)
{
    uint32_t val;

	/* Initialize command response value */
    dev->cmd_response = I3C_CTRRESP_INITIAL_VALUE;

	/* Program transfer argument */
    val = ( (dev->rx_len << I3C_CMDQUE_DATLEN_Pos) | 
            I3C_CMDATTR_TRANSFER_ARG );
    
    /* Program Defining Byte Value */
    if(dev->is_DB)
        val |= (dev->DB << I3C_CMDQUE_DB_Pos);
    
    I3C_DrvMsg("[ DRV ] [ARG val: 0x%08x] - I3C_CtrCCCGet\n", val);
    dev->port->CMDQUE = val;
    
	/* Program transfer command */
	val = (  I3C_CMDQUE_CP_Msk
           | I3C_CMDQUE_RNW_Msk
           | dev->speed_mode
           | (dev->target_index << I3C_CMDQUE_DEVINDX_Pos)
           | (dev->tx_id << I3C_CMDQUE_TID_Pos)
           | (dev->ccc_code << I3C_CMDQUE_CMD_Pos)
           | I3C_CMDATTR_TRANSFER_CMD );
        
    /* Program Defining Byte Present */
    if(dev->is_DB)
        val |= I3C_CMDQUE_DBP_Msk;

	if(dev->is_last_cmd)
    {
        /* Stop and Response Status on Transfer Completion */
		val |= (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk);
    }
        
    I3C_DrvMsg("[ DRV ] [CMD val: 0x%08x] - I3C_CtrCCCGet\n", val);
    dev->port->CMDQUE = val;

    /* Clear parameters */
    dev->is_last_cmd = TRUE;
    dev->is_HDR_cmd  = FALSE;
    dev->is_DB       = FALSE;

	return I3C_STS_NO_ERR;
}

/**
  * @brief  Perform Private Write Oeration in Controller
  */
int32_t I3C_CtrWrite(I3C_DEVICE_T *dev) 
{
	volatile uint32_t i;
	uint32_t val, *p32Buf;

	/* Initialize command response value */
    dev->cmd_response = I3C_CTRRESP_INITIAL_VALUE;

    val = 0;
    
	/* Try to use SDAP */
	if((dev->tx_len > 0) && (dev->tx_len <= I3C_SDAP_MAX_SIZE)) 
    {
		/* Set transfer argument params */
		switch (dev->tx_len) 
        {
            case 3:
                val |= (1 << (I3C_CMDQUE_BYTESTRB_Pos + 2));
                val |= (dev->tx_buf[2] << I3C_CMDQUE_DATBYTE2_Pos);
            case 2:
                val |= (1 << (I3C_CMDQUE_BYTESTRB_Pos + 1));
                val |= (dev->tx_buf[1] << I3C_CMDQUE_DATBYTE1_Pos);
            case 1:
                val |= (1 << (I3C_CMDQUE_BYTESTRB_Pos + 0));
                val |= (dev->tx_buf[0] << I3C_CMDQUE_DATBYTE0_Pos);
            default:
                break;
		}
		val |= I3C_CMDATTR_SHORT_DATA_ARG;
        
        I3C_DrvMsg("[ DRV ] [SDAP val: 0x%08x] - I3C_CtrWrite\n", val);
        dev->port->CMDQUE = val;

		/* Set transfer command params */
		val = I3C_CMDQUE_SDAP_Msk;
	}
	else if(dev->tx_len > I3C_SDAP_MAX_SIZE) 
    {        
        p32Buf = (uint32_t *)dev->tx_buf;
        
		/* Write bytes to tx port */
        if(dev->is_DMA)
        {
            /* Use Tx PDAM */
            I3C_ConfigTxDMA(dev, (uint32_t)(p32Buf), (uint32_t)(&dev->port->TXRXDAT), dev->tx_len);
        }
        else
        {        
            for(i=0; i<((dev->tx_len + 3) / 4); i++)
            {
                dev->port->TXRXDAT = p32Buf[i];
            }
        }
        
		/* Program transfer argument */
		val = ((dev->tx_len << I3C_CMDQUE_DATLEN_Pos) | I3C_CMDATTR_TRANSFER_ARG);
        
        I3C_DrvMsg("[ DRV ] [TRANS val: 0x%08x] - I3C_CtrWrite\n", val);
        dev->port->CMDQUE = val;
        
		val = 0;
	}
    
	/* Program transfer command */
	val |= (  dev->speed_mode
            | (dev->target_index << I3C_CMDQUE_DEVINDX_Pos)
            | (dev->tx_id << I3C_CMDQUE_TID_Pos)
            | I3C_CMDATTR_TRANSFER_CMD );
    
    /* Program HDR command */
    if(dev->is_HDR_cmd)
    {
        val |= (I3C_CMDQUE_CP_Msk | (dev->ccc_code << I3C_CMDQUE_CMD_Pos));
    }
    
	if(dev->is_last_cmd)
    {
        /* Stop and Response Status on Transfer Completion */
		val |= (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk);
    }
    
    I3C_DrvMsg("[ DRV ] [CMD val: 0x%08x] - I3C_CtrWrite\n", val);
    dev->port->CMDQUE = val;
    
    /* Clear parameters */
    dev->is_last_cmd = TRUE;
    dev->is_HDR_cmd  = FALSE;
    dev->is_DB       = FALSE;

	return I3C_STS_NO_ERR;
}

/**
  * @brief  Perform Private Read Oeration in Controller
  */
int32_t I3C_CtrRead(I3C_DEVICE_T *dev)
{
  uint32_t val;

	/* Initialize command response value */
    dev->cmd_response = I3C_CTRRESP_INITIAL_VALUE;

    /* Program transfer argument */
    val = ( (dev->rx_len << I3C_CMDQUE_DATLEN_Pos) | 
          I3C_CMDATTR_TRANSFER_ARG );
    
    I3C_DrvMsg("[ DRV] [ARG val: 0x%08x] - I3C_CtrRead\n", val);
    dev->port->CMDQUE = val;
    
	/* Program transfer command */
	val = (  I3C_CMDQUE_RNW_Msk
           | dev->speed_mode
           | (dev->target_index << I3C_CMDQUE_DEVINDX_Pos)
           | (dev->tx_id << I3C_CMDQUE_TID_Pos)
           | I3C_CMDATTR_TRANSFER_CMD );

    /* Program HDR command */
    if(dev->is_HDR_cmd)
    {
        val |= (I3C_CMDQUE_CP_Msk | (dev->ccc_code << I3C_CMDQUE_CMD_Pos));
    }

	if(dev->is_last_cmd)
    {
        /* Stop and Response Status on Transfer Completion */
		val |= (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk);
    }
        
    I3C_DrvMsg("[ DRV] [CMD val: 0x%08x] - I3C_CtrRead\n", val);
    dev->port->CMDQUE = val;

    /* Clear parameters */
    dev->is_last_cmd = TRUE;
    dev->is_HDR_cmd  = FALSE;
    dev->is_DB       = FALSE;

	return I3C_STS_NO_ERR;
}

/**
  * @brief  Get In-Band Interrupt Event in Controller
  */
int32_t I3C_CtrGetIBI(I3C_DEVICE_T *dev)
{
	volatile uint32_t i;
	uint32_t ibi_len, ibi_id, *p32Buf, word_cnt;

    if(dev->ibi_status & BIT31)
    {
        I3C_DrvMsg("\n[ DRV] NACK IBI, status 0x%08x\n", dev->ibi_status);
    }
    else
    {
        ibi_len = ((dev->ibi_status & I3C_IBISTS_DATLEN_Msk) >> I3C_IBISTS_DATLEN_Pos);
        ibi_id  = (((dev->ibi_status & I3C_IBISTS_IBIID_Msk) >> I3C_IBISTS_IBIID_Pos) >> 1);
        
        if(ibi_len == 0)
        {
            /* For Hot-Join request or CR operation */
            if(ibi_id == 0x2)
            {
                I3C_DrvMsg("\n[ DRV] Hot-Join ID (0x02) is detected ... process ENTDAA\n");
                
                dev->ibi_type = I3C_IBI_TYPE_HJ;
                dev->ibi_id   = ibi_id;
                dev->ibi_len  = ibi_len;
                
                // perform ENTDAA for new Hot-Join Target and update Target's DA table
                dev->target_index = dev->target_count; // set ENTDAA index to max. target count
                dev->target_count = 1; 
                dev->ccc_code     = I3C_CCC_ENTDAA;
                I3C_CtrDAA(dev);
                if(dev->irq_enable)
                {
#if 0                        
                      // parse RESPRDY INTSTS in IBI interrupt for I3C_CCC_ENTDAA operation
//                    // Process in IRQ
//                    while(dev->cmd_response == I3C_CTRRESP_INITIAL_VALUE) {}
#endif                        
                    while((dev->port->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0) {}
                    dev->cmd_response = dev->port->RESPQUE;
                }
                else             
                {
                    while((dev->port->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0) {}
                    dev->cmd_response = dev->port->RESPQUE;
                }
                if((dev->cmd_response&I3C_CTRRESP_ERRSTS_Msk) == I3C_CTRRESP_NO_ERR)
                {
                    I3C_DrvMsg("\t[ ENTDAA PASS ]\n");
                    
                    //dev->target_index++;;
                    dev->target_count = dev->target_index + 1;
                    i = dev->target_index;
                    I3C_DrvMsg("\tTarget #%d:\n", dev->target_index);
                    dev->target_da[i] = ((dev->port->TGTCHAR[i].DADDR & I3C_TGTCHAR4_DADDR_Msk) >> I3C_TGTCHAR4_DADDR_Pos);
                    I3C_DrvMsg("\t - Provisional ID = 0x%08x%02x \n", dev->port->TGTCHAR[i].PIDMSB, dev->port->TGTCHAR[i].PIDLSB);
                    I3C_DrvMsg("\t - BCR, DCR       = 0x%08x \n", dev->port->TGTCHAR[i].BCRDCR);
                    I3C_DrvMsg("\t - DADDR          = 0x%02x \n", dev->target_da[i]);
                }
                else
                {
                    I3C_DrvMsg("\t[ ENTDAA, error code %d ]\n", (uint32_t)((dev->cmd_response&I3C_CTRRESP_ERRSTS_Msk) >> I3C_CTRRESP_ERRSTS_Pos));
                    
                    dev->ibi_id = 0x0;
                    dev->target_count = dev->target_index; // ENTDAA fail, and restore target count
                }
            }
            else
            {
                I3C_DrvMsg("\n[ DRV] Get Target Addr 0x%02x for CR request ...\n", ibi_id);
                
                dev->ibi_type = I3C_IBI_TYPE_CR;
                dev->ibi_id   = ibi_id;
                dev->ibi_len  = ibi_len;

                // accept Target MR request after Target DA matched
                if(dev->ibi_id != 0)
                {                    
                    // Check if Target's DA matched and send GETACCCR CCC
                    for(i=0; i<7; i++)
                    {
                        if(dev->target_da[i] == ibi_id)
                            break;
                    }
                    if(i >= 7)
                        return I3C_STS_NO_ERR; /* No Target's DA matched */
                        
                    dev->target_index = i;
                    dev->rx_len       = 1;
                    dev->ccc_code     = I3C_CCC_GETACCCR;
                    dev->is_last_cmd  = TRUE;
                    I3C_CtrCCCGet(dev);
                    if(dev->irq_enable)
                    {                    
#if 0                        
                        // parse RESPRDY INTSTS in IBI interrupt for I3C_CCC_GETACCCR operation
                        // Process in IRQ
                        //while(dev->cmd_response == I3C_CTRRESP_INITIAL_VALUE) {}
#endif                        
                        while((dev->port->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0) {}
                        dev->cmd_response = dev->port->RESPQUE;
                    }
                    else             
                    {
                        while((dev->port->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0) {}
                        dev->cmd_response = dev->port->RESPQUE;
                    }
                    if((dev->cmd_response&I3C_CTRRESP_ERRSTS_Msk) == I3C_CTRRESP_NO_ERR)
                    {
                        p32Buf = (uint32_t *)&dev->rx_buf[0];
                        p32Buf[0] = dev->port->TXRXDAT;
                        
                        if(ibi_id == ((p32Buf[0]&0xFF) >> 1))
                        {
                            I3C_DrvMsg("\n[ DRV] [ GETACCCR result ] matched: 0x%02x.\n", ibi_id);
                        }
                        else
                        {
                            I3C_DrvMsg("\n[ DRV] [ GETACCCR result ] mismatch: 0x%02x, 0x%02x\n", ibi_id, ((p32Buf[0]&0xFF) >> 1));
                        }                                
                    }
                    else
                    {
                        I3C_DrvMsg("\n[ DRV] [ GETACCCR, error code %d ]\n", (uint32_t)((dev->cmd_response&I3C_CTRRESP_ERRSTS_Msk) >> I3C_CTRRESP_ERRSTS_Pos)); 
                    }
                }
                                
                // monitor bus owner status
                if(dev->port->INTSTS & I3C_INTSTS_BUSOWNER_Msk)
                {
                    dev->port->INTSTS = I3C_INTSTS_BUSOWNER_Msk;
                    
                    dev->port->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
                    while((dev->port->DEVCTL & I3C_DEVCTL_RESUME_Msk) == I3C_DEVCTL_RESUME_Msk) {}

                    if(I3C_IS_MASTER(dev->port))
                    {
                        I3C_DrvMsg("\tI3C role change from Target to Controller\n");
                    }
                    else
                    {
                        I3C_DrvMsg("\tI3C role change from Controller to Target\n");
                    }
                }           
            }
        }
        else
        {
            /* For In-Band interrupt payload */
            p32Buf   = (uint32_t *)&dev->rx_buf[0];
            word_cnt = (ibi_len + 3) / 4;
            for(i=0; i<word_cnt; i++)
                p32Buf[i] = dev->port->IBIQUE;
            
            I3C_DrvMsg("\n[ DRV ] ibi_id: 0x%02x, len: %d\n\t", ibi_id, ibi_len);
            for(i=0; i<ibi_len; i++)
            {
                I3C_DrvMsg(" 0x%02x", dev->rx_buf[i]);
            }
            I3C_DrvMsg("\n");
            
            dev->ibi_type    = I3C_IBI_TYPE_TIR;
            dev->ibi_id      = ibi_id;
            dev->ibi_len     = ibi_len;
            dev->ibi_payload = p32Buf[0];
       }
    }
    
	return I3C_STS_NO_ERR;
}

/**
  * @brief  Parse Error Condition and Recovery in Controller
  */
void I3C_CtrHandleTransErr(I3C_DEVICE_T *dev)
{
	uint32_t    err_status;
	uint8_t     resume = FALSE;
    uint32_t    TID, LEN;

	err_status = (dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk);
    TID        = (((uint32_t)dev->cmd_response & I3C_CTRRESP_TID_Msk) >> I3C_CTRRESP_TID_Pos);
    LEN        = (((uint32_t)dev->cmd_response & I3C_CTRRESP_DATLEN_Msk) >> I3C_CTRRESP_DATLEN_Pos);
    
    I3C_DrvMsg("[ DRV ] Controller error status 0x%08x.\n", err_status);
    switch(err_status) 
    {
        case I3C_CTRRESP_CRC_ERR:
            I3C_DrvMsg("\t# Transfer Error: CRC Error occurred in the HDR-DDR or HDR-BT Read Transfer \n");
            resume = TRUE;
            break;
        case I3C_CTRRESP_PARITY_ERR:
            I3C_DrvMsg("\t# Transfer Error: Parity Error occurred in HDR Read Transfers \n");
            resume = TRUE;
            break;
        case I3C_CTRRESP_FRAME_ERR:
            I3C_DrvMsg("\t# Transfer Error: Frame Error occurred in HDR Read Transfers \n");;
            resume = TRUE;
            break;
        case I3C_CTRRESP_BRD_ADDR_NACK_ERR:
            I3C_DrvMsg("\t# Transfer Error: I3C Broadcast Address NACK Error \n");
            I3C_DrvMsg("\tTID %d, remaining device count %d\n", TID, LEN);
            if(dev->target_count >= LEN)
                dev->target_count = dev->target_count - LEN;        
            resume = TRUE;
            break;
        case I3C_CTRRESP_ADDR_NACK_ERR:
            I3C_DrvMsg("\t# Transfer Error: Target Address NACK \n");
            resume = TRUE;
            break;
        case I3C_CTRRESP_FLOW_ERR:
            I3C_DrvMsg("\t# Transfer Error: Receive Buffer Overflow/Transmit Buffer Underflow in HDR Transfers \n");
            resume = TRUE;
            break;
        case I3C_CTRRESP_TRANS_ABORTED_ERR:
            I3C_DrvMsg("\t# Transfer Error: Transfer Aborted \n");
            resume = TRUE;
            break;
        case I3C_CTRRESP_WRITE_NACK_ERR:
            I3C_DrvMsg("\t# Transfer Error: I2C Target Write Data NACK Error \n");
            resume = TRUE;
            break;
        case I3C_CTRRESP_DA_MISMATCH_ERR:
            I3C_DrvMsg("\t# Transfer Error: Dynamic Address Mismatch Error in GETACCCR \n");
            resume = TRUE;
            break;
        case I3C_CTRRESP_PEC_ERR:
            I3C_DrvMsg("\t# Transfer Error: PEC byte validation error occurs in read transfers \n");
            resume = TRUE;
            break;
        case I3C_CTRRESP_HDRBT_ERR:
            I3C_DrvMsg("\t# Transfer Error: Delay Byte Count expiry in HDR-BT \n");
            resume = TRUE;
            break;
        default:
            I3C_DrvMsg("\t# Unkown Error \n");
            resume = TRUE;
            break;
	}

	/* Resume Controller if necessary */
	if(resume) 
    {
		I3C_DrvMsg("\tResuming the Controller\n\n");
        dev->port->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
	}
}

static void I3C_TgtResetAndResume(I3C_DEVICE_T *dev, uint8_t ExtCmdIdx)
{
    /* Reset all FIFO -> apply resume */
    dev->port->RSTCTL = (I3C_RSTCTL_RESPRST_Msk | I3C_RSTCTL_RXRST_Msk);
    while(dev->port->RSTCTL != 0) {}
        
    /* Reset Extended Command Transmit Data Buffer */
    if(ExtCmdIdx == 0xFF)
        dev->port->EXTDBRST = 0xFF;
    else
        dev->port->EXTDBRST = (1 << ExtCmdIdx);
    while(dev->port->EXTDBRST != 0) {}
        
    dev->port->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
    while((dev->port->DEVCTL&I3C_DEVCTL_RESUME_Msk) != 0) {}
    I3C_DrvMsg("[ DRV ] Target Reset and Resume Completed.\n");
}

/**
  * @brief  Get Target Response Result in Controller Write Operation
  */
int32_t I3C_TgtRecv(I3C_DEVICE_T *dev)
{
    uint8_t             u8DataLen, u8TargetID, u8CmdSize, u8ErrSts;
    volatile uint16_t   i, RxBufIdx;
    volatile uint32_t   u32RespQ;
    uint32_t            *pu32RxBuf;
        
    dev->tgtRespQ[0].ErrSts = (uint8_t)I3C_TGTRESP_INITIAL_VALUE;
    
    if( !(dev->port->INTSTS & I3C_INTSTS_RESPRDY_Msk) )
        return I3C_STS_RESPQ_EMPTY;
            
    dev->tgtRespQ[0].RxBufAddr = 0;
    dev->tgtRespQ[0].RxBufLen  = 0;
    
    /* Step 1. Get and parse the first RespQ */
    u32RespQ = dev->port->RESPQUE;
    
    if( !(u32RespQ & I3C_TGTRESP_FIRST_Msk) )
        return I3C_STS_RESPQ_NOT_FIRST;
    
    u8DataLen  = ((u32RespQ & I3C_TGTRESP_DATLEN_Msk) >> I3C_TGTRESP_DATLEN_Pos);
    u8TargetID = ((u32RespQ & I3C_TGTRESP_TID_Msk) >> I3C_TGTRESP_TID_Pos);
    u8CmdSize  = ((u32RespQ & I3C_TGTRESP_CMDSIZE_Msk) >> I3C_TGTRESP_CMDSIZE_Pos);
    u8ErrSts   = ((u32RespQ & I3C_TGTRESP_ERRSTS_Msk) >> I3C_TGTRESP_ERRSTS_Pos);
    if(u8ErrSts != I3C_STS_NO_ERR)
    {
        //I3C_DrvMsg("\tError RESPQ: 0x%08x (TID: %d) (L-%d)\n", u32RespQ, u8TargetID, __LINE__);
        return (u32RespQ & I3C_TGTRESP_ERRSTS_Msk);
    }
        
    dev->tgtRespQ[0].IsCCCWrite = ((u32RespQ&I3C_TGTRESP_CCCWR_Msk)==0)? 0:1;
    dev->tgtRespQ[0].TargetID = u8TargetID;
    dev->tgtRespQ[0].ErrSts   = u8ErrSts;
    dev->tgtRespQ[0].CmdSize  = u8CmdSize;
    dev->tgtRespQ[0].CmdWord  = 0;
    
    RxBufIdx  = 0;
    pu32RxBuf = (uint32_t *)dev->rx_buf;
    
    /*
        u8CmdSize 0x0: First RX is the first Data Word of the Transfer.
        u8CmdSize 0x1: Command Word in the First RX FIFO. Only byte-0 is valid for the HDR CMD code or the CCC code.
        u8CmdSize 0x2: Command Word in the First RX FIFO. Only byte-0 for the HDR CMD code or the CCC code and byte-1 for CCC Defining Byte.
        u8CmdSize 0x4: Command Word in the First RX FIFO. All 4-bytes are for HDR-BT Transfer Command words 0, 1, 2 and 3.
    */
    if(dev->is_DMA)
    {
        /* Use Rx PDAM */
        if(u8CmdSize != 0)
        {
            I3C_DrvMsg("[ DRV ] CCC/CMD code: 0x%08x\n", pu32RxBuf[RxBufIdx]);
            
            RxBufIdx++;
        }
    }
    else
    {
        if(u8CmdSize != 0)
        {
            pu32RxBuf[RxBufIdx] = dev->port->TXRXDAT;
            I3C_DrvMsg("[ DRV ] CCC/CMD code: 0x%08x\n", pu32RxBuf[RxBufIdx]);
            
            RxBufIdx++;
        }

        for(i=0; i<((u8DataLen + 3) / 4); i++, RxBufIdx++)
        {
            pu32RxBuf[RxBufIdx] = dev->port->TXRXDAT;
            //if( (u32RespQ & I3C_TGTRESP_CCCWR_Msk) ) // for CCC Write operation
            //    I3C_DrvMsg("\tRX: 0x%08x\n", pu32RxBuf[RxBufIdx]);
        }
    }
    I3C_DrvMsg("[ DRV ] RESPQ: 0x%08x (TID: %d) (#0)\n", u32RespQ, u8TargetID);
    
    if(u8CmdSize != 0)
    {
        dev->tgtRespQ[0].CmdWord = dev->rx_buf[0];
        dev->tgtRespQ[0].RxBufAddr = (uint32_t)(&pu32RxBuf[1]); // the First Data entry in the RX FIFO for the transfer's encoded command word
    }
    else
    {
        dev->tgtRespQ[0].RxBufAddr = (uint32_t)(&pu32RxBuf[0]);
    }
    dev->tgtRespQ[0].RxBufLen = u8DataLen;
        
    if( (u32RespQ & I3C_TGTRESP_LAST_Msk) )
        return I3C_STS_NO_ERR;
    
        
    /* Step 2. Parse the remains RespQ(s) */
    while(dev->port->INTSTS & I3C_INTSTS_RESPRDY_Msk)
    {        
        u32RespQ = dev->port->RESPQUE;
                
        u8DataLen  = ((u32RespQ & I3C_TGTRESP_DATLEN_Msk) >> I3C_TGTRESP_DATLEN_Pos);
        u8TargetID = ((u32RespQ & I3C_TGTRESP_TID_Msk) >> I3C_TGTRESP_TID_Pos);
        //u8CmdSize  = ((u32RespQ & I3C_TGTRESP_CMDSIZE_Msk) >> I3C_TGTRESP_CMDSIZE_Pos);
        u8ErrSts   = ((u32RespQ & I3C_TGTRESP_ERRSTS_Msk) >> I3C_TGTRESP_ERRSTS_Pos);
        if(u8ErrSts != I3C_STS_NO_ERR)
        {
            //I3C_DrvMsg("\tError RESPQ: 0x%08x (TID: %d) (L-%d)\n", u32RespQ, u8TargetID, __LINE__);
            return (u32RespQ & I3C_TGTRESP_ERRSTS_Msk);
        }
        
        if(dev->is_DMA)
        {
            /* Use PDAM RX */
        }
        else
        {
            for(i=0; i<((u8DataLen + 3) / 4); i++, RxBufIdx++)
            {
                pu32RxBuf[RxBufIdx] = dev->port->TXRXDAT;
                //if( (u32RespQ & I3C_TGTRESP_CCCWR_Msk) ) // for CCC Write operation
                //    I3C_DrvMsg("\tRX: 0x%08x\n", pu32RxBuf[RxBufIdx]);
            }
        }
        I3C_DrvMsg("[ DRV ] RESPQ: 0x%08x (TID: %d) (#1)\n", u32RespQ, u8TargetID);
        
        dev->tgtRespQ[0].RxBufLen += u8DataLen;
        
        if( (u32RespQ & I3C_TGTRESP_LAST_Msk) )
            return I3C_STS_NO_ERR;
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief  Prepare Target Extended Transmit Command Data for Controller Read Operation
  */
int32_t I3C_TgtSend(I3C_DEVICE_T *dev)
{
    volatile uint32_t i;
    uint8_t target_idx = dev->target_index;
    uint8_t cmd_idx    = dev->target_extcmd;
    uint8_t ccc_code   = dev->ccc_code;
    uint8_t def_byte   = (dev->is_DB==TRUE)? dev->DB:0x0;
    uint32_t txlen     = dev->tx_len;
	uint32_t *p32Buf;
    
    /* Reset EXT CMD Transmit Data Buffer first */
    dev->port->EXTDBRST = ((1 << cmd_idx));
    while(dev->port->EXTDBRST != 0) {}
    
    /* Push data to EXT CMD TX Buffer */
    p32Buf = (uint32_t *)dev->tx_buf;
    if(dev->is_DMA)
    {
        /* Use Tx PDAM */
        I3C_ConfigTxDMA(dev, (uint32_t)(p32Buf), (uint32_t)&dev->port->EXTDAT[cmd_idx], txlen);
    }
    else
    {        
        for(i=0; i<((txlen+3) / 4); i++)
            dev->port->EXTDAT[cmd_idx] = p32Buf[i];
    }
        
    if(ccc_code == 0)
    {
        /* For SDR read transfer */
        
        dev->port->EXTCMD[cmd_idx].WORD2 = (txlen << I3C_TGTCMDW2_DATALEN_Pos);
        
        dev->port->EXTCMD[cmd_idx].WORD3 = 0; // For HDR-BT Transfer Read command
        
        dev->port->EXTCMD[cmd_idx].WORD1 = 
            ( (target_idx << I3C_TGTCMDW1_ADDRIDX_Pos) | 
              I3C_TGTCMDW1_CMDVLD_Msk | 
              I3C_TGTCMDW1_FINITEDL_Msk | 
              I3C_TGTCMDW1_CMDEN );
    }
    else
    {
        /* For CCC, HDR-DDR and BT read transfer */
        
        if(dev->is_HDRBT_cmd == TRUE)
        {
            dev->port->EXTCMD[cmd_idx].WORD2 = dev->HDRBT_cmd;
            
            dev->port->EXTCMD[cmd_idx].WORD3 = (txlen << I3C_TGTCMDW3_BTDL_Pos); 
        }
        else
        {
            if(dev->is_HDR_cmd == TRUE)
                ccc_code = dev->HDR_cmd;
        
            dev->port->EXTCMD[cmd_idx].WORD2 = 
                ( (txlen << I3C_TGTCMDW2_DATALEN_Pos) | 
                  (def_byte << I3C_TGTCMDW2_DEFBYTE_Pos) | 
                  (ccc_code << I3C_TGTCMDW2_CCCHEAD_Pos) );
            
            dev->port->EXTCMD[cmd_idx].WORD3 = 0; // For HDR-BT Transfer Read command
        }
               
        dev->port->EXTCMD[cmd_idx].WORD1 = 
            ( (target_idx << I3C_TGTCMDW1_ADDRIDX_Pos) | 
              ((dev->is_HDR_cmd==TRUE)? 0x0:I3C_TGTCMDW1_CCC_Msk) | 
              I3C_TGTCMDW1_CMDVLD_Msk | 
              ((dev->is_HDRBT_cmd==TRUE)? I3C_TGTCMDW1_BTDLEN_Msk:0x0) | 
              I3C_TGTCMDW1_FINITEDL_Msk | 
              I3C_TGTCMDW1_CMDEN );
    }    
        
    /* 
        User need to parse Extended Command Status in EXTCMD.WORD1[15:8] while EXTFINS set to 1 (EXTCMD Has Finished Status).
        Refer to I3C_TgtGetSendResult(...) API.
    */ 
    
    dev->ccc_code     = 0x0;
    dev->is_DB        = FALSE;
    dev->is_HDR_cmd   = FALSE;
    dev->HDR_cmd      = 0x0;
    dev->is_HDRBT_cmd = FALSE;
    dev->HDRBT_cmd    = 0x0;
    
	return I3C_STS_NO_ERR;
}

/**
  * @brief  Get Target Extended Transmit Command Result in Controller Read Operation
  */
int32_t I3C_TgtGetSendResult(I3C_DEVICE_T *dev)
{
    volatile uint32_t i, u32CMDDoneBitMsk;
    uint32_t CmdRegVal[3], err_sts, dat_len, dev_sts;
        
    if( !(dev->port->INTSTS & I3C_INTSTS_EXTFINS_Msk) )
    {
        return I3C_STS_INVALID_STATE;
    }
        
    u32CMDDoneBitMsk = dev->port->EXTCMDFS;

    for(i=0; i<8; i++)
    {
        if(u32CMDDoneBitMsk & (1<<i))
        {
            CmdRegVal[0] = dev->port->EXTCMD[i].WORD1;
            CmdRegVal[1] = dev->port->EXTCMD[i].WORD2;
            CmdRegVal[2] = dev->port->EXTCMD[i].WORD3;
            
            I3C_DrvMsg("[ DRV ] EXT CMD %d Reg:\n", i);
            I3C_DrvMsg("\tReg 0: 0x%08x\n", CmdRegVal[0]);
            I3C_DrvMsg("\tReg 1: 0x%08x\n", CmdRegVal[1]);
            I3C_DrvMsg("\tReg 2: 0x%08x\n", CmdRegVal[2]);
            I3C_DrvMsg("\t====================\n");
            
            err_sts = (CmdRegVal[0] & I3C_TGTCMDW1_ERRSTS_Msk);
            dat_len = ((CmdRegVal[1] & I3C_TGTCMDW2_DATALEN_Msk) >> I3C_TGTCMDW2_DATALEN_Pos);
            if(err_sts == I3C_TGTCMDSTS_NO_ERR)
            {
                I3C_DrvMsg("\t[ EXTCMD no ERROR ]\n");
                I3C_DrvMsg("\t[ %s Read ]\n", ((CmdRegVal[0]&I3C_TGTCMDW1_CCC_Msk)==0)? "Private":"CCC Direct");
                
                if(CmdRegVal[0] & I3C_TGTCMDW1_BTDLEN_Msk)
                {
                    I3C_DrvMsg("\t[ HDR-BT transfer ]\n");
                    dat_len = ((CmdRegVal[2] & I3C_TGTCMDW3_BTDL_Msk) >> I3C_TGTCMDW3_BTDL_Pos);
                }
                
                if(CmdRegVal[0] & I3C_TGTCMDW1_FINITEDL_Msk) //Finite length transfer
                    I3C_DrvMsg("\t[ Remains %d-bytes ]\n", dat_len);
                else
                    I3C_DrvMsg("\t[ Transfer %d-bytes ]\n", dat_len);
            }
            else
            {                
                I3C_DrvMsg("\t[ EXTCMD ERROR Status : 0x%08x ] (CMD W1: 0x%08x)\n", err_sts, CmdRegVal[0]);
                switch(err_sts) 
                {
                    case I3C_TGTCMDSTS_CRC_ERR:
                        I3C_DrvMsg("\t# CRC Error \n");
                        break;
                    case I3C_TGTCMDSTS_PARITY_ERR:
                        I3C_DrvMsg("\t# Parity Error \n");
                        break;
                    case I3C_TGTCMDSTS_FRAME_ERR:
                        I3C_DrvMsg("\t# Frame Error \n");;
                        break;
                    case I3C_TGTCMDSTS_FLOW_ERR:
                        I3C_DrvMsg("\t# Underflow/Overflow Error \n");
                        break;
                    case I3C_TGTCMDSTS_SDA_STUCK_ERR:
                        I3C_DrvMsg("\t# SDA Stuck Error \n");
                        break;
                    case I3C_TGTCMDSTS_MASTER_TERMINATE_ERR:
                        I3C_DrvMsg("\t# Master early terminal Error \n");
                        break;
                    case I3C_TGTCMDSTS_PEC_ERR:
                        I3C_DrvMsg("\t# PEC Error \n");
                        break;
                    default:
                        I3C_DrvMsg("# Unkown Error \n");
                        break;
                }
                
                dev_sts = dev->port->CCCDEVS;
                I3C_DrvMsg("\t[  Device Operating Status : 0x%08x ]\n", dev_sts);
                if(dev_sts != 0)
                {
                    if(dev_sts & I3C_CCCDEVS_PROTERR_Msk)
                        I3C_DrvMsg("\t# Protocol Error \n");
                    if(dev_sts & I3C_CCCDEVS_UDFERR_Msk)
                        I3C_DrvMsg("\t# Underflow Error \n");
                    if(dev_sts & I3C_CCCDEVS_OVFERR_Msk)
                        I3C_DrvMsg("\t# Overflow Error \n");
                    if(dev_sts & I3C_CCCDEVS_DATNRDY_Msk)
                        I3C_DrvMsg("\t# Data Not Ready \n");
                    if(dev_sts & I3C_CCCDEVS_BFNAVAIL_Msk)
                        I3C_DrvMsg("\t# Buffer Not Available \n");
                    if(dev_sts & I3C_CCCDEVS_FRAMEERR_Msk)
                        I3C_DrvMsg("\t# Frame Error \n");
                    if(dev_sts & I3C_CCCDEVS_SLVBUSY_Msk)
                    {
                        I3C_DrvMsg("\t# Target Busy \n");
                        I3C_TgtResetAndResume(dev, i);
                    }            
                }                

                I3C_DrvMsg("\n");
            }
        }
    }
    
    I3C_DrvMsg("\n");
    
	return I3C_STS_NO_ERR;
}

/**
  * @brief  Issue In-Band Interrupt Event in Target
  */
int32_t I3C_TgtIssueIBI(I3C_DEVICE_T *dev)
{   
    if((dev->port->SLVEVNTS & I3C_SLVEVNTS_SIREN_Msk) == 0)
    {
        I3C_DrvMsg("\n[ DRV ] ERROR. Target Interrupt Request NOT Enabled. \n\n");
        return I3C_STS_INVALID_INPUT;
    }
    if((dev->port->SIR & (I3C_SIR_EN_Msk | I3C_SIR_MR_Msk)) != 0)
    {
        I3C_DrvMsg("\n[ DRV ] ERROR. SIR pending, 0x%x. \n\n", dev->port->SIR);
        return I3C_STS_INVALID_INPUT;
    }
    if(dev->target_index > 4)
    {
        I3C_DrvMsg("\n[ DRV ] ERROR. Invalid Target index, 0x%x. \n\n", dev->target_index);
        return I3C_STS_INVALID_INPUT;
    }    
    
    switch(dev->ibi_type)
    {
        case I3C_IBI_TYPE_TS:
            /* Timestamp Counter Auto Mode Enable */
            dev->port->DEVCTL |= (I3C_DEVCTL_TSCAUTO_Msk); 
        case I3C_IBI_TYPE_TIR:
            break;
        
        case I3C_IBI_TYPE_CR:
            if((dev->port->SLVEVNTS & I3C_SLVEVNTS_MREN_Msk) == 0)
            {
                I3C_DrvMsg("\n[ DRV ] ERROR. Master Request NOT Enabled.\n\n");
                return I3C_STS_INVALID_INPUT;
            }
            
            /* Support "ACK GETACCCR CCC" */ 
            dev->port->DEVCTLE &= ~(I3C_DEVCTLE_MRACKCTL_Msk);
                    
            /* [2:1] 1 = Pull SDA line low and release SDA after sampling SCL low (pulled by Controller). Used for CE3 recovery during Controller Handover procedure. */
            dev->port->SIR = ((dev->target_index << I3C_SIR_TGTIDX_Pos) | (1 << I3C_SIR_CTL_Pos));
            
            /* Trigger MR request */
            dev->port->SIR |= I3C_SIR_MR_Msk;
            
            /* Bus Owner Updated event in Target's IRQ Handler */
            
            /* The Controller sends GETACCCR CCC (Get Accept Controller Role) while received Controller request from Target */

            return I3C_STS_NO_ERR;
            
            break;
                
        default:
            return I3C_STS_INVALID_INPUT;
            break;
    }
    
    /* Check if payload length > 4-bytes */
    if(dev->ibi_len > 4)
        return I3C_STS_INVALID_INPUT;
    
    /* Program IBI payload data, payload length and MDB */
    dev->port->SIR    = ((dev->ibi_len << I3C_SIR_DATLEN_Pos) | (dev->ibi_MDB << I3C_SIR_MDB_Pos) | (0 << I3C_SIR_CTL_Pos));
    dev->port->SIRDAT = dev->ibi_payload;
    
    /* Trigger IBI request */
    /* SIR EN bit be cleared automatically after the Controller accepts the IBI request or Target unable to issue the IBI request */
    if(dev->ibi_type == I3C_IBI_TYPE_TS)
        dev->port->SIR |= ((dev->target_index << I3C_SIR_TGTIDX_Pos) | I3C_SIR_EN_Msk | I3C_SIR_TS_Msk);
    else
        dev->port->SIR |= ((dev->target_index << I3C_SIR_TGTIDX_Pos) | I3C_SIR_EN_Msk);
            
        
	return I3C_STS_NO_ERR;
}

/**
  * @brief  Parse Error Condition and Recovery in Target
  */
void I3C_TgtHandleTransErr(I3C_DEVICE_T *dev)
{
    uint32_t dev_status, err_status;
    uint8_t resume = FALSE;

    dev_status = dev->port->CCCDEVS;
    
    I3C_DrvMsg("[ DRV ] Target device status 0x%08x.\n", dev_status);
	if(dev_status) 
    {
		if(dev_status & I3C_CCCDEVS_SLVBUSY_Msk) 
        {
			I3C_DrvMsg("\t#Target busy status\n");
			resume = TRUE;
		}
		if(dev_status & I3C_CCCDEVS_FRAMEERR_Msk)
			I3C_DrvMsg("\t# Dev frame error\n");
		if(dev_status & I3C_CCCDEVS_BFNAVAIL_Msk)
			I3C_DrvMsg("\t# Dev buffer not available\n");
		if(dev_status & I3C_CCCDEVS_DATNRDY_Msk)
			I3C_DrvMsg("\t# Dev data not ready\n");
		if(dev_status & I3C_CCCDEVS_OVFERR_Msk)
			I3C_DrvMsg("\t# Dev overflow error\n");
		if(dev_status & I3C_CCCDEVS_UDFERR_Msk)
			I3C_DrvMsg("\t# Dev underflow error\n");
		if(dev_status & I3C_CCCDEVS_PROTERR_Msk)
			I3C_DrvMsg("\t# Dev protocol error\n");

		I3C_DrvMsg("\t# Activity Mode:[0x%x] / Pending Interrupt:[0x%x]",
            (uint32_t)((dev_status&I3C_CCCDEVS_ACTMODE_Msk) >> I3C_CCCDEVS_ACTMODE_Pos),
            (uint32_t)((dev_status&I3C_CCCDEVS_PENDINT_Msk) >> I3C_CCCDEVS_PENDINT_Pos));

        /* Resume Target if necessary */
        if(resume) 
        {
            I3C_DrvMsg("\tResuming the Target\n\n");
            
            dev->port->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
        }
	}
    
    err_status = dev->tgtRespQ[0].ErrSts;
    I3C_DrvMsg("[ DRV ] Target response error status 0x%08x.\n", err_status);
    switch(err_status) 
    {
        case I3C_TGTRESP_CRC_ERR:
            I3C_DrvMsg("\t# Transfer Error: CRC Error (Controller write in DDR mode) \n");
            break;
        case I3C_TGTRESP_PARITY_ERR:
            I3C_DrvMsg("\t# Transfer Error: Parity Error (Controller write in both DDR and SDR mode) \n");
            break;
        case I3C_TGTRESP_FRAME_ERR:
            I3C_DrvMsg("\t# Transfer Error: Frame Error (Controller write in HDR mode) \n");;
            break;
        case I3C_TGTRESP_FLOW_ERR:
            I3C_DrvMsg("\t# Transfer Error: Underflow/Overflow Error \n");
            break;
        case I3C_TGTRESP_SDA_STUCK_ERR:
            I3C_DrvMsg("\t# Transfer Error: SDA Stuck Error \n");
            break;
        case I3C_TGTRESP_MASTER_TERMINATE_ERR:
            I3C_DrvMsg("\t# Transfer Error: Master early terminal Error \n");
            break;
        case I3C_TGTRESP_PEC_ERR:
            I3C_DrvMsg("\t# Transfer Error: PEC Error \n");
            break;
        default:
            I3C_DrvMsg("\t# Unkown Error \n");
            break;
	}
    
    I3C_DrvMsg("\n");
}

/**
  * @brief  Handle Target Interrupt Status
  */
void I3C_TgtHandleIntSts(I3C_DEVICE_T *dev)
{
    volatile uint32_t i, idx;
    
    if(dev->intsts & I3C_INTSTS_DA_ASSIGNED)
    {
        /* Main Target Address */
        if(dev->port->DEVADDR & I3C_DEVADDR_DAVALID_Msk)
        {
            I3C_DrvMsg("[ DRV ] Set to I3C mode, DA: 0x%02x.\n", (uint8_t)((dev->port->DEVADDR&I3C_DEVADDR_DA_Msk) >> I3C_DEVADDR_DA_Pos));
            dev->target_da[0] = (uint8_t)((dev->port->DEVADDR&I3C_DEVADDR_DA_Msk) >> I3C_DEVADDR_DA_Pos);
        }
        else
        {
            I3C_DrvMsg("[ DRV ] Set to I2C mode, SA: 0x%02x.\n", (uint8_t)((dev->port->DEVADDR&I3C_DEVADDR_SA_Msk) >> I3C_DEVADDR_SA_Pos));
            dev->target_da[0] = 0x0;
            dev->target_sa[0] = (uint8_t)((dev->port->DEVADDR&I3C_DEVADDR_SA_Msk) >> I3C_DEVADDR_SA_Pos);
        }
        
        /* Virtual Target 1~4 Address */
        for(i=0; i<4; i++)
        {
            idx = i + 1;
            if(dev->port->VTGTCFG[i].ADDR & I3C_VTGTADDR_DAVALID_Msk)
            {
                I3C_DrvMsg("[ DRV ] Set to I3C mode, DA: 0x%02x. (VT%d)\n", (uint8_t)((dev->port->VTGTCFG[i].ADDR&I3C_VTGTADDR_DADDR_Msk)>>I3C_VTGTADDR_DADDR_Pos), idx);
                dev->target_da[idx] = (uint8_t)((dev->port->VTGTCFG[i].ADDR&I3C_VTGTADDR_DADDR_Msk) >> I3C_VTGTADDR_DADDR_Pos);
            }
            else
            {
                I3C_DrvMsg("[ DRV ] Set to I2C mode, SA: 0x%02x. (VT%d)\n", (uint8_t)((dev->port->VTGTCFG[i].ADDR&I3C_VTGTADDR_SADDR_Msk)>>I3C_VTGTADDR_SADDR_Pos), idx);
                dev->target_da[idx] = 0;
                dev->target_sa[idx] = (uint8_t)((dev->port->VTGTCFG[i].ADDR&I3C_VTGTADDR_SADDR_Msk) >> I3C_VTGTADDR_SADDR_Pos);
            }
        }
    }
    
    if(dev->intsts & I3C_INTSTS_GRPDA_ASSIGNED)
    {
        /* Group Address 0~3 */
        for(i=0; i<4; i++)
        {
            idx = i + 5;
            if((dev->port->GRPASTS[i] & (0x1F<<I3C_GRPASTS_GRPAVLD0_Pos)) != 0 )
            {
                dev->target_da[idx] = (uint8_t)(dev->port->GRPASTS[i] & I3C_GRPASTS_GRPADDR_Msk);
                I3C_DrvMsg("[ DRV ] Group-%d Addr 0x%02x ... Enabled to 0x%x.\n", i, dev->target_da[idx], (dev->port->GRPASTS[i]>>I3C_GRPASTS_GRPAVLD0_Pos));
            }
            else
            {
                dev->target_da[idx] = 0;
                I3C_DrvMsg("[ DRV ] Group-%d Addr 0x%02x ... Disabled.\n", i, dev->target_da[idx]);
            }
        }
    }
    
    if(dev->intsts & I3C_INTSTS_CCC_UPDATED)
    {
        if(dev->port->SLVEVNTS & I3C_SLVEVNTS_MWLUPD_Msk)
        {
            dev->port->SLVEVNTS = I3C_SLVEVNTS_MWLUPD_Msk;
            I3C_DrvMsg("[ DRV ] Updated MWL to 0x%x.\n", (uint32_t)((dev->port->SLVMXLEN&I3C_SLVMXLEN_MWL_Msk) >> I3C_SLVMXLEN_MWL_Pos));
        }
        else if(dev->port->SLVEVNTS & I3C_SLVEVNTS_MRLUPD_Msk)
        {
            dev->port->SLVEVNTS = I3C_SLVEVNTS_MRLUPD_Msk;
            I3C_DrvMsg("[ DRV ] Updated MRL to 0x%x. IBI len %d.\n", 
                (uint32_t)((dev->port->SLVMXLEN&I3C_SLVMXLEN_MRL_Msk) >> I3C_SLVMXLEN_MRL_Pos),
                (uint32_t)((dev->port->DEVCTL&I3C_DEVCTL_IBIPSIZE_Msk) >> I3C_DEVCTL_IBIPSIZE_Pos));
            
            I3C_TgtResetAndResume(dev, 0xFF);
        }
        else 
        {
            I3C_DrvMsg("[ DRV ] Updated - ENTAS%d.\n", (uint32_t)((dev->port->SLVEVNTS&I3C_SLVEVNTS_ACTSTSTS_Msk) >> I3C_SLVEVNTS_ACTSTSTS_Pos));
            I3C_DrvMsg("[ DRV ] Updated - HJEN %d.\n", (uint32_t)((dev->port->SLVEVNTS&I3C_SLVEVNTS_HJEN_Msk) >> I3C_SLVEVNTS_HJEN_Pos));
            I3C_DrvMsg("[ DRV ] Updated - SIREN %d.\n", (uint32_t)((dev->port->SLVEVNTS&I3C_SLVEVNTS_SIREN_Msk) >> I3C_SLVEVNTS_SIREN_Pos));
        }
    }

    I3C_DrvMsg("\n");
}

/** @} end of group I3C_EXPORTED_FUNCTIONS */
/** @} end of group I3C_Driver */
/** @} end of group Standard_Driver */
