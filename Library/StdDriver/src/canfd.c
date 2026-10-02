/****************************************************************************
 * @file     canfd.c
 * @version  V1.00
 * @brief    CANFD driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NuMicro.h"
#include "string.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Minimum number of time quanta in a bit. */
#define MIN_TIME_QUANTA    9UL
/* Maximum number of time quanta in a bit. */
#define MAX_TIME_QUANTA    20UL
/* Number of receive FIFOs (1 - 2) */
#define CANFD_NUM_RX_FIFOS  2UL

/*CANFD max nominal bit rate*/
#define MAX_NOMINAL_BAUDRATE (1000000UL)
#define MAX_DATA_BAUDRATE    (8000000UL)

/* Tx Event FIFO Element ESI(Error State Indicator)  */
#define TX_FIFO_E0_EVENT_ESI_Pos   (31)
#define TX_FIFO_E0_EVENT_ESI_Msk   (0x1UL << TX_FIFO_E0_EVENT_ESI_Pos)

/* Tx Event FIFO Element XTD(Extended Identifier)    */
#define TX_FIFO_E0_EVENT_XTD_Pos   (30)
#define TX_FIFO_E0_EVENT_XTD_Msk   (0x1UL << TX_FIFO_E0_EVENT_XTD_Pos)

/* Tx Event FIFO Element RTR(Remote Transmission Request)    */
#define TX_FIFO_E0_EVENT_RTR_Pos   (29)
#define TX_FIFO_E0_EVENT_RTR_Msk   (0x1UL << TX_FIFO_E0_EVENT_RTR_Pos)

/* Tx Event FIFO Element ID(Identifier)    */
#define TX_FIFO_E0_EVENT_ID_Pos    (0)
#define TX_FIFO_E0_EVENT_ID_Msk    (0x1FFFFFFFUL << TX_FIFO_E0_EVENT_ID_Pos)

/* Tx Event FIFO Element MM(Message Marker)    */
#define TX_FIFO_E1_EVENT_MM_Pos    (24)
#define TX_FIFO_E1_EVENT_MM_Msk    (0xFFUL << TX_FIFO_E1_EVENT_MM_Pos)

/* Tx Event FIFO Element ET(Event Type)    */
#define TX_FIFO_E1_EVENT_ET_Pos    (22)
#define TX_FIFO_E1_EVENT_ET_Msk    (0x3UL << TX_FIFO_E1_EVENT_ET_Pos)

/* Tx Event FIFO Element FDF(FD Format)    */
#define TX_FIFO_E1_EVENT_FDF_Pos    (21)
#define TX_FIFO_E1_EVENT_FDF_Msk    (0x1UL << TX_FIFO_E1_EVENT_FDF_Pos)

/* Tx Event FIFO Element BRS(Bit Rate Switch)    */
#define TX_FIFO_E1_EVENT_BRS_Pos    (20)
#define TX_FIFO_E1_EVENT_BRS_Msk    (0x1UL << TX_FIFO_E1_EVENT_BRS_Pos)

/* Tx Event FIFO Element DLC(Data Length Code)    */
#define TX_FIFO_E1_EVENT_DLC_Pos    (16)
#define TX_FIFO_E1_EVENT_DLC_Msk    (0xFUL << TX_FIFO_E1_EVENT_DLC_Pos)

/* Tx Event FIFO Element TXTS(Tx Timestamp)    */
#define TX_FIFO_E1A_EVENT_TXTS_Pos  (0)
#define TX_FIFO_E1A_EVENT_TXTS_Msk  (0xFFFFUL << TX_FIFO_E1A_EVENT_TXTS_Pos)

/* Tx Event FIFO Element MM(Message Marker)    */
#define TX_FIFO_E1B_EVENT_MM_Pos    (8)
#define TX_FIFO_E1B_EVENT_MM_Msk    (0xFFUL << TX_FIFO_E1B_EVENT_MM_Pos)

/* Tx Event FIFO Element TSC(Timestamp Captured)    */
#define TX_FIFO_E1B_EVENT_TSC_Pos   (4)
#define TX_FIFO_E1B_EVENT_TSC_Msk   (0x1UL << TX_FIFO_E1B_EVENT_TSC_Pos)

/* Tx Event FIFO Element TSC(Timestamp Captured)    */
#define TX_FIFO_E1B_EVENT_TXTS_Pos   (0)
#define TX_FIFO_E1B_EVENT_TXTS_Msk   (0xFUL << TX_FIFO_E1B_EVENT_TSC_Pos)

/* Rx Buffer and FIFO Element ESI2(Error State Indicator)    */
#define RX_BUFFER_AND_FIFO_R0_ELEM_ESI_Pos  (31)
#define RX_BUFFER_AND_FIFO_R0_ELEM_ESI_Msk  (0x1UL << RX_BUFFER_AND_FIFO_R0_ELEM_ESI_Pos)

/* Rx Buffer and FIFO Element XTD(Extended Identifier)    */
#define RX_BUFFER_AND_FIFO_R0_ELEM_XTD_Pos  (30)
#define RX_BUFFER_AND_FIFO_R0_ELEM_XTD_Msk  (0x1UL << RX_BUFFER_AND_FIFO_R0_ELEM_XTD_Pos)

/* Rx Buffer and FIFO Element RTR(Remote Transmission Request)    */
#define RX_BUFFER_AND_FIFO_R0_ELEM_RTR_Pos  (29)
#define RX_BUFFER_AND_FIFO_R0_ELEM_RTR_Msk  (0x1UL << RX_BUFFER_AND_FIFO_R0_ELEM_RTR_Pos)

/* Rx Buffer and FIFO Element ID(Identifier)    */
#define RX_BUFFER_AND_FIFO_R0_ELEM_ID_Pos  (0)
#define RX_BUFFER_AND_FIFO_R0_ELEM_ID_Msk  (0x1FFFFFFFUL << RX_BUFFER_AND_FIFO_R0_ELEM_ID_Pos)

/* Rx Buffer and FIFO Element ANMF(Accepted Non-matching Frame)    */
#define RX_BUFFER_AND_FIFO_R1_ELEM_ANMF_Pos  (31)
#define RX_BUFFER_AND_FIFO_R1_ELEM_ANMF_Msk  (0x1UL << RX_BUFFER_AND_FIFO_R1_ELEM_ANMF_Pos)

/* Rx Buffer and FIFO Element FIDX(Filter Index)    */
#define RX_BUFFER_AND_FIFO_R1_ELEM_FIDX_Pos  (24)
#define RX_BUFFER_AND_FIFO_R1_ELEM_FIDX_Msk  (0x7FUL << RX_BUFFER_AND_FIFO_R1_ELEM_FIDX_Pos)

/* Rx Buffer and FIFO Element FDF(FD Format)    */
#define RX_BUFFER_AND_FIFO_R1_ELEM_FDF_Pos  (21)
#define RX_BUFFER_AND_FIFO_R1_ELEM_FDF_Msk  (0x1UL << RX_BUFFER_AND_FIFO_R1_ELEM_FDF_Pos)

/* Rx Buffer and FIFO Element BRS(Bit Rate Swit)    */
#define RX_BUFFER_AND_FIFO_R1_ELEM_BSR_Pos  (20)
#define RX_BUFFER_AND_FIFO_R1_ELEM_BSR_Msk  (0x1UL << RX_BUFFER_AND_FIFO_R1_ELEM_BSR_Pos)

/* Rx Buffer and FIFO Element DLC(Bit Rate Swit)    */
#define RX_BUFFER_AND_FIFO_R1_ELEM_DLC_Pos  (16)
#define RX_BUFFER_AND_FIFO_R1_ELEM_DLC_Msk  (0xFUL << RX_BUFFER_AND_FIFO_R1_ELEM_DLC_Pos)

/* Rx Buffer and FIFO Element RXTS(Rx Timestamp)    */
#define RX_BUFFER_AND_FIFO_R1_ELEM_RXTS_Pos  (0)
#define RX_BUFFER_AND_FIFO_R1_ELEM_RXTS_Msk  (0xFFFFUL << RX_BUFFER_AND_FIFO_R1_ELEM_RXTS_Pos)

/* Tx Buffer Element ESI(Error State Indicator)    */
#define TX_BUFFER_T0_ELEM_ESI_Pos  (31)
#define TX_BUFFER_T0_ELEM_ESI_Msk  (0x1UL << TX_BUFFER_T0_ELEM_ESI_Pos)

/* Tx Buffer Element XTD(Extended Identifier)    */
#define TX_BUFFER_T0_ELEM_XTD_Pos  (30)
#define TX_BUFFER_T0_ELEM_XTD_Msk (0x1UL << TX_BUFFER_T0_ELEM_XTD_Pos)

/* Tx Buffer RTR(Remote Transmission Request)    */
#define TX_BUFFER_T0_ELEM_RTR_Pos  (29)
#define TX_BUFFER_T0_ELEM_RTR_Msk  (0x1UL << TX_BUFFER_T0_ELEM_RTR_Pos)

/* Tx Buffer Element ID(Identifier)    */
#define TX_BUFFER_T0_ELEM_ID_Pos  (0)
#define TX_BUFFER_T0_ELEM_ID_Msk  (0x1FFFFFFFUL << TX_BUFFER_T0_ELEM_ID_Pos)

/* Tx Buffer Element MM(Message Marker)    */
#define TX_BUFFER_T1_ELEM_MM1_Pos  (24)
#define TX_BUFFER_T1_ELEM_MM1_Msk  (0xFFUL << TX_BUFFER_T1_ELEM_MM1_Pos)

/* Tx Buffer Element EFC(Event FIFO Control)    */
#define TX_BUFFER_T1_ELEM_EFC_Pos  (23)
#define TX_BUFFER_T1_ELEM_EFC_Msk  (0xFFUL << TX_BUFFER_T1_ELEM_EFC_Pos)

/* Tx Buffer Element TSCE(Time Stamp Capture Enable for TSU)    */
#define TX_BUFFER_T1_ELEM_TSCE_Pos  (22)
#define TX_BUFFER_T1_ELEM_TSCE_Msk  (0x1UL << TX_BUFFER_T1_ELEM_TSCE_Pos)

/* Tx Buffer Element FDF(FD Format)    */
#define TX_BUFFER_T1_ELEM_FDF_Pos  (21)
#define TX_BUFFER_T1_ELEM_FDF_Msk  (0x1UL << TX_BUFFER_T1_ELEM_FDF_Pos)

/* Tx Buffer Element BRS(Bit Rate Swit)    */
#define TX_BUFFER_T1_ELEM_BSR_Pos  (20)
#define TX_BUFFER_T1_ELEM_BSR_Msk  (0x1UL << TX_BUFFER_T1_ELEM_BSR_Pos)

/* Tx Buffer Element DLC(Bit Rate Swit)    */
#define TX_BUFFER_T1_ELEM_DLC_Pos  (16)
#define TX_BUFFER_T1_ELEM_DLC_Msk  (0xFUL << TX_BUFFER_T1_ELEM_DLC_Pos)

/* Tx Buffer Element MM(Message Marker)    */
#define TX_BUFFER_T1_ELEM_MM0_Pos  (8)
#define TX_BUFFER_T1_ELEM_MM0_Msk  (0xFFUL << TX_BUFFER_T1_ELEM_MM0_Pos)

#define CANFD_RXFS_RFL CANFD_RXF0S_RF0L_Msk

/* CANFD Normal Bit-Rate Parameter */
#define N_TSEG1_MIN 2UL
#define N_TSEG1_MAX 256UL
#define N_TSEG2_MIN 2UL
#define N_TSEG2_MAX 128UL
#define N_BRP_MIN   1UL
#define N_BRP_MAX   512UL
#define N_SJW_MAX   128UL
#define N_BRP_INC   1UL

/* CANFD Data Bit-Rate Parameter */
#define D_TSEG1_MIN 1UL
#define D_TSEG1_MAX 32UL
#define D_TSEG2_MIN 1UL
#define D_TSEG2_MAX 16UL
#define D_BRP_MIN   1UL
#define D_BRP_MAX   32UL
#define D_SJW_MAX   16UL
#define D_BRP_INC   1UL

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup CANFD_Driver CANFD Driver
  @{
*/

/** @addtogroup CANFD_EXPORTED_FUNCTIONS CANFD Exported Functions
  @{
*/

static void CANFD_InitRxFifo(CANFD_T *psCanfd, const uint32_t u32RxFifoNum, const CANFD_RAM_PART_T *psRamConfig, const CANFD_ELEM_SIZE_T *psElemSize, uint32_t u32FifoWM, const E_CANFD_DATA_FIELD_SIZE eFifoSize);
static void CANFD_InitRxDBuf(CANFD_T *psCanfd, const CANFD_RAM_PART_T *psRamConfig, const CANFD_ELEM_SIZE_T *psElemSize, E_CANFD_DATA_FIELD_SIZE eRxBufSize);
static void CANFD_InitTxDBuf(CANFD_T *psCanfd, const CANFD_RAM_PART_T *psRamConfig, const CANFD_ELEM_SIZE_T *psElemSize, E_CANFD_DATA_FIELD_SIZE eTxBufSize);
static void CANFD_InitTxEvntFifo(CANFD_T *psCanfd, const CANFD_RAM_PART_T *psRamConfig, const CANFD_ELEM_SIZE_T *psElemSize, uint32_t u32FifoWaterLvl);
static void CANFD_ConfigSIDFC(CANFD_T *psCanfd, CANFD_RAM_PART_T *psRamConfig, const CANFD_ELEM_SIZE_T *psElemSize);
static void CANFD_ConfigXIDFC(CANFD_T *psCanfd, CANFD_RAM_PART_T *psRamConfig, const CANFD_ELEM_SIZE_T *psElemSize);

/**
 * @brief       Calculates the CAN FD RAM buffer address.
 *
 * @param[in]   psConfigAddr  CAN FD element star address structure.
 * @param[in]   psConfigSize  CAN FD element size structure.
 *
 * @return      None.
 *
 * @details     Calculates the CAN FD RAM buffer address.
 */
static void CANFD_CalculateRamAddress(CANFD_RAM_PART_T *psConfigAddr, const CANFD_ELEM_SIZE_T *psConfigSize)
{
    uint32_t u32RamAddrOffset = 0UL;

    /* Get the Standard Message ID Filter element address */
    if (psConfigSize->u32SIDFC > 0UL)
    {
        psConfigAddr->u32SIDFC_FLSSA = 0UL;
        u32RamAddrOffset += psConfigSize->u32SIDFC * sizeof(CANFD_STD_FILTER_T);
    }

    /* Get the Extended Message ID Filter element address */
    if (psConfigSize->u32XIDFC > 0UL)
    {
        psConfigAddr->u32XIDFC_FLESA = u32RamAddrOffset;
        u32RamAddrOffset += psConfigSize->u32XIDFC * sizeof(CANFD_EXT_FILTER_T);
    }

    /* Get the Rx FIFO0 element address */
    if (psConfigSize->u32RxFifo0 > 0UL)
    {
        psConfigAddr->u32RXF0C_F0SA = u32RamAddrOffset;
        u32RamAddrOffset += psConfigSize->u32RxFifo0 * sizeof(CANFD_BUF_T);
    }

    /* Get the Rx FIFO1 element address */
    if (psConfigSize->u32RxFifo1 > 0UL)
    {
        psConfigAddr->u32RXF1C_F1SA = u32RamAddrOffset;
        u32RamAddrOffset += psConfigSize->u32RxFifo1 * sizeof(CANFD_BUF_T);
    }

    /* Get the Rx Buffer element address */
    if (psConfigSize->u32RxBuf > 0UL)
    {
        psConfigAddr->u32RXBC_RBSA = u32RamAddrOffset;
        u32RamAddrOffset += psConfigSize->u32RxBuf * sizeof(CANFD_BUF_T);
    }

    /* Get the TX Event FIFO element address */
    if (psConfigSize->u32TxEventFifo > 0UL)
    {
        psConfigAddr->u32TXEFC_EFSA = u32RamAddrOffset;
        u32RamAddrOffset += psConfigSize->u32TxEventFifo *  sizeof(CANFD_EXT_FILTER_T);
    }

    /* Get the Tx Buffer element address */
    if (psConfigSize->u32TxBuf > 0UL)
    {
        psConfigAddr->u32TXBC_TBSA = u32RamAddrOffset;
    }

}

/**
 * @brief       Get the default configuration structure.
 *
 * @param[in]   psConfig       Pointer to CAN FD configuration structure.
 * @param[in]   u8OpMode       Setting the CAN FD Operating mode.
 *
 * @return      None.
 *
 * @details     This function initializes the CAN FD configure structure to default value.
 *              The default value are:
 *              sNormBitRate.u32BitRate   = 500000bps;
 *              u32DataBaudRate     = 0(CAN mode) or 1000000(CAN FD mode) ;
 *              u32MRamSize         = 6144 bytes (1536 words);
 *              bEnableLoopBack     = FALSE;
 *              bBitRateSwitch      = FALSE (CAN Mode) or TRUE (CAN FD Mode);
 *              bFDEn               = FALSE (CAN Mode) or TRUE (CAN FD Mode);
*/
void CANFD_GetDefaultConfig(CANFD_FD_T *psConfig, uint8_t u8OpMode)
{
    (void)memset(psConfig, 0, sizeof(CANFD_FD_T));

    psConfig->sBtConfig.sNormBitRate.u32BitRate = 500000UL;

    if (u8OpMode == CANFD_OP_CAN_MODE)
    {
        psConfig->sBtConfig.sDataBitRate.u32BitRate = 0UL;
        psConfig->sBtConfig.bFDEn = FALSE;
        psConfig->sBtConfig.bBitRateSwitch = FALSE;
    }
    else
    {
        psConfig->sBtConfig.sDataBitRate.u32BitRate = 1000000UL;
        psConfig->sBtConfig.bFDEn = TRUE;
        psConfig->sBtConfig.bBitRateSwitch = TRUE;
    }

    /*Disable the Internal Loopback mode */
    psConfig->sBtConfig.bEnableLoopBack = FALSE;
    /*Get the CAN FD memory size(number of byte) */
    psConfig->u32MRamSize  = CANFD_SRAM_SIZE;

    /* CAN FD Standard message ID elements as 64 elements    */
    psConfig->sElemSize.u32SIDFC = 64UL;
    /* CAN FD Extended message ID elements as 64 elements    */
    psConfig->sElemSize.u32XIDFC = 64UL;
    /* CAN FD TX Buffer elements as 8 elements    */
    psConfig->sElemSize.u32TxBuf = 8UL;
    /* CAN FD RX Buffer elements as 8 elements    */
    psConfig->sElemSize.u32RxBuf = 8UL;
    /* CAN FD RX FIFO0 elements as 48 elements    */
    psConfig->sElemSize.u32RxFifo0 = 48UL;
    /* CAN FD RX FIFO1 elements as 8 elements    */
    psConfig->sElemSize.u32RxFifo1 = 8UL;
    /* CAN FD TX Event FOFI elements as 8 elements    */
    psConfig->sElemSize.u32TxEventFifo = 8UL;
    /*Calculates the CAN FD RAM buffer address*/
    CANFD_CalculateRamAddress(&psConfig->sMRamStartAddr, &psConfig->sElemSize);
}


/**
 * @brief       Encode the Data Length Code.
 *
 * @param[in]   u8NumberOfBytes  Number of bytes in a message.
 *
 * @return      Data Length Code.
 *
 * @details     Converts number of bytes in a message into a Data Length Code.
 */
static uint8_t CANFD_EncodeDLC(uint8_t u8NumberOfBytes)
{
    if (u8NumberOfBytes <= 8U)
    {
        return u8NumberOfBytes;
    }
    else if (u8NumberOfBytes <= 12U)
    {
        return 9U;
    }
    else if (u8NumberOfBytes <= 16U)
    {
        return 10U;
    }
    else if (u8NumberOfBytes <= 20U)
    {
        return 11U;
    }
    else if (u8NumberOfBytes <= 24U)
    {
        return 12U;
    }
    else if (u8NumberOfBytes <= 32U)
    {
        return 13U;
    }
    else if (u8NumberOfBytes <= 48U)
    {
        return 14U;
    }
    else
    {
        return 15U;
    }
}


/**
 * @brief       Decode the Data Length Code.
 *
 * @param[in]   u8Dlc   Data Length Code.
 *
 * @return      Number of bytes in a message.
 *
 * @details     Converts a Data Length Code into a number of message bytes.
 */
static uint8_t CANFD_DecodeDLC(uint8_t u8Dlc)
{
    if (u8Dlc <= 8U)
    {
        return u8Dlc;
    }
    else if (u8Dlc == 9U)
    {
        return 12U;
    }
    else if (u8Dlc == 10U)
    {
        return 16U;
    }
    else if (u8Dlc == 11U)
    {
        return 20U;
    }
    else if (u8Dlc == 12U)
    {
        return 24U;
    }
    else if (u8Dlc == 13U)
    {
        return 32U;
    }
    else if (u8Dlc == 14U)
    {
        return 48U;
    }
    else
    {
        return 64U;
    }
}


/// @cond HIDDEN_SYMBOLS
/**
  * @brief Check if the sample point is suitable.
 *
  * @param[in] sampl_pt The sample point position in bit timing.
  * @param[in] tseg  Current tseg value in bit timing.
  * @param[in] tseg1 Current tseg1 value in bit timing.
  * @param[in] tseg2 Current tseg2 value in bit timing.
  * @param[in] u32Set_NBTP Set normal bit time or data bit time.
  * @return The sample point position in bit timing.
 */
static int32_t CANFD_Update_Spt(int32_t sampl_pt, int32_t tseg, int32_t *tseg1, int32_t *tseg2, uint32_t u32Set_NBTP)
{
    int32_t tseg2_max = 0;
    int32_t tseg2_min = 0;
    int32_t tseg1_max = 0;

    if(u32Set_NBTP != 0UL)
    {
        tseg1_max = N_TSEG1_MAX;
        tseg2_min = N_TSEG2_MIN;
        tseg2_max = N_TSEG2_MAX;
    }
    else
    {
        tseg1_max = D_TSEG1_MAX;
        tseg2_min = D_TSEG2_MIN;
        tseg2_max = D_TSEG2_MAX;
    }


    *tseg2 = tseg + 1 - (sampl_pt * (tseg + 1)) / 1000;

    if (*tseg2 < tseg2_min)
    {
        *tseg2 = tseg2_min;
    }

    if (*tseg2 > tseg2_max)
    {
        *tseg2 = tseg2_max;
    }

    *tseg1 = tseg - *tseg2;
    if (*tseg1 > tseg1_max)
    {
        *tseg1 = tseg1_max;
        *tseg2 = tseg - *tseg1;
    }

    return (1000 * (tseg + 1 - *tseg2)) / (tseg + 1);
}


/**
  * @brief Set bus baud-rate.
 *
  * @param[in] psCanfd The pointer to CAN module base address.
  * @param[in] u32BaudRate The target CAN baud-rate. The range of u32BaudRate is 1~1000KHz.
  * @param[in] u32SourceClock_Hz CAN clock source frequency
  * @param[in] u32Set_NBTP Set normal bit time or data bit time.
 *
  * @return Real baud-rate value.
 *
  * @details The function is used to set bus timing parameter according current clock and target baud-rate.
 */
uint32_t CANFD_SetBitRate(CANFD_T *psCanfd, uint32_t u32BaudRate, int32_t u32SourceClock_Hz, uint32_t u32Set_NBTP)
{
    const uint32_t u32TargetBaudRate = u32BaudRate;
    const int32_t  i32SourceClock_Hz = u32SourceClock_Hz;

    uint64_t clock_freq;
    uint32_t u32RealBaudRate;

    int32_t tseg1_min;
    int32_t tseg1_max;
    int32_t tseg2_min;
    int32_t tseg2_max;
    int32_t brp_min;
    int32_t brp_max;
    int32_t sjw_max;
    int32_t brp_inc;
    int32_t sampl_pt;
    int32_t spt;
    int32_t spt_error = 1000;
    int32_t tseg;
    int32_t best_tseg = 0;
    int32_t tseg1 = 0;
    int32_t tseg2 = 0;
    int32_t brp;
    int32_t best_brp = 0;
    int32_t sjw = 2;
    int64_t best_error = 1000000000LL;
    int64_t error;
    uint32_t reg_btp = 0UL;

    clock_freq = (uint64_t)i32SourceClock_Hz;

    if (u32Set_NBTP != 0UL)
    {
        tseg1_min = N_TSEG1_MIN;
        tseg1_max = N_TSEG1_MAX;
        tseg2_min = N_TSEG2_MIN;
        tseg2_max = N_TSEG2_MAX;
        brp_min   = N_BRP_MIN;
        brp_max   = N_BRP_MAX;
        sjw_max   = N_SJW_MAX;
        brp_inc   = N_BRP_INC;
    }
    else
    {
        tseg1_min = D_TSEG1_MIN;
        tseg1_max = D_TSEG1_MAX;
        tseg2_min = D_TSEG2_MIN;
        tseg2_max = D_TSEG2_MAX;
        brp_min   = D_BRP_MIN;
        brp_max   = D_BRP_MAX;
        sjw_max   = D_SJW_MAX;
        brp_inc   = D_BRP_INC;
    }

    if (u32TargetBaudRate > 800000UL)
    {
        sampl_pt = 750;
    }
    else if (u32TargetBaudRate > 500000UL)
    {
        sampl_pt = 800;
    }
    else
    {
        sampl_pt = 875;
    }

    const int32_t tseg_max2 = ((tseg1_max + tseg2_max) * 2) + 1;
    const int32_t tseg_min2 = ((tseg1_min + tseg2_min) * 2);

    for (tseg = tseg_max2; tseg >= tseg_min2; tseg--)
    {
        int32_t tsegall;
        uint64_t brp_u64;
        uint64_t rate_u64;

        tsegall = 1 + (tseg / 2);
        if (tsegall <= 0)
        {
            continue;
        }

        /* brp = clock / (tsegall * baud) + (tseg % 2) */
        brp_u64 = clock_freq / ((uint64_t)tsegall * (uint64_t)u32TargetBaudRate);

        {
            uint32_t tseg_u32;
            uint32_t tseg_lsb_u32;

            tseg_u32     = (uint32_t)tseg;
            tseg_lsb_u32 = tseg_u32 & 1UL;
            brp_u64     += (uint64_t)tseg_lsb_u32;
        }

        if (brp_u64 > (uint64_t)INT32_MAX)
        {
            continue;
        }
        brp = (int32_t)brp_u64;

        /* choose brp step which is possible in system */
        if (brp_inc <= 0)
        {
            continue;
        }
        brp = (brp / brp_inc) * brp_inc;

        if ((brp < brp_min) || (brp > brp_max))
        {
            continue;
        }

        rate_u64 = clock_freq / ((uint64_t)brp * (uint64_t)tsegall);
        error = (int64_t)u32TargetBaudRate - (int64_t)rate_u64;

        if (error < 0LL)
        {
            error = -error;
        }

        if (error > best_error)
        {
            continue;
        }

        best_error = error;

        if (error == 0LL)
        {
            spt = CANFD_Update_Spt(sampl_pt, (tseg / 2), &tseg1, &tseg2, u32Set_NBTP);
            int32_t e2 = sampl_pt - spt;
            if (e2 < 0)
            {
                e2 = -e2;
            }
            if (e2 > spt_error)
            {
                continue;
            }
            spt_error = e2;
        }

        best_tseg = (tseg / 2);
        best_brp = brp;

        if (error == 0LL)
        {
            break;
        }
    }

    (void)CANFD_Update_Spt(sampl_pt, best_tseg, &tseg1, &tseg2, u32Set_NBTP);

    /* check for sjw user settings */
    if (sjw > sjw_max)
    {
        sjw = sjw_max;
    }
    if (tseg2 < sjw)
    {
        sjw = tseg2;
    }

    if ((best_brp <= 0) || (tseg1 <= 0) || (tseg2 <= 0) || (sjw <= 0))
    {
        return 0UL;
    }

    best_brp--;
    sjw--;
    tseg1--;
    tseg2--;


    {
        uint64_t real_rate_u64;
        uint64_t brp_p1_u64;
        uint64_t tseg_total_u64;

        brp_p1_u64     = (uint64_t)best_brp + 1ULL;
        tseg_total_u64 = (uint64_t)tseg1 + (uint64_t)tseg2 + 3ULL;
        real_rate_u64 = clock_freq / (brp_p1_u64 * tseg_total_u64);
        u32RealBaudRate = (uint32_t)real_rate_u64;
    }

    if (u32Set_NBTP != 0UL)
    {
        reg_btp =
            ((uint32_t)best_brp << CANFD_NBTP_NBRP_Pos) |
            ((uint32_t)sjw      << CANFD_NBTP_NSJW_Pos) |
            ((uint32_t)tseg1    << CANFD_NBTP_NTSEG1_Pos) |
            ((uint32_t)tseg2    << CANFD_NBTP_NTSEG2_Pos);

        psCanfd->NBTP = reg_btp;
    }
    else
    {
        /* TDC only needed for bitrates beyond 2.5 MBit/s */
        if (u32RealBaudRate > 2500000UL)
        {
            uint32_t tdco;
            uint32_t ssp;

            ssp = (uint32_t)sampl_pt;
            tdco = (uint32_t)(((clock_freq / 1000ULL) * (uint64_t)ssp) / (uint64_t)u32RealBaudRate);

            if (tdco > 127UL)
            {
                tdco = 127UL;
            }

            reg_btp |= (1UL << 23UL);
            psCanfd->TDCR = (tdco << CANFD_TDCR_TDCO_Pos);
        }

        reg_btp |=
            ((uint32_t)best_brp << CANFD_DBTP_DBRP_Pos) |
            ((uint32_t)sjw      << CANFD_DBTP_DSJW_Pos) |
            ((uint32_t)tseg1    << CANFD_DBTP_DTSEG1_Pos) |
            ((uint32_t)tseg2    << CANFD_DBTP_DTSEG2_Pos);

        psCanfd->DBTP = reg_btp;
    }

    return u32RealBaudRate;
}


/**
 * @brief       Calculates the CAN controller timing values for specific baudrates.
 *
 * @param[in]   psCanfd             Pointer to CAN FD configuration structure.
 * @param[in]   u32NominalBaudRate  The nominal speed in bps.
 * @param[in]   u32DataBaudRate     The data speed in bps. Zero to disable baudrate switching.
 * @param[in]   u32SourceClock_Hz   CAN FD Protocol Engine clock source frequency in Hz.
 * @param[in]   psConfig            Passed is a configuration structure, on return the configuration is stored in the structure
 *
 * @return      true if timing configuration found, false if failed to find configuration.
 *
 * @details     Calculates the CAN controller timing values for specific baudrates.
 */
void CANFD_CalculateTimingValues(CANFD_T *psCanfd,
                                 uint32_t u32NominalBaudRate, uint32_t u32DataBaudRate,
                                 uint32_t u32SourceClock_Hz, CANFD_TIMEING_CONFIG_T *psConfig)
{
    uint32_t u32NominalBR;
    uint32_t u32DataBR;
    uint32_t u32Tmp;

    (void)psConfig;

    u32NominalBR = u32NominalBaudRate;
    u32DataBR    = u32DataBaudRate;

    /* observe baud rate maximums */
    if (u32NominalBR > MAX_NOMINAL_BAUDRATE)
    {
        u32NominalBR = MAX_NOMINAL_BAUDRATE;
    }
    else
    {
        /* u32NominalBR already set */
    }

    if (u32DataBR > MAX_DATA_BAUDRATE)
    {
        u32DataBR = MAX_DATA_BAUDRATE;
    }
    else
    {
        /* u32DataBR already set */
    }

    u32Tmp = CANFD_SetBitRate(psCanfd,
                              u32NominalBR,
                              (int32_t)u32SourceClock_Hz,
                              1U);
    (void)u32Tmp;


    if ((psCanfd->CCCR & CANFD_CCCR_FDOE_Msk) != 0UL)
    {
        u32Tmp = CANFD_SetBitRate(psCanfd,
                                  u32DataBR,
                                  (int32_t)u32SourceClock_Hz,
                                  0U);
        (void)u32Tmp;
    }

}
/// @endcond HIDDEN_SYMBOLS


/**
 * @brief       Config message ram and Set bit-time.
 *
 * @param[in]   psCanfd     The pointer to CAN FD module base address.
 * @param[in]   psCanfdStr  message ram setting and bit-time setting
 *
 * @return      None.
 *
 * @details     Converts a Data Length Code into a number of message bytes.
 */
void CANFD_Open(CANFD_T *psCanfd, CANFD_FD_T *psCanfdStr)
{
    uint32_t u32CanFdClock = 0UL;
    uint32_t u32RegLockLevel =  (uint32_t) SYS_IsRegLocked();

    if (u32RegLockLevel != 0UL)
    {
        SYS_UnlockReg();
    }

    if (psCanfd == (CANFD_T *)CANFD0)
    {
        CLK_EnableModuleClock(CANFD0_MODULE);
        SYS_ResetModule(CANFD0_RST);

        if ((CLK->CLKSEL0 & CLK_CLKSEL0_CANFD0SEL_Msk) == CLK_CLKSEL0_CANFD0SEL_HXT)
        {
            u32CanFdClock = __HXT;  /* Clock source is HXT */
        }
        else if ((CLK->CLKSEL0 & CLK_CLKSEL0_CANFD0SEL_Msk) == CLK_CLKSEL0_CANFD0SEL_HIRC48M)
        {
            u32CanFdClock = __HIRC48;   /* Clock source is HIRC48 */
        }
        else if ((CLK->CLKSEL0 & CLK_CLKSEL0_CANFD0SEL_Msk) == CLK_CLKSEL0_CANFD0SEL_HCLK)
        {
            u32CanFdClock = SystemCoreClock;    /* Clock source is HCLK */
        }
        else
        {
            u32CanFdClock = __HIRC; /* Clock source is HIRC */
        }
    }
    else if (psCanfd == (CANFD_T *)CANFD1)
    {
        CLK_EnableModuleClock(CANFD1_MODULE);
        SYS_ResetModule(CANFD1_RST);

        if ((CLK->CLKSEL0 & CLK_CLKSEL0_CANFD1SEL_Msk) == CLK_CLKSEL0_CANFD1SEL_HXT)
        {
            u32CanFdClock =  __HXT;  /* Clock source is HXT */
        }
        else if ((CLK->CLKSEL0 & CLK_CLKSEL0_CANFD1SEL_Msk) == CLK_CLKSEL0_CANFD1SEL_HIRC48M)
        {
            u32CanFdClock = __HIRC48;   /* Clock source is HIRC48 */
        }
        else if ((CLK->CLKSEL0 & CLK_CLKSEL0_CANFD1SEL_Msk) == CLK_CLKSEL0_CANFD1SEL_HCLK)
        {
            u32CanFdClock = SystemCoreClock;    /* Clock source is HCLK */
        }
        else
        {
            u32CanFdClock = __HIRC; /* Clock source is HIRC */
        }
    }
    else
    {
        if (u32RegLockLevel != 0UL)
        {
            SYS_LockReg();
        }
        return;
    }

    /* configuration change enable */
    psCanfd->CCCR |= CANFD_CCCR_CCE_Msk;

    if (psCanfdStr->sBtConfig.bBitRateSwitch != 0U)
    {
        /* enable FD and baud-rate switching */
        psCanfd->CCCR |= CANFD_CCCR_BRSE_Msk;
    }

    if (psCanfdStr->sBtConfig.bFDEn != 0U)
    {
        /*FD Operation enabled*/
        psCanfd->CCCR |= CANFD_CCCR_FDOE_Msk;
    }

    /*Clear the Rx Fifo0 element setting */
    psCanfd->RXF0C = 0UL;
    /*Clear the Rx Fifo1 element setting */
    psCanfd->RXF1C = 0UL;

    /* calculate and apply timing */
    CANFD_CalculateTimingValues(psCanfd, psCanfdStr->sBtConfig.sNormBitRate.u32BitRate,
                                psCanfdStr->sBtConfig.sDataBitRate.u32BitRate,
                                u32CanFdClock, &psCanfdStr->sBtConfig.sConfigBitTing);

    if (u32RegLockLevel != 0UL)
    {
        SYS_LockReg();
    }
    /* Configures the Standard ID Filter element */
    if (psCanfdStr->sElemSize.u32SIDFC != 0UL)
    {
        CANFD_ConfigSIDFC(psCanfd, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize);
    }
    /*Configures the Extended ID Filter element */
    if (psCanfdStr->sElemSize.u32XIDFC != 0UL)
    {
        CANFD_ConfigXIDFC(psCanfd, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize);
    }
    /*Configures the Tx Buffer element */
    if (psCanfdStr->sElemSize.u32TxBuf != 0UL)
    {
        CANFD_InitTxDBuf(psCanfd, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize, eCANFD_BYTE64);
    }
    /*Configures the Rx Buffer element */
    if (psCanfdStr->sElemSize.u32RxBuf != 0UL)
    {
        CANFD_InitRxDBuf(psCanfd, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize, eCANFD_BYTE64);
    }
    /*Configures the Rx Fifo0 element */
    if (psCanfdStr->sElemSize.u32RxFifo0 != 0UL)
    {
        CANFD_InitRxFifo(psCanfd, 0UL, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize, 0, eCANFD_BYTE64);
    }
    /*Configures the Rx Fifo1 element */
    if (psCanfdStr->sElemSize.u32RxFifo1 != 0UL)
    {
        CANFD_InitRxFifo(psCanfd, 1UL, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize, 0, eCANFD_BYTE64);
    }
    /*Configures the Tx Event FIFO element */
    if (psCanfdStr->sElemSize.u32TxEventFifo != 0UL)
    {
        CANFD_InitTxEvntFifo(psCanfd, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize, 0);
    }
    /*Reject all Non-matching Frames Extended ID and Frames Standard ID,Reject all remote frames with 11-bit standard IDs and 29-bit extended IDs */
    CANFD_SetGFC(psCanfd, eCANFD_REJ_NON_MATCH_FRM, eCANFD_REJ_NON_MATCH_FRM, 1, 1);

    if (psCanfdStr->sBtConfig.bEnableLoopBack)
    {
        psCanfd->CCCR |= CANFD_CCCR_TEST_Msk;
        psCanfd->TEST |= CANFD_TEST_LBCK_Msk;
    }
}


/**
 * @brief       Close the CAN FD Bus.
 *
 * @param[in]   psCanfd   The pointer to CANFD module base address.
 *
 * @return      None.
 *
 * @details     Disable the CAN FD clock and Interrupt.
 */
void CANFD_Close(const CANFD_T *psCanfd)
{
    if (psCanfd == (const CANFD_T *)CANFD0)
    {
        CLK_DisableModuleClock(CANFD0_MODULE);
    }
    else if (psCanfd == (const CANFD_T *)CANFD1)
    {
        CLK_DisableModuleClock(CANFD1_MODULE);
    }
    else
    {
        /* Do nothing */
    }

}


/**
 * @brief       Get the element's address when read transmit buffer.
 *
 * @param[in]   psCanfd      The pointer of the specified CAN FD module.
 * @param[in]   u32Idx       The number of the transmit buffer element
 *
 * @return      Address of the element in transmit buffer.
 *
 * @details     The function is used to get the element's address when read transmit buffer.
 */
static uint32_t CANFD_GetTxBufferElementAddress(const CANFD_T *psCanfd, uint32_t u32Idx)
{
    uint32_t u32Size;
    u32Size = (psCanfd->TXESC & CANFD_TXESC_TBDS_Msk) >> CANFD_TXESC_TBDS_Pos;

    if (u32Size < 5U)
    {
        u32Size += 4U;
    }
    else
    {
        u32Size = ((u32Size * 4UL) - 10UL);
    }
    return ((psCanfd->TXBC & CANFD_TXBC_TBSA_Msk) + ((u32Idx * u32Size) * 4UL));
}

/**
 * @brief       Enables CAN FD interrupts according to provided mask .
 *
 * @param[in]   psCanfd          The pointer of the specified CAN FD module.
 * @param[in]   u32IntLine0      The Interrupt Line 0 type select.
 * @param[in]   u32IntLine1      The Interrupt Line 1 type select.
 *                              - \ref CANFD_IE_ARAE_Msk     : Access to Reserved Address Interrupt
 *                              - \ref CANFD_IE_PEDE_Msk     : Protocol Error in Data Phase Interrupt
 *                              - \ref CANFD_IE_PEAE_Msk     : Protocol Error in Arbitration Phase Interrupt
 *                              - \ref CANFD_IE_WDIE_Msk     : Watchdog Interrupt
 *                              - \ref CANFD_IE_BOE_Msk      : Bus_Off Status Interrupt
 *                              - \ref CANFD_IE_EWE_Msk      : Warning Status Interrupt
 *                              - \ref CANFD_IE_EPE_Msk      : Error Passive Interrupt
 *                              - \ref CANFD_IE_ELOE_Msk     : Error Logging Overflow Interrupt
 *                              - \ref CANFD_IE_BEUE_Msk     : Bit Error Uncorrected Interrupt
 *                              - \ref CANFD_IE_BECE_Msk     : Bit Error Corrected Interrupt
 *                              - \ref CANFD_IE_DRXE_Msk     : Message stored to Dedicated Rx Buffer Interrupt
 *                              - \ref CANFD_IE_TOOE_Msk     : Timeout Occurred Interrupt
 *                              - \ref CANFD_IE_MRAFE_Msk    : Message RAM Access Failure Interrupt
 *                              - \ref CANFD_IE_TSWE_Msk     : Timestamp Wraparound Interrupt
 *                              - \ref CANFD_IE_TEFLE_Msk    : Tx Event FIFO Event Lost Interrupt
 *                              - \ref CANFD_IE_TEFFE_Msk    : Tx Event FIFO Full Interrupt
 *                              - \ref CANFD_IE_TEFWE_Msk    : Tx Event FIFO Watermark Reached Interrupt
 *                              - \ref CANFD_IE_TEFNE_Msk    : Tx Event FIFO New Entry Interrupt
 *                              - \ref CANFD_IE_TFEE_Msk     : Tx FIFO Empty Interrupt
 *                              - \ref CANFD_IE_TCFE_Msk     : Transmission Cancellation Finished Interrupt
 *                              - \ref CANFD_IE_TCE_Msk      : Transmission Completed Interrupt
 *                              - \ref CANFD_IE_HPME_Msk     : High Priority Message Interrupt
 *                              - \ref CANFD_IE_RF1LE_Msk    : Rx FIFO 1 Message Lost Interrupt
 *                              - \ref CANFD_IE_RF1FE_Msk    : Rx FIFO 1 Full Interrupt
 *                              - \ref CANFD_IE_RF1WE_Msk    : Rx FIFO 1 Watermark Reached Interrupt
 *                              - \ref CANFD_IE_RF1NE_Msk    : Rx FIFO 1 New Message Interrupt
 *                              - \ref CANFD_IE_RF0LE_Msk    : Rx FIFO 0 Message Lost Interrupt
 *                              - \ref CANFD_IE_RF0FE_Msk    : Rx FIFO 0 Full Interrupt
 *                              - \ref CANFD_IE_RF0WE_Msk    : Rx FIFO 0 Watermark Reached Interrupt
 *                              - \ref CANFD_IE_RF0NE_Msk    : Rx FIFO 0 New Message Interrupt
 *
 * @param[in]   u32TXBTIE        Enable Tx Buffer Transmission 0-31 Interrupt.
 * @param[in]   u32TXBCIE        Enable Tx Buffer Cancellation Finished 0-31 Interrupt.
 * @return      None.
 *
 * @details     This macro enable specified CAN FD interrupt.
 */
void CANFD_EnableInt(CANFD_T *psCanfd, uint32_t u32IntLine0, uint32_t u32IntLine1, uint32_t u32TXBTIE, uint32_t u32TXBCIE)
{

    if (u32IntLine0 != 0UL)
    {
        /*Setting the CANFD0_IRQ0 Interrupt*/
        psCanfd->IE |= u32IntLine0;
        /*Setting Interrupt Line selection*/
        psCanfd->ILS &= ~u32IntLine0;
        /* Enable CAN FD specified interrupt */
        psCanfd->ILE |= CANFD_ILE_EINT0_Msk;
    }

    if (u32IntLine1 != 0UL)
    {
        /*Setting the CANFD0_IRQ1 Interrupt*/
        psCanfd->IE |= u32IntLine1;
        /*Setting Interrupt Line selection*/
        psCanfd->ILS |= u32IntLine1;
        /* Enable CAN FD specified interrupt */
        psCanfd->ILE |= CANFD_ILE_EINT1_Msk;
    }

    /*Setting the Tx Buffer Transmission Interrupt Enable*/
    psCanfd->TXBTIE |= u32TXBTIE;

    /*Tx Buffer Cancellation Finished Interrupt Enable*/
    psCanfd->TXBCIE |= u32TXBCIE;
}


/**
 * @brief       Disables CAN FD interrupts according to provided mask .
 *
 * @param[in]   psCanfd          The pointer of the specified CAN FD module.
 * @param[in]   u32IntLine0      The Interrupt Line 0 type select.
 * @param[in]   u32IntLine1      The Interrupt Line 1 type select.
 *                              - \ref CANFD_IE_ARAE_Msk     : Access to Reserved Address Interrupt
 *                              - \ref CANFD_IE_PEDE_Msk     : Protocol Error in Data Phase Interrupt
 *                              - \ref CANFD_IE_PEAE_Msk     : Protocol Error in Arbitration Phase Interrupt
 *                              - \ref CANFD_IE_WDIE_Msk     : Watchdog Interrupt
 *                              - \ref CANFD_IE_BOE_Msk      : Bus_Off Status Interrupt
 *                              - \ref CANFD_IE_EWE_Msk      : Warning Status Interrupt
 *                              - \ref CANFD_IE_EPE_Msk      : Error Passive Interrupt
 *                              - \ref CANFD_IE_ELOE_Msk     : Error Logging Overflow Interrupt
 *                              - \ref CANFD_IE_BEUE_Msk     : Bit Error Uncorrected Interrupt
 *                              - \ref CANFD_IE_BECE_Msk     : Bit Error Corrected Interrupt
 *                              - \ref CANFD_IE_DRXE_Msk     : Message stored to Dedicated Rx Buffer Interrupt
 *                              - \ref CANFD_IE_TOOE_Msk     : Timeout Occurred Interrupt
 *                              - \ref CANFD_IE_MRAFE_Msk    : Message RAM Access Failure Interrupt
 *                              - \ref CANFD_IE_TSWE_Msk     : Timestamp Wraparound Interrupt
 *                              - \ref CANFD_IE_TEFLE_Msk    : Tx Event FIFO Event Lost Interrupt
 *                              - \ref CANFD_IE_TEFFE_Msk    : Tx Event FIFO Full Interrupt
 *                              - \ref CANFD_IE_TEFWE_Msk    : Tx Event FIFO Watermark Reached Interrupt
 *                              - \ref CANFD_IE_TEFNE_Msk    : Tx Event FIFO New Entry Interrupt
 *                              - \ref CANFD_IE_TFEE_Msk     : Tx FIFO Empty Interrupt
 *                              - \ref CANFD_IE_TCFE_Msk     : Transmission Cancellation Finished Interrupt
 *                              - \ref CANFD_IE_TCE_Msk      : Transmission Completed Interrupt
 *                              - \ref CANFD_IE_HPME_Msk     : High Priority Message Interrupt
 *                              - \ref CANFD_IE_RF1LE_Msk    : Rx FIFO 1 Message Lost Interrupt
 *                              - \ref CANFD_IE_RF1FE_Msk    : Rx FIFO 1 Full Interrupt
 *                              - \ref CANFD_IE_RF1WE_Msk    : Rx FIFO 1 Watermark Reached Interrupt
 *                              - \ref CANFD_IE_RF1NE_Msk    : Rx FIFO 1 New Message Interrupt
 *                              - \ref CANFD_IE_RF0LE_Msk    : Rx FIFO 0 Message Lost Interrupt
 *                              - \ref CANFD_IE_RF0FE_Msk    : Rx FIFO 0 Full Interrupt
 *                              - \ref CANFD_IE_RF0WE_Msk    : Rx FIFO 0 Watermark Reached Interrupt
 *                              - \ref CANFD_IE_RF0NE_Msk    : Rx FIFO 0 New Message Interrupt
 *
 * @param[in]   u32TXBTIE        Disable Tx Buffer Transmission 0-31 Interrupt.
 * @param[in]   u32TXBCIE        Disable Tx Buffer Cancellation Finished 0-31 Interrupt.
 * @return      None.
 *
 * @details     This macro disable specified CAN FD interrupt.
 */
void CANFD_DisableInt(CANFD_T *psCanfd, uint32_t u32IntLine0, uint32_t u32IntLine1, uint32_t u32TXBTIE, uint32_t u32TXBCIE)
{
    if (u32IntLine0 != 0UL)
    {
        /*Clear the CANFD0_IRQ0 Interrupt*/
        psCanfd->IE &= ~u32IntLine0;
        /* Disable CAN FD specified interrupt */
        psCanfd->ILE &= ~CANFD_ILE_EINT0_Msk;
    }

    if (u32IntLine1 != 0UL)
    {
        /*Clear the CANFD0_IRQ1 Interrupt*/
        psCanfd->IE &= ~u32IntLine1;
        /* Disable CAN FD specified interrupt */
        psCanfd->ILE &= ~CANFD_ILE_EINT1_Msk;
    }

    /*Setting the Tx Buffer Transmission Interrupt Disable*/
    psCanfd->TXBTIE &= ~u32TXBTIE;

    /*Tx Buffer Cancellation Finished Interrupt Disable*/
    psCanfd->TXBCIE &= ~u32TXBCIE;
}


/**
 * @brief       Copy Tx Message to  TX buffer and Request transmission.
 *
 * @param[in]   psCanfd         The pointer to CAN FD module base address.
 * @param[in]   u32TxBufIdx     The Message Buffer index.
 * @param[in]   psTxMsg         Message to be copied.
 *
 * @return      number of tx requests set: 0= Tx Message Buffer is currently in use.
 *                                         1= Write Tx Message Buffer Successfully.
 *
 * @details     Copy Tx Message to FIFO/Queue TX buffer and Request transmission.
 */
uint32_t CANFD_TransmitTxMsg(CANFD_T *psCanfd, uint32_t u32TxBufIdx, CANFD_FD_MSG_T *psTxMsg)
{
    uint32_t u32Success = 0;
    uint32_t u32TimeOutCnt = CANFD_TIMEOUT;

    /* write the message to the message buffer */
    u32Success = CANFD_TransmitDMsg(psCanfd, u32TxBufIdx, psTxMsg);

    if (u32Success == 1UL)
    {
        /* wait for completion */
        while (!(psCanfd->TXBRP & (1UL << u32TxBufIdx)))
        {
            if (--u32TimeOutCnt == 0)
            {
                u32Success = 0;
                break;
            }

        }
    }

    return u32Success;
}


/**
 * @brief       Writes a Tx Message to Transmit Message Buffer.
 *
 * @param[in]   psCanfd        The pointer of the specified CAN FD module.
 * @param[in]   u32TxBufIdx    The Message Buffer index.
 * @param[in]   psTxMsg        Pointer to CAN FD message frame to be sent.
 *
 * @return      1  Write Tx Message Buffer Successfully.
 *              0  Tx Message Buffer is currently in use.
 *
 * @details     This function writes a CANFD Message to the specified Transmit Message Buffer
 *              and changes the Message Buffer state to start CANFD Message transmit. After
 *              that the function returns immediately.
 */
uint32_t CANFD_TransmitDMsg(CANFD_T *psCanfd, uint32_t u32TxBufIdx, CANFD_FD_MSG_T *psTxMsg)
{
    CANFD_BUF_T *psTxBuffer;
    uint32_t u32Idx = 0UL;
    uint32_t u32TimeOutCnt = CANFD_TIMEOUT;
    uint32_t u32Bytes;
    uint32_t u32WordCount;
    uint32_t u32DlcCode;
    uintptr_t addr;

    if (u32TxBufIdx >= CANFD_MAX_TX_BUF_ELEMS)
    {
        return 0UL;
    }

    /* transmission is pending in this message buffer */
    if ((psCanfd->TXBRP & (1UL << u32TxBufIdx)) != 0UL)
    {
        return 0UL;
    }

    /* DLC bytes sanity (assume u32DLC is bytes, max 64 for CAN FD) */
    u32Bytes = psTxMsg->u32DLC;
    if (u32Bytes > 64UL)
    {
        return 0UL;
    }

    /* Get the TX Buffer Start Address in the RAM */
    addr = (uintptr_t)CANFD_SRAM_BASE_ADDR(psCanfd)
           + (uintptr_t)(psCanfd->TXBC & CANFD_TXBC_TBSA_Msk)
           + ((uintptr_t)u32TxBufIdx * (uintptr_t)sizeof(CANFD_BUF_T));

    psTxBuffer = (CANFD_BUF_T *)addr;

    /* ID field */
    if (psTxMsg->eIdType == eCANFD_XID)
    {
        psTxBuffer->u32Id = (TX_BUFFER_T0_ELEM_XTD_Msk | (psTxMsg->u32Id & 0x1FFFFFFFUL));
    }
    else
    {
        psTxBuffer->u32Id = ((psTxMsg->u32Id & 0x7FFUL) << 18UL);
    }

    if (psTxMsg->eFrmType == eCANFD_REMOTE_FRM)
    {
        psTxBuffer->u32Id |= TX_BUFFER_T0_ELEM_RTR_Msk;
    }

    /* Encode DLC from bytes -> DLC code (0..15) */
    u32DlcCode = (uint32_t)CANFD_EncodeDLC((uint8_t)u32Bytes);

    psTxBuffer->u32Config = (u32DlcCode << 16UL);
    if (psTxMsg->bFDFormat != 0U)
    {
        psTxBuffer->u32Config |= TX_BUFFER_T1_ELEM_FDF_Msk;
    }
    if (psTxMsg->bBitRateSwitch != 0U)
    {
        psTxBuffer->u32Config |= TX_BUFFER_T1_ELEM_BSR_Msk;
    }

    u32WordCount = (u32Bytes + 3UL) / 4UL;
    for (u32Idx = 0UL; u32Idx < u32WordCount; u32Idx++)
    {
        psTxBuffer->au32Data[u32Idx] = psTxMsg->au32Data[u32Idx];
    }

    while ((uint32_t)CANFD_GET_COMMUNICATION_STATE(psCanfd) != (uint32_t)eCANFD_IDLE)
    {
        if (--u32TimeOutCnt == 0UL)
        {
            return 0UL;
        }
    }

    psCanfd->TXBAR = (1UL << u32TxBufIdx);
    return 1UL;
}


/**
 * @brief       Global Filter Configuration (GFC).
 *
 * @param[in]   psCanfd          The pointer to CAN FD module base address.
 * @param[in]   eNMStdFrm        Accept/Reject Non-Matching Standard(11-bits) Frames.
 * @param[in]   eEMExtFrm        Accept/Reject Non-Matching Extended(29-bits) Frames.
 * @param[in]   u32RejRmtStdFrm  Reject/Filter Remote Standard Frames.
 * @param[in]   u32RejRmtExtFrm  Reject/Filter Remote Extended Frames.
 *
 * @return      None.
 *
 * @details     Global Filter Configuration.
 */
void CANFD_SetGFC(CANFD_T *psCanfd, E_CANFD_ACC_NON_MATCH_FRM eNMStdFrm, E_CANFD_ACC_NON_MATCH_FRM eEMExtFrm, uint32_t u32RejRmtStdFrm, uint32_t u32RejRmtExtFrm)
{
    uint32_t gfc_val;

    psCanfd->GFC &= (uint32_t)(CANFD_GFC_RRFS_Msk | CANFD_GFC_RRFE_Msk);

    gfc_val =
        ((uint32_t)eNMStdFrm      << CANFD_GFC_ANFS_Pos) |
        ((uint32_t)eEMExtFrm      << CANFD_GFC_ANFE_Pos) |
        ((uint32_t)u32RejRmtStdFrm << CANFD_GFC_RRFS_Pos) |
        ((uint32_t)u32RejRmtExtFrm << CANFD_GFC_RRFE_Pos);

    psCanfd->GFC |= gfc_val;
}


/**
 * @brief       Rx FIFO Configuration for RX_FIFO_0 and RX_FIFO_1.
 *
 * @param[in]   psCanfd          The pointer to CAN FD module base address.
 * @param[in]   u32RxFifoNum     0: RX FIFO_0, 1: RX_FIFO_1.
 * @param[in]   psRamConfig      Rx FIFO Size in number of configuration ram address.
 * @param[in]   psElemSize       Rx FIFO Size in number of Rx FIFO elements (element number (max. = 64)).
 * @param[in]   u32FifoWM        Watermark in number of Rx FIFO elements
 * @param[in]   eFifoSize        Maximum data field size that should be stored in this Rx FIFO
 *                               (configure BYTE64 if you are unsure, as this is the largest data field allowed in CAN FD)
 *
 * @return      None.
 *
 * @details     Rx FIFO Configuration for RX_FIFO_0 and RX_FIFO_1.
 */
static void CANFD_InitRxFifo(CANFD_T *psCanfd, const uint32_t u32RxFifoNum, const CANFD_RAM_PART_T *psRamConfig, const CANFD_ELEM_SIZE_T *psElemSize, uint32_t u32FifoWM, const E_CANFD_DATA_FIELD_SIZE eFifoSize)
{
    uint32_t u32Address;
    uint32_t u32Size;

    /* ignore if index is too high */
    if (u32RxFifoNum > CANFD_NUM_RX_FIFOS)
    {
        return;
    }

    /* ignore if index is too high */
    if (psElemSize->u32RxFifo0 > CANFD_MAX_RX_FIFO0_ELEMS)
    {
        return;
    }

    /* ignore if index is too high */
    if (psElemSize->u32RxFifo1 > CANFD_MAX_RX_FIFO1_ELEMS)
    {
        return;
    }

    switch (u32RxFifoNum)
    {
    case 0:
        if (psElemSize->u32RxFifo0)
        {
            /* set size of Rx FIFO 0, set offset, blocking mode */
            psCanfd->RXF0C = (psRamConfig->u32RXF0C_F0SA) | (psElemSize->u32RxFifo0 << CANFD_RXF0C_F0S_Pos)
                             | (u32FifoWM << CANFD_RXF0C_F0WM_Pos);

            psCanfd->RXESC = (psCanfd->RXESC & ~CANFD_RXESC_F0DS_Msk) |
                             ((uint32_t)eFifoSize << CANFD_RXESC_F0DS_Pos);

            /*Get the RX FIFO 0 Start Address in the RAM*/
            u32Address = CANFD_SRAM_BASE_ADDR(psCanfd) + (psRamConfig->u32RXF0C_F0SA & CANFD_RXF0C_F0SA_Msk);
            u32Size = eFifoSize;

            if (u32Size < 5U)
            {
                u32Size += 4U;
            }
            else
            {
                u32Size = (u32Size * 4U) - 10U;
            }

            /*Clear the RX FIFO 0 Memory*/
            (void)memset((void *)u32Address, 0, (uint32_t)(u32Size * 4U * psElemSize->u32RxFifo0));

        }
        else
        {
            psCanfd->RXF0C = 0;
        }

        break;

    case 1:
        if (psElemSize->u32RxFifo1)
        {
            /* set size of Rx FIFO 1, set offset, blocking mode */
            psCanfd->RXF1C = (psRamConfig->u32RXF1C_F1SA) | (psElemSize->u32RxFifo1 << CANFD_RXF1C_F1S_Pos)
                             | (u32FifoWM << CANFD_RXF1C_F1WM_Pos);
            psCanfd->RXESC =
                (psCanfd->RXESC & ~CANFD_RXESC_F1DS_Msk) |
                ((uint32_t)eFifoSize << CANFD_RXESC_F1DS_Pos);

            /*Get the RX FIFO 1 Start Address in the RAM*/
            u32Address = CANFD_SRAM_BASE_ADDR(psCanfd) + (psRamConfig->u32RXF1C_F1SA & CANFD_RXF1C_F1SA_Msk);

            u32Size = eFifoSize;

            if (u32Size < 5U)
            {
                u32Size += 4U;
            }
            else
            {
                u32Size = (u32Size * 4U) - 10U;
            }

            /*Clear the RX FIFO 0 Memory*/
            (void)memset((void *)u32Address, 0, (uint32_t)(u32Size * 4U * psElemSize->u32RxFifo1));

        }
        else
        {
            psCanfd->RXF1C = 0;
        }
        break;

    default:
        break;
    }
}


/**
 * @brief       Function configures the data structures used by a dedicated Rx Buffer.
 *
 * @param[in]   psCanfd          The pointer to CAN FD module base address.
 * @param[in]   psRamConfig      Tx buffer configuration ram address.
 * @param[in]   psElemSize       Tx buffer configuration element size.
 * @param[in]   eTxBufSize       Maximum data field size that should be stored in a dedicated Tx Buffer
 *                              (configure BYTE64 if you are unsure, as this is the largest data field allowed in CAN FD)largest data field allowed in CAN FD)
 *
 * @return      None.
 *
 * @details     Function configures the data structures used by a dedicated Rx Buffer.
 */
static void CANFD_InitTxDBuf(CANFD_T *psCanfd, const CANFD_RAM_PART_T *psRamConfig, const CANFD_ELEM_SIZE_T *psElemSize, E_CANFD_DATA_FIELD_SIZE eTxBufSize)
{
    uint32_t u32Address;
    uint32_t u32Size;

    /*Setting the Tx Buffer Start Address*/
    psCanfd->TXBC =
        ((uint32_t)(psElemSize->u32TxBuf & 0x3FUL) << CANFD_TXBC_NDTB_Pos) |
        (psRamConfig->u32TXBC_TBSA & CANFD_TXBC_TBSA_Msk);

    /*Get the TX Buffer Start Address in the RAM*/
    u32Address = CANFD_SRAM_BASE_ADDR(psCanfd) + (psRamConfig->u32TXBC_TBSA & CANFD_TXBC_TBSA_Msk);

    /*Setting the Tx Buffer Data Field Size*/
    psCanfd->TXESC =
        (psCanfd->TXESC & ~CANFD_TXESC_TBDS_Msk) |
        ((uint32_t)eTxBufSize << CANFD_TXESC_TBDS_Pos);

    /*Get the Buffer Data Field Size*/
    u32Size = eTxBufSize;

    if (u32Size < 5U)
    {
        u32Size += 4U;
    }
    else
    {
        u32Size = (u32Size * 4U) - 10U;
    }

    /*Clear the TX Buffer Memory*/
    (void)memset((void *)u32Address, 0, (uint32_t)(u32Size * 4U * psElemSize->u32TxBuf));
}

/**
 * @brief       Function configures the data structures used by a dedicated Rx Buffer.
 *
 * @param[in]   psCanfd          The pointer to CAN FD module base address.
 * @param[in]   psRamConfig      Rx buffer configuration ram address.
 * @param[in]   psElemSize       Rx buffer configuration element size.
 * @param[in]   eRxBufSize       Maximum data field size that should be stored in a dedicated Rx Buffer
 *                              (configure BYTE64 if you are unsure, as this is the largest data field allowed in CAN FD)largest data field allowed in CAN FD)
 *
 * @return      None.
 *
 * @details     Function configures the data structures used by a dedicated Rx Buffer.
 */
static void CANFD_InitRxDBuf(CANFD_T *psCanfd, const CANFD_RAM_PART_T *psRamConfig, const CANFD_ELEM_SIZE_T *psElemSize, E_CANFD_DATA_FIELD_SIZE eRxBufSize)
{
    uint32_t u32Address;
    uint32_t u32Size;

    /*Setting the Rx Buffer Start Address*/
    psCanfd->RXBC = (psRamConfig->u32RXBC_RBSA & CANFD_RXBC_RBSA_Msk);

    /*Get the RX Buffer Start Address in the RAM*/
    u32Address = CANFD_SRAM_BASE_ADDR(psCanfd) + (psRamConfig->u32RXBC_RBSA & CANFD_RXBC_RBSA_Msk);

    /*Setting the Rx Buffer Data Field Size*/
    psCanfd->RXESC =
        (psCanfd->RXESC & ~CANFD_RXESC_RBDS_Msk) |
        ((uint32_t)eRxBufSize << CANFD_RXESC_RBDS_Pos);

    /*Get the Buffer Data Field Size*/
    u32Size = eRxBufSize;

    if (u32Size < 5U)
    {
        u32Size += 4U;
    }
    else
    {
        u32Size = (u32Size * 4U) - 10U;
    }

    /*Clear the RX Buffer Memory*/
    (void)memset((void *)u32Address, 0, (uint32_t)(u32Size * 4U * psElemSize->u32RxBuf));

}


/**
 * @brief       Configures the register SIDFC for the 11-bit Standard Message ID Filter elements.
 *
 * @param[in]   psCanfd           The pointer to CAN FD module base address.
 * @param[in]   psRamConfig       Standard ID filter configuration ram address
 * @param[in]   psElemSize        Standard ID filter configuration element size
 *
 * @return      None.
 *
 * @details     Function configures the data structures used by a dedicated Rx Buffer.
 */
static void CANFD_ConfigSIDFC(CANFD_T *psCanfd, CANFD_RAM_PART_T *psRamConfig, const CANFD_ELEM_SIZE_T *psElemSize)
{
    uint32_t u32Address;

    /*Setting the Filter List Standard Start Address and List Size  */
    psCanfd->SIDFC =
        ((uint32_t)(psElemSize->u32SIDFC & 0xFFUL) << CANFD_SIDFC_LSS_Pos) |
        (psRamConfig->u32SIDFC_FLSSA & CANFD_SIDFC_FLSSA_Msk);

    /*Get the Filter List Standard Start Address in the RAM*/
    u32Address =
        CANFD_SRAM_BASE_ADDR(psCanfd) +
        (psRamConfig->u32SIDFC_FLSSA & CANFD_SIDFC_FLSSA_Msk);

    /*Clear the Filter List Memory*/
    (void)memset((void *)u32Address, 0, (uint32_t)(psElemSize->u32SIDFC * sizeof(CANFD_STD_FILTER_T)));
}


/**
 * @brief       Configures the register XIDFC for the 29-bit Extended Message ID Filter elements.
 *
 * @param[in]   psCanfd           The pointer to CAN FD module base address.
 * @param[in]   psRamConfig       Extended ID filter configuration ram address
 * @param[in]   psElemSize        Extended ID filter configuration element size
 *
 * @return      None.
 *
 * @details     Configures the register XIDFC for the 29-bit Extended Message ID Filter elements.
 */
static void CANFD_ConfigXIDFC(CANFD_T *psCanfd, CANFD_RAM_PART_T *psRamConfig, const CANFD_ELEM_SIZE_T *psElemSize)
{
    uint32_t u32Address;

    /*Setting the Filter List Extended Start Address and List Size  */
    psCanfd->XIDFC =
        ((uint32_t)(psElemSize->u32XIDFC & 0xFFUL) << CANFD_XIDFC_LSE_Pos) |
        (psRamConfig->u32XIDFC_FLESA & CANFD_XIDFC_FLESA_Msk);

    /*Get the Filter List Standard Start Address in the RAM*/
    u32Address =
        CANFD_SRAM_BASE_ADDR(psCanfd) +
        (psRamConfig->u32XIDFC_FLESA & CANFD_XIDFC_FLESA_Msk);

    /*Clear the Filter List Memory*/
    (void)memset((void *)u32Address, 0, (uint32_t)(psElemSize->u32XIDFC * sizeof(CANFD_EXT_FILTER_T)));
}

/**
 * @brief       Writes a 11-bit Standard ID filter element in the Message RAM.
 *
 * @param[in]   psCanfd          The pointer to CAN FD module base address.
 * @param[in]   u32FltrIdx       Index at which the filter element should be written in the '11-bit Filter' section of Message RAM
 * @param[in]   u32Filter        Rx Individual filter value.
 *
 * @return      None.
 *
 * @details     Writes a 11-bit Standard ID filter element in the Message RAM.
 */
void CANFD_SetSIDFltr(const CANFD_T *psCanfd, uint32_t u32FltrIdx, uint32_t u32Filter)
{
    CANFD_STD_FILTER_T *psFilter;

    /* ignore if index is too high */
    if (u32FltrIdx >= CANFD_MAX_11_BIT_FTR_ELEMS)
    {
        return;
    }

    /*Get the Filter List Configuration Address in the RAM*/
    psFilter =
        (CANFD_STD_FILTER_T *)
        (CANFD_SRAM_BASE_ADDR(psCanfd) +
         (psCanfd->SIDFC & CANFD_SIDFC_FLSSA_Msk) +
         (u32FltrIdx * sizeof(CANFD_STD_FILTER_T)));

    /*Wirted the Standard ID filter element to RAM */
    psFilter->VALUE = u32Filter;
}

/**
 * @brief       Writes a 29-bit extended id filter element in the Message RAM.
 *              Size of an Extended Id filter element is 2 words. So 2 words are written into the Message RAM for each filter element
 *
 * @param[in]   psCanfd         The pointer to CAN FD module base address.
 * @param[in]   u32FltrIdx      Index at which the filter element should be written in the '29-bit Filter' section of Message RAM.
 * @param[in]   u32FilterLow    Rx Individual filter low value.
 * @param[in]   u32FilterHigh   Rx Individual filter high value.
 *
 * @return      None.
 *
 * @details     Writes a 29-bit extended id filter element in the Message RAM.
 */
void CANFD_SetXIDFltr(const CANFD_T *psCanfd, uint32_t u32FltrIdx, uint32_t u32FilterLow, uint32_t u32FilterHigh)
{
    CANFD_EXT_FILTER_T *psFilter;

    /* ignore if index is too high */
    if (u32FltrIdx >= CANFD_MAX_29_BIT_FTR_ELEMS)
    {
        return;
    }

    /*Get the Filter List Configuration Address on RAM*/
    psFilter =
        (CANFD_EXT_FILTER_T *)
        (CANFD_SRAM_BASE_ADDR(psCanfd) +
         (psCanfd->XIDFC & CANFD_XIDFC_FLESA_Msk) +
         (u32FltrIdx * sizeof(CANFD_EXT_FILTER_T)));

    /*Wirted the Extended ID filter element to RAM */
    psFilter->LOWVALUE  = u32FilterLow;
    psFilter->HIGHVALUE = u32FilterHigh;

}


/**
 * @brief       Reads a CAN FD Message from Receive Message Buffer.
 *
 * @param[in]   psCanfd     The pointer of the specified CAN FD module.
 * @param[in]   u8MbIdx     The CANFD Message Buffer index.
 * @param[in]   psMsgBuf    Pointer to CAN FD message frame structure for reception.
 *
 * @return       1:Rx Message Buffer is full and has been read successfully.
 *               0:Rx Message Buffer is empty.
 *
 * @details     This function reads a CAN message from a specified Receive Message Buffer.
 *              The function fills a receive CAN message frame structure with just received data
 *              and activates the Message Buffer again.The function returns immediately.
*/
uint32_t CANFD_ReadRxBufMsg(CANFD_T *psCanfd, uint8_t u8MbIdx, CANFD_FD_MSG_T *psMsgBuf)
{
    CANFD_BUF_T *psRxBuffer;
    uint32_t u32Success = 0UL;

    if (u8MbIdx < CANFD_MAX_RX_BUF_ELEMS)
    {
        uint32_t newData = 0UL;
        uint32_t mbIdx32 = (uint32_t)u8MbIdx;

        if (mbIdx32 < 32UL)
        {
            newData = (psCanfd->NDAT1 >> mbIdx32) & 1UL;
        }
        else
        {
            newData = (psCanfd->NDAT2 >> (mbIdx32 - 32UL)) & 1UL;
        }

        /* new message is waiting to be read */
        if (newData != 0UL)
        {
            /* get memory location of rx buffer */
            psRxBuffer =
                (CANFD_BUF_T *)
                (CANFD_SRAM_BASE_ADDR(psCanfd) +
                 (psCanfd->RXBC & 0xFFFFUL) +
                 (mbIdx32 * sizeof(CANFD_BUF_T)));

            /* read the message */
            CANFD_CopyDBufToMsgBuf(psRxBuffer, psMsgBuf);

            /* clear 'new data' flag */
            if (mbIdx32 < 32UL)
            {
                psCanfd->NDAT1 |= (1UL << mbIdx32);
            }
            else
            {
                psCanfd->NDAT2 |= (1UL << (mbIdx32 - 32UL));
            }

            u32Success = 1UL;
        }
    }

    return u32Success;
}


/**
 * @brief       Reads a CAN FD Message from Rx FIFO.
 *
 * @param[in]   psCanfd     The pointer of the specified CANFD module.
 * @param[in]   u8FifoIdx   Number of the FIFO, 0 or 1.
 * @param[in]   psMsgBuf    Pointer to CANFD message frame structure for reception.
 *
 * @return      1           Read Message from Rx FIFO successfully.
 *              2           Rx FIFO is already overflowed and has been read successfully
 *              0           Rx FIFO is not enabled.
 *
 * @details     This function reads a CAN message from the CANFD build-in Rx FIFO.
 */
uint32_t CANFD_ReadRxFifoMsg(CANFD_T *psCanfd, uint8_t u8FifoIdx, CANFD_FD_MSG_T *psMsgBuf)
{

    CANFD_BUF_T *pRxBuffer;
    uint32_t u32Success = 0UL;
    uint32_t fifoIdx = (uint32_t)u8FifoIdx;

    /* check for valid FIFO number */
    if (fifoIdx < CANFD_NUM_RX_FIFOS)
    {
        volatile const uint32_t *pRXFC;
        volatile       uint32_t *pRXFA;
        volatile const uint32_t *pRXFS;

        uint32_t msgLostBit;
        uint32_t newDataCount;

        if (fifoIdx == 0UL)
        {
            pRXFS = &(psCanfd->RXF0S);
            pRXFC = &(psCanfd->RXF0C);
            pRXFA = &(psCanfd->RXF0A);
            msgLostBit = 3UL;
        }
        else
        {
            pRXFS = &(psCanfd->RXF1S);
            pRXFC = &(psCanfd->RXF1C);
            pRXFA = &(psCanfd->RXF1A);
            msgLostBit = 7UL;
        }

        /* if FIFO is not empty */
        newDataCount = (*pRXFS & 0x7FUL);
        if (newDataCount > 0UL)
        {
            uint32_t getIndex = (uint32_t)(((*pRXFS) >> 8U) & 0x3FUL);

            pRxBuffer =
                (CANFD_BUF_T *)
                (CANFD_SRAM_BASE_ADDR(psCanfd) +
                 (*pRXFC & 0xFFFFUL) +
                 (getIndex * sizeof(CANFD_BUF_T)));

            CANFD_CopyRxFifoToMsgBuf(pRxBuffer, psMsgBuf);

            /* acknowledge FIFO element */
            *pRXFA = getIndex;

            /* check for overflow */
            if ((*pRXFS & CANFD_RXFS_RFL) != 0UL)
            {
                psCanfd->IR = (1UL << msgLostBit);
                u32Success = 2UL;
            }
            else
            {
                u32Success = 1UL;
            }
        }
    }

    return u32Success;
}


/**
 * @brief       Copies a message from a dedicated Rx buffer into a message buffer.
 *
 * @param[in]   psRxBuf         Buffer to read from.
 * @param[in]   psMsgBuf        Location to store read message.
 *
 * @return      None.
 *
 * @details     Copies a message from a dedicated Rx buffer into a message buffer.
 */
void CANFD_CopyDBufToMsgBuf(CANFD_BUF_T *psRxBuf, CANFD_FD_MSG_T *psMsgBuf)
{
    uint32_t u32Idx;

    if (psRxBuf->u32Id & RX_BUFFER_AND_FIFO_R0_ELEM_ESI_Msk)
    {
        psMsgBuf->bErrStaInd = TRUE;
    }
    else
    {
        psMsgBuf->bErrStaInd = FALSE;
    }
    /* if 29-bit ID */
    if (psRxBuf->u32Id & RX_BUFFER_AND_FIFO_R0_ELEM_XTD_Msk)
    {
        psMsgBuf->u32Id = (psRxBuf->u32Id & RX_BUFFER_AND_FIFO_R0_ELEM_ID_Msk);
        psMsgBuf->eIdType = eCANFD_XID;
    }
    /* if 11-bit ID */
    else
    {
        psMsgBuf->u32Id = (psRxBuf->u32Id  >> 18) & 0x7FFUL;
        psMsgBuf->eIdType = eCANFD_SID;
    }

    if (psRxBuf->u32Id  & RX_BUFFER_AND_FIFO_R0_ELEM_RTR_Msk)
    {
        psMsgBuf->eFrmType = eCANFD_REMOTE_FRM;
    }
    else
    {
        psMsgBuf->eFrmType = eCANFD_DATA_FRM;
    }

    if (psRxBuf->u32Config &  RX_BUFFER_AND_FIFO_R1_ELEM_FDF_Msk)
    {
        psMsgBuf->bFDFormat = TRUE;
    }
    else
    {
        psMsgBuf->bFDFormat = FALSE;
    }
    if (psRxBuf->u32Config &  RX_BUFFER_AND_FIFO_R1_ELEM_BSR_Msk)
    {
        psMsgBuf->bBitRateSwitch = TRUE;
    }
    else
    {
        psMsgBuf->bBitRateSwitch = FALSE;
    }
    psMsgBuf->u32DLC = CANFD_DecodeDLC((psRxBuf->u32Config & RX_BUFFER_AND_FIFO_R1_ELEM_DLC_Msk) >> RX_BUFFER_AND_FIFO_R1_ELEM_DLC_Pos);

    for (u32Idx = 0 ; u32Idx < psMsgBuf->u32DLC ; u32Idx++)
    {
        psMsgBuf->au8Data[u32Idx] = psRxBuf->au8Data[u32Idx];
    }
}


/**
 * @brief       Get Rx FIFO water level.
 *
 * @param[in]   psCanfd         The pointer to CANFD module base address.
 * @param[in]   u32RxFifoNum    0: RX FIFO_0, 1: RX_FIFO_1
 *
 * @return      Rx FIFO water level.
 *
 * @details     Get Rx FIFO water level.
 */
uint32_t CANFD_GetRxFifoWaterLvl(const CANFD_T *psCanfd, uint32_t u32RxFifoNum)
{
    uint32_t u32WaterLevel = 0UL;

    if (u32RxFifoNum == 0UL)
    {
        u32WaterLevel = ((psCanfd->RXF0C & CANFD_RXF0C_F0WM_Msk) >> CANFD_RXF0C_F0WM_Pos);
    }
    else
    {
        u32WaterLevel = ((psCanfd->RXF1C & CANFD_RXF1C_F1WM_Msk) >> CANFD_RXF1C_F1WM_Pos);
    }
    return u32WaterLevel;
}


/**
 * @brief       Copies messages from FIFO into a message buffert.
 *
 * @param[in]   psRxBuf         Buffer to read from.
 * @param[in]   psMsgBuf        Location to store read message.
 *
 * @return      None.
 *
 * @details      Copies messages from FIFO into a message buffert.
 */
void CANFD_CopyRxFifoToMsgBuf(CANFD_BUF_T *psRxBuf, CANFD_FD_MSG_T *psMsgBuf)
{
    /*Copies a message from a dedicated Rx FIFO into a message buffer*/
    CANFD_CopyDBufToMsgBuf(psRxBuf, psMsgBuf);
}


/**
 * @brief       Cancel a Tx buffer transmission request.
 *
 * @param[in]   psCanfd         The pointer to CANFD module base address.
 * @param[in]   u32TxBufIdx     Tx buffer index number
 *
 * @return      None.
 *
 * @details     Cancel a Tx buffer transmission request.
 */
void CANFD_TxBufCancelReq(CANFD_T *psCanfd, uint32_t u32TxBufIdx)
{
    psCanfd->TXBCR |= (0x1UL << u32TxBufIdx);
}


/**
 * @brief       Checks if a Tx buffer cancellation request has been finished or not.
 *
 * @param[in]   psCanfd         The pointer to CAN FD module base address.
 * @param[in]   u32TxBufIdx     Tx buffer index number
 *
 * @return      0: cancellation finished.
 *              1: cancellation fail
 *
 * @details     Checks if a Tx buffer cancellation request has been finished or not.
 */
uint32_t CANFD_IsTxBufCancelFin(const CANFD_T *psCanfd, uint32_t u32TxBufIdx)
{
    /* wait for completion */
    return ((psCanfd->TXBCR & (0x1UL << u32TxBufIdx)) >> u32TxBufIdx);
}


/**
 * @brief       Checks if a Tx buffer transmission has occurred or not.
 *
 * @param[in]   psCanfd         The pointer to CAN FD module base address.
 * @param[in]   u32TxBufIdx     Tx buffer index number
 *
 * @return     0: No transmission occurred.
 *             1: Transmission occurred
 *
 * @details     Checks if a Tx buffer transmission has occurred or not.
 */
uint32_t CANFD_IsTxBufTransmitOccur(const CANFD_T *psCanfd, uint32_t u32TxBufIdx)
{
    return ((psCanfd->TXBTO & (0x1UL << u32TxBufIdx)) >> u32TxBufIdx);
}


/**
 * @brief       Init Tx event fifo
 *
 * @param[in]   psCanfd          The pointer to CAN FD module base address.
 * @param[in]   psRamConfig      Tx Event Fifo configuration ram address.
 * @param[in]   psElemSize       Tx Event Fifo configuration element size
 * @param[in]   u32FifoWaterLvl  FIFO water level
 *
 * @return      None.
 *
 * @details     Init Tx event fifo.
 */
static void CANFD_InitTxEvntFifo(CANFD_T *psCanfd, const CANFD_RAM_PART_T *psRamConfig, const CANFD_ELEM_SIZE_T *psElemSize, const uint32_t u32FifoWaterLvl)
{
    /* Set TX Event FIFO element size,watermark,start address. */
    psCanfd->TXEFC = (u32FifoWaterLvl << CANFD_TXEFC_EFWM_Pos) | (psElemSize->u32TxEventFifo << CANFD_TXEFC_EFS_Pos)
                     | (psRamConfig->u32TXEFC_EFSA & CANFD_TXEFC_EFSA_Msk);
}


/**
 * @brief       Get Tx event fifo water level
 *
 * @param[in]   psCanfd       The pointer to CANFD module base address.
 *
 * @return      Tx event fifo water level.
 *
 * @details     Get Tx event fifo water level.
 */
uint32_t CANFD_GetTxEvntFifoWaterLvl(const CANFD_T *psCanfd)
{
    return ((psCanfd->TXEFC & CANFD_TXEFC_EFWM_Msk) >> CANFD_TXEFC_EFWM_Pos);
}


/**
 * @brief        Copy Event Elements from TX Event FIFO to user buffer
 *
 * @param[in]   psCanfd          The pointer to CAN FD module base address.
 * @param[in]   u32TxEvntNum     Tx Event FIFO number
 * @param[in]   psTxEvntElem     Tx Event Message struct
 *
 * @return      None.
 *
 * @details     Copy all Event Elements from TX Event FIFO to the Software Event List .
 */
void CANFD_CopyTxEvntFifoToUsrBuf(const CANFD_T *psCanfd, uint32_t u32TxEvntNum, CANFD_TX_EVNT_ELEM_T *psTxEvntElem)
{
    const uint32_t *pu32TxEvnt;
    /*Get the Tx Event FIFO Address*/
    pu32TxEvnt = (uint32_t *)(CANFD_SRAM_BASE_ADDR(psCanfd) + CANFD_GetTxBufferElementAddress(psCanfd, u32TxEvntNum));

    /*Get the Error State Indicator*/
    if ((pu32TxEvnt[0] & TX_FIFO_E0_EVENT_ESI_Msk) > 0UL)
    {
        psTxEvntElem->bErrStaInd = TRUE; //Transmitting node is error passive
    }
    else
    {
        psTxEvntElem->bErrStaInd = FALSE;//Transmitting node is error active
    }
    /*Get the Tx FIFO Identifier type and Identifier*/

    if ((pu32TxEvnt[0] & TX_FIFO_E0_EVENT_XTD_Msk) > 0UL)
    {
        psTxEvntElem->eIdType = eCANFD_XID;
        psTxEvntElem->u32Id = (pu32TxEvnt[0] & TX_FIFO_E0_EVENT_ID_Msk);// Extended ID
    }
    else
    {
        psTxEvntElem->eIdType = eCANFD_SID;
        psTxEvntElem->u32Id = (pu32TxEvnt[0] & TX_FIFO_E0_EVENT_ID_Msk) >> 18;// Standard ID
    }

    /*Get the Frame type*/
    if ((pu32TxEvnt[0] & TX_FIFO_E0_EVENT_RTR_Msk) > 0UL)
    {
        psTxEvntElem->bRemote = TRUE; //Remote frame
    }
    else
    {
        psTxEvntElem->bRemote = FALSE; //Data frame
    }
    /*Get the FD Format type*/
    if ((pu32TxEvnt[0] & TX_FIFO_E1_EVENT_FDF_Msk) > 0UL)
    {
        psTxEvntElem->bFDFormat = TRUE; //CAN FD frame format
    }
    else
    {
        psTxEvntElem->bFDFormat = FALSE; //Classical CAN frame format
    }
    /*Get the Bit Rate Switch type*/
    if ((pu32TxEvnt[0] & TX_FIFO_E1_EVENT_BRS_Msk) > 0UL)
    {
        psTxEvntElem->bBitRateSwitch = TRUE; //Frame transmitted with bit rate switching
    }
    else
    {
        psTxEvntElem->bBitRateSwitch = FALSE; //Frame transmitted without bit rate switching
    }
    /*Get the Tx FIFO Data Length  */
    psTxEvntElem->u32DLC = CANFD_DecodeDLC((uint8_t)((pu32TxEvnt[1] & TX_FIFO_E1_EVENT_DLC_Msk) >> TX_FIFO_E1_EVENT_DLC_Pos));

    /*Get the Tx FIFO Timestamp  */
    psTxEvntElem->u32TxTs = (((pu32TxEvnt[1] & TX_FIFO_E1A_EVENT_TXTS_Msk) >> TX_FIFO_E1A_EVENT_TXTS_Pos));
    /*Get the Tx FIFO Message marker  */
    psTxEvntElem->u32MsgMarker = (((pu32TxEvnt[1] & TX_FIFO_E1_EVENT_MM_Msk) >> TX_FIFO_E1_EVENT_MM_Pos));
}


/**
 * @brief       Get CAN FD interrupts status.
 *
 * @param[in]   psCanfd         The pointer of the specified CAN FD module.
 * @param[in]   u32IntTypeFlag  Interrupt Type Flag, should be
 *                              - \ref CANFD_IR_ARA_Msk     : Access to Reserved Address interrupt Indicator
 *                              - \ref CANFD_IR_PED_Msk     : Protocol Error in Data Phase interrupt Indicator
 *                              - \ref CANFD_IR_PEA_Msk     : Protocol Error in Arbitration Phase interrupt Indicator
 *                              - \ref CANFD_IR_WDI_Msk     : Watchdog interrupt Indicator
 *                              - \ref CANFD_IR_BO_Msk      : Bus_Off Status interrupt Indicator
 *                              - \ref CANFD_IR_EW_Msk      : Warning Status interrupt Indicator
 *                              - \ref CANFD_IR_EP_Msk      : Error Passive interrupt Indicator
 *                              - \ref CANFD_IR_ELO_Msk     : Error Logging Overflow interrupt Indicator
 *                              - \ref CANFD_IR_DRX_Msk     : Message stored to Dedicated Rx Buffer interrupt Indicator
 *                              - \ref CANFD_IR_TOO_Msk     : Timeout Occurred interrupt Indicator
 *                              - \ref CANFD_IR_MRAF_Msk    : Message RAM Access Failure interrupt Indicator
 *                              - \ref CANFD_IR_TSW_Msk     : Timestamp Wraparound interrupt Indicator
 *                              - \ref CANFD_IR_TEFL_Msk    : Tx Event FIFO Event Lost interrupt Indicator
 *                              - \ref CANFD_IR_TEFF_Msk    : Tx Event FIFO Full Indicator
 *                              - \ref CANFD_IR_TEFW_Msk    : Tx Event FIFO Watermark Reached Interrupt Indicator
 *                              - \ref CANFD_IR_TEFN_Msk    : Tx Event FIFO New Entry Interrupt Indicator
 *                              - \ref CANFD_IR_TFE_Msk     : Tx FIFO Empty Interrupt Indicator
 *                              - \ref CANFD_IR_TCF_Msk     : Transmission Cancellation Finished Interrupt Indicator
 *                              - \ref CANFD_IR_TC_Msk      : Transmission Completed interrupt Indicator
 *                              - \ref CANFD_IR_HPM_Msk     : High Priority Message Interrupt Indicator
 *                              - \ref CANFD_IR_RF1L_Msk    : Rx FIFO 1 Message Lost Interrupt Indicator
 *                              - \ref CANFD_IR_RF1F_Msk    : Rx FIFO 1 Full Interrupt Indicator
 *                              - \ref CANFD_IR_RF1W_Msk    : Rx FIFO 1 Watermark Reached Interrupt Indicator
 *                              - \ref CANFD_IR_RF1N_Msk    : Rx FIFO 1 New Message Interrupt Indicator
 *                              - \ref CANFD_IR_RF0L_Msk    : Rx FIFO 0 Message Lost Interrupt Indicator
 *                              - \ref CANFD_IR_RF0F_Msk    : Rx FIFO 0 Full Interrupt Indicator
 *                              - \ref CANFD_IR_RF0W_Msk    : Rx FIFO 0 Watermark Reached Interrupt Indicator
 *                              - \ref CANFD_IR_RF0N_Msk    : Rx FIFO 0 New Message Interrupt Indicator
 *
 * @return      None.
 *
 * @details     This function gets all CAN FD interrupt status flags.
 */
uint32_t CANFD_GetStatusFlag(const CANFD_T *psCanfd, uint32_t u32IntTypeFlag)
{
    return (psCanfd->IR & u32IntTypeFlag);
}


/**
 * @brief       Clears the CAN FD module interrupt flags
 *
 * @param[in]   psCanfd           The pointer of the specified CANFD module.
 * @param[in]   u32InterruptFlag  The specified interrupt of CAN FD module
 *                               - \ref CANFD_IR_ARA_Msk     : Access to Reserved Address interrupt Indicator
 *                               - \ref CANFD_IR_PED_Msk     : Protocol Error in Data Phase interrupt Indicator
 *                               - \ref CANFD_IR_PEA_Msk     : Protocol Error in Arbitration Phase interrupt Indicator
 *                               - \ref CANFD_IR_WDI_Msk     : Watchdog interrupt Indicator
 *                               - \ref CANFD_IR_BO_Msk      : Bus_Off Status interrupt Indicator
 *                               - \ref CANFD_IR_EW_Msk      : Warning Status interrupt Indicator
 *                               - \ref CANFD_IR_EP_Msk      : Error Passive interrupt Indicator
 *                               - \ref CANFD_IR_ELO_Msk     : Error Logging Overflow interrupt Indicator
 *                               - \ref CANFD_IR_DRX_Msk     : Message stored to Dedicated Rx Buffer interrupt Indicator
 *                               - \ref CANFD_IR_TOO_Msk     : Timeout Occurred interrupt Indicator
 *                               - \ref CANFD_IR_MRAF_Msk    : Message RAM Access Failure interrupt Indicator
 *                               - \ref CANFD_IR_TSW_Msk     : Timestamp Wraparound interrupt Indicator
 *                               - \ref CANFD_IR_TEFL_Msk    : Tx Event FIFO Event Lost interrupt Indicator
 *                               - \ref CANFD_IR_TEFF_Msk    : Tx Event FIFO Full Indicator
 *                               - \ref CANFD_IR_TEFW_Msk    : Tx Event FIFO Watermark Reached Interrupt Indicator
 *                               - \ref CANFD_IR_TEFN_Msk    : Tx Event FIFO New Entry Interrupt Indicator
 *                               - \ref CANFD_IR_TFE_Msk     : Tx FIFO Empty Interrupt Indicator
 *                               - \ref CANFD_IR_TCF_Msk     : Transmission Cancellation Finished Interrupt Indicator
 *                               - \ref CANFD_IR_TC_Msk      : Transmission Completed interrupt Indicator
 *                               - \ref CANFD_IR_HPM_Msk     : High Priority Message Interrupt Indicator
 *                               - \ref CANFD_IR_RF1L_Msk    : Rx FIFO 1 Message Lost Interrupt Indicator
 *                               - \ref CANFD_IR_RF1F_Msk    : Rx FIFO 1 Full Interrupt Indicator
 *                               - \ref CANFD_IR_RF1W_Msk    : Rx FIFO 1 Watermark Reached Interrupt Indicator
 *                               - \ref CANFD_IR_RF1N_Msk    : Rx FIFO 1 New Message Interrupt Indicator
 *                               - \ref CANFD_IR_RF0L_Msk    : Rx FIFO 0 Message Lost Interrupt Indicator
 *                               - \ref CANFD_IR_RF0F_Msk    : Rx FIFO 0 Full Interrupt Indicator
 *                               - \ref CANFD_IR_RF0W_Msk    : Rx FIFO 0 Watermark Reached Interrupt Indicator
 *                               - \ref CANFD_IR_RF0N_Msk    : Rx FIFO 0 New Message Interrupt Indicator
 *
 * @return      None.
 *
 * @details     This function clears CAN FD interrupt status flags.
 */
void CANFD_ClearStatusFlag(CANFD_T *psCanfd, uint32_t u32InterruptFlag)
{
    /* Write 1 to clear status flag. */
    psCanfd->IR |= u32InterruptFlag;
}


/**
 * @brief       Gets the CAN FD Bus Error Counter value.
 *
 * @param[in]   psCanfd        The pointer of the specified CAN FD module.
 * @param[in]   pu8TxErrBuf    TxErrBuf Buffer to store Tx Error Counter value.
 * @param[in]   pu8RxErrBuf    RxErrBuf Buffer to store Rx Error Counter value.
 *
 * @return      None.
 *
 * @details     This function gets the CAN FD Bus Error Counter value for both Tx and Rx direction.
 *              These values may be needed in the upper layer error handling.
 */
void CANFD_GetBusErrCount(const CANFD_T *psCanfd, uint8_t *pu8TxErrBuf, uint8_t *pu8RxErrBuf)
{
    if (pu8TxErrBuf)
    {
        *pu8TxErrBuf = (uint8_t)((psCanfd->ECR >> CANFD_ECR_TEC_Pos) & CANFD_ECR_TEC_Msk);
    }

    if (pu8RxErrBuf)
    {
        *pu8RxErrBuf = (uint8_t)((psCanfd->ECR >> CANFD_ECR_REC_Pos) & CANFD_ECR_REC_Msk);
    }
}


/**
 * @brief       CAN FD Run to the Normal Operation.
 *
 * @param[in]   psCanfd        The pointer of the specified CAN FD module.
 * @param[in]   u8Enable       TxErrBuf Buffer to store Tx Error Counter value.
 *
 * @retval      CANFD_OK          CANFD operation OK.
 * @retval      CANFD_ERR_TIMEOUT CANFD operation abort due to timeout error.
 *
 * @details     This function gets the CAN FD Bus Error Counter value for both Tx and Rx direction.
 *              These values may be needed in the upper layer error handling.
 */
int32_t CANFD_RunToNormal(CANFD_T *psCanfd, uint8_t u8Enable)
{
    uint32_t u32TimeOutCnt = CANFD_TIMEOUT;

    if (u8Enable)
    {
        /* start operation */
        psCanfd->CCCR &= ~(CANFD_CCCR_CCE_Msk | CANFD_CCCR_INIT_Msk);

        while (psCanfd->CCCR & CANFD_CCCR_INIT_Msk)
        {
            if (--u32TimeOutCnt == 0UL)
            {
                return CANFD_ERR_TIMEOUT;
            }
        }
    }
    else
    {
        /* init mode */
        psCanfd->CCCR |= CANFD_CCCR_INIT_Msk;

        while (!(psCanfd->CCCR & CANFD_CCCR_INIT_Msk))
        {
            if (--u32TimeOutCnt == 0UL)
            {
                return CANFD_ERR_TIMEOUT;
            }
        }
    }

    return CANFD_OK;
}



/*@}*/ /* end of group CANFD_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group CANFD_Driver */

/*@}*/ /* end of group Standard_Driver */
