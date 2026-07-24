/**************************************************************************//**
 * @file     i3c_reg.h
 * @version  V1.00
 * @brief    I3C register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __I3C_REG_H__
#define __I3C_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup I3C I3C Serial Interface Controller (I3C)
    Memory Mapped Structure for I3C Controller
    @{
*/


typedef struct
{
    union
    {
        __IO uint32_t WORD1;        /*!< I3C Target Mode Extended Command Word 1 Register */
        __IO uint32_t CFG;          /*!< Extended Command Configuration */
    };
    union
    {
        __IO uint32_t WORD2;        /*!< I3C Target Mode Extended Command Word 2 Register*/
        __IO uint32_t DATA;         /*!< Data Lenght and Defining Byte for Non HDR-BT Transfer */
        __IO uint32_t BTBYTES;      /*!< Command Bytes for HDR-BT Transfer */
    };
    union
    {
        __IO uint32_t WORD3;        /*!< I3C Target Mode Extended Command Word 3 Register*/
        __IO uint32_t BTRDLEN;      /*!< Data Lenght of HDR-BT Transfer Read Command */
    };
} EXTCMD_T;


typedef struct
{
    union
    {
        __I uint32_t CHAR1;             /*!< I3C Target Device Characteristic 1 Register */
        __I uint32_t PIDMSB;            /*!< The MSB 32-bit Value of Provisional ID */
    };
    union
    {
        __I uint32_t CHAR2;             /*!< I3C Target Device Characteristic 2 Register */
        __I uint32_t PIDLSB;            /*!< The LSB 16-bit Value of Provisional ID */
    };
    union
    {
        __I uint32_t CHAR3;             /*!< I3C Target Device Characteristic 3 Register */
        __I uint32_t BCRDCR;            /*!< Bus Characteristic and Device Characteristic Value */
    };
    union
    {
        __I uint32_t CHAR4;             /*!< I3C Target Device Characteristic 4 Register */
        __I uint32_t DADDR;             /*!< Dynamic Address */
    };
} TGTCHAR_T;


typedef struct
{
    __IO uint32_t ADDR;                 /*!< I3C Virtual Target Address Register */
    __IO uint32_t MID;                  /*!< I3C Virtual Target MIPI Manufacturer ID Register */
    __IO uint32_t PID;                  /*!< I3C Virtual Target Provisional ID Register */
    __IO uint32_t CHAR;                 /*!< I3C Virtual Target Characteristic Register */

    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE[4];           /*!< Reserved */
    /// @endcond //HIDDEN_SYMBOLS
} VTGTCFG_T;


typedef struct
{
    __IO uint32_t DEVCTL;               /*!< [0x0000] I3C Device Control Register */
    __IO uint32_t DEVADDR;              /*!< [0x0004] I3C Device Address Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE0;             /*!< [0x0008] Reserved */
    /// @endcond //HIDDEN_SYMBOLS
    __O uint32_t CMDQUE;                /*!< [0x000c] I3C Command Queue Port Register */
    __I uint32_t RESPQUE;               /*!< [0x0010] I3C Response Queue Port Register */
    //    union
    //    {
    //        __I uint32_t CTRRESP;           /*!< [0x0010] I3C Controller Response Data Structure Register */
    //        __I uint32_t TGTRESP;           /*!< [0x0010] I3C Targer Response Data Structure Register */
    //    };
    __IO uint32_t TXRXDAT;              /*!< [0x0014] I3C Transmit and Receive Data Port Register */
    union
    {
        __I uint32_t IBISTS;            /*!< [0x0018] I3C Controller In-Band Interrupt Queue Status Register */
        __I uint32_t IBIQUE;            /*!< [0x0018] I3C Controller In-Band Interrupt Queue Data Register */
    };
    __IO uint32_t QUETHCTL;             /*!< [0x001c] I3C Queue Threshold Control Register */
    __IO uint32_t DBTHCTL;              /*!< [0x0020] I3C Data Buffer Threshold Control Register */
    __IO uint32_t IBIQCTL;              /*!< [0x0024] I3C Controller IBI Queue Control Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE1[3];          /*!< [0x0028] ~ [0x0030] Reserved */
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t RSTCTL;               /*!< [0x0034] I3C Reset Control Register */
    __IO uint32_t SLVEVNTS;             /*!< [0x0038] I3C Target Event Status Register */
    __IO uint32_t INTSTS;               /*!< [0x003c] I3C Interrupt Status Register */
    __IO uint32_t INTSTSEN;             /*!< [0x0040] I3C Interrupt Status Enable Register */
    __IO uint32_t INTEN;                /*!< [0x0044] I3C Interrupt Signal Enable Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE2;             /*!< [0x0048] Reserved */
    /// @endcond //HIDDEN_SYMBOLS
    __I  uint32_t QUESTSLV;             /*!< [0x004c] I3C Queue Status Level Register */
    __I  uint32_t DBSTSLV;              /*!< [0x0050] I3C Data Buffer Status Level Register */
    __I  uint32_t PRESENTS;             /*!< [0x0054] I3C Present State Register */
    __I  uint32_t CCCDEVS;              /*!< [0x0058] I3C Target Device Operating Status Register */
    __I  uint32_t ADDRTBP;              /*!< [0x005c] I3C Controller Mode Device Address Table Pointer Register */
    __I  uint32_t CHRTBP;               /*!< [0x0060] I3C Controller Mode Device Characteristics Table Pointer Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE3[3];          /*!< [0x0064] ~ [0x006c] Reserved */
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t SLVMID;               /*!< [0x0070] I3C Target MIPI Manufacturer ID Register */
    __IO uint32_t SLVPID;               /*!< [0x0074] I3C Target Normal Provisional ID Register */
    __IO uint32_t SLVCHAR;              /*!< [0x0078] I3C Target Characteristic Register */
    __I  uint32_t SLVMXLEN;             /*!< [0x007c] I3C Target Maximum Write/Read Length Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE4;             /*!< [0x0080] Reserved */
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t MXDS;                 /*!< [0x0084] I3C Target Maximum Data Speed Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE5;             /*!< [0x0088] Reserved */
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t SIR;                  /*!< [0x008c] I3C Target Interrupt Request Control Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE6;             /*!< [0x0090] Reserved */
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t SIRDAT;               /*!< [0x0094] I3C Target Interrupt Request Data Register */
    __IO uint32_t SIRRESP;              /*!< [0x0098] I3C Target Interrupt Request Response Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE7[2];          /*!< [0x009c] ~ [0x00a0] Reserved */
    /// @endcond //HIDDEN_SYMBOLS
    __I  uint32_t INSTSTS;              /*!< [0x00a4] I3C Target Instant Status Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE8[2];          /*!< [0x00a8] ~ [0x00ac] Reserved */
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t DEVCTLE;              /*!< [0x00b0] I3C Device Control Extended Register */
    __IO uint32_t SCLOD;                /*!< [0x00b4] I3C Controller Open Drain SCL Timing Register */
    __IO uint32_t SCLPP;                /*!< [0x00b8] I3C Controller Push Pull SCL Timing Register */
    __IO uint32_t SCLFM;                /*!< [0x00bc] I3C Controller I2C Fast Mode SCL Timing Register */
    __IO uint32_t SCLFMP;               /*!< [0x00c0] I3C Controller I2C Fast Mode Plus SCL Timing Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE9;             /*!< [0x00c4] Reserved */
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t SCLEXTLO;             /*!< [0x00c8] I3C Controller SCL Extended Low Count Timing Register */
    __IO uint32_t SCLEXTTB;             /*!< [0x00cc] I3C Controller SCL Termination Bit Low Count Timing Register */
    __IO uint32_t SDAHOLD;              /*!< [0x00d0] I3C Controller SDA Hold And Mode Switch Delay Timing Register */
    __IO uint32_t BUSFAT;               /*!< [0x00d4] I3C Bus Free And Available Timing Register */
    __IO uint32_t BUSIDLET;             /*!< [0x00d8] I3C Target Bus Idle Timing Register */
    __IO uint32_t SCLLOWTO;             /*!< [0x00dc] I3C Controller SCL Low Timeout Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE10[3];         /*!< [0x00e0] ~ [0x00e8] Reserved */
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t RELSDA;               /*!< [0x00ec] I3C Target Release SDA Timing Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE11;            /*!< [0x00f0] Reserved */
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t BTDELY;               /*!< [0x00f4] I3C HDR-BT Delay Byte Counter Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE12[14];        /*!< [0x00f8] ~ [0x012c] Reserved */
    /// @endcond //HIDDEN_SYMBOLS
    __O  uint32_t EXTDAT[8];            /*!< [0x0130] ~ [0x014c] I3C Target Mode Extended Cpmmand 0 ~ 7 Transfer Data Port Register */
    __I  uint32_t EXTTXTHS;             /*!< [0x0150] I3C Target Mode Extended Command Transmit Threshold Status Register */
    __I  uint32_t EXTDBSL[4];           /*!< [0x0154] ~ [0x0160] I3C Target Mode Extended Command Transmit Data Buffer Status Level Register 0 ~ 3 */
    EXTCMD_T      EXTCMD[8];            /*!< [0x0164] ~ [0x01c0] I3C Target Mode Extended Command 0 ~ 7 Register */
    __IO uint32_t EXTDBRST;             /*!< [0x01c4] I3C Target Mode Extended Command Transmit Data Buffer Reset Register */
    __I  uint32_t EXTCMDFS;             /*!< [0x01c8] I3C Target Mode Extended Command Finished Status Register */
    __IO uint32_t EXTTHLD;              /*!< [0x01cc] I3C Target Mode Extended Command Transmit and Receive Response Threshold Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE13[4];         /*!< [0x01d0] ~ [0x01dc] Reserved */
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t GRPASTS[4];           /*!< [0x01e0] ~ [0x01ec] I3C Target Mode Group Address 0 ~ 3 Status Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE14[4];         /*!< [0x01f0] ~ [0x01fc] Reserved */
    /// @endcond //HIDDEN_SYMBOLS
    TGTCHAR_T     TGTCHAR[7];           /*!< [0x0200] ~ [0x026c] I3C Target Device 1 ~ 7 Characteristic Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE15[4];         /*!< [0x0270] ~ [0x027c] Reserved */
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t TGTCFG[7];            /*!< [0x0280] ~ [0x0298] I3C Target Device 1 ~ 7 Configuration Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE16[153];       /*!< [0x029c] ~ [0x04fc] Reserved */
    /// @endcond //HIDDEN_SYMBOLS
    VTGTCFG_T     VTGTCFG[4];           /*!< [0x0500] ~ [0x056c] I3C Virtual Target 1 ~ 4 Configuration Register */

} I3C_T;

/**
    @addtogroup I3C_CONST I3C Bit Field Definition
    Constant Definitions for I3C Controller
    @{
*/

#define I3C_DEVCTL_IBAINCL_Pos    (0U)                                        /*!< I3C_T::DEVCTL: IBAINCL Position */
#define I3C_DEVCTL_IBAINCL_Msk    (0x1UL << I3C_DEVCTL_IBAINCL_Pos)           /*!< I3C_T::DEVCTL: IBAINCL Mask */

#define I3C_DEVCTL_PENDINT_Pos    (3U)                                        /*!< I3C_T::DEVCTL: PENDINT Position */
#define I3C_DEVCTL_PENDINT_Msk    (0xFUL << I3C_DEVCTL_PENDINT_Pos)           /*!< I3C_T::DEVCTL: PENDINT Mask */

#define I3C_DEVCTL_HJCTL_Pos      (8U)                                        /*!< I3C_T::DEVCTL: HJCTL Position */
#define I3C_DEVCTL_HJCTL_Msk      (0x1UL << I3C_DEVCTL_HJCTL_Pos)             /*!< I3C_T::DEVCTL: HJCTL Mask */

#define I3C_DEVCTL_TSCEN_Pos      (11U)                                       /*!< I3C_T::DEVCTL: TSCEN Position */
#define I3C_DEVCTL_TSCEN_Msk      (0x1UL << I3C_DEVCTL_TSCEN_Pos)             /*!< I3C_T::DEVCTL: TSCEN Mask */

#define I3C_DEVCTL_TSCAUTO_Pos    (12U)                                       /*!< I3C_T::DEVCTL: TSCAUTO Position */
#define I3C_DEVCTL_TSCAUTO_Msk    (0x1UL << I3C_DEVCTL_TSCAUTO_Pos)           /*!< I3C_T::DEVCTL: TSCAUTO Mask */

#define I3C_DEVCTL_TSCCLR_Pos     (15U)                                       /*!< I3C_T::DEVCTL: TSCCLR Position */
#define I3C_DEVCTL_TSCCLR_Msk     (0x1UL << I3C_DEVCTL_TSCCLR_Pos)            /*!< I3C_T::DEVCTL: TSCCLR Mask */

#define I3C_DEVCTL_IBIPSIZE_Pos   (16U)                                       /*!< I3C_T::DEVCTL: IBIPSIZE Position */
#define I3C_DEVCTL_IBIPSIZE_Msk   (0xFFUL << I3C_DEVCTL_IBIPSIZE_Pos)         /*!< I3C_T::DEVCTL: IBIPSIZE Mask */

#define I3C_DEVCTL_IDLECNT_Pos    (24U)                                       /*!< I3C_T::DEVCTL: IDLECNT Position */
#define I3C_DEVCTL_IDLECNT_Msk    (0x3UL << I3C_DEVCTL_IDLECNT_Pos)           /*!< I3C_T::DEVCTL: IDLECNT Mask */

#define I3C_DEVCTL_TPECEN_Pos     (26U)                                       /*!< I3C_T::DEVCTL: TPECEN Position */
#define I3C_DEVCTL_TPECEN_Msk     (0x1UL << I3C_DEVCTL_TPECEN_Pos)            /*!< I3C_T::DEVCTL: TPECEN Mask */

#define I3C_DEVCTL_ADAPTIVE_Pos   (27U)                                       /*!< I3C_T::DEVCTL: ADAPTIVE Position */
#define I3C_DEVCTL_ADAPTIVE_Msk   (0x1UL << I3C_DEVCTL_ADAPTIVE_Pos)          /*!< I3C_T::DEVCTL: ADAPTIVE Mask */

#define I3C_DEVCTL_DMAEN_Pos      (28U)                                       /*!< I3C_T::DEVCTL: DMAEN Position */
#define I3C_DEVCTL_DMAEN_Msk      (0x1UL << I3C_DEVCTL_DMAEN_Pos)             /*!< I3C_T::DEVCTL: DMAEN Mask */

#define I3C_DEVCTL_ABORT_Pos      (29U)                                       /*!< I3C_T::DEVCTL: ABORT Position */
#define I3C_DEVCTL_ABORT_Msk      (0x1UL << I3C_DEVCTL_ABORT_Pos)             /*!< I3C_T::DEVCTL: ABORT Mask */

#define I3C_DEVCTL_RESUME_Pos     (30U)                                       /*!< I3C_T::DEVCTL: RESUME Position */
#define I3C_DEVCTL_RESUME_Msk     (0x1UL << I3C_DEVCTL_RESUME_Pos)            /*!< I3C_T::DEVCTL: RESUME Mask */

#define I3C_DEVCTL_ENABLE_Pos     (31U)                                       /*!< I3C_T::DEVCTL: ENABLE Position */
#define I3C_DEVCTL_ENABLE_Msk     (0x1UL << I3C_DEVCTL_ENABLE_Pos)            /*!< I3C_T::DEVCTL: ENABLE Mask */

#define I3C_DEVADDR_SA_Pos        (0U)                                        /*!< I3C_T::DEVADDR: SA Position */
#define I3C_DEVADDR_SA_Msk        (0x7FUL << I3C_DEVADDR_SA_Pos)              /*!< I3C_T::DEVADDR: SA Mask */

#define I3C_DEVADDR_SAVALID_Pos   (15U)                                       /*!< I3C_T::DEVADDR: SAVALID Position */
#define I3C_DEVADDR_SAVALID_Msk   (0x1UL << I3C_DEVADDR_SAVALID_Pos)          /*!< I3C_T::DEVADDR: SAVALID Mask */

#define I3C_DEVADDR_DA_Pos        (16U)                                       /*!< I3C_T::DEVADDR: DA Position */
#define I3C_DEVADDR_DA_Msk        (0x7FUL << I3C_DEVADDR_DA_Pos)              /*!< I3C_T::DEVADDR: DA Mask */

#define I3C_DEVADDR_DAVALID_Pos   (31U)                                       /*!< I3C_T::DEVADDR: DAVALID Position */
#define I3C_DEVADDR_DAVALID_Msk   (0x1UL << I3C_DEVADDR_DAVALID_Pos)          /*!< I3C_T::DEVADDR: DAVALID Mask */

#define I3C_CMDQUE_CMDQUE_Pos     (0U)                                        /*!< I3C_T::CMDQUE: CMDQUE Position */
#define I3C_CMDQUE_CMDQUE_Msk     (0xFFFFFFFFUL << I3C_CMDQUE_CMDQUE_Pos)     /*!< I3C_T::CMDQUE: CMDQUE Mask */

#define I3C_RESPQUE_RESPQUE_Pos   (0U)                                        /*!< I3C_T::RESPQUE: RESPQUE Position */
#define I3C_RESPQUE_RESPQUE_Msk   (0xFFFFFFFFUL << I3C_RESPQUE_RESPQUE_Pos)   /*!< I3C_T::RESPQUE: RESPQUE Mask */

#define I3C_TXRXDAT_DAT_Pos       (0U)                                        /*!< I3C_T::TXRXDAT: DAT Position */
#define I3C_TXRXDAT_DAT_Msk       (0xFFFFFFFFUL << I3C_TXRXDAT_DAT_Pos)       /*!< I3C_T::TXRXDAT: DAT Mask */

#define I3C_IBISTS_DATLEN_Pos     (0U)                                        /*!< I3C_T::IBISTS: DATLEN Position */
#define I3C_IBISTS_DATLEN_Msk     (0xFFUL << I3C_IBISTS_DATLEN_Pos)           /*!< I3C_T::IBISTS: DATLEN Mask */

#define I3C_IBISTS_IBIID_Pos      (8U)                                        /*!< I3C_T::IBISTS: IBIID Position */
#define I3C_IBISTS_IBIID_Msk      (0xFFUL << I3C_IBISTS_IBIID_Pos)            /*!< I3C_T::IBISTS: IBIID Mask */

#define I3C_IBISTS_IBISTS_Pos     (28U)                                       /*!< I3C_T::IBISTS: IBISTS Position */
#define I3C_IBISTS_IBISTS_Msk     (0xFUL << I3C_IBISTS_IBISTS_Pos)            /*!< I3C_T::IBISTS: IBISTS Mask */

#define I3C_IBIQUE_IBIDAT_Pos     (0U)                                        /*!< I3C_T::IBIQUE: IBIDAT Position */
#define I3C_IBIQUE_IBIDAT_Msk     (0xFFFFFFFFUL << I3C_IBIQUE_IBIDAT_Pos)     /*!< I3C_T::IBIQUE: IBIDAT Mask */

#define I3C_QUETHCTL_CMDETH_Pos   (0U)                                        /*!< I3C_T::QUETHCTL: CMDETH Position */
#define I3C_QUETHCTL_CMDETH_Msk   (0xFFUL << I3C_QUETHCTL_CMDETH_Pos)         /*!< I3C_T::QUETHCTL: CMDETH Mask */

#define I3C_QUETHCTL_RESPTH_Pos   (8U)                                        /*!< I3C_T::QUETHCTL: RESPTH Position */
#define I3C_QUETHCTL_RESPTH_Msk   (0xFFUL << I3C_QUETHCTL_RESPTH_Pos)         /*!< I3C_T::QUETHCTL: RESPTH Mask */

#define I3C_QUETHCTL_IBIDATTH_Pos (16U)                                       /*!< I3C_T::QUETHCTL: IBIDATTH Position */
#define I3C_QUETHCTL_IBIDATTH_Msk (0xFFUL << I3C_QUETHCTL_IBIDATTH_Pos)       /*!< I3C_T::QUETHCTL: IBIDATTH Mask */

#define I3C_QUETHCTL_IBISTSTH_Pos (24U)                                       /*!< I3C_T::QUETHCTL: IBISTSTH Position */
#define I3C_QUETHCTL_IBISTSTH_Msk (0xFFUL << I3C_QUETHCTL_IBISTSTH_Pos)       /*!< I3C_T::QUETHCTL: IBISTSTH Mask */

#define I3C_DBTHCTL_TXTH_Pos      (0U)                                        /*!< I3C_T::DBTHCTL: TXTH Position */
#define I3C_DBTHCTL_TXTH_Msk      (0x7UL << I3C_DBTHCTL_TXTH_Pos)             /*!< I3C_T::DBTHCTL: TXTH Mask */

#define I3C_DBTHCTL_RXTH_Pos      (8U)                                        /*!< I3C_T::DBTHCTL: RXTH Position */
#define I3C_DBTHCTL_RXTH_Msk      (0x7UL << I3C_DBTHCTL_RXTH_Pos)             /*!< I3C_T::DBTHCTL: RXTH Mask */

#define I3C_DBTHCTL_TXSTATH_Pos   (16U)                                       /*!< I3C_T::DBTHCTL: TXSTATH Position */
#define I3C_DBTHCTL_TXSTATH_Msk   (0x7UL << I3C_DBTHCTL_TXSTATH_Pos)          /*!< I3C_T::DBTHCTL: TXSTATH Mask */

#define I3C_DBTHCTL_RXSTATH_Pos   (24U)                                       /*!< I3C_T::DBTHCTL: RXSTATH Position */
#define I3C_DBTHCTL_RXSTATH_Msk   (0x7UL << I3C_DBTHCTL_RXSTATH_Pos)          /*!< I3C_T::DBTHCTL: RXSTATH Mask */

#define I3C_IBIQCTL_HJREJ_Pos     (0U)                                        /*!< I3C_T::IBIQCTL: HJREJ Position */
#define I3C_IBIQCTL_HJREJ_Msk     (0x1UL << I3C_IBIQCTL_HJREJ_Pos)            /*!< I3C_T::IBIQCTL: HJREJ Mask */

#define I3C_IBIQCTL_MRREJ_Pos     (1U)                                        /*!< I3C_T::IBIQCTL: MRREJ Position */
#define I3C_IBIQCTL_MRREJ_Msk     (0x1UL << I3C_IBIQCTL_MRREJ_Pos)            /*!< I3C_T::IBIQCTL: MRREJ Mask */

#define I3C_IBIQCTL_SIRREJ_Pos    (3U)                                        /*!< I3C_T::IBIQCTL: SIRREJ Position */
#define I3C_IBIQCTL_SIRREJ_Msk    (0x1UL << I3C_IBIQCTL_SIRREJ_Pos)           /*!< I3C_T::IBIQCTL: SIRREJ Mask */

#define I3C_RSTCTL_SWRST_Pos      (0U)                                        /*!< I3C_T::RSTCTL: SWRST Position */
#define I3C_RSTCTL_SWRST_Msk      (0x1UL << I3C_RSTCTL_SWRST_Pos)             /*!< I3C_T::RSTCTL: SWRST Mask */

#define I3C_RSTCTL_CMDRST_Pos     (1U)                                        /*!< I3C_T::RSTCTL: CMDRST Position */
#define I3C_RSTCTL_CMDRST_Msk     (0x1UL << I3C_RSTCTL_CMDRST_Pos)            /*!< I3C_T::RSTCTL: CMDRST Mask */

#define I3C_RSTCTL_RESPRST_Pos    (2U)                                        /*!< I3C_T::RSTCTL: RESPRST Position */
#define I3C_RSTCTL_RESPRST_Msk    (0x1UL << I3C_RSTCTL_RESPRST_Pos)           /*!< I3C_T::RSTCTL: RESPRST Mask */

#define I3C_RSTCTL_TXRST_Pos      (3U)                                        /*!< I3C_T::RSTCTL: TXRST Position */
#define I3C_RSTCTL_TXRST_Msk      (0x1UL << I3C_RSTCTL_TXRST_Pos)             /*!< I3C_T::RSTCTL: TXRST Mask */

#define I3C_RSTCTL_RXRST_Pos      (4U)                                        /*!< I3C_T::RSTCTL: RXRST Position */
#define I3C_RSTCTL_RXRST_Msk      (0x1UL << I3C_RSTCTL_RXRST_Pos)             /*!< I3C_T::RSTCTL: RXRST Mask */

#define I3C_RSTCTL_IBIQRST_Pos    (5U)                                        /*!< I3C_T::RSTCTL: IBIQRST Position */
#define I3C_RSTCTL_IBIQRST_Msk    (0x1UL << I3C_RSTCTL_IBIQRST_Pos)           /*!< I3C_T::RSTCTL: IBIQRST Mask */

#define I3C_RSTCTL_BUSRSTPT_Pos   (28U)                                       /*!< I3C_T::RSTCTL: BUSRSTPT Position */
#define I3C_RSTCTL_BUSRSTPT_Msk   (0x7UL << I3C_RSTCTL_BUSRSTPT_Pos)          /*!< I3C_T::RSTCTL: BUSRSTPT Mask */

#define I3C_RSTCTL_BUSRST_Pos     (31U)                                       /*!< I3C_T::RSTCTL: BUSRST Position */
#define I3C_RSTCTL_BUSRST_Msk     (0x1UL << I3C_RSTCTL_BUSRST_Pos)            /*!< I3C_T::RSTCTL: BUSRST Mask */

#define I3C_SLVEVNTS_SIREN_Pos    (0U)                                        /*!< I3C_T::SLVEVNTS: SIREN Position */
#define I3C_SLVEVNTS_SIREN_Msk    (0x1UL << I3C_SLVEVNTS_SIREN_Pos)           /*!< I3C_T::SLVEVNTS: SIREN Mask */

#define I3C_SLVEVNTS_MREN_Pos     (1U)                                        /*!< I3C_T::SLVEVNTS: MREN Position */
#define I3C_SLVEVNTS_MREN_Msk     (0x1UL << I3C_SLVEVNTS_MREN_Pos)            /*!< I3C_T::SLVEVNTS: MREN Mask */

#define I3C_SLVEVNTS_HJEN_Pos     (3U)                                        /*!< I3C_T::SLVEVNTS: HJEN Position */
#define I3C_SLVEVNTS_HJEN_Msk     (0x1UL << I3C_SLVEVNTS_HJEN_Pos)            /*!< I3C_T::SLVEVNTS: HJEN Mask */

#define I3C_SLVEVNTS_ACTSTSTS_Pos (4U)                                        /*!< I3C_T::SLVEVNTS: ACTSTSTS Position */
#define I3C_SLVEVNTS_ACTSTSTS_Msk (0x3UL << I3C_SLVEVNTS_ACTSTSTS_Pos)        /*!< I3C_T::SLVEVNTS: ACTSTSTS Mask */

#define I3C_SLVEVNTS_MRLUPD_Pos   (6U)                                        /*!< I3C_T::SLVEVNTS: MRLUPD Position */
#define I3C_SLVEVNTS_MRLUPD_Msk   (0x1UL << I3C_SLVEVNTS_MRLUPD_Pos)          /*!< I3C_T::SLVEVNTS: MRLUPD Mask */

#define I3C_SLVEVNTS_MWLUPD_Pos   (7U)                                        /*!< I3C_T::SLVEVNTS: MWLUPD Position */
#define I3C_SLVEVNTS_MWLUPD_Msk   (0x1UL << I3C_SLVEVNTS_MWLUPD_Pos)          /*!< I3C_T::SLVEVNTS: MWLUPD Mask */

#define I3C_INTSTS_TXTH_Pos       (0U)                                        /*!< I3C_T::INTSTS: TXTH Position */
#define I3C_INTSTS_TXTH_Msk       (0x1UL << I3C_INTSTS_TXTH_Pos)              /*!< I3C_T::INTSTS: TXTH Mask */

#define I3C_INTSTS_RXTH_Pos       (1U)                                        /*!< I3C_T::INTSTS: RXTH Position */
#define I3C_INTSTS_RXTH_Msk       (0x1UL << I3C_INTSTS_RXTH_Pos)              /*!< I3C_T::INTSTS: RXTH Mask */

#define I3C_INTSTS_IBITH_Pos      (2U)                                        /*!< I3C_T::INTSTS: IBITH Position */
#define I3C_INTSTS_IBITH_Msk      (0x1UL << I3C_INTSTS_IBITH_Pos)             /*!< I3C_T::INTSTS: IBITH Mask */

#define I3C_INTSTS_CMDRDY_Pos     (3U)                                        /*!< I3C_T::INTSTS: CMDRDY Position */
#define I3C_INTSTS_CMDRDY_Msk     (0x1UL << I3C_INTSTS_CMDRDY_Pos)            /*!< I3C_T::INTSTS: CMDRDY Mask */

#define I3C_INTSTS_RESPRDY_Pos    (4U)                                        /*!< I3C_T::INTSTS: RESPRDY Position */
#define I3C_INTSTS_RESPRDY_Msk    (0x1UL << I3C_INTSTS_RESPRDY_Pos)           /*!< I3C_T::INTSTS: RESPRDY Mask */

#define I3C_INTSTS_TFRABORT_Pos   (5U)                                        /*!< I3C_T::INTSTS: TFRABORT Position */
#define I3C_INTSTS_TFRABORT_Msk   (0x1UL << I3C_INTSTS_TFRABORT_Pos)          /*!< I3C_T::INTSTS: TFRABORT Mask */

#define I3C_INTSTS_CCCUPD_Pos     (6U)                                        /*!< I3C_T::INTSTS: CCCUPD Position */
#define I3C_INTSTS_CCCUPD_Msk     (0x1UL << I3C_INTSTS_CCCUPD_Pos)            /*!< I3C_T::INTSTS: CCCUPD Mask */

#define I3C_INTSTS_DAA_Pos        (8U)                                        /*!< I3C_T::INTSTS: DAA Position */
#define I3C_INTSTS_DAA_Msk        (0x1UL << I3C_INTSTS_DAA_Pos)               /*!< I3C_T::INTSTS: DAA Mask */

#define I3C_INTSTS_TFRERR_Pos     (9U)                                        /*!< I3C_T::INTSTS: TFRERR Position */
#define I3C_INTSTS_TFRERR_Msk     (0x1UL << I3C_INTSTS_TFRERR_Pos)            /*!< I3C_T::INTSTS: TFRERR Mask */

#define I3C_INTSTS_DEFTGTS_Pos    (10U)                                       /*!< I3C_T::INTSTS: DEFTGTS Position */
#define I3C_INTSTS_DEFTGTS_Msk    (0x1UL << I3C_INTSTS_DEFTGTS_Pos)           /*!< I3C_T::INTSTS: DEFTGTS Mask */

#define I3C_INTSTS_READREQ_Pos    (11U)                                       /*!< I3C_T::INTSTS: READREQ Position */
#define I3C_INTSTS_READREQ_Msk    (0x1UL << I3C_INTSTS_READREQ_Pos)           /*!< I3C_T::INTSTS: READREQ Mask */

#define I3C_INTSTS_IBIUPD_Pos     (12U)                                       /*!< I3C_T::INTSTS: IBIUPD Position */
#define I3C_INTSTS_IBIUPD_Msk     (0x1UL << I3C_INTSTS_IBIUPD_Pos)            /*!< I3C_T::INTSTS: IBIUPD Mask */

#define I3C_INTSTS_BUSOWNER_Pos   (13U)                                       /*!< I3C_T::INTSTS: BUSOWNER Position */
#define I3C_INTSTS_BUSOWNER_Msk   (0x1UL << I3C_INTSTS_BUSOWNER_Pos)          /*!< I3C_T::INTSTS: BUSOWNER Mask */

#define I3C_INTSTS_BUSRSTDN_Pos   (15U)                                       /*!< I3C_T::INTSTS: BUSRSTDN Position */
#define I3C_INTSTS_BUSRSTDN_Msk   (0x1UL << I3C_INTSTS_BUSRSTDN_Pos)          /*!< I3C_T::INTSTS: BUSRSTDN Mask */

#define I3C_INTSTS_STADET_Pos     (16U)                                       /*!< I3C_T::INTSTS: STADET Position */
#define I3C_INTSTS_STADET_Msk     (0x1UL << I3C_INTSTS_STADET_Pos)            /*!< I3C_T::INTSTS: STADET Mask */

#define I3C_INTSTS_RSTPTDET_Pos   (17U)                                       /*!< I3C_T::INTSTS: RSTPTDET Position */
#define I3C_INTSTS_RSTPTDET_Msk   (0x1UL << I3C_INTSTS_RSTPTDET_Pos)          /*!< I3C_T::INTSTS: RSTPTDET Mask */

#define I3C_INTSTS_GRPADDRA_Pos   (18U)                                       /*!< I3C_T::INTSTS: GRPADDRA Position */
#define I3C_INTSTS_GRPADDRA_Msk   (0x1UL << I3C_INTSTS_GRPADDRA_Pos)          /*!< I3C_T::INTSTS: GRPADDRA Mask */

#define I3C_INTSTS_SDARES_Pos     (19U)                                       /*!< I3C_T::INTSTS: SDARES Position */
#define I3C_INTSTS_SDARES_Msk     (0x1UL << I3C_INTSTS_SDARES_Pos)            /*!< I3C_T::INTSTS: SDARES Mask */

#define I3C_INTSTS_EXTFINS_Pos    (20U)                                       /*!< I3C_T::INTSTS: EXTFINS Position */
#define I3C_INTSTS_EXTFINS_Msk    (0x1UL << I3C_INTSTS_EXTFINS_Pos)           /*!< I3C_T::INTSTS: EXTFINS Mask */

#define I3C_INTSTS_EXTTXTH_Pos    (21U)                                       /*!< I3C_T::INTSTS: EXTTXTH Position */
#define I3C_INTSTS_EXTTXTH_Msk    (0x1UL << I3C_INTSTS_EXTTXTH_Pos)           /*!< I3C_T::INTSTS: EXTTXTH Mask */

#define I3C_INTSTSEN_TXTH_Pos     (0U)                                        /*!< I3C_T::INTSTSEN: TXTH Position */
#define I3C_INTSTSEN_TXTH_Msk     (0x1UL << I3C_INTSTSEN_TXTH_Pos)            /*!< I3C_T::INTSTSEN: TXTH Mask */

#define I3C_INTSTSEN_RXTH_Pos     (1U)                                        /*!< I3C_T::INTSTSEN: RXTH Position */
#define I3C_INTSTSEN_RXTH_Msk     (0x1UL << I3C_INTSTSEN_RXTH_Pos)            /*!< I3C_T::INTSTSEN: RXTH Mask */

#define I3C_INTSTSEN_IBITH_Pos    (2U)                                        /*!< I3C_T::INTSTSEN: IBITH Position */
#define I3C_INTSTSEN_IBITH_Msk    (0x1UL << I3C_INTSTSEN_IBITH_Pos)           /*!< I3C_T::INTSTSEN: IBITH Mask */

#define I3C_INTSTSEN_CMDRDY_Pos   (3U)                                        /*!< I3C_T::INTSTSEN: CMDRDY Position */
#define I3C_INTSTSEN_CMDRDY_Msk   (0x1UL << I3C_INTSTSEN_CMDRDY_Pos)          /*!< I3C_T::INTSTSEN: CMDRDY Mask */

#define I3C_INTSTSEN_RESPRDY_Pos  (4U)                                        /*!< I3C_T::INTSTSEN: RESPRDY Position */
#define I3C_INTSTSEN_RESPRDY_Msk  (0x1UL << I3C_INTSTSEN_RESPRDY_Pos)         /*!< I3C_T::INTSTSEN: RESPRDY Mask */

#define I3C_INTSTSEN_TFRABORT_Pos (5U)                                        /*!< I3C_T::INTSTSEN: TFRABORT Position */
#define I3C_INTSTSEN_TFRABORT_Msk (0x1UL << I3C_INTSTSEN_TFRABORT_Pos)        /*!< I3C_T::INTSTSEN: TFRABORT Mask */

#define I3C_INTSTSEN_CCCUPD_Pos   (6U)                                        /*!< I3C_T::INTSTSEN: CCCUPD Position */
#define I3C_INTSTSEN_CCCUPD_Msk   (0x1UL << I3C_INTSTSEN_CCCUPD_Pos)          /*!< I3C_T::INTSTSEN: CCCUPD Mask */

#define I3C_INTSTSEN_DAA_Pos      (8U)                                        /*!< I3C_T::INTSTSEN: DAA Position */
#define I3C_INTSTSEN_DAA_Msk      (0x1UL << I3C_INTSTSEN_DAA_Pos)             /*!< I3C_T::INTSTSEN: DAA Mask */

#define I3C_INTSTSEN_TFRERR_Pos   (9U)                                        /*!< I3C_T::INTSTSEN: TFRERR Position */
#define I3C_INTSTSEN_TFRERR_Msk   (0x1UL << I3C_INTSTSEN_TFRERR_Pos)          /*!< I3C_T::INTSTSEN: TFRERR Mask */

#define I3C_INTSTSEN_DEFTGTS_Pos  (10U)                                       /*!< I3C_T::INTSTSEN: DEFTGTS Position */
#define I3C_INTSTSEN_DEFTGTS_Msk  (0x1UL << I3C_INTSTSEN_DEFTGTS_Pos)         /*!< I3C_T::INTSTSEN: DEFTGTS Mask */

#define I3C_INTSTSEN_READREQ_Pos  (11U)                                       /*!< I3C_T::INTSTSEN: READREQ Position */
#define I3C_INTSTSEN_READREQ_Msk  (0x1UL << I3C_INTSTSEN_READREQ_Pos)         /*!< I3C_T::INTSTSEN: READREQ Mask */

#define I3C_INTSTSEN_IBIUPD_Pos   (12U)                                       /*!< I3C_T::INTSTSEN: IBIUPD Position */
#define I3C_INTSTSEN_IBIUPD_Msk   (0x1UL << I3C_INTSTSEN_IBIUPD_Pos)          /*!< I3C_T::INTSTSEN: IBIUPD Mask */

#define I3C_INTSTSEN_BUSOWNER_Pos (13U)                                       /*!< I3C_T::INTSTSEN: BUSOWNER Position */
#define I3C_INTSTSEN_BUSOWNER_Msk (0x1UL << I3C_INTSTSEN_BUSOWNER_Pos)        /*!< I3C_T::INTSTSEN: BUSOWNER Mask */

#define I3C_INTSTSEN_BUSRSTDN_Pos (15U)                                       /*!< I3C_T::INTSTSEN: BUSRSTDN Position */
#define I3C_INTSTSEN_BUSRSTDN_Msk (0x1UL << I3C_INTSTSEN_BUSRSTDN_Pos)        /*!< I3C_T::INTSTSEN: BUSRSTDN Mask */

#define I3C_INTSTSEN_STADET_Pos   (16U)                                       /*!< I3C_T::INTSTSEN: STADET Position */
#define I3C_INTSTSEN_STADET_Msk   (0x1UL << I3C_INTSTSEN_STADET_Pos)          /*!< I3C_T::INTSTSEN: STADET Mask */

#define I3C_INTSTSEN_RSTPTDET_Pos (17U)                                       /*!< I3C_T::INTSTSEN: RSTPTDET Position */
#define I3C_INTSTSEN_RSTPTDET_Msk (0x1UL << I3C_INTSTSEN_RSTPTDET_Pos)        /*!< I3C_T::INTSTSEN: RSTPTDET Mask */

#define I3C_INTSTSEN_GRPADDRA_Pos (18U)                                       /*!< I3C_T::INTSTSEN: GRPADDRA Position */
#define I3C_INTSTSEN_GRPADDRA_Msk (0x1UL << I3C_INTSTSEN_GRPADDRA_Pos)        /*!< I3C_T::INTSTSEN: GRPADDRA Mask */

#define I3C_INTSTSEN_SDARES_Pos   (19U)                                       /*!< I3C_T::INTSTSEN: SDARES Position */
#define I3C_INTSTSEN_SDARES_Msk   (0x1UL << I3C_INTSTSEN_SDARES_Pos)          /*!< I3C_T::INTSTSEN: SDARES Mask */

#define I3C_INTSTSEN_EXTFINS_Pos  (20U)                                       /*!< I3C_T::INTSTSEN: EXTFINS Position */
#define I3C_INTSTSEN_EXTFINS_Msk  (0x1UL << I3C_INTSTSEN_EXTFINS_Pos)         /*!< I3C_T::INTSTSEN: EXTFINS Mask */

#define I3C_INTSTSEN_EXTTXTH_Pos  (21U)                                       /*!< I3C_T::INTSTSEN: EXTTXTH Position */
#define I3C_INTSTSEN_EXTTXTH_Msk  (0x1UL << I3C_INTSTSEN_EXTTXTH_Pos)         /*!< I3C_T::INTSTSEN: EXTTXTH Mask */

#define I3C_INTEN_TXTH_Pos        (0U)                                        /*!< I3C_T::INTEN: TXTH Position */
#define I3C_INTEN_TXTH_Msk        (0x1UL << I3C_INTEN_TXTH_Pos)               /*!< I3C_T::INTEN: TXTH Mask */

#define I3C_INTEN_RXTH_Pos        (1U)                                        /*!< I3C_T::INTEN: RXTH Position */
#define I3C_INTEN_RXTH_Msk        (0x1UL << I3C_INTEN_RXTH_Pos)               /*!< I3C_T::INTEN: RXTH Mask */

#define I3C_INTEN_IBITH_Pos       (2U)                                        /*!< I3C_T::INTEN: IBITH Position */
#define I3C_INTEN_IBITH_Msk       (0x1UL << I3C_INTEN_IBITH_Pos)              /*!< I3C_T::INTEN: IBITH Mask */

#define I3C_INTEN_CMDRDY_Pos      (3U)                                        /*!< I3C_T::INTEN: CMDRDY Position */
#define I3C_INTEN_CMDRDY_Msk      (0x1UL << I3C_INTEN_CMDRDY_Pos)             /*!< I3C_T::INTEN: CMDRDY Mask */

#define I3C_INTEN_RESPRDY_Pos     (4U)                                        /*!< I3C_T::INTEN: RESPRDY Position */
#define I3C_INTEN_RESPRDY_Msk     (0x1UL << I3C_INTEN_RESPRDY_Pos)            /*!< I3C_T::INTEN: RESPRDY Mask */

#define I3C_INTEN_TFRABORT_Pos    (5U)                                        /*!< I3C_T::INTEN: TFRABORT Position */
#define I3C_INTEN_TFRABORT_Msk    (0x1UL << I3C_INTEN_TFRABORT_Pos)           /*!< I3C_T::INTEN: TFRABORT Mask */

#define I3C_INTEN_CCCUPD_Pos      (6U)                                        /*!< I3C_T::INTEN: CCCUPD Position */
#define I3C_INTEN_CCCUPD_Msk      (0x1UL << I3C_INTEN_CCCUPD_Pos)             /*!< I3C_T::INTEN: CCCUPD Mask */

#define I3C_INTEN_DAA_Pos         (8U)                                        /*!< I3C_T::INTEN: DAA Position */
#define I3C_INTEN_DAA_Msk         (0x1UL << I3C_INTEN_DAA_Pos)                /*!< I3C_T::INTEN: DAA Mask */

#define I3C_INTEN_TFRERR_Pos      (9U)                                        /*!< I3C_T::INTEN: TFRERR Position */
#define I3C_INTEN_TFRERR_Msk      (0x1UL << I3C_INTEN_TFRERR_Pos)             /*!< I3C_T::INTEN: TFRERR Mask */

#define I3C_INTEN_DEFTGTS_Pos     (10U)                                       /*!< I3C_T::INTEN: DEFTGTS Position */
#define I3C_INTEN_DEFTGTS_Msk     (0x1UL << I3C_INTEN_DEFTGTS_Pos)            /*!< I3C_T::INTEN: DEFTGTS Mask */

#define I3C_INTEN_READREQ_Pos     (11U)                                       /*!< I3C_T::INTEN: READREQ Position */
#define I3C_INTEN_READREQ_Msk     (0x1UL << I3C_INTEN_READREQ_Pos)            /*!< I3C_T::INTEN: READREQ Mask */

#define I3C_INTEN_IBIUPD_Pos      (12U)                                       /*!< I3C_T::INTEN: IBIUPD Position */
#define I3C_INTEN_IBIUPD_Msk      (0x1UL << I3C_INTEN_IBIUPD_Pos)             /*!< I3C_T::INTEN: IBIUPD Mask */

#define I3C_INTEN_BUSOWNER_Pos    (13U)                                       /*!< I3C_T::INTEN: BUSOWNER Position */
#define I3C_INTEN_BUSOWNER_Msk    (0x1UL << I3C_INTEN_BUSOWNER_Pos)           /*!< I3C_T::INTEN: BUSOWNER Mask */

#define I3C_INTEN_BUSRSTDN_Pos    (15U)                                       /*!< I3C_T::INTEN: BUSRSTDN Position */
#define I3C_INTEN_BUSRSTDN_Msk    (0x1UL << I3C_INTEN_BUSRSTDN_Pos)           /*!< I3C_T::INTEN: BUSRSTDN Mask */

#define I3C_INTEN_STADET_Pos      (16U)                                       /*!< I3C_T::INTEN: STADET Position */
#define I3C_INTEN_STADET_Msk      (0x1UL << I3C_INTEN_STADET_Pos)             /*!< I3C_T::INTEN: STADET Mask */

#define I3C_INTEN_RSTPTDET_Pos    (17U)                                       /*!< I3C_T::INTEN: RSTPTDET Position */
#define I3C_INTEN_RSTPTDET_Msk    (0x1UL << I3C_INTEN_RSTPTDET_Pos)           /*!< I3C_T::INTEN: RSTPTDET Mask */

#define I3C_INTEN_GRPADDRA_Pos    (18U)                                       /*!< I3C_T::INTEN: GRPADDRA Position */
#define I3C_INTEN_GRPADDRA_Msk    (0x1UL << I3C_INTEN_GRPADDRA_Pos)           /*!< I3C_T::INTEN: GRPADDRA Mask */

#define I3C_INTEN_SDARES_Pos      (19U)                                       /*!< I3C_T::INTEN: SDARES Position */
#define I3C_INTEN_SDARES_Msk      (0x1UL << I3C_INTEN_SDARES_Pos)             /*!< I3C_T::INTEN: SDARES Mask */

#define I3C_INTEN_EXTFINS_Pos     (20U)                                       /*!< I3C_T::INTEN: EXTFINS Position */
#define I3C_INTEN_EXTFINS_Msk     (0x1UL << I3C_INTEN_EXTFINS_Pos)            /*!< I3C_T::INTEN: EXTFINS Mask */

#define I3C_INTEN_EXTTXTH_Pos     (21U)                                       /*!< I3C_T::INTEN: EXTTXTH Position */
#define I3C_INTEN_EXTTXTH_Msk     (0x1UL << I3C_INTEN_EXTTXTH_Pos)            /*!< I3C_T::INTEN: EXTTXTH Mask */

#define I3C_QUESTSLV_CMDELOC_Pos  (0U)                                        /*!< I3C_T::QUESTSLV: CMDELOC Position */
#define I3C_QUESTSLV_CMDELOC_Msk  (0xFFUL << I3C_QUESTSLV_CMDELOC_Pos)        /*!< I3C_T::QUESTSLV: CMDELOC Mask */

#define I3C_QUESTSLV_RESPLV_Pos   (8U)                                        /*!< I3C_T::QUESTSLV: RESPLV Position */
#define I3C_QUESTSLV_RESPLV_Msk   (0xFFUL << I3C_QUESTSLV_RESPLV_Pos)         /*!< I3C_T::QUESTSLV: RESPLV Mask */

#define I3C_QUESTSLV_IBIBUFLV_Pos (16U)                                       /*!< I3C_T::QUESTSLV: IBIBUFLV Position */
#define I3C_QUESTSLV_IBIBUFLV_Msk (0xFFUL << I3C_QUESTSLV_IBIBUFLV_Pos)       /*!< I3C_T::QUESTSLV: IBIBUFLV Mask */

#define I3C_QUESTSLV_IBISCNT_Pos  (24U)                                       /*!< I3C_T::QUESTSLV: IBISCNT Position */
#define I3C_QUESTSLV_IBISCNT_Msk  (0x1FUL << I3C_QUESTSLV_IBISCNT_Pos)        /*!< I3C_T::QUESTSLV: IBISCNT Mask */

#define I3C_DBSTSLV_TXELV_Pos     (0U)                                        /*!< I3C_T::DBSTSLV: TXELV Position */
#define I3C_DBSTSLV_TXELV_Msk     (0xFFUL << I3C_DBSTSLV_TXELV_Pos)           /*!< I3C_T::DBSTSLV: TXELV Mask */

#define I3C_DBSTSLV_RXLV_Pos      (16U)                                       /*!< I3C_T::DBSTSLV: RXLV Position */
#define I3C_DBSTSLV_RXLV_Msk      (0xFFUL << I3C_DBSTSLV_RXLV_Pos)            /*!< I3C_T::DBSTSLV: RXLV Mask */

#define I3C_PRESENTS_CTRACTS_Pos  (2U)                                        /*!< I3C_T::PRESENTS: CTRACTS Position */
#define I3C_PRESENTS_CTRACTS_Msk  (0x1UL << I3C_PRESENTS_CTRACTS_Pos)         /*!< I3C_T::PRESENTS: CTRACTS Mask */

#define I3C_PRESENTS_TFRTYPE_Pos  (8U)                                        /*!< I3C_T::PRESENTS: TFRTYPE Position */
#define I3C_PRESENTS_TFRTYPE_Msk  (0x3FUL << I3C_PRESENTS_TFRTYPE_Pos)        /*!< I3C_T::PRESENTS: TFRTYPE Mask */

#define I3C_PRESENTS_CTRTFRS_Pos  (16U)                                       /*!< I3C_T::PRESENTS: CTRTFRS Position */
#define I3C_PRESENTS_CTRTFRS_Msk  (0x3FUL << I3C_PRESENTS_CTRTFRS_Pos)        /*!< I3C_T::PRESENTS: CTRTFRS Mask */

#define I3C_PRESENTS_TID_Pos      (24U)                                       /*!< I3C_T::PRESENTS: TID Position */
#define I3C_PRESENTS_TID_Msk      (0xFUL << I3C_PRESENTS_TID_Pos)             /*!< I3C_T::PRESENTS: TID Mask */

#define I3C_PRESENTS_CTRIDLES_Pos (28U)                                       /*!< I3C_T::PRESENTS: CTRIDLES Position */
#define I3C_PRESENTS_CTRIDLES_Msk (0x1UL << I3C_PRESENTS_CTRIDLES_Pos)        /*!< I3C_T::PRESENTS: CTRIDLES Mask */

#define I3C_CCCDEVS_PENDINT_Pos   (0U)                                        /*!< I3C_T::CCCDEVS: PENDINT Position */
#define I3C_CCCDEVS_PENDINT_Msk   (0xFUL << I3C_CCCDEVS_PENDINT_Pos)          /*!< I3C_T::CCCDEVS: PENDINT Mask */

#define I3C_CCCDEVS_PROTERR_Pos   (5U)                                        /*!< I3C_T::CCCDEVS: PROTERR Position */
#define I3C_CCCDEVS_PROTERR_Msk   (0x1UL << I3C_CCCDEVS_PROTERR_Pos)          /*!< I3C_T::CCCDEVS: PROTERR Mask */

#define I3C_CCCDEVS_ACTMODE_Pos   (6U)                                        /*!< I3C_T::CCCDEVS: ACTMODE Position */
#define I3C_CCCDEVS_ACTMODE_Msk   (0x3UL << I3C_CCCDEVS_ACTMODE_Pos)          /*!< I3C_T::CCCDEVS: ACTMODE Mask */

#define I3C_CCCDEVS_UDFERR_Pos    (8U)                                        /*!< I3C_T::CCCDEVS: UDFERR Position */
#define I3C_CCCDEVS_UDFERR_Msk    (0x1UL << I3C_CCCDEVS_UDFERR_Pos)           /*!< I3C_T::CCCDEVS: UDFERR Mask */

#define I3C_CCCDEVS_SLVBUSY_Pos   (9U)                                        /*!< I3C_T::CCCDEVS: SLVBUSY Position */
#define I3C_CCCDEVS_SLVBUSY_Msk   (0x1UL << I3C_CCCDEVS_SLVBUSY_Pos)          /*!< I3C_T::CCCDEVS: SLVBUSY Mask */

#define I3C_CCCDEVS_OVFERR_Pos    (10U)                                       /*!< I3C_T::CCCDEVS: OVFERR Position */
#define I3C_CCCDEVS_OVFERR_Msk    (0x1UL << I3C_CCCDEVS_OVFERR_Pos)           /*!< I3C_T::CCCDEVS: OVFERR Mask */

#define I3C_CCCDEVS_DATNRDY_Pos   (11U)                                       /*!< I3C_T::CCCDEVS: DATNRDY Position */
#define I3C_CCCDEVS_DATNRDY_Msk   (0x1UL << I3C_CCCDEVS_DATNRDY_Pos)          /*!< I3C_T::CCCDEVS: DATNRDY Mask */

#define I3C_CCCDEVS_BFNAVAIL_Pos  (12U)                                       /*!< I3C_T::CCCDEVS: BFNAVAIL Position */
#define I3C_CCCDEVS_BFNAVAIL_Msk  (0x1UL << I3C_CCCDEVS_BFNAVAIL_Pos)         /*!< I3C_T::CCCDEVS: BFNAVAIL Mask */

#define I3C_CCCDEVS_FRAMEERR_Pos  (13U)                                       /*!< I3C_T::CCCDEVS: FRAMEERR Position */
#define I3C_CCCDEVS_FRAMEERR_Msk  (0x1UL << I3C_CCCDEVS_FRAMEERR_Pos)         /*!< I3C_T::CCCDEVS: FRAMEERR Mask */

#define I3C_CHRTBP_CHRINDEX_Pos   (19U)                                       /*!< I3C_T::CHRTBP: CHRINDEX Position */
#define I3C_CHRTBP_CHRINDEX_Msk   (0x1FFFUL << I3C_CHRTBP_CHRINDEX_Pos)       /*!< I3C_T::CHRTBP: CHRINDEX Mask */

#define I3C_SLVMID_PIDTYPE_Pos    (0U)                                        /*!< I3C_T::SLVMID: PIDTYPE Position */
#define I3C_SLVMID_PIDTYPE_Msk    (0x1UL << I3C_SLVMID_PIDTYPE_Pos)           /*!< I3C_T::SLVMID: PIDTYPE Mask */

#define I3C_SLVMID_MID_Pos        (1U)                                        /*!< I3C_T::SLVMID: MID Position */
#define I3C_SLVMID_MID_Msk        (0x7FFFUL << I3C_SLVMID_MID_Pos)            /*!< I3C_T::SLVMID: MID Mask */

#define I3C_SLVPID_ADDLMEAN_Pos   (0U)                                        /*!< I3C_T::SLVPID: ADDLMEAN Position */
#define I3C_SLVPID_ADDLMEAN_Msk   (0xFFFUL << I3C_SLVPID_ADDLMEAN_Pos)        /*!< I3C_T::SLVPID: ADDLMEAN Mask */

#define I3C_SVLPID_INSTID_Pos     (12U)                                       /*!< I3C_T::SLVPID: INSTID Position */
#define I3C_SVLPID_INSTID_Msk     (0xFUL << I3C_SVLPID_INSTID_Pos)            /*!< I3C_T::SLVPID: INSTID Mask */

#define I3C_SLVPID_PARTID_Pos     (16U)                                       /*!< I3C_T::SLVPID: PARTID Position */
#define I3C_SLVPID_PARTID_Msk     (0xFFFFUL << I3C_SLVPID_PARTID_Pos)         /*!< I3C_T::SLVPID: PARTID Mask */

#define I3C_SLVCHAR_MXDSLIMT_Pos  (0U)                                        /*!< I3C_T::SLVCHAR: MXDSLIMT Position */
#define I3C_SLVCHAR_MXDSLIMT_Msk  (0x1UL << I3C_SLVCHAR_MXDSLIMT_Pos)         /*!< I3C_T::SLVCHAR: MXDSLIMT Mask */

#define I3C_SLVCHAR_IBICAP_Pos    (1U)                                        /*!< I3C_T::SLVCHAR: IBICAP Position */
#define I3C_SLVCHAR_IBICAP_Msk    (0x1UL << I3C_SLVCHAR_IBICAP_Pos)           /*!< I3C_T::SLVCHAR: IBICAP Mask */

#define I3C_SLVCHAR_IBIPL_Pos     (2U)                                        /*!< I3C_T::SLVCHAR: IBIPL Position */
#define I3C_SLVCHAR_IBIPL_Msk     (0x1UL << I3C_SLVCHAR_IBIPL_Pos)            /*!< I3C_T::SLVCHAR: IBIPL Mask */

#define I3C_SLVCHAR_OLINECAP_Pos  (3U)                                        /*!< I3C_T::SLVCHAR: OLINECAP Position */
#define I3C_SLVCHAR_OLINECAP_Msk  (0x1UL << I3C_SLVCHAR_OLINECAP_Pos)         /*!< I3C_T::SLVCHAR: OLINECAP Mask */

#define I3C_SLVCHAR_BRIDGEID_Pos  (4U)                                        /*!< I3C_T::SLVCHAR: BRIDGEID Position */
#define I3C_SLVCHAR_BRIDGEID_Msk  (0x1UL << I3C_SLVCHAR_BRIDGEID_Pos)         /*!< I3C_T::SLVCHAR: BRIDGEID Mask */

#define I3C_SLVCHAR_HDRCAP_Pos    (5U)                                        /*!< I3C_T::SLVCHAR: HDRCAP Position */
#define I3C_SLVCHAR_HDRCAP_Msk    (0x1UL << I3C_SLVCHAR_HDRCAP_Pos)           /*!< I3C_T::SLVCHAR: HDRCAP Mask */

#define I3C_SLVCHAR_DEVROLE_Pos   (6U)                                        /*!< I3C_T::SLVCHAR: DEVROLE Position */
#define I3C_SLVCHAR_DEVROLE_Msk   (0x3UL << I3C_SLVCHAR_DEVROLE_Pos)          /*!< I3C_T::SLVCHAR: DEVROLE Mask */

#define I3C_SLVCHAR_DCR_Pos       (8U)                                        /*!< I3C_T::SLVCHAR: DCR Position */
#define I3C_SLVCHAR_DCR_Msk       (0xFFUL << I3C_SLVCHAR_DCR_Pos)             /*!< I3C_T::SLVCHAR: DCR Mask */

#define I3C_SLVCHAR_HDRCAPV_Pos   (16U)                                       /*!< I3C_T::SLVCHAR: HDRCAPV Position */
#define I3C_SLVCHAR_HDRCAPV_Msk   (0xFFUL << I3C_SLVCHAR_HDRCAPV_Pos)         /*!< I3C_T::SLVCHAR: HDRCAPV Mask */

#define I3C_SLVMXLEN_MWL_Pos      (0U)                                        /*!< I3C_T::SLVMXLEN: MWL Position */
#define I3C_SLVMXLEN_MWL_Msk      (0xFFFFUL << I3C_SLVMXLEN_MWL_Pos)          /*!< I3C_T::SLVMXLEN: MWL Mask */

#define I3C_SLVMXLEN_MRL_Pos      (16U)                                       /*!< I3C_T::SLVMXLEN: MRL Position */
#define I3C_SLVMXLEN_MRL_Msk      (0xFFFFUL << I3C_SLVMXLEN_MRL_Pos)          /*!< I3C_T::SLVMXLEN: MRL Mask */

#define I3C_MXDS_MXWR_Pos         (0U)                                        /*!< I3C_T::MXDS: MXWR Position */
#define I3C_MXDS_MXWR_Msk         (0x7UL << I3C_MXDS_MXWR_Pos)                /*!< I3C_T::MXDS: MXWR Mask */

#define I3C_MXDS_DEFBYTE_Pos      (3U)                                        /*!< I3C_T::MXDS: DEFBYTE Position */
#define I3C_MXDS_DEFBYTE_Msk      (0x1UL << I3C_MXDS_DEFBYTE_Pos)             /*!< I3C_T::MXDS: DEFBYTE Mask */

#define I3C_MXDS_MXRD_Pos         (8U)                                        /*!< I3C_T::MXDS: MXRD Position */
#define I3C_MXDS_MXRD_Msk         (0x7UL << I3C_MXDS_MXRD_Pos)                /*!< I3C_T::MXDS: MXRD Mask */

#define I3C_MXDS_STPPERM_Pos      (19U)                                       /*!< I3C_T::MXDS: STPPERM Position */
#define I3C_MXDS_STPPERM_Msk      (0x1UL << I3C_MXDS_STPPERM_Pos)             /*!< I3C_T::MXDS: STPPERM Mask */

#define I3C_MXDS_ACTST_Pos        (24U)                                       /*!< I3C_T::MXDS: ACTST Position */
#define I3C_MXDS_ACTST_Msk        (0x3UL << I3C_MXDS_ACTST_Pos)               /*!< I3C_T::MXDS: ACTST Mask */

#define I3C_MXDS_SETACTST_Pos     (26U)                                       /*!< I3C_T::MXDS: SETACTST Position */
#define I3C_MXDS_SETACTST_Msk     (0x1UL << I3C_MXDS_SETACTST_Pos)            /*!< I3C_T::MXDS: SETACTST Mask */

#define I3C_SIR_EN_Pos            (0U)                                        /*!< I3C_T::SIR: EN Position */
#define I3C_SIR_EN_Msk            (0x1UL << I3C_SIR_EN_Pos)                   /*!< I3C_T::SIR: EN Mask */

#define I3C_SIR_CTL_Pos           (1U)                                        /*!< I3C_T::SIR: CTL Position */
#define I3C_SIR_CTL_Msk           (0x3UL << I3C_SIR_CTL_Pos)                  /*!< I3C_T::SIR: CTL Mask */

#define I3C_SIR_MR_Pos            (3U)                                        /*!< I3C_T::SIR: MR Position */
#define I3C_SIR_MR_Msk            (0x1UL << I3C_SIR_MR_Pos)                   /*!< I3C_T::SIR: MR Mask */

#define I3C_SIR_TS_Pos            (4U)                                        /*!< I3C_T::SIR: TS Position */
#define I3C_SIR_TS_Msk            (0x1UL << I3C_SIR_TS_Pos)                   /*!< I3C_T::SIR: TS Mask */

#define I3C_SIR_CE3REC_Pos        (7U)                                        /*!< I3C_T::SIR: CE3REC Position */
#define I3C_SIR_CE3REC_Msk        (0x1UL << I3C_SIR_CE3REC_Pos)               /*!< I3C_T::SIR: CE3REC Mask */

#define I3C_SIR_MDB_Pos           (8U)                                        /*!< I3C_T::SIR: MDB Position */
#define I3C_SIR_MDB_Msk           (0xFFUL << I3C_SIR_MDB_Pos)                 /*!< I3C_T::SIR: MDB Mask */

#define I3C_SIR_DATLEN_Pos        (16U)                                       /*!< I3C_T::SIR: DATLEN Position */
#define I3C_SIR_DATLEN_Msk        (0xFFUL << I3C_SIR_DATLEN_Pos)              /*!< I3C_T::SIR: DATLEN Mask */

#define I3C_SIR_TGTIDX_Pos        (28U)                                       /*!< I3C_T::SIR: TGTIDX Position */
#define I3C_SIR_TGTIDX_Msk        (0xFUL << I3C_SIR_TGTIDX_Pos)               /*!< I3C_T::SIR: TGTIDX Mask */

#define I3C_SIRDAT_DAT0_Pos       (0U)                                        /*!< I3C_T::SIRDAT: DAT0 Position */
#define I3C_SIRDAT_DAT0_Msk       (0xFFUL << I3C_SIRDAT_DAT0_Pos)             /*!< I3C_T::SIRDAT: DAT0 Mask */

#define I3C_SIRDAT_DAT1_Pos       (8U)                                        /*!< I3C_T::SIRDAT: DAT1 Position */
#define I3C_SIRDAT_DAT1_Msk       (0xFFUL << I3C_SIRDAT_DAT1_Pos)             /*!< I3C_T::SIRDAT: DAT1 Mask */

#define I3C_SIRDAT_DAT2_Pos       (16U)                                       /*!< I3C_T::SIRDAT: DAT2 Position */
#define I3C_SIRDAT_DAT2_Msk       (0xFFUL << I3C_SIRDAT_DAT2_Pos)             /*!< I3C_T::SIRDAT: DAT2 Mask */

#define I3C_SIRDAT_DAT3_Pos       (24U)                                       /*!< I3C_T::SIRDAT: DAT3 Position */
#define I3C_SIRDAT_DAT3_Msk       (0xFFUL << I3C_SIRDAT_DAT3_Pos)             /*!< I3C_T::SIRDAT: DAT3 Mask */

#define I3C_SIRRESP_IBISTS_Pos    (0U)                                        /*!< I3C_T::SIRRESP: IBISTS Position */
#define I3C_SIRRESP_IBISTS_Msk    (0x7UL << I3C_SIRRESP_IBISTS_Pos)           /*!< I3C_T::SIRRESP: IBISTS Mask */

#define I3C_SIRRESP_DATLEN_Pos    (8U)                                        /*!< I3C_T::SIRRESP: DATLEN Position */
#define I3C_SIRRESP_DATLEN_Msk    (0xFFFFUL << I3C_SIRRESP_DATLEN_Pos)        /*!< I3C_T::SIRRESP: DATLEN Mask */

#define I3C_INSTSTS_INTERSTS_Pos  (0U)                                        /*!< I3C_T::INSTSTS: INTERSTS Position */
#define I3C_INSTSTS_INTERSTS_Msk  (0x3FUL << I3C_INSTSTS_INTERSTS_Pos)        /*!< I3C_T::INSTSTS: INTERSTS Mask */

#define I3C_INSTSTS_CCCING_Pos    (6U)                                        /*!< I3C_T::INSTSTS: CCCING Position */
#define I3C_INSTSTS_CCCING_Msk    (0x1UL << I3C_INSTSTS_CCCING_Pos)           /*!< I3C_T::INSTSTS: CCCING Mask */

#define I3C_INSTSTS_WRING_Pos     (7U)                                        /*!< I3C_T::INSTSTS: WRING Position */
#define I3C_INSTSTS_WRING_Msk     (0x1UL << I3C_INSTSTS_WRING_Pos)            /*!< I3C_T::INSTSTS: WRING Mask */

#define I3C_INSTSTS_WAKEUP_Pos    (8U)                                        /*!< I3C_T::INSTSTS: WAKEUP Position */
#define I3C_INSTSTS_WAKEUP_Msk    (0x1UL << I3C_INSTSTS_WAKEUP_Pos)           /*!< I3C_T::INSTSTS: WAKEUP Mask */

#define I3C_DEVCTLE_OPERMODE_Pos  (0U)                                        /*!< I3C_T::DEVCTLE: OPERMODE Position */
#define I3C_DEVCTLE_OPERMODE_Msk  (0x3UL << I3C_DEVCTLE_OPERMODE_Pos)         /*!< I3C_T::DEVCTLE: OPERMODE Mask */

#define I3C_DEVCTLE_MRACKCTL_Pos  (3U)                                        /*!< I3C_T::DEVCTLE: MRACKCTL Position */
#define I3C_DEVCTLE_MRACKCTL_Msk  (0x1UL << I3C_DEVCTLE_MRACKCTL_Pos)         /*!< I3C_T::DEVCTLE: MRACKCTL Mask */

#define I3C_SCLOD_ODLCNT_Pos      (0U)                                        /*!< I3C_T::SCLOD: ODLCNT Position */
#define I3C_SCLOD_ODLCNT_Msk      (0xFFUL << I3C_SCLOD_ODLCNT_Pos)            /*!< I3C_T::SCLOD: ODLCNT Mask */

#define I3C_SCLOD_ODHCNT_Pos      (16U)                                       /*!< I3C_T::SCLOD: ODHCNT Position */
#define I3C_SCLOD_ODHCNT_Msk      (0xFFUL << I3C_SCLOD_ODHCNT_Pos)            /*!< I3C_T::SCLOD: ODHCNT Mask */

#define I3C_SCLPP_PPLCNT_Pos      (0U)                                        /*!< I3C_T::SCLPP: PPLCNT Position */
#define I3C_SCLPP_PPLCNT_Msk      (0xFFUL << I3C_SCLPP_PPLCNT_Pos)            /*!< I3C_T::SCLPP: PPLCNT Mask */

#define I3C_SCLPP_PPHCNT_Pos      (16U)                                       /*!< I3C_T::SCLPP: PPHCNT Position */
#define I3C_SCLPP_PPHCNT_Msk      (0xFFUL << I3C_SCLPP_PPHCNT_Pos)            /*!< I3C_T::SCLPP: PPHCNT Mask */

#define I3C_SCLFM_FMLCNT_Pos      (0U)                                        /*!< I3C_T::SCLFM: FMLCNT Position */
#define I3C_SCLFM_FMLCNT_Msk      (0xFFFFUL << I3C_SCLFM_FMLCNT_Pos)          /*!< I3C_T::SCLFM: FMLCNT Mask */

#define I3C_SCLFM_FMHCNT_Pos      (16U)                                       /*!< I3C_T::SCLFM: FMHCNT Position */
#define I3C_SCLFM_FMHCNT_Msk      (0xFFFFUL << I3C_SCLFM_FMHCNT_Pos)          /*!< I3C_T::SCLFM: FMHCNT Mask */

#define I3C_SCLFMP_FMPLCNT_Pos    (0U)                                        /*!< I3C_T::SCLFMP: FMPLCNT Position */
#define I3C_SCLFMP_FMPLCNT_Msk    (0xFFFFUL << I3C_SCLFMP_FMPLCNT_Pos)        /*!< I3C_T::SCLFMP: FMPLCNT Mask */

#define I3C_SCLFMP_FMPHCNT_Pos    (16U)                                       /*!< I3C_T::SCLFMP: FMPHCNT Position */
#define I3C_SCLFMP_FMPHCNT_Msk    (0xFFUL << I3C_SCLFMP_FMPHCNT_Pos)          /*!< I3C_T::SCLFMP: FMPHCNT Mask */

#define I3C_SCLEXTLO_EXTLCNT1_Pos (0U)                                        /*!< I3C_T::SCLEXTLO: EXTLCNT1 Position */
#define I3C_SCLEXTLO_EXTLCNT1_Msk (0xFFUL << I3C_SCLEXTLO_EXTLCNT1_Pos)       /*!< I3C_T::SCLEXTLO: EXTLCNT1 Mask */

#define I3C_SCLEXTLO_EXTLCNT2_Pos (8U)                                        /*!< I3C_T::SCLEXTLO: EXTLCNT2 Position */
#define I3C_SCLEXTLO_EXTLCNT2_Msk (0xFFUL << I3C_SCLEXTLO_EXTLCNT2_Pos)       /*!< I3C_T::SCLEXTLO: EXTLCNT2 Mask */

#define I3C_SCLEXTLO_EXTLCNT3_Pos (16U)                                       /*!< I3C_T::SCLEXTLO: EXTLCNT3 Position */
#define I3C_SCLEXTLO_EXTLCNT3_Msk (0xFFUL << I3C_SCLEXTLO_EXTLCNT3_Pos)       /*!< I3C_T::SCLEXTLO: EXTLCNT3 Mask */

#define I3C_SCLEXTLO_EXTLCNT4_Pos (24U)                                       /*!< I3C_T::SCLEXTLO: EXTLCNT4 Position */
#define I3C_SCLEXTLO_EXTLCNT4_Msk (0xFFUL << I3C_SCLEXTLO_EXTLCNT4_Pos)       /*!< I3C_T::SCLEXTLO: EXTLCNT4 Mask */

#define I3C_SCLEXTTB_TERMCNT_Pos  (0U)                                        /*!< I3C_T::SCLEXTTB: TERMCNT Position */
#define I3C_SCLEXTTB_TERMCNTM_Msk (0xFUL << I3C_SCLEXTTB_TERMCNT_Pos)         /*!< I3C_T::SCLEXTTB: TERMCNT Mask */

#define I3C_SCLEXTTB_STPCNT_Pos   (28U)                                       /*!< I3C_T::SCLEXTTB: STPCNT Position */
#define I3C_SCLEXTTB_STPCNT_Msk   (0xFUL << I3C_SCLEXTTB_STPCNT_Pos)          /*!< I3C_T::SCLEXTTB: STPCNT Mask */

#define I3C_SDAHOLD_SDAOPDLY_Pos  (0U)                                        /*!< I3C_T::SDAHOLD: SDAOPDLY Position */
#define I3C_SDAHOLD_SDAOPDLY_Msk  (0x7UL << I3C_SDAHOLD_SDAOPDLY_Pos)         /*!< I3C_T::SDAHOLD: SDAOPDLY Mask */

#define I3C_SDAHOLD_SDAPODLY_Pos  (8U)                                        /*!< I3C_T::SDAHOLD: SDAPODLY Position */
#define I3C_SDAHOLD_SDAPODLY_Msk  (0x7UL << I3C_SDAHOLD_SDAPODLY_Pos)         /*!< I3C_T::SDAHOLD: SDAPODLY Mask */

#define I3C_SDAHOLD_TXHOLD_Pos    (16U)                                       /*!< I3C_T::SDAHOLD: TXHOLD Position */
#define I3C_SDAHOLD_TXHOLD_Msk    (0x7UL << I3C_SDAHOLD_TXHOLD_Pos)           /*!< I3C_T::SDAHOLD: TXHOLD Mask */

#define I3C_BUSFAT_FREETC_Pos     (0U)                                        /*!< I3C_T::BUSFAT: FREETC Position */
#define I3C_BUSFAT_FREETC_Msk     (0xFFFFUL << I3C_BUSFAT_FREETC_Pos)         /*!< I3C_T::BUSFAT: FREETC Mask */

#define I3C_BUSFAT_AVAILTC_Pos    (16U)                                       /*!< I3C_T::BUSFAT: AVAILTC Position */
#define I3C_BUSFAT_AVAILTC_Msk    (0xFFFFUL << I3C_BUSFAT_AVAILTC_Pos)        /*!< I3C_T::BUSFAT: AVAILTC Mask */

#define I3C_BUSIDLET_IDLETC_Pos   (0U)                                        /*!< I3C_T::BUSIDLET: IDLETC Position */
#define I3C_BUSIDLET_IDLETC_Msk   (0xFFFFFUL << I3C_BUSIDLET_IDLETC_Pos)      /*!< I3C_T::BUSIDLET: IDLETC Mask */

#define I3C_SCLLOWTO_LOWCNT_Pos   (0U)                                        /*!< I3C_T::SCLLOWTO: LOWCNT Position */
#define I3C_SCLLOWTO_LOWCNT_Msk   (0x3FFFFFFUL << I3C_SCLLOWTO_LOWCNT_Pos)    /*!< I3C_T::SCLLOWTO: LOWCNT Mask */

#define I3C_RELSDA_RELSDATC_Pos   (0U)                                        /*!< I3C_T::RELSDA: RELSDATC Position */
#define I3C_RELSDA_RELSDATC_Msk   (0xFFFFFUL << I3C_RELSDA_RELSDATC_Pos)      /*!< I3C_T::RELSDA: RELSDATC Mask */

#define I3C_BTDELY_DLYBCNT_Pos    (0U)                                        /*!< I3C_T::BTDELY: DLYBCNT Position */
#define I3C_BTDELY_DLYBCNT_Msk    (0x7FFUL << I3C_BTDELY_DLYBCNT_Pos)         /*!< I3C_T::BTDELY: DLYBCNT Mask */

#define I3C_EXTDAT_EXTDAT_Pos     (0U)                                        /*!< I3C_T::EXTDAT: EXTDAT Position */
#define I3C_EXTDAT_EXTDAT_Msk     (0xFFFFFFFFUL << I3C_EXTDAT_EXTDAT_Pos)     /*!< I3C_T::EXTDAT: EXTDAT Mask */

#define I3C_EXTTXTHS_ECMDTTS0_Pos (0U)                                        /*!< I3C_T::EXTTXTHS: ECMDTTS0 Position */
#define I3C_EXTTXTHS_ECMDTTS0_Msk (0x1UL << I3C_EXTTXTHS_ECMDTTS0_Pos)        /*!< I3C_T::EXTTXTHS: ECMDTTS0 Mask */

#define I3C_EXTTXTHS_ECMDTTS1_Pos (1U)                                        /*!< I3C_T::EXTTXTHS: ECMDTTS1 Position */
#define I3C_EXTTXTHS_ECMDTTS1_Msk (0x1UL << I3C_EXTTXTHS_ECMDTTS1_Pos)        /*!< I3C_T::EXTTXTHS: ECMDTTS1 Mask */

#define I3C_EXTTXTHS_ECMDTTS2_Pos (2U)                                        /*!< I3C_T::EXTTXTHS: ECMDTTS2 Position */
#define I3C_EXTTXTHS_ECMDTTS2_Msk (0x1UL << I3C_EXTTXTHS_ECMDTTS2_Pos)        /*!< I3C_T::EXTTXTHS: ECMDTTS2 Mask */

#define I3C_EXTTXTHS_ECMDTTS3_Pos (3U)                                        /*!< I3C_T::EXTTXTHS: ECMDTTS3 Position */
#define I3C_EXTTXTHS_ECMDTTS3_Msk (0x1UL << I3C_EXTTXTHS_ECMDTTS3_Pos)        /*!< I3C_T::EXTTXTHS: ECMDTTS3 Mask */

#define I3C_EXTTXTHS_ECMDTTS4_Pos (4U)                                        /*!< I3C_T::EXTTXTHS: ECMDTTS4 Position */
#define I3C_EXTTXTHS_ECMDTTS4_Msk (0x1UL << I3C_EXTTXTHS_ECMDTTS4_Pos)        /*!< I3C_T::EXTTXTHS: ECMDTTS4 Mask */

#define I3C_EXTTXTHS_ECMDTTS5_Pos (5U)                                        /*!< I3C_T::EXTTXTHS: ECMDTTS5 Position */
#define I3C_EXTTXTHS_ECMDTTS5_Msk (0x1UL << I3C_EXTTXTHS_ECMDTTS5_Pos)        /*!< I3C_T::EXTTXTHS: ECMDTTS5 Mask */

#define I3C_EXTTXTHS_ECMDTTS6_Pos (6U)                                        /*!< I3C_T::EXTTXTHS: ECMDTTS6 Position */
#define I3C_EXTTXTHS_ECMDTTS6_Msk (0x1UL << I3C_EXTTXTHS_ECMDTTS6_Pos)        /*!< I3C_T::EXTTXTHS: ECMDTTS6 Mask */

#define I3C_EXTTXTHS_ECMDTTS7_Pos (7U)                                        /*!< I3C_T::EXTTXTHS: ECMDTTS7 Position */
#define I3C_EXTTXTHS_ECMDTTS7_Msk (0x1UL << I3C_EXTTXTHS_ECMDTTS7_Pos)        /*!< I3C_T::EXTTXTHS: ECMDTTS7 Mask */

#define I3C_EXTDBSL0_TDBE0LVL_Pos (0U)                                        /*!< I3C_T::EXTDBSL0: TDBE0LVL Position */
#define I3C_EXTDBSL0_TDBE0LVL_Msk (0xFFUL << I3C_EXTDBSL0_TDBE0LVL_Pos)       /*!< I3C_T::EXTDBSL0: TDBE0LVL Mask */

#define I3C_EXTDBSL0_TDBE1LVL_Pos (16U)                                       /*!< I3C_T::EXTDBSL0: TDBE1LVL Position */
#define I3C_EXTDBSL0_TDBE1LVL_Msk (0xFFUL << I3C_EXTDBSL0_TDBE1LVL_Pos)       /*!< I3C_T::EXTDBSL0: TDBE1LVL Mask */

#define I3C_EXTDBSL1_TDBE2LVL_Pos (0U)                                        /*!< I3C_T::EXTDBSL1: TDBE2LVL Position */
#define I3C_EXTDBSL1_TDBE2LVL_Msk (0xFFUL << I3C_EXTDBSL1_TDBE2LVL_Pos)       /*!< I3C_T::EXTDBSL1: TDBE2LVL Mask */

#define I3C_EXTDBSL1_TDBE3LVL_Pos (16U)                                       /*!< I3C_T::EXTDBSL1: TDBE3LVL Position */
#define I3C_EXTDBSL1_TDBE3LVL_Msk (0xFFUL << I3C_EXTDBSL1_TDBE3LVL_Pos)       /*!< I3C_T::EXTDBSL1: TDBE3LVL Mask */

#define I3C_EXTDBSL2_TDBE4LVL_Pos (0U)                                        /*!< I3C_T::EXTDBSL2: TDBE4LVL Position */
#define I3C_EXTDBSL2_TDBE4LVL_Msk (0xFFUL << I3C_EXTDBSL2_TDBE4LVL_Pos)       /*!< I3C_T::EXTDBSL2: TDBE4LVL Mask */

#define I3C_EXTDBSL2_TDBE5LVL_Pos (16U)                                       /*!< I3C_T::EXTDBSL2: TDBE5LVL Position */
#define I3C_EXTDBSL2_TDBE5LVL_Msk (0xFFUL << I3C_EXTDBSL2_TDBE5LVL_Pos)       /*!< I3C_T::EXTDBSL2: TDBE5LVL Mask */

#define I3C_EXTDBSL3_TDBE6LVL_Pos (0U)                                        /*!< I3C_T::EXTDBSL3: TDBE6LVL Position */
#define I3C_EXTDBSL3_TDBE6LVL_Msk (0xFFUL << I3C_EXTDBSL3_TDBE6LVL_Pos)       /*!< I3C_T::EXTDBSL3: TDBE6LVL Mask */

#define I3C_EXTDBSL3_TDBE7LVL_Pos (16U)                                       /*!< I3C_T::EXTDBSL3: TDBE7LVL Position */
#define I3C_EXTDBSL3_TDBE7LVL_Msk (0xFFUL << I3C_EXTDBSL3_TDBE7LVL_Pos)       /*!< I3C_T::EXTDBSL3: TDBE7LVL Mask */

#define I3C_EXTCMDW1_CMDWORD1_Pos (0U)                                        /*!< I3C_T::EXTCMDW1: CMDWORD1 Position */
#define I3C_EXTCMDW1_CMDWORD1_Msk (0xFFFFFFFFUL << I3C_EXTCMDW1_CMDWORD1_Pos) /*!< I3C_T::EXTCMDW1: CMDWORD1 Mask */

#define I3C_EXTCMDW2_CMDWORD2_Pos (0U)                                        /*!< I3C_T::EXTCMDW2: CMDWORD2 Position */
#define I3C_EXTCMDW2_CMDWORD2_Msk (0xFFFFFFFFUL << I3C_EXTCMDW2_CMDWORD2_Pos) /*!< I3C_T::EXTCMDW2: CMDWORD2 Mask */

#define I3C_EXTCMDW3_CMDWORD3_Pos (0U)                                        /*!< I3C_T::EXTCMDW3: CMDWORD3 Position */
#define I3C_EXTCMDW3_CMDWORD3_Msk (0xFFFFFFFFUL << I3C_EXTCMDW3_CMDWORD3_Pos) /*!< I3C_T::EXTCMDW3: CMDWORD3 Mask */

#define I3C_EXTDBRST_EXT0RST_Pos  (0U)                                        /*!< I3C_T::EXTDBRST: EXT0RST Position */
#define I3C_EXTDBRST_EXT0RST_Msk  (0x1UL << I3C_EXTDBRST_EXT0RST_Pos)         /*!< I3C_T::EXTDBRST: EXT0RST Mask */

#define I3C_EXTDBRST_EXT1RST_Pos  (1U)                                        /*!< I3C_T::EXTDBRST: EXT1RST Position */
#define I3C_EXTDBRST_EXT1RST_Msk  (0x1UL << I3C_EXTDBRST_EXT1RST_Pos)         /*!< I3C_T::EXTDBRST: EXT1RST Mask */

#define I3C_EXTDBRST_EXT2RST_Pos  (2U)                                        /*!< I3C_T::EXTDBRST: EXT2RST Position */
#define I3C_EXTDBRST_EXT2RST_Msk  (0x1UL << I3C_EXTDBRST_EXT2RST_Pos)         /*!< I3C_T::EXTDBRST: EXT2RST Mask */

#define I3C_EXTDBRST_EXT3RST_Pos  (3U)                                        /*!< I3C_T::EXTDBRST: EXT3RST Position */
#define I3C_EXTDBRST_EXT3RST_Msk  (0x1UL << I3C_EXTDBRST_EXT3RST_Pos)         /*!< I3C_T::EXTDBRST: EXT3RST Mask */

#define I3C_EXTDBRST_EXT4RST_Pos  (4U)                                        /*!< I3C_T::EXTDBRST: EXT4RST Position */
#define I3C_EXTDBRST_EXT4RST_Msk  (0x1UL << I3C_EXTDBRST_EXT4RST_Pos)         /*!< I3C_T::EXTDBRST: EXT4RST Mask */

#define I3C_EXTDBRST_EXT5RST_Pos  (5U)                                        /*!< I3C_T::EXTDBRST: EXT5RST Position */
#define I3C_EXTDBRST_EXT5RST_Msk  (0x1UL << I3C_EXTDBRST_EXT5RST_Pos)         /*!< I3C_T::EXTDBRST: EXT5RST Mask */

#define I3C_EXTDBRST_EXT6RST_Pos  (6U)                                        /*!< I3C_T::EXTDBRST: EXT6RST Position */
#define I3C_EXTDBRST_EXT6RST_Msk  (0x1UL << I3C_EXTDBRST_EXT6RST_Pos)         /*!< I3C_T::EXTDBRST: EXT6RST Mask */

#define I3C_EXTDBRST_EXT7RST_Pos  (7U)                                        /*!< I3C_T::EXTDBRST: EXT7RST Position */
#define I3C_EXTDBRST_EXT7RST_Msk  (0x1UL << I3C_EXTDBRST_EXT7RST_Pos)         /*!< I3C_T::EXTDBRST: EXT7RST Mask */

#define I3C_EXTCMDFS_ECMD0FIN_Pos (0U)                                        /*!< I3C_T::EXTCMDFS: ECMD0FIN Position */
#define I3C_EXTCMDFS_ECMD0FIN_Msk (0x1UL << I3C_EXTCMDFS_ECMD0FIN_Pos)        /*!< I3C_T::EXTCMDFS: ECMD0FIN Mask */

#define I3C_EXTCMDFS_ECMD1FIN_Pos (1U)                                        /*!< I3C_T::EXTCMDFS: ECMD1FIN Position */
#define I3C_EXTCMDFS_ECMD1FIN_Msk (0x1UL << I3C_EXTCMDFS_ECMD1FIN_Pos)        /*!< I3C_T::EXTCMDFS: ECMD1FIN Mask */

#define I3C_EXTCMDFS_ECMD2FIN_Pos (2U)                                        /*!< I3C_T::EXTCMDFS: ECMD2FIN Position */
#define I3C_EXTCMDFS_ECMD2FIN_Msk (0x1UL << I3C_EXTCMDFS_ECMD2FIN_Pos)        /*!< I3C_T::EXTCMDFS: ECMD2FIN Mask */

#define I3C_EXTCMDFS_ECMD3FIN_Pos (3U)                                        /*!< I3C_T::EXTCMDFS: ECMD3FIN Position */
#define I3C_EXTCMDFS_ECMD3FIN_Msk (0x1UL << I3C_EXTCMDFS_ECMD3FIN_Pos)        /*!< I3C_T::EXTCMDFS: ECMD3FIN Mask */

#define I3C_EXTCMDFS_ECMD4FIN_Pos (4U)                                        /*!< I3C_T::EXTCMDFS: ECMD4FIN Position */
#define I3C_EXTCMDFS_ECMD4FIN_Msk (0x1UL << I3C_EXTCMDFS_ECMD4FIN_Pos)        /*!< I3C_T::EXTCMDFS: ECMD4FIN Mask */

#define I3C_EXTCMDFS_ECMD5FIN_Pos (5U)                                        /*!< I3C_T::EXTCMDFS: ECMD5FIN Position */
#define I3C_EXTCMDFS_ECMD5FIN_Msk (0x1UL << I3C_EXTCMDFS_ECMD5FIN_Pos)        /*!< I3C_T::EXTCMDFS: ECMD5FIN Mask */

#define I3C_EXTCMDFS_ECMD6FIN_Pos (6U)                                        /*!< I3C_T::EXTCMDFS: ECMD6FIN Position */
#define I3C_EXTCMDFS_ECMD6FIN_Msk (0x1UL << I3C_EXTCMDFS_ECMD6FIN_Pos)        /*!< I3C_T::EXTCMDFS: ECMD6FIN Mask */

#define I3C_EXTCMDFS_ECMD7FIN_Pos (7U)                                        /*!< I3C_T::EXTCMDFS: ECMD7FIN Position */
#define I3C_EXTCMDFS_ECMD7FIN_Msk (0x1UL << I3C_EXTCMDFS_ECMD7FIN_Pos)        /*!< I3C_T::EXTCMDFS: ECMD7FIN Mask */

#define I3C_EXTTHLD_RXRSPTH_Pos   (0U)                                        /*!< I3C_T::EXTTHLD: RXRSPTH Position */
#define I3C_EXTTHLD_RXRSPTH_Msk   (0x7UL << I3C_EXTTHLD_RXRSPTH_Pos)          /*!< I3C_T::EXTTHLD: RXRSPTH Mask */

#define I3C_EXTTHLD_TXEBTH_Pos    (16U)                                       /*!< I3C_T::EXTTHLD: TXEBTH Position */
#define I3C_EXTTHLD_TXEBTH_Msk    (0x7UL << I3C_EXTTHLD_TXEBTH_Pos)           /*!< I3C_T::EXTTHLD: TXEBTH Mask */

#define I3C_GRPASTS_GRPADDR_Pos   (0U)                                        /*!< I3C_T::GRPASTS: GRPADDR Position */
#define I3C_GRPASTS_GRPADDR_Msk   (0x7FUL << I3C_GRPASTS_GRPADDR_Pos)         /*!< I3C_T::GRPASTS: GRPADDR Mask */

#define I3C_GRPASTS_GRPAVLD0_Pos  (8U)                                        /*!< I3C_T::GRPASTS: GRPAVLD0 Position */
#define I3C_GRPASTS_GRPAVLD0_Msk  (0x1UL << I3C_GRPASTS_GRPAVLD0_Pos)         /*!< I3C_T::GRPASTS: GRPAVLD0 Mask */

#define I3C_GRPASTS_GRPAVLD1_Pos  (9U)                                        /*!< I3C_T::GRPASTS: GRPAVLD1 Position */
#define I3C_GRPASTS_GRPAVLD1_Msk  (0x1UL << I3C_GRPASTS_GRPAVLD1_Pos)         /*!< I3C_T::GRPASTS: GRPAVLD1 Mask */

#define I3C_GRPASTS_GRPAVLD2_Pos  (10U)                                       /*!< I3C_T::GRPASTS: GRPAVLD2 Position */
#define I3C_GRPASTS_GRPAVLD2_Msk  (0x1UL << I3C_GRPASTS_GRPAVLD2_Pos)         /*!< I3C_T::GRPASTS: GRPAVLD2 Mask */

#define I3C_GRPASTS_GRPAVLD3_Pos  (11U)                                       /*!< I3C_T::GRPASTS: GRPAVLD3 Position */
#define I3C_GRPASTS_GRPAVLD3_Msk  (0x1UL << I3C_GRPASTS_GRPAVLD3_Pos)         /*!< I3C_T::GRPASTS: GRPAVLD3 Mask */

#define I3C_GRPASTS_GRPAVLD4_Pos  (12U)                                       /*!< I3C_T::GRPASTS: GRPAVLD4 Position */
#define I3C_GRPASTS_GRPAVLD4_Msk  (0x1UL << I3C_GRPASTS_GRPAVLD4_Pos)         /*!< I3C_T::GRPASTS: GRPAVLD4 Mask */

#define I3C_TGTCHAR1_PIDMSB_Pos   (0U)                                        /*!< I3C_T::TGTCHAR1: PIDMSB Position */
#define I3C_TGTCHAR1_PIDMSB_Msk   (0xFFFFFFFFUL << I3C_TGTCHAR1_PIDMSB_Pos)   /*!< I3C_T::TGTCHAR1: PIDMSB Mask */

#define I3C_TGTCHAR2_PIDLSB_Pos   (0U)                                        /*!< I3C_T::TGTCHAR2: PIDLSB Position */
#define I3C_TGTCHAR2_PIDLSB_Msk   (0xFFFFUL << I3C_TGTCHAR2_PIDLSB_Pos)       /*!< I3C_T::TGTCHAR2: PIDLSB Mask */

#define I3C_TGTCHAR3_DCR_Pos      (0U)                                        /*!< I3C_T::TGTCHAR3: DCR Position */
#define I3C_TGTCHAR3_DCR_Msk      (0xFFUL << I3C_TGTCHAR3_DCR_Pos)            /*!< I3C_T::TGTCHAR3: DCR Mask */

#define I3C_TGTCHAR3_BCR_Pos      (8U)                                        /*!< I3C_T::TGTCHAR3: BCR Position */
#define I3C_TGTCHAR3_BCR_Msk      (0xFFUL << I3C_TGTCHAR3_BCR_Pos)            /*!< I3C_T::TGTCHAR3: BCR Mask */

#define I3C_TGTCHAR4_DADDR_Pos    (0U)                                        /*!< I3C_T::TGTCHAR4: DADDR Position */
#define I3C_TGTCHAR4_DADDR_Msk    (0xFFUL << I3C_TGTCHAR4_DADDR_Pos)          /*!< I3C_T::TGTCHAR4: DADDR Mask */

#define I3C_TGTCFG_SADDR_Pos      (0U)                                        /*!< I3C_T::TGTCFG: SADDR Position */
#define I3C_TGTCFG_SADDR_Msk      (0x7FUL << I3C_TGTCFG_SADDR_Pos)            /*!< I3C_T::TGTCFG: SADDR Mask */

#define I3C_TGTCFG_IBIPECEN_Pos   (11U)                                       /*!< I3C_T::TGTCFG: IBIPECEN Position */
#define I3C_TGTCFG_IBIPECEN_Msk   (0x1UL << I3C_TGTCFG_IBIPECEN_Pos)          /*!< I3C_T::TGTCFG: IBIPECEN Mask */

#define I3C_TGTCFG_IBIWMDB_Pos    (12U)                                       /*!< I3C_T::TGTCFG: IBIWMDB Position */
#define I3C_TGTCFG_IBIWMDB_Msk    (0x1UL << I3C_TGTCFG_IBIWMDB_Pos)           /*!< I3C_T::TGTCFG: IBIWMDB Mask */

#define I3C_TGTCFG_SIRREJ_Pos     (13U)                                       /*!< I3C_T::TGTCFG: SIRREJ Position */
#define I3C_TGTCFG_SIRREJ_Msk     (0x1UL << I3C_TGTCFG_SIRREJ_Pos)            /*!< I3C_T::TGTCFG: SIRREJ Mask */

#define I3C_TGTCFG_MRREJ_Pos      (14U)                                       /*!< I3C_T::TGTCFG: MRREJ Position */
#define I3C_TGTCFG_MRREJ_Msk      (0x1UL << I3C_TGTCFG_MRREJ_Pos)             /*!< I3C_T::TGTCFG: MRREJ Mask */

#define I3C_TGTCFG_DADDR_Pos      (16U)                                       /*!< I3C_T::TGTCFG: DADDR Position */
#define I3C_TGTCFG_DADDR_Msk      (0x7FUL << I3C_TGTCFG_DADDR_Pos)            /*!< I3C_T::TGTCFG: DADDR Mask */

#define I3C_TGTCFG_DAOPBIT_Pos    (23U)                                       /*!< I3C_T::TGTCFG: DAOPBIT Position */
#define I3C_TGTCFG_DAOPBIT_Msk    (0x1UL << I3C_TGTCFG_DAOPBIT_Pos)           /*!< I3C_T::TGTCFG: DAOPBIT Mask */

#define I3C_TGTCFG_NACKRTY_Pos    (29U)                                       /*!< I3C_T::TGTCFG: NACKRTY Position */
#define I3C_TGTCFG_NACKRTY_Msk    (0x3UL << I3C_TGTCFG_NACKRTY_Pos)           /*!< I3C_T::TGTCFG: NACKRTY Mask */

#define I3C_TGTCFG_DEVTYPE_Pos    (31U)                                       /*!< I3C_T::TGTCFG: DEVTYPE Position */
#define I3C_TGTCFG_DEVTYPE_Msk    (0x1UL << I3C_TGTCFG_DEVTYPE_Pos)           /*!< I3C_T::TGTCFG: DEVTYPE Mask */

#define I3C_VTGTADDR_SADDR_Pos    (0U)                                        /*!< I3C_T::VTGTADDR: SADDR Position */
#define I3C_VTGTADDR_SADDR_Msk    (0x7FUL << I3C_VTGTADDR_SADDR_Pos)          /*!< I3C_T::VTGTADDR: SADDR Mask */

#define I3C_VTGTADDR_SAVALID_Pos  (7U)                                        /*!< I3C_T::VTGTADDR: SAVALID Position */
#define I3C_VTGTADDR_SAVALID_Msk  (0x1UL << I3C_VTGTADDR_SAVALID_Pos)         /*!< I3C_T::VTGTADDR: SAVALID Mask */

#define I3C_VTGTADDR_DADDR_Pos    (8U)                                        /*!< I3C_T::VTGTADDR: DADDR Position */
#define I3C_VTGTADDR_DADDR_Msk    (0x7FUL << I3C_VTGTADDR_DADDR_Pos)          /*!< I3C_T::VTGTADDR: DADDR Mask */

#define I3C_VTGTADDR_DAVALID_Pos  (15U)                                       /*!< I3C_T::VTGTADDR: DAVALID Position */
#define I3C_VTGTADDR_DAVALID_Msk  (0x1UL << I3C_VTGTADDR_DAVALID_Pos)         /*!< I3C_T::VTGTADDR: DAVALID Mask */

#define I3C_VTGTADDR_ENABLE_Pos   (31U)                                       /*!< I3C_T::VTGTADDR: ENABLE Position */
#define I3C_VTGTADDR_ENABLE_Msk   (0x1UL << I3C_VTGTADDR_ENABLE_Pos)          /*!< I3C_T::VTGTADDR: ENABLE Mask */

#define I3C_VTGTMID_PIDTYPE_Pos   (0U)                                        /*!< I3C_T::VTGTMID: PIDTYPE Position */
#define I3C_VTGTMID_PIDTYPE_Msk   (0x1UL << I3C_VTGTMID_PIDTYPE_Pos)          /*!< I3C_T::VTGTMID: PIDTYPE Mask */

#define I3C_VTGTMID_MID_Pos       (1U)                                        /*!< I3C_T::VTGTMID: MID Position */
#define I3C_VTGTMID_MID_Msk       (0x7FFFUL << I3C_VTGTMID_MID_Pos)           /*!< I3C_T::VTGTMID: MID Mask */

#define I3C_VTGTPID_ADDLMEAN_Pos  (0U)                                        /*!< I3C_T::VTGTPID: ADDLMEAN Position */
#define I3C_VTGTPID_ADDLMEAN_Msk  (0xFFFUL << I3C_VTGTPID_ADDLMEAN_Pos)       /*!< I3C_T::VTGTPID: ADDLMEAN Mask */

#define I3C_VTGTPID_INSTID_Pos    (12U)                                       /*!< I3C_T::VTGTPID: INSTID Position */
#define I3C_VTGTPID_INSTID_Msk    (0xFUL << I3C_VTGTPID_INSTID_Pos)           /*!< I3C_T::VTGTPID: INSTID Mask */

#define I3C_VTGTPID_PARTID_Pos    (16U)                                       /*!< I3C_T::VTGTPID: PARTID Position */
#define I3C_VTGTPID_PARTID_Msk    (0xFFFFUL << I3C_VTGTPID_PARTID_Pos)        /*!< I3C_T::VTGTPID: PARTID Mask */

#define I3C_VTGTCHAR_MXDSLIMT_Pos (0U)                                        /*!< I3C_T::VTGTCHAR: MXDSLIMT Position */
#define I3C_VTGTCHAR_MXDSLIMT_Msk (0x1UL << I3C_VTGTCHAR_MXDSLIMT_Pos)        /*!< I3C_T::VTGTCHAR: MXDSLIMT Mask */

#define I3C_VTGTCHAR_IBICAP_Pos   (1U)                                        /*!< I3C_T::VTGTCHAR: IBICAP Position */
#define I3C_VTGTCHAR_IBICAP_Msk   (0x1UL << I3C_VTGTCHAR_IBICAP_Pos)          /*!< I3C_T::VTGTCHAR: IBICAP Mask */

#define I3C_VTGTCHAR_IBIPL_Pos    (2U)                                        /*!< I3C_T::VTGTCHAR: IBIPL Position */
#define I3C_VTGTCHAR_IBIPL_Msk    (0x1UL << I3C_VTGTCHAR_IBIPL_Pos)           /*!< I3C_T::VTGTCHAR: IBIPL Mask */

#define I3C_VTGTCHAR_OLINECAP_Pos (3U)                                        /*!< I3C_T::VTGTCHAR: OLINECAP Position */
#define I3C_VTGTCHAR_OLINECAP_Msk (0x1UL << I3C_VTGTCHAR_OLINECAP_Pos)        /*!< I3C_T::VTGTCHAR: OLINECAP Mask */

#define I3C_VTGTCHAR_BRIDGEID_Pos (4U)                                        /*!< I3C_T::VTGTCHAR: BRIDGEID Position */
#define I3C_VTGTCHAR_BRIDGEID_Msk (0x1UL << I3C_VTGTCHAR_BRIDGEID_Pos)        /*!< I3C_T::VTGTCHAR: BRIDGEID Mask */

#define I3C_VTGTCHAR_HDRCAP_Pos   (5U)                                        /*!< I3C_T::VTGTCHAR: HDRCAP Position */
#define I3C_VTGTCHAR_HDRCAP_Msk   (0x1UL << I3C_VTGTCHAR_HDRCAP_Pos)          /*!< I3C_T::VTGTCHAR: HDRCAP Mask */

#define I3C_VTGTCHAR_DEVROLE_Pos  (6U)                                        /*!< I3C_T::VTGTCHAR: DEVROLE Position */
#define I3C_VTGTCHAR_DEVROLE_Msk  (0x3UL << I3C_VTGTCHAR_DEVROLE_Pos)         /*!< I3C_T::VTGTCHAR: DEVROLE Mask */

#define I3C_VTGTCHAR_DCR_Pos      (8U)                                        /*!< I3C_T::VTGTCHAR: DCR Position */
#define I3C_VTGTCHAR_DCR_Msk      (0xFFUL << I3C_VTGTCHAR_DCR_Pos)            /*!< I3C_T::VTGTCHAR: DCR Mask */

#define I3C_VTGTCHAR_HDRCAPV_Pos  (16U)                                       /*!< I3C_T::VTGTCHAR: HDRCAPV Position */
#define I3C_VTGTCHAR_HDRCAPV_Msk  (0xFFUL << I3C_VTGTCHAR_HDRCAPV_Pos)        /*!< I3C_T::VTGTCHAR: HDRCAPV Mask */

/** @} I3C_CONST */
/** @} end of I3C register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __I3C_REG_H__ */

