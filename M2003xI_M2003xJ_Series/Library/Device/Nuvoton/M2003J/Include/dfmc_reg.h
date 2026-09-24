/**************************************************************************//**
 * @file     dfmc_reg.h
 * @version  V1.00
 * @brief    DFMC register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __DFMC_REG_H__
#define __DFMC_REG_H__

/** @addtogroup REGISTER Control Register

  @{

*/


/*---------------------- Data Flash Memory Controller -------------------------*/
/**
    @addtogroup Data Flash Memory Controller(DFMC)
    Memory Mapped Structure for DFMC Controller
  @{
*/

typedef struct
{


    /**
     * @var DFMC_T::ISPCTL
     * Offset: 0x00  ISP Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPEN     |ISP Enable Bit (Write Protect)
     * |        |          |ISP function enable bit. Set this bit to enable ISP function.
     * |        |          |0 = ISP function Disabled.
     * |        |          |1 = ISP function Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[3]     |DATAEN    |Data Flash Update Enable Bit (Write Protect)
     * |        |          |0 = Data Flash cannot be updated.
     * |        |          |1 = Data Flash can be updated.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[6]     |ISPFF     |ISP Fail Flag (Write Protect)
     * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
     * |        |          |This bit needs to be cleared by writing 1 to it.
     * |        |          |- Data Flash writes to itself if DATAEN is set to 0.
     * |        |          |- Erase or Program command at brown-out detected
     * |        |          |- Destination address is illegal, such as over an available range.
     * |        |          |- Invalid ISP commands
     * |        |          |- Checksum or Flash All One Verification is not executed in their valid range
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[24]    |ISPIFEN   |ISP Interrupt Enable bit (Write Protect)
     * |        |          |0 = ISP Interrupt Disabled.
     * |        |          |1 = ISP Interrupt Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var DFMC_T::ISPADDR
     * Offset: 0x04  ISP Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPADDR   |ISP Address
     * |        |          |The M2003J is equipped with embedded Data Flash
     * |        |          |ISPADDR[1:0] must be kept 00 for ISP 32-bit operation.
     * |        |          |For CRC32 Checksum Calculation command, this field is the Data Flash starting address for checksum calculation, 25612 bytes alignment is necessary for CRC32 checksum calculation.
     * |        |          |For Data Flash32-bit Program, ISP address needs word alignment (4-byte).
     * @var DFMC_T::ISPDAT
     * Offset: 0x08  ISP Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPDAT    |ISP Data
     * |        |          |Write data to this register before ISP program operation.
     * |        |          |Read data from this register after ISP read operation.
     * |        |          |When ISPFF (DFMC_ISPCTL[6]) is 1, ISPDAT = 0xffff_ffff
     * |        |          |For Run CRC32 Checksum Calculation command, ISPDAT is the memory size (byte) and 25612 bytes alignment
     * |        |          |For ISP Read CRC32 Checksum command, ISPDAT is the checksum result
     * |        |          |If ISPDAT = 0x0000_0000, it means that (1) the checksum calculation is in progress, or (2) the memory range for checksum calculation is incorrect
     * @var DFMC_T::ISPCMD
     * Offset: 0x0C  ISP Command Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[6:0]   |CMD       |ISP Command
     * |        |          |ISP command table is shown below:
     * |        |          |0x00= Data FLASH Read.
     * |        |          |0x30= Data FLASH Read without ECC.
     * |        |          |0x08= Read Data Flash All-One Result.
     * |        |          |0x0B= Read Company ID.
     * |        |          |0x0C= Read Device ID.
     * |        |          |0x0D= Read Checksum.
     * |        |          |0x21= Data FLASH 32-bit Program.
     * |        |          |0x22= Data FLASH Page Erase. Erase any page in Data Flash.
     * |        |          |0x26= Data FLASH Mass Erase. Erase all pages in Data Flash.
     * |        |          |0x28= Run Data Flash All-One Verification.
     * |        |          |0x2D= Run Checksum Calculation.
     * |        |          |The other commands are invalid.
     * @var DFMC_T::ISPTRG
     * Offset: 0x10  ISP Trigger Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPGO     |ISP Start Trigger (Write Protect)
     * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP operation is finished.
     * |        |          |0 = ISP operation is finished.
     * |        |          |1 = ISP is progressed.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var DFMC_T::ISPSTS
     * Offset: 0x40  ISP Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPBUSY   |ISP Busy Flag (Read Only)
     * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP operation is finished.
     * |        |          |This bit is the mirror of ISPGO(DFMC_ISPTRG[0]).
     * |        |          |0 = ISP operation is finished.
     * |        |          |1 = ISP is progressed.
     * |[6]     |ISPFF     |ISP Fail Flag (Write Protect)
     * |        |          |This bit is the mirror of ISPFF (DFMC_ISPCTL[6]), it needs to be cleared by writing 1 to DFMC_ISPCTL[6] or DFMC_ISPSTS[6]
     * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
     * |        |          |- Data Flash writes to itself if DATAEN is set to 0.
     * |        |          |- Erase or Program command at brown-out detected
     * |        |          |- Destination address is illegal, such as over an available range.
     * |        |          |- Invalid ISP commands
     * |        |          |- Checksum or Flash All One Verification is not executed in their valid range
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[7]     |ALLONE    |Data Flash All-one Verification Flag
     * |        |          |This bit is set by hardware if all of Flash bits are 1, and clear if Flash bits are not all 1 after "Run Data Flash All-One Verification" complete; this bit also can be clear by writing 1
     * |        |          |0 = Data Flash bits are not all 1 after "Run Data Flash All-One Verification" is complete.
     * |        |          |1 = All of Data Flash bits are 1 after "Run Data Flash All-One Verification" is complete.
     * |[24]    |ISPIF     |ISP Interrupt Flag
     * |        |          |0 = ISP command not finish or ISP fail flag is 0.
     * |        |          |1 = ISP command finish or ISP fail is 1.
     * |        |          |Note: Write 1 to clear this bit.
     * @var DFMC_T::CYCCTL
     * Offset: 0x4C  Data Flash Access Cycle Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |CYCLE     |Data Flash Access Cycle Control (Write Protect)
     * |        |          |This register is updated by software.
     * |        |          |0001 = CPU access with one wait cycle if cache miss; Data Flash access cycle is 1;.
     * |        |          |The HCLK working frequency range range is<27 MHz
     * |        |          |0010 = CPU access with two wait cycles if cache miss; Data Flash access cycle is 2;.
     * |        |          |The optimized HCLK working frequency range is 27~40 MHz
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var DFMC_T::MPDAT2
     * Offset: 0x88  ISP Data2 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |ISPDAT2   |ISP Data 2
     * |        |          |Read data from this register after ISP 32-bit read operation or ECCSEBCF(DFMC_ECCSTS[0]) is high can know which bit is error
     * |        |          |ISPDAT2 does not continuously store the position information of ECC errors
     * |        |          |Therefore, if the user wants to know which bit generated the error, they need to read it before the next ISP action after an ECC single-bit error occurs, or access that position again.
     * |        |          |When ISPFF (DFMC_ISPCTL[6]) is 1, ISPDAT2 = 0xff.
     * |        |          |This register is the ECC 7-bit data.
     * @var DFMC_T::ECCCTL
     * Offset: 0x130  DFMC Error Code Correction Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SEBDINTEN |Single Error Bits Detection Interrupt Enable (Write Protect)
     * |        |          |1 = Single Error Bit Detection Interrupt Enabled.
     * |        |          |0 = Single Error Bit Detection Interrupt Disabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |DEBDINTEN |Double Error Bits Detection Interrupt Enable (Write Protect)
     * |        |          |1 = Double Error Bits Detection Interrupt Enabled.
     * |        |          |0 = Double Error Bits Detection Interrupt Disabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var DFMC_T::ECCSTS
     * Offset: 0x134  DFMC Error Code Correction status
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ECCSEBCF  |ECC Single Error Bits Correction Flag
     * |        |          |This bit is set by hardware when DFMC detects and corrects a ECC single error bits fault when CPU access Flash or ISP read
     * |        |          |It need to be cleared by writing 1 to ECCSEBCF
     * |        |          |When ECCSEBCF and SEBDINTEN(DFMC_ECCCTL[0])=1 cause interrupt.
     * |        |          |1 = DFMC detects and corrects ECC single error bits fault.
     * |        |          |0 = DFMC does not detect ECC single error bits fault.
     * |[1]     |ECCDEBDF  |ECC Double Error Bits Detection Flag
     * |        |          |This bit is set by hardware when DFMC detects a ECC double error bits fault when CPU access Flash or ISP read
     * |        |          |It need to be cleared by writing 1 to ECCDEBDF
     * |        |          |When ECCDEBDF and DEBDINTEN(DFMC_ECCCTL[1])=1 cause interrupt.
     * |        |          |1 = DFMC detects ECC double error bits fault.
     * |        |          |0 = DFMC does not detect ECC double error bits fault.
     * @var DFMC_T::ECCSEFAR
     * Offset: 0x138  DFMC ECC Single Error fault Adress Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ECCSEFAR  |ECC Single Error Fault Address Register (Read Only)
     * |        |          |When DFMC detects a ECCSEBCF(DFMC_ECCSTS[0]) would record fault address at the register
     * |        |          |The register is cleared by writing 1 to ECCSEBCF
     * |        |          |ECCSEFAR is 64-bit alignment so ECCSEFAR[2:0] are always 0.
     * @var DFMC_T::ECCDEFAR
     * Offset: 0x13C  DFMC ECC Double Error fault Adress Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ECCDEFAR  |ECC Double Error Fault Address Register (Read Only)
     * |        |          |When DFMC detects a ECCDEBDF(DFMC_ECCSTS[1]) would record fault address at the register
     * |        |          |The register is cleared by writing 1 to ECCDEBDF
     * |        |          |ECCDEBDF is 64-bit alignment so ECCDEFAR[2:0] are always 0.
 */
    __IO uint32_t ISPCTL;                /*!< [0x0000] ISP Control Register                                             */
    __IO uint32_t ISPADDR;               /*!< [0x0004] ISP Address Register                                             */
    __IO uint32_t ISPDAT;                /*!< [0x0008] ISP Data Register                                                */
    __IO uint32_t ISPCMD;                /*!< [0x000c] ISP Command Register                                             */
    __IO uint32_t ISPTRG;                /*!< [0x0010] ISP Trigger Control Register                                     */
    __I  uint32_t RESERVE0[11];
    __IO uint32_t ISPSTS;                /*!< [0x0040] ISP Status Register                                              */
    __I  uint32_t RESERVE1[2];
    __IO uint32_t CYCCTL;                /*!< [0x004c] Data Flash Access Cycle Control Register                         */
    __I  uint32_t RESERVE2[14];
    __IO uint32_t MPDAT2;                /*!< [0x0088] ISP Data2 Register                                               */
    __I  uint32_t RESERVE3[41];
    __IO uint32_t ECCCTL;                /*!< [0x0130] DFMC Error Code Correction Control                               */
    __IO uint32_t ECCSTS;                /*!< [0x0134] DFMC Error Code Correction status                                */
    __I  uint32_t ECCSEFAR;              /*!< [0x0138] DFMC ECC Single Error fault Adress Register                      */
    __I  uint32_t ECCDEFAR;              /*!< [0x013c] DFMC ECC Double Error fault Adress Register                      */

} DFMC_T;

/**
    @addtogroup DFMC_CONST DFMC Bit Field Definition
    Constant Definitions for DFMC Controller
  @{
*/

#define DFMC_ISPCTL_ISPEN_Pos            (0)                                               /*!< DFMC_T::ISPCTL: ISPEN Position         */
#define DFMC_ISPCTL_ISPEN_Msk            (0x1ul << DFMC_ISPCTL_ISPEN_Pos)                  /*!< DFMC_T::ISPCTL: ISPEN Mask             */

#define DFMC_ISPCTL_DATAEN_Pos           (3)                                               /*!< DFMC_T::ISPCTL: DATAEN Position        */
#define DFMC_ISPCTL_DATAEN_Msk           (0x1ul << DFMC_ISPCTL_DATAEN_Pos)                 /*!< DFMC_T::ISPCTL: DATAEN Mask            */

#define DFMC_ISPCTL_ISPFF_Pos            (6)                                               /*!< DFMC_T::ISPCTL: ISPFF Position         */
#define DFMC_ISPCTL_ISPFF_Msk            (0x1ul << DFMC_ISPCTL_ISPFF_Pos)                  /*!< DFMC_T::ISPCTL: ISPFF Mask             */

#define DFMC_ISPCTL_ISPIFEN_Pos          (24)                                              /*!< DFMC_T::ISPCTL: ISPIFEN Position       */
#define DFMC_ISPCTL_ISPIFEN_Msk          (0x1ul << DFMC_ISPCTL_ISPIFEN_Pos)                /*!< DFMC_T::ISPCTL: ISPIFEN Mask           */

#define DFMC_ISPADDR_ISPADDR_Pos         (0)                                               /*!< DFMC_T::ISPADDR: ISPADDR Position      */
#define DFMC_ISPADDR_ISPADDR_Msk         (0xfffffffful << DFMC_ISPADDR_ISPADDR_Pos)        /*!< DFMC_T::ISPADDR: ISPADDR Mask          */

#define DFMC_ISPDAT_ISPDAT_Pos           (0)                                               /*!< DFMC_T::ISPDAT: ISPDAT Position        */
#define DFMC_ISPDAT_ISPDAT_Msk           (0xfffffffful << DFMC_ISPDAT_ISPDAT_Pos)          /*!< DFMC_T::ISPDAT: ISPDAT Mask            */

#define DFMC_ISPCMD_CMD_Pos              (0)                                               /*!< DFMC_T::ISPCMD: CMD Position           */
#define DFMC_ISPCMD_CMD_Msk              (0x7ful << DFMC_ISPCMD_CMD_Pos)                   /*!< DFMC_T::ISPCMD: CMD Mask               */

#define DFMC_ISPTRG_ISPGO_Pos            (0)                                               /*!< DFMC_T::ISPTRG: ISPGO Position         */
#define DFMC_ISPTRG_ISPGO_Msk            (0x1ul << DFMC_ISPTRG_ISPGO_Pos)                  /*!< DFMC_T::ISPTRG: ISPGO Mask             */

#define DFMC_ISPSTS_ISPBUSY_Pos          (0)                                               /*!< DFMC_T::ISPSTS: ISPBUSY Position       */
#define DFMC_ISPSTS_ISPBUSY_Msk          (0x1ul << DFMC_ISPSTS_ISPBUSY_Pos)                /*!< DFMC_T::ISPSTS: ISPBUSY Mask           */

#define DFMC_ISPSTS_ISPFF_Pos            (6)                                               /*!< DFMC_T::ISPSTS: ISPFF Position         */
#define DFMC_ISPSTS_ISPFF_Msk            (0x1ul << DFMC_ISPSTS_ISPFF_Pos)                  /*!< DFMC_T::ISPSTS: ISPFF Mask             */

#define DFMC_ISPSTS_ALLONE_Pos           (7)                                               /*!< DFMC_T::ISPSTS: ALLONE Position        */
#define DFMC_ISPSTS_ALLONE_Msk           (0x1ul << DFMC_ISPSTS_ALLONE_Pos)                 /*!< DFMC_T::ISPSTS: ALLONE Mask            */

#define DFMC_ISPSTS_ISPIF_Pos            (24)                                              /*!< DFMC_T::ISPSTS: ISPIF Position         */
#define DFMC_ISPSTS_ISPIF_Msk            (0x1ul << DFMC_ISPSTS_ISPIF_Pos)                  /*!< DFMC_T::ISPSTS: ISPIF Mask             */

#define DFMC_CYCCTL_CYCLE_Pos            (0)                                               /*!< DFMC_T::CYCCTL: CYCLE Position         */
#define DFMC_CYCCTL_CYCLE_Msk            (0xful << DFMC_CYCCTL_CYCLE_Pos)                  /*!< DFMC_T::CYCCTL: CYCLE Mask             */

#define DFMC_MPDAT2_ISPDAT2_Pos          (0)                                               /*!< DFMC_T::MPDAT2: ISPDAT2 Position       */
#define DFMC_MPDAT2_ISPDAT2_Msk          (0xfful << DFMC_MPDAT2_ISPDAT2_Pos)               /*!< DFMC_T::MPDAT2: ISPDAT2 Mask           */

#define DFMC_ECCCTL_SEBDINTEN_Pos        (0)                                               /*!< DFMC_T::ECCCTL: SEBDINTEN Position     */
#define DFMC_ECCCTL_SEBDINTEN_Msk        (0x1ul << DFMC_ECCCTL_SEBDINTEN_Pos)              /*!< DFMC_T::ECCCTL: SEBDINTEN Mask         */

#define DFMC_ECCCTL_DEBDINTEN_Pos        (1)                                               /*!< DFMC_T::ECCCTL: DEBDINTEN Position     */
#define DFMC_ECCCTL_DEBDINTEN_Msk        (0x1ul << DFMC_ECCCTL_DEBDINTEN_Pos)              /*!< DFMC_T::ECCCTL: DEBDINTEN Mask         */

#define DFMC_ECCSTS_ECCSEBCF_Pos         (0)                                               /*!< DFMC_T::ECCSTS: ECCSEBCF Position      */
#define DFMC_ECCSTS_ECCSEBCF_Msk         (0x1ul << DFMC_ECCSTS_ECCSEBCF_Pos)               /*!< DFMC_T::ECCSTS: ECCSEBCF Mask          */

#define DFMC_ECCSTS_ECCDEBDF_Pos         (1)                                               /*!< DFMC_T::ECCSTS: ECCDEBDF Position      */
#define DFMC_ECCSTS_ECCDEBDF_Msk         (0x1ul << DFMC_ECCSTS_ECCDEBDF_Pos)               /*!< DFMC_T::ECCSTS: ECCDEBDF Mask          */

#define DFMC_ECCSEFAR_ECCSEFAR_Pos       (0)                                               /*!< DFMC_T::ECCSEFAR: ECCSEFAR Position    */
#define DFMC_ECCSEFAR_ECCSEFAR_Msk       (0xfffffffful << DFMC_ECCSEFAR_ECCSEFAR_Pos)      /*!< DFMC_T::ECCSEFAR: ECCSEFAR Mask        */

#define DFMC_ECCDEFAR_ECCDEFAR_Pos       (0)                                               /*!< DFMC_T::ECCDEFAR: ECCDEFAR Position    */
#define DFMC_ECCDEFAR_ECCDEFAR_Msk       (0xfffffffful << DFMC_ECCDEFAR_ECCDEFAR_Pos)      /*!< DFMC_T::ECCDEFAR: ECCDEFAR Mask        */

/**@}*/ /* DFMC_CONST */
/**@}*/ /* end of DFMC register group */
/**@}*/ /* end of REGISTER group */

#endif /* __DFMC_REG_H__ */
