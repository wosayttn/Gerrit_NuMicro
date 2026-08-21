/**************************************************************************//**
 * @file     fmc_reg.h
 * @version  V1.00
 * @brief    FMC register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __FMC_REG_H__
#define __FMC_REG_H__

/** @addtogroup REGISTER Control Register

  @{

*/


/*---------------------- Flash Memory Controller -------------------------*/
/**
    @addtogroup FMC Flash Memory Controller(FMC)
    Memory Mapped Structure for FMC Controller
  @{
*/

typedef struct
{


    /**
     * @var FMC_T::ISPCTL
     * Offset: 0x00  ISP Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPEN     |ISP Enable Flag Bit (Read Only)
     * |        |          |0 = ISP function is Disabled status.
     * |        |          |1 = ISP function is Enabled status.
     * |        |          |Note: This bit is read only to show ISP function enable.
     * |[1]     |BS        |Boot Select (Write Protect)
     * |        |          |Set/clear this bit to select next booting from LDROM/APROM, respectively
     * |        |          |This bit also functions as chip booting status flag, which can be used to check where chip booted from
     * |        |          |This bit is initiated with the inversed value of CBS (CONFIG0[7]) after any reset has happened except CPU reset (CPU is 1) or system reset (SYS) has happened
     * |        |          |0 = Booting from APROM.
     * |        |          |1 = Booting from LDROM.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2]     |SPUEN     |SPROM Update Enable Bit (Write Protect)
     * |        |          |0 = SPROM cannot be updated.
     * |        |          |1 = SPROM can be updated.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[3]     |APUEN     |APROM Update Enable Bit (Write Protect)
     * |        |          |0 = APROM cannot be updated when the chip runs in APROM.
     * |        |          |1 = APROM can be updated when the chip runs in APROM.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[4]     |CFGUEN    |CONFIG Update Enable Bit (Write Protect)
     * |        |          |0 = CONFIG cannot be updated.
     * |        |          |1 = CONFIG can be updated.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[5]     |LDUEN     |LDROM Update Enable Bit (Write Protect)
     * |        |          |0 = LDROM cannot be updated.
     * |        |          |1 = LDROM can be updated.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[6]     |ISPFF     |ISP Fail Flag (Write Protect)
     * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
     * |        |          |This bit needs to be cleared by writing 1 to it.
     * |        |          |- APROM writes to itself if APUEN is set to 0.
     * |        |          |- LDROM writes to itself if LDUEN is set to 0.
     * |        |          |- CONFIG is erased/programmed if CFGUEN is set to 0.
     * |        |          |- Page Erase command at LOCK mode with ICE connection.
     * |        |          |- Erase or Program command at brown-out detected
     * |        |          |- Destination address is illegal, such as over an available range.
     * |        |          |- Invalid ISP commands
     * |        |          |- Mass erase when MERASE (CFG0[13]) is disabled
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[24]    |INTEN     |INT Enable Bit (Write Protect)
     * |        |          |0= ISP INT Disabled.
     * |        |          |1= ISP INT Enabled.
     * |        |          |Note: This bit is write protected
     * |        |          |Refer to the SYS_REGLCTL register
     * |        |          |Before using INT, user needs to clear the INTFLAG(FMC_ISPSTS[24]) make sure INT happen at correct time.
     * @var FMC_T::ISPADDR
     * Offset: 0x04  ISP Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPADDR   |ISP Address
     * |        |          |Since equipped with embedded Flash. ISPADDR[1:0] must be kept 00 for ISP 32-bit operation. I
     * |        |          |For CRC32 Checksum Calculation command, this field is the Flash starting address for checksum calculation, 0.5 Kbytes alignment is necessary for CRC32 checksum calculation.
     * |        |          |For Flash32-bit Program, ISP address needs word alignment (4-byte).
     * @var FMC_T::ISPDAT
     * Offset: 0x08  ISP Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPDAT    |ISP Data
     * |        |          |Write data to this register before ISP program operation.
     * |        |          |Read data from this register after ISP read operation.
     * |        |          |When ISPFF (FMC_ISPCTL[6]) is 1, ISPDAT = 0xffff_ffff
     * |        |          |For Run CRC32 Checksum Calculation command, ISPDAT is the memory size (byte) and 0.5 Kbytes alignment
     * |        |          |For ISP Read CRC32 Checksum command, ISPDAT is the checksum result
     * |        |          |If ISPDAT = 0x0000_0000, it means that (1) the checksum calculation is in progress, or (2) the memory range for checksum calculation is incorrect.
     * @var FMC_T::ISPCMD
     * Offset: 0x0C  ISP Command Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[6:0]   |CMD       |ISP Command
     * |        |          |The ISP command table is shown below:
     * |        |          |0x00= FLASH Read.
     * |        |          |0x04= Read Unique ID.
     * |        |          |0x08= Read Flash All-One Result.
     * |        |          |0x0B= Read Company ID.
     * |        |          |0x0C= Read Device ID.
     * |        |          |0x0D= Read Checksum.
     * |        |          |0x21= FLASH 32-bit Program.
     * |        |          |0x22= FLASH Page Erase. Erase any page in APROM, LDROM and SPROM.
     * |        |          |0x28= Run Flash All-One Verification.
     * |        |          |0x2D= Run Checksum Calculation.
     * |        |          |0x2E= Vector Remap.
     * |        |          |0x40 = FLASH 64-bit Read.
     * |        |          |0x60 = FLASH 64-bit Read without ECC.
     * |        |          |0x61 = FLASH 64-bit Program.
     * |        |          |The other commands are invalid.
     * @var FMC_T::ISPTRG
     * Offset: 0x10  ISP Trigger Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPGO     |ISP Start Trigger (Write Protect)
     * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP operation is finished
     * |        |          |When ISPGO=1, the operation of writing value to address from FMC_BA+0x00 to FMC_BA+0x68 would be ignored.
     * |        |          |0 = ISP operation is finished.
     * |        |          |1 = ISP is progressed.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var FMC_T::ISPSTS
     * Offset: 0x40  ISP Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPBUSY   |ISP Busy Flag (Read Only)
     * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP operation is finished.
     * |        |          |This bit is the mirror of ISPGO(FMC_ISPTRG[0]).
     * |        |          |0 = ISP operation is finished.
     * |        |          |1 = ISP is progressed.
     * |[2]     |CBS       |Boot Selection of CONFIG (Read Only)
     * |        |          |This bit is initiated with the CBS (CONFIG0[7]) after any reset is happened except CPU reset (CPU is 1) or system reset (SYS) is happened.
     * |        |          |0 = LDROM with IAP mode.
     * |        |          |1 = APROM with IAP mode.
     * |[6]     |ISPFF     |ISP Fail Flag (Write Protect)
     * |        |          |This bit is the mirror of ISPFF (FMC_ISPCTL[6]), it needs to be cleared by writing 1 to FMC_ISPCTL[6] or FMC_ISPSTS[6]
     * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
     * |        |          |- APROM writes to itself if APUEN is set to 0.
     * |        |          |- LDROM writes to itself if LDUEN is set to 0.
     * |        |          |- CONFIG is erased/programmed if CFGUEN is set to 0.
     * |        |          |- Page Erase command at LOCK mode with ICE connection.
     * |        |          |- Erase or Program command at brown-out detected
     * |        |          |- Destination address is illegal, such as over an available range.
     * |        |          |- Invalid ISP commands
     * |        |          |- Mass erase when MERASE (CFG0[13]) is disabled
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[7]     |ALLONE    |Flash All-one Verification Flag
     * |        |          |This bit is set by hardware if all of Flash bits are 1, and cleared if Flash bits are not all 1 after "Run Flash All-One Verification" is complete; this bit also can be cleared by writing 1
     * |        |          |0 = Flash bits are not all 1 after "Run Flash All-One Verification" is complete.
     * |        |          |1 = All of Flash bits are 1 after "Run Flash All-One Verification" is complete.
     * |[8]     |INTFLAG   |ISP Interrupt Flag
     * |        |          |0 = ISP Not Finished.
     * |        |          |1 = ISP done or ISPFF set.
     * |        |          |Note: This function needs to be enabled by FMC_ISPCTRL[24].Reserved.
     * |[27:9]  |VECMAP    |Vector Page Mapping Address (Read Only)
     * |        |          |All access to 0x0000_0000~0x0000_01FF is remapped to the Flash memory address {VECMAP[14:0], 9'h000} ~ {VECMAP[14:0], 9'h1FF}
     * |[30]    |FBS       |Flash Bank Selection (Read Only)
     * |        |          |This bit indicate which APROM segment is selected to boot.
     * |        |          |0 = Booting from APROM segment 0.
     * |        |          |1 = Booting from APROM segment 1.
     * |[31]    |SCODE     |Security Code Active Flag
     * |        |          |This bit is set by hardware when detecting SPROM secured code is active at Flash initiation, or software writes 1 to this bit to make secured code active; this bit is clear by SPROM page erase operation.
     * |        |          |0 = Secured code is inactive.
     * |        |          |1 = Secured code is active.
     * @var FMC_T::CYCCTL
     * Offset: 0x4C  Flash Access Cycle Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |CYCLE     |Flash Access Cycle Control (Write Protect)
     * |        |          |This register is updated by software. User needs to check the speed of HCLK and set the cycle >0.
     * |        |          |0001 = Flash access cycle is 1.
     * |        |          |The HCLK working frequency range is<25 MHz
     * |        |          |0010 = CPU access with two wait cycles; Flash access cycle is 2;.
     * |        |          |The optimized HCLK working frequency range is 26~40 MHz
     * |        |          |The others are reserved.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var FMC_T::MPDAT0
     * Offset: 0x80  ISP Data0 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPDAT0   |ISP Data 0
     * |        |          |Write data to this register before ISP 64-bit program operation.
     * |        |          |Read data from this register after ISP 64-bit read operation.
     * |        |          |When ISPFF (FMC_ISPCTL[6]) is 1, ISPDAT0 = 0xffff_ffff.
     * |        |          |This register is the first 32-bit read data or write data for 32-bit/64-bit programming, and it is also the mirror of FMC_ISPDAT, both registers keep the same data
     * @var FMC_T::MPDAT1
     * Offset: 0x84  ISP Data1 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPDAT1   |ISP Data 1
     * |        |          |Write data to this register before ISP 64-bit program operation.
     * |        |          |Read data from this register after ISP 64-bit read operation.
     * |        |          |When ISPFF (FMC_ISPCTL[6]) is 1, ISPDAT1 = 0xffff_ffff.
     * |        |          |This register is the second 32-bit read data or write data for 64-bit programming.
     * @var FMC_T::MPDAT2
     * Offset: 0x88  ISP Data2 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |ISPDAT2   |ISP Data 2
     * |        |          |Read data from this register after ISP 64-bit read operation or ECCSEBCF(FMC_ECCSTS[0]) is high can know which bit is error
     * |        |          |ISPDAT2 does not continuously store the position information of ECC errors
     * |        |          |Therefore, if the user wants to know which bit generated the error, they need to read it before the next ISP action after an ECC single-bit error occurs, or access that position again.
     * |        |          |When ISPFF (FMC_ISPCTL[6]) is 1, ISPDAT2 = 0xff.
     * |        |          |This register is the ECC 8-bit data.
     * @var FMC_T::ECCCTL
     * Offset: 0x130  FMC Error Code Correction Control
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
     * @var FMC_T::ECCSTS
     * Offset: 0x134  FMC Error Code Correction status
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ECCSEBCF  |ECC Single Error Bits Correction Flag
     * |        |          |This bit is set by hardware when FMC detects and corrects a ECC single error bits fault when CPU access Flash or ISP read
     * |        |          |It need to be cleared by writing 1 to ECCSEBCF
     * |        |          |When ECCSEBCF and SEBDINTEN(FMC_ECCCTL[0])=1 cause interrupt.
     * |        |          |1 = FMC detects and corrects ECC single error bits fault.
     * |        |          |0 = FMC does not detect ECC single error bits fault.
     * |[1]     |ECCDEBDF  |ECC Double Error Bits Detection Flag
     * |        |          |This bit is set by hardware when FMC detects a ECC double error bits fault when CPU access Flash or ISP read
     * |        |          |It need to be cleared by writing 1 to ECCDEBDF
     * |        |          |When ECCDEBDF and DEBDINTEN(FMC_ECCCTL[1])=1 cause interrupt.
     * |        |          |1 = FMC detects ECC double error bits fault.
     * |        |          |0 = FMC does not detect ECC double error bits fault.
     * @var FMC_T::ECCSEFAR
     * Offset: 0x138  FMC ECC Single Error fault Adress Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ECCSEFAR  |ECC Single Error Fault Address Register (Read Only)
     * |        |          |When FMC detects a ECCSEBCF(FMC_ECCSTS[0]) would record fault address at the register
     * |        |          |The register is cleared by writing 1 to ECCSEBCF
     * |        |          |ECCSEFAR is 64-bit alignment so ECCSEFAR[2:0] are always 0.
     * @var FMC_T::ECCDEFAR
     * Offset: 0x13C  FMC ECC Double Error fault Adress Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ECCDEFAR  |ECC Double Error Fault Address Register (Read Only)
     * |        |          |When FMC detects a ECCDEBDF(FMC_ECCSTS[1]) would record fault address at the register
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
    __IO uint32_t CYCCTL;                /*!< [0x004c] Flash Access Cycle Control Register                              */
    __I  uint32_t RESERVE2[12];
    __IO uint32_t MPDAT0;                /*!< [0x0080] ISP Data0 Register                                               */
    __IO uint32_t MPDAT1;                /*!< [0x0084] ISP Data1 Register                                               */
    __IO uint32_t MPDAT2;                /*!< [0x0088] ISP Data2 Register                                               */
    __I  uint32_t RESERVE3[41];
    __IO uint32_t ECCCTL;                /*!< [0x0130] FMC Error Code Correction Control                                */
    __IO uint32_t ECCSTS;                /*!< [0x0134] FMC Error Code Correction status                                 */
    __I  uint32_t ECCSEFAR;              /*!< [0x0138] FMC ECC Single Error fault Adress Register                       */
    __I  uint32_t ECCDEFAR;              /*!< [0x013c] FMC ECC Double Error fault Adress Register                       */

} FMC_T;

/**
    @addtogroup FMC_CONST FMC Bit Field Definition
    Constant Definitions for FMC Controller
  @{
*/

#define FMC_ISPCTL_ISPEN_Pos             (0)                                               /*!< FMC_T::ISPCTL: ISPEN Position          */
#define FMC_ISPCTL_ISPEN_Msk             (0x1ul << FMC_ISPCTL_ISPEN_Pos)                   /*!< FMC_T::ISPCTL: ISPEN Mask              */

#define FMC_ISPCTL_BS_Pos                (1)                                               /*!< FMC_T::ISPCTL: BS Position             */
#define FMC_ISPCTL_BS_Msk                (0x1ul << FMC_ISPCTL_BS_Pos)                      /*!< FMC_T::ISPCTL: BS Mask                 */

#define FMC_ISPCTL_SPUEN_Pos             (2)                                               /*!< FMC_T::ISPCTL: SPUEN Position          */
#define FMC_ISPCTL_SPUEN_Msk             (0x1ul << FMC_ISPCTL_SPUEN_Pos)                   /*!< FMC_T::ISPCTL: SPUEN Mask              */

#define FMC_ISPCTL_APUEN_Pos             (3)                                               /*!< FMC_T::ISPCTL: APUEN Position          */
#define FMC_ISPCTL_APUEN_Msk             (0x1ul << FMC_ISPCTL_APUEN_Pos)                   /*!< FMC_T::ISPCTL: APUEN Mask              */

#define FMC_ISPCTL_CFGUEN_Pos            (4)                                               /*!< FMC_T::ISPCTL: CFGUEN Position         */
#define FMC_ISPCTL_CFGUEN_Msk            (0x1ul << FMC_ISPCTL_CFGUEN_Pos)                  /*!< FMC_T::ISPCTL: CFGUEN Mask             */

#define FMC_ISPCTL_LDUEN_Pos             (5)                                               /*!< FMC_T::ISPCTL: LDUEN Position          */
#define FMC_ISPCTL_LDUEN_Msk             (0x1ul << FMC_ISPCTL_LDUEN_Pos)                   /*!< FMC_T::ISPCTL: LDUEN Mask              */

#define FMC_ISPCTL_ISPFF_Pos             (6)                                               /*!< FMC_T::ISPCTL: ISPFF Position          */
#define FMC_ISPCTL_ISPFF_Msk             (0x1ul << FMC_ISPCTL_ISPFF_Pos)                   /*!< FMC_T::ISPCTL: ISPFF Mask              */

#define FMC_ISPCTL_INTEN_Pos             (24)                                              /*!< FMC_T::ISPCTL: INTEN Position          */
#define FMC_ISPCTL_INTEN_Msk             (0x1ul << FMC_ISPCTL_INTEN_Pos)                   /*!< FMC_T::ISPCTL: INTEN Mask              */

#define FMC_ISPADDR_ISPADDR_Pos          (0)                                               /*!< FMC_T::ISPADDR: ISPADDR Position       */
#define FMC_ISPADDR_ISPADDR_Msk          (0xfffffffful << FMC_ISPADDR_ISPADDR_Pos)         /*!< FMC_T::ISPADDR: ISPADDR Mask           */

#define FMC_ISPDAT_ISPDAT_Pos            (0)                                               /*!< FMC_T::ISPDAT: ISPDAT Position         */
#define FMC_ISPDAT_ISPDAT_Msk            (0xfffffffful << FMC_ISPDAT_ISPDAT_Pos)           /*!< FMC_T::ISPDAT: ISPDAT Mask             */

#define FMC_ISPCMD_CMD_Pos               (0)                                               /*!< FMC_T::ISPCMD: CMD Position            */
#define FMC_ISPCMD_CMD_Msk               (0x7ful << FMC_ISPCMD_CMD_Pos)                    /*!< FMC_T::ISPCMD: CMD Mask                */

#define FMC_ISPTRG_ISPGO_Pos             (0)                                               /*!< FMC_T::ISPTRG: ISPGO Position          */
#define FMC_ISPTRG_ISPGO_Msk             (0x1ul << FMC_ISPTRG_ISPGO_Pos)                   /*!< FMC_T::ISPTRG: ISPGO Mask              */

#define FMC_ISPSTS_ISPBUSY_Pos           (0)                                               /*!< FMC_T::ISPSTS: ISPBUSY Position        */
#define FMC_ISPSTS_ISPBUSY_Msk           (0x1ul << FMC_ISPSTS_ISPBUSY_Pos)                 /*!< FMC_T::ISPSTS: ISPBUSY Mask            */

#define FMC_ISPSTS_CBS_Pos               (2)                                               /*!< FMC_T::ISPSTS: CBS Position            */
#define FMC_ISPSTS_CBS_Msk               (0x1ul << FMC_ISPSTS_CBS_Pos)                     /*!< FMC_T::ISPSTS: CBS Mask                */

#define FMC_ISPSTS_ISPFF_Pos             (6)                                               /*!< FMC_T::ISPSTS: ISPFF Position          */
#define FMC_ISPSTS_ISPFF_Msk             (0x1ul << FMC_ISPSTS_ISPFF_Pos)                   /*!< FMC_T::ISPSTS: ISPFF Mask              */

#define FMC_ISPSTS_ALLONE_Pos            (7)                                               /*!< FMC_T::ISPSTS: ALLONE Position         */
#define FMC_ISPSTS_ALLONE_Msk            (0x1ul << FMC_ISPSTS_ALLONE_Pos)                  /*!< FMC_T::ISPSTS: ALLONE Mask             */

#define FMC_ISPSTS_INTFLAG_Pos           (8)                                               /*!< FMC_T::ISPSTS: INTFLAG Position        */
#define FMC_ISPSTS_INTFLAG_Msk           (0x1ul << FMC_ISPSTS_INTFLAG_Pos)                 /*!< FMC_T::ISPSTS: INTFLAG Mask            */

#define FMC_ISPSTS_VECMAP_Pos            (9)                                               /*!< FMC_T::ISPSTS: VECMAP Position         */
#define FMC_ISPSTS_VECMAP_Msk            (0x7fffful << FMC_ISPSTS_VECMAP_Pos)              /*!< FMC_T::ISPSTS: VECMAP Mask             */

#define FMC_ISPSTS_FBS_Pos               (30)                                              /*!< FMC_T::ISPSTS: FBS Position            */
#define FMC_ISPSTS_FBS_Msk               (0x1ul << FMC_ISPSTS_FBS_Pos)                     /*!< FMC_T::ISPSTS: FBS Mask                */

#define FMC_ISPSTS_SCODE_Pos             (31)                                              /*!< FMC_T::ISPSTS: SCODE Position          */
#define FMC_ISPSTS_SCODE_Msk             (0x1ul << FMC_ISPSTS_SCODE_Pos)                   /*!< FMC_T::ISPSTS: SCODE Mask              */

#define FMC_CYCCTL_CYCLE_Pos             (0)                                               /*!< FMC_T::CYCCTL: CYCLE Position          */
#define FMC_CYCCTL_CYCLE_Msk             (0xful << FMC_CYCCTL_CYCLE_Pos)                   /*!< FMC_T::CYCCTL: CYCLE Mask              */

#define FMC_MPDAT0_ISPDAT0_Pos           (0)                                               /*!< FMC_T::MPDAT0: ISPDAT0 Position        */
#define FMC_MPDAT0_ISPDAT0_Msk           (0xfffffffful << FMC_MPDAT0_ISPDAT0_Pos)          /*!< FMC_T::MPDAT0: ISPDAT0 Mask            */

#define FMC_MPDAT1_ISPDAT1_Pos           (0)                                               /*!< FMC_T::MPDAT1: ISPDAT1 Position        */
#define FMC_MPDAT1_ISPDAT1_Msk           (0xfffffffful << FMC_MPDAT1_ISPDAT1_Pos)          /*!< FMC_T::MPDAT1: ISPDAT1 Mask            */

#define FMC_MPDAT2_ISPDAT2_Pos           (0)                                               /*!< FMC_T::MPDAT2: ISPDAT2 Position        */
#define FMC_MPDAT2_ISPDAT2_Msk           (0xfful << FMC_MPDAT2_ISPDAT2_Pos)                /*!< FMC_T::MPDAT2: ISPDAT2 Mask            */

#define FMC_ECCCTL_SEBDINTEN_Pos         (0)                                               /*!< FMC_T::ECCCTL: SEBDINTEN Position      */
#define FMC_ECCCTL_SEBDINTEN_Msk         (0x1ul << FMC_ECCCTL_SEBDINTEN_Pos)               /*!< FMC_T::ECCCTL: SEBDINTEN Mask          */

#define FMC_ECCCTL_DEBDINTEN_Pos         (1)                                               /*!< FMC_T::ECCCTL: DEBDINTEN Position      */
#define FMC_ECCCTL_DEBDINTEN_Msk         (0x1ul << FMC_ECCCTL_DEBDINTEN_Pos)               /*!< FMC_T::ECCCTL: DEBDINTEN Mask          */


#define FMC_ECCSTS_ECCSEBCF_Pos          (0)                                               /*!< FMC_T::ECCSTS: ECCSEBCF Position       */
#define FMC_ECCSTS_ECCSEBCF_Msk          (0x1ul << FMC_ECCSTS_ECCSEBCF_Pos)                /*!< FMC_T::ECCSTS: ECCSEBCF Mask           */

#define FMC_ECCSTS_ECCDEBDF_Pos          (1)                                               /*!< FMC_T::ECCSTS: ECCDEBDF Position       */
#define FMC_ECCSTS_ECCDEBDF_Msk          (0x1ul << FMC_ECCSTS_ECCDEBDF_Pos)                /*!< FMC_T::ECCSTS: ECCDEBDF Mask           */

#define FMC_ECCSEFAR_ECCSEFAR_Pos        (0)                                               /*!< FMC_T::ECCSEFAR: ECCSEFAR Position     */
#define FMC_ECCSEFAR_ECCSEFAR_Msk        (0xfffffffful << FMC_ECCSEFAR_ECCSEFAR_Pos)       /*!< FMC_T::ECCSEFAR: ECCSEFAR Mask         */

#define FMC_ECCDEFAR_ECCDEFAR_Pos        (0)                                               /*!< FMC_T::ECCDEFAR: ECCDEFAR Position     */
#define FMC_ECCDEFAR_ECCDEFAR_Msk        (0xfffffffful << FMC_ECCDEFAR_ECCDEFAR_Pos)       /*!< FMC_T::ECCDEFAR: ECCDEFAR Mask         */


/**@}*/ /* FMC_CONST */
/**@}*/ /* end of FMC register group */
/**@}*/ /* end of REGISTER group */

#endif /* __FMC_REG_H__ */
