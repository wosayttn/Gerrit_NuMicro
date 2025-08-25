
/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

/** @addtogroup REGISTER Control Register

  @{

*/


/*-------------------- Data Flash Memory Controller ----------------------*/
/**
    @addtogroup DFMC Data Flash Memory Controller (DFMC)
    Memory Mapped Structure for DFMC Controller
@{ */

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
 * |        |          |l Data Flash writes to itself if DATAEN is set to 0.
 * |        |          |l Erase or Program command at brown-out detected
 * |        |          |l Destination address is illegal, such as over an available range.
 * |        |          |l Invalid ISP commands
 * |        |          |l Violate the load code read protection
 * |        |          |l Checksum or Flash All One Verification is not executed in their valid range
 * |        |          |l Mass erase is not executed in Data Flash
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
 * |        |          |The M3351471 is equipped with embedded Data Flash
 * |        |          |ISPADDR[1:0] must be kept 00 for ISP 32-bit operation.
 * |        |          |For CRC32 Checksum Calculation command, this field is the Data Flash starting address for checksum calculation, 256 bytes alignment is necessary for CRC32 checksum calculation.
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
 * |        |          |For Run CRC32 Checksum Calculation command, ISPDAT is the memory size (byte) and 256 bytes alignment
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
 * |        |          |0x08= Read Data Flash All-One Result.
 * |        |          |0x09= Data FLASH Read (Program Verify).
 * |        |          |0x0A= Data FLASH Read (Erase Verify).
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
 * @var DFMC_T::ICPCTL
 * Offset: 0x1C  ICP Enabled Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ICPEN     |Data Flash ICP Enabled Control (Write Protect)
 * |        |          |0 =Data Flash ICP mode Disabled(default).
 * |        |          |1 = Data Flash ICP mode Enabled.
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
 * |        |          |l Data Flash writes to itself if DATAEN is set to 0.
 * |        |          |l Erase or Program command at brown-out detected
 * |        |          |l Destination address is illegal, such as over an available range.
 * |        |          |l Invalid ISP commands
 * |        |          |l Violate the load code read protection
 * |        |          |l Checksum or Flash All One Verification is not executed in their valid range
 * |        |          |l Mass erase is not executed in Data Flash
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[7]     |ALLONE    |Data Flash All-one Verification Flag
 * |        |          |This bit is set by hardware if all of Flash bits are 1, and clear if Flash bits are not all 1 after u201CRun Data Flash All-One Verificationu201D complete; this bit also can be clear by writing 1
 * |        |          |0 = Data Flash bits are not all 1 after u201CRun Data Flash All-One Verificationu201D is complete.
 * |        |          |1 = All of Data Flash bits are 1 after u201CRun Data Flash All-One Verificationu201D is complete.
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
 * |        |          |0000 = CPU access with zero wait cycle ; Flash access cycle is 1;.
 * |        |          |The HCLK working frequency range is <27 MHz; Cache is disabled by hardware.
 * |        |          |0001 = CPU access with one wait cycle if cache miss; Flash access cycle is 1;.
 * |        |          |The HCLK working frequency range range is<27 MHz
 * |        |          |0010 = CPU access with two wait cycles if cache miss; Flash access cycle is 2;.
 * |        |          |The optimized HCLK working frequency range is 27~54 MHz
 * |        |          |0011 = CPU access with three wait cycles if cache miss; Flash access cycle is 3;.
 * |        |          |The optimized HCLK working frequency range is 54~81 MHz
 * |        |          |0100 = CPU access with four wait cycles if cache miss; Flash access cycle is 4;.
 * |        |          |The optimized HCLK working frequency range is81~108 MHz
 * |        |          |0101 = CPU access with five wait cycles if cache miss; Flash access cycle is 5;.
 * |        |          |The optimized HCLK working frequency range is 108~135 MHz
 * |        |          |0110 = CPU access with six wait cycles if cache miss; Flash access cycle is 6;.
 * |        |          |The optimized HCLK working frequency range is 135~162 MHz
 * |        |          |0111 = CPU access with seven wait cycles if cache miss; Flash access cycle is 7;.
 * |        |          |The optimized HCLK working frequency range is 162~192 MHz
 * |        |          |1000 = CPU access with eight wait cycles if cache miss; Flash access cycle is 8;.
 * |        |          |The optimized HCLK working frequency range is >192 MHz
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * @var DFMC_T::MPDAT2
 * Offset: 0x88  ISP Data2 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |ISPDAT2   |ISP Data 2
 * |        |          |Write data to this register before ISP 64-bit program(0x41) operation.
 * |        |          |Read data from this register after ISP 32-bit read operation or ECCSEBCF(FMC_ECCSTS[0]) is high can know which bit is error
 * |        |          |ISPDAT2 does not continuously store the position information of ECC errors
 * |        |          |Therefore, if the user wants to know which bit generated the error, they need to read it before the next ISP action after an ECC single-bit error occurs, or access that position again.
 * |        |          |When ISPFF (FMC_ISPCTL[6]) is 1, ISPDAT2 = 0xff.
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
 * |[31]    |BEDIS     |Bus Error Disable
 * |        |          |1 = Bus Error is disable.
 * |        |          |0 = Bus Error is enable.
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * @var DFMC_T::ECCSTS
 * Offset: 0x134  DFMC Error Code Correction status
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
 * @var DFMC_T::ECCSEFAR
 * Offset: 0x138  DFMC ECC Single Error fault Adress Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |ECCSEFAR  |ECC Single Error Fault Address Register (Read Only)
 * |        |          |When FMC detects a ECCSEBCF(FMC_ECCSTS[0]) would record fault address at the register
 * |        |          |The register is cleared by writing 1 to ECCSEBCF
 * |        |          |ECCSEFAR is 64-bit alignment so ECCSEFAR[2:0] are always 0.
 * @var DFMC_T::ECCDEFAR
 * Offset: 0x13C  DFMC ECC Double Error fault Adress Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |ECCDEFAR  |ECC Double Error Fault Address Register (Read Only)
 * |        |          |When FMC detects a ECCDEBDF(FMC_ECCSTS[1]) would record fault address at the register
 * |        |          |The register is cleared by writing 1 to ECCDEBDF
 * |        |          |ECCDEBDF is 64-bit alignment so ECCDEFAR[2:0] are always 0.
 * @var DFMC_T::VERSION
 * Offset: 0xFFC  DFMC Version
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |MINOR     |RTL Design MINOR Version Number
 * |        |          |Minor version number is dependent on moduleu2019s ECO version control.
 * |        |          |0x0000:(Current Minor Version Number)
 * |[23:16] |SUB       |RTL Design SUB Version Number
 * |        |          |Sub version number is correlated to moduleu2019s key feature.
 * |        |          |0x01:(Current Sub Version Number)
 * |[31:24] |MAJOR     |RTL Design MAJOR Version Number
 * |        |          |Major version number is correlated to Product Line
 * |        |          |0x04:(Current Major Version Number)
 */
    __IO uint32_t ISPCTL;                /*!< [0x0000] ISP Control Register                                             */
    __IO uint32_t ISPADDR;               /*!< [0x0004] ISP Address Register                                             */
    __IO uint32_t ISPDAT;                /*!< [0x0008] ISP Data Register                                                */
    __IO uint32_t ISPCMD;                /*!< [0x000c] ISP Command Register                                             */
    __IO uint32_t ISPTRG;                /*!< [0x0010] ISP Trigger Control Register                                     */
    __I  uint32_t RESERVE0[2];
    __IO uint32_t ICPCTL;                /*!< [0x001c] ICP Enabled Control Register                                     */
    __I  uint32_t RESERVE1[8];
    __IO uint32_t ISPSTS;                /*!< [0x0040] ISP Status Register                                              */
    __I  uint32_t RESERVE2[2];
    __IO uint32_t CYCCTL;                /*!< [0x004c] Data Flash Access Cycle Control Register                         */
    __I  uint32_t RESERVE3[14];
    __IO uint32_t MPDAT2;                /*!< [0x0088] ISP Data2 Register                                               */
    __I  uint32_t RESERVE4[41];
    __IO uint32_t ECCCTL;                /*!< [0x0130] DFMC Error Code Correction Control                               */
    __IO uint32_t ECCSTS;                /*!< [0x0134] DFMC Error Code Correction status                                */
    __I  uint32_t ECCSEFAR;              /*!< [0x0138] DFMC ECC Single Error fault Adress Register                      */
    __I  uint32_t ECCDEFAR;              /*!< [0x013c] DFMC ECC Double Error fault Adress Register                      */
    __I  uint32_t EEP_STS;               /*!< [0x0140] DFMC EEPROM emulation status register                            */
    __I  uint32_t RESERVE5[942];
    __I  uint32_t VERSION;               /*!< [0x0ffc] DFMC Version                                                     */

} DFMC_T;

/**
    @addtogroup DFMC_CONST DFMC Bit Field Definition
    Constant Definitions for DFMC Controller
@{ */

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

#define DFMC_ICPCTL_ICPEN_Pos            (0)                                               /*!< DFMC_T::ICPCTL: ICPEN Position         */
#define DFMC_ICPCTL_ICPEN_Msk            (0x1ul << DFMC_ICPCTL_ICPEN_Pos)                  /*!< DFMC_T::ICPCTL: ICPEN Mask             */

#define DFMC_ISPSTS_ISPBUSY_Pos          (0)                                               /*!< DFMC_T::ISPSTS: ISPBUSY Position       */
#define DFMC_ISPSTS_ISPBUSY_Msk          (0x1ul << DFMC_ISPSTS_ISPBUSY_Pos)                /*!< DFMC_T::ISPSTS: ISPBUSY Mask           */

#define DFMC_ISPSTS_ISPFF_Pos            (6)                                               /*!< DFMC_T::ISPSTS: ISPFF Position         */
#define DFMC_ISPSTS_ISPFF_Msk            (0x1ul << DFMC_ISPSTS_ISPFF_Pos)                  /*!< DFMC_T::ISPSTS: ISPFF Mask             */

#define DFMC_ISPSTS_ALLONE_Pos           (7)                                               /*!< DFMC_T::ISPSTS: ALLONE Position        */
#define DFMC_ISPSTS_ALLONE_Msk           (0x1ul << DFMC_ISPSTS_ALLONE_Pos)                 /*!< DFMC_T::ISPSTS: ALLONE Mask            */

#define DFMC_ISPSTS_EEP_PF_Pos           (8)                                               /*!< DFMC_T::ISPSTS: EEP_PF Position        */
#define DFMC_ISPSTS_EEP_PF_Msk           (0x1ul << DFMC_ISPSTS_EEP_PF_Pos)                 /*!< DFMC_T::ISPSTS: EEP_PF Mask            */

#define DFMC_ISPSTS_EEP_INIT_Pos         (9)                                               /*!< DFMC_T::ISPSTS: EEP_INIT Position      */
#define DFMC_ISPSTS_EEP_INIT_Msk         (0x1ul << DFMC_ISPSTS_EEP_INIT_Pos)               /*!< DFMC_T::ISPSTS: EEP_INIT Mask          */

#define DFMC_ISPSTS_EEP_PE_Pos           (10)                                              /*!< DFMC_T::ISPSTS: EEP_PE Position        */
#define DFMC_ISPSTS_EEP_PE_Msk           (0x1ul << DFMC_ISPSTS_EEP_PE_Pos)                 /*!< DFMC_T::ISPSTS: EEP_PE Mask            */

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

#define DFMC_ECCCTL_BEDIS_Pos            (31)                                              /*!< DFMC_T::ECCCTL: BEDIS Position         */
#define DFMC_ECCCTL_BEDIS_Msk            (0x1ul << DFMC_ECCCTL_BEDIS_Pos)                  /*!< DFMC_T::ECCCTL: BEDIS Mask             */

#define DFMC_ECCSTS_ECCSEBCF_Pos         (0)                                               /*!< DFMC_T::ECCSTS: ECCSEBCF Position      */
#define DFMC_ECCSTS_ECCSEBCF_Msk         (0x1ul << DFMC_ECCSTS_ECCSEBCF_Pos)               /*!< DFMC_T::ECCSTS: ECCSEBCF Mask          */

#define DFMC_ECCSTS_ECCDEBDF_Pos         (1)                                               /*!< DFMC_T::ECCSTS: ECCDEBDF Position      */
#define DFMC_ECCSTS_ECCDEBDF_Msk         (0x1ul << DFMC_ECCSTS_ECCDEBDF_Pos)               /*!< DFMC_T::ECCSTS: ECCDEBDF Mask          */

#define DFMC_ECCSEFAR_ECCSEFAR_Pos       (0)                                               /*!< DFMC_T::ECCSEFAR: ECCSEFAR Position    */
#define DFMC_ECCSEFAR_ECCSEFAR_Msk       (0xfffffffful << DFMC_ECCSEFAR_ECCSEFAR_Pos)      /*!< DFMC_T::ECCSEFAR: ECCSEFAR Mask        */

#define DFMC_ECCDEFAR_ECCDEFAR_Pos       (0)                                               /*!< DFMC_T::ECCDEFAR: ECCDEFAR Position    */
#define DFMC_ECCDEFAR_ECCDEFAR_Msk       (0xfffffffful << DFMC_ECCDEFAR_ECCDEFAR_Pos)      /*!< DFMC_T::ECCDEFAR: ECCDEFAR Mask        */

#define DFMC_VERSION_MINOR_Pos           (0)                                               /*!< DFMC_T::VERSION: MINOR Position        */
#define DFMC_VERSION_MINOR_Msk           (0xfffful << DFMC_VERSION_MINOR_Pos)              /*!< DFMC_T::VERSION: MINOR Mask            */

#define DFMC_VERSION_SUB_Pos             (16)                                              /*!< DFMC_T::VERSION: SUB Position          */
#define DFMC_VERSION_SUB_Msk             (0xfful << DFMC_VERSION_SUB_Pos)                  /*!< DFMC_T::VERSION: SUB Mask              */

#define DFMC_VERSION_MAJOR_Pos           (24)                                              /*!< DFMC_T::VERSION: MAJOR Position        */
#define DFMC_VERSION_MAJOR_Msk           (0xfful << DFMC_VERSION_MAJOR_Pos)                /*!< DFMC_T::VERSION: MAJOR Mask            */

/**@}*/ /* DFMC_CONST */
/**@}*/ /* end of DFMC register group */
/**@}*/ /* end of REGISTER group */
