
/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

/** @addtogroup REGISTER Control Register

  @{

*/


/*---------------------- Flash Memory Controller -------------------------*/
/**
    @addtogroup FMC Flash Memory Controller(FMC)
    Memory Mapped Structure for FMC Controller
@{ */

typedef struct
{


/**
 * @var FMC_T::ISPCTL
 * Offset: 0x00  ISP Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ISPEN     |ISP Enable Bit (Write Protect)
 * |        |          |0 = ISP function Disabled.
 * |        |          |1 = ISP function Enabled.
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[1]     |BS        |Boot Select (Write Protect)
 * |        |          |When MBYTES in CONFIG0 is 1, set/clear this bit to select next booting from LDROM/APROM, respectively
 * |        |          |This bit also functions as chip booting status flag, which can be used to check where chip booted from
 * |        |          |This bit is initiated with the inversed value of CBS[1] (CONFIG0[7]) after any reset is happened except CPU reset (CPU is 1) or system reset (SYS) is happened
 * |        |          |0 = Booting from APROM when MBYTES (CONFIG0[5]) is 1.
 * |        |          |1 = Booting from LDROM when MBYTES (CONFIG0[5]) is 1.
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
 * |        |          |u00B7 APROM writes to itself if APUEN is set to 0.
 * |        |          |u00B7 LDROM writes to itself if LDUEN is set to 0.
 * |        |          |u00B7 CONFIG is erased/programmed if CFGUEN is set to 0.
 * |        |          |u00B7 Page Erase command at LOCK mode with ICE connection
 * |        |          |u00B7 Erase or Program command at brown-out detected
 * |        |          |u00B7 Destination address is illegal, such as over an available range.
 * |        |          |u00B7 Invalid ISP commands
 * |        |          |u00B7 Read any content of boot loader with ICE connection
 * |        |          |u00B7 The address of block erase and bank erase is not in APROM
 * |        |          |u00B7 ISP CMD in XOM region, except mass erase, page erase and chksum command
 * |        |          |u00B7 The wrong setting of page erase ISP CMD in XOM
 * |        |          |u00B7 Violate XOM setting one time protection
 * |        |          |u00B7 Page erase ISP CMD in Secure/Non-secure region setting page
 * |        |          |u00B7 Mass erase when MERASE (CFG0[13]) is disabled
 * |        |          |u00B7 Page erase, mass erase, multi-word program or 64-bit word program in OTP
 * |        |          |u00B7 Read ISP CMD when Load code mode is active
 * |        |          |u00B7 Mass erase when mass erase disable bit (ROMMAP) is disable
 * |        |          |u00B7 ISP conflict error
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[10:8]  |PT        |Flash Program Time (Write Protect)
 * |        |          |To be 000, if Reserved
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[14:12] |ET        |Flash Erase Time (Write Protect)
 * |        |          |To be 000, if Reserved
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[16]    |BL        |Boot Loader Booting (Write Protect)
 * |        |          |This bit is initiated with the inversed value of MBYTES (CONFIG0[5])
 * |        |          |Any reset, except CPU reset (CPU is 1) or system reset (SYS), BL will be reloaded
 * |        |          |This bit is used to check chip boot from Boot Loader or not
 * |        |          |User should keep original value of this bit when updating FMC_ISPCTL register.
 * |        |          |0 = Booting from APROM or LDROM.
 * |        |          |1 = Booting from Boot Loader.
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[24]    |INTEN     |Secure ISP INT Enable Bit (Write Protect)
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
 * |        |          |The M3355 series is equipped with embedded Flash
 * |        |          |ISPADDR[1:0] must be kept 00 for ISP 32-bit operation
 * |        |          |ISPADDR[2:0] must be kept 000 for ISP 64-bit operation.
 * |        |          |For CRC32 Checksum Calculation command, this field is the Flash starting address for checksum calculation, 8 Kbytes alignment is necessary for CRC32 checksum calculation.
 * @var FMC_T::ISPDAT
 * Offset: 0x08  ISP Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |ISPDAT    |ISP Data
 * |        |          |Write data to this register before ISP program operation.
 * |        |          |Read data from this register after ISP read operation.
 * |        |          |When ISPFF (FMC_ISPCTL[6]) is 1, ISPDAT = 0xffff_ffff
 * |        |          |For Run CRC32 Checksum Calculation command, ISPDAT is the memory size (byte) and 8 Kbytes alignment
 * |        |          |For ISP Read CRC32 Checksum command, ISPDAT is the checksum result
 * |        |          |If ISPDAT = 0x0000_0000, it means that (1) the checksum calculation is in progress, or (2) the memory range for checksum calculation is incorrect
 * |        |          |For XOM page erase function, ISPDAT = 0x0055_aa03.
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
 * |        |          |0x09= FLASH Read (Program Verify).
 * |        |          |0x0A= FLASH Read (Erase Verify).
 * |        |          |0x0B= Read Company ID.
 * |        |          |0x0C= Read Device ID.
 * |        |          |0x0D= Read Checksum.
 * |        |          |0x21= FLASH 32-bit Program.
 * |        |          |0x22= FLASH Page Erase. Erase any page in two banks, except for OTP.
 * |        |          |0x23= FLASH Bank Erase. Erase all pages of APROM in BANK0 or BANK1.
 * |        |          |0x24= FLASH CFG word erase.
 * |        |          |0x25= FLASH Block Erase. Erase four pages alignment of APROM in BANK0 or BANK1.
 * |        |          |0x26= FLASH Mass Erase. Erase all pages in two banks, except for OTP and SPROMs.
 * |        |          |0x27= FLASH Multi-Word Program.
 * |        |          |0x28= Run Flash All-One Verification.
 * |        |          |0x2C=Bank REMAP.
 * |        |          |0x2D= Run Checksum Calculation.
 * |        |          |0x2E= Vector Remap.
 * |        |          |0x40= FLASH 64-bit Read.
 * |        |          |0x61= FLASH 64-bit Program.
 * |        |          |The other commands are invalid.
 * @var FMC_T::ISPTRG
 * Offset: 0x10  ISP Trigger Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ISPGO     |ISP Start Trigger (Write Protect)
 * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP operation is finished
 * |        |          |When ISPGO=1,the operation of writing value to address from FMC_BA+0x00 to FMC_BA+0x68 would be ignored.
 * |        |          |0 = ISP operation is finished.
 * |        |          |1 = ISP is progressed.
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * @var FMC_T::FTCTL
 * Offset: 0x18  Flash Access Time Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var FMC_T::ICPCTL
 * Offset: 0x1C  ICP Enabled Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ICPEN     |Flash ICP Enabled Control (Write Protect)
 * |        |          |0 = Flash ICP mode Disabled(default).
 * |        |          |1 = Flash ICP mode Enabled.
 * |        |          |When FMC_ICPCTL[1] enable,user set ICPEN from 1 to 0.CHIP would reset,when last ISP address is at CONFIG page range.
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[1]     |UNLOAD    |Flash Reload Control (Write Protect)
 * |        |          |0 = Flash Reload Enabled while ICP exit (default).
 * |        |          |1 = Flash Reload Disabled while ICP exit.
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
 * |        |          |The following function is valid when MBYTES (FMC_ISPSTS[3])= 1.
 * |        |          |0 = LDROM with IAP mode.
 * |        |          |1 = APROM with IAP mode.
 * |[3]     |MBS       |Boot From Boot Loader Selection Flag (Read Only)
 * |        |          |This bit is initiated with the MBYTES (CONFIG0[5]) after any reset is happened except CPU reset (CPU is 1) or system reset (SYS) is happened
 * |        |          |0 = Booting from Boot Loader.
 * |        |          |1 = Booting from LDROM/APROM.(.see CBS bit setting)
 * |[4]     |SCFF      |Secure Conceal Fail Flag (Write Protect)
 * |        |          |This bit is set by hardware if any ISP command acceesses secure region when secure conceal function is enable.
 * |        |          |This bit needs to be cleared by writing 1 to it.
 * |        |          |0 = ISP is not accessed secure conceal region.
 * |        |          |1 = ISP is accessed secure conceal region.
 * |[5]     |CFGWEFF   |Config Word Erase fail Flag(Read Only)
 * |        |          |Last time CFG word erase function after initial status
 * |        |          |If CFGWEFF is set to 1, user needs to erase CFG region again.
 * |        |          |0 = Success.
 * |        |          |1 = Fail.
 * |[6]     |ISPFF     |ISP Fail Flag (Write Protect)
 * |        |          |This bit is the mirror of ISPFF (FMC_ISPCTL[6]), it needs to be cleared by writing 1 to FMC_ISPCTL[6] or FMC_ISPSTS[6]
 * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
 * |        |          |u00B7 APROM writes to itself if APUEN is set to 0.
 * |        |          |u00B7 LDROM writes to itself if LDUEN is set to 0.
 * |        |          |u00B7 CONFIG is erased/programmed if CFGUEN is set to 0.
 * |        |          |u00B7 Page Erase command at LOCK mode with ICE connection
 * |        |          |u00B7 Erase or Program command at brown-out detected
 * |        |          |u00B7 Destination address is illegal, such as over an available range.
 * |        |          |u00B7 Invalid ISP commands
 * |        |          |u00B7 Read any content of boot loader with ICE connection
 * |        |          |u00B7 The address of block erase and bank erase is not in APROM
 * |        |          |u00B7 ISP CMD in XOM region, except mass erase, page erase and chksum command
 * |        |          |u00B7 The wrong setting of page erase ISP CMD in XOM
 * |        |          |u00B7 Violate XOM setting one time protection
 * |        |          |u00B7 Page erase ISP CMD in Secure/Non-secure region setting page
 * |        |          |u00B7 Mass erase when MERASE (CFG0[13]) is disabled
 * |        |          |u00B7 Page erase, mass erase, multi-word program or 64-bit word program in OTP
 * |        |          |u00B7 Read ISP CMD when Load code mode is active
 * |        |          |u00B7 Mass erase when mass erase disable bit (ROMMAP) is disable
 * |        |          |u00B7 ISP conflict error
 * |        |          |u00B7 Low voltege do ISP operation
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[7]     |ALLONE    |Flash All-one Verification Flag (Write Protect)
 * |        |          |This bit is set by hardware if all of Flash bits are 1, and cleared if Flash bits are not all 1 after u201CRun Flash All-One Verificationu201D is complete; this bit also can be cleared by writing 1
 * |        |          |0 = Flash bits are not all 1 after u201CRun Flash All-One Verificationu201D complete.
 * |        |          |1 = All of Flash bits are 1 after u201CRun Flash All-One Verificationu201D complete.
 * |[8]     |INTFLAG   |ISP Interrupt Flag (Write Protect)
 * |        |          |This bit shows when FMC occurs interrupt, it needs to be cleared by writing 1 to FMC_ISPSTS[8].
 * |        |          |0 = ISP Not Finished.
 * |        |          |1 = ISP done or ISPFF set.
 * |        |          |Note: This function needs to be enabled by FMC_ISPCTRL[24].
 * |[27:10] |VECMAP    |Vector Page Mapping Address (Read Only)
 * |        |          |All access to 0x0000_0000~0x0000_03FF is remapped to the Flash memory address {VECMAP[17:0], 10u2019h000} ~ {VECMAP[17:0], 10u2019h3FF}
 * |[28]    |ISPCERR   |ISP Conflict Error (Write Protect)
 * |        |          |This bit shows when FMC is doing ISP operation, it needs to be cleared by writing 1 to FMC_ISPSTS[28]
 * |        |          |User cannot access FMC_ISP_ADDR,FMC_ISPDAT,FMC_ISPCMD,FMC_ISPTRG
 * |        |          |It would cause ISPFF.
 * |[29]    |MIRBOUND  |Mirror Boundary (Read Only)
 * |        |          |0 = Mirror Boundary Disabled.
 * |        |          |1 = Mirror Boundary Enabled.
 * |[30]    |FBS       |Flash Bank Selection (Read Only)
 * |        |          |This bit indicate which bank is selected to boot.
 * |        |          |0 = Booting from BANK0.
 * |        |          |1 = Booting from BANK1.
 * @var FMC_T::CYCCTL
 * Offset: 0x4C  Flash Access Cycle Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |CYCLE     |Flash Access Cycle Control (Write Protect)
 * |        |          |This register is updated by software.User needs to check the speed of HCLK and set the cycle >0.
 * |        |          |0001 = CPU access with one wait cycle if cache miss; Flash access cycle is 1;.
 * |        |          |The HCLK working frequency range range is<25 MHz
 * |        |          |0010 = CPU access with two wait cycles if cache miss; Flash access cycle is 2;.
 * |        |          |The optimized HCLK working frequency range is 26~50 MHz
 * |        |          |0011 = CPU access with three wait cycles if cache miss; Flash access cycle is 3;.
 * |        |          |The optimized HCLK working frequency range is 51~75 MHz
 * |        |          |0100 = CPU access with four wait cycles if cache miss; Flash access cycle is 4;.
 * |        |          |The optimized HCLK working frequency range is 76~100 MHz
 * |        |          |0101 = CPU access with five wait cycles if cache miss; Flash access cycle is 5;.
 * |        |          |The optimized HCLK working frequency range is 101~125 MHz
 * |        |          |0110 = CPU access with six wait cycles if cache miss; Flash access cycle is 6;.
 * |        |          |The optimized HCLK working frequency range is 126~150 MHz
 * |        |          |0111 = CPU access with seven wait cycles if cache miss; Flash access cycle is 7;.
 * |        |          |The optimized HCLK working frequency range is 151~175 MHz
 * |        |          |1000 = CPU access with eight wait cycles if cache miss; Flash access cycle is 8;.
 * |        |          |The optimized HCLK working frequency range is 176~200 MHz
 * |        |          |1001 = CPU access with nine wait cycles if cache miss; Flash access cycle is 9;.
 * |        |          |The optimized HCLK working frequency range is 201~225 MHz
 * |        |          |1010 = CPU access with ten wait cycles if cache miss; Flash access cycle is 10;.
 * |        |          |The optimized HCLK working frequency range is 226~240 MHz
 * |        |          |1000 = CPU access with eight wait cycles if cache miss; Flash access cycle is 8;.
 * |        |          |The optimized HCLK working frequency range is >192 MHz
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * @var FMC_T::MPDAT0
 * Offset: 0x80  ISP Data0 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |ISPDAT0   |ISP Data 0
 * |        |          |This register is the first 32-bit data for 32-bit/64-bit/multi-word programming, and it is also the mirror of FMC_ISPDAT, both registers keep the same data
 * @var FMC_T::MPDAT1
 * Offset: 0x84  ISP Data1 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |ISPDAT1   |ISP Data 1
 * |        |          |This register is the second 32-bit data for 64-bit/multi-word programming.
 * @var FMC_T::MPDAT2
 * Offset: 0x88  ISP Data2 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |ISPDAT2   |ISP Data 2
 * |        |          |This register is the third 32-bit data for multi-word programming.
 * @var FMC_T::MPDAT3
 * Offset: 0x8C  ISP Data3 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |ISPDATE0  |ISP Data
 * |        |          |Write data to this register before ISP 64-bit program(0x41) operation.
 * |        |          |Read data from this register after ISP 64-bit read operation or ECCSEBCF(FMC_ECCSTS[0]) is high can know which bit is error
 * |        |          |ISPDATE does not continuously store the position information of ECC errors
 * |        |          |Therefore, if the user wants to know which bit generated the error, they need to read it before the next ISP action after an ECC single-bit error occurs, or access that position again.
 * |        |          |When ISPFF (FMC_ISPCTL[6]) is 1, ISPDATE0 = 0xff.
 * |        |          |This register is the ECC 8-bit data.
 * |[15:8]  |ISPDATE1  |ISP Data
 * |        |          |Write data to this register before ISP 64-bit program(0x41) operation.
 * |        |          |Read data from this register after ISP 64-bit read operation or ECCSEBCF(FMC_ECCSTS[1]) is high can know which bit is error
 * |        |          |ISPDATE does not continuously store the position information of ECC errors
 * |        |          |Therefore, if the user wants to know which bit generated the error, they need to read it before the next ISP action after an ECC single-bit error occurs, or access that position again.
 * |        |          |When ISPFF (FMC_ISPCTL[6]) is 1, ISPEDATE1 = 0xff.
 * |        |          |This register is the ECC 8-bit data.
 * @var FMC_T::MPDATE
 * Offset: 0x90  ISP Data Error Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var FMC_T::XOMR0STS
 * Offset: 0xD0  XOM Region 0 Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |SIZE      |XOM Region 0 Size
 * |        |          |SIZE is the page number of XOM Region 0.
 * |[31:8]  |BASE      |XOM Region 0 Base Address
 * |        |          |BASE is the base address of XOM Region 0.
 * |        |          |Note: The base address is page aligned.
 * @var FMC_T::XOMR1STS
 * Offset: 0xD4  XOM Region 1 Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |SIZE      |XOM Region 1 Size
 * |        |          |SIZE is the page number of XOM Region 1.
 * |[31:8]  |BASE      |XOM Region 1 Base Address
 * |        |          |BASE is the base address of XOM Region 1.
 * |        |          |Note: The base address is page aligned.
 * @var FMC_T::XOMSTS
 * Offset: 0xE0  XOM Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SOTPLOCK  |Secure OTP Lock (Read Only)
 * |        |          |Secure OTP Storage is for provisioning at the manufacturing stage
 * |        |          |This flag indicates that the SOTP Storage is provisioned and immutable.
 * |        |          |0 = Secure OTP is unlock.
 * |        |          |1 = Secure OTP is locked.
 * |[1]     |SOTPREVOKE|Secure OTP Revoke (Read Only)
 * |        |          |The flag indicates the end of the life cycle of SOTP
 * |        |          |All contents in SOTP would never be accessible when the SOTPREVOKE flag is set.
 * |        |          |0 = Secure OTP is not Revoked.
 * |        |          |1 = Secure OTP is Revoked.
 * @var FMC_T::SOTPSTS
 * Offset: 0xE4  Secure OTP Status
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var FMC_T::APWPROT0
 * Offset: 0x110  APROM Write Protect Register0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |APPROEN0  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x00010_0000 + n*(0x8000) to 0x00010_7fff + n*(0x8000)
 * |[1]     |APPROEN1  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x00010_0000 + n*(0x8000) to 0x00010_7fff + n*(0x8000)
 * |[2]     |APPROEN2  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x00010_0000 + n*(0x8000) to 0x00010_7fff + n*(0x8000)
 * |[3]     |APPROEN3  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x00010_0000 + n*(0x8000) to 0x00010_7fff + n*(0x8000)
 * |[4]     |APPROEN4  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x00010_0000 + n*(0x8000) to 0x00010_7fff + n*(0x8000)
 * |[5]     |APPROEN5  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x00010_0000 + n*(0x8000) to 0x00010_7fff + n*(0x8000)
 * |[6]     |APPROEN6  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x00010_0000 + n*(0x8000) to 0x00010_7fff + n*(0x8000)
 * |[7]     |APPROEN7  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x00010_0000 + n*(0x8000) to 0x00010_7fff + n*(0x8000)
 * |[8]     |APPROEN8  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x00010_0000 + n*(0x8000) to 0x00010_7fff + n*(0x8000)
 * |[9]     |APPROEN9  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x00010_0000 + n*(0x8000) to 0x00010_7fff + n*(0x8000)
 * |[10]    |APPROEN10 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x00010_0000 + n*(0x8000) to 0x00010_7fff + n*(0x8000)
 * |[11]    |APPROEN11 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x00010_0000 + n*(0x8000) to 0x00010_7fff + n*(0x8000)
 * |[12]    |APPROEN12 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x00010_0000 + n*(0x8000) to 0x00010_7fff + n*(0x8000)
 * |[13]    |APPROEN13 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x00010_0000 + n*(0x8000) to 0x00010_7fff + n*(0x8000)
 * |[14]    |APPROEN14 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x00010_0000 + n*(0x8000) to 0x00010_7fff + n*(0x8000)
 * |[15]    |APPROEN15 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x00010_0000 + n*(0x8000) to 0x00010_7fff + n*(0x8000)
 * @var FMC_T::APWPROT1
 * Offset: 0x114  APROM Write Protect Register1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |APPROEN16 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x000280_0000 + n*(0x8000) to 0x00820_7fff + n*(0x8000)
 * |[1]     |APPROEN17 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x000280_0000 + n*(0x8000) to 0x00820_7fff + n*(0x8000)
 * |[2]     |APPROEN18 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x000280_0000 + n*(0x8000) to 0x00820_7fff + n*(0x8000)
 * |[3]     |APPROEN19 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x000280_0000 + n*(0x8000) to 0x00820_7fff + n*(0x8000)
 * |[4]     |APPROEN20 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x000280_0000 + n*(0x8000) to 0x00820_7fff + n*(0x8000)
 * |[5]     |APPROEN21 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x000280_0000 + n*(0x8000) to 0x00820_7fff + n*(0x8000)
 * |[6]     |APPROEN22 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x000280_0000 + n*(0x8000) to 0x00820_7fff + n*(0x8000)
 * |[7]     |APPROEN23 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x000280_0000 + n*(0x8000) to 0x00820_7fff + n*(0x8000)
 * |[8]     |APPROEN24 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x000280_0000 + n*(0x8000) to 0x00820_7fff + n*(0x8000)
 * |[9]     |APPROEN25 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x000280_0000 + n*(0x8000) to 0x00820_7fff + n*(0x8000)
 * |[10]    |APPROEN26 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x000280_0000 + n*(0x8000) to 0x00820_7fff + n*(0x8000)
 * |[11]    |APPROEN27 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x000280_0000 + n*(0x8000) to 0x00820_7fff + n*(0x8000)
 * |[12]    |APPROEN28 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x000280_0000 + n*(0x8000) to 0x00820_7fff + n*(0x8000)
 * |[13]    |APPROEN29 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x000280_0000 + n*(0x8000) to 0x00820_7fff + n*(0x8000)
 * |[14]    |APPROEN30 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x000280_0000 + n*(0x8000) to 0x00820_7fff + n*(0x8000)
 * |[15]    |APPROEN31 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x000280_0000 + n*(0x8000) to 0x00820_7fff + n*(0x8000)
 * @var FMC_T::APWPKEEP
 * Offset: 0x118  APROM Write Protect Keep Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |APWPKEEP0 |APROM Write Protect Keep 0
 * |        |          |0x55AA = APWPROT0 register is be locked.
 * |        |          |Others = APWPROT0 register is free.
 * |[31:16] |APWPKEEP1 |APROM Write Protect Keep 1
 * |        |          |0x55AA = APWPROT1 register is be locked.
 * |        |          |Others = APWPROT1 register is free.
 * @var FMC_T::SCACT
 * Offset: 0x11C  APROM Secure Conceal Active Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SCACT     |Secure Conceal Function Active(Write Protect)
 * |        |          |0 = secure conceal function inactive.
 * |        |          |1 = secure conceal function active.
 * |        |          |Note: secure conceal function active will base on this bit and setting of CONFIG6 is all 0.
 * @var FMC_T::LDWPROT
 * Offset: 0x120  LDROM Write Protect Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |LDPROEN0  |LDROM Protect Enable (Write Protect)
 * |        |          |This bit indicates which LDROM region is protected.
 * |        |          |0 = LDROM region n is not protected.
 * |        |          |1 = LDROM region n is protected.
 * |        |          |Note 1: LDROM protect region is 0xF10_0000 + n*(0x4000) to 0xF10_3fff + n*(0x4000).
 * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
 * @var FMC_T::LDWPKEEP
 * Offset: 0x124  LDROM Write Protect Keep Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |LDWPKEEP  |LDROM Write Protect Keep (Write Protect)
 * |        |          |0x55AA = LDWPROT and LDWPKEEP register are locked.
 * |        |          |Others = LDWPROT and LDWPKEEP register are free to modify.
 * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
 * @var FMC_T::DFWPROT
 * Offset: 0x128  Data Flash Write Protect Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |DFPROENn  |Data Flash Protect Enable (Write Protect)
 * |        |          |This bit indicates which Data Flash region is protected.
 * |        |          |0 = Data Flash region n is not protected.
 * |        |          |1 = Data Flash region n is protected.
 * |        |          |Note 1: Data Flash protect region is 0xF20_0000 + n*(0x2000) to 0xF20_1FFF + n*(0x2000).
 * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
 * @var FMC_T::DFWPKEEP
 * Offset: 0x12C  Data Flash Write Protect Keep Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |ECCDEFAR  |ECC Double Error Fault Address Register (Read Only)
 * |        |          |When FMC detects a ECCDEBDF(FMC_ECCSTS[1]) would record fault address at the register
 * |        |          |The register is cleared by writing 1 to ECCDEBDF
 * |        |          |ECCDEBDF is 64-bit alignment so ECCDEFAR[2:0] are always 0.
 * @var FMC_T::ECCCTL
 * Offset: 0x130  FMC ECC Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var FMC_T::ECCSTS
 * Offset: 0x134  FMC ECC Status
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var FMC_T::ECCSEFAR
 * Offset: 0x138  FMC ECC Single Eror Fault Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var FMC_T::ECCDEFAR
 * Offset: 0x13C  FMC ECC Double Eror Fault Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var FMC_T::FUNCASS
 * Offset: 0xFF8  Chip Function Assignment
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |USB       |USB Function Enable Flag (Read Only)
 * |        |          |This bit is load from USB(MAP0[16])
 * |        |          |0 = USB function is disabled.at chip assignment.
 * |        |          |1 = USB function is enabled at chip assignment.
 * |        |          |Note:TC82673/M3355M1 do not have the setting.
 * |[1]     |CAN0      |CAN0 Function Enable Flag (Read Only)
 * |        |          |This bit is load from CAN0(MAP0[5])
 * |        |          |0 = CAN0 function is disabled.at chip assignment.
 * |        |          |1 = CAN0 function is enabled at chip assignment.
 * |        |          |Note:TC82673/M3355M1 do not have the setting.
 * |[2]     |CAN1      |CAN1 Function Enable Flag (Read Only)
 * |        |          |This bit is load from CAN1(MAP0[6])
 * |        |          |0 = CAN1 function is disabled.at chip assignment.
 * |        |          |1 = CAN1 function is enabled at chip assignment.
 * |        |          |Note:TC82673/M33M55M1 do not have the setting.
 * |[7:4]   |ROMSIZE   |The Size or APROM (Read Only)
 * |        |          |0xB = 1024 Kbytes.
 * |        |          |0xC = 2048 Kbytes.
 * |        |          |Others = 2048 Kbytes.
 * |[13:12] |RAMSIZE   |The Size of SRAM
 * |        |          |00 =1344K.
 * |        |          |01 =512K + 320K.
 * |        |          |10 =1024K.
 * |        |          |11 =512K.
 * @var FMC_T::VERSION
 * Offset: 0xFFC  FMC Version
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
    __I  uint32_t RESERVE0[1];
    __IO uint32_t FTCTL;                 /*!< [0x0018] Flash Access Time Control Register                               */
    __IO uint32_t ICPCTL;                /*!< [0x001c] ICP Enabled Control Register                                     */
    __I  uint32_t RESERVE1[8];
    __IO uint32_t ISPSTS;                /*!< [0x0040] ISP Status Register                                              */
    __I  uint32_t RESERVE2[2];
    __IO uint32_t CYCCTL;                /*!< [0x004c] Flash Access Cycle Control Register                              */
    __I  uint32_t RESERVE3[12];
    __IO uint32_t MPDAT0;                /*!< [0x0080] ISP Data0 Register                                               */
    __IO uint32_t MPDAT1;                /*!< [0x0084] ISP Data1 Register                                               */
    __IO uint32_t MPDAT2;                /*!< [0x0088] ISP Data2 Register                                               */
    __IO uint32_t MPDAT3;                /*!< [0x008c] ISP Data3 Register                                               */
    __IO uint32_t MPDATE;                /*!< [0x0090] ISP Data Error Register                                          */
    __I  uint32_t RESERVE4[11];
    __I  uint32_t MPSTS;                 /*!< [0x00c0] ISP Multi-Program Status Register                                */
    __I  uint32_t RESERVE41[3];
    __I  uint32_t XOMR0STS;              /*!< [0x00d0] XOM Region 0 Status Register                                     */
    __I  uint32_t XOMR1STS;              /*!< [0x00d4] XOM Region 1 Status Register                                     */
    __I  uint32_t RESERVE5[2];
    __I  uint32_t XOMSTS;                /*!< [0x00e0] XOM Status Register                                              */
    __IO uint32_t SOTPSTS;               /*!< [0x00e4] Secure OTP Status                                                */
    __I  uint32_t RESERVE6[10];
    __IO uint32_t APWPROT[2];            /*!< [0x0110-0x114] APROM Write Protect Register0/1                            */
    __IO uint32_t APWPKEEP;              /*!< [0x0118] APROM Write Protect Keep Register                                */
    __IO uint32_t SCACT;                 /*!< [0x011c] APROM Secure Conceal Active Register                             */
    __IO uint32_t LDWPROT;               /*!< [0x0120] LDROM Write Protect Register                                     */
    __IO uint32_t LDWPKEEP;              /*!< [0x0124] LDROM Write Protect Keep Register                                */
    __IO uint32_t DFWPROT;               /*!< [0x0128] Data Flash Write Protect Register                                */
    __IO uint32_t DFWPKEEP;              /*!< [0x012c] Data Flash Write Protect Keep Register                           */
    __IO uint32_t ECCCTL;                /*!< [0x0130] FMC ECC Control                                                  */
    __IO uint32_t ECCSTS;                /*!< [0x0134] FMC ECC Status                                                   */
    __IO uint32_t ECCSEFAR;              /*!< [0x0138] FMC ECC Single Eror Fault Address Register                       */
    __IO uint32_t ECCDEFAR;              /*!< [0x013c] FMC ECC Double Eror Fault Address Register                       */
    __I  uint32_t RESERVE7[942];
    __I  uint32_t FUNCASS;               /*!< [0x0ff8] Chip Function Assignment                                         */
    __I  uint32_t VERSION;               /*!< [0x0ffc] FMC Version                                                      */

} FMC_T;

/**
    @addtogroup FMC_CONST FMC Bit Field Definition
    Constant Definitions for FMC Controller
@{ */

#define FMC_ISPCTL_ISPEN_Pos             (0)                                               /*!< FMC_T::ISPCTL: ISPEN Position          */
#define FMC_ISPCTL_ISPEN_Msk             (0x1ul << FMC_ISPCTL_ISPEN_Pos)                   /*!< FMC_T::ISPCTL: ISPEN Mask              */

#define FMC_ISPCTL_BS_Pos                (1)                                               /*!< FMC_T::ISPCTL: BS Position             */
#define FMC_ISPCTL_BS_Msk                (0x1ul << FMC_ISPCTL_BS_Pos)                      /*!< FMC_T::ISPCTL: BS Mask                 */

#define FMC_ISPCTL_APUEN_Pos             (3)                                               /*!< FMC_T::ISPCTL: APUEN Position          */
#define FMC_ISPCTL_APUEN_Msk             (0x1ul << FMC_ISPCTL_APUEN_Pos)                   /*!< FMC_T::ISPCTL: APUEN Mask              */

#define FMC_ISPCTL_CFGUEN_Pos            (4)                                               /*!< FMC_T::ISPCTL: CFGUEN Position         */
#define FMC_ISPCTL_CFGUEN_Msk            (0x1ul << FMC_ISPCTL_CFGUEN_Pos)                  /*!< FMC_T::ISPCTL: CFGUEN Mask             */

#define FMC_ISPCTL_LDUEN_Pos             (5)                                               /*!< FMC_T::ISPCTL: LDUEN Position          */
#define FMC_ISPCTL_LDUEN_Msk             (0x1ul << FMC_ISPCTL_LDUEN_Pos)                   /*!< FMC_T::ISPCTL: LDUEN Mask              */

#define FMC_ISPCTL_ISPFF_Pos             (6)                                               /*!< FMC_T::ISPCTL: ISPFF Position          */
#define FMC_ISPCTL_ISPFF_Msk             (0x1ul << FMC_ISPCTL_ISPFF_Pos)                   /*!< FMC_T::ISPCTL: ISPFF Mask              */

#define FMC_ISPCTL_DFUEN_Pos             (7)                                               /*!< FMC_T::ISPCTL: DFUEN Position          */
#define FMC_ISPCTL_DFUEN_Msk             (0x1ul << FMC_ISPCTL_DFUEN_Pos)                   /*!< FMC_T::ISPCTL: DFUEN Mask              */

#define FMC_ISPCTL_PT_Pos                (8)                                               /*!< FMC_T::ISPCTL: PT Position             */
#define FMC_ISPCTL_PT_Msk                (0x7ul << FMC_ISPCTL_PT_Pos)                      /*!< FMC_T::ISPCTL: PT Mask                 */

#define FMC_ISPCTL_ET_Pos                (12)                                              /*!< FMC_T::ISPCTL: ET Position             */
#define FMC_ISPCTL_ET_Msk                (0x7ul << FMC_ISPCTL_ET_Pos)                      /*!< FMC_T::ISPCTL: ET Mask                 */

#define FMC_ISPCTL_BL_Pos                (16)                                              /*!< FMC_T::ISPCTL: BL Position             */
#define FMC_ISPCTL_BL_Msk                (0x1ul << FMC_ISPCTL_BL_Pos)                      /*!< FMC_T::ISPCTL: BL Mask                 */

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

#define FMC_ICPCTL_ICPEN_Pos             (0)                                               /*!< FMC_T::ICPCTL: ICPEN Position          */
#define FMC_ICPCTL_ICPEN_Msk             (0x1ul << FMC_ICPCTL_ICPEN_Pos)                   /*!< FMC_T::ICPCTL: ICPEN Mask              */

#define FMC_ICPCTL_UNLOAD_Pos            (1)                                               /*!< FMC_T::ICPCTL: UNLOAD Position         */
#define FMC_ICPCTL_UNLOAD_Msk            (0x1ul << FMC_ICPCTL_UNLOAD_Pos)                  /*!< FMC_T::ICPCTL: UNLOAD Mask             */

#define FMC_ISPSTS_ISPBUSY_Pos           (0)                                               /*!< FMC_T::ISPSTS: ISPBUSY Position        */
#define FMC_ISPSTS_ISPBUSY_Msk           (0x1ul << FMC_ISPSTS_ISPBUSY_Pos)                 /*!< FMC_T::ISPSTS: ISPBUSY Mask            */

#define FMC_ISPSTS_CBS_Pos               (2)                                               /*!< FMC_T::ISPSTS: CBS Position            */
#define FMC_ISPSTS_CBS_Msk               (0x1ul << FMC_ISPSTS_CBS_Pos)                     /*!< FMC_T::ISPSTS: CBS Mask                */

#define FMC_ISPSTS_MBS_Pos               (3)                                               /*!< FMC_T::ISPSTS: MBS Position            */
#define FMC_ISPSTS_MBS_Msk               (0x1ul << FMC_ISPSTS_MBS_Pos)                     /*!< FMC_T::ISPSTS: MBS Mask                */

#define FMC_ISPSTS_SCFF_Pos              (4)                                               /*!< FMC_T::ISPSTS: SCFF Position           */
#define FMC_ISPSTS_SCFF_Msk              (0x1ul << FMC_ISPSTS_SCFF_Pos)                    /*!< FMC_T::ISPSTS: SCFF Mask               */

#define FMC_ISPSTS_CFGWEFF_Pos           (5)                                               /*!< FMC_T::ISPSTS: CFGWEFF Position        */
#define FMC_ISPSTS_CFGWEFF_Msk           (0x1ul << FMC_ISPSTS_CFGWEFF_Pos)                 /*!< FMC_T::ISPSTS: CFGWEFF Mask            */

#define FMC_ISPSTS_ISPFF_Pos             (6)                                               /*!< FMC_T::ISPSTS: ISPFF Position          */
#define FMC_ISPSTS_ISPFF_Msk             (0x1ul << FMC_ISPSTS_ISPFF_Pos)                   /*!< FMC_T::ISPSTS: ISPFF Mask              */

#define FMC_ISPSTS_ALLONE_Pos            (7)                                               /*!< FMC_T::ISPSTS: ALLONE Position         */
#define FMC_ISPSTS_ALLONE_Msk            (0x1ul << FMC_ISPSTS_ALLONE_Pos)                  /*!< FMC_T::ISPSTS: ALLONE Mask             */

#define FMC_ISPSTS_INTFLAG_Pos           (8)                                               /*!< FMC_T::ISPSTS: INTFLAG Position        */
#define FMC_ISPSTS_INTFLAG_Msk           (0x1ul << FMC_ISPSTS_INTFLAG_Pos)                 /*!< FMC_T::ISPSTS: INTFLAG Mask            */

#define FMC_ISPSTS_VECMAP_Pos            (10)                                              /*!< FMC_T::ISPSTS: VECMAP Position         */
#define FMC_ISPSTS_VECMAP_Msk            (0x3fffful << FMC_ISPSTS_VECMAP_Pos)              /*!< FMC_T::ISPSTS: VECMAP Mask             */

#define FMC_ISPSTS_ISPCERR_Pos           (28)                                              /*!< FMC_T::ISPSTS: ISPCERR Position        */
#define FMC_ISPSTS_ISPCERR_Msk           (0x1ul << FMC_ISPSTS_ISPCERR_Pos)                 /*!< FMC_T::ISPSTS: ISPCERR Mask            */

#define FMC_ISPSTS_MIRBOUND_Pos          (29)                                              /*!< FMC_T::ISPSTS: MIRBOUND Position       */
#define FMC_ISPSTS_MIRBOUND_Msk          (0x1ul << FMC_ISPSTS_MIRBOUND_Pos)                /*!< FMC_T::ISPSTS: MIRBOUND Mask           */

#define FMC_ISPSTS_FBS_Pos               (30)                                              /*!< FMC_T::ISPSTS: FBS Position            */
#define FMC_ISPSTS_FBS_Msk               (0x1ul << FMC_ISPSTS_FBS_Pos)                     /*!< FMC_T::ISPSTS: FBS Mask                */

#define FMC_CYCCTL_CYCLE_Pos             (0)                                               /*!< FMC_T::CYCCTL: CYCLE Position          */
#define FMC_CYCCTL_CYCLE_Msk             (0xful << FMC_CYCCTL_CYCLE_Pos)                   /*!< FMC_T::CYCCTL: CYCLE Mask              */

#define FMC_MPDAT0_ISPDAT0_Pos           (0)                                               /*!< FMC_T::MPDAT0: ISPDAT0 Position        */
#define FMC_MPDAT0_ISPDAT0_Msk           (0xfffffffful << FMC_MPDAT0_ISPDAT0_Pos)          /*!< FMC_T::MPDAT0: ISPDAT0 Mask            */

#define FMC_MPDAT1_ISPDAT1_Pos           (0)                                               /*!< FMC_T::MPDAT1: ISPDAT1 Position        */
#define FMC_MPDAT1_ISPDAT1_Msk           (0xfffffffful << FMC_MPDAT1_ISPDAT1_Pos)          /*!< FMC_T::MPDAT1: ISPDAT1 Mask            */

#define FMC_MPDAT2_ISPDAT2_Pos           (0)                                               /*!< FMC_T::MPDAT2: ISPDAT2 Position        */
#define FMC_MPDAT2_ISPDAT2_Msk           (0xfffffffful << FMC_MPDAT2_ISPDAT2_Pos)          /*!< FMC_T::MPDAT2: ISPDAT2 Mask            */

#define FMC_MPDAT3_ISPDATE0_Pos          (0)                                               /*!< FMC_T::MPDAT3: ISPDATE0 Position       */
#define FMC_MPDAT3_ISPDATE0_Msk          (0xfful << FMC_MPDAT3_ISPDATE0_Pos)               /*!< FMC_T::MPDAT3: ISPDATE0 Mask           */

#define FMC_MPDAT3_ISPDATE1_Pos          (8)                                               /*!< FMC_T::MPDAT3: ISPDATE1 Position       */
#define FMC_MPDAT3_ISPDATE1_Msk          (0xfful << FMC_MPDAT3_ISPDATE1_Pos)               /*!< FMC_T::MPDAT3: ISPDATE1 Mask           */

#define FMC_MPDAT2_ISPDAT2_Pos           (0)                                               /*!< FMC_T::MPDAT2: ISPDAT2 Position        */
#define FMC_MPDAT2_ISPDAT2_Msk           (0xfffffffful << FMC_MPDAT2_ISPDAT2_Pos)          /*!< FMC_T::MPDAT2: ISPDAT2 Mask            */

#define FMC_MPDAT3_ISPDAT3_Pos           (0)                                               /*!< FMC_T::MPDAT3: ISPDAT3 Position        */
#define FMC_MPDAT3_ISPDAT3_Msk           (0xfffffffful << FMC_MPDAT3_ISPDAT3_Pos)          /*!< FMC_T::MPDAT3: ISPDAT3 Mask            */

#define FMC_MPSTS_MPBUSY_Pos             (0)                                               /*!< FMC_T::MPSTS: MPBUSY Position          */
#define FMC_MPSTS_MPBUSY_Msk             (0x1ul << FMC_MPSTS_MPBUSY_Pos)                   /*!< FMC_T::MPSTS: MPBUSY Mask              */

#define FMC_MPSTS_PPGO_Pos               (1)                                               /*!< FMC_T::MPSTS: PPGO Position            */
#define FMC_MPSTS_PPGO_Msk               (0x1ul << FMC_MPSTS_PPGO_Pos)                     /*!< FMC_T::MPSTS: PPGO Mask                */

#define FMC_MPSTS_ISPFF_Pos              (2)                                               /*!< FMC_T::MPSTS: ISPFF Position           */
#define FMC_MPSTS_ISPFF_Msk              (0x1ul << FMC_MPSTS_ISPFF_Pos)                    /*!< FMC_T::MPSTS: ISPFF Mask               */

#define FMC_MPSTS_D0_Pos                 (4)                                               /*!< FMC_T::MPSTS: D0 Position              */
#define FMC_MPSTS_D0_Msk                 (0x1ul << FMC_MPSTS_D0_Pos)                       /*!< FMC_T::MPSTS: D0 Mask                  */

#define FMC_MPSTS_D1_Pos                 (5)                                               /*!< FMC_T::MPSTS: D1 Position              */
#define FMC_MPSTS_D1_Msk                 (0x1ul << FMC_MPSTS_D1_Pos)                       /*!< FMC_T::MPSTS: D1 Mask                  */

#define FMC_MPSTS_D2_Pos                 (6)                                               /*!< FMC_T::MPSTS: D2 Position              */
#define FMC_MPSTS_D2_Msk                 (0x1ul << FMC_MPSTS_D2_Pos)                       /*!< FMC_T::MPSTS: D2 Mask                  */

#define FMC_MPSTS_D3_Pos                 (7)                                               /*!< FMC_T::MPSTS: D3 Position              */
#define FMC_MPSTS_D3_Msk                 (0x1ul << FMC_MPSTS_D3_Pos)                       /*!< FMC_T::MPSTS: D3 Mask                  */

#define FMC_XOMR0STS_SIZE_Pos            (0)                                               /*!< FMC_T::XOMR0STS: SIZE Position         */
#define FMC_XOMR0STS_SIZE_Msk            (0xfful << FMC_XOMR0STS_SIZE_Pos)                 /*!< FMC_T::XOMR0STS: SIZE Mask             */

#define FMC_XOMR0STS_BASE_Pos            (8)                                               /*!< FMC_T::XOMR0STS: BASE Position         */
#define FMC_XOMR0STS_BASE_Msk            (0xfffffful << FMC_XOMR0STS_BASE_Pos)             /*!< FMC_T::XOMR0STS: BASE Mask             */

#define FMC_XOMR1STS_SIZE_Pos            (0)                                               /*!< FMC_T::XOMR1STS: SIZE Position         */
#define FMC_XOMR1STS_SIZE_Msk            (0xfful << FMC_XOMR1STS_SIZE_Pos)                 /*!< FMC_T::XOMR1STS: SIZE Mask             */

#define FMC_XOMR1STS_BASE_Pos            (8)                                               /*!< FMC_T::XOMR1STS: BASE Position         */
#define FMC_XOMR1STS_BASE_Msk            (0xfffffful << FMC_XOMR1STS_BASE_Pos)             /*!< FMC_T::XOMR1STS: BASE Mask             */

#define FMC_XOMSTS_XOMR0ON_Pos           (0)                                               /*!< FMC_T::XOMSTS: XOMR0ON Position        */
#define FMC_XOMSTS_XOMR0ON_Msk           (0x1ul << FMC_XOMSTS_XOMR0ON_Pos)                 /*!< FMC_T::XOMSTS: XOMR0ON Mask            */

#define FMC_XOMSTS_XOMR1ON_Pos           (1)                                               /*!< FMC_T::XOMSTS: XOMR1ON Position        */
#define FMC_XOMSTS_XOMR1ON_Msk           (0x1ul << FMC_XOMSTS_XOMR1ON_Pos)                 /*!< FMC_T::XOMSTS: XOMR1ON Mask            */

#define FMC_XOMSTS_XOMPEF_Pos            (4)                                               /*!< FMC_T::XOMSTS: XOMPEF Position         */
#define FMC_XOMSTS_XOMPEF_Msk            (0x1ul << FMC_XOMSTS_XOMPEF_Pos)                  /*!< FMC_T::XOMSTS: XOMPEF Mask             */

#define FMC_SOTPSTS_SOTPLOCK_Pos         (0)                                               /*!< FMC_T::SOTPSTS: SOTPLOCK Position      */
#define FMC_SOTPSTS_SOTPLOCK_Msk         (0x1ul << FMC_SOTPSTS_SOTPLOCK_Pos)               /*!< FMC_T::SOTPSTS: SOTPLOCK Mask          */

#define FMC_SOTPSTS_SOTPREVOKE_Pos       (1)                                               /*!< FMC_T::SOTPSTS: SOTPREVOKE Position    */
#define FMC_SOTPSTS_SOTPREVOKE_Msk       (0x1ul << FMC_SOTPSTS_SOTPREVOKE_Pos)             /*!< FMC_T::SOTPSTS: SOTPREVOKE Mask        */

#define FMC_APWPROT0_APPROEN0_Pos        (0)                                               /*!< FMC_T::APWPROT0: APPROEN0 Position     */
#define FMC_APWPROT0_APPROEN0_Msk        (0x1ul << FMC_APWPROT0_APPROEN0_Pos)              /*!< FMC_T::APWPROT0: APPROEN0 Mask         */

#define FMC_APWPROT0_APPROEN1_Pos        (1)                                               /*!< FMC_T::APWPROT0: APPROEN1 Position     */
#define FMC_APWPROT0_APPROEN1_Msk        (0x1ul << FMC_APWPROT0_APPROEN1_Pos)              /*!< FMC_T::APWPROT0: APPROEN1 Mask         */

#define FMC_APWPROT0_APPROEN2_Pos        (2)                                               /*!< FMC_T::APWPROT0: APPROEN2 Position     */
#define FMC_APWPROT0_APPROEN2_Msk        (0x1ul << FMC_APWPROT0_APPROEN2_Pos)              /*!< FMC_T::APWPROT0: APPROEN2 Mask         */

#define FMC_APWPROT0_APPROEN3_Pos        (3)                                               /*!< FMC_T::APWPROT0: APPROEN3 Position     */
#define FMC_APWPROT0_APPROEN3_Msk        (0x1ul << FMC_APWPROT0_APPROEN3_Pos)              /*!< FMC_T::APWPROT0: APPROEN3 Mask         */

#define FMC_APWPROT0_APPROEN4_Pos        (4)                                               /*!< FMC_T::APWPROT0: APPROEN4 Position     */
#define FMC_APWPROT0_APPROEN4_Msk        (0x1ul << FMC_APWPROT0_APPROEN4_Pos)              /*!< FMC_T::APWPROT0: APPROEN4 Mask         */

#define FMC_APWPROT0_APPROEN5_Pos        (5)                                               /*!< FMC_T::APWPROT0: APPROEN5 Position     */
#define FMC_APWPROT0_APPROEN5_Msk        (0x1ul << FMC_APWPROT0_APPROEN5_Pos)              /*!< FMC_T::APWPROT0: APPROEN5 Mask         */

#define FMC_APWPROT0_APPROEN6_Pos        (6)                                               /*!< FMC_T::APWPROT0: APPROEN6 Position     */
#define FMC_APWPROT0_APPROEN6_Msk        (0x1ul << FMC_APWPROT0_APPROEN6_Pos)              /*!< FMC_T::APWPROT0: APPROEN6 Mask         */

#define FMC_APWPROT0_APPROEN7_Pos        (7)                                               /*!< FMC_T::APWPROT0: APPROEN7 Position     */
#define FMC_APWPROT0_APPROEN7_Msk        (0x1ul << FMC_APWPROT0_APPROEN7_Pos)              /*!< FMC_T::APWPROT0: APPROEN7 Mask         */

#define FMC_APWPROT0_APPROEN8_Pos        (8)                                               /*!< FMC_T::APWPROT0: APPROEN8 Position     */
#define FMC_APWPROT0_APPROEN8_Msk        (0x1ul << FMC_APWPROT0_APPROEN8_Pos)              /*!< FMC_T::APWPROT0: APPROEN8 Mask         */

#define FMC_APWPROT0_APPROEN9_Pos        (9)                                               /*!< FMC_T::APWPROT0: APPROEN9 Position     */
#define FMC_APWPROT0_APPROEN9_Msk        (0x1ul << FMC_APWPROT0_APPROEN9_Pos)              /*!< FMC_T::APWPROT0: APPROEN9 Mask         */

#define FMC_APWPROT0_APPROEN10_Pos       (10)                                              /*!< FMC_T::APWPROT0: APPROEN10 Position    */
#define FMC_APWPROT0_APPROEN10_Msk       (0x1ul << FMC_APWPROT0_APPROEN10_Pos)             /*!< FMC_T::APWPROT0: APPROEN10 Mask        */

#define FMC_APWPROT0_APPROEN11_Pos       (11)                                              /*!< FMC_T::APWPROT0: APPROEN11 Position    */
#define FMC_APWPROT0_APPROEN11_Msk       (0x1ul << FMC_APWPROT0_APPROEN11_Pos)             /*!< FMC_T::APWPROT0: APPROEN11 Mask        */

#define FMC_APWPROT0_APPROEN12_Pos       (12)                                              /*!< FMC_T::APWPROT0: APPROEN12 Position    */
#define FMC_APWPROT0_APPROEN12_Msk       (0x1ul << FMC_APWPROT0_APPROEN12_Pos)             /*!< FMC_T::APWPROT0: APPROEN12 Mask        */

#define FMC_APWPROT0_APPROEN13_Pos       (13)                                              /*!< FMC_T::APWPROT0: APPROEN13 Position    */
#define FMC_APWPROT0_APPROEN13_Msk       (0x1ul << FMC_APWPROT0_APPROEN13_Pos)             /*!< FMC_T::APWPROT0: APPROEN13 Mask        */

#define FMC_APWPROT0_APPROEN14_Pos       (14)                                              /*!< FMC_T::APWPROT0: APPROEN14 Position    */
#define FMC_APWPROT0_APPROEN14_Msk       (0x1ul << FMC_APWPROT0_APPROEN14_Pos)             /*!< FMC_T::APWPROT0: APPROEN14 Mask        */

#define FMC_APWPROT0_APPROEN15_Pos       (15)                                              /*!< FMC_T::APWPROT0: APPROEN15 Position    */
#define FMC_APWPROT0_APPROEN15_Msk       (0x1ul << FMC_APWPROT0_APPROEN15_Pos)             /*!< FMC_T::APWPROT0: APPROEN15 Mask        */

#define FMC_APWPROT1_APPROEN16_Pos       (0)                                               /*!< FMC_T::APWPROT1: APPROEN16 Position    */
#define FMC_APWPROT1_APPROEN16_Msk       (0x1ul << FMC_APWPROT1_APPROEN16_Pos)             /*!< FMC_T::APWPROT1: APPROEN16 Mask        */

#define FMC_APWPROT1_APPROEN17_Pos       (1)                                               /*!< FMC_T::APWPROT1: APPROEN17 Position    */
#define FMC_APWPROT1_APPROEN17_Msk       (0x1ul << FMC_APWPROT1_APPROEN17_Pos)             /*!< FMC_T::APWPROT1: APPROEN17 Mask        */

#define FMC_APWPROT1_APPROEN18_Pos       (2)                                               /*!< FMC_T::APWPROT1: APPROEN18 Position    */
#define FMC_APWPROT1_APPROEN18_Msk       (0x1ul << FMC_APWPROT1_APPROEN18_Pos)             /*!< FMC_T::APWPROT1: APPROEN18 Mask        */

#define FMC_APWPROT1_APPROEN19_Pos       (3)                                               /*!< FMC_T::APWPROT1: APPROEN19 Position    */
#define FMC_APWPROT1_APPROEN19_Msk       (0x1ul << FMC_APWPROT1_APPROEN19_Pos)             /*!< FMC_T::APWPROT1: APPROEN19 Mask        */

#define FMC_APWPROT1_APPROEN20_Pos       (4)                                               /*!< FMC_T::APWPROT1: APPROEN20 Position    */
#define FMC_APWPROT1_APPROEN20_Msk       (0x1ul << FMC_APWPROT1_APPROEN20_Pos)             /*!< FMC_T::APWPROT1: APPROEN20 Mask        */

#define FMC_APWPROT1_APPROEN21_Pos       (5)                                               /*!< FMC_T::APWPROT1: APPROEN21 Position    */
#define FMC_APWPROT1_APPROEN21_Msk       (0x1ul << FMC_APWPROT1_APPROEN21_Pos)             /*!< FMC_T::APWPROT1: APPROEN21 Mask        */

#define FMC_APWPROT1_APPROEN22_Pos       (6)                                               /*!< FMC_T::APWPROT1: APPROEN22 Position    */
#define FMC_APWPROT1_APPROEN22_Msk       (0x1ul << FMC_APWPROT1_APPROEN22_Pos)             /*!< FMC_T::APWPROT1: APPROEN22 Mask        */

#define FMC_APWPROT1_APPROEN23_Pos       (7)                                               /*!< FMC_T::APWPROT1: APPROEN23 Position    */
#define FMC_APWPROT1_APPROEN23_Msk       (0x1ul << FMC_APWPROT1_APPROEN23_Pos)             /*!< FMC_T::APWPROT1: APPROEN23 Mask        */

#define FMC_APWPROT1_APPROEN24_Pos       (8)                                               /*!< FMC_T::APWPROT1: APPROEN24 Position    */
#define FMC_APWPROT1_APPROEN24_Msk       (0x1ul << FMC_APWPROT1_APPROEN24_Pos)             /*!< FMC_T::APWPROT1: APPROEN24 Mask        */

#define FMC_APWPROT1_APPROEN25_Pos       (9)                                               /*!< FMC_T::APWPROT1: APPROEN25 Position    */
#define FMC_APWPROT1_APPROEN25_Msk       (0x1ul << FMC_APWPROT1_APPROEN25_Pos)             /*!< FMC_T::APWPROT1: APPROEN25 Mask        */

#define FMC_APWPROT1_APPROEN26_Pos       (10)                                              /*!< FMC_T::APWPROT1: APPROEN26 Position    */
#define FMC_APWPROT1_APPROEN26_Msk       (0x1ul << FMC_APWPROT1_APPROEN26_Pos)             /*!< FMC_T::APWPROT1: APPROEN26 Mask        */

#define FMC_APWPROT1_APPROEN27_Pos       (11)                                              /*!< FMC_T::APWPROT1: APPROEN27 Position    */
#define FMC_APWPROT1_APPROEN27_Msk       (0x1ul << FMC_APWPROT1_APPROEN27_Pos)             /*!< FMC_T::APWPROT1: APPROEN27 Mask        */

#define FMC_APWPROT1_APPROEN28_Pos       (12)                                              /*!< FMC_T::APWPROT1: APPROEN28 Position    */
#define FMC_APWPROT1_APPROEN28_Msk       (0x1ul << FMC_APWPROT1_APPROEN28_Pos)             /*!< FMC_T::APWPROT1: APPROEN28 Mask        */

#define FMC_APWPROT1_APPROEN29_Pos       (13)                                              /*!< FMC_T::APWPROT1: APPROEN29 Position    */
#define FMC_APWPROT1_APPROEN29_Msk       (0x1ul << FMC_APWPROT1_APPROEN29_Pos)             /*!< FMC_T::APWPROT1: APPROEN29 Mask        */

#define FMC_APWPROT1_APPROEN30_Pos       (14)                                              /*!< FMC_T::APWPROT1: APPROEN30 Position    */
#define FMC_APWPROT1_APPROEN30_Msk       (0x1ul << FMC_APWPROT1_APPROEN30_Pos)             /*!< FMC_T::APWPROT1: APPROEN30 Mask        */

#define FMC_APWPROT1_APPROEN31_Pos       (15)                                              /*!< FMC_T::APWPROT1: APPROEN31 Position    */
#define FMC_APWPROT1_APPROEN31_Msk       (0x1ul << FMC_APWPROT1_APPROEN31_Pos)             /*!< FMC_T::APWPROT1: APPROEN31 Mask        */

#define FMC_APWPKEEP_APWPKEEP0_Pos       (0)                                               /*!< FMC_T::APWPKEEP: APWPKEEP0 Position    */
#define FMC_APWPKEEP_APWPKEEP0_Msk       (0xfffful << FMC_APWPKEEP_APWPKEEP0_Pos)          /*!< FMC_T::APWPKEEP: APWPKEEP0 Mask        */

#define FMC_APWPKEEP_APWPKEEP1_Pos       (16)                                              /*!< FMC_T::APWPKEEP: APWPKEEP1 Position    */
#define FMC_APWPKEEP_APWPKEEP1_Msk       (0xfffful << FMC_APWPKEEP_APWPKEEP1_Pos)          /*!< FMC_T::APWPKEEP: APWPKEEP1 Mask        */

#define FMC_SCACT_SCACT_Pos              (0)                                               /*!< FMC_T::SCACT: SCACT Position           */
#define FMC_SCACT_SCACT_Msk              (0x1ul << FMC_SCACT_SCACT_Pos)                    /*!< FMC_T::SCACT: SCACT Mask               */

#define FMC_LDWPROT_LDPROEN0_Pos         (0)                                               /*!< FMC_T::LDWPROT: LDPROEN0 Position      */
#define FMC_LDWPROT_LDPROEN0_Msk         (0x1ul << FMC_LDWPROT_LDPROEN0_Pos)               /*!< FMC_T::LDWPROT: LDPROEN0 Mask          */

#define FMC_LDWPKEEP_LDWPKEEP_Pos        (0)                                               /*!< FMC_T::LDWPKEEP: LDWPKEEP Position     */
#define FMC_LDWPKEEP_LDWPKEEP_Msk        (0xfffful << FMC_LDWPKEEP_LDWPKEEP_Pos)           /*!< FMC_T::LDWPKEEP: LDWPKEEP Mask         */

#define FMC_DFWPROT_DFPROENn_Pos         (0)                                               /*!< FMC_T::DFWPROT: DFPROENn Position      */
#define FMC_DFWPROT_DFPROENn_Msk         (0x1ul << FMC_DFWPROT_DFPROENn_Pos)               /*!< FMC_T::DFWPROT: DFPROENn Mask          */

#define FMC_DFWPKEEP_ECCDEFAR_Pos        (0)                                               /*!< FMC_T::DFWPKEEP: ECCDEFAR Position     */
#define FMC_DFWPKEEP_ECCDEFAR_Msk        (0xfffffffful << FMC_DFWPKEEP_ECCDEFAR_Pos)       /*!< FMC_T::DFWPKEEP: ECCDEFAR Mask         */

#define FMC_ECCCTL_SECINTEN_Pos          (0)                                               /*!< FMC_T::ECCCTL: SECINTEN Position       */
#define FMC_ECCCTL_SECINTEN_Msk          (0x1ul << FMC_ECCCTL_SECINTEN_Pos)                /*!< FMC_T::ECCCTL: SECINTEN Mask           */

#define FMC_ECCCTL_DEBDINTEN_Pos         (1)                                               /*!< FMC_T::ECCCTL: DEBDINTEN Position      */
#define FMC_ECCCTL_DEBDINTEN_Msk         (0x1ul << FMC_ECCCTL_DEBDINTEN_Pos)               /*!< FMC_T::ECCCTL: DEBDINTEN Mask          */

#define FMC_ECCCTL_BEDIS_Pos             (31)                                              /*!< FMC_T::ECCCTL: BEDIS Position          */
#define FMC_ECCCTL_BEDIS_Msk             (0x1ul << FMC_ECCCTL_BEDIS_Pos)                   /*!< FMC_T::ECCCTL: BEDIS Mask              */

#define FMC_ECCSTS_ECCSEBCF_Pos          (0)                                               /*!< FMC_T::ECCSTS: ECCSEBCF Position       */
#define FMC_ECCSTS_ECCSEBCF_Msk          (0x1ul << FMC_ECCSTS_ECCSEBCF_Pos)                /*!< FMC_T::ECCSTS: ECCSEBCF Mask           */

#define FMC_ECCSTS_ECCDEBDF_Pos          (1)                                               /*!< FMC_T::ECCSTS: ECCDEBDF Position       */
#define FMC_ECCSTS_ECCDEBDF_Msk          (0x1ul << FMC_ECCSTS_ECCDEBDF_Pos)                /*!< FMC_T::ECCSTS: ECCDEBDF Mask           */

#define FMC_ECCSEFAR_ECCSEFAR_Pos        (0)                                               /*!< FMC_T::ECCSEFAR: ECCSEFAR Position     */
#define FMC_ECCSEFAR_ECCSEFAR_Msk        (0xfffffffful << FMC_ECCSEFAR_ECCSEFAR_Pos)       /*!< FMC_T::ECCSEFAR: ECCSEFAR Mask         */

#define FMC_ECCDEFAR_ECCDEFAR_Pos        (0)                                               /*!< FMC_T::ECCDEFAR: ECCDEFAR Position     */
#define FMC_ECCDEFAR_ECCDEFAR_Msk        (0xfffffffful << FMC_ECCDEFAR_ECCDEFAR_Pos)       /*!< FMC_T::ECCDEFAR: ECCDEFAR Mask         */

#define FMC_FUNCASS_USB_Pos              (0)                                               /*!< FMC_T::FUNCASS: USB Position           */
#define FMC_FUNCASS_USB_Msk              (0x1ul << FMC_FUNCASS_USB_Pos)                    /*!< FMC_T::FUNCASS: USB Mask               */

#define FMC_FUNCASS_CAN0_Pos             (1)                                               /*!< FMC_T::FUNCASS: CAN0 Position          */
#define FMC_FUNCASS_CAN0_Msk             (0x1ul << FMC_FUNCASS_CAN0_Pos)                   /*!< FMC_T::FUNCASS: CAN0 Mask              */

#define FMC_FUNCASS_CAN1_Pos             (2)                                               /*!< FMC_T::FUNCASS: CAN1 Position          */
#define FMC_FUNCASS_CAN1_Msk             (0x1ul << FMC_FUNCASS_CAN1_Pos)                   /*!< FMC_T::FUNCASS: CAN1 Mask              */

#define FMC_FUNCASS_ROMSIZE_Pos          (4)                                               /*!< FMC_T::FUNCASS: ROMSIZE Position       */
#define FMC_FUNCASS_ROMSIZE_Msk          (0xful << FMC_FUNCASS_ROMSIZE_Pos)                /*!< FMC_T::FUNCASS: ROMSIZE Mask           */

#define FMC_FUNCASS_RAMSIZE_Pos          (12)                                              /*!< FMC_T::FUNCASS: RAMSIZE Position       */
#define FMC_FUNCASS_RAMSIZE_Msk          (0x3ul << FMC_FUNCASS_RAMSIZE_Pos)                /*!< FMC_T::FUNCASS: RAMSIZE Mask           */

#define FMC_VERSION_MINOR_Pos            (0)                                               /*!< FMC_T::VERSION: MINOR Position         */
#define FMC_VERSION_MINOR_Msk            (0xfffful << FMC_VERSION_MINOR_Pos)               /*!< FMC_T::VERSION: MINOR Mask             */

#define FMC_VERSION_SUB_Pos              (16)                                              /*!< FMC_T::VERSION: SUB Position           */
#define FMC_VERSION_SUB_Msk              (0xfful << FMC_VERSION_SUB_Pos)                   /*!< FMC_T::VERSION: SUB Mask               */

#define FMC_VERSION_MAJOR_Pos            (24)                                              /*!< FMC_T::VERSION: MAJOR Position         */
#define FMC_VERSION_MAJOR_Msk            (0xfful << FMC_VERSION_MAJOR_Pos)                 /*!< FMC_T::VERSION: MAJOR Mask             */

/**@}*/ /* FMC_CONST */
/**@}*/ /* end of FMC register group */


/*---------------------- ?????????????????????????????????????????? -------------------------*/
/**
    @addtogroup FMC_NSBA ??????????????????????????????????????????(FMC_NSBA)
    Memory Mapped Structure for FMC_NSBA Controller
@{ */

typedef struct
{


/**
 * @var FMC_NS_T::FMC_ISPCTL
 * Offset: 0x00  Non-secure ISP Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ISPEN     |ISP Enable Bit (Write Protect)
 * |        |          |ISP function enable.
 * |        |          |0 = ISP function Disabled.
 * |        |          |1 = ISP function Enabled.
 * |        |          |Note: This bit is read only to show ISP function enable.
 * |[3]     |APUEN     |APROM Update Enable Bit (Write Protect)
 * |        |          |0 = APROM cannot be updated when the chip runs in APROM.
 * |        |          |1 = APROM can be updated when the chip runs in APROM.
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[6]     |NS_ISPFF  |Non-sec ISP Fail Flag (Write Protect)
 * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
 * |        |          |This bit needs to be cleared by writing 1 to it.
 * |        |          |u00B7 APROM writes to itself if APUEN is set to 0.
 * |        |          |u00B7 Page Erase command at LOCK mode with ICE connection
 * |        |          |u00B7 Erase or Program command at brown-out detected
 * |        |          |u00B7 Destination address is illegal, such as over an available range.
 * |        |          |u00B7 Invalid ISP commands
 * |        |          |u00B7 Read any content of boot loader with ICE connection
 * |        |          |u00B7 ISP CMD in XOM region, except xom page erase and chksum command
 * |        |          |u00B7 The wrong setting of page erase ISP CMD in XOM
 * |        |          |u00B7 Violate XOM setting one time protection
 * |        |          |u00B7 Page erase ISP CMD in Secure region setting page
 * |        |          |u00B7 Page erase, mass erase, multi-word program or 64-bit word program in OTP
 * |        |          |u00B7 Read ISP CMD when Load code mode is active
 * |        |          |u00B7 NS_ISP Conflict Error
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[24]    |NS_INTEN  |Non-secure ISP interrupt Enable Bit
 * |        |          |0 = NS_ISP INT Disabled.
 * |        |          |1 = NS_ISP INT Enabled.
 * |        |          |Note: This bit is write protected
 * |        |          |Refer to the SYS_REGLCTL register
 * |        |          |Before using INT, user needs to clear the INTFLAG(FMC_ISPSTS[24]) make sure INT happen at correct time.
 * @var FMC_NS_T::FMC_ISPADDR
 * Offset: 0x04  Non-secure ISP Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |NS_ISPADDR|Non-sec ISP Address
 * |        |          |The M3355 series is equipped with embedded Flash
 * |        |          |ISPADDR[1:0] must be kept 00 for ISP 32-bit operation.
 * |        |          |For CRC32 Checksum Calculation command, this field is the Flash starting address for checksum calculation, 8 Kbytes alignment is necessary for CRC32 checksum calculation.
 * |        |          |For Flash32-bit Program, ISP address needs word alignment (4-byte)
 * |        |          |For Flash 64-bit Program, ISP address needs double word alignment (8-byte).
 * |        |          |Non-sec ISP address must be active at Non-sec region.
 * @var FMC_NS_T::FMC_ISPDAT
 * Offset: 0x08  Non-secure ISP Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |NS_ISPDAT |Non-sec ISP Data
 * |        |          |Write data to this register before ISP program operation.
 * |        |          |Read data from this register after ISP read operation.
 * |        |          |When ISPFF (FMC_ISPCTL[6]) is 1, ISPDAT = 0xffff_ffff
 * |        |          |For Run CRC32 Checksum Calculation command, ISPDAT is the memory size (byte) and 8 Kbytes alignment
 * |        |          |For ISP Read CRC32 Checksum command, ISPDAT is the checksum result
 * |        |          |If ISPDAT = 0x0000_0000, it means that (1) the checksum calculation is in progress, or (2) the memory range for checksum calculation is incorrect
 * |        |          |For XOM page erase function, SPDAT = 0x0055_aa03.
 * @var FMC_NS_T::FMC_ISPCMD
 * Offset: 0x0C  Non-secure ISP Command Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[6:0]   |NS_CMD    |Non-sec ISP Command
 * |        |          |ISP command table is shown below:
 * |        |          |0x00= FLASH Read.
 * |        |          |0x04= Read Unique ID.
 * |        |          |0x08= Read Flash All-One Result.
 * |        |          |0x0B= Read Company ID.
 * |        |          |0x0C= Read Device ID.
 * |        |          |0x0D= Read Checksum.
 * |        |          |0x21= FLASH 32-bit Program.
 * |        |          |0x22= FLASH Page Erase. Erase only non-secure page in two banks.
 * |        |          |0x28= Run Flash All-One Verification.
 * |        |          |0x2D= Run Checksum Calculation.
 * |        |          |The other commands are invalid.
 * @var FMC_NS_T::FMC_ISPTRG
 * Offset: 0x10  Non-secure ISP Trigger Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |NS_ISPGO  |Non-sec ISP Start Trigger (Write Protect)
 * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP operation is finished
 * |        |          |When ISPGO=1,the operation of writing value to address from FMC_BA+0x00 to FMC_BA+0x68 would be ignored.
 * |        |          |0 = ISP operation is finished.
 * |        |          |1 = ISP is progressed.
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * @var FMC_NS_T::FMC_ISPSTS
 * Offset: 0x40  Non-secure ISP Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |NS_ISPBUSY|Non-sec ISP Busy Flag (Read Only)
 * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP operation is finished.
 * |        |          |This bit is the mirror of ISPGO(FMC_ISPTRG[0]).
 * |        |          |0 = ISP operation is finished.
 * |        |          |1 = ISP is progressed.
 * |[6]     |NS_ISPFF  |Non-sec ISP Fail Flag (Write Protect)
 * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
 * |        |          |This bit needs to be cleared by writing 1 to it.
 * |        |          |u00B7 APROM writes to itself if APUEN is set to 0.
 * |        |          |u00B7 Page Erase command at LOCK mode with ICE connection
 * |        |          |u00B7 Erase or Program command at brown-out detected
 * |        |          |u00B7 Destination address is illegal, such as over an available range.
 * |        |          |u00B7 Invalid ISP commands
 * |        |          |u00B7 Read any content of boot loader with ICE connection
 * |        |          |u00B7 ISP CMD in XOM region, except XOM page erase and chksum command
 * |        |          |u00B7 The wrong setting of page erase ISP CMD in XOM
 * |        |          |u00B7 Violate XOM setting one time protection
 * |        |          |u00B7 Page erase ISP CMD in Secure region setting page
 * |        |          |u00B7 Page erase, mass erase, multi-word program or 64-bit word program in OTP
 * |        |          |u00B7 Read ISP CMD when Load code mode is active
 * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
 * |[7]     |ALLONE    |Flash All-one Verification Flag
 * |        |          |This bit is set by hardware if all of Flash bits are 1, and cleared if Flash bits are not all 1 after u201CRun Flash All-One Verificationu201D is complete; this bit also can be cleared by writing 1
 * |        |          |0 = Flash bits are not all 1 after u201CRun Flash All-One Verificationu201D complete.
 * |        |          |1 = All of Flash bits are 1 after u201CRun Flash All-One Verificationu201D complete.
 * |[24]    |NSINTFLAG |Non-sec ISP interrupt Flag
 * |        |          |0 = ISP Not Finished.
 * |        |          |1 = ISP done or ISPFF set.
 * |        |          |Note: This function needs to be enabled by Non-secure FMC_ISPCTRL[24].
 * |[28]    |NSISPCER  |Non-secure ISP Conflict Error
 * |        |          |This bit shows when FMC is doing ISP operation
 * |        |          |User cannot access NS_FMC_ISP_ADDR,NS_FMC_ISPDAT,NS_FMC_ISPCMD,NS_FMC_ISPTRG
 * |        |          |It would cause ISPFF.
 * |[29]    |MIRBOUND  |Mirror Boundary (Read Only)
 * |        |          |0 = Mirror Boundary Disabled.
 * |        |          |1 = Mirror Boundary Enabled.
 * |[30]    |FBS       |Flash Bank Selection (Read Only)
 * |        |          |This bit indicate which bank is selected to boot.
 * |        |          |0 = Booting from BANK0.
 * |        |          |1 = Booting from BANK1.
 * @var FMC_NS_T::FMC_MPDAT0
 * Offset: 0x80  Non-secure ISP Data0 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var FMC_NS_T::FMC_MPDAT1
 * Offset: 0x84  Non-secure ISP Data1 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var FMC_NS_T::FMC_MPDAT2
 * Offset: 0x88  Non-secure ISP Data2 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var FMC_NS_T::FMC_MPDAT3
 * Offset: 0x8C  Non-secure ISP Data3 Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var FMC_NS_T::FMC_MPDATE
 * Offset: 0x90  Non-secure ISP ERROR Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var FMC_NS_T::FMC_XOMR0STS
 * Offset: 0xD0  Non-secure XOM Region 0 Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |SIZE      |XOM Region 0 Size
 * |        |          |SIZE is the page number of XOM Region 0.
 * |[31:8]  |BASE      |XOM Region 0 Base Address
 * |        |          |BASE is the base address of XOM Region 0.
 * |        |          |Note: The base address is page aligned.
 * @var FMC_NS_T::FMC_XOMR1STS
 * Offset: 0xD4  Non-secure XOM Region 1 Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |SIZE      |XOM Region 1 Size
 * |        |          |SIZE is the page number of XOM Region 1.
 * |[31:8]  |BASE      |XOM Region 1 Base Address
 * |        |          |BASE is the base address of XOM Region 1.
 * |        |          |Note: The base address is page aligned.
 * @var FMC_NS_T::FMC_XOMSTS
 * Offset: 0xE0  Non-secure XOM Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |XOMR0ON   |XOM Region 0 On
 * |        |          |XOM Region 0 active status.
 * |        |          |0 = Not active.
 * |        |          |1 = XOM region 0 is active.
 * |[1]     |XOMR1ON   |XOM Region 1 On
 * |        |          |XOM Region 1 active status.
 * |        |          |0 = Not active.
 * |        |          |1 = XOM region 1 is active.
 * |[4]     |XOMPEF    |XOM Page Erase Function Fail
 * |        |          |XOM page erase function status. If XOMPEF is set to 1, user needs to erase XOM region again.
 * |        |          |0 = Success.
 * |        |          |1 = Fail.
 * @var FMC_NS_T::FMC_APWPROT0
 * Offset: 0x110  APROM Write Protect Register0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |APPROEN0  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[1]     |APPROEN1  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[2]     |APPROEN2  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[3]     |APPROEN3  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[4]     |APPROEN4  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[5]     |APPROEN5  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[6]     |APPROEN6  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[7]     |APPROEN7  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[8]     |APPROEN8  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[9]     |APPROEN9  |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[10]    |APPROEN10 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[11]    |APPROEN11 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[12]    |APPROEN12 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[13]    |APPROEN13 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[14]    |APPROEN14 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[15]    |APPROEN15 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[16]    |APPROEN16 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[17]    |APPROEN17 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[18]    |APPROEN18 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[19]    |APPROEN19 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[20]    |APPROEN20 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[21]    |APPROEN21 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[22]    |APPROEN22 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[23]    |APPROEN23 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[24]    |APPROEN24 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[25]    |APPROEN25 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[26]    |APPROEN26 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[27]    |APPROEN27 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[28]    |APPROEN28 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[29]    |APPROEN29 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[30]    |APPROEN30 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * |[31]    |APPROEN31 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1010_0000 + n*(0x8000) to 0x1010_7fff + n*(0x8000)
 * @var FMC_NS_T::FMC_APWPROT1
 * Offset: 0x114  APROM Write Protect Register1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |APPROEN32 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[1]     |APPROEN33 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[2]     |APPROEN34 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[3]     |APPROEN35 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[4]     |APPROEN36 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[5]     |APPROEN37 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[6]     |APPROEN38 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[7]     |APPROEN39 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[8]     |APPROEN40 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[9]     |APPROEN41 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[10]    |APPROEN42 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[11]    |APPROEN43 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[12]    |APPROEN44 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[13]    |APPROEN45 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[14]    |APPROEN46 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[15]    |APPROEN47 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[16]    |APPROEN48 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[17]    |APPROEN49 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[18]    |APPROEN50 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[19]    |APPROEN51 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[20]    |APPROEN52 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[21]    |APPROEN53 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[22]    |APPROEN54 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[23]    |APPROEN55 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[24]    |APPROEN56 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[25]    |APPROEN57 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[26]    |APPROEN58 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[27]    |APPROEN59 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[28]    |APPROEN60 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[29]    |APPROEN61 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[30]    |APPROEN62 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * |[31]    |APPROEN63 |APROM Proect enable
 * |        |          |This bit indicates which APROM region is protected.
 * |        |          |0 = APROM region n is not protected.
 * |        |          |1 = APROM region n is protected.
 * |        |          |Note: APROM protect region is 0x1020_0000 + n*(0x8000) to 0x1020_7fff + n*(0x8000)
 * @var FMC_NS_T::FMC_APWPKEEP
 * Offset: 0x118  APROM Write Protect Keep Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |APWPKEEP0 |APROM Write Protect Keep 0
 * |        |          |0x55AA = APWPROT0 register is be locked.
 * |        |          |others = APWPROT0 register is free.
 * |[31:16] |APWPKEEP1 |APROM Write Protect Keep 1
 * |        |          |0x55AA = APWPROT1 register is be locked.
 * |        |          |others = APWPROT1 register is free.
 * @var FMC_NS_T::FMC_SCACT
 * Offset: 0x11C  APROM Secure Conceal Active Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SCACT     |Secure Conceal Function Active
 * |        |          |0 = secure conceal function inactive.
 * |        |          |1 = secure conceal function active.
 * |        |          |Note: secure conceal function active will base on this bit and setting of CONFIG6 is all 0.
 * @var FMC_NS_T::FMC_ECCCTL
 * Offset: 0x130  Non-secure FMC ECC Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var FMC_NS_T::FMC_ECCSTS
 * Offset: 0x134  Non-secure FMC ECC Status
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var FMC_NS_T::FMC_ECCSEFAR
 * Offset: 0x138  Non-secure FMC ECC Single Error Fault Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var FMC_NS_T::FMC_ECCDEFAR
 * Offset: 0x13C  Non-secure FMC ECC Double Error Fault Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 */
    __IO uint32_t ISPCTL;            /*!< [0x0000] Non-secure ISP Control Register                                  */
    __IO uint32_t ISPADDR;           /*!< [0x0004] Non-secure ISP Address Register                                  */
    __IO uint32_t ISPDAT;            /*!< [0x0008] Non-secure ISP Data Register                                     */
    __IO uint32_t ISPCMD;            /*!< [0x000c] Non-secure ISP Command Register                                  */
    __IO uint32_t ISPTRG;            /*!< [0x0010] Non-secure ISP Trigger Control Register                          */
    __I  uint32_t RESERVE0[11];
    __IO uint32_t ISPSTS;            /*!< [0x0040] Non-secure ISP Status Register                                   */
    __I  uint32_t RESERVE1[15];
    __IO uint32_t MPDAT0;            /*!< [0x0080] Non-secure ISP Data0 Register                                    */
    __IO uint32_t MPDAT1;            /*!< [0x0084] Non-secure ISP Data1 Register                                    */
    __IO uint32_t MPDAT2;            /*!< [0x0088] Non-secure ISP Data2 Register                                    */
    __IO uint32_t MPDAT3;            /*!< [0x008c] Non-secure ISP Data3 Register                                    */
    __IO uint32_t MPDATE;            /*!< [0x0090] Non-secure ISP ERROR Data Register                               */
    __I  uint32_t RESERVE2[15];
    __I  uint32_t XOMR0STS;          /*!< [0x00d0] Non-secure XOM Region 0 Status Register                          */
    __I  uint32_t XOMR1STS;          /*!< [0x00d4] Non-secure XOM Region 1 Status Register                          */
    __I  uint32_t RESERVE3[2];
    __I  uint32_t XOMSTS;            /*!< [0x00e0] Non-secure XOM Status Register                                   */
    __I  uint32_t RESERVE4[11];
    __I  uint32_t APWPROT0;          /*!< [0x0110] APROM Write Protect Register0                                    */
    __I  uint32_t APWPROT1;          /*!< [0x0114] APROM Write Protect Register1                                    */
    __I  uint32_t APWPKEEP;          /*!< [0x0118] APROM Write Protect Keep Register                                */
    __I  uint32_t SCACT;             /*!< [0x011c] APROM Secure Conceal Active Register                             */
    __I  uint32_t RESERVE5[4];
    __IO uint32_t ECCCTL;            /*!< [0x0130] Non-secure FMC ECC Control                                       */
    __IO uint32_t ECCSTS;            /*!< [0x0134] Non-secure FMC ECC Status                                        */
    __IO uint32_t ECCSEFAR;          /*!< [0x0138] Non-secure FMC ECC Single Error Fault Address Register           */
    __IO uint32_t ECCDEFAR;          /*!< [0x013c] Non-secure FMC ECC Double Error Fault Address Register           */

} FMC_NS_T;

/**
    @addtogroup FMC_NS_CONST FMC_NSBA Bit Field Definition
    Constant Definitions for FMC_NSBA Controller
@{ */

#define FMC_NS_ISPCTL_ISPEN_Pos         (0)                                               /*!< FMC_NS_T::FMC_ISPCTL: ISPEN Position          */
#define FMC_NS_ISPCTL_ISPEN_Msk         (0x1ul << FMC_NS_ISPCTL_ISPEN_Pos)                /*!< FMC_NS_T::FMC_ISPCTL: ISPEN Mask              */

#define FMC_NS_ISPCTL_APUEN_Pos         (3)                                               /*!< FMC_NS_T::FMC_ISPCTL: APUEN Position          */
#define FMC_NS_ISPCTL_APUEN_Msk         (0x1ul << FMC_NS_ISPCTL_APUEN_Pos)                /*!< FMC_NS_T::FMC_ISPCTL: APUEN Mask              */

#define FMC_NS_ISPCTL_ISPFF_Pos         (6)                                               /*!< FMC_NS_T::FMC_ISPCTL: NS_ISPFF Position       */
#define FMC_NS_ISPCTL_ISPFF_Msk         (0x1ul << FMC_NS_ISPCTL_ISPFF_Pos)                /*!< FMC_NS_T::FMC_ISPCTL: NS_ISPFF Mask           */

#define FMC_NS_ISPCTL_INTEN_Pos         (24)                                              /*!< FMC_NS_T::FMC_ISPCTL: NS_INTEN Position       */
#define FMC_NS_ISPCTL_INTEN_Msk         (0x1ul << FMC_NS_ISPCTL_INTEN_Pos)                /*!< FMC_NS_T::FMC_ISPCTL: NS_INTEN Mask           */

#define FMC_NS_ISPADDR_ISPADDR_Pos      (0)                                               /*!< FMC_NS_T::FMC_ISPADDR: NS_ISPADDR Position    */
#define FMC_NS_ISPADDR_ISPADDR_Msk      (0xfffffffful << FMC_NS_ISPADDR_ISPADDR_Pos)      /*!< FMC_NS_T::FMC_ISPADDR: NS_ISPADDR Mask        */

#define FMC_NS_ISPDAT_ISPDAT_Pos        (0)                                               /*!< FMC_NS_T::FMC_ISPDAT: NS_ISPDAT Position      */
#define FMC_NS_ISPDAT_ISPDAT_Msk        (0xfffffffful << FMC_NS_ISPDAT_ISPDAT_Pos)        /*!< FMC_NS_T::FMC_ISPDAT: NS_ISPDAT Mask          */

#define FMC_NS_ISPCMD_CMD_Pos           (0)                                               /*!< FMC_NS_T::FMC_ISPCMD: NS_CMD Position         */
#define FMC_NS_ISPCMD_CMD_Msk           (0x7ful << FMC_NS_ISPCMD_CMD_Pos)                 /*!< FMC_NS_T::FMC_ISPCMD: NS_CMD Mask             */

#define FMC_NS_ISPTRG_ISPGO_Pos         (0)                                               /*!< FMC_NS_T::FMC_ISPTRG: NS_ISPGO Position       */
#define FMC_NS_ISPTRG_ISPGO_Msk         (0x1ul << FMC_NS_ISPTRG_ISPGO_Pos)                /*!< FMC_NS_T::FMC_ISPTRG: NS_ISPGO Mask           */

#define FMC_NS_ISPSTS_ISPBUSY_Pos       (0)                                               /*!< FMC_NS_T::FMC_ISPSTS: NS_ISPBUSY Position     */
#define FMC_NS_ISPSTS_ISPBUSY_Msk       (0x1ul << FMC_NS_ISPSTS_ISPBUSY_Pos)              /*!< FMC_NS_T::FMC_ISPSTS: NS_ISPBUSY Mask         */

#define FMC_NS_ISPSTS_ISPFF_Pos         (6)                                               /*!< FMC_NS_T::FMC_ISPSTS: NS_ISPFF Position       */
#define FMC_NS_ISPSTS_ISPFF_Msk         (0x1ul << FMC_NS_ISPSTS_ISPFF_Pos)                /*!< FMC_NS_T::FMC_ISPSTS: NS_ISPFF Mask           */

#define FMC_NS_ISPSTS_ALLONE_Pos        (7)                                               /*!< FMC_NS_T::FMC_ISPSTS: ALLONE Position         */
#define FMC_NS_ISPSTS_ALLONE_Msk        (0x1ul << FMC_NS_ISPSTS_ALLONE_Pos)               /*!< FMC_NS_T::FMC_ISPSTS: ALLONE Mask             */

#define FMC_NS_ISPSTS_INTFLAG_Pos       (24)                                              /*!< FMC_NS_T::FMC_ISPSTS: NSINTFLAG Position      */
#define FMC_NS_ISPSTS_INTFLAG_Msk       (0x1ul << FMC_NS_ISPSTS_INTFLAG_Pos)              /*!< FMC_NS_T::FMC_ISPSTS: NSINTFLAG Mask          */

#define FMC_NS_ISPSTS_ISPCER_Pos        (28)                                              /*!< FMC_NS_T::FMC_ISPSTS: NSISPCER Position       */
#define FMC_NS_ISPSTS_ISPCER_Msk        (0x1ul << FMC_NS_ISPSTS_ISPCER_Pos)               /*!< FMC_NS_T::FMC_ISPSTS: NSISPCER Mask           */

#define FMC_NS_ISPSTS_MIRBOUND_Pos      (29)                                              /*!< FMC_NS_T::FMC_ISPSTS: MIRBOUND Position       */
#define FMC_NS_ISPSTS_MIRBOUND_Msk      (0x1ul << FMC_NS_ISPSTS_MIRBOUND_Pos)             /*!< FMC_NS_T::FMC_ISPSTS: MIRBOUND Mask           */

#define FMC_NS_ISPSTS_FBS_Pos           (30)                                              /*!< FMC_NS_T::FMC_ISPSTS: FBS Position            */
#define FMC_NS_ISPSTS_FBS_Msk           (0x1ul << FMC_NS_ISPSTS_FBS_Pos)                  /*!< FMC_NS_T::FMC_ISPSTS: FBS Mask                */

#define FMC_NS_XOMR0STS_SIZE_Pos        (0)                                               /*!< FMC_NS_T::FMC_XOMR0STS: SIZE Position         */
#define FMC_NS_XOMR0STS_SIZE_Msk        (0xfful << FMC_NS_XOMR0STS_SIZE_Pos)              /*!< FMC_NS_T::FMC_XOMR0STS: SIZE Mask             */

#define FMC_NS_XOMR0STS_BASE_Pos        (8)                                               /*!< FMC_NS_T::FMC_XOMR0STS: BASE Position         */
#define FMC_NS_XOMR0STS_BASE_Msk        (0xfffffful << FMC_NS_XOMR0STS_BASE_Pos)          /*!< FMC_NS_T::FMC_XOMR0STS: BASE Mask             */

#define FMC_NS_XOMR1STS_SIZE_Pos        (0)                                               /*!< FMC_NS_T::FMC_XOMR1STS: SIZE Position         */
#define FMC_NS_XOMR1STS_SIZE_Msk        (0xfful << FMC_NS_XOMR1STS_SIZE_Pos)              /*!< FMC_NS_T::FMC_XOMR1STS: SIZE Mask             */

#define FMC_NS_XOMR1STS_BASE_Pos        (8)                                               /*!< FMC_NS_T::FMC_XOMR1STS: BASE Position         */
#define FMC_NS_XOMR1STS_BASE_Msk        (0xfffffful << FMC_NS_XOMR1STS_BASE_Pos)          /*!< FMC_NS_T::FMC_XOMR1STS: BASE Mask             */

#define FMC_NS_XOMSTS_XOMR0ON_Pos       (0)                                               /*!< FMC_NS_T::FMC_XOMSTS: XOMR0ON Position        */
#define FMC_NS_XOMSTS_XOMR0ON_Msk       (0x1ul << FMC_NS_XOMSTS_XOMR0ON_Pos)              /*!< FMC_NS_T::FMC_XOMSTS: XOMR0ON Mask            */

#define FMC_NS_XOMSTS_XOMR1ON_Pos       (1)                                               /*!< FMC_NS_T::FMC_XOMSTS: XOMR1ON Position        */
#define FMC_NS_XOMSTS_XOMR1ON_Msk       (0x1ul << FMC_NS_XOMSTS_XOMR1ON_Pos)              /*!< FMC_NS_T::FMC_XOMSTS: XOMR1ON Mask            */

#define FMC_NS_XOMSTS_XOMPEF_Pos        (4)                                               /*!< FMC_NS_T::FMC_XOMSTS: XOMPEF Position         */
#define FMC_NS_XOMSTS_XOMPEF_Msk        (0x1ul << FMC_NS_XOMSTS_XOMPEF_Pos)               /*!< FMC_NS_T::FMC_XOMSTS: XOMPEF Mask             */

#define FMC_NS_APWPROT0_APPROEN0_Pos    (0)                                               /*!< FMC_NS_T::FMC_APWPROT0: APPROEN0 Position     */
#define FMC_NS_APWPROT0_APPROEN0_Msk    (0x1ul << FMC_NS_APWPROT0_APPROEN0_Pos)           /*!< FMC_NS_T::FMC_APWPROT0: APPROEN0 Mask         */

#define FMC_NS_APWPROT0_APPROEN1_Pos    (1)                                               /*!< FMC_NS_T::FMC_APWPROT0: APPROEN1 Position     */
#define FMC_NS_APWPROT0_APPROEN1_Msk    (0x1ul << FMC_NS_APWPROT0_APPROEN1_Pos)           /*!< FMC_NS_T::FMC_APWPROT0: APPROEN1 Mask         */

#define FMC_NS_APWPROT0_APPROEN2_Pos    (2)                                               /*!< FMC_NS_T::FMC_APWPROT0: APPROEN2 Position     */
#define FMC_NS_APWPROT0_APPROEN2_Msk    (0x1ul << FMC_NS_APWPROT0_APPROEN2_Pos)           /*!< FMC_NS_T::FMC_APWPROT0: APPROEN2 Mask         */

#define FMC_NS_APWPROT0_APPROEN3_Pos    (3)                                               /*!< FMC_NS_T::FMC_APWPROT0: APPROEN3 Position     */
#define FMC_NS_APWPROT0_APPROEN3_Msk    (0x1ul << FMC_NS_APWPROT0_APPROEN3_Pos)           /*!< FMC_NS_T::FMC_APWPROT0: APPROEN3 Mask         */

#define FMC_NS_APWPROT0_APPROEN4_Pos    (4)                                               /*!< FMC_NS_T::FMC_APWPROT0: APPROEN4 Position     */
#define FMC_NS_APWPROT0_APPROEN4_Msk    (0x1ul << FMC_NS_APWPROT0_APPROEN4_Pos)           /*!< FMC_NS_T::FMC_APWPROT0: APPROEN4 Mask         */

#define FMC_NS_APWPROT0_APPROEN5_Pos    (5)                                               /*!< FMC_NS_T::FMC_APWPROT0: APPROEN5 Position     */
#define FMC_NS_APWPROT0_APPROEN5_Msk    (0x1ul << FMC_NS_APWPROT0_APPROEN5_Pos)           /*!< FMC_NS_T::FMC_APWPROT0: APPROEN5 Mask         */

#define FMC_NS_APWPROT0_APPROEN6_Pos    (6)                                               /*!< FMC_NS_T::FMC_APWPROT0: APPROEN6 Position     */
#define FMC_NS_APWPROT0_APPROEN6_Msk    (0x1ul << FMC_NS_APWPROT0_APPROEN6_Pos)           /*!< FMC_NS_T::FMC_APWPROT0: APPROEN6 Mask         */

#define FMC_NS_APWPROT0_APPROEN7_Pos    (7)                                               /*!< FMC_NS_T::FMC_APWPROT0: APPROEN7 Position     */
#define FMC_NS_APWPROT0_APPROEN7_Msk    (0x1ul << FMC_NS_APWPROT0_APPROEN7_Pos)           /*!< FMC_NS_T::FMC_APWPROT0: APPROEN7 Mask         */

#define FMC_NS_APWPROT0_APPROEN8_Pos    (8)                                               /*!< FMC_NS_T::FMC_APWPROT0: APPROEN8 Position     */
#define FMC_NS_APWPROT0_APPROEN8_Msk    (0x1ul << FMC_NS_APWPROT0_APPROEN8_Pos)           /*!< FMC_NS_T::FMC_APWPROT0: APPROEN8 Mask         */

#define FMC_NS_APWPROT0_APPROEN9_Pos    (9)                                               /*!< FMC_NS_T::FMC_APWPROT0: APPROEN9 Position     */
#define FMC_NS_APWPROT0_APPROEN9_Msk    (0x1ul << FMC_NS_APWPROT0_APPROEN9_Pos)           /*!< FMC_NS_T::FMC_APWPROT0: APPROEN9 Mask         */

#define FMC_NS_APWPROT0_APPROEN10_Pos   (10)                                              /*!< FMC_NS_T::FMC_APWPROT0: APPROEN10 Position    */
#define FMC_NS_APWPROT0_APPROEN10_Msk   (0x1ul << FMC_NS_APWPROT0_APPROEN10_Pos)          /*!< FMC_NS_T::FMC_APWPROT0: APPROEN10 Mask        */

#define FMC_NS_APWPROT0_APPROEN11_Pos   (11)                                              /*!< FMC_NS_T::FMC_APWPROT0: APPROEN11 Position    */
#define FMC_NS_APWPROT0_APPROEN11_Msk   (0x1ul << FMC_NS_APWPROT0_APPROEN11_Pos)          /*!< FMC_NS_T::FMC_APWPROT0: APPROEN11 Mask        */

#define FMC_NS_APWPROT0_APPROEN12_Pos   (12)                                              /*!< FMC_NS_T::FMC_APWPROT0: APPROEN12 Position    */
#define FMC_NS_APWPROT0_APPROEN12_Msk   (0x1ul << FMC_NS_APWPROT0_APPROEN12_Pos)          /*!< FMC_NS_T::FMC_APWPROT0: APPROEN12 Mask        */

#define FMC_NS_APWPROT0_APPROEN13_Pos   (13)                                              /*!< FMC_NS_T::FMC_APWPROT0: APPROEN13 Position    */
#define FMC_NS_APWPROT0_APPROEN13_Msk   (0x1ul << FMC_NS_APWPROT0_APPROEN13_Pos)          /*!< FMC_NS_T::FMC_APWPROT0: APPROEN13 Mask        */

#define FMC_NS_APWPROT0_APPROEN14_Pos   (14)                                              /*!< FMC_NS_T::FMC_APWPROT0: APPROEN14 Position    */
#define FMC_NS_APWPROT0_APPROEN14_Msk   (0x1ul << FMC_NS_APWPROT0_APPROEN14_Pos)          /*!< FMC_NS_T::FMC_APWPROT0: APPROEN14 Mask        */

#define FMC_NS_APWPROT0_APPROEN15_Pos   (15)                                              /*!< FMC_NS_T::FMC_APWPROT0: APPROEN15 Position    */
#define FMC_NS_APWPROT0_APPROEN15_Msk   (0x1ul << FMC_NS_APWPROT0_APPROEN15_Pos)          /*!< FMC_NS_T::FMC_APWPROT0: APPROEN15 Mask        */

#define FMC_NS_APWPROT1_APPROEN32_Pos   (0)                                               /*!< FMC_NS_T::FMC_APWPROT1: APPROEN32 Position    */
#define FMC_NS_APWPROT1_APPROEN32_Msk   (0x1ul << FMC_NS_APWPROT1_APPROEN32_Pos)          /*!< FMC_NS_T::FMC_APWPROT1: APPROEN32 Mask        */

#define FMC_NS_APWPKEEP_APWPKEEP0_Pos   (0)                                               /*!< FMC_NS_T::FMC_APWPKEEP: APWPKEEP0 Position    */
#define FMC_NS_APWPKEEP_APWPKEEP0_Msk   (0xfffful << FMC_NS_APWPKEEP_APWPKEEP0_Pos)       /*!< FMC_NS_T::FMC_APWPKEEP: APWPKEEP0 Mask        */

#define FMC_NS_SCACT_SCACT_Pos          (0)                                               /*!< FMC_NS_T::FMC_SCACT: SCACT Position           */
#define FMC_NS_SCACT_SCACT_Msk          (0x1ul << FMC_NS_SCACT_SCACT_Pos)                 /*!< FMC_NS_T::FMC_SCACT: SCACT Mask               */

/**@}*/ /* FMC_NS_CONST */
/**@}*/ /* end of FMC_NSBA register group */


/**@}*/ /* end of REGISTER group */
