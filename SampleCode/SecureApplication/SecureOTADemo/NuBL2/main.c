/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to generate the first booting image, NuBL2.
 *          After NuBL2 runs, NuBL2 will authenticate NuBL32 and NuBL33 then jump to execute in NuBL32.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "NuBL2.h"
#include "ota.h"

typedef void (*PFN_FUNC)(void);

static volatile ISP_INFO_T s_sNuBL2ISPInfo = { 0 };
// Initial Private and Public key for OTA Stage 1 Key Exchange Handshaking
// Command: ./imgtool.exe getpriv -k FwKey/OTA_ECC-P256.pem --minimal
const char gc_strOTA_PrivKey[] = "571f62dcd7478f6969c6afc0676806c82264653967af552b14bc949d1de887e6";
// Command: ./imgtool.exe getpub -k FwKey/OTA_ECC-P256.pem > OTA_ECC-P256.pub
// Copy public key to OTA_License.txt in plain text and pass to OTA Server APP
const char gc_strOTA_PubKey0[] = "161dfedda82f38223f6c12370778888b0712644923aa95f318173bd1b0188b60";
const char gc_strOTA_PubKey1[] = "035809a2f5644b6ee9b68d2c19c0ab60eb8101203d90dc7f8b96f928a20c66ce";

NVT_ITCM void BootNuBL32(uint32_t u32NuBL32Base)
{
    PFN_FUNC pfnNuBL32Entry;

    SYS_UnlockReg();
    FMC_Open();

    /* Disable all interrupt */
    __set_PRIMASK(1);

    /* SCB.VTOR points to the NuBL32 vector table base address. */
    SCB->VTOR = u32NuBL32Base;

    /* 2nd entry contains the address of the Reset_Handler (CMSIS-CORE) function */
    pfnNuBL32Entry = ((PFN_FUNC)(*(((uint32_t *)SCB->VTOR) + 1)));
    /* execute NuBL32 FW */
    pfnNuBL32Entry();
}

static int32_t CheckROTPKStatus(void)
{
    /* Configure module clock */
    CLK_EnableModuleClock(KS0_MODULE);

    KS_Open();

    if ((KS->OTPSTS & 0x3) == 0)
    {
        printf("ROTPK absent ! NuBL2 execution via direct boot.\n\n");

        // TODO: User must set ROTPK in KS OTP0 and OTP1
        //       and then the Secure Bootloader (NuBL1) can perform trusted boot to execute NuBL2.

        /*
            The NuBL2 ROTPK is a ECC P-256 public key pair.
            ECC P-256 private key file of this sample is FwKey/ECC-P256.pem.
            The public key pair can be generated with command - "./imgtool.exe getpub -k FwKey/ECC-P256.pem"
            const unsigned char ecdsa_pub_key[] = {
                0x30, 0x59, 0x30, 0x13, 0x06, 0x07, 0x2a, 0x86,
                0x48, 0xce, 0x3d, 0x02, 0x01, 0x06, 0x08, 0x2a,
                0x86, 0x48, 0xce, 0x3d, 0x03, 0x01, 0x07, 0x03,
                0x42, 0x00, 0x04, 0xd2, 0xc3, 0xe0, 0x3e, 0x54,
                0xc9, 0xcd, 0x32, 0xaa, 0x84, 0x70, 0xb6, 0x7e,
                0xe3, 0xaa, 0x81, 0x39, 0x2c, 0x2f, 0xa6, 0x71,
                0xea, 0x8e, 0xc3, 0x00, 0xa1, 0xd1, 0x9b, 0x1e,
                0xad, 0xbe, 0x65, 0x0e, 0x32, 0xb3, 0x3d, 0x83,
                0x28, 0x88, 0x09, 0x6a, 0xbf, 0x71, 0x0c, 0x08,
                0x2e, 0x46, 0xa2, 0x13, 0x16, 0x6f, 0x55, 0x76,
                0x49, 0xc7, 0x85, 0x9b, 0x0b, 0x60, 0x7a, 0x8d,
                0x34, 0x71, 0xc2,
            };

            Public key 1 starts from ecdsa_pub_key[27] and public key 2 starts from ecdsa_pub_key[59]
              Public key 1 = D2C3E03E 54C9CD32 AA8470B6 7EE3AA81 392C2FA6 71EA8EC3 00A1D19B 1EADBE65
              Public key 2 = 0E32B33D 83288809 6ABF710C 082E46A2 13166F55 7649C785 9B0B607A 8D3471C2

            Notes of programming NuBL2 ECC ROTPK:
            * The ROTPK arrary for KS_WriteOTP API
              !!! Please take care inversed order of public key in g_au32ROTPK !!!
                const uint32_t g_au32ROTPK[16] =
                {
                    // Public key 1
                    0x1EADBE65, 0x00A1D19B, 0x71EA8EC3, 0x392C2FA6,
                    0x7EE3AA81, 0xAA8470B6, 0x54C9CD32, 0xD2C3E03E,
                    // Public key 2
                    0x8D3471C2, 0x9B0B607A, 0x7649C785, 0x13166F55,
                    0x082E46A2, 0x6ABF710C, 0x83288809, 0x0E32B33D,
                };

            * User can write ECC-P256.pem through NuMicro ICP Programming Tool
              or use the C code example to program KS OPT0 and OPT1
                // Enable module clock
                CLK_EnableModuleClock(KS_MODULE);
                KS_Open();
                KS_WriteOTP(0, (KS_META_256 | KS_META_ECC | KS_META_BOOT), (uint32_t *)&g_au32ROTPK[0]);
                KS_WriteOTP(1, (KS_META_256 | KS_META_ECC | KS_META_BOOT), (uint32_t *)&g_au32ROTPK[8]);
        */
    }
    else
    {
        printf("ROTPK present. NuBL2 execution via secure boot.\n\n");
    }

    return 0;
}

uint32_t GetMaxAPROMSize(void)
{
    return FMC_APROM_END - FMC_APROM_BASE;
}

/**
  * @brief Check write address and size is valid
  * @param[in]  addr   Flash address
  * @param[in]  size   write size
  * @retval     0           Success
  * @retval     others      Failed
  * @details    Check write address and size is valid in system.
  */
int32_t _IsValidFlashRegion(uint32_t u32Addr, uint32_t u32ByteSize, uint32_t u32CmdMask)
{
    NVT_UNUSED(u32CmdMask);
    printf("[Check flash region] Addr: 0x%x, Size: 0x%x.\n", u32Addr, u32ByteSize);

    /* Not use u32CmdMask */

    u32Addr &= ~NS_OFFSET;

    /* Check address and length */
    if (((u32Addr % 4) != 0) || (u32ByteSize == 0))
        return ERR_INVALID_ADDRESS;

    if ((u32Addr < GetMaxAPROMSize()) && (u32Addr >= FMC_APROM_BASE))
    {
        if (((u32Addr + u32ByteSize) > GetMaxAPROMSize()) || ((u32Addr + u32ByteSize) < u32Addr))
            return ERR_OVER_RANGE;

        FMC_ENABLE_AP_UPDATE();
    }
    else if ((u32Addr < FMC_LDROM_END) && (u32Addr >= FMC_LDROM_BASE))
    {
        if (((u32Addr + u32ByteSize) > FMC_LDROM_END) || ((u32Addr + u32ByteSize) < u32Addr))
            return ERR_OVER_RANGE;

        FMC_ENABLE_LD_UPDATE();
    }
    else
    {
        return ERR_OVER_RANGE;
    }

    return 0;
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable module clock */
    CLK_EnableModuleClock(ISP0_MODULE);
    CLK_EnableModuleClock(GPIOA_MODULE);
    /* Enable CRYPTO module clock */
    CLK_EnableModuleClock(CRYPTO0_MODULE);
    CLK_EnableModuleClock(TRNG0_MODULE);
    /* Reset CRYPTO module */
    SYS_ResetModule(SYS_CRYPTO0RST);
    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

#if defined( NUMAKER_BOARD )
    SET_GPIO_PA0();
    GPIO_SetPullCtl(PA, BIT0, GPIO_PUSEL_PULL_UP);
    GPIO_SetMode(PA, BIT0, GPIO_MODE_INPUT);
#endif

    /* Initial Random Number Generator */
    RNG_Open();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 *       APROM Layout
 * |---- 0x0010_0000 ----|
 * |                     |
 * |        NuBL2        |
 * |                     |
 * |---- 0x0014_0000 ----|
 * |                     |
 * |        NuBL32       |
 * |                     |
 * |---- 0x001A_0000 ----|
 * |                     |
 * |        NuBL33       |
 * |                     |
 * |---- 0x001F_FFFF ----|
 */
/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    int32_t  bNeedReset = FALSE;
    uint32_t u32NuBL32Base, u32NuBL33Base;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("\n\nCPU Clock: %d Hz\n", SystemCoreClock);
    printf("Flash Non-secure Base Address: 0x%08X\n", SCU_GET_FLASH_NS_BASE());
    printf("+----------------------------------+\n");
    printf("| M5531 Secure OTA Sample Code     |\n");
    printf("|   BL32 Base Address: 0x%08X  |\n", (uint32_t)NUBL32_FW_BASE);
    printf("|   BL33 Base Address: 0x%08X  |\n", (uint32_t)NUBL33_FW_BASE);
    printf("+----------------------------------+\n\n");
    printf(" * Compiler option for Wi-Fi module pin selection:\n");
#if defined( NUMAKER_BOARD )
    printf("     NUMAKER_BOARD: For NuMaker board.\n");
    printf(" * Force PA.0 to GND to enable OTA update process directly.\n");
#endif

    CheckROTPKStatus();
    SYS_UnlockReg();
    /* Enable crypto interrupt */
    NVIC_EnableIRQ(CRYPTO_IRQn);
    ECC_ENABLE_INT(CRYPTO);
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    if (bNeedReset)
        goto RESET;

    if (PA0 == 0)
        goto OTA_PROCESS;

    /* Authenticate NuBL32 FW */
    if (NuBL_VerifyNuBL3x(NUBL32_FW_BASE, NUBL32_FW_SIZE, &u32NuBL32Base) != 0)
    {
        FMC_Erase(NUBL32_FW_BASE);
        printf("\n\nNuBL2 verified NuBL32 FAIL !\n");
        goto VERIFY_FAIL;
    }

    printf("\nNuBL2 verified NuBL32 FW PASS.\n");

    /* Authenticate NuBL33 FW */
    if (NuBL_VerifyNuBL3x(NUBL33_FW_BASE, NUBL33_FW_SIZE, &u32NuBL33Base) != 0)
    {
        FMC_Erase(NUBL33_FW_BASE);
        printf("\n\nNuBL2 verified NuBL33 FAIL !\n");
        goto VERIFY_FAIL;
    }

    printf("\nNuBL2 verified NuBL33 FW PASS.\n");

    /* Jump to execute NuBL32 FW */
    printf("\nPress 'o' to enter OTA process or others to execute NuBL32@0x%08X.\n", u32NuBL32Base);

    if (getchar() != 'o')
        BootNuBL32(u32NuBL32Base);

VERIFY_FAIL:
OTA_PROCESS:
    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    printf("Start OTA process ...\n");
    /* Init Wi-Fi module and connect to OTA server */
    OTA_Init(__HSI, (ISP_INFO_T *)&s_sNuBL2ISPInfo);

    if (OTA_TaskProcess() == 0)
    {
        /* Check OTA status and re-boot for update firmware */
        bNeedReset = TRUE;
    }

RESET:

    if (bNeedReset)
    {
        UART_WAIT_TX_EMPTY(DEBUG_PORT);
        SYS_ResetChip();
    }

    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
