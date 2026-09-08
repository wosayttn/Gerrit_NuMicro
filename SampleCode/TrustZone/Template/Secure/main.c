/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Secure sample code for TrustZone
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <arm_cmse.h>
#include <stdio.h>
#include "NuMicro.h"
#include "partition_M3351.h"
#include "lib/nsclib.h"

#define LOOP_HERE       0xE7FEE7FF      /* Instruction Code of "B ." */

static volatile uint32_t s_u32Ticks = 0;

int32_t LED_On(void);
int32_t LED_Off(void);
void SysTick_Handler(void);

/*---------------------------------------------------------------------------
 * Secure function exported to Non-secure application
 * Must decalre with __NONSECURE_ENTRY and place in Non-secure Callable region
 *---------------------------------------------------------------------------*/
__NONSECURE_ENTRY
int32_t SecureFunc(void)
{
    printf("Secure function call by Non-secure.\n");
    return 1;
}

__NONSECURE_ENTRY
int32_t Secure_LED_On(uint32_t u32Ticks)
{
    printf("[%d] Secure LED On call by Non-secure.\n", u32Ticks);
    PH4 = 0;
    return 0;
}

__NONSECURE_ENTRY
int32_t Secure_LED_Off(uint32_t u32Ticks)
{
    printf("[%d] Secure LED Off call by Non-secure.\n", u32Ticks);
    PH4 = 1;
    return 1;
}

/*---------------------------------------------------------------------------
 * For Non-secure code to register its callback function for Secure code
 *---------------------------------------------------------------------------*/
static PFN_NON_SECURE_FUNC pfNonSecure_LED_On  = (PFN_NON_SECURE_FUNC)NULL;
static PFN_NON_SECURE_FUNC pfNonSecure_LED_Off = (PFN_NON_SECURE_FUNC)NULL;

__NONSECURE_ENTRY
int32_t Secure_LED_On_Callback(PFN_NON_SECURE_FUNC pfnCallback)
{
    pfNonSecure_LED_On = (PFN_NON_SECURE_FUNC)cmse_nsfptr_create(pfnCallback);
    return 0;
}

__NONSECURE_ENTRY
int32_t Secure_LED_Off_Callback(PFN_NON_SECURE_FUNC pfnCallback)
{
    pfNonSecure_LED_Off = (PFN_NON_SECURE_FUNC)cmse_nsfptr_create(pfnCallback);
    return 0;
}

/*----------------------------------------------------------------------------
 * Secure LED control function
 *----------------------------------------------------------------------------*/
int32_t LED_On(void)
{
    printf("Secure LED On\n");
    PH5 = 0;
    return 1;
}

int32_t LED_Off(void)
{
    printf("Secure LED Off\n");
    PH5 = 1;
    return 1;
}

/*----------------------------------------------------------------------------
 * Helper functions to call Non-secure functions
 *----------------------------------------------------------------------------*/
__attribute__((noinline))
static void Call_NonSecure_LED_On(uint32_t u32Ticks)
{
    if (pfNonSecure_LED_On != NULL)
    {
        pfNonSecure_LED_On(u32Ticks);
    }
}

__attribute__((noinline))
static void Call_NonSecure_LED_Off(uint32_t u32Ticks)
{
    if (pfNonSecure_LED_Off != NULL)
    {
        pfNonSecure_LED_Off(u32Ticks);
    }
}

/*----------------------------------------------------------------------------
 * Secure SysTick IRQ Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    s_u32Ticks++;

    switch (s_u32Ticks)
    {
        case 100:
            LED_On();
            break;

        case 200:
            LED_Off();
            break;

        case 300:
            Call_NonSecure_LED_On(s_u32Ticks);
            break;

        case 400:
            Call_NonSecure_LED_Off(s_u32Ticks);
            break;

        default:
            if (s_u32Ticks > 600)
                s_u32Ticks = 0;

            break;
    }
}

void SYS_Init(void);
void Boot_NonSecure(uint32_t u32NonSecureBase);

/*---------------------------------------------------------------------------
 * Boot_NonSecure function is used to jump to Non-secure boot code.
 *---------------------------------------------------------------------------*/
void Boot_NonSecure(uint32_t u32NonSecureBase)
{
    PFN_NON_SECURE_FUNC pfnNonSecureEntry;

    /* SCB_NS.VTOR points to the Non-secure vector table base address. */
    SCB_NS->VTOR = u32NonSecureBase;

    /* 1st Entry in the vector table is the Non-secure Main Stack Pointer. */
    __TZ_set_MSP_NS(*((uint32_t *)SCB_NS->VTOR));      /* Set up MSP in Non-secure code */

    /* 2nd entry contains the address of the Reset_Handler (CMSIS-CORE) function */
    pfnNonSecureEntry = ((PFN_NON_SECURE_FUNC)(*(((uint32_t *)SCB_NS->VTOR) + 1)));

    /* Clear the LSB of the function address to indicate the function-call
       will cause a state switch from Secure to Non-secure */
    pfnNonSecureEntry = cmse_nsfptr_create(pfnNonSecureEntry);

    /* Check if the Reset_Handler address is in Non-secure space */
    if (cmse_is_nsfptr(pfnNonSecureEntry) && (((uint32_t)pfnNonSecureEntry & 0xF0000000) == NS_OFFSET))
    {
        printf("Execute Non-secure code ...\n");
        pfnNonSecureEntry(0);   /* Non-secure function entry */
    }
    else
    {
        /* Something went wrong */
        printf("No code in Non-secure region !\n");
        printf("CPU will halted at Non-secure state.\n");

        /* Set a temporary Non-secure MSP in Non-secure region here.
           Normally it should set to  1st entry in the vector table of Non-secure fw. */
        __TZ_set_MSP_NS(NON_SECURE_SRAM_BASE + 512);

        /* Try to halted in non-secure state (SRAM) */
        M32(NON_SECURE_SRAM_BASE) = LOOP_HERE;
        pfnNonSecureEntry = (PFN_NON_SECURE_FUNC)(NON_SECURE_SRAM_BASE + 1);
        pfnNonSecureEntry(0);

        while (1);
    }
}

/*----------------------------------------------------------------------------
 * Main function
 *----------------------------------------------------------------------------*/
int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("+----------------------------------+\n");
    printf("|      Secure code is running      |\n");
    printf("+----------------------------------+\n");
    /* Check Secure/Non-secure base address configuration */
    printf("NSCBA:     0x%08X\n", SCU->FNSADDR);
    printf("SRAMNSSET: 0x%08X\n", SCU->SRAMNSSET);

    /* Init GPIO Port H for Secure LED control */
    GPIO_SetMode(PH, BIT4 | BIT5, GPIO_MODE_OUTPUT);

    /* Generate Systick interrupt each 10 ms */
    SysTick_Config(SystemCoreClock / 100);

    /* Init and jump to Non-secure code */
    Boot_NonSecure(FMC_NON_SECURE_BASE);

    do
    {
        __WFI();
    } while (1);

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PCLK0 to HCLK/2 */
    CLK_SET_PCLK0DIV(CLK_PCLKDIV_APB0DIV_DIV2);
    /* Set PCLK1 to HCLK/2 */
    CLK_SET_PCLK1DIV(CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPH_MODULE);

    /* Enable debug UART module clock */
    SetDebugUartCLK();

    /* Init debug UART I/O Multi-function */
    SetDebugUartMFP();

    /*-----------------------------------------------------------------------
     * Init UART1 for Non-secure console and GPIO clock for Non-secure GPIO
     * (Configure Non-secure GPIO & UART1 to Non-secure in partition_M3351.h)
     *-----------------------------------------------------------------------*/
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
    SET_UART1_RXD_PA2();
    SET_UART1_TXD_PA3();

    CLK_EnableModuleClock(GPC_MODULE);
    SET_GPIO_PC0();
    SET_GPIO_PC1();

    /* Lock protected registers */
    SYS_LockReg();
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
