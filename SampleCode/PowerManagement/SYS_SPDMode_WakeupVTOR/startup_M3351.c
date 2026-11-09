/**************************************************************************//**
 * @file     startup_M3351.c
 * @version  V1.00
 * @brief    CMSIS Device Startup File for NuMicro M3351
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/


#include <inttypes.h>
#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
extern uint32_t __INITIAL_SP;
extern uint32_t __STACK_LIMIT;

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
    extern uint32_t __STACK_SEAL;
#endif

extern __NO_RETURN void __PROGRAM_START(void);

/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
uint32_t g_u32ReturnAddr   = 0;
uint32_t g_bReturnToAddr = FALSE;
__NO_RETURN void Reset_Handler(void);
void Default_Handler(void);
void SCU_IRQHandler(void);

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* Exceptions */
void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak));
void MemManage_Handler(void) __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SecureFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__((weak, alias("Default_Handler")));

void BODOUT_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void IRC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PWRWU_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SRAMPE_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CKFAIL_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ISP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RTC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void WDT0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void WWDT0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EINT0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EINT1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EINT2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EINT3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EINT4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EINT5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPA_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPB_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPD_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPE_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPF_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void QSPI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void BRAKE0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PWM0P0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PWM0P1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PWM0P2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void BRAKE1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PWM1P0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PWM1P1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PWM1P2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIMER0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIMER1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIMER2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIMER3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PDMA0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DAC0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EADC00_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EADC01_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ACMP01_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EADC02_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EADC03_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USBD_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USBH_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ETI_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CRC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void NS_ISP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
//void SCU_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void WDT1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CRYPTO_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EINT6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USCI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USCI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void BPWM0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void BPWM1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EQEI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EQEI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ECAP0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPH_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EINT7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void WWDT1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void NS_SRAMPE_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CACHE_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART8_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART9_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TRNG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UART7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EADC10_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EADC11_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EADC12_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EADC13_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I3C0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DFMC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CANFD00_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CANFD01_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CANFD10_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CANFD11_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void NS_DFMC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LLSI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LLSI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LLSI2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LLSI3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LLSI4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LLSI5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LLSI6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LLSI7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LLSI8_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LLSI9_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ELLSI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ELLSI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
#if defined ( __GNUC__ )
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpedantic"
#endif

/* Static vector table */
const VECTOR_TABLE_Type __VECTOR_TABLE[] __VECTOR_TABLE_ATTRIBUTE =
{
    (VECTOR_TABLE_Type)(&__INITIAL_SP),       /*!<      Initial Stack Pointer                            */
    Reset_Handler,                            /*!<      Reset Handler                                    */
    NMI_Handler,                              /*!<  -14 NMI Handler                                      */
    HardFault_Handler,                        /*!<  -13 Hard Fault Handler                               */
    MemManage_Handler,                        /*!<  -12 MPU Fault Handler                                */
    BusFault_Handler,                         /*!<  -11 Bus Fault Handler                                */
    UsageFault_Handler,                       /*!<  -10 Usage Fault Handler                              */
    SecureFault_Handler,                      /*!<   -9 Secure Fault Handler                             */
    NULL,                                     /*!<   -8 Reserved                                         */
    NULL,                                     /*!<   -7 Reserved                                         */
    NULL,                                     /*!<   -6 Reserved                                         */
    SVC_Handler,                              /*!<   -5 SVC Handler                                      */
    DebugMon_Handler,                         /*!<   -4 Debug Monitor Handler                            */
    NULL,                                     /*!<   -3 Reserved                                         */
    PendSV_Handler,                           /*!<   -2 PendSV Handler Handler                           */
    SysTick_Handler,                          /*!<   -1 SysTick Handler                                  */

    BODOUT_IRQHandler,                        /*!<    0 Brown-Out Low Voltage Detected Interrupt Handler */
    IRC_IRQHandler,                           /*!<    1 IRC TRIM Interrupt Handler                       */
    PWRWU_IRQHandler,                         /*!<    2 Power Mode Controller Interrupt Handler          */
    SRAMPE_IRQHandler,                        /*!<    3 SRAM Parity Check Error Interrupt Handler        */
    CKFAIL_IRQHandler,                        /*!<    4 Clock Fail Detected Interrupt Handler            */
    ISP_IRQHandler,                           /*!<    5 FMC ISP Interrupt Handler                        */
    RTC_IRQHandler,                           /*!<    6 Real Time Clock Interrupt Handler                */
    NULL,                                     /*!<    7 Reserved                                         */
    WDT0_IRQHandler,                          /*!<    8 Watchdog Timer 0 Interrupt Handler               */
    WWDT0_IRQHandler,                         /*!<    9 Window Watchdog Timer 0 Interrupt Handler        */

    EINT0_IRQHandler,                         /*!<   10 External Input 0 Interrupt Handler               */
    EINT1_IRQHandler,                         /*!<   11 External Input 1 Interrupt Handler               */
    EINT2_IRQHandler,                         /*!<   12 External Input 2 Interrupt Handler               */
    EINT3_IRQHandler,                         /*!<   13 External Input 3 Interrupt Handler               */
    EINT4_IRQHandler,                         /*!<   14 External Input 4 Interrupt Handler               */
    EINT5_IRQHandler,                         /*!<   15 External Input 5 Interrupt Handler               */
    GPA_IRQHandler,                           /*!<   16 GPIO Port A Interrupt Handler                    */
    GPB_IRQHandler,                           /*!<   17 GPIO Port B Interrupt Handler                    */
    GPC_IRQHandler,                           /*!<   18 GPIO Port C Interrupt Handler                    */
    GPD_IRQHandler,                           /*!<   19 GPIO Port D Interrupt Handler                    */

    GPE_IRQHandler,                           /*!<   20 GPIO Port E Interrupt Handler                    */
    GPF_IRQHandler,                           /*!<   21 GPIO Port F Interrupt Handler                    */
    QSPI0_IRQHandler,                         /*!<   22 QSPI0 Interrupt Handler                          */
    SPI0_IRQHandler,                          /*!<   23 SPI0 Interrupt Handler                           */
    BRAKE0_IRQHandler,                        /*!<   24 PWM0 Brake Interrupt Handler                     */
    PWM0P0_IRQHandler,                        /*!<   25 PWM0 Pair 0 Interrupt Handler                    */
    PWM0P1_IRQHandler,                        /*!<   26 PWM0 Pair 1 Interrupt Handler                    */
    PWM0P2_IRQHandler,                        /*!<   27 PWM0 Pair 2 Interrupt Handler                    */
    BRAKE1_IRQHandler,                        /*!<   28 PWM1 Brake Interrupt Handler                     */
    PWM1P0_IRQHandler,                        /*!<   29 PWM1 Pair 0 Interrupt Handler                    */

    PWM1P1_IRQHandler,                        /*!<   30 PWM1 Pair 1 Interrupt Handler                    */
    PWM1P2_IRQHandler,                        /*!<   31 PWM1 Pair 2 Interrupt Handler                    */
    TIMER0_IRQHandler,                        /*!<   32 Timer0 Interrupt Handler                         */
    TIMER1_IRQHandler,                        /*!<   33 Timer1 Interrupt Handler                         */
    TIMER2_IRQHandler,                        /*!<   34 Timer2 Interrupt Handler                         */
    TIMER3_IRQHandler,                        /*!<   35 Timer3 Interrupt Handler                         */
    UART0_IRQHandler,                         /*!<   36 UART0 Interrupt Handler                          */
    UART1_IRQHandler,                         /*!<   37 UART1 Interrupt Handler                          */
    I2C0_IRQHandler,                          /*!<   38 I2C0 Interrupt Handler                           */
    I2C1_IRQHandler,                          /*!<   39 I2C1 Interrupt Handler                           */

    PDMA0_IRQHandler,                         /*!<   40 PDMA0 Interrupt Handler                          */
    DAC0_IRQHandler,                          /*!<   41 DAC0 Interrupt Handler                           */
    EADC00_IRQHandler,                        /*!<   42 EADC0 Interrupt 0 Handler                        */
    EADC01_IRQHandler,                        /*!<   43 EADC0 Interrupt 1 Handler                        */
    ACMP01_IRQHandler,                        /*!<   44 ACMP0 and ACMP1 Interrupt Handler                */
    NULL,                                     /*!<   45 Reserved                                         */
    EADC02_IRQHandler,                        /*!<   46 EADC0 Interrupt 2 Handler                        */
    EADC03_IRQHandler,                        /*!<   47 EADC0 Interrupt 3 Handler                        */
    UART2_IRQHandler,                         /*!<   48 UART2 Interrupt Handler                          */
    UART3_IRQHandler,                         /*!<   49 UART3 Interrupt Handler                          */

    NULL,                                     /*!<   50 Reserved                                         */
    SPI1_IRQHandler,                          /*!<   51 SPI1 Interrupt Handler                           */
    NULL,                                     /*!<   52 Reserved                                         */
    USBD_IRQHandler,                          /*!<   53 USB Device Interrupt Handler                     */
    USBH_IRQHandler,                          /*!<   54 USB Host Interrupt Handler                       */
    NULL,                                     /*!<   55 Reserved                                         */
    ETI_IRQHandler,                           /*!<   56 ETI Interrupt Handler                            */
    CRC_IRQHandler,                           /*!<   57 CRC Interrupt Handler                            */
    NULL,                                     /*!<   58 Reserved                                         */
    NS_ISP_IRQHandler,                        /*!<   59 FMC_NS ISP Interrupt Handler                     */

    SCU_IRQHandler,                           /*!<   60 SCU Interrupt Handler                            */
    NULL,                                     /*!<   61 Reserved                                         */
    NULL,                                     /*!<   62 Reserved                                         */
    NULL,                                     /*!<   63 Reserved                                         */
    NULL,                                     /*!<   64 Reserved                                         */
    NULL,                                     /*!<   65 Reserved                                         */
    NULL,                                     /*!<   66 Reserved                                         */
    WDT1_IRQHandler,                          /*!<   67 Watchdog Timer 1 Interrupt Handler               */
    NULL,                                     /*!<   68 Reserved                                         */
    NULL,                                     /*!<   69 Reserved                                         */

    NULL,                                     /*!<   70 Reserved                                         */
    CRYPTO_IRQHandler,                        /*!<   71 CRYPTO Interrupt Handler                         */
    GPG_IRQHandler,                           /*!<   72 GPIO Port G Interrupt Handler                    */
    EINT6_IRQHandler,                         /*!<   73 External Input 6 Interrupt Handler               */
    UART4_IRQHandler,                         /*!<   74 UART4 Interrupt Handler                          */
    UART5_IRQHandler,                         /*!<   75 UART5 Interrupt Handler                          */
    USCI0_IRQHandler,                         /*!<   76 USCI0 Interrupt Handler                          */
    USCI1_IRQHandler,                         /*!<   77 USCI1 Interrupt Handler                          */
    BPWM0_IRQHandler,                         /*!<   78 BPWM0 Interrupt Handler                          */
    BPWM1_IRQHandler,                         /*!<   79 BPWM1 Interrupt Handler                          */

    NULL,                                     /*!<   80 Reserved                                         */
    NULL,                                     /*!<   81 Reserved                                         */
    I2C2_IRQHandler,                          /*!<   82 I2C2 Interrupt Handler                           */
    NULL,                                     /*!<   83 Reserved                                         */
    EQEI0_IRQHandler,                         /*!<   84 EQEI0 Interrupt Handler                          */
    EQEI1_IRQHandler,                         /*!<   85 EQEI1 Interrupt Handler                          */
    ECAP0_IRQHandler,                         /*!<   86 ECAP0 Interrupt Handler                          */
    NULL,                                     /*!<   87 Reserved                                         */
    GPH_IRQHandler,                           /*!<   88 GPIO Port H Interrupt Handler                    */
    EINT7_IRQHandler,                         /*!<   89 External Input 7 Interrupt Handler               */

    NULL,                                     /*!<   90 Reserved                                         */
    WWDT1_IRQHandler,                         /*!<   91 Window Watchdog Timer 1 Interrupt Handler        */
    NULL,                                     /*!<   92 Reserved                                         */
    NULL,                                     /*!<   93 Reserved                                         */
    NULL,                                     /*!<   94 Reserved                                         */
    NULL,                                     /*!<   95 Reserved                                         */
    NS_SRAMPE_IRQHandler,                     /*!<   96 NS SRAM parity check failed Interrupt Handler    */
    CACHE_IRQHandler,                         /*!<   97 Cache Interrupt Handler                          */
    NULL,                                     /*!<   98 Reserved                                         */
    UART8_IRQHandler,                         /*!<   99 UART8 Interrupt Handler                          */

    UART9_IRQHandler,                         /*!<  100 UART9 Interrupt Handler                          */
    TRNG_IRQHandler,                          /*!<  101 TRNG Interrupt Handler                           */
    UART6_IRQHandler,                         /*!<  102 UART6 Interrupt Handler                          */
    UART7_IRQHandler,                         /*!<  103 UART7 Interrupt Handler                          */
    EADC10_IRQHandler,                        /*!<  104 EADC1 Interrupt 0 Handler                        */
    EADC11_IRQHandler,                        /*!<  105 EADC1 Interrupt 1 Handler                        */
    EADC12_IRQHandler,                        /*!<  106 EADC1 Interrupt 2 Handler                        */
    EADC13_IRQHandler,                        /*!<  107 EADC1 Interrupt 3 Handler                        */
    I3C0_IRQHandler,                          /*!<  108 I3C0 Interrupt Handler                           */
    NULL,                                     /*!<  109 Reserved                                         */

    NULL,                                     /*!<  110 Reserved                                         */
    DFMC_IRQHandler,                          /*!<  111 DFMC Interrupt Handler                           */
    CANFD00_IRQHandler,                       /*!<  112 CANFD00 Interrupt Handler                        */
    CANFD01_IRQHandler,                       /*!<  113 CANFD01 Interrupt Handler                        */
    CANFD10_IRQHandler,                       /*!<  114 CANFD10 Interrupt Handler                        */
    CANFD11_IRQHandler,                       /*!<  115 CANFD11 Interrupt Handler                        */
    NULL,                                     /*!<  116 Reserved                                         */
    NULL,                                     /*!<  117 Reserved                                         */
    NULL,                                     /*!<  118 Reserved                                         */
    NS_DFMC_IRQHandler,                       /*!<  119 NS DFMC Interrupt Handler                        */

    NULL,                                     /*!<  120 Reserved                                         */
    NULL,                                     /*!<  121 Reserved                                         */
    NULL,                                     /*!<  122 Reserved                                         */
    NULL,                                     /*!<  123 Reserved                                         */
    NULL,                                     /*!<  124 Reserved                                         */
    NULL,                                     /*!<  125 Reserved                                         */
    NULL,                                     /*!<  126 Reserved                                         */
    NULL,                                     /*!<  127 Reserved                                         */
    LLSI0_IRQHandler,                         /*!<  128 LLSI0 Interrupt Handler                          */
    LLSI1_IRQHandler,                         /*!<  129 LLSI1 Interrupt Handler                          */

    LLSI2_IRQHandler,                         /*!<  130 LLSI2 Interrupt Handler                          */
    LLSI3_IRQHandler,                         /*!<  131 LLSI3 Interrupt Handler                          */
    LLSI4_IRQHandler,                         /*!<  132 LLSI4 Interrupt Handler                          */
    LLSI5_IRQHandler,                         /*!<  133 LLSI5 Interrupt Handler                          */
    LLSI6_IRQHandler,                         /*!<  134 LLSI6 Interrupt Handler                          */
    LLSI7_IRQHandler,                         /*!<  135 LLSI7 Interrupt Handler                          */
    LLSI8_IRQHandler,                         /*!<  136 LLSI8 Interrupt Handler                          */
    LLSI9_IRQHandler,                         /*!<  137 LLSI9 Interrupt Handler                          */
    ELLSI0_IRQHandler,                        /*!<  138 ELLSI0 Interrupt Handler                         */
    ELLSI1_IRQHandler,                        /*!<  139 ELLSI1 Interrupt Handler                         */
};

void SYS_Init(void);

#if defined ( __GNUC__ )
    #pragma GCC diagnostic pop
#endif

/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void)
{
    __set_PSP((uint32_t)(&__INITIAL_SP));

    __set_MSPLIM((uint32_t)(&__STACK_LIMIT));
    __set_PSPLIM((uint32_t)(&__STACK_LIMIT));

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
    __TZ_set_STACKSEAL_S((uint32_t *)(&__STACK_SEAL));
#endif

    if (CLK->PMUSTS)
    {
        /* Init System, IP clock and multi-function I/O */
        SYS_Init();

        /* Init Debug UART to 115200-8N1 for print message */
        InitDebugUart();

        /* Power-down wake-up is set => Try to return to address before power-down */
        /* Clear all wake-up flag */
        CLK->PMUSTS |= CLK_PMUSTS_CLRWK_Msk;

        /* Clear return flag */
        g_bReturnToAddr = FALSE;
        /* Replaced LR with saved return address before reset */
        __ASM volatile("MOV lr, %0" : : "r"(g_u32ReturnAddr) :);
        __ASM volatile("MOV r0, #1");
        __ASM volatile("B SPDMode_WakeupAndReturn \n");
    }
    else
    {
        SystemInit();
        __PROGRAM_START();                        /* Enter PreMain (C library entry point) */
    }

    /* To avoid warning: function declared 'noreturn' should not return */
    while (1) ;
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif

__attribute__((noinline))
void SPDMode_WakeupAndReturn(uint32_t bReturn)
{
    if (bReturn == FALSE)
    {
        uint32_t u32TimeOutCnt;

        /* Save return address in g_u32ReturnAddr to return after reset */
        __ASM volatile("mov %0, lr\n" : "=r"(g_u32ReturnAddr));


        printf("[Before Power-down] LR: 0x%08X\n", g_u32ReturnAddr);
        g_bReturnToAddr = TRUE;

        printf("Enter to SPD Power-down mode...\n");

        /* Select Power-down mode */
        CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_SPD0);

        /* Enable wake-up timer and set wake-up Time-out Interval */
        CLK_SET_WKTMR_INTERVAL(CLK_PMUCTL_WKTMRIS_512);
        CLK_ENABLE_WKTMR();

        /* Check if all the debug messages are finished */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        UART_WAIT_TX_EMPTY(DEBUG_PORT)

        if (--u32TimeOutCnt == 0) break;

        /* Clear all wake-up flag */
        CLK->PMUSTS |= CLK_PMUSTS_CLRWK_Msk;

        /* Enter to Power-down mode */
        CLK_PowerDown();
    }
    else
    {
        printf("[After Reset] Resotred LR: 0x%08X\n", g_u32ReturnAddr);

        /* Clear return address */
        g_u32ReturnAddr = 0;
        printf("Continue execution ...\n");
    }
}
/*----------------------------------------------------------------------------
  Hard Fault Handler
 *----------------------------------------------------------------------------*/
extern void ProcessHardFault(uint32_t *excContext);

struct ExcContext
{
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t xPsr;
};

#if defined(__ICCARM__)
static inline void GetIRQnAndStack(uint32_t *pIrq, uint32_t **ppStackFrame)
{
    __asm volatile(
        "MRS   r2, IPSR       \n"   /* r2 = Exception number (including system exceptions)   */
        "SUB   r2, r2, #16    \n"   /* Convert to IRQn range (subtract 16 system exceptions) */
        "TST   lr, #4         \n"   /* EXC_RETURN bit2：0->MSP, 1->PSP                       */
        "ITE   EQ             \n"
        "MRSEQ r3, MSP        \n"   /* r3 = MSP                                              */
        "MRSNE r3, PSP        \n"   /* r3 = PSP                                              */
        "STR   r2, [r0]       \n"   /* *pIrq = r2                                            */
        "STR   r3, [r1]       \n"   /* *ppStackFrame = (uint32_t*)r3                         */
    );
}
#endif

__WEAK void HardFault_Handler(void)
{
    uint32_t u32IRQ = 0;
    struct StackFrame *psStackFrame = NULL;
    (void)u32IRQ;

#if defined(__ICCARM__)
    GetIRQnAndStack(&u32IRQ, (uint32_t **)&psStackFrame);
#else

    __ASM volatile("mrs %0, ipsr            \n" // Read IPSR (Exception number)
                   "sub %0, #16             \n" // Get it into IRQn_Type range
                   "tst lr, #4              \n" // Select the stack which was in use
                   "ite eq                  \n"
                   "mrseq %1, msp           \n"
                   "mrsne %1, psp           \n"
                   : "=r"(u32IRQ), "=r"(psStackFrame));
#endif
    // Get the instruction caused the hardfault
    ProcessHardFault((uint32_t *)psStackFrame);
}

/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void)
{
#if 0   // Uncomment to check unhandle IRQ
    uint32_t u32IPSR;

    u32IPSR = __get_IPSR();
    printf("\nUnexpected IRQ: %d\n", u32IPSR - 16);
#endif

    while (1);
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic pop
#endif
