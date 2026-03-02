/******************************************************************************
 * @file     startup_m3331.c
 * @brief    CMSIS-Core(M) Device Startup File for
 *           Device m3331
 * @version  V1.0.0
 * @date     November 5, 2025
 ******************************************************************************/
/*
 * Copyright (c) 2009-2021 Arm Limited. All rights reserved.
 * Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "NuMicro.h"

/*---------------------------------------------------------------------------
  External References
 *---------------------------------------------------------------------------*/
extern uint32_t __INITIAL_SP;
extern uint32_t __STACK_LIMIT;
#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
extern uint64_t __STACK_SEAL;
#endif

extern __NO_RETURN void __PROGRAM_START(void);

/*---------------------------------------------------------------------------
  Internal References
 *---------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler  (void);
void Default_Handler(void);

/*---------------------------------------------------------------------------
  Exception / Interrupt Handler
 *---------------------------------------------------------------------------*/
/* Exceptions */
void NMI_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler      (void) __attribute__ ((weak));
void MemManage_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));
void SecureFault_Handler    (void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler        (void) __attribute__ ((weak, alias("Default_Handler")));

/* ToDo: Add your device specific interrupt handler */
void BOD_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void IRC_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void PWRWU_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void RAMPE_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void CKFAIL_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void ISP_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void WDT0_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void WWDT0_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void EINT0_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void EINT1_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void EINT2_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void EINT3_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void EINT4_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void EINT5_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void GPA_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void GPB_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void GPC_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void GPD_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void GPE_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void GPF_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void QSPI0_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI0_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void BRAKE0_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void EPWM0P0_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void EPWM0P1_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void EPWM0P2_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void BRAKE1_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void EPWM1P0_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void EPWM1P1_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void EPWM1P2_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void TMR0_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void TMR1_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void TMR2_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void TMR3_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void UART0_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void UART1_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C0_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void PDMA0_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void EADC00_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void EADC01_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void ACMP01_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void EADC02_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void EADC03_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void UART2_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void UART3_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI1_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI2_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void OHCI_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void ETI_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void CRC_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void NS_ISP_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void SCU_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void SDH0_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void USBD20_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void WDT1_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void I2S0_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void GPG_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void EINT6_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void UART4_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void USCI0_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void USCI1_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void BPWM0_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void BPWM1_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void EQEI0_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void ECAP0_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void GPH_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void EINT7_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void WWDT1_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void EHCI_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void USBOTG20_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void NS_RAMPE_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void PDMA1_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void I3C0_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void CANFD00_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void CANFD01_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void CANFD10_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void CANFD11_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void PDCI_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void LLSI0_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void LLSI1_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void LLSI2_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void LLSI3_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void LLSI4_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void LLSI5_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void LLSI6_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void LLSI7_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void LLSI8_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void LLSI9_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void ELLSI0_IRQHandler  (void) __attribute__ ((weak, alias("Default_Handler")));
void BPWM2_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void BPWM3_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void BPWM4_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));
void BPWM5_IRQHandler   (void) __attribute__ ((weak, alias("Default_Handler")));


/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/

#if defined ( __GNUC__ )
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

/* ToDo: Add Cortex exception vectors according the used Cortex-Core */
extern const VECTOR_TABLE_Type __VECTOR_TABLE[161];
       const VECTOR_TABLE_Type __VECTOR_TABLE[161] __VECTOR_TABLE_ATTRIBUTE = {
  (VECTOR_TABLE_Type)(&__INITIAL_SP),  /*     Initial Stack Pointer */
  Reset_Handler,                       /*     Reset Handler */
  NMI_Handler,                         /* -14 NMI Handler */
  HardFault_Handler,                   /* -13 Hard Fault Handler */
  MemManage_Handler,                   /* -12 MPU Fault Handler */
  BusFault_Handler,                    /* -11 Bus Fault Handler */
  UsageFault_Handler,                  /* -10 Usage Fault Handler */
  SecureFault_Handler,                 /*  -9 Secure Fault Handler */
  0,                                   /*     Reserved */
  0,                                   /*     Reserved */
  0,                                   /*     Reserved */
  SVC_Handler,                         /*  -5 SVCall Handler */
  DebugMon_Handler,                    /*  -4 Debug Monitor Handler */
  0,                                   /*     Reserved */
  PendSV_Handler,                      /*  -2 PendSV Handler */
  SysTick_Handler,                     /*  -1 SysTick Handler */

  /* Interrupts */
  BOD_IRQHandler,                      /*   0: Brown Out detection */
  IRC_IRQHandler,                      /*   1: Internal RC */
  PWRWU_IRQHandler,                    /*   2: Power Down Wake Up */
  RAMPE_IRQHandler,                    /*   3: SRAM parity check failed */
  CKFAIL_IRQHandler,                   /*   4: Clock failed */
  ISP_IRQHandler,                      /*   5: FMC ISP */
  RTC_IRQHandler,                      /*   6: Real Time Clock */
  Default_Handler,                     /*   7: Reserved */
  WDT0_IRQHandler,                     /*   8: Watchdog timer 0 */
  WWDT0_IRQHandler,                    /*   9: Window Watchdog timer 0 */
  EINT0_IRQHandler,                    /*  10: External Input 0 */
  EINT1_IRQHandler,                    /*  11: External Input 1 */
  EINT2_IRQHandler,                    /*  12: External Input 2 */
  EINT3_IRQHandler,                    /*  13: External Input 3 */
  EINT4_IRQHandler,                    /*  14: External Input 4 */
  EINT5_IRQHandler,                    /*  15: External Input 5 */
  GPA_IRQHandler,                      /*  16: GPIO Port A */
  GPB_IRQHandler,                      /*  17: GPIO Port B */
  GPC_IRQHandler,                      /*  18: GPIO Port C */
  GPD_IRQHandler,                      /*  19: GPIO Port D */
  GPE_IRQHandler,                      /*  20: GPIO Port E */
  GPF_IRQHandler,                      /*  21: GPIO Port F */
  QSPI0_IRQHandler,                    /*  22: QSPI0 */
  SPI0_IRQHandler,                     /*  23: SPI0 */
  BRAKE0_IRQHandler,                   /*  24: BRAKE0 */
  EPWM0P0_IRQHandler,                  /*  25: EPWM0P0 */
  EPWM0P1_IRQHandler,                  /*  26: EPWM0P1 */
  EPWM0P2_IRQHandler,                  /*  27: EPWM0P2 */
  BRAKE1_IRQHandler,                   /*  28: EPWM1 brake */
  EPWM1P0_IRQHandler,                  /*  29: EPWM1 pair 0 */
  EPWM1P1_IRQHandler,                  /*  30: EPWM1 pair 1 */
  EPWM1P2_IRQHandler,                  /*  31: EPWM1 pair 2 */
  TMR0_IRQHandler,                     /*  32: Timer 0 */
  TMR1_IRQHandler,                     /*  33: Timer 1 */
  TMR2_IRQHandler,                     /*  34: Timer 2 */
  TMR3_IRQHandler,                     /*  35: Timer 3 */
  UART0_IRQHandler,                    /*  36: UART0 */
  UART1_IRQHandler,                    /*  37: UART1 */
  I2C0_IRQHandler,                     /*  38: I2C0 */
  I2C1_IRQHandler,                     /*  39: I2C1 */
  PDMA0_IRQHandler,                    /*  40: Peripheral DMA 0 */
  Default_Handler,                     /*  41: Default Handler */
  EADC00_IRQHandler,                   /*  42: EADC0 interrupt source 0 */
  EADC01_IRQHandler,                   /*  43: EADC0 interrupt source 1 */
  ACMP01_IRQHandler,                   /*  44: ACMP0 and ACMP1 */
  Default_Handler,                     /*  45: Default Handler */
  EADC02_IRQHandler,                   /*  46: EADC0 interrupt source 2 */
  EADC03_IRQHandler,                   /*  47: EADC0 interrupt source 3 */
  UART2_IRQHandler,                    /*  48: UART2 */
  UART3_IRQHandler,                    /*  49: UART3 */
  Default_Handler,                     /*  50: Default Handler */
  SPI1_IRQHandler,                     /*  51: SPI1 */
  SPI2_IRQHandler,                     /*  52: SPI2 */
  Default_Handler,                     /*  53: Default Handler */
  OHCI_IRQHandler,                     /*  54: OHCI */
  Default_Handler,                     /*  55: Default Handler */
  ETI_IRQHandler,                      /*  56: ETI */
  CRC_IRQHandler,                      /*  57: CRC */
  Default_Handler,                     /*  58: Default Handler */
  NS_ISP_IRQHandler,                   /*  59: Non-secure ISP */
  SCU_IRQHandler,                      /*  60: SCU */
  Default_Handler,                     /*  61: Default Handler */
  Default_Handler,                     /*  62: Default Handler */
  Default_Handler,                     /*  63: Default Handler */
  SDH0_IRQHandler,                     /*  64: SDH0 */
  USBD20_IRQHandler,                   /*  65: USBD20 */
  Default_Handler,                     /*  66: Default Handler */
  WDT1_IRQHandler,                     /*  67: Watchdog timer 1 */
  I2S0_IRQHandler,                     /*  68: I2S0 */
  Default_Handler,                     /*  69: Default Handler */
  Default_Handler,                     /*  70: Default Handler */
  Default_Handler,                     /*  71: Default Handler */
  GPG_IRQHandler,                      /*  72: GPIO Port G */
  EINT6_IRQHandler,                    /*  73: External Input 6 */
  UART4_IRQHandler,                    /*  74: UART4 */
  Default_Handler,                     /*  75: Default Handler */
  USCI0_IRQHandler,                    /*  76: USCI0 */
  USCI1_IRQHandler,                    /*  77: USCI1 */
  BPWM0_IRQHandler,                    /*  78: BPWM0 */
  BPWM1_IRQHandler,                    /*  79: BPWM1 */
  Default_Handler,                     /*  80: Default Handler */
  Default_Handler,                     /*  81: Default Handler */
  I2C2_IRQHandler,                     /*  82: I2C2 */
  Default_Handler,                     /*  83: Default Handler */
  EQEI0_IRQHandler,                    /*  84: EQEI0 */
  Default_Handler,                     /*  85: Default Handler */
  ECAP0_IRQHandler,                    /*  86: ECAP0 */
  Default_Handler,                     /*  87: Default Handler */
  GPH_IRQHandler,                      /*  88: GPIO Port H */
  EINT7_IRQHandler,                    /*  89: External Input 7 */
  Default_Handler,                     /*  90: Default Handler */
  WWDT1_IRQHandler,                    /*  91: Window watchdog timer 1 */
  EHCI_IRQHandler,                     /*  92: EHCI */
  USBOTG20_IRQHandler,                 /*  93: HSOTG */
  Default_Handler,                     /*  94: Default Handler */
  Default_Handler,                     /*  95: Default Handler */
  NS_RAMPE_IRQHandler,                 /*  96: Non-secure RAM parity error */
  Default_Handler,                     /*  97: Default Handler */
  PDMA1_IRQHandler,                    /*  98: PDMA1 */
  Default_Handler,                     /*  99: Default Handler */
  Default_Handler,                     /* 100: Default Handler */
  Default_Handler,                     /* 101: Default Handler */
  Default_Handler,                     /* 102: Default Handler */
  Default_Handler,                     /* 103: Default Handler */
  Default_Handler,                     /* 104: Default Handler */
  Default_Handler,                     /* 105: Default Handler */
  Default_Handler,                     /* 106: Default Handler */
  Default_Handler,                     /* 107: Default Handler */
  I3C0_IRQHandler,                     /* 108: I3C0 */
  Default_Handler,                     /* 109: Default Handler */
  Default_Handler,                     /* 110: Default Handler */
  Default_Handler,                     /* 111: Default Handler */
  CANFD00_IRQHandler,                  /* 112: CANFD0 interrupt source 0 */
  CANFD01_IRQHandler,                  /* 113: CANFD0 interrupt source 1 */
  CANFD10_IRQHandler,                  /* 114: CANFD1 interrupt source 0 */
  CANFD11_IRQHandler,                  /* 115: CANFD1 interrupt source 1 */
  Default_Handler,                     /* 116: Default Handler */
  Default_Handler,                     /* 117: Default Handler */
  Default_Handler,                     /* 118: Default Handler */
  PDCI_IRQHandler,                     /* 119: PDCI interrupt */
  Default_Handler,                     /* 120: Default Handler */
  Default_Handler,                     /* 121: Default Handler */
  Default_Handler,                     /* 122: Default Handler */
  Default_Handler,                     /* 123: Default Handler */
  Default_Handler,                     /* 124: Default Handler */
  Default_Handler,                     /* 125: Default Handler */
  Default_Handler,                     /* 126: Default Handler */
  Default_Handler,                     /* 127: Default Handler */
  LLSI0_IRQHandler,                    /* 128: LLSI0 */
  LLSI1_IRQHandler,                    /* 129: LLSI1 */
  LLSI2_IRQHandler,                    /* 130: LLSI2 */
  LLSI3_IRQHandler,                    /* 131: LLSI3 */
  LLSI4_IRQHandler,                    /* 132: LLSI4 */
  LLSI5_IRQHandler,                    /* 133: LLSI5 */
  LLSI6_IRQHandler,                    /* 134: LLSI6 */
  LLSI7_IRQHandler,                    /* 135: LLSI7 */
  LLSI8_IRQHandler,                    /* 136: LLSI8 */
  LLSI9_IRQHandler,                    /* 137: LLSI9 */
  ELLSI0_IRQHandler,                   /* 138: ELLSI0 */
  BPWM2_IRQHandler,                    /* 139: BPWM2 */
  BPWM3_IRQHandler,                    /* 140: BPWM3 */
  BPWM4_IRQHandler,                    /* 141: BPWM4 */
  BPWM5_IRQHandler                     /* 142: BPWM5 */
};

#if defined ( __GNUC__ )
#pragma GCC diagnostic pop
#endif

/*---------------------------------------------------------------------------
  Reset Handler called on controller reset
 *---------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void)
{
  __set_PSP((uint32_t)(&__INITIAL_SP));

  __set_MSPLIM((uint32_t)(&__STACK_LIMIT));
  __set_PSPLIM((uint32_t)(&__STACK_LIMIT));

  SystemInit();                    /* CMSIS System Initialization */
  __PROGRAM_START();               /* Enter PreMain (C library entry point) */
}


#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif

/*---------------------------------------------------------------------------
  Hard Fault Handler
 *---------------------------------------------------------------------------*/
void HardFault_Handler(void)
{
  while(1);
}

/*---------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *---------------------------------------------------------------------------*/
void Default_Handler(void)
{
  while(1);
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic pop
#endif
