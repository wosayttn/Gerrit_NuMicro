# 1 "../targetdev.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 391 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "../targetdev.c" 2








# 1 "..\\targetdev.h" 1
# 10 "..\\targetdev.h"
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\NuMicro.h" 1
# 13 "../../../../Library/Device/Nuvoton/M2003J/Include\\NuMicro.h"
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 1
# 76 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h"
typedef enum IRQn
{

    NonMaskableInt_IRQn = -14,
    HardFault_IRQn = -13,
    SVCall_IRQn = -5,
    PendSV_IRQn = -2,
    SysTick_IRQn = -1,


    BOD_IRQn = 0,
    IRC_IRQn = 1,
    PWRWU_IRQn = 2,
    SR0ECC_IRQn = 3,
    CLKFAIL_IRQn = 4,
    FMC_IRQn = 5,
    RTC_IRQn = 6,
    SR1ECC_IRQn = 7,
    WDT_IRQn = 8,
    WWDT_IRQn = 9,
    EINT0_IRQn = 10,
    EINT1_IRQn = 11,
    EINT2_IRQn = 12,
    EINT3_IRQn = 13,
    EINT4_IRQn = 14,
    EINT5_IRQn = 15,
    GPA_IRQn = 16,
    GPB_IRQn = 17,
    GPC_IRQn = 18,
    GPD_IRQn = 19,
    GPE_IRQn = 20,
    GPF_IRQn = 21,
    TMR0_IRQn = 32,
    TMR1_IRQn = 33,
    TMR2_IRQn = 34,
    TMR3_IRQn = 35,
    UART0_IRQn = 36,
    UART1_IRQn = 37,
    I2C0_IRQn = 38,
    I2C1_IRQn = 39,
    PDMA0_IRQn = 40,
    ADC0_IRQn = 42,
    ADC_IRQn = 42,
    UART2_IRQn = 48,
    UART3_IRQn = 49,
    USCI0_IRQn = 52,
    USCI1_IRQn = 53,
    CRC_IRQn = 57,
    USCI2_IRQn = 61,
    USCI3_IRQn = 62,
    USCI4_IRQn = 63,
    TMR4_IRQn = 64,
    TMR5_IRQn = 65,
    TMR6_IRQn = 66,
    TMR7_IRQn = 67,
    TMR8_IRQn = 68,
    GPG_IRQn = 72,
    UART4_IRQn = 74,
    BPWM0_IRQn = 77,
    BPWM1_IRQn = 78,
    DFMC_IRQn = 81,
    I2C2_IRQn = 82,
} IRQn_Type;
# 151 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h"
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wc11-extensions"
#pragma clang diagnostic ignored "-Wreserved-id-macro"
# 178 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h"
# 1 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 1
# 27 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3







# 1 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\stdint.h" 1 3
# 56 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\stdint.h" 3
typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int int64_t;


typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long int uint64_t;





typedef signed char int_least8_t;
typedef signed short int int_least16_t;
typedef signed int int_least32_t;
typedef signed long long int int_least64_t;


typedef unsigned char uint_least8_t;
typedef unsigned short int uint_least16_t;
typedef unsigned int uint_least32_t;
typedef unsigned long long int uint_least64_t;




typedef signed int int_fast8_t;
typedef signed int int_fast16_t;
typedef signed int int_fast32_t;
typedef signed long long int int_fast64_t;


typedef unsigned int uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned int uint_fast32_t;
typedef unsigned long long int uint_fast64_t;






typedef signed int intptr_t;
typedef unsigned int uintptr_t;



typedef signed long long intmax_t;
typedef unsigned long long uintmax_t;
# 35 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 2 3
# 63 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
# 1 "../../../../Library/CMSIS/Core/Include\\cmsis_version.h" 1 3
# 27 "../../../../Library/CMSIS/Core/Include\\cmsis_version.h" 3
# 64 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 2 3
# 116 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
# 1 "../../../../Library/CMSIS/Core/Include\\cmsis_compiler.h" 1 3
# 32 "../../../../Library/CMSIS/Core/Include\\cmsis_compiler.h" 3
# 1 "../../../../Library/CMSIS/Core/Include\\cmsis_armclang.h" 1 3
# 29 "../../../../Library/CMSIS/Core/Include\\cmsis_armclang.h" 3


# 1 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\arm_acle.h" 1 3
# 45 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\arm_acle.h" 3
static __inline__ void __attribute__((__always_inline__, __nodebug__)) __wfi(void) {
  __builtin_arm_wfi();
}



static __inline__ void __attribute__((__always_inline__, __nodebug__)) __wfe(void) {
  __builtin_arm_wfe();
}



static __inline__ void __attribute__((__always_inline__, __nodebug__)) __sev(void) {
  __builtin_arm_sev();
}



static __inline__ void __attribute__((__always_inline__, __nodebug__)) __sevl(void) {
  __builtin_arm_sevl();
}



static __inline__ void __attribute__((__always_inline__, __nodebug__)) __yield(void) {
  __builtin_arm_yield();
}







static __inline__ uint32_t __attribute__((__always_inline__, __nodebug__))
__swp(uint32_t __x, volatile uint32_t *__p) {
  uint32_t v;
  do
    v = __builtin_arm_ldrex(__p);
  while (__builtin_arm_strex(__x, __p));
  return v;
}
# 113 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\arm_acle.h" 3
static __inline__ void __attribute__((__always_inline__, __nodebug__)) __nop(void) {
  __builtin_arm_nop();
}





static __inline__ uint32_t __attribute__((__always_inline__, __nodebug__))
__ror(uint32_t __x, uint32_t __y) {
  __y %= 32;
  if (__y == 0)
    return __x;
  return (__x >> __y) | (__x << (32 - __y));
}

static __inline__ uint64_t __attribute__((__always_inline__, __nodebug__))
__rorll(uint64_t __x, uint32_t __y) {
  __y %= 64;
  if (__y == 0)
    return __x;
  return (__x >> __y) | (__x << (64 - __y));
}

static __inline__ unsigned long __attribute__((__always_inline__, __nodebug__))
__rorl(unsigned long __x, uint32_t __y) {

  return __ror(__x, __y);



}



static __inline__ unsigned int __attribute__((__always_inline__, __nodebug__))
__clz(uint32_t __t) {
  return __builtin_arm_clz(__t);
}

static __inline__ unsigned int __attribute__((__always_inline__, __nodebug__))
__clzl(unsigned long __t) {

  return __builtin_arm_clz(__t);



}

static __inline__ unsigned int __attribute__((__always_inline__, __nodebug__))
__clzll(uint64_t __t) {
  return __builtin_arm_clz64(__t);
}


static __inline__ unsigned int __attribute__((__always_inline__, __nodebug__))
__cls(uint32_t __t) {
  return __builtin_arm_cls(__t);
}

static __inline__ unsigned int __attribute__((__always_inline__, __nodebug__))
__clsl(unsigned long __t) {

  return __builtin_arm_cls(__t);



}

static __inline__ unsigned int __attribute__((__always_inline__, __nodebug__))
__clsll(uint64_t __t) {
  return __builtin_arm_cls64(__t);
}


static __inline__ uint32_t __attribute__((__always_inline__, __nodebug__))
__rev(uint32_t __t) {
  return __builtin_bswap32(__t);
}

static __inline__ unsigned long __attribute__((__always_inline__, __nodebug__))
__revl(unsigned long __t) {

  return __builtin_bswap32(__t);



}

static __inline__ uint64_t __attribute__((__always_inline__, __nodebug__))
__revll(uint64_t __t) {
  return __builtin_bswap64(__t);
}


static __inline__ uint32_t __attribute__((__always_inline__, __nodebug__))
__rev16(uint32_t __t) {
  return __ror(__rev(__t), 16);
}

static __inline__ uint64_t __attribute__((__always_inline__, __nodebug__))
__rev16ll(uint64_t __t) {
  return (((uint64_t)__rev16(__t >> 32)) << 32) | (uint64_t)__rev16((uint32_t)__t);
}

static __inline__ unsigned long __attribute__((__always_inline__, __nodebug__))
__rev16l(unsigned long __t) {

    return __rev16(__t);



}


static __inline__ int16_t __attribute__((__always_inline__, __nodebug__))
__revsh(int16_t __t) {
  return (int16_t)__builtin_bswap16((uint16_t)__t);
}


static __inline__ uint32_t __attribute__((__always_inline__, __nodebug__))
__rbit(uint32_t __t) {
  return __builtin_arm_rbit(__t);
}

static __inline__ uint64_t __attribute__((__always_inline__, __nodebug__))
__rbitll(uint64_t __t) {

  return (((uint64_t)__builtin_arm_rbit(__t)) << 32) |
         __builtin_arm_rbit(__t >> 32);



}

static __inline__ unsigned long __attribute__((__always_inline__, __nodebug__))
__rbitl(unsigned long __t) {

  return __rbit(__t);



}
# 32 "../../../../Library/CMSIS/Core/Include\\cmsis_armclang.h" 2 3
# 71 "../../../../Library/CMSIS/Core/Include\\cmsis_armclang.h" 3
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpacked"
  struct __attribute__((packed, aligned(1))) T_UINT16_WRITE { uint16_t v; };
#pragma clang diagnostic pop



#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpacked"
  struct __attribute__((packed, aligned(1))) T_UINT16_READ { uint16_t v; };
#pragma clang diagnostic pop



#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpacked"
  struct __attribute__((packed, aligned(1))) T_UINT32_WRITE { uint32_t v; };
#pragma clang diagnostic pop



#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpacked"
  struct __attribute__((packed, aligned(1))) T_UINT32_READ { uint32_t v; };
#pragma clang diagnostic pop
# 282 "../../../../Library/CMSIS/Core/Include\\cmsis_armclang.h" 3
__attribute__((always_inline)) static inline int32_t __SSAT(int32_t val, uint32_t sat)
{
  if ((sat >= 1U) && (sat <= 32U))
  {
    const int32_t max = (int32_t)((1U << (sat - 1U)) - 1U);
    const int32_t min = -1 - max ;
    if (val > max)
    {
      return (max);
    }
    else if (val < min)
    {
      return (min);
    }
  }
  return (val);
}
# 308 "../../../../Library/CMSIS/Core/Include\\cmsis_armclang.h" 3
__attribute__((always_inline)) static inline uint32_t __USAT(int32_t val, uint32_t sat)
{
  if (sat <= 31U)
  {
    const uint32_t max = ((1U << sat) - 1U);
    if (val > (int32_t)max)
    {
      return (max);
    }
    else if (val < 0)
    {
      return (0U);
    }
  }
  return ((uint32_t)val);
}
# 470 "../../../../Library/CMSIS/Core/Include\\cmsis_armclang.h" 3
__attribute__((always_inline)) static inline uint8_t __LDAB(volatile uint8_t *ptr)
{
  uint32_t result;

  __asm volatile ("ldab %0, %1" : "=r" (result) : "Q" (*ptr) : "memory" );
  return ((uint8_t)result);
}
# 485 "../../../../Library/CMSIS/Core/Include\\cmsis_armclang.h" 3
__attribute__((always_inline)) static inline uint16_t __LDAH(volatile uint16_t *ptr)
{
  uint32_t result;

  __asm volatile ("ldah %0, %1" : "=r" (result) : "Q" (*ptr) : "memory" );
  return ((uint16_t)result);
}
# 500 "../../../../Library/CMSIS/Core/Include\\cmsis_armclang.h" 3
__attribute__((always_inline)) static inline uint32_t __LDA(volatile uint32_t *ptr)
{
  uint32_t result;

  __asm volatile ("lda %0, %1" : "=r" (result) : "Q" (*ptr) : "memory" );
  return (result);
}
# 515 "../../../../Library/CMSIS/Core/Include\\cmsis_armclang.h" 3
__attribute__((always_inline)) static inline void __STLB(uint8_t value, volatile uint8_t *ptr)
{
  __asm volatile ("stlb %1, %0" : "=Q" (*ptr) : "r" ((uint32_t)value) : "memory" );
}
# 527 "../../../../Library/CMSIS/Core/Include\\cmsis_armclang.h" 3
__attribute__((always_inline)) static inline void __STLH(uint16_t value, volatile uint16_t *ptr)
{
  __asm volatile ("stlh %1, %0" : "=Q" (*ptr) : "r" ((uint32_t)value) : "memory" );
}
# 539 "../../../../Library/CMSIS/Core/Include\\cmsis_armclang.h" 3
__attribute__((always_inline)) static inline void __STL(uint32_t value, volatile uint32_t *ptr)
{
  __asm volatile ("stl %1, %0" : "=Q" (*ptr) : "r" ((uint32_t)value) : "memory" );
}
# 621 "../../../../Library/CMSIS/Core/Include\\cmsis_armclang.h" 3
__attribute__((always_inline)) static inline void __enable_irq(void)
{
  __asm volatile ("cpsie i" : : : "memory");
}
# 634 "../../../../Library/CMSIS/Core/Include\\cmsis_armclang.h" 3
__attribute__((always_inline)) static inline void __disable_irq(void)
{
  __asm volatile ("cpsid i" : : : "memory");
}
# 670 "../../../../Library/CMSIS/Core/Include\\cmsis_armclang.h" 3
__attribute__((always_inline)) static inline uint32_t __get_FPSCR(void)
{



  return (0U);

}







__attribute__((always_inline)) static inline void __set_FPSCR(uint32_t fpscr)
{



  (void)fpscr;

}
# 702 "../../../../Library/CMSIS/Core/Include\\cmsis_armclang.h" 3
# 1 "../../../../Library/CMSIS/Core/Include\\./m-profile/cmsis_armclang_m.h" 1 3
# 27 "../../../../Library/CMSIS/Core/Include\\./m-profile/cmsis_armclang_m.h" 3
# 128 "../../../../Library/CMSIS/Core/Include\\./m-profile/cmsis_armclang_m.h" 3
__attribute__((always_inline)) static inline uint32_t __get_CONTROL(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, control" : "=r" (result) );
  return (result);
}
# 158 "../../../../Library/CMSIS/Core/Include\\./m-profile/cmsis_armclang_m.h" 3
__attribute__((always_inline)) static inline void __set_CONTROL(uint32_t control)
{
  __asm volatile ("MSR control, %0" : : "r" (control) : "memory");
  __builtin_arm_isb(0xF);
}
# 184 "../../../../Library/CMSIS/Core/Include\\./m-profile/cmsis_armclang_m.h" 3
__attribute__((always_inline)) static inline uint32_t __get_IPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, ipsr" : "=r" (result) );
  return (result);
}







__attribute__((always_inline)) static inline uint32_t __get_APSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, apsr" : "=r" (result) );
  return (result);
}







__attribute__((always_inline)) static inline uint32_t __get_xPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, xpsr" : "=r" (result) );
  return (result);
}







__attribute__((always_inline)) static inline uint32_t __get_PSP(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, psp" : "=r" (result) );
  return (result);
}
# 256 "../../../../Library/CMSIS/Core/Include\\./m-profile/cmsis_armclang_m.h" 3
__attribute__((always_inline)) static inline void __set_PSP(uint32_t topOfProcStack)
{
  __asm volatile ("MSR psp, %0" : : "r" (topOfProcStack) : );
}
# 280 "../../../../Library/CMSIS/Core/Include\\./m-profile/cmsis_armclang_m.h" 3
__attribute__((always_inline)) static inline uint32_t __get_MSP(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, msp" : "=r" (result) );
  return (result);
}
# 310 "../../../../Library/CMSIS/Core/Include\\./m-profile/cmsis_armclang_m.h" 3
__attribute__((always_inline)) static inline void __set_MSP(uint32_t topOfMainStack)
{
  __asm volatile ("MSR msp, %0" : : "r" (topOfMainStack) : );
}
# 361 "../../../../Library/CMSIS/Core/Include\\./m-profile/cmsis_armclang_m.h" 3
__attribute__((always_inline)) static inline uint32_t __get_PRIMASK(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, primask" : "=r" (result) );
  return (result);
}
# 391 "../../../../Library/CMSIS/Core/Include\\./m-profile/cmsis_armclang_m.h" 3
__attribute__((always_inline)) static inline void __set_PRIMASK(uint32_t priMask)
{
  __asm volatile ("MSR primask, %0" : : "r" (priMask) : "memory");
}
# 543 "../../../../Library/CMSIS/Core/Include\\./m-profile/cmsis_armclang_m.h" 3
__attribute__((always_inline)) static inline uint32_t __get_PSPLIM(void)
{




  return (0U);





}
# 590 "../../../../Library/CMSIS/Core/Include\\./m-profile/cmsis_armclang_m.h" 3
__attribute__((always_inline)) static inline void __set_PSPLIM(uint32_t ProcStackPtrLimit)
{




  (void)ProcStackPtrLimit;



}
# 633 "../../../../Library/CMSIS/Core/Include\\./m-profile/cmsis_armclang_m.h" 3
__attribute__((always_inline)) static inline uint32_t __get_MSPLIM(void)
{




  return (0U);





}
# 680 "../../../../Library/CMSIS/Core/Include\\./m-profile/cmsis_armclang_m.h" 3
__attribute__((always_inline)) static inline void __set_MSPLIM(uint32_t MainStackPtrLimit)
{




  (void)MainStackPtrLimit;



}
# 703 "../../../../Library/CMSIS/Core/Include\\cmsis_armclang.h" 2 3
# 33 "../../../../Library/CMSIS/Core/Include\\cmsis_compiler.h" 2 3
# 117 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 2 3
# 234 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
typedef union
{
  struct
  {
    uint32_t _reserved0:28;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} APSR_Type;
# 264 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:23;
  } b;
  uint32_t w;
} IPSR_Type;
# 282 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:15;
    uint32_t T:1;
    uint32_t _reserved1:3;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} xPSR_Type;
# 321 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
typedef union
{
  struct
  {
    uint32_t nPRIV:1;
    uint32_t SPSEL:1;
    uint32_t _reserved1:30;
  } b;
  uint32_t w;
} CONTROL_Type;
# 352 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
typedef struct
{
  volatile uint32_t ISER[16U];
        uint32_t RESERVED0[16U];
  volatile uint32_t ICER[16U];
        uint32_t RESERVED1[16U];
  volatile uint32_t ISPR[16U];
        uint32_t RESERVED2[16U];
  volatile uint32_t ICPR[16U];
        uint32_t RESERVED3[16U];
  volatile uint32_t IABR[16U];
        uint32_t RESERVED4[16U];
  volatile uint32_t ITNS[16U];
        uint32_t RESERVED5[16U];
  volatile uint32_t IPR[124U];
} NVIC_Type;
# 382 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
typedef struct
{
  volatile const uint32_t CPUID;
  volatile uint32_t ICSR;

  volatile uint32_t VTOR;



  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
        uint32_t RESERVED1;
  volatile uint32_t SHPR[2U];
  volatile uint32_t SHCSR;
} SCB_Type;
# 559 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile const uint32_t CALIB;
} SysTick_Type;
# 611 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t CYCCNT;
  volatile uint32_t CPICNT;
  volatile uint32_t EXCCNT;
  volatile uint32_t SLEEPCNT;
  volatile uint32_t LSUCNT;
  volatile uint32_t FOLDCNT;
  volatile const uint32_t PCSR;
  volatile uint32_t COMP0;
        uint32_t RESERVED1[1U];
  volatile uint32_t FUNCTION0;
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP1;
        uint32_t RESERVED3[1U];
  volatile uint32_t FUNCTION1;
        uint32_t RESERVED14[992U];
  volatile const uint32_t DEVARCH;
} DWT_Type;
# 677 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
typedef struct
{
  volatile const uint32_t SSPSR;
  volatile uint32_t CSPSR;
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;
        uint32_t RESERVED2[131U];
  volatile const uint32_t FFSR;
  volatile uint32_t FFCR;
  volatile uint32_t PSCR;
        uint32_t RESERVED3[759U];
  volatile const uint32_t TRIGGER;
  volatile const uint32_t ITFTTD0;
  volatile uint32_t ITATBCTR2;
        uint32_t RESERVED4[1U];
  volatile const uint32_t ITATBCTR0;
  volatile const uint32_t ITFTTD1;
  volatile uint32_t ITCTRL;
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;
  volatile uint32_t CLAIMCLR;
        uint32_t RESERVED7[8U];
  volatile const uint32_t DEVID;
  volatile const uint32_t DEVTYPE;
} TPIU_Type;
# 855 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
typedef struct
{
  volatile const uint32_t TYPE;
  volatile uint32_t CTRL;
  volatile uint32_t RNR;
  volatile uint32_t RBAR;
  volatile uint32_t RLAR;
        uint32_t RESERVED0[7U];
  union {
  volatile uint32_t MAIR[2];
  struct {
  volatile uint32_t MAIR0;
  volatile uint32_t MAIR1;
  };
  };
} MPU_Type;
# 1020 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
typedef struct
{
  volatile uint32_t DHCSR;
  volatile uint32_t DCRSR;
  volatile uint32_t DCRDR;
  volatile uint32_t DEMCR;
        uint32_t RESERVED0[1U];
  volatile uint32_t DAUTHCTRL;
  volatile uint32_t DSCSR;
} DCB_Type;
# 1131 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
typedef struct
{
  volatile uint32_t DLAR;
  volatile const uint32_t DLSR;
  volatile const uint32_t DAUTHSTATUS;
  volatile const uint32_t DDEVARCH;
  volatile const uint32_t DDEVTYPE;
} DIB_Type;
# 1290 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
typedef struct
{
  volatile uint32_t DHCSR;
  volatile uint32_t DCRSR;
  volatile uint32_t DCRDR;
  volatile uint32_t DEMCR;
        uint32_t RESERVED0[1U];
  volatile uint32_t DAUTHCTRL;
  volatile uint32_t DSCSR;
} CoreDebug_Type;
# 1480 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
static inline void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    __asm volatile("":::"memory");
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __asm volatile("":::"memory");
  }
}
# 1499 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
static inline uint32_t __NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}
# 1518 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
static inline void __NVIC_DisableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __builtin_arm_dsb(0xF);
    __builtin_arm_isb(0xF);
  }
}
# 1537 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
static inline uint32_t __NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}
# 1556 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
static inline void __NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}
# 1571 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
static inline void __NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}
# 1588 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
static inline uint32_t __NVIC_GetActive(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}
# 1677 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
static inline void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IPR[( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )] = ((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IPR[( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )] & ~(0xFFUL << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL))) |
       (((priority << (8U - 2U)) & (uint32_t)0xFFUL) << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL)));
  }
  else
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHPR[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] = ((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHPR[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] & ~(0xFFUL << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL))) |
       (((priority << (8U - 2U)) & (uint32_t)0xFFUL) << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL)));
  }
}
# 1701 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
static inline uint32_t __NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IPR[ ( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )] >> ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL) ) & (uint32_t)0xFFUL) >> (8U - 2U)));
  }
  else
  {
    return((uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHPR[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] >> ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL) ) & (uint32_t)0xFFUL) >> (8U - 2U)));
  }
}
# 1726 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
static inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(2U)) ? (uint32_t)(2U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits = ((PriorityGroupTmp + (uint32_t)(2U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(2U));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority & (uint32_t)((1UL << (SubPriorityBits )) - 1UL)))
         );
}
# 1753 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
static inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(2U)) ? (uint32_t)(2U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits = ((PriorityGroupTmp + (uint32_t)(2U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(2U));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority = (Priority ) & (uint32_t)((1UL << (SubPriorityBits )) - 1UL);
}
# 1777 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
static inline void __NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{

  uint32_t *vectors = (uint32_t *) ((uintptr_t) ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR);



  vectors[(int32_t)IRQn + 16] = vector;
  __builtin_arm_dsb(0xF);
}
# 1797 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
static inline uint32_t __NVIC_GetVector(IRQn_Type IRQn)
{

  uint32_t *vectors = (uint32_t *) ((uintptr_t) ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR);



  return vectors[(int32_t)IRQn + 16];
}






__attribute__((__noreturn__)) static inline void __NVIC_SystemReset(void)
{
  __builtin_arm_dsb(0xF);

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR = ((0x5FAUL << 16U) |
                 (1UL << 2U));
  __builtin_arm_dsb(0xF);

  for(;;)
  {
    __nop();
  }
}
# 2002 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
# 1 "../../../../Library/CMSIS/Core/Include\\m-profile/armv8m_mpu.h" 1 3
# 30 "../../../../Library/CMSIS/Core/Include\\m-profile/armv8m_mpu.h" 3
# 182 "../../../../Library/CMSIS/Core/Include\\m-profile/armv8m_mpu.h" 3
typedef struct {
  uint32_t RBAR;
  uint32_t RLAR;
} ARM_MPU_Region_t;





static inline uint32_t ARM_MPU_TYPE()
{
  return ((((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->TYPE) >> 8);
}




static inline void ARM_MPU_Enable(uint32_t MPU_Control)
{
  __builtin_arm_dmb(0xF);
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL = MPU_Control | (1UL );



  __builtin_arm_dsb(0xF);
  __builtin_arm_isb(0xF);
}



static inline void ARM_MPU_Disable(void)
{
  __builtin_arm_dmb(0xF);



  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL &= ~(1UL );
  __builtin_arm_dsb(0xF);
  __builtin_arm_isb(0xF);
}
# 257 "../../../../Library/CMSIS/Core/Include\\m-profile/armv8m_mpu.h" 3
static inline void ARM_MPU_SetMemAttrEx(MPU_Type* mpu, uint8_t idx, uint8_t attr)
{
  const uint8_t reg = idx / 4U;
  const uint32_t pos = ((idx % 4U) * 8U);
  const uint32_t mask = 0xFFU << pos;

  if (reg >= (sizeof(mpu->MAIR) / sizeof(mpu->MAIR[0]))) {
    return;
  }

  mpu->MAIR[reg] = ((mpu->MAIR[reg] & ~mask) | ((attr << pos) & mask));
}





static inline void ARM_MPU_SetMemAttr(uint8_t idx, uint8_t attr)
{
  ARM_MPU_SetMemAttrEx(((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) ), idx, attr);
}
# 294 "../../../../Library/CMSIS/Core/Include\\m-profile/armv8m_mpu.h" 3
static inline void ARM_MPU_ClrRegionEx(MPU_Type* mpu, uint32_t rnr)
{
  mpu->RNR = rnr;
  mpu->RLAR = 0U;
}




static inline void ARM_MPU_ClrRegion(uint32_t rnr)
{
  ARM_MPU_ClrRegionEx(((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) ), rnr);
}
# 324 "../../../../Library/CMSIS/Core/Include\\m-profile/armv8m_mpu.h" 3
static inline void ARM_MPU_SetRegionEx(MPU_Type* mpu, uint32_t rnr, uint32_t rbar, uint32_t rlar)
{
  mpu->RNR = rnr;
  mpu->RBAR = rbar;
  mpu->RLAR = rlar;
}






static inline void ARM_MPU_SetRegion(uint32_t rnr, uint32_t rbar, uint32_t rlar)
{
  ARM_MPU_SetRegionEx(((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) ), rnr, rbar, rlar);
}
# 358 "../../../../Library/CMSIS/Core/Include\\m-profile/armv8m_mpu.h" 3
static inline void ARM_MPU_OrderedMemcpy(volatile uint32_t* dst, const uint32_t* __restrict src, uint32_t len)
{
  uint32_t i;
  for (i = 0U; i < len; ++i)
  {
    dst[i] = src[i];
  }
}







static inline void ARM_MPU_LoadEx(MPU_Type* mpu, uint32_t rnr, ARM_MPU_Region_t const* table, uint32_t cnt)
{
  const uint32_t rowWordSize = sizeof(ARM_MPU_Region_t)/4U;
  if (cnt == 1U) {
    mpu->RNR = rnr;
    ARM_MPU_OrderedMemcpy(&(mpu->RBAR), &(table->RBAR), rowWordSize);
  } else {
    uint32_t rnrBase = rnr & ~(1U -1U);
    uint32_t rnrOffset = rnr % 1U;

    mpu->RNR = rnrBase;
    while ((rnrOffset + cnt) > 1U) {
      uint32_t c = 1U - rnrOffset;
      ARM_MPU_OrderedMemcpy(&(mpu->RBAR)+(rnrOffset*2U), &(table->RBAR), c*rowWordSize);
      table += c;
      cnt -= c;
      rnrOffset = 0U;
      rnrBase += 1U;
      mpu->RNR = rnrBase;
    }

    ARM_MPU_OrderedMemcpy(&(mpu->RBAR)+(rnrOffset*2U), &(table->RBAR), cnt*rowWordSize);
  }
}






static inline void ARM_MPU_Load(uint32_t rnr, ARM_MPU_Region_t const* table, uint32_t cnt)
{
  ARM_MPU_LoadEx(((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) ), rnr, table, cnt);
}
# 2003 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 2 3
# 2023 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
static inline uint32_t SCB_GetFPUType(void)
{
    return 0U;
}
# 2083 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
static inline void DCB_SetAuthCtrl(uint32_t value)
{
    __builtin_arm_dsb(0xF);
    __builtin_arm_isb(0xF);
    ((DCB_Type *) (0xE000EDF0UL) )->DAUTHCTRL = value;
    __builtin_arm_dsb(0xF);
    __builtin_arm_isb(0xF);
}







static inline uint32_t DCB_GetAuthCtrl(void)
{
    return (((DCB_Type *) (0xE000EDF0UL) )->DAUTHCTRL);
}
# 2150 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
static inline uint32_t DIB_GetAuthStatus(void)
{
    return (((DIB_Type *) (0xE000EFB0UL) )->DAUTHSTATUS);
}
# 2194 "../../../../Library/CMSIS/Core/Include\\core_cm23.h" 3
static inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = (uint32_t)(ticks - 1UL);
  __NVIC_SetPriority (SysTick_IRQn, (1UL << 2U) - 1UL);
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL = 0UL;
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2U) |
                   (1UL << 1U) |
                   (1UL );
  return (0UL);
}
# 179 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\system_M2003J.h" 1
# 36 "../../../../Library/Device/Nuvoton/M2003J/Include\\system_M2003J.h"
extern uint32_t SystemCoreClock;
extern uint32_t CyclesPerUs;
extern uint32_t PllClock;




typedef void(*VECTOR_TABLE_Type)(void);
# 55 "../../../../Library/Device/Nuvoton/M2003J/Include\\system_M2003J.h"
extern void SystemInit(void);
# 66 "../../../../Library/Device/Nuvoton/M2003J/Include\\system_M2003J.h"
extern void SystemCoreClockUpdate(void);
# 76 "../../../../Library/Device/Nuvoton/M2003J/Include\\system_M2003J.h"
extern void Uart0DefaultMPF(void);
# 88 "../../../../Library/Device/Nuvoton/M2003J/Include\\system_M2003J.h"
extern int IsDebugFifoEmpty(void);
# 180 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 190 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h"
extern void SystemInit(void);
# 203 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h"
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\adc_reg.h" 1
# 26 "../../../../Library/Device/Nuvoton/M2003J/Include\\adc_reg.h"
typedef struct
{
# 713 "../../../../Library/Device/Nuvoton/M2003J/Include\\adc_reg.h"
    volatile const uint32_t ADDR[30];
    volatile const uint32_t RESERVE1[2];
    volatile uint32_t ADCR;
    volatile uint32_t ADCHER;
    volatile uint32_t ADCMPR[2];
    volatile uint32_t ADSR0;
    volatile const uint32_t ADSR1;
    volatile const uint32_t ADSR2;
    volatile const uint32_t RESERVE2[1];
    volatile uint32_t ESMPCTL;
    volatile const uint32_t RESERVE3[23];
    volatile const uint32_t ADPDMA;
} ADC_T;
# 204 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\bpwm_reg.h" 1
# 26 "../../../../Library/Device/Nuvoton/M2003J/Include\\bpwm_reg.h"
typedef struct
{
# 44 "../../../../Library/Device/Nuvoton/M2003J/Include\\bpwm_reg.h"
    volatile uint32_t RCAPDAT;
    volatile uint32_t FCAPDAT;
} BCAPDAT_T;

typedef struct
{
# 1072 "../../../../Library/Device/Nuvoton/M2003J/Include\\bpwm_reg.h"
    volatile uint32_t CTL0;
    volatile uint32_t CTL1;

    volatile const uint32_t RESERVE0[2];

    volatile uint32_t CLKSRC;
    volatile uint32_t CLKPSC;

    volatile const uint32_t RESERVE1[2];

    volatile uint32_t CNTEN;
    volatile uint32_t CNTCLR;

    volatile const uint32_t RESERVE2[2];

    volatile uint32_t PERIOD;

    volatile const uint32_t RESERVE3[7];

    volatile uint32_t CMPDAT[6];

    volatile const uint32_t RESERVE4[10];

    volatile const uint32_t CNT;

    volatile const uint32_t RESERVE5[7];

    volatile uint32_t WGCTL0;
    volatile uint32_t WGCTL1;
    volatile uint32_t MSKEN;
    volatile uint32_t MSK;

    volatile const uint32_t RESERVE6[5];

    volatile uint32_t POLCTL;
    volatile uint32_t POEN;

    volatile const uint32_t RESERVE7[1];

    volatile uint32_t INTEN;

    volatile const uint32_t RESERVE8[1];

    volatile uint32_t INTSTS;

    volatile const uint32_t RESERVE9[3];

    volatile uint32_t ADCTS0;
    volatile uint32_t ADCTS1;

    volatile const uint32_t RESERVE10[4];

    volatile uint32_t SSCTL;
    volatile uint32_t SSTRG;

    volatile const uint32_t RESERVE11[2];

    volatile uint32_t STATUS;

    volatile const uint32_t RESERVE12[55];

    volatile uint32_t CAPINEN;
    volatile uint32_t CAPCTL;
    volatile const uint32_t CAPSTS;

    volatile const uint32_t RCAPDAT0;
    volatile const uint32_t FCAPDAT0;
    volatile const uint32_t RCAPDAT1;
    volatile const uint32_t FCAPDAT1;
    volatile const uint32_t RCAPDAT2;
    volatile const uint32_t FCAPDAT2;
    volatile const uint32_t RCAPDAT3;
    volatile const uint32_t FCAPDAT3;
    volatile const uint32_t RCAPDAT4;
    volatile const uint32_t FCAPDAT4;
    volatile const uint32_t RCAPDAT5;
    volatile const uint32_t FCAPDAT5;


    volatile const uint32_t RESERVE13[5];

    volatile uint32_t CAPIEN;
    volatile uint32_t CAPIF;

    volatile const uint32_t RESERVE14[43];

    volatile const uint32_t PBUF;

    volatile const uint32_t RESERVE15[5];

    volatile const uint32_t CMPBUF[6];

} BPWM_T;
# 205 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\clk_reg.h" 1
# 23 "../../../../Library/Device/Nuvoton/M2003J/Include\\clk_reg.h"
typedef struct
{
# 736 "../../../../Library/Device/Nuvoton/M2003J/Include\\clk_reg.h"
    volatile uint32_t SRCCTL;
    volatile const uint32_t STATUS;
    volatile uint32_t LXTCTL;
    volatile uint32_t HIRCCTL;
    volatile uint32_t HXTCTL;
    volatile const uint32_t RESERVE0[1];
    volatile uint32_t PLLCTL;
    volatile const uint32_t RESERVE1[3];
    volatile uint32_t CLKOCTL;
    volatile const uint32_t RESERVE2[1];
    volatile uint32_t CLKDCTL;
    volatile uint32_t CLKDSTS;
    volatile uint32_t CDUPB;
    volatile uint32_t CDLOWB;
    volatile const uint32_t RESERVE3[16];
    volatile uint32_t ADCCTL;
    volatile uint32_t BPWMCTL;
    volatile uint32_t CRCCTL;
    volatile uint32_t FMCCTL;
    volatile uint32_t GPIOCTL;
    volatile uint32_t I2CCTL;
    volatile uint32_t PDMACTL;
    volatile uint32_t RTCCTL;
    volatile uint32_t SRAMCTL;
    volatile uint32_t STCTL;
    volatile uint32_t TMRCTL;
    volatile uint32_t UARTCTL;
    volatile uint32_t USCICTL;
    volatile uint32_t WDTCTL;
    volatile uint32_t WWDTCTL;
    volatile const uint32_t RESERVE4[17];
    volatile uint32_t HCLKSEL;
    volatile uint32_t BPWMSEL;
    volatile uint32_t CLKOSEL;
    volatile uint32_t TMRSEL0;
    volatile uint32_t TMRSEL1;
    volatile uint32_t UARTSEL;
    volatile uint32_t WDTSEL;
    volatile uint32_t WWDTSEL;
    volatile const uint32_t RESERVE5[24];
    volatile uint32_t HCLKDIV;
    volatile uint32_t PCLKDIV;
    volatile const uint32_t RESERVE6[2];
    volatile uint32_t ADCDIV;
    volatile uint32_t UARTDIV;
} CLK_T;
# 206 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\crc_reg.h" 1
# 26 "../../../../Library/Device/Nuvoton/M2003J/Include\\crc_reg.h"
typedef struct
{
# 169 "../../../../Library/Device/Nuvoton/M2003J/Include\\crc_reg.h"
    volatile uint32_t CTL;
    volatile uint32_t DAT;
    volatile uint32_t SEED;
    volatile const uint32_t CHECKSUM;
    volatile uint32_t POLYNOMIAL;
    volatile const uint32_t RESERVE0[11];
    volatile uint32_t DMACTL;
    volatile uint32_t DMASTS;
    volatile uint32_t SADDR;
    volatile uint32_t DMACNT;
    volatile const uint32_t RESERVE1[1003];
    volatile const uint32_t VERSION;

} CRC_T;
# 207 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\fmc_reg.h" 1
# 26 "../../../../Library/Device/Nuvoton/M2003J/Include\\fmc_reg.h"
typedef struct
{
# 318 "../../../../Library/Device/Nuvoton/M2003J/Include\\fmc_reg.h"
    volatile uint32_t ISPCTL;
    volatile uint32_t ISPADDR;
    volatile uint32_t ISPDAT;
    volatile uint32_t ISPCMD;
    volatile uint32_t ISPTRG;
    volatile const uint32_t RESERVE0[2];
    volatile uint32_t ICPCTL;
    volatile const uint32_t RESERVE1[8];
    volatile uint32_t ISPSTS;
    volatile const uint32_t RESERVE2[2];
    volatile uint32_t CYCCTL;
    volatile const uint32_t RESERVE3[12];
    volatile uint32_t MPDAT0;
    volatile uint32_t MPDAT1;
    volatile uint32_t MPDAT2;
    volatile const uint32_t RESERVE4[41];
    volatile uint32_t ECCCTL;
    volatile uint32_t ECCSTS;
    volatile const uint32_t ECCSEFAR;
    volatile const uint32_t ECCDEFAR;
    volatile const uint32_t RESERVE5[943];
    volatile const uint32_t VERSION;

} FMC_T;
# 208 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\dfmc_reg.h" 1
# 26 "../../../../Library/Device/Nuvoton/M2003J/Include\\dfmc_reg.h"
typedef struct
{
# 257 "../../../../Library/Device/Nuvoton/M2003J/Include\\dfmc_reg.h"
    volatile uint32_t ISPCTL;
    volatile uint32_t ISPADDR;
    volatile uint32_t ISPDAT;
    volatile uint32_t ISPCMD;
    volatile uint32_t ISPTRG;
    volatile const uint32_t RESERVE0[2];
    volatile uint32_t ICPCTL;
    volatile const uint32_t RESERVE1[8];
    volatile uint32_t ISPSTS;
    volatile const uint32_t RESERVE2[2];
    volatile uint32_t CYCCTL;
    volatile const uint32_t RESERVE3[14];
    volatile uint32_t MPDAT2;
    volatile const uint32_t RESERVE4[41];
    volatile uint32_t ECCCTL;
    volatile uint32_t ECCSTS;
    volatile const uint32_t ECCSEFAR;
    volatile const uint32_t ECCDEFAR;
    volatile const uint32_t RESERVE5[943];
    volatile const uint32_t VERSION;

} DFMC_T;
# 209 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\gpio_reg.h" 1
# 25 "../../../../Library/Device/Nuvoton/M2003J/Include\\gpio_reg.h"
typedef struct
{
# 231 "../../../../Library/Device/Nuvoton/M2003J/Include\\gpio_reg.h"
    volatile uint32_t MODE;
    volatile uint32_t DINOFF;
    volatile uint32_t DOUT;
    volatile uint32_t DATMSK;
    volatile const uint32_t PIN;
    volatile uint32_t DBEN;
    volatile uint32_t INTTYPE;
    volatile uint32_t INTEN;
    volatile uint32_t INTSRC;
    volatile uint32_t SMTEN;
    volatile uint32_t SLEWCTL;
    volatile const uint32_t RESERVE0[1];
    volatile uint32_t PUSEL;
    volatile uint32_t DBCTL;
} GPIO_T;

typedef struct
{
# 299 "../../../../Library/Device/Nuvoton/M2003J/Include\\gpio_reg.h"
    volatile uint32_t INT0_INNF;
    volatile uint32_t INT0_EDETCTL;
    volatile uint32_t INT0_EDINTEN;
    volatile uint32_t INT0_EDSTS;
    volatile const uint32_t RESERVE0[4];
    volatile uint32_t INT1_INNF;
    volatile uint32_t INT1_EDETCTL;
    volatile uint32_t INT1_EDINTEN;
    volatile uint32_t INT1_EDSTS;
    volatile const uint32_t RESERVE1[4];
    volatile uint32_t INT2_INNF;
    volatile uint32_t INT2_EDETCTL;
    volatile uint32_t INT2_EDINTEN;
    volatile uint32_t INT2_EDSTS;
    volatile const uint32_t RESERVE2[4];
    volatile uint32_t INT3_INNF;
    volatile uint32_t INT3_EDETCTL;
    volatile uint32_t INT3_EDINTEN;
    volatile uint32_t INT3_EDSTS;
    volatile const uint32_t RESERVE3[4];
    volatile uint32_t INT4_INNF;
    volatile uint32_t INT4_EDETCTL;
    volatile uint32_t INT4_EDINTEN;
    volatile uint32_t INT4_EDSTS;
    volatile const uint32_t RESERVE4[4];
    volatile uint32_t INT5_INNF;
    volatile uint32_t INT5_EDETCTL;
    volatile uint32_t INT5_EDINTEN;
    volatile uint32_t INT5_EDSTS;

} GPIO_INT_T;
# 210 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\i2c_reg.h" 1
# 26 "../../../../Library/Device/Nuvoton/M2003J/Include\\i2c_reg.h"
typedef struct
{
# 374 "../../../../Library/Device/Nuvoton/M2003J/Include\\i2c_reg.h"
    volatile uint32_t CTL0;
    volatile uint32_t ADDR0;
    volatile uint32_t DAT;
    volatile const uint32_t STATUS0;
    volatile uint32_t CLKDIV;
    volatile uint32_t TOCTL;
    volatile uint32_t ADDR1;
    volatile uint32_t ADDR2;
    volatile uint32_t ADDR3;
    volatile uint32_t ADDRMSK0;
    volatile uint32_t ADDRMSK1;
    volatile uint32_t ADDRMSK2;
    volatile uint32_t ADDRMSK3;
    volatile const uint32_t RESERVE0[2];
    volatile uint32_t WKCTL;
    volatile uint32_t WKSTS;
    volatile uint32_t CTL1;
    volatile uint32_t STATUS1;
    volatile uint32_t TMCTL;
    volatile const uint32_t RESERVE1[1003];
    volatile const uint32_t VERSION;

} I2C_T;
# 211 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\pdma_reg.h" 1
# 27 "../../../../Library/Device/Nuvoton/M2003J/Include\\pdma_reg.h"
typedef struct
{
# 130 "../../../../Library/Device/Nuvoton/M2003J/Include\\pdma_reg.h"
    volatile uint32_t CTL;
    volatile uint32_t SA;
    volatile uint32_t DA;
    volatile uint32_t NEXT;

} DSCT_T;


typedef struct
{
# 686 "../../../../Library/Device/Nuvoton/M2003J/Include\\pdma_reg.h"
    DSCT_T DSCT[2];
    volatile const uint32_t RESERVE0[56];
    volatile const uint32_t CURSCAT[2];
    volatile const uint32_t RESERVE1[190];
    volatile uint32_t CHCTL;
    volatile uint32_t PAUSE;
    volatile uint32_t SWREQ;
    volatile const uint32_t TRGSTS;
    volatile uint32_t PRISET;
    volatile uint32_t PRICLR;
    volatile uint32_t INTEN;
    volatile uint32_t INTSTS;
    volatile uint32_t ABTSTS;
    volatile uint32_t TDSTS;
    volatile uint32_t ALIGN;
    volatile const uint32_t TACTSTS;
    volatile uint32_t TOUTPSC;
    volatile uint32_t TOUTEN;
    volatile uint32_t TOUTIEN;
    volatile uint32_t SCATBA;
    volatile uint32_t TOC;
    volatile const uint32_t RESERVE2[7];
    volatile uint32_t CHRST;
    volatile uint32_t SPI;
    volatile const uint32_t RESERVE3[6];
    volatile uint32_t REQSEL0_3;
    volatile const uint32_t RESERVE4[31];
} PDMA_T;
# 212 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\rtc_reg.h" 1
# 25 "../../../../Library/Device/Nuvoton/M2003J/Include\\rtc_reg.h"
typedef struct
{
# 286 "../../../../Library/Device/Nuvoton/M2003J/Include\\rtc_reg.h"
    volatile uint32_t INIT;
    volatile const uint32_t RESERVE0[1];
    volatile uint32_t FREQADJ;
    volatile uint32_t TIME;
    volatile uint32_t CAL;
    volatile uint32_t CLKFMT;
    volatile uint32_t WEEKDAY;
    volatile uint32_t TALM;
    volatile uint32_t CALM;
    volatile const uint32_t LEAPYEAR;
    volatile uint32_t INTEN;
    volatile uint32_t INTSTS;
    volatile uint32_t TICK;
    volatile uint32_t TAMSK;
    volatile uint32_t CAMSK;
    volatile const uint32_t RESERVE1[49];
    volatile uint32_t LXTCTL;
    volatile const uint32_t RESERVE2[3];
    volatile uint32_t DSTCTL;
    volatile const uint32_t RESERVE3[55];
    volatile uint32_t TEST;
    volatile const uint32_t RESERVE4[3];
    volatile uint32_t ACCCTL;
    volatile const uint32_t RESERVE5[894];
    volatile const uint32_t VERSION;

} RTC_T;
# 213 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\sys_reg.h" 1
# 23 "../../../../Library/Device/Nuvoton/M2003J/Include\\sys_reg.h"
typedef struct
{
# 1466 "../../../../Library/Device/Nuvoton/M2003J/Include\\sys_reg.h"
    volatile const uint32_t PDID;
    volatile uint32_t RSTCTL;
    volatile uint32_t RSTSTS;
    volatile uint32_t VTORSET;
    volatile const uint32_t RESERVE0[4];
    volatile uint32_t BODCTL;
    volatile uint32_t BODSTS;
    volatile uint32_t PORMKCTL;
    volatile uint32_t PORCTL;
    volatile uint32_t GPA_MFPL;
    volatile uint32_t GPA_MFPH;
    volatile uint32_t GPB_MFPL;
    volatile uint32_t GPB_MFPH;
    volatile uint32_t GPC_MFPL;
    volatile uint32_t GPC_MFPH;
    volatile uint32_t GPD_MFPL;
    volatile uint32_t GPD_MFPH;
    volatile uint32_t GPE_MFPL;
    volatile uint32_t GPE_MFPH;
    volatile uint32_t GPF_MFPL;
    volatile uint32_t GPF_MFPH;
    volatile const uint32_t RESERVE1[1];
    volatile uint32_t GPG_MFPH;
    volatile const uint32_t RESERVE2[6];
    volatile uint32_t GPA_MFOS;
    volatile uint32_t GPB_MFOS;
    volatile uint32_t GPC_MFOS;
    volatile uint32_t GPD_MFOS;
    volatile uint32_t GPE_MFOS;
    volatile uint32_t GPF_MFOS;
    volatile uint32_t GPG_MFOS;
    volatile const uint32_t RESERVE3[25];
    volatile uint32_t REGLCTL;
    volatile uint32_t CPUCFG;
    volatile const uint32_t RESERVE4[2];
    volatile uint32_t TCTLHIRC;
    volatile uint32_t TIENHIRC;
    volatile uint32_t TISTSHIRC;
    volatile const uint32_t RESERVE5[1];
    volatile uint32_t ADCRST;
    volatile uint32_t BPWMRST;
    volatile uint32_t CRCRST;
    volatile uint32_t FMCRST;
    volatile uint32_t GPIORST;
    volatile uint32_t I2CRST;
    volatile uint32_t PDMARST;
    volatile uint32_t RTCRST;
    volatile uint32_t TMRRST;
    volatile uint32_t UARTRST;
    volatile uint32_t USCIRST;
    volatile uint32_t WWDTRST;
    volatile const uint32_t RESERVE6[4];
    volatile uint32_t NMIEN;
    volatile const uint32_t NMISTS;
    volatile uint32_t NMIMSEL;
    volatile uint32_t AHBCTL;
    volatile uint32_t SRAMBCTL;
    volatile const uint32_t SRAMBFF;
    volatile const uint32_t SRAMBRF;
    volatile const uint32_t RESERVE7[1];
    volatile uint32_t SRAM0ICTL;
    volatile uint32_t SRAM0STS;
    volatile const uint32_t SRAM0EADR;
    volatile const uint32_t RESERVE8[1];
    volatile uint32_t SRAM1ICTL;
    volatile uint32_t SRAM1STS;
    volatile const uint32_t SRAM1EADR;
    volatile const uint32_t RESERVE9[25];
    volatile uint32_t INTEN;
    volatile uint32_t INTSTS;
    volatile const uint32_t RESERVE10[62];
    volatile uint32_t PWRCTL;
} SYS_T;
# 214 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\timer_reg.h" 1
# 26 "../../../../Library/Device/Nuvoton/M2003J/Include\\timer_reg.h"
typedef struct
{
# 578 "../../../../Library/Device/Nuvoton/M2003J/Include\\timer_reg.h"
    volatile uint32_t CTL;
    volatile uint32_t CMP;
    volatile uint32_t INTSTS;
    volatile uint32_t CNT;
    volatile const uint32_t CAP;
    volatile uint32_t EXTCTL;
    volatile uint32_t EINTSTS;
    volatile uint32_t TRGCTL;
    volatile const uint32_t RESERVE0[1];
    volatile uint32_t CAPNF;
    volatile const uint32_t RESERVE1[20];
    volatile const uint32_t CAP1;
    volatile const uint32_t CAP2;
    volatile const uint32_t CAP3;

} TIMER_T;
# 215 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\uart_reg.h" 1
# 26 "../../../../Library/Device/Nuvoton/M2003J/Include\\uart_reg.h"
typedef struct
{
# 1024 "../../../../Library/Device/Nuvoton/M2003J/Include\\uart_reg.h"
    volatile uint32_t DAT;
    volatile uint32_t INTEN;
    volatile uint32_t FIFO;
    volatile uint32_t LINE;
    volatile uint32_t MODEM;
    volatile uint32_t MODEMSTS;
    volatile uint32_t FIFOSTS;
    volatile uint32_t INTSTS;
    volatile uint32_t TOUT;
    volatile uint32_t BAUD;
    volatile uint32_t IRDA;
    volatile uint32_t ALTCTL;
    volatile uint32_t FUNCSEL;
    volatile uint32_t LINCTL;
    volatile uint32_t LINSTS;
    volatile uint32_t BRCOMP;
    volatile uint32_t WKCTL;
    volatile uint32_t WKSTS;
    volatile uint32_t DWKCOMP;
    volatile uint32_t RS485DD;
    volatile uint32_t LINRTOUT;
    volatile uint32_t LINWKCTL;
    volatile uint32_t AUTOCTL;
    volatile uint32_t AUTOSTS;
    volatile const uint32_t RESERVE2[999];
    volatile const uint32_t VERSION;

} UART_T;
# 216 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\uuart_reg.h" 1
# 26 "../../../../Library/Device/Nuvoton/M2003J/Include\\uuart_reg.h"
typedef struct
{
# 448 "../../../../Library/Device/Nuvoton/M2003J/Include\\uuart_reg.h"
    volatile uint32_t CTL;
    volatile uint32_t INTEN;
    volatile uint32_t BRGEN;
    volatile const uint32_t RESERVE0[1];
    volatile uint32_t DATIN0;
    volatile const uint32_t RESERVE1[3];
    volatile uint32_t CTLIN0;
    volatile const uint32_t RESERVE2[1];
    volatile uint32_t CLKIN;
    volatile uint32_t LINECTL;
    volatile uint32_t TXDAT;
    volatile const uint32_t RXDAT;
    volatile uint32_t BUFCTL;
    volatile uint32_t BUFSTS;
    volatile uint32_t PDMACTL;
    volatile const uint32_t RESERVE3[4];
    volatile uint32_t WKCTL;
    volatile uint32_t WKSTS;
    volatile uint32_t PROTCTL;
    volatile uint32_t PROTIEN;
    volatile uint32_t PROTSTS;
    volatile const uint32_t RESERVE4[7];
    volatile uint32_t IUR;
    volatile const uint32_t RESERVE16[3959];
    volatile const uint32_t VERSION;

} UUART_T;
# 217 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\ui2c_reg.h" 1
# 26 "../../../../Library/Device/Nuvoton/M2003J/Include\\ui2c_reg.h"
typedef struct
{
# 374 "../../../../Library/Device/Nuvoton/M2003J/Include\\ui2c_reg.h"
    volatile uint32_t CTL;

    volatile const uint32_t RESERVE0[1];

    volatile uint32_t BRGEN;

    volatile const uint32_t RESERVE1[8];

    volatile uint32_t LINECTL;
    volatile uint32_t TXDAT;
    volatile const uint32_t RXDAT;
    volatile uint32_t BUFCTL;
    volatile uint32_t BUFSTS;
    volatile const uint32_t RESERVE2[1];

    volatile uint32_t DEVADDR0;
    volatile uint32_t DEVADDR1;
    volatile uint32_t ADDRMSK0;
    volatile uint32_t ADDRMSK1;
    volatile uint32_t WKCTL;
    volatile uint32_t WKSTS;
    volatile uint32_t PROTCTL;
    volatile uint32_t PROTIEN;
    volatile uint32_t PROTSTS;

    volatile const uint32_t RESERVE3[8];

    volatile uint32_t ADMAT;
    volatile uint32_t TMCTL;

} UI2C_T;
# 218 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\uspi_reg.h" 1
# 26 "../../../../Library/Device/Nuvoton/M2003J/Include\\uspi_reg.h"
typedef struct
{
# 428 "../../../../Library/Device/Nuvoton/M2003J/Include\\uspi_reg.h"
    volatile uint32_t CTL;
    volatile uint32_t INTEN;
    volatile uint32_t BRGEN;

    volatile const uint32_t RESERVE0[1];

    volatile uint32_t DATIN0;

    volatile const uint32_t RESERVE1[3];

    volatile uint32_t CTLIN0;

    volatile const uint32_t RESERVE2[1];

    volatile uint32_t CLKIN;
    volatile uint32_t LINECTL;
    volatile uint32_t TXDAT;
    volatile const uint32_t RXDAT;
    volatile uint32_t BUFCTL;
    volatile uint32_t BUFSTS;
    volatile uint32_t PDMACTL;

    volatile const uint32_t RESERVE3[4];

    volatile uint32_t WKCTL;
    volatile uint32_t WKSTS;
    volatile uint32_t PROTCTL;
    volatile uint32_t PROTIEN;
    volatile uint32_t PROTSTS;
    volatile const uint32_t RESERVE4[7];
    volatile uint32_t IUR;

} USPI_T;
# 219 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\wdt_reg.h" 1
# 26 "../../../../Library/Device/Nuvoton/M2003J/Include\\wdt_reg.h"
typedef struct
{
# 140 "../../../../Library/Device/Nuvoton/M2003J/Include\\wdt_reg.h"
    volatile uint32_t CTL;
    volatile uint32_t ALTCTL;
    volatile uint32_t RSTCNT;
    volatile uint32_t STATUS;

} WDT_T;
# 220 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/Device/Nuvoton/M2003J/Include\\wwdt_reg.h" 1
# 26 "../../../../Library/Device/Nuvoton/M2003J/Include\\wwdt_reg.h"
typedef struct
{
# 101 "../../../../Library/Device/Nuvoton/M2003J/Include\\wwdt_reg.h"
    volatile uint32_t RLDCNT;
    volatile uint32_t CTL;
    volatile uint32_t STATUS;
    volatile const uint32_t CNT;

} WWDT_T;
# 221 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 376 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h"
#pragma clang diagnostic pop
# 401 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h"
typedef volatile unsigned char vu8;
typedef volatile unsigned long vu32;
typedef volatile unsigned short vu16;
# 601 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h"
# 1 "../../../../Library/StdDriver/inc\\adc.h" 1
# 98 "../../../../Library/StdDriver/inc\\adc.h"
extern int32_t g_ADC_i32ErrCode;
# 351 "../../../../Library/StdDriver/inc\\adc.h"
void ADC_Open(ADC_T *adc,
              uint32_t u32InputMode,
              uint32_t u32OpMode,
              uint32_t u32ChMask);
void ADC_Close(ADC_T *adc);
void ADC_EnableHWTrigger(ADC_T *adc,
                         uint32_t u32Source,
                         uint32_t u32Param);
void ADC_DisableHWTrigger(ADC_T *adc);
void ADC_EnableInt (ADC_T *adc, uint32_t u32Mask);
void ADC_DisableInt(ADC_T *adc, uint32_t u32Mask);
void ADC_SetExtendSampleTime(ADC_T *adc,
                             uint32_t u32ModuleNum,
                             uint32_t u32ExtendSampleTime);
uint32_t ADC_WaitPowerOnReady(ADC_T *adc);
# 602 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/StdDriver/inc\\bpwm.h" 1
# 385 "../../../../Library/StdDriver/inc\\bpwm.h"
uint32_t BPWM_ConfigCaptureChannel(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32UnitTimeNsec, uint32_t u32CaptureEdge);
uint32_t BPWM_ConfigOutputChannel(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Frequency, uint32_t u32DutyCycle);
void BPWM_Start(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_Stop(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_ForceStop(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableADCTrigger(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void BPWM_DisableADCTrigger(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearADCTriggerFlag(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t BPWM_GetADCTriggerFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableCapture(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_DisableCapture(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_DisableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableCaptureInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void BPWM_DisableCaptureInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void BPWM_ClearCaptureIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge);
uint32_t BPWM_GetCaptureIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void BPWM_DisableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32IntPeriodType);
void BPWM_DisablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableZeroInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_DisableZeroInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearZeroIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetZeroIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableLoadMode(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void BPWM_DisableLoadMode(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32LoadMode);
void BPWM_SetClockSource(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32ClkSrcSel);
uint32_t BPWM_GetWrapAroundFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearWrapAroundFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
# 603 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/StdDriver/inc\\clk.h" 1
# 443 "../../../../Library/StdDriver/inc\\clk.h"
extern int32_t g_CLK_i32ErrCode;
# 453 "../../../../Library/StdDriver/inc\\clk.h"
static inline void CLK_SysTickDelay(uint32_t us);
static inline void CLK_SysTickLongDelay(uint32_t us);
# 467 "../../../../Library/StdDriver/inc\\clk.h"
static inline void CLK_SysTickDelay(uint32_t us)
{
    ((CLK_T *) (((uint32_t)0x40000000UL) + 0x01000UL))->STCTL |= (0x1ul << (0));

    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = us * CyclesPerUs;
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL = (0x0UL);
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL );


    while((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL & (1UL << 16U)) == 0UL)
    {
    }


    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = 0UL;

    ((CLK_T *) (((uint32_t)0x40000000UL) + 0x01000UL))->STCTL &= ~(0x1ul << (0));
}
# 494 "../../../../Library/StdDriver/inc\\clk.h"
static inline void CLK_SysTickLongDelay(uint32_t us)
{
    uint32_t u32Delay;


    u32Delay = 65536UL;

    do
    {
        if(us > u32Delay)
        {
            us -= u32Delay;
        }
        else
        {
            u32Delay = us;
            us = 0UL;
        }

        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = u32Delay * CyclesPerUs;
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL = (0x0UL);
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL );


        while((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL & (1UL << 16U)) == 0UL);


        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = 0UL;

    }
    while(us > 0UL);
}


void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetHXTFreq(void);
uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetPCLK0Freq(void);
uint32_t CLK_GetPCLK1Freq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint64_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint64_t u32ModuleIdx);
void CLK_DisableModuleClock(uint64_t u32ModuleIdx);
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq);
void CLK_DisablePLL(void);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_DisableSysTick(void);
uint32_t CLK_GetPLLClockFreq(void);
uint32_t CLK_GetModuleClockSource(uint64_t u32ModuleIdx);
uint32_t CLK_GetModuleClockDivider(uint64_t u32ModuleIdx);
# 604 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/StdDriver/inc\\crc.h" 1
# 111 "../../../../Library/StdDriver/inc\\crc.h"
void CRC_Open(uint32_t u32Mode, uint32_t u32Attribute, uint32_t u32Seed, uint32_t u32DataLen);
uint32_t CRC_GetChecksum(void);
# 605 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/StdDriver/inc\\fmc.h" 1
# 333 "../../../../Library/StdDriver/inc\\fmc.h"
extern int32_t g_FMC_i32ErrCode;
# 343 "../../../../Library/StdDriver/inc\\fmc.h"
static inline uint32_t FMC_ReadCID(void);
static inline uint32_t FMC_ReadUID(uint8_t u8Index);
static inline uint32_t FMC_ReadUCID(uint32_t u32Index);
static inline int32_t FMC_SetVectorPageAddr(uint32_t u32PageAddr);
static inline uint32_t FMC_GetVECMAP(void);







static inline uint32_t FMC_GetVECMAP(void)
{
    return (((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPSTS & (0x7fffful << (9)));
}
# 369 "../../../../Library/StdDriver/inc\\fmc.h"
static inline uint32_t FMC_ReadCID(void)
{
    int32_t i32TimeOutCnt = (SystemCoreClock>>3);

    g_FMC_i32ErrCode = 0;

    ((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPCMD = 0x0BUL;
    ((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPADDR = 0x0u;
    ((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPTRG = (0x1ul << (0));
    while(((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPTRG & (0x1ul << (0)))
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = (-2L);
            return 0xFFFFFFFF;
        }
    }

    return ((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPDAT;
}
# 399 "../../../../Library/StdDriver/inc\\fmc.h"
static inline uint32_t FMC_ReadUID(uint8_t u8Index)
{
    int32_t i32TimeOutCnt = (SystemCoreClock>>3);

    g_FMC_i32ErrCode = 0;

    ((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPCMD = 0x04UL;
    ((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPADDR = ((uint32_t)u8Index << 2u);
    ((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPDAT = 0u;
    ((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPTRG = 0x1u;
    while(((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPTRG & (0x1ul << (0)))
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = (-2L);
            return 0xFFFFFFFF;
        }
    }

    return ((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPDAT;
}
# 430 "../../../../Library/StdDriver/inc\\fmc.h"
static inline uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    int32_t i32TimeOutCnt = (SystemCoreClock>>3);

    g_FMC_i32ErrCode = 0;

    ((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPCMD = 0x04UL;
    ((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPADDR = (0x04u * u32Index) + 0x10u;
    ((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPTRG = (0x1ul << (0));
    while(((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPTRG & (0x1ul << (0)))
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = (-2L);
            return 0xFFFFFFFF;
        }
    }

    return ((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPDAT;
}
# 462 "../../../../Library/StdDriver/inc\\fmc.h"
static inline int32_t FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    int32_t i32TimeOutCnt = (SystemCoreClock>>3);

    g_FMC_i32ErrCode = 0;

    ((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPCMD = 0x2EUL;
    ((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPADDR = u32PageAddr;
    ((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPTRG = 0x1u;
    while(((FMC_T *) (((uint32_t)0x40000000UL) + 0x0C000UL))->ISPTRG)
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_FMC_i32ErrCode = (-2L);
            return -1;
        }
    }

    return 0;
}






extern void FMC_Open(void);
extern void FMC_Close(void);
extern int32_t FMC_Erase(uint32_t u32PageAddr);
extern int32_t FMC_Erase_Bank(uint32_t u32BankAddr);
extern int32_t FMC_Erase_SPROM(void);
extern uint32_t FMC_Read(uint32_t u32Addr);
extern int32_t FMC_Read_64(uint32_t u32addr, uint32_t * u32data0, uint32_t * u32data1);
extern int32_t FMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern int32_t FMC_Write8Bytes(uint32_t u32addr, uint32_t u32data0, uint32_t u32data1);
extern int32_t FMC_ReadConfig(uint32_t u32Config[], uint32_t u32Count);
extern int32_t FMC_WriteConfig(uint32_t u32Config[], uint32_t u32Count);
extern uint32_t FMC_GetChkSum(uint32_t u32addr, uint32_t u32count);
extern uint32_t FMC_CheckAllOne(uint32_t u32addr, uint32_t u32count);
extern int32_t FMC_RemapBank(uint32_t u32Bank);
# 606 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/StdDriver/inc\\dfmc.h" 1
# 212 "../../../../Library/StdDriver/inc\\dfmc.h"
extern int32_t g_DFMC_i32ErrCode;
# 222 "../../../../Library/StdDriver/inc\\dfmc.h"
static inline uint32_t DFMC_ReadCID(void);
static inline uint32_t DFMC_ReadUID(uint8_t u8Index);
static inline uint32_t DFMC_ReadUCID(uint32_t u32Index);
# 236 "../../../../Library/StdDriver/inc\\dfmc.h"
static inline uint32_t DFMC_ReadCID(void)
{
    int32_t i32TimeOutCnt = (SystemCoreClock>>3);

    g_DFMC_i32ErrCode = 0;

    ((DFMC_T *) (((uint32_t)0x40000000UL) + 0x0F000UL))->ISPCMD = 0x0BUL;
    ((DFMC_T *) (((uint32_t)0x40000000UL) + 0x0F000UL))->ISPADDR = 0x0u;
    ((DFMC_T *) (((uint32_t)0x40000000UL) + 0x0F000UL))->ISPTRG = (0x1ul << (0));
    while(((DFMC_T *) (((uint32_t)0x40000000UL) + 0x0F000UL))->ISPTRG & (0x1ul << (0)))
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_DFMC_i32ErrCode = (-2L);
            return 0xFFFFFFFF;
        }
    }

    return ((DFMC_T *) (((uint32_t)0x40000000UL) + 0x0F000UL))->ISPDAT;
}
# 267 "../../../../Library/StdDriver/inc\\dfmc.h"
static inline uint32_t DFMC_ReadUID(uint8_t u8Index)
{
    int32_t i32TimeOutCnt = (SystemCoreClock>>3);

    g_DFMC_i32ErrCode = 0;

    ((DFMC_T *) (((uint32_t)0x40000000UL) + 0x0F000UL))->ISPCMD = 0x04UL;
    ((DFMC_T *) (((uint32_t)0x40000000UL) + 0x0F000UL))->ISPADDR = ((uint32_t)u8Index << 2u);
    ((DFMC_T *) (((uint32_t)0x40000000UL) + 0x0F000UL))->ISPDAT = 0u;
    ((DFMC_T *) (((uint32_t)0x40000000UL) + 0x0F000UL))->ISPTRG = 0x1u;
    while(((DFMC_T *) (((uint32_t)0x40000000UL) + 0x0F000UL))->ISPTRG & (0x1ul << (0)))
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_DFMC_i32ErrCode = (-2L);
            return 0xFFFFFFFF;
        }
    }

    return ((DFMC_T *) (((uint32_t)0x40000000UL) + 0x0F000UL))->ISPDAT;
}
# 299 "../../../../Library/StdDriver/inc\\dfmc.h"
static inline uint32_t DFMC_ReadUCID(uint32_t u32Index)
{
    int32_t i32TimeOutCnt = (SystemCoreClock>>3);

    g_DFMC_i32ErrCode = 0;

    ((DFMC_T *) (((uint32_t)0x40000000UL) + 0x0F000UL))->ISPCMD = 0x04UL;
    ((DFMC_T *) (((uint32_t)0x40000000UL) + 0x0F000UL))->ISPADDR = (0x04u * u32Index) + 0x10u;
    ((DFMC_T *) (((uint32_t)0x40000000UL) + 0x0F000UL))->ISPTRG = (0x1ul << (0));
    while(((DFMC_T *) (((uint32_t)0x40000000UL) + 0x0F000UL))->ISPTRG & (0x1ul << (0)))
    {
        if(i32TimeOutCnt-- <= 0)
        {
            g_DFMC_i32ErrCode = (-2L);
            return 0xFFFFFFFF;
        }
    }

    return ((DFMC_T *) (((uint32_t)0x40000000UL) + 0x0F000UL))->ISPDAT;
}





extern void DFMC_Close(void);
extern int32_t DFMC_Erase(uint32_t u32PageAddr);
extern void DFMC_Open(void);
extern uint32_t DFMC_Read(uint32_t u32Addr);
extern int32_t DFMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern uint32_t DFMC_GetChkSum(uint32_t u32addr, uint32_t u32count);
extern uint32_t DFMC_CheckAllOne(uint32_t u32addr, uint32_t u32count);
# 607 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/StdDriver/inc\\gpio.h" 1
# 484 "../../../../Library/StdDriver/inc\\gpio.h"
void GPIO_SetMode (GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt (GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt (GPIO_T *port, uint32_t u32Pin);
void GPIO_SetSlewCtl (GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_SetPullCtl (GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableEINT(uint32_t u32EINTn, uint32_t u32IntAttribs);
void GPIO_DisableEINT(uint32_t u32EINTn);
# 608 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/StdDriver/inc\\i2c.h" 1
# 74 "../../../../Library/StdDriver/inc\\i2c.h"
extern int32_t g_I2C_i32ErrCode;
# 297 "../../../../Library/StdDriver/inc\\i2c.h"
static inline void I2C_STOP(I2C_T *i2c);
# 308 "../../../../Library/StdDriver/inc\\i2c.h"
static inline void I2C_STOP(I2C_T *i2c)
{
    uint32_t u32TimeOutCount = SystemCoreClock;

    (i2c)->CTL0 |= ((0x1ul << (3)) | (0x1ul << (4)));

    while(i2c->CTL0 & (0x1ul << (4)))
    {
        if (--u32TimeOutCount == 0) break;
    }
}

void I2C_ClearTimeoutFlag(I2C_T *i2c);
void I2C_Close(I2C_T *i2c);
void I2C_Trigger(I2C_T *i2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Si, uint8_t u8Ack);
void I2C_DisableInt(I2C_T *i2c);
void I2C_EnableInt(I2C_T *i2c);
uint32_t I2C_GetBusClockFreq(I2C_T *i2c);
uint32_t I2C_GetIntFlag(I2C_T *i2c);
uint32_t I2C_GetStatus(I2C_T *i2c);
uint32_t I2C_Open(I2C_T *i2c, uint32_t u32BusClock);
uint8_t I2C_GetData(I2C_T *i2c);
void I2C_SetSlaveAddr(I2C_T *i2c, uint8_t u8SlaveNo, uint16_t u16SlaveAddr, uint8_t u8GCMode);
void I2C_SetSlaveAddrMask(I2C_T *i2c, uint8_t u8SlaveNo, uint16_t u16SlaveAddrMask);
uint32_t I2C_SetBusClockFreq(I2C_T *i2c, uint32_t u32BusClock);
void I2C_EnableTimeout(I2C_T *i2c, uint8_t u8LongTimeout);
void I2C_DisableTimeout(I2C_T *i2c);
void I2C_EnableWakeup(I2C_T *i2c);
void I2C_DisableWakeup(I2C_T *i2c);
void I2C_SetData(I2C_T *i2c, uint8_t u8Data);
void I2C_EnableTwoBufferMode(I2C_T *i2c, uint32_t u32BitCount);
void I2C_DisableTwoBufferMode(I2C_T *i2c);
uint32_t I2C_WriteMultiBytes(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t data[], uint32_t u32wLen);
uint32_t I2C_WriteMultiBytesOneReg(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t data[], uint32_t u32wLen);
uint32_t I2C_WriteMultiBytesTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data[], uint32_t u32wLen);
uint32_t I2C_ReadMultiBytes(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t rdata[], uint32_t u32rLen);
uint32_t I2C_ReadMultiBytesOneReg(I2C_T *i2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t rdata[], uint32_t u32rLen);
uint32_t I2C_ReadMultiBytesTwoRegs(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t rdata[], uint32_t u32rLen);
# 609 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/StdDriver/inc\\pdma.h" 1
# 349 "../../../../Library/StdDriver/inc\\pdma.h"
void PDMA_Open(PDMA_T *pdma, uint32_t u32Mask);
void PDMA_Close(PDMA_T *pdma);
void PDMA_SetTransferCnt(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount);
void PDMA_SetTransferAddr(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl);
void PDMA_SetTransferMode(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32Peripheral, uint32_t u32ScatterEn, uint32_t u32DescAddr);
void PDMA_SetBurstType(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32BurstType, uint32_t u32BurstSize);
void PDMA_EnableTimeout(PDMA_T *pdma, uint32_t u32Mask);
void PDMA_DisableTimeout(PDMA_T *pdma, uint32_t u32Mask);
void PDMA_SetTimeOut(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32OnOff, uint32_t u32TimeOutCnt);
void PDMA_Trigger(PDMA_T *pdma, uint32_t u32Ch);
void PDMA_EnableInt(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32Mask);
void PDMA_DisableInt(PDMA_T *pdma, uint32_t u32Ch, uint32_t u32Mask);
# 610 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/StdDriver/inc\\rtc.h" 1
# 124 "../../../../Library/StdDriver/inc\\rtc.h"
typedef struct
{
    uint32_t u32Year;
    uint32_t u32Month;
    uint32_t u32Day;
    uint32_t u32DayOfWeek;
    uint32_t u32Hour;
    uint32_t u32Minute;
    uint32_t u32Second;
    uint32_t u32TimeScale;
    uint32_t u32AmPm;
} S_RTC_TIME_DATA_T;
# 268 "../../../../Library/StdDriver/inc\\rtc.h"
int32_t RTC_Open(S_RTC_TIME_DATA_T *psPt);
void RTC_Close(void);
void RTC_32KCalibration(int32_t i32FrequencyX10000);
void RTC_GetDateAndTime(S_RTC_TIME_DATA_T *psPt);
void RTC_GetAlarmDateAndTime(S_RTC_TIME_DATA_T *psPt);
void RTC_SetDateAndTime(S_RTC_TIME_DATA_T *psPt);
void RTC_SetAlarmDateAndTime(S_RTC_TIME_DATA_T *psPt);
void RTC_SetDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day, uint32_t u32DayOfWeek);
void RTC_SetTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm);
void RTC_SetAlarmDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day);
void RTC_SetAlarmTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm);
void RTC_SetAlarmDateMask(uint8_t u8IsTenYMsk, uint8_t u8IsYMsk, uint8_t u8IsTenMMsk, uint8_t u8IsMMsk, uint8_t u8IsTenDMsk, uint8_t u8IsDMsk);
void RTC_SetAlarmTimeMask(uint8_t u8IsTenHMsk, uint8_t u8IsHMsk, uint8_t u8IsTenMMsk, uint8_t u8IsMMsk, uint8_t u8IsTenSMsk, uint8_t u8IsSMsk);
uint32_t RTC_GetDayOfWeek(void);
void RTC_SetTickPeriod(uint32_t u32TickSelection);
void RTC_EnableInt(uint32_t u32IntFlagMask);
void RTC_DisableInt(uint32_t u32IntFlagMask);
# 611 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/StdDriver/inc\\sys.h" 1
# 1564 "../../../../Library/StdDriver/inc\\sys.h"
static inline void SYS_UnlockReg(void);
static inline void SYS_LockReg(void);
# 1575 "../../../../Library/StdDriver/inc\\sys.h"
static inline void SYS_UnlockReg(void)
{
    uint32_t u32TimeOutCount = SystemCoreClock;

    do
    {
        ((SYS_T *) (((uint32_t)0x40000000UL) + 0x00000UL))->REGLCTL = 0x59UL;
        ((SYS_T *) (((uint32_t)0x40000000UL) + 0x00000UL))->REGLCTL = 0x16UL;
        ((SYS_T *) (((uint32_t)0x40000000UL) + 0x00000UL))->REGLCTL = 0x88UL;

        if (--u32TimeOutCount == 0) break;
    } while(((SYS_T *) (((uint32_t)0x40000000UL) + 0x00000UL))->REGLCTL == 0UL);
}
# 1596 "../../../../Library/StdDriver/inc\\sys.h"
static inline void SYS_LockReg(void)
{
    ((SYS_T *) (((uint32_t)0x40000000UL) + 0x00000UL))->REGLCTL = 0UL;
}


void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
uint32_t SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
int32_t SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
int32_t SYS_DisableBOD(void);
void SYS_SRAM_ECC_Enable(uint32_t u32Bank);
void SYS_SRAM_ECC_Disable(uint32_t u32Bank);
void SYS_SRAM_ECC_EnableInt(uint32_t u32Bank, uint32_t u32Mask);
void SYS_SRAM_ECC_DisableInt(uint32_t u32Bank, uint32_t u32Mask);
void SYS_SRAM_ECC_ClearIntFlag(uint32_t u32Bank, uint32_t u32Mask);
# 612 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/StdDriver/inc\\timer.h" 1
# 224 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_Start(TIMER_T *timer);
static inline void TIMER_Stop(TIMER_T *timer);
static inline void TIMER_EnableWakeup(TIMER_T *timer);
static inline void TIMER_DisableWakeup(TIMER_T *timer);
static inline void TIMER_StartCapture(TIMER_T *timer);
static inline void TIMER_StopCapture(TIMER_T *timer);
static inline void TIMER_EnableCaptureDebounce(TIMER_T *timer);
static inline void TIMER_DisableCaptureDebounce(TIMER_T *timer);
static inline void TIMER_EnableEventCounterDebounce(TIMER_T *timer);
static inline void TIMER_DisableEventCounterDebounce(TIMER_T *timer);
static inline void TIMER_EnableInt(TIMER_T *timer);
static inline void TIMER_DisableInt(TIMER_T *timer);
static inline void TIMER_EnableCaptureInt(TIMER_T *timer);
static inline void TIMER_DisableCaptureInt(TIMER_T *timer);
static inline uint32_t TIMER_GetIntFlag(TIMER_T *timer);
static inline void TIMER_ClearIntFlag(TIMER_T *timer);
static inline uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer);
static inline uint32_t TIMER_GetCaptureIntFlagOV(TIMER_T *timer);
static inline void TIMER_ClearCaptureIntFlag(TIMER_T *timer);
static inline uint32_t TIMER_GetWakeupFlag(TIMER_T *timer);
static inline void TIMER_ClearWakeupFlag(TIMER_T *timer);
static inline uint32_t TIMER_GetCaptureData(TIMER_T *timer);
static inline uint32_t TIMER_GetCounter(TIMER_T *timer);
static inline void TIMER_EventCounterSelect(TIMER_T *timer, uint32_t u32Src);
# 258 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_Start(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (30));
}
# 272 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_Stop(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (30));
}
# 288 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_EnableWakeup(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (23));
}
# 302 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_DisableWakeup(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (23));
}
# 316 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_StartCapture(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (3));
}
# 330 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_StopCapture(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (3));
}
# 344 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_EnableCaptureDebounce(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (6));
}
# 358 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_DisableCaptureDebounce(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (6));
}
# 372 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_EnableEventCounterDebounce(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (7));
}
# 386 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_DisableEventCounterDebounce(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (7));
}
# 400 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_EnableInt(TIMER_T *timer)
{
    timer->CTL |= (0x1ul << (29));
}
# 414 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_DisableInt(TIMER_T *timer)
{
    timer->CTL &= ~(0x1ul << (29));
}
# 428 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_EnableCaptureInt(TIMER_T *timer)
{
    timer->EXTCTL |= (0x1ul << (5));
}
# 442 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_DisableCaptureInt(TIMER_T *timer)
{
    timer->EXTCTL &= ~(0x1ul << (5));
}
# 457 "../../../../Library/StdDriver/inc\\timer.h"
static inline uint32_t TIMER_GetIntFlag(TIMER_T *timer)
{
    return ((timer->INTSTS & (0x1ul << (0))) ? 1UL : 0UL);
}
# 471 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_ClearIntFlag(TIMER_T *timer)
{
    timer->INTSTS = (0x1ul << (0));
}
# 486 "../../../../Library/StdDriver/inc\\timer.h"
static inline uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer)
{
    return timer->EINTSTS;
}
# 501 "../../../../Library/StdDriver/inc\\timer.h"
static inline uint32_t TIMER_GetCaptureIntFlagOV(TIMER_T *timer)
{
    return ((timer->EINTSTS & (0x1ul << (1))) ? 1UL : 0UL);
}
# 515 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_ClearCaptureIntFlag(TIMER_T *timer)
{
    timer->EINTSTS = (0x1ul << (0));
}
# 530 "../../../../Library/StdDriver/inc\\timer.h"
static inline uint32_t TIMER_GetWakeupFlag(TIMER_T *timer)
{
    return (timer->INTSTS & (0x1ul << (1)) ? 1UL : 0UL);
}
# 544 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_ClearWakeupFlag(TIMER_T *timer)
{
    timer->INTSTS = (0x1ul << (1));
}
# 558 "../../../../Library/StdDriver/inc\\timer.h"
static inline uint32_t TIMER_GetCaptureData(TIMER_T *timer)
{
    return timer->CAP;
}
# 572 "../../../../Library/StdDriver/inc\\timer.h"
static inline uint32_t TIMER_GetCounter(TIMER_T *timer)
{
    return timer->CNT;
}
# 592 "../../../../Library/StdDriver/inc\\timer.h"
static inline void TIMER_EventCounterSelect(TIMER_T *timer, uint32_t u32Src)
{
    timer->EXTCTL = (timer->EXTCTL & ~(0x7ul << (16))) | u32Src;
}

uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq);
void TIMER_Close(TIMER_T *timer);
int32_t TIMER_Delay(TIMER_T *timer, uint32_t u32Usec);
void TIMER_EnableCapture(TIMER_T *timer, uint32_t u32CapMode, uint32_t u32Edge);
void TIMER_CaptureSelect(TIMER_T *timer, uint32_t u32Src);
void TIMER_DisableCapture(TIMER_T *timer);
void TIMER_EnableEventCounter(TIMER_T *timer, uint32_t u32Edge);
void TIMER_DisableEventCounter(TIMER_T *timer);
uint32_t TIMER_GetModuleClock(TIMER_T *timer);
void TIMER_EnableFreqCounter(TIMER_T *timer,
                             uint32_t u32DropCount,
                             uint32_t u32Timeout,
                             uint32_t u32EnableInt);
void TIMER_DisableFreqCounter(TIMER_T *timer);
void TIMER_SetTriggerSource(TIMER_T *timer, uint32_t u32Src);
void TIMER_SetTriggerTarget(TIMER_T *timer, uint32_t u32Mask);
int32_t TIMER_ResetCounter(TIMER_T *timer);
void TIMER_EnableCaptureInputNoiseFilter(TIMER_T *timer, uint32_t u32FilterCount, uint32_t u32ClkSrcSel);
void TIMER_DisableCaptureInputNoiseFilter(TIMER_T *timer);
# 613 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/StdDriver/inc\\uart.h" 1
# 438 "../../../../Library/StdDriver/inc\\uart.h"
static inline void UART_CLEAR_RTS(UART_T* uart);
static inline void UART_SET_RTS(UART_T* uart);
# 452 "../../../../Library/StdDriver/inc\\uart.h"
static inline void UART_CLEAR_RTS(UART_T* uart)
{
    uart->MODEM |= (0x1ul << (9));
    uart->MODEM &= ~(0x1ul << (1));
}
# 469 "../../../../Library/StdDriver/inc\\uart.h"
static inline void UART_SET_RTS(UART_T* uart)
{
    uart->MODEM |= (0x1ul << (9)) | (0x1ul << (1));
}
# 507 "../../../../Library/StdDriver/inc\\uart.h"
void UART_ClearIntFlag(UART_T* uart, uint32_t u32InterruptFlag);
void UART_Close(UART_T* uart);
void UART_DisableFlowCtrl(UART_T* uart);
void UART_DisableInt(UART_T* uart, uint32_t u32InterruptFlag);
void UART_EnableFlowCtrl(UART_T* uart);
void UART_EnableInt(UART_T* uart, uint32_t u32InterruptFlag);
void UART_Open(UART_T* uart, uint32_t u32baudrate);
uint32_t UART_Read(UART_T* uart, uint8_t pu8RxBuf[], uint32_t u32ReadBytes);
void UART_SetLineConfig(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t u32stop_bits);
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC);
void UART_SelectIrDAMode(UART_T* uart, uint32_t u32Buadrate, uint32_t u32Direction);
void UART_SelectRS485Mode(UART_T* uart, uint32_t u32Mode, uint32_t u32Addr);
void UART_SelectLINMode(UART_T* uart, uint32_t u32Mode, uint32_t u32BreakLength);
uint32_t UART_Write(UART_T* uart, uint8_t pu8TxBuf[], uint32_t u32WriteBytes);
void UART_SelectSingleWireMode(UART_T *uart);
# 614 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/StdDriver/inc\\usci_i2c.h" 1
# 33 "../../../../Library/StdDriver/inc\\usci_i2c.h"
enum UI2C_MASTER_EVENT
{
    MASTER_SEND_ADDRESS = 10,
    MASTER_SEND_H_WR_ADDRESS,
    MASTER_SEND_H_RD_ADDRESS,
    MASTER_SEND_L_ADDRESS,
    MASTER_SEND_DATA,
    MASTER_SEND_REPEAT_START,
    MASTER_READ_DATA,
    MASTER_STOP,
    MASTER_SEND_START
};




enum UI2C_SLAVE_EVENT
{
    SLAVE_ADDRESS_ACK = 100,
    SLAVE_H_WR_ADDRESS_ACK,
    SLAVE_L_WR_ADDRESS_ACK,
    SLAVE_GET_DATA,
    SLAVE_SEND_DATA,
    SLAVE_H_RD_ADDRESS_ACK,
    SLAVE_L_RD_ADDRESS_ACK
};
# 101 "../../../../Library/StdDriver/inc\\usci_i2c.h"
extern int32_t g_UI2C_i32ErrCode;
# 283 "../../../../Library/StdDriver/inc\\usci_i2c.h"
uint32_t UI2C_Open(UI2C_T *ui2c, uint32_t u32BusClock);
void UI2C_Close(UI2C_T *ui2c);
void UI2C_ClearTimeoutFlag(UI2C_T *ui2c);
void UI2C_Trigger(UI2C_T *ui2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Ptrg, uint8_t u8Ack);
void UI2C_DisableInt(UI2C_T *ui2c, uint32_t u32Mask);
void UI2C_EnableInt(UI2C_T *ui2c, uint32_t u32Mask);
uint32_t UI2C_GetBusClockFreq(UI2C_T *ui2c);
uint32_t UI2C_SetBusClockFreq(UI2C_T *ui2c, uint32_t u32BusClock);
uint32_t UI2C_GetIntFlag(UI2C_T *ui2c, uint32_t u32Mask);
void UI2C_ClearIntFlag(UI2C_T *ui2c, uint32_t u32Mask);
uint32_t UI2C_GetData(UI2C_T *ui2c);
void UI2C_SetData(UI2C_T *ui2c, uint8_t u8Data);
void UI2C_SetSlaveAddr(UI2C_T *ui2c, uint8_t u8SlaveNo, uint16_t u16SlaveAddr, uint8_t u8GCMode);
void UI2C_SetSlaveAddrMask(UI2C_T *ui2c, uint8_t u8SlaveNo, uint16_t u16SlaveAddrMask);
void UI2C_EnableTimeout(UI2C_T *ui2c, uint32_t u32TimeoutCnt);
void UI2C_DisableTimeout(UI2C_T *ui2c);
void UI2C_EnableWakeup(UI2C_T *ui2c, uint8_t u8WakeupMode);
void UI2C_DisableWakeup(UI2C_T *ui2c);
uint8_t UI2C_WriteByte(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t data);
uint32_t UI2C_WriteMultiBytes(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t *data, uint32_t u32wLen);
uint8_t UI2C_WriteByteOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t data);
uint32_t UI2C_WriteMultiBytesOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t *data, uint32_t u32wLen);
uint8_t UI2C_WriteByteTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data);
uint32_t UI2C_WriteMultiBytesTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t *data, uint32_t u32wLen);
uint8_t UI2C_ReadByte(UI2C_T *ui2c, uint8_t u8SlaveAddr);
uint32_t UI2C_ReadMultiBytes(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t *rdata, uint32_t u32rLen);
uint8_t UI2C_ReadByteOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr);
uint32_t UI2C_ReadMultiBytesOneReg(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint8_t u8DataAddr, uint8_t *rdata, uint32_t u32rLen);
uint8_t UI2C_ReadByteTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr);
uint32_t UI2C_ReadMultiBytesTwoRegs(UI2C_T *ui2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t *rdata, uint32_t u32rLen);
# 615 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/StdDriver/inc\\usci_spi.h" 1
# 396 "../../../../Library/StdDriver/inc\\usci_spi.h"
uint32_t USPI_Open(USPI_T *uspi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void USPI_Close(USPI_T *uspi);
void USPI_ClearRxBuf(USPI_T *uspi);
void USPI_ClearTxBuf(USPI_T *uspi);
void USPI_DisableAutoSS(USPI_T *uspi);
void USPI_EnableAutoSS(USPI_T *uspi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t USPI_SetBusClock(USPI_T *uspi, uint32_t u32BusClock);
uint32_t USPI_GetBusClock(USPI_T *uspi);
void USPI_EnableInt(USPI_T *uspi, uint32_t u32Mask);
void USPI_DisableInt(USPI_T *uspi, uint32_t u32Mask);
uint32_t USPI_GetIntFlag(USPI_T *uspi, uint32_t u32Mask);
void USPI_ClearIntFlag(USPI_T *uspi, uint32_t u32Mask);
uint32_t USPI_GetStatus(USPI_T *uspi, uint32_t u32Mask);
void USPI_EnableWakeup(USPI_T *uspi);
void USPI_DisableWakeup(USPI_T *uspi);
# 616 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/StdDriver/inc\\usci_uart.h" 1
# 493 "../../../../Library/StdDriver/inc\\usci_uart.h"
void UUART_ClearIntFlag(UUART_T* uuart, uint32_t u32Mask);
uint32_t UUART_GetIntFlag(UUART_T* uuart, uint32_t u32Mask);
void UUART_Close(UUART_T* uuart);
void UUART_DisableInt(UUART_T* uuart, uint32_t u32Mask);
void UUART_EnableInt(UUART_T* uuart, uint32_t u32Mask);
uint32_t UUART_Open(UUART_T* uuart, uint32_t u32baudrate);
uint32_t UUART_Read(UUART_T* uuart, uint8_t pu8RxBuf[], uint32_t u32ReadBytes);
uint32_t UUART_SetLine_Config(UUART_T* uuart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t u32stop_bits);
uint32_t UUART_Write(UUART_T* uuart, uint8_t pu8TxBuf[], uint32_t u32WriteBytes);
void UUART_EnableWakeup(UUART_T* uuart, uint32_t u32WakeupMode);
void UUART_DisableWakeup(UUART_T* uuart);
void UUART_EnableFlowCtrl(UUART_T* uuart);
void UUART_DisableFlowCtrl(UUART_T* uuart);
# 617 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/StdDriver/inc\\wdt.h" 1
# 163 "../../../../Library/StdDriver/inc\\wdt.h"
static inline void WDT_Close(void);
static inline void WDT_EnableInt(void);
static inline void WDT_DisableInt(void);
# 176 "../../../../Library/StdDriver/inc\\wdt.h"
static inline void WDT_Close(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock;

    ((WDT_T *) ((((uint32_t)0x40000000UL) + 0x00040000UL) + 0x00000UL))->CTL = 0UL;
    while(((WDT_T *) ((((uint32_t)0x40000000UL) + 0x00040000UL) + 0x00000UL))->CTL & (0x1ul << (30)))
    {
        if(--u32TimeOutCnt == 0) break;
    }
}
# 196 "../../../../Library/StdDriver/inc\\wdt.h"
static inline void WDT_EnableInt(void)
{
    ((WDT_T *) ((((uint32_t)0x40000000UL) + 0x00040000UL) + 0x00000UL))->CTL |= (0x1ul << (6));
    return;
}
# 211 "../../../../Library/StdDriver/inc\\wdt.h"
static inline void WDT_DisableInt(void)
{

    ((WDT_T *) ((((uint32_t)0x40000000UL) + 0x00040000UL) + 0x00000UL))->CTL &= ~((0x1ul << (6)));
    return;
}







int32_t WDT_Open(uint32_t u32TimeoutInterval, uint32_t u32ResetDelay, uint32_t u32EnableReset, uint32_t u32EnableWakeup);
# 618 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 1 "../../../../Library/StdDriver/inc\\wwdt.h" 1
# 138 "../../../../Library/StdDriver/inc\\wwdt.h"
void WWDT_Open(uint32_t u32PreScale, uint32_t u32CmpValue, uint32_t u32EnableInt);
# 619 "../../../../Library/Device/Nuvoton/M2003J/Include\\M2003J.h" 2
# 14 "../../../../Library/Device/Nuvoton/M2003J/Include\\NuMicro.h" 2
# 11 "..\\targetdev.h" 2
# 1 "..\\isp_user.h" 1
# 14 "..\\isp_user.h"
# 1 "..\\fmc_user.h" 1
# 12 "..\\fmc_user.h"
# 1 "..\\targetdev.h" 1
# 11 "..\\targetdev.h"
# 1 "..\\isp_user.h" 1
# 12 "..\\targetdev.h" 2
# 1 "..\\i2c_transfer.h" 1
# 13 "..\\i2c_transfer.h"
extern volatile uint8_t bI2cDataReady;
extern uint8_t i2c_rcvbuf[];


void I2C_Init(void);
# 13 "..\\targetdev.h" 2
# 13 "..\\fmc_user.h" 2

extern int FMC_Proc(uint32_t u32Cmd, uint32_t addr_start, uint32_t addr_end, uint32_t *data);
# 58 "..\\fmc_user.h"
extern void UpdateConfig(uint32_t *data, uint32_t *res);
# 15 "..\\isp_user.h" 2
# 1 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 1 3
# 51 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 3
    typedef unsigned int size_t;






extern __attribute__((__nothrow__)) void *memcpy(void * __restrict ,
                    const void * __restrict , size_t ) __attribute__((__nonnull__(1,2)));






extern __attribute__((__nothrow__)) void *memmove(void * ,
                    const void * , size_t ) __attribute__((__nonnull__(1,2)));
# 77 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strcpy(char * __restrict , const char * __restrict ) __attribute__((__nonnull__(1,2)));






extern __attribute__((__nothrow__)) char *strncpy(char * __restrict , const char * __restrict , size_t ) __attribute__((__nonnull__(1,2)));
# 93 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strcat(char * __restrict , const char * __restrict ) __attribute__((__nonnull__(1,2)));






extern __attribute__((__nothrow__)) char *strncat(char * __restrict , const char * __restrict , size_t ) __attribute__((__nonnull__(1,2)));
# 117 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) int memcmp(const void * , const void * , size_t ) __attribute__((__nonnull__(1,2)));







extern __attribute__((__nothrow__)) int strcmp(const char * , const char * ) __attribute__((__nonnull__(1,2)));






extern __attribute__((__nothrow__)) int strncmp(const char * , const char * , size_t ) __attribute__((__nonnull__(1,2)));
# 141 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) int strcasecmp(const char * , const char * ) __attribute__((__nonnull__(1,2)));







extern __attribute__((__nothrow__)) int strncasecmp(const char * , const char * , size_t ) __attribute__((__nonnull__(1,2)));
# 158 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) int strcoll(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 169 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) size_t strxfrm(char * __restrict , const char * __restrict , size_t ) __attribute__((__nonnull__(2)));
# 193 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) void *memchr(const void * , int , size_t ) __attribute__((__nonnull__(1)));
# 209 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strchr(const char * , int ) __attribute__((__nonnull__(1)));
# 218 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) size_t strcspn(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 232 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strpbrk(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 247 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strrchr(const char * , int ) __attribute__((__nonnull__(1)));
# 257 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) size_t strspn(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 270 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strstr(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 280 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strtok(char * __restrict , const char * __restrict ) __attribute__((__nonnull__(2)));
extern __attribute__((__nothrow__)) char *_strtok_r(char * , const char * , char ** ) __attribute__((__nonnull__(2,3)));
# 321 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) void *memset(void * , int , size_t ) __attribute__((__nonnull__(1)));





extern __attribute__((__nothrow__)) char *strerror(int );







extern __attribute__((__nothrow__)) size_t strlen(const char * ) __attribute__((__nonnull__(1)));





extern __attribute__((__nothrow__)) size_t strnlen(const char * , size_t ) __attribute__((__nonnull__(1)));







extern __attribute__((__nothrow__)) size_t strlcpy(char * , const char * , size_t ) __attribute__((__nonnull__(1,2)));
# 369 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) size_t strlcat(char * , const char * , size_t ) __attribute__((__nonnull__(1,2)));
# 395 "C:\\Keil_v540\\ARM\\ARMCLANG\\bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) void _membitcpybl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitcpybb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitcpyhl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitcpyhb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitcpywl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitcpywb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitmovebl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitmovebb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitmovehl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitmovehb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitmovewl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitmovewb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
# 16 "..\\isp_user.h" 2
# 35 "..\\isp_user.h"
extern void GetDataFlashInfo(uint32_t *addr, uint32_t *size);
extern uint32_t GetApromSize(void);


extern int ParseCmd(unsigned char *buffer, uint8_t len);
extern uint32_t g_apromSize, g_dataFlashAddr, g_dataFlashSize;





extern uint8_t response_buff[64] __attribute__((aligned (4)));
extern uint8_t usb_rcvbuf[] __attribute__((aligned (4)));

extern volatile uint8_t bISPDataReady;
# 12 "..\\targetdev.h" 2
# 10 "../targetdev.c" 2


uint32_t GetApromSize()
{
    uint32_t size = 0x4000, data;
    int result;

    do
    {
        result = (FMC_Proc(0x00UL, size, (size) + 4, &data));

        if (result < 0)
        {
            return size;
        }
        else
        {
            size *= 2;
        }
    }
    while (1);
}

void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    uint32_t uData;
    *size = 0;
    (FMC_Proc(0x00UL, 0x0F300000UL, (0x0F300000UL) + 4, &uData));

    if ((uData & 0x01) == 0)
    {
        (FMC_Proc(0x00UL, (0x0F300000UL +4), ((0x0F300000UL +4)) + 4, &uData));

        uData &= 0x000FFFFF;

        if (uData > g_apromSize || (uData & (0x200UL - 1)))
        {
            uData = g_apromSize;
        }

        *addr = uData;
        *size = g_apromSize - uData;
    }
    else
    {
        *addr = g_apromSize;
        *size = 0;
    }
}
