/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate the LCD blinking function by using RHE6616TP01(8-COM,
 *           45-SEG, 1/4 Bias) LCD.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "NuMicro.h"
#include "lcdlib.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Constants and Struct declaration                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define OPT_SEG45_LCD       0
#define LCD_ALPHABET_NUM    7
#define T_60SEC             60
#define T_60MIN             60

typedef enum
{
    G_A0, G_A1, G_A2, G_A3, G_A4, G_A5, G_A6, G_A7, G_A8, G_A9, G_A10, G_A11, G_A12, G_A13, G_A14, G_A15,
    G_B0, G_B1, G_B2, G_B3, G_B4, G_B5, G_B6, G_B7, G_B8, G_B9, G_B10, G_B11, G_B12, G_B13, G_B14, G_B15,
    G_C0, G_C1, G_C2, G_C3, G_C4, G_C5, G_C6, G_C7, G_C8, G_C9, G_C10, G_C11, G_C12, G_C13, G_C14, G_C15,
    G_D0, G_D1, G_D2, G_D3, G_D4, G_D5, G_D6, G_D7, G_D8, G_D9, G_D10, G_D11, G_D12, G_D13, G_D14, G_D15,
    G_E0, G_E1, G_E2, G_E3, G_E4, G_E5, G_E6, G_E7, G_E8, G_E9, G_E10, G_E11, G_E12, G_E13, G_E14, G_E15,
    G_F0, G_F1, G_F2, G_F3, G_F4, G_F5, G_F6, G_F7, G_F8, G_F9, G_F10, G_F11, G_F12, G_F13, G_F14, G_F15,
    G_G0, G_G1, G_G2, G_G3, G_G4, G_G5, G_G6, G_G7, G_G8, G_G9, G_G10, G_G11, G_G12, G_G13, G_G14, G_G15,
    G_H0, G_H1, G_H2, G_H3, G_H4, G_H5, G_H6, G_H7, G_H8, G_H9, G_H10, G_H11, G_H12, G_H13, G_H14, G_H15
} E_PIN_DEF;

typedef struct
{
    uint8_t PinID;
    uint8_t FunVal;
} MFP_LCD;

typedef struct
{
    uint8_t PinID;
    uint8_t FunVal;
    uint8_t SegNum;
} MFP_LCD_SEG;

/* V1 ~ 3 */
const MFP_LCD LCD_V123_PIN[] =
{
    G_B1,   0x08,
    G_B0,   0x08,
    G_A11,  0x08
};

/* DH1 ~ 2 */
const MFP_LCD LCD_DH12_PIN[] =
{
    G_A9,   0x08,
    G_A10,  0x08,
};

/* COM0 ~ 7 */
const MFP_LCD LCD_COM_PIN[] =
{
    G_B5,   0x08,       //COM0
    G_B4,   0x08,       //COM1
    G_B3,   0x08,       //COM2
    G_B2,   0x08,       //COM3
    G_D11,  0x08,       //COM5, SEG43
    G_D10,  0x08,       //COM4, SEG42
    G_E13,  0x08,       //COM6, SEG41
    G_C8,   0x08,       //COM7, SEG40
};

/* SEG0 ~ 45 */
const MFP_LCD_SEG LCD_SEG_PIN[] =
{
    G_A8,   0x08,   0,  //SEG00
    G_C12,  0x08,   1,  //SEG01
    G_C9,   0x08,   2,  //SEG02
    G_C10,  0x08,   3,  //SEG03
    G_B6,   0x08,   4,  //SEG04
    G_B7,   0x08,   5,  //SEG05
    G_B8,   0x08,   6,  //SEG06
    G_B9,   0x08,   7,  //SEG07
    G_B10,  0x08,   8,  //SEG08
    G_B11,  0x08,   9,  //SEG09
#if OPT_NuMaker_TNLCDSub_M254K_4P8V_V1_1    
    G_H9,   0x08,   44, //SEG44, COM6, SEG18
    G_H8,   0x08,   45, //SEG45, COM7, SEG17
#else
    G_B12,  0x08,   10, //SEG10
    G_B13,  0x08,   11, //SEG11
#endif
    G_B14,  0x08,   12, //SEG12
    G_B15,  0x08,   13, //SEG13
    G_C14,  0x08,   14, //SEG14
    G_E6,   0x08,   15, //SEG15
    G_E7,   0x08,   16, //SEG16
    G_E11,  0x08,   17, //SEG17
    G_E10,  0x08,   18, //SEG18
    G_E9,   0x08,   19, //SEG19
    G_E8,   0x08,   20, //SEG20
    G_D13,  0x08,   21, //SEG21
    G_D0,   0x08,   22, //SEG22
    G_D1,   0x08,   23, //SEG23
    G_D2,   0x08,   24, //SEG24
    G_D3,   0x08,   25, //SEG25
    G_C0,   0x08,   26, //SEG26
    G_C1,   0x08,   27, //SEG27
    G_C2,   0x08,   28, //SEG28
    G_C3,   0x08,   29, //SEG29
    G_C4,   0x08,   30, //SEG30
    G_C5,   0x08,   31, //SEG31
    G_D8,   0x08,   32, //SEG32
    G_D9,   0x08,   33, //SEG33
    G_E14,  0x08,   34, //SEG34
    G_C11,  0x08,   35, //SEG35
    G_A6,   0x08,   36, //SEG36
    G_A7,   0x08,   37, //SEG37
    G_C6,   0x08,   38, //SEG38
    G_C7,   0x08,   39, //SEG39

#if OPT_SEG45_LCD
    G_C8,   0x08,   40, //SEG40, COM7
    G_E13,  0x08,   41, //SEG41, COM6
    G_D10,  0x08,   42, //SEG42, COM4
    G_D11,  0x08,   43, //SEG43, COM5
    G_H9,   0x08,   44, //SEG44, COM6, SEG18
    G_H8,   0x08,   45, //SEG45, COM7, SEG17
#endif
};

typedef enum
{
    eCHARGE_PUMP = 0x0,
    eR_MODE,
    eC_MODE,
} E_POWER_MODE;

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static S_LCD_CFG_T g_LCDCfg =
{
    __LIRC,                             /*!< LCD clock source frequency */
    LCD_COM_DUTY_1_8,                   /*!< COM duty */
    LCD_BIAS_LV_1_4,                    /*!< 1/4 Bias level */
    64,                                 /*!< Operation frame rate */
    LCD_WAVEFORM_TYPE_B_NORMAL,         /*!< Waveform type */
    LCD_DISABLE_ALL_INT,                /*!< Interrupt source */
    LCD_CP_VOLTAGE_VL1_120,             /*!< VL1 voltage selected to 1.20 V */
    LCD_VOLTAGE_SOURCE_CP               /*!< Voltage source */
};

volatile uint8_t g_u8IsRTCAlarmINT = 0;

void LCD_IRQHandler(void);
void LCD_Init(void);
void SYS_Init(void);
void UART_Init(void);
void RTC_Init(void);
void RTC_Update_AlarmTime(void);
void Min_Current_Init(void);
void PowerDownFunction(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Main function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    S_RTC_TIME_DATA_T sCurTime;
    
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* minimum current settings */
    Min_Current_Init();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    while (SYS_SetPowerRegulator(SYS_PLCTL_MVRS_LDO) == 0);

    /* Lock protected registers */
    SYS_LockReg();

    /* Init LCD multi-function pins and settings */
    LCD_Init();
    
    /* Init RTC */
    RTC_Init();
    RTC_Update_AlarmTime();

    /* Get the current time */
    RTC_GetDateAndTime(&sCurTime);

    /* Configure the LCD to display "Hour : Min : Sec" message */
    LCDLIB_SetSymbol(SYMBOL_COL_59, 1);
    LCDLIB_SetSymbol(SYMBOL_COL_63, 1);
    LCDLIB_PrintNumber(ZONE_MAIN_DIGIT, sCurTime.u32Hour * 10000 + sCurTime.u32Minute * 100 + sCurTime.u32Second);

    /* Enable LCD display */
    LCD_ENABLE_DISPLAY();

    /* Enter Power-down mode and wake up periodically at 1 Hz via RTC */
    do
    {
        if (g_u8IsRTCAlarmINT)
        {
            /* Get the current time */
            RTC_GetDateAndTime(&sCurTime);
            LCDLIB_PrintNumber(ZONE_MAIN_DIGIT, sCurTime.u32Hour * 10000 + sCurTime.u32Minute * 100 + sCurTime.u32Second);

            RTC_Update_AlarmTime();
            g_u8IsRTCAlarmINT = 0;
        }

        /* Set Power-down mode */
        PowerDownFunction();
        
    } while(1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RTC related function                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void RTC_IRQHandler(void)
{
    /* To check if RTC alarm interrupt occurred */
    if (RTC_GET_ALARM_INT_FLAG() == 1)
    {
        /* Clear RTC alarm interrupt flag */
        RTC_CLEAR_ALARM_INT_FLAG();

        g_u8IsRTCAlarmINT++;
    }
}

void RTC_Update_AlarmTime(void)
{
    S_RTC_TIME_DATA_T sAlarmTime;

    /* Get the current time */
    RTC_GetDateAndTime(&sAlarmTime);
    sAlarmTime.u32Second += 1;

    if (sAlarmTime.u32Second >= T_60SEC)
    {
        sAlarmTime.u32Minute += sAlarmTime.u32Second / T_60SEC;
        sAlarmTime.u32Second = sAlarmTime.u32Second % T_60SEC;
    }

    if (sAlarmTime.u32Minute >= T_60MIN)
    {
        sAlarmTime.u32Hour += sAlarmTime.u32Minute / T_60MIN;
        sAlarmTime.u32Minute = sAlarmTime.u32Minute % T_60MIN;
    }

    if (sAlarmTime.u32Hour >= 24)
    {
        sAlarmTime.u32Hour -= 24;
        sAlarmTime.u32Day += 1;
    }

    printf(" sAlarmTime Time:%u/%02u/%02u %02u:%02u:%02u\n", sAlarmTime.u32Year, sAlarmTime.u32Month,
           sAlarmTime.u32Day, sAlarmTime.u32Hour, sAlarmTime.u32Minute, sAlarmTime.u32Second);

    /* Set the alarm time */
    RTC_SetAlarmDateAndTime(&sAlarmTime);
}

void RTC_Init(void)
{
    S_RTC_TIME_DATA_T sInitRTC;

    /* Open RTC */
    sInitRTC.u32Year       = 2025;
    sInitRTC.u32Month      = 3;
    sInitRTC.u32Day        = 15;
    sInitRTC.u32DayOfWeek  = RTC_SATURDAY;
    sInitRTC.u32Hour       = 23;
    sInitRTC.u32Minute     = 59;
    sInitRTC.u32Second     = 50;
    sInitRTC.u32TimeScale  = RTC_CLOCK_24;
    if (RTC_Open(&sInitRTC) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");
        while (1);
    }

    /* Mask the calendar day for generating alarm time interrupt */
    RTC->CAMSK = 0x3f;

    /* Select RTC clock source from LIRC */
    RTC->LXTCTL |= RTC_LXTCTL_RTCCKSEL_Msk;
    
    /* Enable RTC alarm interrupt, and wake-up function will be also enabled */
    RTC_DisableInt(RTC_INTEN_TICKIEN_Msk);
    RTC_EnableInt(RTC_INTEN_ALMIEN_Msk);

    /* Enable RTC NVIC */
    NVIC_EnableIRQ(RTC_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LCD related function                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
static void Set_MFP(uint8_t u8Pin, uint8_t FunEnBit)
{
    uint8_t *GPAddr = (uint8_t *) & (SYS->GPA_MFPL);
    uint8_t  pin_offset;

    pin_offset = u8Pin & 0x01;

    if (u8Pin >= 2) u8Pin >>= 1;

    *(GPAddr + u8Pin) &= (uint8_t)(~((0x0FU) << (pin_offset * 4)));
    *(GPAddr + u8Pin) |= (uint8_t)(FunEnBit << (pin_offset * 4));
}

void LCD_Init(void)
{
    uint32_t u32ActiveFPS;
    uint8_t pin_cnt;
    MFP_LCD_SEG *Ptr_seg = (MFP_LCD_SEG *) LCD_SEG_PIN;

    /* Configure LCD multi-function pins */
    /* COM pin */
    MFP_LCD *Ptr = (MFP_LCD *) LCD_COM_PIN;

    for (pin_cnt = 0; pin_cnt < sizeof(LCD_COM_PIN) / sizeof(MFP_LCD); pin_cnt++)
        Set_MFP((Ptr + pin_cnt)->PinID, (Ptr + pin_cnt)->FunVal);

    /* SEG pin */
    for (pin_cnt = 0; pin_cnt < sizeof(LCD_SEG_PIN) / sizeof(MFP_LCD_SEG); pin_cnt++)
        Set_MFP((Ptr_seg + pin_cnt)->PinID, (Ptr_seg + pin_cnt)->FunVal);

    /* V pin */
    Ptr = (MFP_LCD *) LCD_V123_PIN;

    for (pin_cnt = 0; pin_cnt < sizeof(LCD_V123_PIN) / sizeof(MFP_LCD); pin_cnt++)
        Set_MFP((Ptr + pin_cnt)->PinID, (Ptr + pin_cnt)->FunVal);

    /* DH pin */
    Ptr = (MFP_LCD *) LCD_DH12_PIN;

    for (pin_cnt = 0; pin_cnt < sizeof(LCD_DH12_PIN) / sizeof(MFP_LCD); pin_cnt++)
        Set_MFP((Ptr + pin_cnt)->PinID, (Ptr + pin_cnt)->FunVal);

    /* Reset LCD module */
    SYS_ResetModule(LCD_RST);

#if OPT_SEG45_LCD
    /* Output Setting Select */
    LCD_OUTPUT_SET(LCD_OUTPUT_SEL9_COM4_TO_SEG43 | LCD_OUTPUT_SEL10_COM5_TO_SEG42 | LCD_OUTPUT_SEL15_COM6_TO_SEG41 |
                   LCD_OUTPUT_SEL16_COM7_TO_SEG40 | LCD_OUTPUT_SEL37_SEG18_TO_SEG45 | LCD_OUTPUT_SEL38_SEG17_TO_SEG44);
#endif

#if OPT_NuMaker_TNLCDSub_M254K_4P8V_V1_1
    /* PH8/9 replaces PB12/13 */
    LCD_OUTPUT_SET(LCD_OUTPUT_SEL37_SEG18_TO_SEG45 | LCD_OUTPUT_SEL38_SEG17_TO_SEG44);
#endif

    /* LCD Initialize and calculate real frame rate */
    u32ActiveFPS = LCD_Open(&g_LCDCfg);
    printf(" LCD real frame rate is %d. \n", u32ActiveFPS);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Other functions                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable LIRC, LXT and HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk | CLK_PWRCTL_LXTEN_Msk | CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for LIRC, LXT and HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk | CLK_STATUS_LXTSTB_Msk | CLK_STATUS_HIRCSTB_Msk);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set LCD module clock from LIRC */
    CLK_SetModuleClock(LCD_MODULE, CLK_CLKSEL1_LCDSEL_LIRC, 0);

    /* Enable LCD module clock */
    CLK_EnableModuleClock(LCD_MODULE);

    /* Set LCD charge pump clock from LCD clcok divided by 32 */
    CLK_SetModuleClock(LCDCP_MODULE, CLK_CLKSEL1_LCDCPSEL_DIV32, 0);

    /* Enable LCD charge pump clock */
    CLK_EnableModuleClock(LCDCP_MODULE);

    /* Enable RTC module clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
#if !defined(DEBUG_ENABLE_SEMIHOST) && !defined(OS_USE_SEMIHOSTING)
    Uart0DefaultMPF();
#endif
}

void UART_Init(void)
{
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART0, 115200);
}

void PowerDownFunction(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Select Power-down mode */
    CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_NPD1);

    /* Enter to Power-down mode */
    CLK_PowerDown();

    /* Lock protected registers */
    SYS_LockReg();
}

void ALL_GPIO_HIGH(void)
{
    GPIO_SET_OUT_DATA(PA, 0xFFFF);
    GPIO_SET_OUT_DATA(PB, 0xFFFF);
    GPIO_SET_OUT_DATA(PC, 0xFFFF);
    GPIO_SET_OUT_DATA(PD, 0xFFFF);
    GPIO_SET_OUT_DATA(PE, 0xFFFF);
    GPIO_SET_OUT_DATA(PF, 0xFFFF);
    GPIO_SET_OUT_DATA(PG, 0xFFFF);
    GPIO_SET_OUT_DATA(PH, 0xFFFF);
}

void GPIO_ReInit(void)
{
    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPB_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);
    CLK_EnableModuleClock(GPD_MODULE);
    CLK_EnableModuleClock(GPE_MODULE);
    CLK_EnableModuleClock(GPF_MODULE);
    CLK_EnableModuleClock(GPG_MODULE);
    CLK_EnableModuleClock(GPH_MODULE);

    SYS->GPA_MFPL = 0;
    SYS->GPA_MFPH = 0;

    SYS->GPB_MFPL = 0;
    SYS->GPB_MFPH = 0;

    SYS->GPC_MFPL = 0;
    SYS->GPC_MFPH = 0;

    SYS->GPD_MFPL = 0;
    SYS->GPD_MFPH = 0;

    SYS->GPE_MFPL = 0;
    SYS->GPE_MFPH = 0;

    /* Except for ICE pin (PF0/PF1) & LXT pin (PF4/PF5)*/
    SYS->GPF_MFPL = 0xAA00EE;
    SYS->GPF_MFPH = 0;

    SYS->GPG_MFPL = 0;
    SYS->GPH_MFPH = 0;

    /* Set all GPIO as output mode */
    GPIO_SetMode(PA, BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT12, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PD, BIT4 | BIT5 | BIT6 | BIT7 | BIT12 | BIT15, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PE, BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT12 | BIT15, GPIO_MODE_OUTPUT);
    /* Except for ICE pin (PF0/PF1) & LXT pin (PF4/PF5) */
    GPIO_SetMode(PF, 0xFFCC, GPIO_MODE_OUTPUT);
    GPIO_SetPullCtl(PF, BIT0 | BIT1, GPIO_PUSEL_PULL_UP);

    GPIO_SetMode(PG, BIT2 | BIT3 | BIT4, GPIO_MODE_OUTPUT);

    ALL_GPIO_HIGH();

    /* Disable LCD pin digital path */
    GPIO_DISABLE_DIGITAL_PATH(PA, BIT6 | BIT7 | BIT8 | BIT9 | BIT10);
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7 \
                                | BIT8 | BIT9 | BIT10 | BIT11 | BIT14 | BIT15);
    GPIO_DISABLE_DIGITAL_PATH(PC, (0xFFFFUL));
    GPIO_DISABLE_DIGITAL_PATH(PD, BIT0 | BIT1 | BIT2 | BIT3 | BIT8 | BIT9 | BIT10 | BIT11 | BIT13);
    GPIO_DISABLE_DIGITAL_PATH(PE, BIT6 | BIT7 | BIT8 | BIT9 | BIT10 | BIT11 | BIT13 | BIT14);
    GPIO_DISABLE_DIGITAL_PATH(PH, BIT8 | BIT9);

    /* Disable GPIO clock */
    CLK_DisableModuleClock(GPA_MODULE);
    CLK_DisableModuleClock(GPB_MODULE);
    CLK_DisableModuleClock(GPC_MODULE);
    CLK_DisableModuleClock(GPD_MODULE);
    CLK_DisableModuleClock(GPE_MODULE);
    CLK_DisableModuleClock(GPF_MODULE);
    CLK_DisableModuleClock(GPG_MODULE);
    CLK_DisableModuleClock(GPH_MODULE);
}

void Min_Current_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Disable POR and BOD */
    SYS_DISABLE_POR();
    SYS_DISABLE_BOD();

    /* Enable GPIO clock and pull high all GPIO pins */
    GPIO_ReInit();
}
