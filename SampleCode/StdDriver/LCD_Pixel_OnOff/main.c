/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to set pixel on and off on RHE6616TP01(8-COM, 40-SEG, 1/4 Bias) LCD.
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
#define OPT_SEG45_LCD   0

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
const MFP_LCD LCD_V123_PIN[]=
{
    G_B1,   0x08,
    G_B0,   0x08,
    G_A11,  0x08
};

/* DH1 ~ 2 */
const MFP_LCD LCD_DH12_PIN[]=
{
    G_A9,   0x08,
    G_A10,  0x08,
};

/* COM0 ~ 7 */
const MFP_LCD LCD_COM_PIN[]=
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
const MFP_LCD_SEG LCD_SEG_PIN[]=
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
    G_B12,  0x08,   10, //SEG10
    G_B13,  0x08,   11, //SEG11
    G_B14,  0x08,   12, //SEG12
    G_B15,  0x08,   13, //SEG13, COM1
    G_C14,  0x08,   14, //SEG14, COM0
    G_E6,   0x08,   15, //SEG15
    G_E7,   0x08,   16, //SEG16
    G_E11,  0x08,   17, //SEG17, COM3
    G_E10,  0x08,   18, //SEG18, COM2
    G_E9,   0x08,   19, //SEG19, COM1
    G_E8,   0x08,   20, //SEG20, COM0
    G_D13,  0x08,   21, //SEG21
    G_D0,   0x08,   22, //SEG22
    G_D1,   0x08,   23, //SEG23
    G_D2,   0x08,   24, //SEG24
    G_D3,   0x08,   25, //SEG25
    G_C0,   0x08,   26, //SEG26, COM3
    G_C1,   0x08,   27, //SEG27, COM2
    G_C2,   0x08,   28, //SEG28, COM7
    G_C3,   0x08,   29, //SEG29, COM6
    G_C4,   0x08,   30, //SEG30, COM5
    G_C5,   0x08,   31, //SEG31, COM4
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
long long char_to_int(char c);
long long local_atoi(char text[]);
uint32_t sysGetNum(void);
void LCD_Init(void);
void SYS_Init(void);
void UART_Init(void);

static S_LCD_CFG_T g_LCDCfg =
{
    __LIRC,                             /*!< LCD clock source frequency */
    LCD_COM_DUTY_1_8,                   /*!< COM duty */
    LCD_BIAS_LV_1_4,                    /*!< 1/4 Bias level */
    64,                                 /*!< Operation frame rate */
    LCD_WAVEFORM_TYPE_A_NORMAL,         /*!< Waveform type */
    LCD_DISABLE_ALL_INT,                /*!< Interrupt source */
    LCD_CP_VOLTAGE_VL1_120,             /*!< VL1 voltage selected to 1.20 V */
    LCD_VOLTAGE_SOURCE_CP               /*!< Voltage source */
};

void LCD_IRQHandler(void);
void LCD_Init(void);
void SYS_Init(void);
void UART_Init(void);

long long char_to_int(char c)
{
    if (c == '0') return 0;
    else if (c == '1') return 1;
    else if (c == '2') return 2;
    else if (c == '3') return 3;
    else if (c == '4') return 4;
    else if (c == '5') return 5;
    else if (c == '6') return 6;
    else if (c == '7') return 7;
    else if (c == '8') return 8;
    else if (c == '9') return 9;

    return -1;
}

long long local_atoi(char text[])
{
    int i, len = (int)strlen(text);
    int negflag = 0;
    long long mul = len;
    long long mul2 = 1;
    long long result = 0;

    if (text[0] == '-')
    {
        negflag = 1;
        int len2 = len - 1;

        for (i = 0; i < len2; i++)
        {
            text[i] = text[i + 1];
        }

        text[i] = '\0';
        len--;
        mul = len;
    }

    for (i = 0; i < len; i++)
    {
        if (mul == 1)
        {
            mul2 = 1;
        }
        else if (mul > 1)
        {
            long long j;

            for (j = 0; j < (mul - 1); j++)
                mul2 *= 10;
        }

        result += mul2 * char_to_int(text[i]);
        mul--;
        mul2 = 1;
    }

    if (negflag == 1)
        result = 0 - result;

    return result;
}

uint32_t sysGetNum(void)
{
    uint8_t cInputTemp = 0x00, InputString[16] = {0};
    uint32_t nLoop = 0;

    while (cInputTemp != 0x0D)
    {
        cInputTemp = (uint8_t)getchar();

        if (cInputTemp == 27)
        {
            return cInputTemp;
        }

        if ((cInputTemp == 'x') || (cInputTemp == 'X') || (cInputTemp == 'f') ||
                (cInputTemp == 'F') || (cInputTemp == 'r') || (cInputTemp == 'R'))
        {
            return cInputTemp;
        }

        if (cInputTemp == '-')
        {
            InputString[nLoop] = cInputTemp;
            printf("%c", cInputTemp);
            nLoop++;
        }
        else if ((cInputTemp >= '0') && (cInputTemp <= '9'))
        {
            InputString[nLoop] = cInputTemp;
            printf("%c", cInputTemp);
            nLoop++;
        }
    }

    return (uint32_t)local_atoi((char *)InputString);
}

/**
 * @brief       IRQ Handler for LCD Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The LCD_IRQHandler is default IRQ of LCD, declared in startup_M251.s.
 */
void LCD_IRQHandler(void)
{
    if (LCD_GET_FRAME_END_FLAG() == 1)
    {
        LCD_CLEAR_FRAME_END_FLAG();
    }

    if (LCD_GET_FRAME_COUNTING_END_FLAG() == 1)
    {
        LCD_CLEAR_FRAME_COUNTING_END_FLAG();
    }
}

//---LCD Set multi function pin---//
static void Set_MFP(uint8_t u8Pin, uint8_t FunEnBit)
{
    uint8_t *GPAddr= (uint8_t *)&(SYS->GPA_MFPL);
    uint8_t  pin_offset;

    pin_offset = u8Pin & 0x01;
    if(u8Pin >= 2) u8Pin >>= 1;

    *(GPAddr + u8Pin) &= (uint8_t)( ~((0x0FU)<<(pin_offset*4)) );
    *(GPAddr + u8Pin) |= (uint8_t)( FunEnBit << (pin_offset*4));
}

void LCD_Init(void)
{
    uint32_t u32ActiveFPS;
    uint8_t pin_cnt;
    MFP_LCD_SEG *Ptr_seg = (MFP_LCD_SEG *) LCD_SEG_PIN;

    /* Configure LCD multi-function pins */
    /* COM pin */
    MFP_LCD *Ptr = (MFP_LCD *) LCD_COM_PIN;
    for(pin_cnt=0; pin_cnt<sizeof(LCD_COM_PIN)/sizeof(MFP_LCD); pin_cnt++)
        Set_MFP((Ptr+pin_cnt)->PinID, (Ptr+pin_cnt)->FunVal);

    /* SEG pin */
    for(pin_cnt=0; pin_cnt<sizeof(LCD_SEG_PIN)/sizeof(MFP_LCD_SEG); pin_cnt++)
        Set_MFP((Ptr_seg+pin_cnt)->PinID, (Ptr_seg+pin_cnt)->FunVal );

    /* V pin */
    Ptr = (MFP_LCD *) LCD_V123_PIN;
    for(pin_cnt=0; pin_cnt<sizeof(LCD_V123_PIN)/sizeof(MFP_LCD); pin_cnt++)
        Set_MFP((Ptr+pin_cnt)->PinID, (Ptr+pin_cnt)->FunVal);

    /* DH pin */
    Ptr = (MFP_LCD *) LCD_DH12_PIN;
    for(pin_cnt=0; pin_cnt<sizeof(LCD_DH12_PIN)/sizeof(MFP_LCD); pin_cnt++)
        Set_MFP((Ptr+pin_cnt)->PinID, (Ptr+pin_cnt)->FunVal);

    /* Reset LCD module */
    SYS_ResetModule(LCD_RST);

#if OPT_SEG45_LCD
    /* Output Setting Select */
    LCD_OUTPUT_SET(LCD_OUTPUT_SEL9_COM4_TO_SEG43 | LCD_OUTPUT_SEL10_COM5_TO_SEG42 | LCD_OUTPUT_SEL15_COM6_TO_SEG41 |
                   LCD_OUTPUT_SEL16_COM7_TO_SEG40 | LCD_OUTPUT_SEL37_SEG18_TO_SEG45 | LCD_OUTPUT_SEL38_SEG17_TO_SEG44);
#endif

    /* LCD Initialize and calculate real frame rate */
    u32ActiveFPS = LCD_Open(&g_LCDCfg);
    printf("Working frame rate is %uHz on Type-%c.\n\n", u32ActiveFPS, (g_LCDCfg.u32WaveformType == LCD_PSET_TYPE_Msk) ? 'B' : 'A');
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable LIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Waiting for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch the core clock to 40MHz from the MIRC */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set LCD module clock from LIRC */
    CLK_SetModuleClock(LCD_MODULE, CLK_CLKSEL1_LCDSEL_LIRC, 0);

    /* Enable LCD module clock */
    CLK_EnableModuleClock(LCD_MODULE);

    /* Set LCD charge pump clock from LCD clcok divided by 8C */
    CLK_SetModuleClock(LCDCP_MODULE, CLK_CLKSEL1_LCDCPSEL_DIV32, 0);

    /* Enable LCD charge pump clock */
    CLK_EnableModuleClock(LCDCP_MODULE);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

#if !defined(DEBUG_ENABLE_SEMIHOST) && !defined(OS_USE_SEMIHOSTING)
    /* Configure UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV_UART0(1));
#endif

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
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
#if !defined(DEBUG_ENABLE_SEMIHOST) && !defined(OS_USE_SEMIHOSTING)
    UART_Init();
#endif

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %u Hz\n", SystemCoreClock);
    printf("+------------------------------------+\n");
    printf("|    LCD Pixel On/Off Sample Code    |\n");
    printf("+------------------------------------+\n\n");
    printf("LCD configurations:\n");
    printf(" * Clock source is LIRC\n");
    printf(" * 8 COM, 40 SEG and 1/4 Bias\n");
    printf(" * Driving waveform is Type-%c\n", (g_LCDCfg.u32WaveformType == LCD_PSET_TYPE_Msk) ? 'B' : 'A');
    printf(" * Target frame rate is %uHz\n\n", g_LCDCfg.u32Framerate);
    printf("*** Notes:\n");
    printf("   Since PB12 pin is used as SEG10, if this sample code is executed in the NuMaker Board, \n");
    printf("   please do not enable the VCOM setting for PB12 in Nu-Link2-Me ICE. \n\n");

    /* Init LCD multi-function pins and settings */
    LCD_Init();

    /* Enable LCD display */
    LCD_ENABLE_DISPLAY();

    while (1)
    {
        printf("Pixel On/Off (1:On, 0:Off): \n");
        uint32_t onoff = sysGetNum();
        if (onoff > 0)
            onoff = 1;
        else
            onoff = 0;
        printf(" ... %s\n", (onoff == 1) ? "On" : "Off");

        printf("Input Com: ");
        uint32_t com = sysGetNum();
        printf("\nInput Segment: ");
        uint32_t seg = sysGetNum();
        if (onoff)
            LCD_SetPixel(com, seg, 1);
        else
            LCD_SetPixel(com, seg, 0);

        printf("\n\n");
    }
}
