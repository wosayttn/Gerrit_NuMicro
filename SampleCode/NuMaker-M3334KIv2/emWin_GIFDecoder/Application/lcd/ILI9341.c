/*********************************************************************
*                 SEGGER Software GmbH                               *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2018  SEGGER Microcontroller GmbH                *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.48 - Graphical user interface for embedded applications **
All  Intellectual Property rights in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product. This file may
only be used in accordance with the following terms:

The  software has  been licensed by SEGGER Software GmbH to Nuvoton Technology Corporation
at the address: No. 4, Creation Rd. III, Hsinchu Science Park, Taiwan
for the purposes  of  creating  libraries  for its
Arm Cortex-M and  Arm9 32-bit microcontrollers, commercialized and distributed by Nuvoton Technology Corporation
under  the terms and conditions  of  an  End  User
License  Agreement  supplied  with  the libraries.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
Licensing information
Licensor:                 SEGGER Software GmbH
Licensed to:              Nuvoton Technology Corporation, No. 4, Creation Rd. III, Hsinchu Science Park, 30077 Hsinchu City, Taiwan
Licensed SEGGER software: emWin
License number:           GUI-00735
License model:            emWin License Agreement, signed February 27, 2018
Licensed platform:        Cortex-M and ARM9 32-bit series microcontroller designed and manufactured by or for Nuvoton Technology Corporation
----------------------------------------------------------------------
Support and Update Agreement (SUA)
SUA period:               2018-03-26 - 2019-03-27
Contact to extend SUA:    sales@segger.com
----------------------------------------------------------------------
File        : LCDConf.c
Purpose     : Display controller configuration (single layer)
---------------------------END-OF-HEADER------------------------------
*/

#include <string.h>
#include "NuMicro.h"

#include "lcm.h"

// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <o> GPIO Slew Rate Control
// <0=> Normal <1=> High <2=> Fast
#define SlewRateMode    1
// *** <<< end of configuration section >>> ***

//
// Hardware related
//
#define SPI_LCD_PORT  SPI1

#define GPIO_SPI_SS PH9
#define GPIOPORT_SPI_SS PH
#define PINMASK_SPI_SS BIT9

#define GPIO_LCM_DC PH10
#define GPIOPORT_LCM_DC PH
#define PINMASK_LCM_DC BIT10

#define GPIO_LCM_LED PH11
#define GPIOPORT_LCM_LED PH
#define PINMASK_LCM_LED BIT11

#define GPIO_LCM_RESET PD14
#define GPIOPORT_LCM_RESET PD
#define PINMASK_LCM_RESET BIT14

#define SPI_CS_SET    GPIO_SPI_SS = 1
#define SPI_CS_CLR    GPIO_SPI_SS = 0

#define LCM_DC_SET    GPIO_LCM_DC = 1
#define LCM_DC_CLR    GPIO_LCM_DC = 0

#define LCM_RESET_SET GPIO_LCM_RESET = 1
#define LCM_RESET_CLR GPIO_LCM_RESET = 0

#define ILI9341_LED     GPIO_LCM_LED

#define SPI_TX_PDMA_CH  0

extern void GUI_X_Delay(int ms);

/*********************************************************************
*
*       _Read1
*/
unsigned char _Read1(void)
{
#if 1
    /* FIXME if panel supports read back feature */
    return 0;
#else
    LCM_DC_SET;
    SPI_CS_CLR;
    SPI_WRITE_TX(SPI_LCD_PORT, 0x00);
    SPI_READ_RX(SPI_LCD_PORT);
    SPI_CS_SET;
    return (SPI_READ_RX(SPI_LCD_PORT));
#endif
}

/*********************************************************************
*
*       _ReadM1
*/
void _ReadM1(unsigned char * pData, int NumItems)
{
#if 1
    /* FIXME if panel supports read back feature */
    memset(pData, 0, NumItems);
#else
    LCM_DC_SET;
    SPI_CS_CLR;
    while(NumItems--)
    {
        SPI_WRITE_TX(SPI_LCD_PORT, 0x00);
        while(SPI_IS_BUSY(SPI_LCD_PORT));
        *pData++ = SPI_READ_RX(SPI_LCD_PORT);
    }
    SPI_CS_SET;
#endif
}

/*********************************************************************
*
*       _Write0
*/
void _Write0(unsigned char Cmd)
{
    LCM_DC_CLR;

    SPI_WRITE_TX(SPI_LCD_PORT, Cmd);
    while(SPI_IS_BUSY(SPI_LCD_PORT));
}

/*********************************************************************
*
*       _Write1
*/
void _Write1(unsigned char Data)
{
    LCM_DC_SET;

    SPI_WRITE_TX(SPI_LCD_PORT, Data);
    while(SPI_IS_BUSY(SPI_LCD_PORT));
}

/*********************************************************************
*
*       _WriteM1
*/
void _WriteM1(unsigned char * pData, int NumItems)
{
    LCM_DC_SET;

    if (NumItems < 8)
    {
        while(NumItems--)
        {
            while(SPI_LCD_PORT->STATUS & SPI_STATUS_TXFULL_Msk);
            SPI_WRITE_TX(SPI_LCD_PORT, *pData++);
        }

        return;
    }
    /* Set transfer count */
    PDMA_SET_TRANS_CNT(PDMA0, SPI_TX_PDMA_CH, NumItems);
    /* Set source address */
    PDMA_SET_SRC_ADDR(PDMA0, SPI_TX_PDMA_CH, (uint32_t)pData++);
    /* Set basic mode */
    PDMA0->DSCT[SPI_TX_PDMA_CH].CTL = (PDMA0->DSCT[SPI_TX_PDMA_CH].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_BASIC;

    /* Enable SPI master's PDMA transfer function */
    SPI_TRIGGER_TX_PDMA(SPI_LCD_PORT);

    /* Check the PDMA transfer done flag */
    while((PDMA_GET_TD_STS(PDMA0) & (1 << SPI_TX_PDMA_CH)) == 0);

    /* Clear the PDMA transfer done flag */
    PDMA_CLR_TD_FLAG(PDMA0, (1 << SPI_TX_PDMA_CH));
}

static void _Open_SPI(void)
{
    /* Set control pins MFP to GPIO */
    SET_GPIO_PH10();   /* LCM_DC  = PH10 */
    SET_GPIO_PH11();   /* LCM_EN  = PH11 */
    SET_GPIO_PD14();   /* LCM_RST = PD14 */

    GPIO_SetMode(GPIOPORT_LCM_DC, PINMASK_LCM_DC, GPIO_MODE_OUTPUT);
    GPIO_SetMode(GPIOPORT_LCM_LED, PINMASK_LCM_LED, GPIO_MODE_OUTPUT);
    GPIO_SetMode(GPIOPORT_LCM_RESET, PINMASK_LCM_RESET, GPIO_MODE_OUTPUT);
//    GPIO_SetMode(GPIOPORT_SPI_SS, PINMASK_SPI_SS, GPIO_MODE_OUTPUT); //cs pin for gpiod

    /* Setup SPI multi-function pins */
//    SYS->GPD_MFP0 &= ~(SYS_GPD_MFP0_PD3MFP_Msk | SYS_GPD_MFP0_PD2MFP_Msk | SYS_GPD_MFP0_PD1MFP_Msk | SYS_GPD_MFP0_PD0MFP_Msk);
//    SYS->GPD_MFP0 |= (SYS_GPD_MFP0_PD3MFP_SPI0_SS | SYS_GPD_MFP0_PD2MFP_SPI0_CLK | SYS_GPD_MFP0_PD1MFP_SPI0_MISO | SYS_GPD_MFP0_PD0MFP_SPI0_MOSI);
//    SYS->GPA_MFP2 &= ~(SYS_GPA_MFP2_PA11MFP_Msk | SYS_GPA_MFP2_PA10MFP_Msk | SYS_GPA_MFP2_PA9MFP_Msk | SYS_GPA_MFP2_PA8MFP_Msk);
//    SYS->GPA_MFP2 |= (SYS_GPA_MFP2_PA11MFP_SPI2_SS | SYS_GPA_MFP2_PA10MFP_SPI2_CLK | SYS_GPA_MFP2_PA9MFP_SPI2_MISO | SYS_GPA_MFP2_PA8MFP_SPI2_MOSI);

    /* Set IO to high slew rate */
//    PD->SLEWCTL |= 3;
//    PD->SLEWCTL |= (3 << 8);
    /* Setup SPI1 multi-function pins */
    SET_SPI1_MOSI_PE0();
    SET_SPI1_MISO_PE1();
    SET_SPI1_CLK_PH8();
    SET_SPI1_SS_PH9();

    /* Enable SPI1 clock pin (PH8) schmitt trigger */
    PH->SMTEN |= GPIO_SMTEN_SMTEN8_Msk;

#if (SlewRateMode == 0)
    /* Enable SPI1 I/O normal slew rate */
    GPIO_SetSlewCtl(PE, BIT0 | BIT1, GPIO_SLEWCTL_NORMAL);
    GPIO_SetSlewCtl(PH, BIT8 | BIT9, GPIO_SLEWCTL_NORMAL);
#elif (SlewRateMode == 1)
    /* Enable SPI1 I/O high slew rate */
    GPIO_SetSlewCtl(PE, BIT0 | BIT1, GPIO_SLEWCTL_HIGH);
    GPIO_SetSlewCtl(PH, BIT8 | BIT9, GPIO_SLEWCTL_HIGH);
#elif (SlewRateMode == 2)
    /* Enable SPI1 I/O fast slew rate */
    GPIO_SetSlewCtl(PE, BIT0 | BIT1, GPIO_SLEWCTL_FAST);
    GPIO_SetSlewCtl(PH, BIT8 | BIT9, GPIO_SLEWCTL_FAST);
#endif

    /* Enable SPI */
//    CLK_EnableModuleClock(SPI0_MODULE);
//    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, 0);
    /* Enable SPI1 module clock */
    CLK_EnableModuleClock(SPI1_MODULE);
    /* Select SPI1 module clock source as PCLK0 */
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PCLK0, MODULE_NoMsk);

//    SPI_Open(SPI_LCD_PORT, SPI_MASTER, SPI_MODE_0, 8, 36000000);
    /* Configure SPI_LCD_PORT as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 48MHz */
    SPI_Open(SPI_LCD_PORT, SPI_MASTER, SPI_MODE_0, 8, 48000000);

    /* Clear suspend interval */
    SPI_LCD_PORT->CTL &= (~SPI_CTL_SUSPITV_Msk);
    SPI_LCD_PORT->CTL |= (0 << SPI_CTL_SUSPITV_Pos);

    /* Enable auto SS function */
    SPI_EnableAutoSS(SPI_LCD_PORT, SPI_SS, SPI_SS_ACTIVE_LOW);
    SPI_ENABLE(SPI_LCD_PORT);
    /* Disable auto SS function, control SS signal manually. */
//    SPI_DisableAutoSS(SPI_LCD_PORT);
}

static void _Open_PDMA(void)
{
    /* Reset PDMA module */
    SYS_ResetModule(PDMA0_RST);

    /* Enable PDMA channel */
    PDMA_Open(PDMA0, (1 << SPI_TX_PDMA_CH));

    /* Set transfer width (8 bits) */
    PDMA0->DSCT[SPI_TX_PDMA_CH].CTL = (PDMA0->DSCT[SPI_TX_PDMA_CH].CTL & ~PDMA_DSCT_CTL_TXWIDTH_Msk) | PDMA_WIDTH_8;
    /* Set destination address */
    PDMA_SET_DST_ADDR(PDMA0, SPI_TX_PDMA_CH, (uint32_t)&SPI_LCD_PORT->TX);
    /* Set source/destination attributes */
    PDMA0->DSCT[SPI_TX_PDMA_CH].CTL = (PDMA0->DSCT[SPI_TX_PDMA_CH].CTL & ~(PDMA_DSCT_CTL_SAINC_Msk | PDMA_DSCT_CTL_DAINC_Msk)) | (PDMA_SAR_INC | PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, SPI_TX_PDMA_CH, PDMA_SPI1_TX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA0, SPI_TX_PDMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA0->DSCT[SPI_TX_PDMA_CH].CTL |= PDMA_TBINTDIS_DISABLE;
}

/*********************************************************************
*
*       _InitController
*
* Purpose:
*   Initializes the display controller
*/
void _InitController(void)
{
    static uint8_t s_InitOnce = 0;

    if(s_InitOnce == 0)
        s_InitOnce = 1;
    else
        return;

    _Open_SPI();
#if 0
    _Open_PDMA();
#endif
    /* Configure DC/RESET/LED pins */
    GPIO_LCM_DC = 0;
    GPIO_LCM_RESET = 1;
    ILI9341_LED = 0;

    /* Hardware reset */
    GPIO_LCM_RESET = 0;
    GUI_X_Delay(20);
    GPIO_LCM_RESET = 1;
    GUI_X_Delay(120);

    //************* Start Initial Sequence **********//

    _Write0(0xCF);
    _Write1(0x00);
    _Write1(0xC1);
    _Write1(0x30);

    _Write0(0xED);
    _Write1(0x64);
    _Write1(0x03);
    _Write1(0x12);
    _Write1(0x81);

    _Write0(0xE8);
    _Write1(0x85);
    _Write1(0x00);
    _Write1(0x78);

    _Write0(0xCB);
    _Write1(0x39);
    _Write1(0x2C);
    _Write1(0x00);
    _Write1(0x34);
    _Write1(0x02);

    _Write0(0xF7);
    _Write1(0x20);

    _Write0(0xEA);
    _Write1(0x00);
    _Write1(0x00);

    _Write0(0xC0);    //Power control
    _Write1(0x23);   //VRH[5:0]

    _Write0(0xC1);    //Power control
    _Write1(0x10);   //SAP[2:0];BT[3:0]

    _Write0(0xC5);    //VCM control
    _Write1(0x3E);
    _Write1(0x28);

    _Write0(0xC7);    //VCM control2
    _Write1(0x86);

    _Write0(0x36);    // Memory Access Control
    _Write1(0xE8);

    _Write0(0x3A);
    _Write1(0x55);

    _Write0(0xB1);
    _Write1(0x00);
    _Write1(0x18);

    _Write0(0xB6);    // Display Function Control
    _Write1(0x08);
    _Write1(0x82);
    _Write1(0x27);

    _Write0(0xF2);    // 3Gamma Function Disable
    _Write1(0x00);

    _Write0(0x26);    //Gamma curve selected
    _Write1(0x01);

    _Write0(0xE0);    //Set Gamma
    _Write1(0x0F);
    _Write1(0x31);
    _Write1(0x2B);
    _Write1(0x0C);
    _Write1(0x0E);
    _Write1(0x08);
    _Write1(0x4E);
    _Write1(0xF1);
    _Write1(0x37);
    _Write1(0x07);
    _Write1(0x10);
    _Write1(0x03);
    _Write1(0x0E);
    _Write1(0x09);
    _Write1(0x00);

    _Write0(0xE1);    //Set Gamma
    _Write1(0x00);
    _Write1(0x0E);
    _Write1(0x14);
    _Write1(0x03);
    _Write1(0x11);
    _Write1(0x07);
    _Write1(0x31);
    _Write1(0xC1);
    _Write1(0x48);
    _Write1(0x08);
    _Write1(0x0F);
    _Write1(0x0C);
    _Write1(0x31);
    _Write1(0x36);
    _Write1(0x0F);

    _Write0(0x11);    //Exit Sleep
    GUI_X_Delay(120);
    _Write0(0x29);    //Display on

    ILI9341_LED = 1;
}

#define REG_CASET         0x2A   /* - Column address set */
#define REG_RASET         0x2B   /* - Row    address set */
#define REG_RAM_ACCESS_W  0x2C   /* - RAM write mode */

void NVT_SetPhysRAMAddr(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1)
{
    _Write0(REG_CASET);
    _Write1(x0>>8);
    _Write1(x0);
    _Write1(x1>>8);
    _Write1(x1);
    _Write0(REG_RASET);
    _Write1(y0>>8);
    _Write1(y0);
    _Write1(y1>>8);
    _Write1(y1);
    _Write0(REG_RAM_ACCESS_W);
}

#if 0
void NVT_FillRect(void)
{
    int x, y;

    NVT_SetPhysRAMAddr(0, 0, 320-1, 240-1);
    for (y = 0; y < 240; y++)
        for (x = 0; x < 320; x++)
        {
            // RGB565: Red
            _Write1(0xF800 >> 8);
            _Write1(0xF800 & 0xFF);

            // RGB565: Green
//            _Write1(0x07E0 >> 8);
//            _Write1(0x07E0 & 0xFF);

            // RGB565: Blue
//            _Write1(0x001F >> 8);
//            _Write1(0x001F & 0xFF);
        }
}
#endif