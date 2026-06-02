/******************************************************************************//**
 * @file     led_control.c
 * @version  V3.00
 * @brief    Control LED lighting effects sample file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdlib.h>
#include "NuMicro.h"
#include "global_variable.h"
#include "hdiv.h"
#include "led_control.h"
#include "led_gen2_control.h"

void LLSI0_IRQHandler(void);
void LLSI1_IRQHandler(void);
void LLSI2_IRQHandler(void);
void LLSI3_IRQHandler(void);
void LLSI4_IRQHandler(void);
void LLSI5_IRQHandler(void);
void LLSI6_IRQHandler(void);
void LLSI7_IRQHandler(void);
void LLSI8_IRQHandler(void);
void LLSI9_IRQHandler(void);

uint32_t TimeCounterFrameUpdate = 0;
uint8_t LED_Frame_Update_flag = 0;

void *const Mode_Function[LED_MODE_COUNT] = {FUNC_Off, FUNC_Static, FUNC_Breathing, FUNC_Strobe, FUNC_Cycling,
                                             FUNC_Random, FUNC_Music, FUNC_Wave, FUNC_Spring, FUNC_Water,
                                             FUNC_Rainbow
                                            };

/* Initial Serial LED Data Array */
__attribute__((aligned(4))) uint8_t StripALEDData[LED_STRIPA_LEN];
__attribute__((aligned(4))) uint8_t StripBLEDData[LED_STRIPB_LEN];
__attribute__((aligned(4))) uint8_t FAN1LEDData[LED_FAN1LED_LEN];
__attribute__((aligned(4))) uint8_t FAN2LEDData[LED_FAN2LED_LEN];
__attribute__((aligned(4))) uint8_t FAN3LEDData[LED_FAN3LED_LEN];
__attribute__((aligned(4))) uint8_t FAN4LEDData[LED_FAN4LED_LEN];
__attribute__((aligned(4))) uint8_t FAN5LEDData[LED_FAN5LED_LEN];
__attribute__((aligned(4))) uint8_t FAN6LEDData[LED_FAN6LED_LEN];
__attribute__((aligned(4))) uint8_t FAN7LEDData[LED_FAN7LED_LEN];
__attribute__((aligned(4))) uint8_t FAN8LEDData[LED_FAN8LED_LEN];
uint8_t *LED_Data_Array_Mapping[TOTAL_LED_AREA] =
{
    StripALEDData, StripBLEDData,
    FAN1LEDData, FAN2LEDData, FAN3LEDData, FAN4LEDData, FAN5LEDData, FAN6LEDData, FAN7LEDData, FAN8LEDData
};
const uint32_t LED_Data_Array_Size[TOTAL_LED_AREA] =
{
    LED_STRIPA_LEN, LED_STRIPB_LEN,
    LED_FAN1LED_LEN, LED_FAN2LED_LEN, LED_FAN3LED_LEN, LED_FAN4LED_LEN, LED_FAN5LED_LEN, LED_FAN6LED_LEN, LED_FAN7LED_LEN, LED_FAN8LED_LEN
};

/* Initial Strip1 Setting */
__attribute__((aligned(4))) volatile LED_Setting_T StripA_LEDSetting = {0, 0, cStripA_LED, 1, 255, 0, 0, 0xFF, 0, Dir_Forward, Type_GRB,
                                                                        1, 1, 0, FUNC_Static, StripALEDData, 0, 0, 0, Music_POP, cStripA_LED * 3U, 0U, eColorRed, 0U
                                                                       };
/* Initial Strip2 Setting */
__attribute__((aligned(4))) volatile LED_Setting_T StripB_LEDSetting = {0, 0, cStripB_LED, 1, 255, 0, 0, 0xFF, 0, Dir_Forward, Type_GRB,
                                                                        1, 1, 0, FUNC_Static, StripBLEDData, 0, 0, 0, Music_POP, cStripB_LED * 3U, 1U, eColorRed, 0U
                                                                       };
/* Initial Strip3 Setting */
__attribute__((aligned(4))) volatile LED_Setting_T FAN1_LEDSetting = {0, 0, cFAN1_LED, 1, 255, 0, 0, 0xFF, 0, Dir_Forward, Type_GRB,
                                                                      1, 1, 0, FUNC_Static, FAN1LEDData, 0, 0, 0, Music_POP, cFAN1_LED * 3U, 2U, eColorRed, 0U
                                                                     };
/* Initial Strip4 Setting */
__attribute__((aligned(4))) volatile LED_Setting_T FAN2_LEDSetting = {0, 0, cFAN2_LED, 1, 255, 0, 0, 0xFF, 0, Dir_Forward, Type_GRB,
                                                                      1, 1, 0, FUNC_Static, FAN2LEDData, 0, 0, 0, Music_POP, cFAN2_LED * 3U, 3U, eColorRed, 0U
                                                                     };
/* Initial Strip5 Setting */
__attribute__((aligned(4))) volatile LED_Setting_T FAN3_LEDSetting = {0, 0, cFAN3_LED, 1, 255, 0, 0, 0xFF, 0, Dir_Forward, Type_GRB,
                                                                      1, 1, 0, FUNC_Static, FAN3LEDData, 0, 0, 0, Music_POP, cFAN3_LED * 3U, 4U, eColorRed, 0U
                                                                     };
/* Initial Strip6 Setting */
__attribute__((aligned(4))) volatile LED_Setting_T FAN4_LEDSetting = {0, 0, cFAN4_LED, 1, 255, 0, 0, 0xFF, 0, Dir_Forward, Type_GRB,
                                                                      1, 1, 0, FUNC_Static, FAN4LEDData, 0, 0, 0, Music_POP, cFAN4_LED * 3U, 5U, eColorRed, 0U
                                                                     };
/* Initial Strip7 Setting */
__attribute__((aligned(4))) volatile LED_Setting_T FAN5_LEDSetting = {0, 0, cFAN5_LED, 1, 255, 0, 0, 0xFF, 0, Dir_Forward, Type_GRB,
                                                                      1, 1, 0, FUNC_Static, FAN5LEDData, 0, 0, 0, Music_POP, cFAN5_LED * 3U, 6U, eColorRed, 0U
                                                                     };
/* Initial Strip8 Setting */
__attribute__((aligned(4))) volatile LED_Setting_T FAN6_LEDSetting = {0, 0, cFAN6_LED, 1, 255, 0, 0, 0xFF, 0, Dir_Forward, Type_GRB,
                                                                      1, 1, 0, FUNC_Static, FAN6LEDData, 0, 0, 0, Music_POP, cFAN6_LED * 3U, 7U, eColorRed, 0U
                                                                     };
/* Initial Strip9 Setting */
__attribute__((aligned(4))) volatile LED_Setting_T FAN7_LEDSetting = {0, 0, cFAN7_LED, 1, 255, 0, 0, 0xFF, 0, Dir_Forward, Type_GRB,
                                                                      1, 1, 0, FUNC_Static, FAN7LEDData, 0, 0, 0, Music_POP, cFAN7_LED * 3U, 8U, eColorRed, 0U
                                                                     };
/* Initial Strip10 Setting */
__attribute__((aligned(4))) volatile LED_Setting_T FAN8_LEDSetting = {0, 0, cFAN8_LED, 1, 255, 0, 0, 0xFF, 0, Dir_Forward, Type_GRB,
                                                                      1, 1, 0, FUNC_Static, FAN8LEDData, 0, 0, 0, Music_POP, cFAN8_LED * 3U, 9U, eColorRed, 0U
                                                                     };

volatile LLSI_T *LLSI_Port_Mapping[TOTAL_LED_AREA] = {LLSI0, LLSI1, LLSI2, LLSI3, LLSI4, LLSI5, LLSI6, LLSI7, LLSI8, LLSI9};                         // The sequence should match to number of LLSI_Num

/* LLSI port GPIO setting table */
const LED_LLSI_IO_SETTING LED_LLSI_IO_Setting[TOTAL_LED_AREA] =
{
    {PB, 15, BIT15, GPB_IRQn,  0x51C /* SYS->GPB_MFP3 */, 0x7C, SYS_GPB_MFP3_PB15MFP_Msk, SYS_GPB_MFP3_PB15MFP_LLSI0_OUT},
    {PB, 14, BIT14, GPB_IRQn,  0x51C /* SYS->GPB_MFP3 */, 0x78, SYS_GPB_MFP3_PB14MFP_Msk, SYS_GPB_MFP3_PB14MFP_LLSI1_OUT},
    {PC, 3,  BIT3,  GPC_IRQn,  0x520 /* SYS->GPC_MFP0 */, 0x8C, SYS_GPC_MFP0_PC3MFP_Msk,  SYS_GPC_MFP0_PC3MFP_LLSI2_OUT},
    {PC, 2,  BIT2,  GPC_IRQn,  0x520 /* SYS->GPC_MFP0 */, 0x88, SYS_GPC_MFP0_PC2MFP_Msk,  SYS_GPC_MFP0_PC2MFP_LLSI3_OUT},
    {PB, 5,  BIT5,  GPB_IRQn,  0x514 /* SYS->GPB_MFP1 */, 0x54, SYS_GPB_MFP1_PB5MFP_Msk,  SYS_GPB_MFP1_PB5MFP_LLSI4_OUT},
    {PB, 4,  BIT4,  GPB_IRQn,  0x514 /* SYS->GPB_MFP1 */, 0x50, SYS_GPB_MFP1_PB4MFP_Msk,  SYS_GPB_MFP1_PB4MFP_LLSI5_OUT},
    {PB, 3,  BIT3,  GPB_IRQn,  0x510 /* SYS->GPB_MFP0 */, 0x4C, SYS_GPB_MFP0_PB3MFP_Msk,  SYS_GPB_MFP0_PB3MFP_LLSI6_OUT},
    {PB, 2,  BIT2,  GPB_IRQn,  0x510 /* SYS->GPB_MFP0 */, 0x48, SYS_GPB_MFP0_PB2MFP_Msk,  SYS_GPB_MFP0_PB2MFP_LLSI7_OUT},
    {PC, 1,  BIT1,  GPC_IRQn,  0x520 /* SYS->GPC_MFP0 */, 0x84, SYS_GPC_MFP0_PC1MFP_Msk,  SYS_GPC_MFP0_PC1MFP_LLSI8_OUT},
    {PC, 0,  BIT0,  GPC_IRQn,  0x520 /* SYS->GPC_MFP0 */, 0x80, SYS_GPC_MFP0_PC0MFP_Msk,  SYS_GPC_MFP0_PC0MFP_LLSI9_OUT}
};

volatile LED_Setting_T *LED_Mapping[TOTAL_LED_AREA] = {&StripA_LEDSetting, &StripB_LEDSetting, &FAN1_LEDSetting, &FAN2_LEDSetting, &FAN3_LEDSetting,
                                                       &FAN4_LEDSetting, &FAN5_LEDSetting, &FAN6_LEDSetting, &FAN7_LEDSetting, &FAN8_LEDSetting
                                                      };    // The sequence should match to number of LLSI_Num

volatile LED_Setting_T *PDMA_Mapping[TOTAL_LED_AREA] = {&StripA_LEDSetting, &StripB_LEDSetting, &FAN1_LEDSetting, &FAN2_LEDSetting, &FAN3_LEDSetting,
                                                        &FAN4_LEDSetting, &FAN5_LEDSetting, &FAN6_LEDSetting, &FAN7_LEDSetting, &FAN8_LEDSetting
                                                       };    // The sequence should match to number of LLSI_Num

#define BreathingArraySize 150U

/* Number of LED for each mode */
#define cMeteor_LED 4U

#define RainbowSize 8U
static const uint8_t RainbowColor[RainbowSize][3]    // (R, G, B)
= {{255,   0,   0},   // Red
    {255,  85,   0},   // Orange
    {255, 255,   0},   // Yellow
    {  0, 255,   0},   // Green
    {  0, 127, 255},   // Cyan
    {  0,   0, 255},   // Blue
    {127,   0, 255},   // Indigo
    {255,   0, 255}
};  // Purple

void LLSI_Initial(uint8_t port)
{
    uint32_t module;
    const int32_t llsi_irqn = (int32_t)LLSI0_IRQn + (int32_t)port;
    S_LLSI_CONFIG_T sConfig;
    static const uint32_t llsi_modules[TOTAL_LED_AREA] =
    {
        LLSI0_MODULE,
        LLSI1_MODULE,
        LLSI2_MODULE,
        LLSI3_MODULE,
        LLSI4_MODULE,
        LLSI5_MODULE,
        LLSI6_MODULE,
        LLSI7_MODULE,
        LLSI8_MODULE,
        LLSI9_MODULE
    };

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable module clock */
    module = llsi_modules[port];
    CLK_EnableModuleClock(module);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set LLSI multi-function pin */
    *(uint32_t *)(SYS_BASE + LED_LLSI_IO_Setting[port].IO_MFP_OFFSET) =
        (*(uint32_t *)(SYS_BASE + LED_LLSI_IO_Setting[port].IO_MFP_OFFSET) & (~(0xFUL << ((uint32_t)LED_LLSI_IO_Setting[port].IO_Number * 4U)))) | \
        LED_LLSI_IO_Setting[port].IO_LLSI_MSK;

    /* Set LLSI configuration */
    sConfig.u32LLSIMode = LLSI_MODE_PDMA;
    sConfig.u32OutputFormat = LLSI_FORMAT_GRB;
    sConfig.sTimeInfo.u32BusClock = LLSI_BUS_CLK;
    sConfig.sTimeInfo.u32TransferTimeNsec = 1200;
    sConfig.sTimeInfo.u32T0HTimeNsec = 300;
    sConfig.sTimeInfo.u32T1HTimeNsec = 900;
    sConfig.sTimeInfo.u32ResetTimeNsec = 50000;
    sConfig.u32PCNT = LED_Mapping[port]->LEDNum;
    sConfig.u32IDOS = LLSI_IDLE_LOW;
    LLSI_OpenbyConfig((LLSI_T *)LLSI_Port_Mapping[port], &sConfig);

    /* Set LLSI RGB format */
    if (PDMA_Mapping[port]->LED_Type == (uint8_t)Type_RGB)
    {
        LLSI_SET_RGB_FORMAT((LLSI_T *)(PERIPH_BASE +
                                       (0x1000UL * (PDMA_Mapping[port]->LLSI_Num % 2U)) +
                                       0x88000UL +
                                       (0x200UL * (PDMA_Mapping[port]->LLSI_Num / 2U))));
    }

    /* Enable reset command function */
    LLSI_ENABLE_RESET_COMMAND(LLSI_Port_Mapping[port]);

    /* Enable Reset command interrupt */
    LLSI_EnableInt((LLSI_T *)LLSI_Port_Mapping[port], LLSI_RSTC_INT_MASK);

    /* Enable NVIC for LLSI */
    NVIC_EnableIRQ((IRQn_Type)llsi_irqn);

    /* Change interrupt priority to normal */
    NVIC_SetPriority((IRQn_Type)llsi_irqn, INT_PRIORITY_HIGH);

    /* Lock protected registers */
    SYS_LockReg();

    /* Enable LLSI for LLSI TX*/
    LLSI_ENABLE(LLSI_Port_Mapping[port]);
}

void PDMA_Initial(uint8_t port)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Open Channel */
    PDMA_Open(PDMA0, 1UL << (uint32_t)port);

    /* PDMA Setting for LLSI */
    /* Transfer type is single transfer */
    PDMA_SetBurstType(PDMA0, port, PDMA_REQ_SINGLE, 0);

    /* Set source address, destination address */
    PDMA_SetTransferAddr(PDMA0, port, (uint32_t)LED_Mapping[port]->LED_Data, PDMA_SAR_INC, (uint32_t) & (LLSI_Port_Mapping[port]->DATA), PDMA_DAR_FIX);

    /* Lock protected registers */
    SYS_LockReg();
}

void LLSI_Underflow_Handler(uint8_t target_port)
{
    LLSI_T *target_LLSI = (LLSI_T *)LLSI_Port_Mapping[target_port];
    uint32_t pdma_mask;

    /* Check underflow flag */
    if (target_LLSI->STATUS & LLSI_STATUS_UNDFLIF_Msk)
    {
        /* Set LLSI to SW mode */
        LLSI_SET_SW_MODE(target_LLSI);

        /* Disable LLSI */
        LLSI_DISABLE(target_LLSI);

        while (target_LLSI->CTL & LLSI_CTL_LLSIEN_Msk)
        {
        }

        /* Clear underflow flag */
        target_LLSI->STATUS = LLSI_STATUS_UNDFLIF_Msk;

        /* Reset PDMA */
        if (LED_GEN2_MAX_SUPPORT_PORT > target_port)
        {
            LED_Gen2_Port_Setting[target_port].LLSI_INT_Count = 0;
        }

        pdma_mask = 1UL << (uint32_t)target_port;
        PDMA0->CHRST = pdma_mask;

        while ((PDMA0->CHCTL & pdma_mask) != 0U)
        {
        }

        //        PDMA_SetREQSEL(PDMA0, target_port);
        PDMA0->DSCT[target_port].CTL &= ~PDMA_DSCT_CTL_OPMODE_Msk;

        /* Set PDMA setting */
        if (LED_GEN2_MAX_SUPPORT_PORT > target_port)
        {
            if (LED_Gen2_Port_Setting[target_port].Use_Gen2 == TRUE)
            {
                uint32_t i;
                const uint32_t gen2_desc_count = (uint32_t)LED_GEN2_MAX_STRIP_COUNT * (uint32_t)LED_GEN2_PDMA_DESC_NUM;

                //                PDMA_SetREQSEL(PDMA0, target_port);

                /* Point to first PDMA descriptor */
                PDMA0->DSCT[target_port].NEXT = (uint32_t)&LED_Gen2_PDMA_DESC[target_port][0];// - (PDMA0->SCATBA);

                /* Even descriptor table configuration */
                for (i = 0U; i < gen2_desc_count; i += 2U)
                {
                    /* Configure next descriptor table address */
                    LED_Gen2_PDMA_DESC[target_port][i].NEXT = (uint32_t)&LED_Gen2_PDMA_DESC[target_port][i + 1U];
                }

                /* Odd descriptor table configuration */
                for (i = 1U; i < (gen2_desc_count - 1U); i += 2U)
                {
                    /* Configure next descriptor table address */
                    LED_Gen2_PDMA_DESC[target_port][i].NEXT = (uint32_t)&LED_Gen2_PDMA_DESC[target_port][i + 1U];
                }

                /* Last descriptor table configuration */
                /* Configure next descriptor table address */
                LED_Gen2_PDMA_DESC[target_port][gen2_desc_count - 1U].NEXT = 0U; /* No next operation table. No effect in basic mode */
            }
            else
            {
                /* Transfer type is single transfer */
                PDMA_SetBurstType(PDMA0, target_port, PDMA_REQ_SINGLE, 0);

                /* Set source address, destination address */
                PDMA_SetTransferAddr(PDMA0, target_port, (uint32_t)PDMA_Mapping[target_port]->LED_Data, PDMA_SAR_INC, (uint32_t) & (target_LLSI->DATA), PDMA_DAR_FIX);
            }
        }

        /* Eanble PDMA */
        PDMA0->CHCTL |= (1UL << (uint32_t)target_port);

        /* Write dummy data */
        LLSI_ENABLE(target_LLSI);
        LLSI_SET_LAST_DATA(target_LLSI);
        target_LLSI->PCNT = 1;
        target_LLSI->DATA = 0x00;
    }
}

void Polling_PDMA_Abort(void)
{
    /* Check abort flag */
    if (PDMA0->INTSTS & PDMA_INTSTS_ABTIF_Msk)
    {
        /* Check which PDMA channel aborts */
        for (uint8_t target_port_tmp = 0U; target_port_tmp < TOTAL_LED_AREA; target_port_tmp++)
        {
            uint32_t pdma_mask = 1UL << (uint32_t)target_port_tmp;

            if ((PDMA0->ABTSTS & pdma_mask) != 0U)
            {
                for (uint8_t target_port = target_port_tmp; target_port < LED_GEN2_MAX_STRIP_COUNT; target_port++)
                {
                    LLSI_T *target_LLSI = (LLSI_T *)LLSI_Port_Mapping[target_port];

                    /* Set target LLSI port */
                    /* Set LLSI to SW mode */
                    LLSI_SET_SW_MODE(target_LLSI);

                    /* Disable LLSI */
                    LLSI_DISABLE(target_LLSI);

                    while (target_LLSI->CTL & LLSI_CTL_LLSIEN_Msk)
                    {
                    }

                    /* Reset PDMA */
                    if (LED_GEN2_MAX_SUPPORT_PORT > target_port)
                    {
                        LED_Gen2_Port_Setting[target_port].LLSI_INT_Count = 0;
                    }

                    pdma_mask = 1UL << (uint32_t)target_port;
                    PDMA0->CHRST = pdma_mask;

                    while ((PDMA0->CHCTL & pdma_mask) != 0U)
                    {
                    }

                    //                    PDMA_SetREQSEL(PDMA0, target_port);
                    PDMA0->DSCT[target_port].CTL &= ~PDMA_DSCT_CTL_OPMODE_Msk;

                    /* Clear abort flag */
                    PDMA0->ABTSTS = pdma_mask;

                    /* Set PDMA setting */
                    if (LED_GEN2_MAX_SUPPORT_PORT > target_port)
                    {
                        if (LED_Gen2_Port_Setting[target_port].Use_Gen2 == TRUE)
                        {
                            uint32_t i;
                            const uint32_t gen2_desc_count = (uint32_t)LED_GEN2_MAX_STRIP_COUNT * (uint32_t)LED_GEN2_PDMA_DESC_NUM;

                            //                            PDMA_SetREQSEL(PDMA0, target_port);

                            /* Point to first PDMA descriptor */
                            PDMA0->DSCT[target_port].NEXT = (uint32_t)&LED_Gen2_PDMA_DESC[target_port][0];// - (PDMA0->SCATBA);

                            /* Even descriptor table configuration */
                            for (i = 0U; i < gen2_desc_count; i += 2U)
                            {
                                /* Configure next descriptor table address */
                                LED_Gen2_PDMA_DESC[target_port][i].NEXT = (uint32_t)&LED_Gen2_PDMA_DESC[target_port][i + 1U];
                            }

                            /* Odd descriptor table configuration */
                            for (i = 1U; i < (gen2_desc_count - 1U); i += 2U)
                            {
                                /* Configure next descriptor table address */
                                LED_Gen2_PDMA_DESC[target_port][i].NEXT = (uint32_t)&LED_Gen2_PDMA_DESC[target_port][i + 1U];
                            }

                            /* Last descriptor table configuration */
                            /* Configure next descriptor table address */
                            LED_Gen2_PDMA_DESC[target_port][gen2_desc_count - 1U].NEXT = 0U; /* No next operation table. No effect in basic mode */
                        }
                        else
                        {
                            /* Transfer type is single transfer */
                            PDMA_SetBurstType(PDMA0, target_port, PDMA_REQ_SINGLE, 0);

                            /* Set source address, destination address */
                            PDMA_SetTransferAddr(PDMA0, target_port, (uint32_t)PDMA_Mapping[target_port]->LED_Data, PDMA_SAR_INC, (uint32_t) & (target_LLSI->DATA), PDMA_DAR_FIX);
                        }
                    }

                    /* Eanble PDMA */
                    PDMA0->CHCTL |= (1UL << (uint32_t)target_port);

                    /* Write dummy data */
                    LLSI_ENABLE(target_LLSI);
                    LLSI_SET_LAST_DATA(target_LLSI);
                    target_LLSI->PCNT = 1;
                    target_LLSI->DATA = 0x00;
                }
            }
        }
    }
}

void LLSI0_IRQHandler(void)
{
    if (!PDMA_Mapping[0]->fPDMA_Done)
    {
        if (LED_Gen2_Port_Setting[0].LLSI_INT_Count > 0U)
        {
            LED_Gen2_Port_Setting[0].LLSI_INT_Count--;

            if (LED_Gen2_Port_Setting[0].LLSI_INT_Count == 0U)
            {
                PDMA_Mapping[0]->fPDMA_Done = 1;
                LLSI_SET_SW_MODE(LLSI0);
            }
        }
        else
        {
            PDMA_Mapping[0]->fPDMA_Done = 1;
            LLSI_SET_SW_MODE(LLSI0);
        }
    }

    /* Clear interrupt flag */
    LLSI_ClearIntFlag(LLSI0, LLSI_RSTC_INT_MASK | LLSI_FEND_INT_MASK);

    /* Check if LLSI underflow */
    LLSI_Underflow_Handler(0);
}

void LLSI1_IRQHandler(void)
{
    if (!PDMA_Mapping[1]->fPDMA_Done)
    {
        if (LED_Gen2_Port_Setting[1].LLSI_INT_Count > 0U)
        {
            LED_Gen2_Port_Setting[1].LLSI_INT_Count--;

            if (LED_Gen2_Port_Setting[1].LLSI_INT_Count == 0U)
            {
                PDMA_Mapping[1]->fPDMA_Done = 1;
                LLSI_SET_SW_MODE(LLSI1);
            }
        }
        else
        {
            PDMA_Mapping[1]->fPDMA_Done = 1;
            LLSI_SET_SW_MODE(LLSI1);
        }
    }

    /* Clear interrupt flag */
    LLSI_ClearIntFlag(LLSI1, LLSI_RSTC_INT_MASK | LLSI_FEND_INT_MASK);

    /* Check if LLSI underflow */
    LLSI_Underflow_Handler(1);
}

void LLSI2_IRQHandler(void)
{
    if (!PDMA_Mapping[2]->fPDMA_Done)
    {
        if (LED_Gen2_Port_Setting[2].LLSI_INT_Count > 0U)
        {
            LED_Gen2_Port_Setting[2].LLSI_INT_Count--;

            if (LED_Gen2_Port_Setting[2].LLSI_INT_Count == 0U)
            {
                PDMA_Mapping[2]->fPDMA_Done = 1;
                LLSI_SET_SW_MODE(LLSI2);
            }
        }
        else
        {
            PDMA_Mapping[2]->fPDMA_Done = 1;
            LLSI_SET_SW_MODE(LLSI2);
        }
    }

    /* Clear interrupt flag */
    LLSI_ClearIntFlag(LLSI2, LLSI_RSTC_INT_MASK | LLSI_FEND_INT_MASK);

    /* Check if LLSI underflow */
    LLSI_Underflow_Handler(2);
}

void LLSI3_IRQHandler(void)
{
    if (!PDMA_Mapping[3]->fPDMA_Done)
    {
        if (LED_Gen2_Port_Setting[3].LLSI_INT_Count > 0U)
        {
            LED_Gen2_Port_Setting[3].LLSI_INT_Count--;

            if (LED_Gen2_Port_Setting[3].LLSI_INT_Count == 0U)
            {
                PDMA_Mapping[3]->fPDMA_Done = 1;
                LLSI_SET_SW_MODE(LLSI3);
            }
        }
        else
        {
            PDMA_Mapping[3]->fPDMA_Done = 1;
            LLSI_SET_SW_MODE(LLSI3);
        }
    }

    /* Clear interrupt flag */
    LLSI_ClearIntFlag(LLSI3, LLSI_RSTC_INT_MASK | LLSI_FEND_INT_MASK);

    /* Check if LLSI underflow */
    LLSI_Underflow_Handler(3);
}

void LLSI4_IRQHandler(void)
{
    if (!PDMA_Mapping[4]->fPDMA_Done)
    {
        if (LED_Gen2_Port_Setting[4].LLSI_INT_Count > 0U)
        {
            LED_Gen2_Port_Setting[4].LLSI_INT_Count--;

            if (LED_Gen2_Port_Setting[4].LLSI_INT_Count == 0U)
            {
                PDMA_Mapping[4]->fPDMA_Done = 1;
                LLSI_SET_SW_MODE(LLSI4);
            }
        }
        else
        {
            PDMA_Mapping[4]->fPDMA_Done = 1;
            LLSI_SET_SW_MODE(LLSI4);
        }
    }

    /* Clear interrupt flag */
    LLSI_ClearIntFlag(LLSI4, LLSI_RSTC_INT_MASK | LLSI_FEND_INT_MASK);

    /* Check if LLSI underflow */
    LLSI_Underflow_Handler(4);
}

void LLSI5_IRQHandler(void)
{
    if (!PDMA_Mapping[5]->fPDMA_Done)
    {
        if (LED_Gen2_Port_Setting[5].LLSI_INT_Count > 0U)
        {
            LED_Gen2_Port_Setting[5].LLSI_INT_Count--;

            if (LED_Gen2_Port_Setting[5].LLSI_INT_Count == 0U)
            {
                PDMA_Mapping[5]->fPDMA_Done = 1;
                LLSI_SET_SW_MODE(LLSI5);
            }
        }
        else
        {
            PDMA_Mapping[5]->fPDMA_Done = 1;
            LLSI_SET_SW_MODE(LLSI5);
        }
    }

    /* Clear interrupt flag */
    LLSI_ClearIntFlag(LLSI5, LLSI_RSTC_INT_MASK | LLSI_FEND_INT_MASK);

    /* Check if LLSI underflow */
    LLSI_Underflow_Handler(5);
}

void LLSI6_IRQHandler(void)
{
    if (!PDMA_Mapping[6]->fPDMA_Done)
    {
        if (LED_Gen2_Port_Setting[6].LLSI_INT_Count > 0U)
        {
            LED_Gen2_Port_Setting[6].LLSI_INT_Count--;

            if (LED_Gen2_Port_Setting[6].LLSI_INT_Count == 0U)
            {
                PDMA_Mapping[6]->fPDMA_Done = 1;
                LLSI_SET_SW_MODE(LLSI6);
            }
        }
        else
        {
            PDMA_Mapping[6]->fPDMA_Done = 1;
            LLSI_SET_SW_MODE(LLSI6);
        }
    }

    /* Clear interrupt flag */
    LLSI_ClearIntFlag(LLSI6, LLSI_RSTC_INT_MASK | LLSI_FEND_INT_MASK);

    /* Check if LLSI underflow */
    LLSI_Underflow_Handler(6);
}

void LLSI7_IRQHandler(void)
{
    if (!PDMA_Mapping[7]->fPDMA_Done)
    {
        if (LED_Gen2_Port_Setting[7].LLSI_INT_Count > 0U)
        {
            LED_Gen2_Port_Setting[7].LLSI_INT_Count--;

            if (LED_Gen2_Port_Setting[7].LLSI_INT_Count == 0U)
            {
                PDMA_Mapping[7]->fPDMA_Done = 1;
                LLSI_SET_SW_MODE(LLSI7);
            }
        }
        else
        {
            PDMA_Mapping[7]->fPDMA_Done = 1;
            LLSI_SET_SW_MODE(LLSI7);
        }
    }

    /* Clear interrupt flag */
    LLSI_ClearIntFlag(LLSI7, LLSI_RSTC_INT_MASK | LLSI_FEND_INT_MASK);

    /* Check if LLSI underflow */
    LLSI_Underflow_Handler(7);
}

void LLSI8_IRQHandler(void)
{
    if (!PDMA_Mapping[8]->fPDMA_Done)
    {
        if (LED_Gen2_Port_Setting[8].LLSI_INT_Count > 0U)
        {
            LED_Gen2_Port_Setting[8].LLSI_INT_Count--;

            if (LED_Gen2_Port_Setting[8].LLSI_INT_Count == 0U)
            {
                PDMA_Mapping[8]->fPDMA_Done = 1;
                LLSI_SET_SW_MODE(LLSI8);
            }
        }
        else
        {
            PDMA_Mapping[8]->fPDMA_Done = 1;
            LLSI_SET_SW_MODE(LLSI8);
        }
    }

    /* Clear interrupt flag */
    LLSI_ClearIntFlag(LLSI8, LLSI_RSTC_INT_MASK | LLSI_FEND_INT_MASK);

    /* Check if LLSI underflow */
    LLSI_Underflow_Handler(8);
}

void LLSI9_IRQHandler(void)
{
    if (!PDMA_Mapping[9]->fPDMA_Done)
    {
        if (LED_Gen2_Port_Setting[9].LLSI_INT_Count > 0U)
        {
            LED_Gen2_Port_Setting[9].LLSI_INT_Count--;

            if (LED_Gen2_Port_Setting[9].LLSI_INT_Count == 0U)
            {
                PDMA_Mapping[9]->fPDMA_Done = 1;
                LLSI_SET_SW_MODE(LLSI9);
            }
        }
        else
        {
            PDMA_Mapping[9]->fPDMA_Done = 1;
            LLSI_SET_SW_MODE(LLSI9);
        }
    }

    /* Clear interrupt flag */
    LLSI_ClearIntFlag(LLSI9, LLSI_RSTC_INT_MASK | LLSI_FEND_INT_MASK);

    /* Check if LLSI underflow */
    LLSI_Underflow_Handler(9);
}

static uint8_t LED_GetColorComponent(const volatile struct LED_Setting_Tag *led_setting, uint32_t index)
{
    uint8_t color;

    if (index == 0U)
    {
        color = led_setting->Color_R;
    }
    else if (index == 1U)
    {
        color = led_setting->Color_G;
    }
    else
    {
        color = led_setting->Color_B;
    }

    return color;
}

void Set_Single(uint8_t *LED_DATA, uint32_t TotalLED, uint32_t Offset, uint8_t Data_R, uint8_t Data_G, uint8_t Data_B)
{
    uint32_t i;

    for (i = 0; i < TotalLED; i++)
    {
        LED_DATA[Offset + (i * 3U)] = Data_R;
        LED_DATA[Offset + (i * 3U) + 1U] = Data_G;
        LED_DATA[Offset + (i * 3U) + 2U] = Data_B;
    }
}

void Set_Array(uint8_t *LED_DATA, uint32_t TotalLED, uint32_t Offset, const uint8_t *DisplayData, uint8_t MaxBright)
{
    uint32_t i;

    for (i = 0; i < TotalLED; i++)
    {
        uint8_t TempR;
        uint8_t TempG;
        uint8_t TempB;

        TempR = HDIV_Div(DisplayData[i * 3U] * MaxBright, 0xFF);
        TempG = HDIV_Div(DisplayData[(i * 3U) + 1U] * MaxBright, 0xFF);
        TempB = HDIV_Div(DisplayData[(i * 3U) + 2U] * MaxBright, 0xFF);
        LED_DATA[Offset + (i * 3U)] = TempR;
        LED_DATA[Offset + (i * 3U) + 1U] = TempG;
        LED_DATA[Offset + (i * 3U) + 2U] = TempB;
    }
}

void Set_InverseArray(uint8_t *LED_DATA, uint32_t TotalLED, uint32_t Offset, const uint8_t *DisplayData, uint8_t MaxBright)
{
    uint32_t i;
    uint32_t j;

    j = TotalLED - 1U;

    for (i = 0; i < TotalLED; i++)
    {
        uint8_t TempR;
        uint8_t TempG;
        uint8_t TempB;

        TempR = HDIV_Div(DisplayData[j * 3U] * MaxBright, 0xFF);
        TempG = HDIV_Div(DisplayData[(j * 3U) + 1U] * MaxBright, 0xFF);
        TempB = HDIV_Div(DisplayData[(j * 3U) + 2U] * MaxBright, 0xFF);
        LED_DATA[Offset + (i * 3U)] = TempR;
        LED_DATA[Offset + (i * 3U) + 1U] = TempG;
        LED_DATA[Offset + (i * 3U) + 2U] = TempB;

        j--;
    }
}

void Set_LED_Data(volatile struct LED_Setting_Tag *LED_Setting)
{
    uint32_t u32Count;

    /* Calculate transfer count */
    u32Count = (LED_Setting->LEDNum * 3U) / 4U;

    if (((LED_Setting->LEDNum * 3U) % 4U) != 0U)
    {
        u32Count++;
    }

    /* Set transfer count */
    PDMA_SetTransferCnt(PDMA0, LED_Setting->LLSI_Num, PDMA_WIDTH_32, u32Count);

    /* Set request source */
    PDMA_SetTransferMode(PDMA0, LED_Setting->LLSI_Num, PDMA_LLSI0_TX + LED_Setting->LLSI_Num, FALSE, 0);    // PDMA_LLSI0_TX = 52

    /* Clear done flag */
    LED_Setting->fPDMA_Done = 0;
}

void Clear_LED_Data(volatile struct LED_Setting_Tag *LED_Setting)
{
    uint32_t i;

    for (i = 0; i < LED_Setting->Array_Size; i++)
    {
        LED_Setting->LED_Data[i] = 0x0;
    }

    /* Set data */
    Set_LED_Data(LED_Setting);
}

/*------Lighting Mode-----------------------------------------*/
void FUNC_Off(volatile struct LED_Setting_Tag *LED_Setting)
{
    /* Mapping Color to LED Format */
    Set_Single(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, 0, 0, 0);
}

void FUNC_Static(volatile struct LED_Setting_Tag *LED_Setting)
{
    uint32_t TempR;
    uint32_t TempG;
    uint32_t TempB;

    /* Calculate Color */
    TempR = HDIV_Div((LED_Setting->Color_R * LED_Setting->Brightness), 0xFF);
    TempG = HDIV_Div((LED_Setting->Color_G * LED_Setting->Brightness), 0xFF);
    TempB = HDIV_Div((LED_Setting->Color_B * LED_Setting->Brightness), 0xFF);

    /* Mapping Color to LED Format */
    Set_Single(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, TempR, TempG, TempB);
}

void FUNC_Breathing(volatile struct LED_Setting_Tag *LED_Setting)
{
    static const uint8_t BreathingBright[BreathingArraySize] = {  0,   0,   0,   0,   1,   1,   2,   3,   4,   5,
                                                                  7,   8,  10,  11,  13,  15,  17,  19,  21,  23,
                                                                  25,  28,  30,  32,  35,  37,  40,  42,  45,  47,
                                                                  50,  52,  55,  57,  60,  62,  65,  67,  70,  72,
                                                                  75,  77,  79,  81,  83,  85,  87,  89,  90,  92,
                                                                  93,  95,  96,  97,  98,  99,  99, 100, 100, 100,
                                                                  100, 100, 100, 100, 100,  99,  99,  98,  97,  96,
                                                                  95,  93,  92,  90,  89,  87,  85,  83,  81,  79,
                                                                  77,  75,  72,  70,  67,  65,  62,  60,  57,  55,
                                                                  52,  50,  47,  45,  42,  40,  37,  34,  32,  30,
                                                                  28,  25,  23,  21,  19,  17,  15,  13,  11,  10,
                                                                  8,   7,   5,   4,   3,   2,   1,   1,   0,   0,
                                                                  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                                                                  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                                                                  0,   0,   0,   0,   0,   0,   0,   0,   0,   0
                                                               };
    uint32_t Temp;
    uint32_t TempR;
    uint32_t TempG;
    uint32_t TempB;
    uint32_t breathing_period;

    /* Calculate Color */
    Temp = HDIV_Mod(HDIV_Div(LED_Setting->TimeCounter, 8 + HDIV_Div(LED_Setting->Speed, 10)), BreathingArraySize);
    TempR = HDIV_Div((LED_Setting->Color_R * LED_Setting->Brightness * BreathingBright[Temp]), 0xFF * 100);
    TempG = HDIV_Div((LED_Setting->Color_G * LED_Setting->Brightness * BreathingBright[Temp]), 0xFF * 100);
    TempB = HDIV_Div((LED_Setting->Color_B * LED_Setting->Brightness * BreathingBright[Temp]), 0xFF * 100);

    /* Mapping Color to LED Format */
    Set_Single(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, TempR, TempG, TempB);

    /* Reset CountingTime */
    breathing_period = (8U + (uint32_t)HDIV_Div(LED_Setting->Speed, 10)) * BreathingArraySize;

    if (LED_Setting->TimeCounter >= breathing_period)
    {
        LED_Setting->TimeCounter -= breathing_period;
    }
}

void FUNC_Strobe(volatile struct LED_Setting_Tag *LED_Setting)
{
    const uint32_t strobe_phase = (5U * LED_Setting->Speed) + 275U;

    /* Reset CountingTime */
    while (LED_Setting->TimeCounter >= (strobe_phase * 2U))
    {
        LED_Setting->TimeCounter -= (strobe_phase * 2U);
    }

    /* Extinguish */
    if (LED_Setting->TimeCounter < strobe_phase)
    {
        /* Mapping Color to LED Format */
        Set_Single(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, 0U, 0U, 0U);
    }
    /* Lighten */
    else
    {
        uint32_t TempR;
        uint32_t TempG;
        uint32_t TempB;

        /* Calculate Color */
        TempR = HDIV_Div((LED_Setting->Color_R * LED_Setting->Brightness), 0xFF);
        TempG = HDIV_Div((LED_Setting->Color_G * LED_Setting->Brightness), 0xFF);
        TempB = HDIV_Div((LED_Setting->Color_B * LED_Setting->Brightness), 0xFF);

        /* Mapping Color to LED Format */
        Set_Single(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, TempR, TempG, TempB);
    }
}

void FUNC_Cycling(volatile struct LED_Setting_Tag *LED_Setting)
{
    /* Reset CountingTime */
    if (LED_Setting->TimeCounter >= (500U + (LED_Setting->Speed * 10U)))
    {
        LED_Setting->TimeCounter -= (500U + (LED_Setting->Speed * 10U));

        /* Switch to next color state. */
        if (LED_Setting->ColorIndex == (uint8_t)eColorRed)
        {
            LED_Setting->ColorIndex = (uint8_t)eColorGreen;
        }
        else if (LED_Setting->ColorIndex == (uint8_t)eColorGreen)
        {
            LED_Setting->ColorIndex = (uint8_t)eColorBlue;
        }
        else if (LED_Setting->ColorIndex == (uint8_t)eColorBlue)
        {
            LED_Setting->ColorIndex = (uint8_t)eColorRed;
        }
        else
        {
        }
    }

    /* Mapping Color to LED Format */
    Set_Single(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, \
               HDIV_Div((RainbowColor[LED_Setting->ColorIndex][0] * LED_Setting->Brightness), 0xFF), \
               HDIV_Div((RainbowColor[LED_Setting->ColorIndex][1] * LED_Setting->Brightness), 0xFF), \
               HDIV_Div((RainbowColor[LED_Setting->ColorIndex][2] * LED_Setting->Brightness), 0xFF));
}

void FUNC_Random(volatile struct LED_Setting_Tag *LED_Setting)
{
    static uint8_t u8RandColorIndex = eColorRed;

    /* Reset CountingTime */
    if (LED_Setting->TimeCounter >= (500U + (LED_Setting->Speed * 10U)))
    {
        uint8_t temp;

        LED_Setting->TimeCounter -= (500U + (LED_Setting->Speed * 10U));

        /* Calculate next random color state. */
        /* If next state is same as current state, random again. */
        do
        {
            int32_t random_mod = rand() % 1024;
            uint32_t random_color_index = (RainbowSize * (uint32_t)random_mod) / 1024U;

            temp = (uint8_t)random_color_index;
        } while (temp == u8RandColorIndex);

        /* Set next color state index. */
        u8RandColorIndex = temp;
    }

    /* Mapping Color to LED Format */
    Set_Single(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, \
               HDIV_Div((RainbowColor[u8RandColorIndex][0] * LED_Setting->Brightness), 0xFF), \
               HDIV_Div((RainbowColor[u8RandColorIndex][1] * LED_Setting->Brightness), 0xFF), \
               HDIV_Div((RainbowColor[u8RandColorIndex][2] * LED_Setting->Brightness), 0xFF));
}

void FUNC_Music(volatile struct LED_Setting_Tag *LED_Setting)
{
    static uint32_t i;
    static uint32_t j;
    uint8_t POP_Color[3] = {0};
    uint32_t Unit_Volume = HDIV_Div((LED_Setting->Main_Volume * LED_Setting->LEDNum), 100);
    uint8_t JAZZ_Display[LED_GEN2_MAX_LED_NUMBER][3];

    /* Init Array */
    for (j = 0; j < LED_Setting->LEDNum; j++)
    {
        for (i = 0; i < 3U; i++)
        {
            JAZZ_Display[j][i] = 0;
        }
    }

    /* POP */
    if (LED_Setting->Music_Action == (uint8_t)Music_POP)
    {
        for (i = 0; i < 3U; i++)
        {
            POP_Color[i] = HDIV_Div(RainbowColor[(uint8_t)eColorBlue][i] * (100U - LED_Setting->Main_Volume), 100)\
                           + HDIV_Div(RainbowColor[(uint8_t)eColorRed][i] * LED_Setting->Main_Volume, 100);
        }

        /* Mapping Color to LED Format */
        Set_Single(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, \
                   HDIV_Div((POP_Color[0] * LED_Setting->Brightness), 0xFF), \
                   HDIV_Div((POP_Color[1] * LED_Setting->Brightness), 0xFF), \
                   HDIV_Div((POP_Color[2] * LED_Setting->Brightness), 0xFF));
    }
    /* JAZZ */
    else if (LED_Setting->Music_Action == (uint8_t)Music_JAZZ)
    {
        for (j = 0; j < Unit_Volume; j++)
        {
            for (i = 0; i < 3U; i++)
            {
                JAZZ_Display[j][i] = LED_GetColorComponent(LED_Setting, i);
            }
        }

        /* Mapping Color to LED Format */
        /* Direction forward or backward */
        if (LED_Setting->Direction == (uint8_t)Dir_Forward)
        {
            Set_Array(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, (uint8_t *)JAZZ_Display, LED_Setting->Brightness);
        }
        else if (LED_Setting->Direction == (uint8_t)Dir_Backward)
        {
            Set_InverseArray(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, (uint8_t *)JAZZ_Display, LED_Setting->Brightness);
        }
        else
        {
        }
    }
    /* Mixed */
    else if (LED_Setting->Music_Action == (uint8_t)Music_Mixed)
    {
        for (i = 0; i < 3U; i++)
        {
            POP_Color[i] = HDIV_Div(RainbowColor[(uint8_t)eColorBlue][i] * (100U - LED_Setting->Main_Volume), 100)\
                           + HDIV_Div(RainbowColor[(uint8_t)eColorRed][i] * LED_Setting->Main_Volume, 100);
        }

        for (j = 0; j < Unit_Volume; j++)
        {
            for (i = 0; i < 3U; i++)
            {
                JAZZ_Display[j][i] = POP_Color[i];
            }
        }

        /* Mapping Color to LED Format */
        /* Direction forward or backward */
        if (LED_Setting->Direction == (uint8_t)Dir_Forward)
        {
            Set_Array(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, (uint8_t *)JAZZ_Display, LED_Setting->Brightness);
        }
        else if (LED_Setting->Direction == (uint8_t)Dir_Backward)
        {
            Set_InverseArray(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, (uint8_t *)JAZZ_Display, LED_Setting->Brightness);
        }
        else
        {
        }
    }
    else
    {
    }
}

void FUNC_Wave(volatile struct LED_Setting_Tag *LED_Setting)
{
    static uint32_t i;
    uint32_t DeltaT = 800U + (LED_Setting->Speed * 4U);
    uint32_t CurrentTime;
    uint8_t Color1[3];
    uint8_t Color2[3];
    uint8_t DisplayColor[3];
    uint32_t Temp;

    if (LED_Setting->Direction == (uint8_t)Dir_Forward)
    {
        /* Take Color1 and Color2 from Rainbow array by TimeCounter */
        Temp = HDIV_Mod(HDIV_Div(LED_Setting->TimeCounter, DeltaT), RainbowSize);

        for (i = 0; i < 3U; i++)
        {
            /* Set Color1 */
            Color1[i] = RainbowColor[Temp][i];
            /* Set Color2 */
            Color2[i] = RainbowColor[HDIV_Mod((Temp + 1U), RainbowSize)][i];
        }
    }
    else if (LED_Setting->Direction == (uint8_t)Dir_Backward)
    {
        /* Take Color1 and Color2 from Rainbow array by TimeCounter */
        Temp = HDIV_Mod(HDIV_Div(LED_Setting->TimeCounter, DeltaT), RainbowSize);

        for (i = 0; i < 3U; i++)
        {
            /* Set Color1 */
            Color1[i] = RainbowColor[RainbowSize - 1U - Temp][i];
            /* Set Color2 */
            Color2[i] = RainbowColor[HDIV_Mod((RainbowSize + RainbowSize - 2U - Temp), RainbowSize)][i];
        }
    }
    else
    {
    }

    /* Reset CountingTime */
    if (LED_Setting->TimeCounter >= (DeltaT * RainbowSize))
    {
        LED_Setting->TimeCounter -= (DeltaT * RainbowSize);
    }

    /* Judge current time and set mixed Displaycolor by color1 and color2 */
    CurrentTime = HDIV_Mod(LED_Setting->TimeCounter, DeltaT);

    for (i = 0; i < 3U; i++)
    {
        DisplayColor[i] = HDIV_Div(Color1[i] * (DeltaT - CurrentTime), DeltaT) + HDIV_Div(Color2[i] * CurrentTime, DeltaT);
    }

    /* Mapping Color to LED Format */
    Set_Single(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, DisplayColor[0], DisplayColor[1], DisplayColor[2]);
}

void FUNC_Spring(volatile struct LED_Setting_Tag *LED_Setting)
{
    uint8_t DisplayColor[LED_GEN2_MAX_LED_NUMBER][3];
    uint8_t Color[cMeteor_LED][3];
    uint32_t Duration;
    uint32_t Loop;
    uint32_t DeltaT;
    uint32_t DelayT;
    uint32_t Spring_i;
    uint32_t Spring_j;
    uint32_t TempColor;

    /* Init Array */
    for (Spring_j = 0; Spring_j < (LED_Setting->LEDNum + (2U * cMeteor_LED)); Spring_j++)
    {
        for (Spring_i = 0; Spring_i < 3U; Spring_i++)
        {
            DisplayColor[Spring_j][Spring_i] = 0;
        }
    }

    /* DeltaT is unit time */
    DeltaT = 5U + (uint32_t)HDIV_Div((int32_t)LED_Setting->Speed, 20);

    /* Duration is one time pass through */
    Duration = DeltaT * (LED_Setting->LEDNum + (cMeteor_LED * 2U));

    /* DelayT is delay on two ends */
    DelayT = HDIV_Div(LED_Setting->LEDNum * DeltaT, 2);

    /* Reset CountingTime */
    if (LED_Setting->TimeCounter >= (2U * (Duration + DelayT)))
    {
        LED_Setting->TimeCounter -= (2U * (Duration + DelayT));
    }

    /* Forward */
    if (LED_Setting->TimeCounter < Duration)
    {
        /* Loop to indicate which LED for first Color */
        Loop = HDIV_Div(LED_Setting->TimeCounter, DeltaT);

        for (Spring_i = 0; Spring_i < 3U; Spring_i++)
        {
            TempColor = LED_GetColorComponent(LED_Setting, Spring_i);

            /* 1st 100%*/
            Color[0][Spring_i] = TempColor;
            /* 2nd 75% */
            Color[1][Spring_i] = (TempColor * 3U) >> 2U;
            /* 3rd 50% */
            Color[2][Spring_i] = TempColor >> 1;
            /* 4th 25% */
            Color[3][Spring_i] = TempColor >> 2;
        }

        for (Spring_j = Loop; Spring_j < (Loop + cMeteor_LED); Spring_j++)
        {
            for (Spring_i = 0; Spring_i < 3U; Spring_i++)
            {
                DisplayColor[Spring_j][Spring_i] = Color[Spring_j - Loop][Spring_i];
            }
        }
    }
    /* All Extinguish */
    else if (LED_Setting->TimeCounter < (Duration + DelayT))
    {
        for (Spring_j = 0; Spring_j < (LED_Setting->LEDNum + (2U * cMeteor_LED)); Spring_j++)
        {
            for (Spring_i = 0; Spring_i < 3U; Spring_i++)
            {
                DisplayColor[Spring_j][Spring_i] = 0;
            }
        }
    }
    else if (LED_Setting->TimeCounter < ((2U * Duration) + DelayT))
    {
        /* Loop to indicate which LED for first Color */
        Loop = HDIV_Div(LED_Setting->TimeCounter - (Duration + DelayT), DeltaT);

        for (Spring_i = 0; Spring_i < 3U; Spring_i++)
        {
            TempColor = LED_GetColorComponent(LED_Setting, Spring_i);

            /* 1st 25%*/
            Color[0][Spring_i] = TempColor >> 2;
            /* 2nd 50% */
            Color[1][Spring_i] = TempColor >> 1;
            /* 3rd 75% */
            Color[2][Spring_i] = (TempColor * 3U) >> 2U;
            /* 4th 100% */
            Color[3][Spring_i] = TempColor;
        }

        for (Spring_j = (LED_Setting->LEDNum + cMeteor_LED - Loop); Spring_j < (LED_Setting->LEDNum + cMeteor_LED - Loop + cMeteor_LED); Spring_j++)
        {
            for (Spring_i = 0; Spring_i < 3U; Spring_i++)
            {
                DisplayColor[Spring_j][Spring_i] = Color[Spring_j - (LED_Setting->LEDNum + cMeteor_LED - Loop)][Spring_i];
            }
        }
    }
    /* All Extinguish */
    else
    {
        for (Spring_j = 0; Spring_j < (LED_Setting->LEDNum + (2U * cMeteor_LED)); Spring_j++)
        {
            for (Spring_i = 0; Spring_i < 3U; Spring_i++)
            {
                DisplayColor[Spring_j][Spring_i] = 0;
            }
        }
    }

    /* Mapping Color to LED Format */
    /* Direction forward or backward */
    if (LED_Setting->Direction == (uint8_t)Dir_Forward)
    {
        Set_Array(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, &DisplayColor[cMeteor_LED][0], LED_Setting->Brightness);
    }
    else if (LED_Setting->Direction == (uint8_t)Dir_Backward)
    {
        Set_InverseArray(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, &DisplayColor[cMeteor_LED][0], LED_Setting->Brightness);
    }
    else
    {
    }
}

void FUNC_Water(volatile struct LED_Setting_Tag *LED_Setting)
{
    const uint8_t Brightness[4] = {100, 15, 5, 15};
    const uint8_t MeteorNum = 4;
    static uint32_t i;
    static uint32_t j;
    uint32_t Cursor;
    uint8_t DisplayColor[LED_GEN2_MAX_LED_NUMBER][3];
    uint8_t Color[3];
    uint8_t next_color_index;
    uint32_t MotionPeriod;
    uint32_t TimeInPhase;
    uint32_t PhasePeriod;

    PhasePeriod = 1000U + (LED_Setting->Speed * 4U);
    MotionPeriod = HDIV_Div(PhasePeriod, 8);
    TimeInPhase = HDIV_Mod(LED_Setting->TimeCounter, PhasePeriod);

    /* Reset CountingTime */
    if (LED_Setting->TimeCounter >= PhasePeriod)
    {
        LED_Setting->TimeCounter -= PhasePeriod;

        /* If color index is overflow, reset color index. */
        if ((uint32_t)LED_Setting->ColorIndex >= (RainbowSize - 1U))
        {
            LED_Setting->ColorIndex = (uint8_t)eColorRed;
        }
        else
        {
            LED_Setting->ColorIndex++;
        }
    }

    next_color_index = (uint8_t)HDIV_Mod((LED_Setting->ColorIndex + 1U), RainbowSize);

    /* Set Mixed Color = Color1 + Color2 */
    for (i = 0; i < 3U; i++)
    {
        Color[i] = HDIV_Div(RainbowColor[LED_Setting->ColorIndex][i] * (PhasePeriod - TimeInPhase), PhasePeriod)\
                   + HDIV_Div(RainbowColor[next_color_index][i] * TimeInPhase, PhasePeriod);
    }

    /* Calculate the cursor for moving the Brightness */
    Cursor = HDIV_Mod(HDIV_Div(LED_Setting->TimeCounter, MotionPeriod), MeteorNum);

    /* Sets the Light Bar array and modified the brightness for meteor effect. */
    for (j = 0; j < LED_Setting->LEDNum; j++)
    {
        for (i = 0; i < 3U; i++)
        {
            DisplayColor[j][i] = HDIV_Div(Color[i] * Brightness[HDIV_Mod((Cursor + j), MeteorNum)], 100);
        }
    }

    /* Mapping Color to LED Format */
    /* Direction forward or backward */
    if (LED_Setting->Direction == (uint8_t)Dir_Forward)
    {
        Set_InverseArray(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, (uint8_t *)DisplayColor, LED_Setting->Brightness);
    }
    else if (LED_Setting->Direction == (uint8_t)Dir_Backward)
    {
        Set_Array(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, (uint8_t *)DisplayColor, LED_Setting->Brightness);
    }
    else
    {
    }
}

void FUNC_Rainbow(volatile struct LED_Setting_Tag *LED_Setting)
{
    uint8_t DisplayColor[LED_GEN2_MAX_LED_NUMBER][3];
    static uint32_t i;
    static uint32_t j;
    uint32_t deltaT;
    uint32_t TotalDuration;
    uint32_t CurrentTime;
    uint32_t NextTime;
    uint8_t ColorCursor1;
    int32_t speed_divisor;

    /* Init Array */
    for (j = 0; j < LED_Setting->LEDNum; j++)
    {
        for (i = 0; i < 3U; i++)
        {
            DisplayColor[j][i] = 0;
        }
    }

    /* Duration for one time effect */
    speed_divisor = HDIV_Div(255 - (int32_t)LED_Setting->Speed, 3);
    speed_divisor += 5;
    TotalDuration = (uint32_t)HDIV_Div(60000, (int16_t)speed_divisor);

    /* Next LED time */
    NextTime = HDIV_Div(TotalDuration, 60);

    /* Current Time */
    CurrentTime = HDIV_Mod(LED_Setting->TimeCounter, TotalDuration);

    /* Unit time for each color */
    deltaT = HDIV_Div(TotalDuration, RainbowSize);

    /* Reset CountingTime */
    if (LED_Setting->TimeCounter >= TotalDuration)
    {
        LED_Setting->TimeCounter -= TotalDuration;
    }

    /* Mix Color */
    for (j = 0; j < LED_Setting->LEDNum; j++)
    {
        if (CurrentTime >= TotalDuration)
        {
            CurrentTime -= TotalDuration;
        }

        if (CurrentTime >= (deltaT * RainbowSize))
        {
            /* Color Cursor */
            ColorCursor1 = 0;

            for (i = 0; i < 3U; i++)
            {
                DisplayColor[LED_Setting->LEDNum - j - 1U][i] = RainbowColor[ColorCursor1][i];
            }
        }
        else
        {
            uint8_t ColorCursor2;

            /* Color Cursor */
            ColorCursor1 = HDIV_Mod(HDIV_Div(CurrentTime, deltaT), RainbowSize);
            ColorCursor2 = (uint8_t)HDIV_Mod((ColorCursor1 + 1U), RainbowSize);

            for (i = 0; i < 3U; i++)
            {
                DisplayColor[LED_Setting->LEDNum - j - 1U][i] = HDIV_Div(((RainbowColor[ColorCursor1][i] * ((((uint32_t)ColorCursor1 + 1U) * deltaT) - CurrentTime)) + \
                                                                          (RainbowColor[ColorCursor2][i] * (CurrentTime - ((uint32_t)ColorCursor1 * deltaT)))), deltaT);
            }
        }

        CurrentTime += NextTime;
    }

    /* Mapping Color to LED Format */
    /* Direction forward or backward */
    if (LED_Setting->Direction == (uint8_t)Dir_Forward)
    {
        Set_Array(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, (uint8_t *)DisplayColor, LED_Setting->Brightness);
    }
    else if (LED_Setting->Direction == (uint8_t)Dir_Backward)
    {
        Set_InverseArray(LED_Setting->LED_Data, LED_Setting->LEDNum, LED_Setting->LED_Offset, (uint8_t *)DisplayColor, LED_Setting->Brightness);
    }
    else
    {
    }
}
