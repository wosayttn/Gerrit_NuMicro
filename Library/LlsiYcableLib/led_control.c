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

LED_FUNC Mode_Function[LED_MODE_COUNT] = {FUNC_Off, FUNC_Static, FUNC_Breathing, FUNC_Strobe, FUNC_Cycling,
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
    &StripALEDData[0], &StripBLEDData[0],
    &FAN1LEDData[0], &FAN2LEDData[0], &FAN3LEDData[0], &FAN4LEDData[0], &FAN5LEDData[0], &FAN6LEDData[0], &FAN7LEDData[0], &FAN8LEDData[0]
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

LLSI_T *LLSI_Port_Mapping[TOTAL_LED_AREA] = {LLSI0, LLSI1, LLSI2, LLSI3, LLSI4, LLSI5, LLSI6, LLSI7, LLSI8, LLSI9};                         // The sequence should match to number of LLSI_Num

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
= {{255U,   0U,   0U},   // Red
    {255U,  85U,   0U},   // Orange
    {255U, 255U,   0U},   // Yellow
    {  0U, 255U,   0U},   // Green
    {  0U, 127U, 255U},   // Cyan
    {  0U,   0U, 255U},   // Blue
    {127U,   0U, 255U},   // Indigo
    {255U,   0U, 255U}
};  // Purple

void LLSI_Initial(uint8_t port)
{
    uint32_t module;
    const volatile LED_Setting_T *pdma_setting = PDMA_Mapping[port];
    const uint8_t pdma_led_type = pdma_setting->LED_Type;
    const uint8_t pdma_llsi_num = pdma_setting->LLSI_Num;
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
    LLSI_OpenbyConfig(LLSI_Port_Mapping[port], &sConfig);

    /* Set LLSI RGB format */
    if (pdma_led_type == (uint8_t)Type_RGB)
    {
        LLSI_SET_RGB_FORMAT((LLSI_T *)(PERIPH_BASE +
                                       (0x1000UL * ((uint32_t)pdma_llsi_num % 2U)) +
                                       0x88000UL +
                                       (0x200UL * ((uint32_t)pdma_llsi_num / 2U))));
    }

    /* Enable reset command function */
    LLSI_ENABLE_RESET_COMMAND(LLSI_Port_Mapping[port]);

    /* Enable Reset command interrupt */
    LLSI_EnableInt(LLSI_Port_Mapping[port], LLSI_RSTC_INT_MASK);

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
    LLSI_T *target_LLSI = LLSI_Port_Mapping[target_port];
    uint32_t pdma_mask;

    /* Check underflow flag */
    if ((target_LLSI->STATUS & LLSI_STATUS_UNDFLIF_Msk) != 0UL)
    {
        /* Set LLSI to SW mode */
        LLSI_SET_SW_MODE(target_LLSI);

        /* Disable LLSI */
        LLSI_DISABLE(target_LLSI);

        while ((target_LLSI->CTL & LLSI_CTL_LLSIEN_Msk) != 0UL)
        {
        }

        /* Clear underflow flag */
        target_LLSI->STATUS = LLSI_STATUS_UNDFLIF_Msk;

        /* Reset PDMA */
        LED_Gen2_Port_Setting[target_port].LLSI_INT_Count = 0;

        pdma_mask = 1UL << (uint32_t)target_port;
        PDMA0->CHRST = pdma_mask;

        while ((PDMA0->CHCTL & pdma_mask) != 0U)
        {
        }

        //        PDMA_SetREQSEL(PDMA0, target_port);
        PDMA0->DSCT[target_port].CTL &= ~PDMA_DSCT_CTL_OPMODE_Msk;

        /* Set PDMA setting */
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
    if ((PDMA0->INTSTS & PDMA_INTSTS_ABTIF_Msk) != 0UL)
    {
        /* Check which PDMA channel aborts */
        for (uint8_t target_port_tmp = 0U; target_port_tmp < TOTAL_LED_AREA; target_port_tmp++)
        {
            uint32_t pdma_mask = 1UL << (uint32_t)target_port_tmp;

            if ((PDMA0->ABTSTS & pdma_mask) != 0U)
            {
                for (uint8_t target_port = target_port_tmp; target_port < LED_GEN2_MAX_STRIP_COUNT; target_port++)
                {
                    LLSI_T *target_LLSI = LLSI_Port_Mapping[target_port];

                    /* Set target LLSI port */
                    /* Set LLSI to SW mode */
                    LLSI_SET_SW_MODE(target_LLSI);

                    /* Disable LLSI */
                    LLSI_DISABLE(target_LLSI);

                    while ((target_LLSI->CTL & LLSI_CTL_LLSIEN_Msk) != 0UL)
                    {
                    }

                    /* Reset PDMA */
                    LED_Gen2_Port_Setting[target_port].LLSI_INT_Count = 0;

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
    if (PDMA_Mapping[0]->fPDMA_Done == 0U)
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
    if (PDMA_Mapping[1]->fPDMA_Done == 0U)
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
    if (PDMA_Mapping[2]->fPDMA_Done == 0U)
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
    if (PDMA_Mapping[3]->fPDMA_Done == 0U)
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
    if (PDMA_Mapping[4]->fPDMA_Done == 0U)
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
    if (PDMA_Mapping[5]->fPDMA_Done == 0U)
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
    if (PDMA_Mapping[6]->fPDMA_Done == 0U)
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
    if (PDMA_Mapping[7]->fPDMA_Done == 0U)
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
    if (PDMA_Mapping[8]->fPDMA_Done == 0U)
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
    if (PDMA_Mapping[9]->fPDMA_Done == 0U)
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

static uint8_t LED_ScaleColor(uint8_t color, uint8_t brightness)
{
    const uint32_t product = (uint32_t)color * (uint32_t)brightness;

    return (uint8_t)HDIV_Div((int32_t)product, (int16_t)255);
}

static uint32_t LED_Divide(uint32_t numerator, uint32_t denominator)
{
    return (uint32_t)HDIV_Div((int32_t)numerator, (int16_t)denominator);
}

static uint32_t LED_Modulo(uint32_t numerator, uint32_t denominator)
{
    return (uint32_t)HDIV_Mod((int32_t)numerator, (int16_t)denominator);
}

static uint8_t LED_MixColor(uint8_t color_a, uint8_t color_b, uint8_t mix)
{
    const uint32_t color_a_part = LED_Divide((uint32_t)color_a * (100U - (uint32_t)mix), 100U);
    const uint32_t color_b_part = LED_Divide((uint32_t)color_b * (uint32_t)mix, 100U);

    return (uint8_t)(color_a_part + color_b_part);
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
        const uint32_t source_index = i * 3U;
        const uint32_t target_index = Offset + source_index;

        LED_DATA[target_index] = LED_ScaleColor(DisplayData[source_index], MaxBright);
        LED_DATA[target_index + 1U] = LED_ScaleColor(DisplayData[source_index + 1U], MaxBright);
        LED_DATA[target_index + 2U] = LED_ScaleColor(DisplayData[source_index + 2U], MaxBright);
    }
}

void Set_InverseArray(uint8_t *LED_DATA, uint32_t TotalLED, uint32_t Offset, const uint8_t *DisplayData, uint8_t MaxBright)
{
    uint32_t i;
    uint32_t j;

    j = TotalLED - 1U;

    for (i = 0; i < TotalLED; i++)
    {
        const uint32_t source_index = j * 3U;
        const uint32_t target_index = Offset + (i * 3U);

        LED_DATA[target_index] = LED_ScaleColor(DisplayData[source_index], MaxBright);
        LED_DATA[target_index + 1U] = LED_ScaleColor(DisplayData[source_index + 1U], MaxBright);
        LED_DATA[target_index + 2U] = LED_ScaleColor(DisplayData[source_index + 2U], MaxBright);

        j--;
    }
}

static void LED_SetSingleFromSetting(volatile struct LED_Setting_Tag *LED_Setting,
                                     uint8_t data_r, uint8_t data_g, uint8_t data_b)
{
    uint8_t *led_data = LED_Setting->LED_Data;
    const uint32_t led_num = (uint32_t)LED_Setting->LEDNum;
    const uint32_t led_offset = (uint32_t)LED_Setting->LED_Offset;

    Set_Single(led_data, led_num, led_offset, data_r, data_g, data_b);
}

static void LED_SetArrayFromSetting(volatile struct LED_Setting_Tag *LED_Setting,
                                    const uint8_t *display_data, uint8_t brightness, uint8_t direction)
{
    uint8_t *led_data = LED_Setting->LED_Data;
    const uint32_t led_num = (uint32_t)LED_Setting->LEDNum;
    const uint32_t led_offset = (uint32_t)LED_Setting->LED_Offset;

    if (direction == (uint8_t)Dir_Forward)
    {
        Set_Array(led_data, led_num, led_offset, display_data, brightness);
    }
    else if (direction == (uint8_t)Dir_Backward)
    {
        Set_InverseArray(led_data, led_num, led_offset, display_data, brightness);
    }
    else
    {
    }
}

void Set_LED_Data(volatile struct LED_Setting_Tag *LED_Setting)
{
    uint32_t u32Count;
    const uint32_t led_num = (uint32_t)LED_Setting->LEDNum;
    const uint8_t llsi_num = LED_Setting->LLSI_Num;

    /* Calculate transfer count */
    u32Count = (led_num * 3U) / 4U;

    if (((led_num * 3U) % 4U) != 0U)
    {
        u32Count++;
    }

    /* Set transfer count */
    PDMA_SetTransferCnt(PDMA0, llsi_num, PDMA_WIDTH_32, u32Count);

    /* Set request source */
    PDMA_SetTransferMode(PDMA0, llsi_num, PDMA_LLSI0_TX + (uint32_t)llsi_num, FALSE, 0);    // PDMA_LLSI0_TX = 52

    /* Clear done flag */
    LED_Setting->fPDMA_Done = 0;
}

void Clear_LED_Data(volatile struct LED_Setting_Tag *LED_Setting)
{
    const uint32_t array_size = LED_Setting->Array_Size;
    uint8_t *led_data = LED_Setting->LED_Data;

    for (uint32_t clear_led_index = 0U; clear_led_index < array_size; clear_led_index++)
    {
        led_data[clear_led_index] = 0x0U;
    }

    /* Set data */
    Set_LED_Data(LED_Setting);
}

/*------Lighting Mode-----------------------------------------*/
void FUNC_Off(volatile struct LED_Setting_Tag *LED_Setting)
{
    /* Mapping Color to LED Format */
    LED_SetSingleFromSetting(LED_Setting, 0U, 0U, 0U);
}

void FUNC_Static(volatile struct LED_Setting_Tag *LED_Setting)
{
    const uint8_t color_r = LED_Setting->Color_R;
    const uint8_t color_g = LED_Setting->Color_G;
    const uint8_t color_b = LED_Setting->Color_B;
    const uint8_t brightness = LED_Setting->Brightness;
    const uint8_t temp_r = LED_ScaleColor(color_r, brightness);
    const uint8_t temp_g = LED_ScaleColor(color_g, brightness);
    const uint8_t temp_b = LED_ScaleColor(color_b, brightness);

    /* Mapping Color to LED Format */
    LED_SetSingleFromSetting(LED_Setting, temp_r, temp_g, temp_b);
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
    uint32_t temp;
    uint8_t temp_r;
    uint8_t temp_g;
    uint8_t temp_b;
    uint32_t breathing_period;
    const uint32_t time_counter = LED_Setting->TimeCounter;
    const uint32_t speed = (uint32_t)LED_Setting->Speed;
    const uint8_t color_r = LED_Setting->Color_R;
    const uint8_t color_g = LED_Setting->Color_G;
    const uint8_t color_b = LED_Setting->Color_B;
    const uint8_t brightness = LED_Setting->Brightness;
    const uint32_t speed_period = 8U + LED_Divide(speed, 10U);

    /* Calculate Color */
    temp = LED_Modulo(LED_Divide(time_counter, speed_period), BreathingArraySize);
    temp_r = (uint8_t)LED_Divide((uint32_t)color_r * (uint32_t)brightness * (uint32_t)BreathingBright[temp], 255U * 100U);
    temp_g = (uint8_t)LED_Divide((uint32_t)color_g * (uint32_t)brightness * (uint32_t)BreathingBright[temp], 255U * 100U);
    temp_b = (uint8_t)LED_Divide((uint32_t)color_b * (uint32_t)brightness * (uint32_t)BreathingBright[temp], 255U * 100U);

    /* Mapping Color to LED Format */
    LED_SetSingleFromSetting(LED_Setting, temp_r, temp_g, temp_b);

    /* Reset CountingTime */
    breathing_period = speed_period * BreathingArraySize;

    if (time_counter >= breathing_period)
    {
        LED_Setting->TimeCounter = time_counter - breathing_period;
    }
}

void FUNC_Strobe(volatile struct LED_Setting_Tag *LED_Setting)
{
    uint32_t time_counter = LED_Setting->TimeCounter;
    const uint32_t speed = (uint32_t)LED_Setting->Speed;
    const uint32_t strobe_phase = (5U * speed) + 275U;
    const uint32_t strobe_period = strobe_phase * 2U;

    /* Reset CountingTime */
    while (time_counter >= strobe_period)
    {
        time_counter -= strobe_period;
    }
    LED_Setting->TimeCounter = time_counter;

    /* Extinguish */
    if (time_counter < strobe_phase)
    {
        /* Mapping Color to LED Format */
        LED_SetSingleFromSetting(LED_Setting, 0U, 0U, 0U);
    }
    /* Lighten */
    else
    {
        const uint8_t color_r = LED_Setting->Color_R;
        const uint8_t color_g = LED_Setting->Color_G;
        const uint8_t color_b = LED_Setting->Color_B;
        const uint8_t brightness = LED_Setting->Brightness;
        const uint8_t temp_r = LED_ScaleColor(color_r, brightness);
        const uint8_t temp_g = LED_ScaleColor(color_g, brightness);
        const uint8_t temp_b = LED_ScaleColor(color_b, brightness);

        /* Calculate Color */
        /* Mapping Color to LED Format */
        LED_SetSingleFromSetting(LED_Setting, temp_r, temp_g, temp_b);
    }
}

void FUNC_Cycling(volatile struct LED_Setting_Tag *LED_Setting)
{
    uint32_t time_counter = LED_Setting->TimeCounter;
    uint8_t color_index = LED_Setting->ColorIndex;
    const uint8_t speed = LED_Setting->Speed;
    const uint8_t brightness = LED_Setting->Brightness;
    const uint32_t cycle_period = 500U + ((uint32_t)speed * 10U);

    /* Reset CountingTime */
    if (time_counter >= cycle_period)
    {
        time_counter -= cycle_period;

        /* Switch to next color state. */
        if (color_index == (uint8_t)eColorRed)
        {
            color_index = (uint8_t)eColorGreen;
        }
        else if (color_index == (uint8_t)eColorGreen)
        {
            color_index = (uint8_t)eColorBlue;
        }
        else if (color_index == (uint8_t)eColorBlue)
        {
            color_index = (uint8_t)eColorRed;
        }
        else
        {
        }
    }
    LED_Setting->TimeCounter = time_counter;
    LED_Setting->ColorIndex = color_index;

    /* Mapping Color to LED Format */
    LED_SetSingleFromSetting(LED_Setting,
                             LED_ScaleColor(RainbowColor[color_index][0], brightness),
                             LED_ScaleColor(RainbowColor[color_index][1], brightness),
                             LED_ScaleColor(RainbowColor[color_index][2], brightness));
}

void FUNC_Random(volatile struct LED_Setting_Tag *LED_Setting)
{
    uint32_t time_counter = LED_Setting->TimeCounter;
    uint32_t random_state = LED_Setting->Random;
    uint8_t color_index = LED_Setting->ColorIndex;
    const uint8_t speed = LED_Setting->Speed;
    const uint8_t brightness = LED_Setting->Brightness;
    const uint32_t random_period = 500U + ((uint32_t)speed * 10U);

    /* Reset CountingTime */
    if (time_counter >= random_period)
    {
        uint8_t random_color_index;

        time_counter -= random_period;

        /* Calculate next random color state. */
        /* If next state is same as current state, random again. */
        do
        {
            random_state = (1664525UL * random_state) + 1013904223UL;
            random_color_index = (uint8_t)(random_state % RainbowSize);
        } while (random_color_index == color_index);

        /* Set next color state index. */
        color_index = random_color_index;
    }
    LED_Setting->TimeCounter = time_counter;
    LED_Setting->Random = random_state;
    LED_Setting->ColorIndex = color_index;

    /* Mapping Color to LED Format */
    LED_SetSingleFromSetting(LED_Setting,
                             LED_ScaleColor(RainbowColor[color_index][0], brightness),
                             LED_ScaleColor(RainbowColor[color_index][1], brightness),
                             LED_ScaleColor(RainbowColor[color_index][2], brightness));
}

void FUNC_Music(volatile struct LED_Setting_Tag *LED_Setting)
{
    uint32_t i;
    uint32_t j;
    uint8_t POP_Color[3] = {0};
    const uint32_t led_num = (uint32_t)LED_Setting->LEDNum;
    const uint8_t main_volume = LED_Setting->Main_Volume;
    const uint8_t brightness = LED_Setting->Brightness;
    const uint8_t music_action = LED_Setting->Music_Action;
    const uint8_t direction = LED_Setting->Direction;
    const uint32_t unit_volume = LED_Divide((uint32_t)main_volume * (uint32_t)led_num, 100U);
    uint8_t JAZZ_Display[LED_GEN2_MAX_LED_NUMBER][3] = {0};

    /* Init Array */
    for (uint32_t music_init_led_index = 0U; music_init_led_index < (uint32_t)led_num; music_init_led_index++)
    {
        for (i = 0; i < 3U; i++)
        {
            JAZZ_Display[music_init_led_index][i] = 0;
        }
    }

    /* POP */
    if (music_action == (uint8_t)Music_POP)
    {
        for (i = 0; i < 3U; i++)
        {
            POP_Color[i] = LED_MixColor(RainbowColor[(uint8_t)eColorBlue][i],
                                        RainbowColor[(uint8_t)eColorRed][i], main_volume);
        }

        /* Mapping Color to LED Format */
        LED_SetSingleFromSetting(LED_Setting,
                                 LED_ScaleColor(POP_Color[0], brightness),
                                 LED_ScaleColor(POP_Color[1], brightness),
                                 LED_ScaleColor(POP_Color[2], brightness));
    }
    /* JAZZ */
    else if (music_action == (uint8_t)Music_JAZZ)
    {
        for (j = 0U; j < unit_volume; j++)
        {
            for (i = 0; i < 3U; i++)
            {
                JAZZ_Display[j][i] = LED_GetColorComponent(LED_Setting, i);
            }
        }

        /* Mapping Color to LED Format */
        /* Direction forward or backward */
        if (direction == (uint8_t)Dir_Forward)
        {
            LED_SetArrayFromSetting(LED_Setting, &JAZZ_Display[0][0], brightness, direction);
        }
        else if (direction == (uint8_t)Dir_Backward)
        {
            LED_SetArrayFromSetting(LED_Setting, &JAZZ_Display[0][0], brightness, direction);
        }
        else
        {
        }
    }
    /* Mixed */
    else if (music_action == (uint8_t)Music_Mixed)
    {
        for (i = 0; i < 3U; i++)
        {
            POP_Color[i] = LED_MixColor(RainbowColor[(uint8_t)eColorBlue][i],
                                        RainbowColor[(uint8_t)eColorRed][i], main_volume);
        }

        for (j = 0U; j < unit_volume; j++)
        {
            for (i = 0; i < 3U; i++)
            {
                JAZZ_Display[j][i] = POP_Color[i];
            }
        }

        /* Mapping Color to LED Format */
        /* Direction forward or backward */
        LED_SetArrayFromSetting(LED_Setting, &JAZZ_Display[0][0], brightness, direction);
    }
    else
    {
    }
}

void FUNC_Wave(volatile struct LED_Setting_Tag *LED_Setting)
{
    uint32_t i;
    const uint8_t direction = LED_Setting->Direction;
    const uint32_t speed = (uint32_t)LED_Setting->Speed;
    uint32_t delta_t = 800U + (speed * 4U);
    uint32_t time_counter = LED_Setting->TimeCounter;
    uint32_t CurrentTime;
    uint8_t Color1[3] = {0};
    uint8_t Color2[3] = {0};
    uint8_t DisplayColor[3] = {0};
    uint32_t Temp;

    if (direction == (uint8_t)Dir_Forward)
    {
        /* Take Color1 and Color2 from Rainbow array by TimeCounter */
        Temp = LED_Modulo(LED_Divide(time_counter, delta_t), RainbowSize);

        for (i = 0; i < 3U; i++)
        {
            /* Set Color1 */
            Color1[i] = RainbowColor[Temp][i];
            /* Set Color2 */
            Color2[i] = RainbowColor[LED_Modulo(Temp + 1U, RainbowSize)][i];
        }
    }
    else if (direction == (uint8_t)Dir_Backward)
    {
        /* Take Color1 and Color2 from Rainbow array by TimeCounter */
        Temp = LED_Modulo(LED_Divide(time_counter, delta_t), RainbowSize);

        for (i = 0; i < 3U; i++)
        {
            /* Set Color1 */
            Color1[i] = RainbowColor[RainbowSize - 1U - Temp][i];
            /* Set Color2 */
            Color2[i] = RainbowColor[LED_Modulo((RainbowSize + RainbowSize - 2U) - Temp, RainbowSize)][i];
        }
    }
    else
    {
    }

    /* Reset CountingTime */
    if (time_counter >= (delta_t * RainbowSize))
    {
        time_counter -= (delta_t * RainbowSize);
    }
    LED_Setting->TimeCounter = time_counter;

    /* Judge current time and set mixed Displaycolor by color1 and color2 */
    CurrentTime = LED_Modulo(time_counter, delta_t);

    for (i = 0; i < 3U; i++)
    {
        const uint32_t color1_product = (uint32_t)Color1[i] * (delta_t - CurrentTime);
        const uint32_t color2_product = (uint32_t)Color2[i] * CurrentTime;

        DisplayColor[i] = (uint8_t)(LED_Divide(color1_product, delta_t) + LED_Divide(color2_product, delta_t));
    }

    /* Mapping Color to LED Format */
    LED_SetSingleFromSetting(LED_Setting, DisplayColor[0], DisplayColor[1], DisplayColor[2]);
}

void FUNC_Spring(volatile struct LED_Setting_Tag *LED_Setting)
{
    uint8_t DisplayColor[LED_GEN2_MAX_LED_NUMBER][3] = {0};
    uint8_t Color[cMeteor_LED][3];
    uint32_t loop;
    uint32_t spring_i;
    uint32_t time_counter = LED_Setting->TimeCounter;
    const uint32_t led_num = (uint32_t)LED_Setting->LEDNum;
    const uint32_t display_size = led_num + (2U * cMeteor_LED);
    const uint32_t delta_t = 5U + LED_Divide((uint32_t)LED_Setting->Speed, 20U);
    const uint32_t duration = delta_t * display_size;
    const uint32_t delay_t = LED_Divide(led_num * delta_t, 2U);
    const uint8_t direction = LED_Setting->Direction;
    const uint8_t brightness = LED_Setting->Brightness;

    /* Init Array */
    for (uint32_t spring_init_index = 0U; spring_init_index < display_size; spring_init_index++)
    {
        for (spring_i = 0U; spring_i < 3U; spring_i++)
        {
            DisplayColor[spring_init_index][spring_i] = 0U;
        }
    }

    /* Reset CountingTime */
    if (time_counter >= (2U * (duration + delay_t)))
    {
        time_counter -= (2U * (duration + delay_t));
    }

    /* Forward */
    if (time_counter < duration)
    {
        /* Loop to indicate which LED for first Color */
        loop = LED_Divide(time_counter, delta_t);

        for (spring_i = 0U; spring_i < 3U; spring_i++)
        {
            const uint8_t temp_color = LED_GetColorComponent(LED_Setting, spring_i);

            /* 1st 100%*/
            Color[0][spring_i] = temp_color;
            /* 2nd 75% */
            Color[1][spring_i] = (uint8_t)(((uint32_t)temp_color * 3U) >> 2U);
            /* 3rd 50% */
            Color[2][spring_i] = (uint8_t)((uint32_t)temp_color >> 1U);
            /* 4th 25% */
            Color[3][spring_i] = (uint8_t)((uint32_t)temp_color >> 2U);
        }

        for (uint32_t spring_forward_index = loop; spring_forward_index < (loop + cMeteor_LED); spring_forward_index++)
        {
            for (spring_i = 0U; spring_i < 3U; spring_i++)
            {
                DisplayColor[spring_forward_index][spring_i] = Color[spring_forward_index - loop][spring_i];
            }
        }
    }
    /* All Extinguish */
    else if (time_counter < (duration + delay_t))
    {
        for (uint32_t spring_first_extinguish_index = 0U; spring_first_extinguish_index < display_size; spring_first_extinguish_index++)
        {
            for (spring_i = 0U; spring_i < 3U; spring_i++)
            {
                DisplayColor[spring_first_extinguish_index][spring_i] = 0U;
            }
        }
    }
    else if (time_counter < ((2U * duration) + delay_t))
    {
        /* Loop to indicate which LED for first Color */
        loop = LED_Divide(time_counter - (duration + delay_t), delta_t);

        for (spring_i = 0U; spring_i < 3U; spring_i++)
        {
            const uint8_t temp_color = LED_GetColorComponent(LED_Setting, spring_i);

            /* 1st 25%*/
            Color[0][spring_i] = (uint8_t)((uint32_t)temp_color >> 2U);
            /* 2nd 50% */
            Color[1][spring_i] = (uint8_t)((uint32_t)temp_color >> 1U);
            /* 3rd 75% */
            Color[2][spring_i] = (uint8_t)(((uint32_t)temp_color * 3U) >> 2U);
            /* 4th 100% */
            Color[3][spring_i] = temp_color;
        }

        const uint32_t spring_start = led_num + cMeteor_LED - loop;

        for (uint32_t spring_backward_index = spring_start; spring_backward_index < (spring_start + cMeteor_LED); spring_backward_index++)
        {
            for (spring_i = 0U; spring_i < 3U; spring_i++)
            {
                DisplayColor[spring_backward_index][spring_i] = Color[spring_backward_index - spring_start][spring_i];
            }
        }
    }
    /* All Extinguish */
    else
    {
        for (uint32_t spring_second_extinguish_index = 0U; spring_second_extinguish_index < display_size; spring_second_extinguish_index++)
        {
            for (spring_i = 0U; spring_i < 3U; spring_i++)
            {
                DisplayColor[spring_second_extinguish_index][spring_i] = 0U;
            }
        }
    }
    LED_Setting->TimeCounter = time_counter;

    /* Mapping Color to LED Format */
    /* Direction forward or backward */
    LED_SetArrayFromSetting(LED_Setting, &DisplayColor[cMeteor_LED][0], brightness, direction);
}

void FUNC_Water(volatile struct LED_Setting_Tag *LED_Setting)
{
    const uint8_t Brightness[4] = {100, 15, 5, 15};
    const uint32_t meteor_num = 4U;
    uint32_t i;
    uint32_t cursor;
    uint8_t DisplayColor[LED_GEN2_MAX_LED_NUMBER][3] = {0};
    uint8_t Color[3];
    uint8_t next_color_index;
    const uint32_t led_num = (uint32_t)LED_Setting->LEDNum;
    uint32_t time_counter = LED_Setting->TimeCounter;
    uint8_t color_index = LED_Setting->ColorIndex;
    const uint8_t direction = LED_Setting->Direction;
    const uint8_t brightness = LED_Setting->Brightness;
    const uint32_t phase_period = 1000U + ((uint32_t)LED_Setting->Speed * 4U);
    const uint32_t motion_period = LED_Divide(phase_period, 8U);
    const uint32_t time_in_phase = LED_Modulo(time_counter, phase_period);

    /* Reset CountingTime */
    if (time_counter >= phase_period)
    {
        time_counter -= phase_period;

        /* If color index is overflow, reset color index. */
        if ((uint32_t)color_index >= (RainbowSize - 1U))
        {
            color_index = (uint8_t)eColorRed;
        }
        else
        {
            color_index++;
        }
    }
    LED_Setting->TimeCounter = time_counter;
    LED_Setting->ColorIndex = color_index;

    next_color_index = (uint8_t)LED_Modulo((uint32_t)color_index + 1U, RainbowSize);

    /* Set Mixed Color = Color1 + Color2 */
    for (i = 0; i < 3U; i++)
    {
        const uint32_t current_color = LED_Divide((uint32_t)RainbowColor[color_index][i] * (phase_period - time_in_phase), phase_period);
        const uint32_t next_color = LED_Divide((uint32_t)RainbowColor[next_color_index][i] * time_in_phase, phase_period);

        Color[i] = (uint8_t)(current_color + next_color);
    }

    /* Calculate the cursor for moving the Brightness */
    cursor = LED_Modulo(LED_Divide(time_counter, motion_period), meteor_num);

    /* Sets the Light Bar array and modified the brightness for meteor effect. */
    for (uint32_t water_led_index = 0U; water_led_index < led_num; water_led_index++)
    {
        for (i = 0; i < 3U; i++)
        {
            const uint32_t brightness_index = LED_Modulo(cursor + water_led_index, meteor_num);

            DisplayColor[water_led_index][i] = (uint8_t)LED_Divide((uint32_t)Color[i] * (uint32_t)Brightness[brightness_index], 100U);
        }
    }

    /* Mapping Color to LED Format */
    /* Direction forward or backward */
    if (direction == (uint8_t)Dir_Forward)
    {
        LED_SetArrayFromSetting(LED_Setting, &DisplayColor[0][0], brightness, (uint8_t)Dir_Backward);
    }
    else if (direction == (uint8_t)Dir_Backward)
    {
        LED_SetArrayFromSetting(LED_Setting, &DisplayColor[0][0], brightness, (uint8_t)Dir_Forward);
    }
    else
    {
    }
}

void FUNC_Rainbow(volatile struct LED_Setting_Tag *LED_Setting)
{
    uint8_t DisplayColor[LED_GEN2_MAX_LED_NUMBER][3] = {0};
    uint32_t i;
    uint32_t current_time;
    uint8_t color_cursor1;
    const uint32_t led_num = (uint32_t)LED_Setting->LEDNum;
    const uint32_t speed = (uint32_t)LED_Setting->Speed;
    const uint8_t direction = LED_Setting->Direction;
    const uint8_t brightness = LED_Setting->Brightness;
    uint32_t time_counter = LED_Setting->TimeCounter;
    const uint32_t speed_divisor = LED_Divide(255U - speed, 3U) + 5U;
    const uint32_t total_duration = LED_Divide(60000U, speed_divisor);
    const uint32_t next_time = LED_Divide(total_duration, 60U);
    const uint32_t delta_t = LED_Divide(total_duration, RainbowSize);

    /* Init Array */
    for (uint32_t rainbow_init_led_index = 0U; rainbow_init_led_index < led_num; rainbow_init_led_index++)
    {
        for (i = 0; i < 3U; i++)
        {
            DisplayColor[rainbow_init_led_index][i] = 0;
        }
    }

    /* Duration for one time effect */
    /* Current Time */
    current_time = LED_Modulo(time_counter, total_duration);

    /* Reset CountingTime */
    if (time_counter >= total_duration)
    {
        time_counter -= total_duration;
    }
    LED_Setting->TimeCounter = time_counter;

    /* Mix Color */
    for (uint32_t rainbow_mix_led_index = 0U; rainbow_mix_led_index < led_num; rainbow_mix_led_index++)
    {
        if (current_time >= total_duration)
        {
            current_time -= total_duration;
        }

        if (current_time >= (delta_t * RainbowSize))
        {
            /* Color Cursor */
            color_cursor1 = 0U;

            for (i = 0; i < 3U; i++)
            {
                DisplayColor[led_num - rainbow_mix_led_index - 1U][i] = RainbowColor[color_cursor1][i];
            }
        }
        else
        {
            uint8_t color_cursor2;

            /* Color Cursor */
            color_cursor1 = (uint8_t)LED_Modulo(LED_Divide(current_time, delta_t), RainbowSize);
            color_cursor2 = (uint8_t)LED_Modulo((uint32_t)color_cursor1 + 1U, RainbowSize);

            for (i = 0; i < 3U; i++)
            {
                const uint32_t color1_weight = (((uint32_t)color_cursor1 + 1U) * delta_t) - current_time;
                const uint32_t color2_weight = current_time - ((uint32_t)color_cursor1 * delta_t);
                const uint32_t color_product = ((uint32_t)RainbowColor[color_cursor1][i] * color1_weight) +
                                                ((uint32_t)RainbowColor[color_cursor2][i] * color2_weight);

                DisplayColor[led_num - rainbow_mix_led_index - 1U][i] = (uint8_t)LED_Divide(color_product, delta_t);
            }
        }

        current_time += next_time;
    }

    /* Mapping Color to LED Format */
    /* Direction forward or backward */
    LED_SetArrayFromSetting(LED_Setting, &DisplayColor[0][0], brightness, direction);
}
