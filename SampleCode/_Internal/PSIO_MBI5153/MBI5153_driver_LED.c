/******************************************************************************
 * @file        main.c
 * @version     V3.00
 * $Revision:   1 $
 * $Date:       17/08/04 15:36 $
 * @brief       MBI5153 LED Driver
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <string.h>
#include "NuMicro.h"
#include "MBI5153_driver_LED.h"

#if (PATTERN_ON_FLASH==1)
    static uint32_t *mbi5153_buf;
    static MBI5153_t mbi5153;
#else
    /* PSIO fetch 32 bit data by PDMA */
    static uint16_t mbi5153_buf[PSIO_DMA_PER_LINE_DATA_CNT];
    static MBI5153_t mbi5153;
#endif
//static uint8_t u8TDFlag =0;

//void PDMA_IRQHandler(void)
//{
//    volatile uint32_t u32Reg;

//    u32Reg = PDMA->INTSTS;
//    if((u32Reg & PDMA_INTSTS_TDIF_Msk) == PDMA_INTSTS_TDIF_Msk)     /* transfer down */
//    {
//        u32Reg = PDMA->TDSTS;

//        if(u32Reg&(PDMA_TDSTS_TDIF0_Msk<<PDMA_CHANNEL)){
//            u8TDFlag=1;
//
//            /* Wait for slot controller is not busy */
//            while(PSIO_GET_BUSY_FLAG(PSIO, SC_CTRL));
//        }

//        PDMA->TDSTS = u32Reg;
//    }
//}

static void PSIO_Setting(TRANSFER_TYPE type)
{
    switch (type)
    {
        case    TRANSFER_PREACTIVE:
            /* Disable output by PDMA */
            PSIO_CLEAR_PDMA_OUTPUT(PSIO, (PSIO_PDMACTL_OPIN0EN_Msk << LE_PIN) | (PSIO_PDMACTL_OPIN1EN_Msk << DATA_PIN));

            /* Loop slot0~slot1 repeat 13 */
            PSIO_SET_SCCTL(PSIO, SC_CTRL, PSIO_SLOT0, PSIO_SLOT1, 13, PSIO_REPEAT_DISABLE);

            /* Set Pin output data width as 13 bit */
            PSIO_SET_WIDTH(PSIO, LE_PIN, 0, 13);
            PSIO_SET_WIDTH(PSIO, DATA_PIN, 0, 13);

            /* Set Pin output data depth as 1 */
            PSIO_SET_OUTPUT_DEPTH(PSIO, LE_PIN, PSIO_DEPTH1);
            PSIO_SET_OUTPUT_DEPTH(PSIO, DATA_PIN, PSIO_DEPTH1);

            /* Set output data */
            PSIO_SET_OUTPUT_DATA(PSIO, LE_PIN, 0x3FFF);
            PSIO_SET_OUTPUT_DATA(PSIO, DATA_PIN, 0x0);
            break;

        case    TRANSFER_SWRESET:
            /* Disable output by PDMA */
            PSIO_CLEAR_PDMA_OUTPUT(PSIO, (PSIO_PDMACTL_OPIN0EN_Msk << LE_PIN) | (PSIO_PDMACTL_OPIN1EN_Msk << DATA_PIN));

            /* Loop slot0~slot1 repeat 1 */
            PSIO_SET_SCCTL(PSIO, SC_CTRL, PSIO_SLOT0, PSIO_SLOT1, 0x9, PSIO_REPEAT_DISABLE);

            /* Set Pin output data width as 10 bit */
            PSIO_SET_WIDTH(PSIO, LE_PIN, 0, 10);
            PSIO_SET_WIDTH(PSIO, DATA_PIN, 0, 10);

            /* Set Pin output data depth as 1 */
            PSIO_SET_OUTPUT_DEPTH(PSIO, LE_PIN, PSIO_DEPTH1);
            PSIO_SET_OUTPUT_DEPTH(PSIO, DATA_PIN, PSIO_DEPTH1);

            /* Set output data */
            PSIO_SET_OUTPUT_DATA(PSIO, LE_PIN, 0x3FF);
            PSIO_SET_OUTPUT_DATA(PSIO, DATA_PIN, 0x0);
            break;

        case    TRANSFER_DATA:
            /* Loop slot0~slot1 unlimit */
            PSIO_SET_SCCTL(PSIO, SC_CTRL, PSIO_SLOT0, PSIO_SLOT1, 0x3F, PSIO_REPEAT_DISABLE);

            /* Set Pin output data width as 16 bit */
            PSIO_SET_WIDTH(PSIO, LE_PIN, 0, 16);
            PSIO_SET_WIDTH(PSIO, DATA_PIN, 0, 16);

            /* Set Pin output data depth as 2 */
            PSIO_SET_OUTPUT_DEPTH(PSIO, LE_PIN, PSIO_DEPTH2);
            PSIO_SET_OUTPUT_DEPTH(PSIO, DATA_PIN, PSIO_DEPTH2);

            /* Set slot controller trigger source as software trigger */
            PSIO_SET_TRIGSRC(PSIO, SC_CTRL, PSIO_SW_TRIGGER);

            /* Enable PDMA channel x */
            PDMA->CHCTL |= (1 << PDMA_CHANNEL);

            PSIO_SET_PDMA_OUTPUT(PSIO, SC_CTRL, (PSIO_PDMACTL_OPIN0EN_Msk << LE_PIN) | (PSIO_PDMACTL_OPIN0EN_Msk << DATA_PIN));

            /* Set request source */
            PDMA_SetTransferMode(PDMA, PDMA_CHANNEL, PDMA_PSIO_TX, 0, 0);
#if (PATTERN_ON_FLASH==1)
            //Set PDMA transfer configuration
            PDMA->DSCT[PDMA_CHANNEL].CTL =
                PDMA_OP_BASIC |
                PDMA_REQ_SINGLE |
                PDMA_SAR_INC |                /* source address -> incremented */
                PDMA_DAR_FIX |                /* destination address -> fixed(PSIO) */
                PDMA_WIDTH_32 |               /* transfer width -> 32-bit */
                //PDMA_TBINT_DISABLE |
                (((PSIO_DMA_PER_LINE_DATA_CNT / 2) * MBI5152_SCAN_LINE_NUM) << PDMA_DSCT_CTL_TXCNT_Pos);
            PDMA->DSCT[PDMA_CHANNEL].SA = (uint32_t)mbi5153_buf;
            PDMA->DSCT[PDMA_CHANNEL].DA = (uint32_t)(&(PSIO->PODAT));
            break;
#else
            //Set PDMA transfer configuration
            PDMA->DSCT[PDMA_CHANNEL].CTL =
                PDMA_OP_BASIC |
                PDMA_REQ_SINGLE |
                PDMA_SAR_INC |                /* source address -> incremented */
                PDMA_DAR_FIX |                /* destination address -> fixed(PSIO) */
                PDMA_WIDTH_32 |               /* transfer width -> 32-bit */
                //PDMA_TBINT_DISABLE |
                ((PSIO_DMA_PER_LINE_DATA_CNT / 2) << PDMA_DSCT_CTL_TXCNT_Pos);
            PDMA->DSCT[PDMA_CHANNEL].SA = (uint32_t)mbi5153_buf;
            PDMA->DSCT[PDMA_CHANNEL].DA = (uint32_t)(&(PSIO->PODAT));
            break;
#endif

        case    TRANSFER_VERTICALSYNC:
            /* Disable output by PDMA */
            PSIO_CLEAR_PDMA_OUTPUT(PSIO, (PSIO_PDMACTL_OPIN0EN_Msk << LE_PIN) | (PSIO_PDMACTL_OPIN1EN_Msk << DATA_PIN));

            /* Loop slot0~slot1 repeat 1 */
            PSIO_SET_SCCTL(PSIO, SC_CTRL, PSIO_SLOT0, PSIO_SLOT1, 0x1, PSIO_REPEAT_DISABLE);

            /* Set Pin output data width as 2 bit */
            PSIO_SET_WIDTH(PSIO, LE_PIN, 0, 2);
            PSIO_SET_WIDTH(PSIO, DATA_PIN, 0, 2);

            /* Set Pin output data depth as 1 */
            PSIO_SET_OUTPUT_DEPTH(PSIO, LE_PIN, PSIO_DEPTH1);
            PSIO_SET_OUTPUT_DEPTH(PSIO, DATA_PIN, PSIO_DEPTH1);

            /* Set output data */
            PSIO_SET_OUTPUT_DATA(PSIO, LE_PIN, 0x3);
            PSIO_SET_OUTPUT_DATA(PSIO, DATA_PIN, 0x0);
            break;

        case TRANSFER_CONFIG:
            /* Disable output by PDMA */
            PSIO_CLEAR_PDMA_OUTPUT(PSIO, (PSIO_PDMACTL_OPIN0EN_Msk << LE_PIN) | (PSIO_PDMACTL_OPIN1EN_Msk << DATA_PIN));

            /* Loop slot0~slot1 repeat 1 */
            PSIO_SET_SCCTL(PSIO, SC_CTRL, PSIO_SLOT0, PSIO_SLOT1, 15, PSIO_REPEAT_DISABLE);

            /* Set Pin output data width as 16 bit */
            PSIO_SET_WIDTH(PSIO, LE_PIN, 0, 16);
            PSIO_SET_WIDTH(PSIO, DATA_PIN, 0, 16);

            /* Set Pin output data depth as 1 */
            PSIO_SET_OUTPUT_DEPTH(PSIO, LE_PIN, PSIO_DEPTH1);
            PSIO_SET_OUTPUT_DEPTH(PSIO, DATA_PIN, PSIO_DEPTH1);
            break;

        default:
            break;
    }
}

void MBI5153_Open(void)
{
    int i;

    /* PSIO pin general setting */
    PSIO_SET_GENCTL(PSIO, LE_PIN, PSIO_PIN_ENABLE, SC_CTRL, PSIO_OUTPUT_MODE, PSIO_LOW_LEVEL, PSIO_LOW_LEVEL);
    PSIO_SET_GENCTL(PSIO, DATA_PIN, PSIO_PIN_ENABLE, SC_CTRL, PSIO_OUTPUT_MODE, PSIO_LOW_LEVEL, PSIO_LOW_LEVEL);
    PSIO_SET_GENCTL(PSIO, DCLK_PIN, PSIO_PIN_ENABLE, SC_CTRL, PSIO_OUTPUT_MODE, PSIO_LOW_LEVEL, PSIO_LOW_LEVEL);

    /* Set data order ad MSB */
    PSIO_SET_ORDER(PSIO, LE_PIN, PSIO_MSB);
    PSIO_SET_ORDER(PSIO, DATA_PIN, PSIO_MSB);

    PSIO_SET_CHECKPOINT(PSIO, LE_PIN, PSIO_CP0, PSIO_SLOT0);
    //    PSIO_SET_CHECKPOINT(PSIO, LE_PIN, PSIO_CP1, PSIO_SLOT1);
    PSIO_SET_CHECKPOINT(PSIO, DATA_PIN, PSIO_CP0, PSIO_SLOT0);
    //    PSIO_SET_CHECKPOINT(PSIO, DATA_PIN, PSIO_CP1, PSIO_SLOT1);
    PSIO_SET_CHECKPOINT(PSIO, DCLK_PIN, PSIO_CP0, PSIO_SLOT0);
    PSIO_SET_CHECKPOINT(PSIO, DCLK_PIN, PSIO_CP1, PSIO_SLOT1);

    /* Set slot0/1 tick count as 1 */
    PSIO_SCSLOT_SET_SLOT(PSIO, SC_CTRL, PSIO_SLOT0, 1);
    PSIO_SCSLOT_SET_SLOT(PSIO, SC_CTRL, PSIO_SLOT1, 1);

    /* Set Pin check point action */
    PSIO_SET_ACTION(PSIO, LE_PIN, PSIO_CP0, PSIO_OUT_BUFFER);
    PSIO_SET_ACTION(PSIO, LE_PIN, PSIO_CP1, PSIO_IN_STATUS_UPDATE);
    PSIO_SET_ACTION(PSIO, DATA_PIN, PSIO_CP0, PSIO_OUT_BUFFER);
    PSIO_SET_ACTION(PSIO, DATA_PIN, PSIO_CP1, PSIO_IN_STATUS_UPDATE);
    PSIO_SET_ACTION(PSIO, DCLK_PIN, PSIO_CP0, PSIO_OUT_LOW);
    PSIO_SET_ACTION(PSIO, DCLK_PIN, PSIO_CP1, PSIO_OUT_HIGH);

    PSIO_Setting(TRANSFER_SWRESET);

    /* Trigger slot controller */
    PSIO_START_SC(PSIO, SC_CTRL);

    /* Wait for slot controller is not busy */
    while (PSIO_GET_BUSY_FLAG(PSIO, SC_CTRL));

    /* Set Config Register 1  */
    for (i = 0; i < RED_PCS; i++)
        mbi5153.red[i].configReg1 = CONFIG_REG1_VAL;

    for (i = 0; i < GREEN_PCS; i++)
        mbi5153.green[i].configReg1 = CONFIG_REG1_VAL;

    for (i = 0; i < BLUE_PCS; i++)
        mbi5153.blue[i].configReg1 = CONFIG_REG1_VAL;

    for (i = 0; i < LINE_PCS; i++)
        mbi5153.line[i].configReg1 = CONFIG_REG1_VAL;

    PSIO_Setting(TRANSFER_PREACTIVE);

    /* Trigger slot controller */
    PSIO_START_SC(PSIO, SC_CTRL);

    /* Wait for slot controller is not busy */
    while (PSIO_GET_BUSY_FLAG(PSIO, SC_CTRL));

    PSIO_Setting(TRANSFER_CONFIG);

    /* Transfer  Config Register 1  */
    for (i = (LINE_PCS - 1); i >= 0; i--)
    {
        /* Set output data */
        PSIO_SET_OUTPUT_DATA(PSIO, LE_PIN, 0x00);
        PSIO_SET_OUTPUT_DATA(PSIO, DATA_PIN, mbi5153.line[i].configReg1);

        /* Trigger slot controller */
        PSIO_START_SC(PSIO, SC_CTRL);

        /* Wait for slot controller is not busy */
        while (PSIO_GET_BUSY_FLAG(PSIO, SC_CTRL));
    }

    for (i = (RED_PCS - 1); i >= 0; i--)
    {
        /* Set output data */
        PSIO_SET_OUTPUT_DATA(PSIO, LE_PIN, 0x00);
        PSIO_SET_OUTPUT_DATA(PSIO, DATA_PIN, mbi5153.red[i].configReg1);

        /* Trigger slot controller */
        PSIO_START_SC(PSIO, SC_CTRL);

        /* Wait for slot controller is not busy */
        while (PSIO_GET_BUSY_FLAG(PSIO, SC_CTRL));
    }

    for (i = (GREEN_PCS - 1); i >= 0; i--)
    {
        /* Set output data */
        PSIO_SET_OUTPUT_DATA(PSIO, LE_PIN, 0x00);
        PSIO_SET_OUTPUT_DATA(PSIO, DATA_PIN, mbi5153.green[i].configReg1);

        /* Trigger slot controller */
        PSIO_START_SC(PSIO, SC_CTRL);

        /* Wait for slot controller is not busy */
        while (PSIO_GET_BUSY_FLAG(PSIO, SC_CTRL));
    }

    for (i = (BLUE_PCS - 1); i >= 0; i--)
    {
        if (i == 0)
        {
            PSIO_SET_OUTPUT_DATA(PSIO, LE_PIN, 0x0F);
            PSIO_SET_OUTPUT_DATA(PSIO, DATA_PIN, mbi5153.blue[i].configReg1);

            /* Trigger slot controller */
            PSIO_START_SC(PSIO, SC_CTRL);

            /* Wait for slot controller is not busy */
            while (PSIO_GET_BUSY_FLAG(PSIO, SC_CTRL));
        }
        else
        {
            PSIO_SET_OUTPUT_DATA(PSIO, LE_PIN, 0x0);
            PSIO_SET_OUTPUT_DATA(PSIO, DATA_PIN, mbi5153.blue[i].configReg1);

            /* Trigger slot controller */
            PSIO_START_SC(PSIO, SC_CTRL);

            /* Wait for slot controller is not busy */
            while (PSIO_GET_BUSY_FLAG(PSIO, SC_CTRL));
        }
    }


    /* Set Config Register 2  */
    for (i = 0; i < RED_PCS; i++)
        mbi5153.red[i].configReg2 = CONFIG_REG2_VAL_FOR_R;

    for (i = 0; i < GREEN_PCS; i++)
        mbi5153.green[i].configReg2 = CONFIG_REG2_VAL_FOR_R/*CONFIG_REG2_VAL_FOR_GB*/;

    for (i = 0; i < BLUE_PCS; i++)
        mbi5153.blue[i].configReg2 = CONFIG_REG2_VAL_FOR_R/*CONFIG_REG2_VAL_FOR_GB*/;

    for (i = 0; i < LINE_PCS; i++)
        mbi5153.line[i].configReg2 = CONFIG_REG2_VAL_FOR_R;

    PSIO_Setting(TRANSFER_PREACTIVE);

    /* Trigger slot controller */
    PSIO_START_SC(PSIO, SC_CTRL);

    /* Wait for slot controller is not busy */
    while (PSIO_GET_BUSY_FLAG(PSIO, SC_CTRL));

    PSIO_Setting(TRANSFER_CONFIG);

    /* Transfer  Config Register 2  */
    for (i = (LINE_PCS - 1); i >= 0; i--)
    {
        /* Set output data */
        PSIO_SET_OUTPUT_DATA(PSIO, LE_PIN, 0x00);
        PSIO_SET_OUTPUT_DATA(PSIO, DATA_PIN, mbi5153.line[i].configReg2);

        /* Trigger slot controller */
        PSIO_START_SC(PSIO, SC_CTRL);

        /* Wait for slot controller is not busy */
        while (PSIO_GET_BUSY_FLAG(PSIO, SC_CTRL));
    }

    for (i = (RED_PCS - 1); i >= 0; i--)
    {
        /* Set output data */
        PSIO_SET_OUTPUT_DATA(PSIO, LE_PIN, 0x00);
        PSIO_SET_OUTPUT_DATA(PSIO, DATA_PIN, mbi5153.red[i].configReg2);

        /* Trigger slot controller */
        PSIO_START_SC(PSIO, SC_CTRL);

        /* Wait for slot controller is not busy */
        while (PSIO_GET_BUSY_FLAG(PSIO, SC_CTRL));
    }

    for (i = (GREEN_PCS - 1); i >= 0; i--)
    {
        /* Set output data */
        PSIO_SET_OUTPUT_DATA(PSIO, LE_PIN, 0x00);
        PSIO_SET_OUTPUT_DATA(PSIO, DATA_PIN, mbi5153.green[i].configReg2);

        /* Trigger slot controller */
        PSIO_START_SC(PSIO, SC_CTRL);

        /* Wait for slot controller is not busy */
        while (PSIO_GET_BUSY_FLAG(PSIO, SC_CTRL));
    }

    for (i = (BLUE_PCS - 1); i >= 0; i--)
    {
        if (i == 0)
        {
            PSIO_SET_OUTPUT_DATA(PSIO, LE_PIN, 0xFF);
            PSIO_SET_OUTPUT_DATA(PSIO, DATA_PIN, mbi5153.blue[i].configReg2);

            /* Trigger slot controller */
            PSIO_START_SC(PSIO, SC_CTRL);

            /* Wait for slot controller is not busy */
            while (PSIO_GET_BUSY_FLAG(PSIO, SC_CTRL));
        }
        else
        {
            PSIO_SET_OUTPUT_DATA(PSIO, LE_PIN, 0x0);
            PSIO_SET_OUTPUT_DATA(PSIO, DATA_PIN, mbi5153.blue[i].configReg2);

            /* Trigger slot controller */
            PSIO_START_SC(PSIO, SC_CTRL);

            /* Wait for slot controller is not busy */
            while (PSIO_GET_BUSY_FLAG(PSIO, SC_CTRL));
        }
    }
}

#if (PATTERN_ON_FLASH==1)
void MBI5153_LoadPattern(FRAME_BUF_t sLED_Pattern)
{
    mbi5153_buf = sLED_Pattern.pu32Pattern;
}

#else
void MBI5153_LoadPattern(FRAME_BUF_t *sLED_Pattern, uint8_t u8Line)
{
    uint32_t i, index, color_buf_index;

    memset(mbi5153_buf, 0x0000, sizeof(uint16_t) * PSIO_DMA_PER_LINE_DATA_CNT);

    if (u8Line < MBI5153_CHANNEL_NUM)
    {
        index = FIRST_LINE_BUF_INDEX + (MBI5153_TOTAL_PCS * u8Line * 2);
    }
    else
    {
        index = SECEND_LINE_BUF_INDEX + (MBI5153_TOTAL_PCS * (u8Line - MBI5153_CHANNEL_NUM) * 2);
    }

    mbi5153_buf[index] = 0xFFFC;

    color_buf_index = u8Line * PER_LINE_COLOR_DATA_CNT;

    for (i = 0; i < MBI5153_CHANNEL_NUM; i++)
    {
        index = FIRRST_RED_BUF_INDEX + (i * PER_CHANNEL_DATA_CNT);

        mbi5153_buf[index] = sLED_Pattern->red[(4 * MBI5153_CHANNEL_NUM - 1) - i + color_buf_index];
        mbi5153_buf[index + 1] = sLED_Pattern->red[(3 * MBI5153_CHANNEL_NUM - 1) - i + color_buf_index];
        mbi5153_buf[index + 4] = sLED_Pattern->red[(2 * MBI5153_CHANNEL_NUM - 1) - i + color_buf_index];
        mbi5153_buf[index + 5] = sLED_Pattern->red[(1 * MBI5153_CHANNEL_NUM - 1) - i + color_buf_index];

        index = FIRRST_GREEN_BUF_INDEX + (i * PER_CHANNEL_DATA_CNT);
        mbi5153_buf[index] = sLED_Pattern->green[(4 * MBI5153_CHANNEL_NUM - 1) - i + color_buf_index];
        mbi5153_buf[index + 1] = sLED_Pattern->green[(3 * MBI5153_CHANNEL_NUM - 1) - i + color_buf_index];
        mbi5153_buf[index + 4] = sLED_Pattern->green[(2 * MBI5153_CHANNEL_NUM - 1) - i + color_buf_index];
        mbi5153_buf[index + 5] = sLED_Pattern->green[(1 * MBI5153_CHANNEL_NUM - 1) - i + color_buf_index];

        index = FIRST_BLUE_BUF_INDEX + (i * PER_CHANNEL_DATA_CNT);
        mbi5153_buf[index] = sLED_Pattern->blue[(4 * MBI5153_CHANNEL_NUM - 1) - i + color_buf_index];
        mbi5153_buf[index + 1] = sLED_Pattern->blue[(3 * MBI5153_CHANNEL_NUM - 1) - i + color_buf_index];
        mbi5153_buf[index + 4] = sLED_Pattern->blue[(2 * MBI5153_CHANNEL_NUM - 1) - i + color_buf_index];
        mbi5153_buf[index + 5] = sLED_Pattern->blue[(1 * MBI5153_CHANNEL_NUM - 1) - i + color_buf_index];

        mbi5153_buf[index + 3] = 0x0001;
    }

    //    for(i = 0; i < PSIO_DMA_PER_LINE_DATA_CNT; i=i+2){
    //        printf("0x%04x%04x, ", mbi5153_buf[i+1], mbi5153_buf[i]);
    //        if((!(i%20))&&(i!=0)){
    //            printf("\n");
    //        }
    //    }
    //    printf("\n");
}

#endif

void MBI5153_SetGrayScale(void)
{
    PSIO_Setting(TRANSFER_DATA);
    GCLK_ON();
    /* Trigger slot controller */
    PSIO_START_SC(PSIO, SC_CTRL);
}

TRANSFER_STATUS MBI5153_CheckStatus(void)
{
    if ((PDMA->TDSTS & (PDMA_TDSTS_TDIF0_Msk << PDMA_CHANNEL)) && (!PSIO_GET_BUSY_FLAG(PSIO, SC_CTRL)))
    {
        PDMA->TDSTS = PDMA_TDSTS_TDIF0_Msk << PDMA_CHANNEL;
        return TRANSFER_DONE;
    }
    else
    {
        return TRANSFER_BUSY;
    }
}

void MBI5153_verticalSync(void)
{
    GCLK_OFF();

    PSIO_Setting(TRANSFER_VERTICALSYNC);

    /* Trigger slot controller */
    PSIO_START_SC(PSIO, SC_CTRL);

    /* Wait for slot controller is not busy */
    while (PSIO_GET_BUSY_FLAG(PSIO, SC_CTRL));

    CLK_SysTickDelay(5);

    GCLK_ON();
}

void MBI5153_Close(void)
{
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
