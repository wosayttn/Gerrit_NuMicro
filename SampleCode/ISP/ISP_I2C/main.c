/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to update chip flash data through I2C interface.
 *           Nuvoton NuMicro ISP Programming Tool is also required in this
 *           sample code to connect with chip I2C and assign update file
 *           of Flash.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "targetdev.h"
#include "isp_user.h"
#include "i2c_transfer.h"

int32_t SYS_Init(void)
{
    /*------------------------------------------------------------------------*/
    /* Init System Clock                                                      */
    /*------------------------------------------------------------------------*/
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 to HCLK/2 */
    CLK_SET_PCLK0DIV(CLK_PCLKDIV_APB0DIV_DIV2);
    /* Set PCLK1 to HCLK/2 */
    CLK_SET_PCLK1DIV(CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable module clock */
    CLK_EnableModuleClock(ISP_MODULE);
    /* Enable I2C1 clock */
    CLK_EnableModuleClock(I2C1_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set I2C1 multi-function pins */
    SET_I2C1_SDA_PA2();
    SET_I2C1_SCL_PA3();

    /* I2C pin enable schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk | GPIO_SMTEN_SMTEN3_Msk;

    return 0;
}

int32_t main(void)
{
    uint32_t au32CmdBuff[16];
    int32_t  i32TimeoutInMS = 300;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable ISP */
    FMC_Open();

    /* Init System, peripheral clock and multi-function I/O */
    if (SYS_Init() < 0)
        goto _APROM;

    /* Init I2C */
    I2C_Init();
    /* Enable APROM and Data Flash update */
    FMC_ENABLE_AP_UPDATE();
    FMC_ENABLE_DF_UPDATE();

    /* Get APROM and Data Flash size */
    g_u32ApromSize = GetApromSize();

    while (i32TimeoutInMS > 0)
    {
        if (bI2cDataReady == 1)
        {
            goto _ISP;
        }

        CLK_SysTickDelay(1000);
        i32TimeoutInMS--;
    }

    /* Timeout then go to APROM */
    if (i32TimeoutInMS == 0)
        goto _APROM;

_ISP:

    while (1)
    {
        if (bI2cDataReady == 1)
        {
            /* Disable I2C IRQ until ParseCmd() is finished to prevent returning incomplete data prematurely */
            NVIC_DisableIRQ(I2C1_IRQn);
            memcpy(au32CmdBuff, i2c_rcvbuf, 64);
            bI2cDataReady = 0;
            ParseCmd((unsigned char *)au32CmdBuff, 64);
            NVIC_EnableIRQ(I2C1_IRQn);
        }
    }

_APROM:
    /* Reset system and boot from APROM */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);
    NVIC_SystemReset();

    /* Code should not reach here ! */
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
