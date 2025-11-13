/**************************************************************************//**
 * @file     TK_TC8260_AllInOne_Led.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/09/07 3:36p $
 * @brief    Touch Key LED Indicator
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"

#define OPT_USE_LED

#ifdef OPT_USE_LED

typedef struct tagLEG
{
    unsigned char u8Addr;
    unsigned char u8Msk;
} LED_T;


/**
  * @brief      Light the led(s)
  * @param[in]  onOff    0: Touch Key LED turn off
  *                      1: Touch Key LED turn on
  * @param[in]  chanN    the channel number to be configured. (TK1 = 1, TK2 = 2, TK3 = 3, TK4 = 4)
  * @retval     None.
  * @details    This function is used to light the led(s)
  */

void TK_lightLED(unsigned char onOff, int chanN)
{
#ifdef BOARD_TIM
    /* Tim */

    /*
     *  TK1 --> PA.8
     *  TK2 --> PA.9
     *  TK3 --> PA.10
     *  TK4 --> PA.11
     */
    if( (chanN >= 1) && (chanN <= 4) )
    {
        if(onOff == 1)      //On
            PA->DOUT = PA->DOUT & ~(1<<(chanN+7));
        else                //Off
            PA->DOUT = PA->DOUT | (1<<(chanN+7));
    }

#else
    /*
     *  TK1 --> PA.8
     *  TK2 --> PA.9
     *  TK3 --> PA.10
     *  TK4 --> PA.11
     */
    if( (chanN >= 1) && (chanN <= 4) )
    {
        if(onOff == 1)      //On
            PA->DOUT = PA->DOUT & ~(1<<(chanN+7));
        else                //Off
            PA->DOUT = PA->DOUT | (1<<(chanN+7));
    }
#endif
}

/**
  * @brief      Initialize touch key LED output state
  * @param      None.
  * @retval     None.
  * @details    This function is used to initialize touch key LED output state. Default MFP is GPIO
  *             Foe code size, not to set MFP regster.
  */
void InitLEDIO(void)
{

#ifdef BOARD_TIM
    /* Tim */

    /*  IF PA8 set to 1,  KEY1 LED will be turn off. If PA8 set to 0,  KEY1 LED will be turn on.
     *  IF PA9 set to 1,  KEY2 LED will be turn off. If PA9 set to 0,  KEY2 LED will be turn on.
     *  IF PA10 set to 1,  KEY3 LED will be turn off. If PA10 set to 0,  KEY3 LED will be turn on.
     *  IF PA11 set to 1,  KEY4 LED will be turn off. If PA11 set to 0,  KEY4 LED will be turn on.
     */
    GPIO_SetMode(PA, BIT8 | BIT9 | BIT10 | BIT11, GPIO_MODE_QUASI);

#else

    /*  IF PA8 set to 1,  KEY1 LED will be turn off. If PA8 set to 0,  KEY1 LED will be turn on.
     *  IF PA9 set to 1,  KEY2 LED will be turn off. If PA9 set to 0,  KEY2 LED will be turn on.
     *  IF PA10 set to 1,  KEY3 LED will be turn off. If PA10 set to 0,  KEY3 LED will be turn on.
     *  IF PA11 set to 1,  KEY4 LED will be turn off. If PA11 set to 0,  KEY4 LED will be turn on.
     */
    GPIO_SetMode(PA, BIT8 | BIT9 | BIT10 | BIT11, GPIO_MODE_QUASI);

#endif

}

#endif
