#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"
#include "tklib.h"
#include "TK_Demo.h"

extern volatile uint8_t gbIsFineTuneDone;
extern volatile uint8_t gFineTuneDoneTimeOut;
int8_t i8MpTickEventID = 0;

/**
  * The function was called if time out only to make sure no risk condition
  *
  *
  */
void TK_MP_Close()
{
    TickClearTickEvent(i8MpTickEventID);                    /* Stop time out timer */
}
/**
  * The function is time out callback function.
  *
  *
  */
#if defined(MASS_FINETUNE)
void TickCallback_FineTuneTimeOut(void)
{
//    S_TKFEAT* psTkFeat;

//    psTkFeat = TK_GetFeaturePtr();
    gFineTuneDoneTimeOut = 1;
    gbIsFineTuneDone = 1;
    TK_MP_Close();
}


void TK_MP_Open()
{
    gFineTuneDoneTimeOut = 0x0;
    gbIsFineTuneDone = 0x0;
    i8MpTickEventID = TickSetTickEvent(150, (void *)TickCallback_FineTuneTimeOut);    /* 3 second time out */
}
#endif

#if defined(MASS_FINETUNE)
void TK_MassProduction(int8_t* pai8Signal)
{
    uint16_t u16ChnMsk;
    uint8_t i;
//    S_KEYINFO* pKeyInfo;
    S_TKFEAT* psTkFeat;

//    pKeyInfo = TK_GetKeyInfoPtr();
    psTkFeat = TK_GetFeaturePtr();


    if(gFineTuneDoneTimeOut == 1)         /* Time out return directly */
        return;


    if(psTkFeat->u8BaseLineRound == 1)    /* Set by UART command - 'A': modify to update baseline time  */
    {
        //Check fine tune CCB stable?

        u16ChnMsk = TK_GetEnabledChannelMask(TK_KEY);
        u16ChnMsk |= TK_GetEnabledChannelMask(TK_SLIDER);
        u16ChnMsk |= TK_GetEnabledChannelMask(TK_WHEEL);

        gbIsFineTuneDone = 0xFF;
        for (i = 0; i < /*TKLIB_TOL_NUM_KEY*/ u8MaxScKeyNum ; i++)
        {
            if (u16ChnMsk & (1ul << i))
            {
                if(abs(pai8Signal[i]) > 4)  /* Change to 4 from 2 @2020/09/02 */
                {
                    gbIsFineTuneDone = 0;   /* If any channel's signal > 2, fine tune not yet complete */
                    break;
                }
            }
        }

        if( gbIsFineTuneDone == 0xFF )
        {
            /* callback function if fine tune done */
            gbIsFineTuneDone = 1;                                   /* Fine tune done */
        }
    }
}
#endif
