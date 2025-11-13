/**************************************************************************//**
 * @file     TK_Utility.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/09/07 3:36p $
 * @brief    Some Utility Funcion for Calibration Using.
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include "NuMicro.h"
#include "tklib.h"
#include "TK_Demo.h"

/**************************************************************************//**
 * TK0      PA.7
 * TK1      PA.6
 * TK2      PD.15
 * TK3      PA.5
 * TK4      PA.4
 * TK5      PA.3
 * TK6      PA.2
 * TK7      PA.1
 * TK8      PA.0  
 * TK9      PE.14
 * TK10     PC.5
 * TK11     PC.4
 * TK12     PC.3
 * TK13     PC.2
 * TK14     PD03 or PD7
 * TK15     PD02 or PD6
 * TK16     PD01 or PD5
 * TK16     PD00 or PD4 
 ******************************************************************************/

/**************************************************************************//**
  * @brief      Configure multi-function pin to touch key function
  * @param[in]  u16TkMsk Combination of enabled scan keys. Each bit corresponds to a touch key.
  *             Bit 0 represents touch key 0, bit 1 represents touch key 1...
  * @retval     None.
  * @details    This function is used to configure multi-function pin to touch key function
 ******************************************************************************/
void SetTkMultiFun(uint32_t u32TkMsk)
{
    S_TKFEAT* psTkFeat;
    psTkFeat = TK_GetFeaturePtr();
    unsigned int i;

    for (i = 0; i < (u8MaxScKeyNum /*TKLIB_TOL_NUM_KEY*/ +2); i++)
    {
        if ((1ul << i) & u32TkMsk)
        {
            switch(i)
            {
            case 0: /* TC8260 : PA.7 */
				DBG_PRINTF("TK0 MF\n");
                SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(0xFFUL << (3*8))) | (16UL << (3*8)); //TK0 PA07  16
                break;
            case 1: /* TC8260 : PA.6 */
                DBG_PRINTF("TK1 MF\n");
                SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(0xFFUL << (2*8))) | (16UL << (2*8)); //TK1 PA06  16
                break;
			case 2: /* M2L31 : PD.15 */
							
//				DBG_PRINTF("TK2 MF\n");
//              SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(0xFFUL << (1*8))) | (0UL << (1*8)); //TK2 PA05  16
						
                DBG_PRINTF("TK2 MF\n");
                SYS->GPD_MFP3 = (SYS->GPD_MFP3 & ~(0xFFUL << (3*8))) | (16UL << (3*8)); //TK2 PD15  16
                break;
			case 3: /* M2L31 : PA.5 */
                DBG_PRINTF("TK3 MF\n");
                SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(0xFFUL << (1*8))) | (16UL << (1*8)); //TK3 PA05  16
                break;
			case 4: /* M2L31 : PA.4 */
                DBG_PRINTF("TK4 MF\n");
                SYS->GPA_MFP1 = (SYS->GPA_MFP1 & ~(0xFFUL << (0*8))) | (16UL << (0*8)); //TK4 PA04  16
                break;
			case 5: /* M2L31 : PA.3 */
                DBG_PRINTF("TK5 MF\n");
                SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(0xFFUL << (3*8))) | (16UL << (3*8)); //TK5 PA03  16
						    //SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(0xFFUL << (3*8))) | (6UL << (3*8)); //TK5_SE PA03  16
                break;
			case 6: /* M2L31 : PA.2 */
                DBG_PRINTF("TK6\n");
                SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(0xFFUL << (2*8))) | (16UL << (2*8)); //TK6 PA02  16
                break;
			case 7: /* M2L31 : PA.1 */
                DBG_PRINTF("TK7\n");
                SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(0xFFUL << (1*8))) | (16UL << (1*8)); //TK7 PA01  16
                break;
			case 8: /* M2L31 : PA.0 */
                DBG_PRINTF("TK8\n");
                SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(0xFFUL << (0*8))) | (16UL << (0*8)); //TK8 PA00  16
                break;
			case 9: /* M2L31 : PE.14 */
                DBG_PRINTF("TK9\n");
                SYS->GPE_MFP3 = (SYS->GPE_MFP3 & ~(0xFFUL << (2*8))) | (16UL << (2*8)); //TK9 PE14  16
                break;
			case 10: /* M2L31 : PC.5*/
                DBG_PRINTF("TK10\n");
                SYS->GPC_MFP1 = (SYS->GPC_MFP1 & ~(0xFFUL << (1*8))) | (16UL << (1*8)); //TK10 PC05  16
                break;
			case 11: /* M2L31 : PC.4*/
                DBG_PRINTF("TK11\n");
                SYS->GPC_MFP1 = (SYS->GPC_MFP1 & ~(0xFFUL << (0*8))) | (16UL << (0*8)); //TK11 PC04  16
                break;
			case 12: /* M2L31 : PC.3 */
                DBG_PRINTF("TK12\n");
                SYS->GPC_MFP0 = (SYS->GPC_MFP0 & ~(0xFFUL << (3*8))) | (16UL << (3*8)); //TK12 PC03  16
                break;
            case 13: /* M2L31 : PC.2 */
                DBG_PRINTF("TK11\n");
                SYS->GPC_MFP0 = (SYS->GPC_MFP0 & ~(0xFFUL << (2*8))) | (16UL << (2*8)); //TK11 PC02  16
                break;
						
            case 14: /* M2L31 : PD03 or PD07 */
                DBG_PRINTF("TK14\n");
				DBG_PRINTF("PINSEL = 0x%x\n", psTkFeat->u32PinSel);
#if 1
                if (((psTkFeat->u32PinSel >> (14*2))&0x3) == 0)
                    SYS->GPD_MFP0 = (SYS->GPD_MFP0 & ~(0xFFUL << (3*8))) | (16UL << (3*8)); //TK14 PD03  16
                else
                    SYS->GPD_MFP1 = (SYS->GPD_MFP1 & ~(0xFFUL << (3*8))) | (16UL << (3*8)); //TK14 PD07  16
#else
                if (((psTkFeat->u32PinSel >> (13*2))&0x3) == 0)     //Because M25x is from TK13
                    SYS->GPD_MFP0 = (SYS->GPD_MFP0 & ~(0xFFUL << (3*8))) | (16UL << (3*8)); //TK14 PD03  16
                else
                    SYS->GPD_MFP1 = (SYS->GPD_MFP1 & ~(0xFFUL << (3*8))) | (16UL << (3*8)); //TK14 PD07  16
#endif
                break;
			case 15: /* M2L31 : PD02 or PD06 */
                DBG_PRINTF("TK15\n");
				DBG_PRINTF("PINSEL = 0x%x\n", psTkFeat->u32PinSel);
#if 1
                if (((psTkFeat->u32PinSel >> (15*2))&0x3) == 0)
                    SYS->GPD_MFP0 = (SYS->GPD_MFP0 & ~(0xFFUL << (2*8))) | (16UL << (2*8)); //TK15 PD02  16
                else
                    SYS->GPD_MFP1 = (SYS->GPD_MFP1 & ~(0xFFUL << (2*8))) | (16UL << (2*8)); //TK15 PD06  16
#else
                if (((psTkFeat->u32PinSel >> (14*2))&0x3) == 0)   //Because M25x is from TK14
                    SYS->GPD_MFP0 = (SYS->GPD_MFP0 & ~(0xFFUL << (2*8))) | (16UL << (2*8)); //TK14 PD02  16
                else
                    SYS->GPD_MFP1 = (SYS->GPD_MFP1 & ~(0xFFUL << (2*8))) | (16UL << (2*8)); //TK14 PD06  16
#endif
                break;
            case 16: /* M2L31 : PD01 or PD05 */
                DBG_PRINTF("TK16 MF\n");
				DBG_PRINTF("PINSEL_1 = 0x%x\n", psTkFeat->u32PinSel1);
#if 1
                if (((psTkFeat->u32PinSel1)&0x3) == 0)          //Because M25x is from TK15
                    SYS->GPD_MFP0 = (SYS->GPD_MFP0 & ~(0xFFUL << (1*8))) | (16UL << (1*8)); //TK16 PD01  16
                else
                    SYS->GPD_MFP1 = (SYS->GPD_MFP1 & ~(0xFFUL << (1*8))) | (16UL << (1*8)); //TK16 PD05  16
#else
                if (((psTkFeat->u32PinSel >> (15*2))&0x3) == 0)
                    SYS->GPD_MFP0 = (SYS->GPD_MFP0 & ~(0xFFUL << (1*8))) | (16UL << (1*8)); //TK16 PD01  16
                else
                    SYS->GPD_MFP1 = (SYS->GPD_MFP1 & ~(0xFFUL << (1*8))) | (16UL << (1*8)); //TK16 PD05  16
#endif
#if 0
                /*TC8260 Touch Key Control Board */
                SYS->GPD_MFP1 = (SYS->GPD_MFP1 & ~(0xFFUL << (1*8))) | (16UL << (1*8)); //TK16 PD05  16 (A)
#endif
                break;
            case 17: /* M2L31 : PD00 or PD04 */
                DBG_PRINTF("TK17\n");
				DBG_PRINTF("PINSEL_1 = 0x%x\n", psTkFeat->u32PinSel1);
#if 1
                if (((psTkFeat->u32PinSel1 >> (1*2))&0x3) == 0)
                    SYS->GPD_MFP0 = (SYS->GPD_MFP0 & ~(0xFFUL << (0*8))) | (16UL << (0*8)); //TK17 PD00  16
                else
                    SYS->GPD_MFP1 = (SYS->GPD_MFP1 & ~(0xFFUL << (0*8))) | (16UL << (0*8)); //TK17 PD04  16
#else
                if (((psTkFeat->u32PinSel1)&0x3) == 0)
                    SYS->GPD_MFP0 = (SYS->GPD_MFP0 & ~(0xFFUL << (0*8))) | (16UL << (0*8)); //TK17 PD00  16
                else
                    SYS->GPD_MFP1 = (SYS->GPD_MFP1 & ~(0xFFUL << (0*8))) | (16UL << (0*8)); //TK17 PD04  16
#endif
#if 0
                SYS->GPD_MFP1 = (SYS->GPD_MFP1 & ~(0xFFUL << (0*8))) | (16UL << (0*8)); //TK17 PD04  16   (A)
#endif
                break;


            /* CKO shielding start */
            //SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(0xFFUL << (3*8))) | (06UL << (3*8)); //TK_SE PA03  06
            //SYS->GPB_MFP3 = (SYS->GPB_MFP3 & ~(0xFFUL << (2*8))) | (16UL << (2*8)); //TK_SE PB14  16
            //SYS->GPC_MFP3 = (SYS->GPC_MFP3 & ~(0xFFUL << (1*8))) | (16UL << (1*8)); //TK_SE PC13  16
            //SYS->GPD_MFP3 = (SYS->GPD_MFP3 & ~(0xFFUL << (0*8))) | (16UL << (0*8)); //TK_SE PD12  16
            //SYS->GPD_MFP3 = (SYS->GPD_MFP3 & ~(0xFFUL << (1*8))) | (16UL << (1*8)); //TK_SE PD13  16
            //SYS->GPG_MFP3 = (SYS->GPG_MFP3 & ~(0xFFUL << (3*8))) | (16UL << (3*8)); //TK_SE PG15  16
            case 18:
                DBG_PRINTF("TK18 CKO\n");
                //SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(0xFFUL << (3*8))) | (06UL << (3*8)); //TK_SE PA03  06
                //SYS->GPB_MFP3 = (SYS->GPB_MFP3 & ~(0xFFUL << (2*8))) | (16UL << (2*8)); //TK_SE PB14  16

                //SYS->GPC_MFP3 = (SYS->GPC_MFP3 & ~(0xFFUL << (1*8))) | (16UL << (1*8)); //TK_SE PC13  16  ==> J7
                //SYS->GPD_MFP3 = (SYS->GPD_MFP3 & ~(0xFFUL << (0*8))) | (16UL << (0*8)); //TK_SE PD12  16  ==> J5
                //SYS->GPD_MFP3 = (SYS->GPD_MFP3 & ~(0xFFUL << (1*8))) | (16UL << (1*8)); //TK_SE PD13  16  ==> J6
                SYS->GPG_MFP3 = (SYS->GPG_MFP3 & ~(0xFFUL << (3*8))) | (16UL << (3*8)); //TK_SE PG15  16  ==> J8
                break;
            case 19:
                DBG_PRINTF("TK19 CKO\n");
                SYS->GPB_MFP3 = (SYS->GPB_MFP3 & ~(0xFFUL << (2*8))) | (16UL << (2*8)); //TK_SE PB14  16
                break;
            case 20:
                DBG_PRINTF("TK20 CKO\n");
                SYS->GPC_MFP3 = (SYS->GPC_MFP3 & ~(0xFFUL << (1*8))) | (16UL << (1*8)); //TK_SE PC13  16
                break;
            case 21:
                DBG_PRINTF("TK21 CKO\n");
                SYS->GPD_MFP3 = (SYS->GPD_MFP3 & ~(0xFFUL << (0*8))) | (16UL << (0*8)); //TK_SE PD12  16
                break;
            case 22:
                DBG_PRINTF("TK22 CKO\n");
                SYS->GPD_MFP3 = (SYS->GPD_MFP3 & ~(0xFFUL << (1*8))) | (16UL << (1*8)); //TK_SE PD13  16
                break;
            case 23:
                DBG_PRINTF("TK23 CKO\n");
                SYS->GPG_MFP3 = (SYS->GPG_MFP3 & ~(0xFFUL << (3*8))) | (16UL << (3*8)); //TK_SE PG15  16
                break;	
            default:
                break;
            }
        }
    }
}
