/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to use APIs of CIR to convert the output signal of an Infrared Receiver
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"
#include "queue.h"

/* CIR timing configuration for the Panasonic-compatible remote controller. */
#define CIR_TIMING_UNIT_US          (26U)
#define CIR_BOUNDARY_TOLERANCE      (15U)
#define CIR_REPEAT_CODE             (0xFFFFFFFFUL)
#define CIR_EXPECTED_BIT_COUNT      (32U)
#define CIR_DATA_CMP_VALUE          (0x40U)
#define CIR_DATA_CMP_BIT_COUNT      (7U)

#define CIR_HEADER_HIGH_US          (9100U)
#define CIR_HEADER_LOW_US           (4450U)
#define CIR_DATA0_HIGH_US           (576U)
#define CIR_DATA0_LOW_US            (557U)
#define CIR_DATA1_HIGH_US           (576U)
#define CIR_DATA1_LOW_US            (1690U)
#define CIR_SPECIAL_HIGH_US         (9065U)
#define CIR_SPECIAL_LOW_US          (2282U)
#define CIR_STOP_HIGH_US            (542U)
#define CIR_STOP_LOW_US             (17000U)

#define CIR_TO_TICKS(high_us, low_us) \
    (((high_us) + (low_us)) / CIR_TIMING_UNIT_US)
#define CIR_UPPER_BOUND(high_us, low_us) \
    (CIR_TO_TICKS((high_us), (low_us)) + CIR_BOUNDARY_TOLERANCE)
#define CIR_LOWER_BOUND(high_us, low_us) \
    (CIR_TO_TICKS((high_us), (low_us)) - CIR_BOUNDARY_TOLERANCE)

/* Queue shared by the CIR interrupt handler and the foreground parser. */
uint32_t Queue[MAX_QUEUE];

/**
  * Receive error int:
  *     The CIR_STATUS_RERRF_Msk will be raised if following reasons.
  *     1. The coming pulse width does not meet the upper and lower bound of all patterns.
  *     2. The first pulse width of coming pattern is not header pattern if pattern type is CIR_POSITIVE_EDGE or CIR_NEGATIVE_EDGE.
  *
  * Due to the repeat code appears alone and is not header patteren. It will cause the CIR_STATUS_RERRF_Msk flag to be raised.
  * Programmer needs to judge if the error event caused by repeat code.
  */
void CIR_IRQHandler(void)
{
    uint32_t status = CIR_GetIntFlag(CIR0) & CIR_GetEnabledIntMask(CIR0);

    if(status & CIR_STATUS_RERRF_Msk)
    {
        uint32_t u32Hb, u32Lb;
        /*
         * Due to the special pattern (repeat code) out of header pattern and end pattern.
         * It will generate a error event. As raised a error event,
         * programmer needs to judge if the event caused by special pattern.
         */
        CIR_GetPatternBoundary(CIR0, CIR_SPECIAL_PAT, &u32Hb, &u32Lb);
        if((CIR_GetLatchedTimerValue(CIR0)< u32Hb)   &&   (CIR_GetLatchedTimerValue(CIR0) > u32Lb))
            Push(Queue, CIR_REPEAT_CODE);                           /* Represent a repeat frame with a dedicated software code. */
        CIR_ClearIntFlag(CIR0, CIR_STATUS_RERRF_Msk);
    }
    else if(status & CIR_STATUS_EPMF_Msk)
    {
        uint32_t u32Data0, u32Data1;

        /* End Patter match int */
        CIR_ClearIntFlag(CIR0, CIR_STATUS_EPMF_Msk);

        CIR_GetData(CIR0, &u32Data0, &u32Data1);
        Push(Queue, u32Data0);

        /* Clear data fields and bit count for next receiving */
        CIR_ClearDataFieldBitCount(CIR0);
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as 96MHz from PLL */
    CLK_SetCoreClock(FREQ_96MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable UART module clock */
    CLK_EnableModuleClock(CIR0_MODULE);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Select CIR module clock source as LIRC */
    CLK_SetModuleClock(CIR0_MODULE, CLK_CLKSEL2_CIR0SEL_LIRC, (uint32_t)NULL);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set GPC14 multi-function pins for CIR */
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~SYS_GPC_MFPH_PC14MFP_Msk) | SYS_GPC_MFPH_PC14MFP_CIR0_RXD;

    CLK->PCLKDIV &= ~CLK_PCLKDIV_APB1DIV_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/**
 *    Use AI-26C generic remote controller by setting TV system to Panasonic:
 *    Pressed "Setup" up to 3 seconds, then pressed "0", "0", "1" to assign the remote controller to operate on Panasonic mode
 *
 *    CIR Clock Source From __LIRC. T = 26.04us
 *
 *    unit: us
 *    Header    9100+4450
 *    Spec      9065+2282
 *    D0        576+557
 *    D1        576+1690
 *    Stop      542+17000
 */
void CIR_Init(void)
{
    CIR_SetClockPrescaler(CIR0, CIR_PRESCALER_1);

    /* Convert protocol pulse widths from microseconds to CIR timer ticks.
     * A tolerance is applied to normal patterns to accommodate clock and
     * receiver timing variation. */
    CIR_SetPatternBoundary(CIR0, CIR_HEADER_PAT,
                           CIR_UPPER_BOUND(CIR_HEADER_HIGH_US, CIR_HEADER_LOW_US),
                           CIR_LOWER_BOUND(CIR_HEADER_HIGH_US, CIR_HEADER_LOW_US));
    CIR_SetPatternBoundary(CIR0, CIR_DATA1_PAT,
                           CIR_UPPER_BOUND(CIR_DATA1_HIGH_US, CIR_DATA1_LOW_US),
                           CIR_LOWER_BOUND(CIR_DATA1_HIGH_US, CIR_DATA1_LOW_US));
    CIR_SetPatternBoundary(CIR0, CIR_DATA0_PAT,
                           CIR_UPPER_BOUND(CIR_DATA0_HIGH_US, CIR_DATA0_LOW_US),
                           CIR_LOWER_BOUND(CIR_DATA0_HIGH_US, CIR_DATA0_LOW_US));
    CIR_SetPatternBoundary(CIR0, CIR_SPECIAL_PAT,
                           CIR_UPPER_BOUND(CIR_SPECIAL_HIGH_US, CIR_SPECIAL_LOW_US),
                           CIR_LOWER_BOUND(CIR_SPECIAL_HIGH_US, CIR_SPECIAL_LOW_US));
    CIR_SetPatternBoundary(CIR0, CIR_END_PAT, 0U,
                           CIR_TO_TICKS(CIR_STOP_HIGH_US, CIR_STOP_LOW_US));

    /* Configure the optional data comparator and require a complete 32-bit frame. */
    CIR_EnableDataCmpWakeup(CIR0, CIR_DATA_CMP_VALUE, CIR_DATA_CMP_BIT_COUNT);
    CIR_EnableRecvBitCountMatch(CIR0, CIR_EXPECTED_BIT_COUNT);

    CIR_EnableInt(CIR0, CIR_INTCTL_PERRIEN_Msk|CIR_INTCTL_EPMIEN_Msk);

    CIR_SetInputType(CIR0, CIR_POSITIVE_EDGE, CIR_INVERSE);
    NVIC_EnableIRQ(CIR_IRQn);
    CIR_Open(CIR0);
}

typedef struct
{
    char* KeyString;
    uint32_t IrCode0;
    uint32_t u32Len;
} IR_INFO;

const IR_INFO Array[] =
{
    "POWER",        0xED12BF40, 32,
    "IMAGE",        0xEB14BF40, 32,
    "0",            0xFF00BF40, 32,
    "1",            0xFE01BF40, 32,
    "2",            0xFD02BF40, 32,
    "3",            0xFC03BF40, 32,
    "4",            0xFB04BF40, 32,
    "5",            0xFA05BF40, 32,
    "6",            0xF906BF40, 32,
    "7",            0xF807BF40, 32,
    "8",            0xF708BF40, 32,
    "9",            0xF609BF40, 32,
    "TV",           0xEC13BF40, 32,
    "DISPLAY",      0xE916BF40, 32,
    "CATV",         0xAE51BF40, 32,
    "SLEEP",        0xE11EBF40, 32,
    "RETURN",       0xF30CBF40, 32,
    "INPUT",        0xF40BBF40, 32,
    "SILENT",       0xEF10BF40, 32,
    "VOL_UP",       0xE51ABF40, 32,
    "VOL_DN",       0xE11EBF40, 32,
    "VOL_SILENT",   0xE718BF40, 32,
    "CHAN_UP",      0xE41BBF40, 32,
    "CHAN_DN",      0xE01FBF40, 32,
    "REPEAT",       0xFFFFFFFF, 32,
};


/**
 * @brief Decode received CIR frames and print the corresponding key name.
 *
 * The interrupt handler stores received frames in Queue[]. This routine runs
 * in the foreground, removes entries from the queue, and searches Array[] for
 * a matching remote-control command. A repeat frame is handled separately.
 */
void CIR_Parsing(void)
{
    uint32_t u32IrCode;
    uint32_t i;
    do
    {
        if(isEmpty() != 1)
        {
            u32IrCode = Pop(Queue);
            if(u32IrCode != CIR_REPEAT_CODE)
            {
                for(i=0; i<(sizeof(Array) / sizeof(Array[0])); i=i+1)
                {
                    if(u32IrCode == Array[i].IrCode0)
                        printf("%s Pressed\n", Array[i].KeyString);
                }
            }
            else
            {
                printf("REPEAT\n");
            }
        }
    }
    while(1);
}

int main()
{

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    printf("\n");
    printf("+------------------------------------------+\n");
    printf("|    M471 CIR Sample Code                  |\n");
    printf("+------------------------------------------+\n");

    CIR_Init();                        /* Base on the remote control to set the timing */

    CIR_Parsing();

    printf("\nCIR Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
