/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use CIR to wake up system from Power-down mode while first 8 bits data matched
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"
#include "queue.h"

/*---------------------------------------------------------------------------------------------------------*/
/* CIR sample configuration                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define UART0_BAUD_RATE                  (115200UL)  /*!< UART0 baud rate used for debug messages. */
#define CIR_TIMING_UNIT_US               (26UL)      /*!< Timing unit used to convert IR pulse width from us to CIR counter units. */
#define CIR_TIMING_DEVIATION             (15UL)      /*!< Allowed timing deviation in CIR counter units. */
#define CIR_DATA_BIT_COUNT               (32UL)      /*!< Number of bits contained in one complete IR data frame. */
#define CIR_WAKEUP_COMPARE_DATA          (0x40UL)    /*!< Data value used by the CIR data-compare wake-up function. */
#define CIR_WAKEUP_COMPARE_BITS          (7UL)       /*!< Number of bits used by the CIR data-compare wake-up function. */
#define CIR_REPEAT_CODE                  (0xFFFFFFFFUL) /*!< Software-defined value used to represent a repeat code. */
#define CIR_WAKEUP_DELAY_US              (1000000UL) /*!< Delay after the system wakes up, in microseconds. */

/* Panasonic-like IR timing used by this sample, unit: microseconds. */
#define CIR_HEADER_LOW_US                (9100UL)
#define CIR_HEADER_HIGH_US               (4450UL)
#define CIR_DATA0_LOW_US                 (576UL)
#define CIR_DATA0_HIGH_US                (557UL)
#define CIR_DATA1_LOW_US                 (576UL)
#define CIR_DATA1_HIGH_US                (1690UL)
#define CIR_SPECIAL_LOW_US               (9065UL)
#define CIR_SPECIAL_HIGH_US              (2282UL)
#define CIR_END_LOW_US                   (542UL)
#define CIR_END_HIGH_US                  (17000UL)

#define CIR_PATTERN_UPPER_BOUND(low, high) \
    ((((low) + (high)) / CIR_TIMING_UNIT_US) + CIR_TIMING_DEVIATION)
#define CIR_PATTERN_LOWER_BOUND(low, high) \
    ((((low) + (high)) / CIR_TIMING_UNIT_US) - CIR_TIMING_DEVIATION)
#define CIR_PATTERN_END_BOUND(low, high) \
    (((low) + (high)) / CIR_TIMING_UNIT_US)

#define IR_INFO_COUNT                    (sizeof(Array) / sizeof(Array[0]))

uint32_t Queue[MAX_QUEUE];


/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Unlock protected registers because CLK_PowerDown() accesses protected clock-control registers. */
    SYS_UnlockReg();

    /* Make sure all debug messages have been transmitted before the system enters Power-down mode. */
    UART_WAIT_TX_EMPTY(UART0);

    /* Enter Power-down mode. The CIR peripheral remains responsible for detecting the wake-up pattern. */
    CLK_PowerDown();

    /* Restore register protection after the system wakes up. */
    SYS_LockReg();
}

/**
 * @brief CIR interrupt handler.
 *
 * The receive-error flag (CIR_STATUS_RERRF_Msk) may be raised when:
 * 1. The received pulse width is outside the upper/lower boundaries of all configured patterns.
 * 2. For positive-edge or negative-edge decoding, the first received pulse does not match the header pattern.
 *
 * A remote-control repeat code may arrive without a normal header pattern. In that case, the hardware can report
 * a receive error even though the captured timing actually represents the expected repeat code. Therefore, this
 * handler checks the latched timer value against the configured special-pattern boundary before treating the event
 * as a real receive error.
 */
void CIR_IRQHandler(void)
{
    uint32_t status = CIR_GetIntFlag(CIR0) & CIR_GetEnabledIntMask(CIR0);

    if(status & CIR_STATUS_RERRF_Msk)
    {
        uint32_t u32Hb, u32Lb;

        /*
         * A repeat code is handled as CIR_SPECIAL_PAT. Because it does not follow the normal header/end sequence,
         * the CIR may first raise a receive-error event. Check the captured pulse width to determine whether this
         * event is actually the expected repeat code.
         */
        CIR_GetPatternBoundary(CIR0, CIR_SPECIAL_PAT, &u32Hb, &u32Lb);
        if((CIR_GetLatchedTimerValue(CIR0) < u32Hb) && (CIR_GetLatchedTimerValue(CIR0) > u32Lb))
            Push(Queue, CIR_REPEAT_CODE);

        CIR_ClearIntFlag(CIR0, CIR_STATUS_RERRF_Msk);
    }
    else if(status & CIR_STATUS_COMPMF_Msk)
    {
        /* Received data matches the configured comparison value. */
        CIR_ClearIntFlag(CIR0, CIR_STATUS_COMPMF_Msk);
    }
    else if(status & CIR_STATUS_PDWKF_Msk)
    {
        /* CIR has generated a Power-down wake-up event. */
        CIR_ClearIntFlag(CIR0, CIR_STATUS_PDWKF_Msk);
    }
    else if(status & CIR_STATUS_EPMF_Msk)
    {
        uint32_t u32Data0, u32Data1;

        /* A complete frame has reached the configured end pattern. */
        CIR_ClearIntFlag(CIR0, CIR_STATUS_EPMF_Msk);

        /* Read the decoded CIR data and store the first 32-bit data field in the software queue. */
        CIR_GetData(CIR0, &u32Data0, &u32Data1);
        Push(Queue, u32Data0);

        /* Clear the hardware data fields and bit counter before receiving the next frame. */
        CIR_ClearDataFieldBitCount(CIR0);
    }
}


void SYS_Init(void)
{
    /* Unlock protected registers. */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode. */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable the internal high-speed RC oscillator (HIRC). */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait until HIRC is stable before switching/configuring clocks. */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set the system core clock to 96 MHz. */
    CLK_SetCoreClock(FREQ_96MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 during initial peripheral clock configuration. */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART0 and CIR0 module clocks. */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(CIR0_MODULE);

    /* Use HIRC as the UART0 clock source with divider = 1. */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Use LIRC as the CIR0 clock source. */
    CLK_SetModuleClock(CIR0_MODULE, CLK_CLKSEL2_CIR0SEL_LIRC, (uint32_t)NULL);

    /* Refresh SystemCoreClock after all clock settings are complete. */
    SystemCoreClockUpdate();

    /* Configure PB.12/PB.13 as UART0 RXD/TXD. */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Configure PC.14 as CIR0 RXD input. */
    SYS->GPC_MFPH = (SYS->GPC_MFPH & ~SYS_GPC_MFPH_PC14MFP_Msk) | SYS_GPC_MFPH_PC14MFP_CIR0_RXD;

    /* Keep the original sample setting for APB1 divider. */
    CLK->PCLKDIV &= ~CLK_PCLKDIV_APB1DIV_Msk;

    /* Lock protected registers. */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Configure UART0 for sample debug output. */
    UART_Open(UART0, UART0_BAUD_RATE);
}


/**
 * @brief Initialize CIR receiver timing and wake-up conditions.
 *
 * The AI-26C generic remote controller can be configured for Panasonic mode by holding "Setup" for approximately
 * three seconds and then entering "0", "0", "1".
 *
 * CIR timing is derived from LIRC. The raw remote-control timings are specified in microseconds and converted into
 * CIR counter units by CIR_TIMING_UNIT_US. CIR_TIMING_DEVIATION provides tolerance for normal header/data/special
 * pattern matching.
 *
 * Typical timing used by this sample (unit: us):
 * Header  : 9100 + 4450
 * Data 0  :  576 +  557
 * Data 1  :  576 + 1690
 * Special : 9065 + 2282
 * End     :  542 + 17000
 */
void CIR_Init(void)
{
    /* CIR counter runs with the selected clock source and no additional prescaling. */
    CIR_SetClockPrescaler(CIR0, CIR_PRESCALER_1);

    /* Configure accepted timing windows for each received IR pattern. */
    CIR_SetPatternBoundary(CIR0, CIR_HEADER_PAT,
                           CIR_PATTERN_UPPER_BOUND(CIR_HEADER_LOW_US, CIR_HEADER_HIGH_US),
                           CIR_PATTERN_LOWER_BOUND(CIR_HEADER_LOW_US, CIR_HEADER_HIGH_US));
    CIR_SetPatternBoundary(CIR0, CIR_DATA1_PAT,
                           CIR_PATTERN_UPPER_BOUND(CIR_DATA1_LOW_US, CIR_DATA1_HIGH_US),
                           CIR_PATTERN_LOWER_BOUND(CIR_DATA1_LOW_US, CIR_DATA1_HIGH_US));
    CIR_SetPatternBoundary(CIR0, CIR_DATA0_PAT,
                           CIR_PATTERN_UPPER_BOUND(CIR_DATA0_LOW_US, CIR_DATA0_HIGH_US),
                           CIR_PATTERN_LOWER_BOUND(CIR_DATA0_LOW_US, CIR_DATA0_HIGH_US));
    CIR_SetPatternBoundary(CIR0, CIR_SPECIAL_PAT,
                           CIR_PATTERN_UPPER_BOUND(CIR_SPECIAL_LOW_US, CIR_SPECIAL_HIGH_US),
                           CIR_PATTERN_LOWER_BOUND(CIR_SPECIAL_LOW_US, CIR_SPECIAL_HIGH_US));
    CIR_SetPatternBoundary(CIR0, CIR_END_PAT, 0UL,
                           CIR_PATTERN_END_BOUND(CIR_END_LOW_US, CIR_END_HIGH_US));

    /*
     * Enable CIR data comparison for Power-down wake-up and configure the expected receive-frame length.
     * The original sample uses value 0x40 and a compare length of 7 bits.
     */
    CIR_EnableDataCmpWakeup(CIR0, CIR_WAKEUP_COMPARE_DATA, CIR_WAKEUP_COMPARE_BITS);
    CIR_EnableRecvBitCountMatch(CIR0, CIR_DATA_BIT_COUNT);

    /* Enable receive-error, end-pattern, compare-match, and Power-down wake-up interrupts. */
    CIR_EnableInt(CIR0, CIR_INTCTL_PERRIEN_Msk | CIR_INTCTL_EPMIEN_Msk |
                        CIR_INTCTL_CMPMIEN_Msk | CIR_INTCTL_PDWKIEN_Msk);

    /* Decode the incoming waveform on positive edges and invert the input polarity. */
    CIR_SetInputType(CIR0, CIR_POSITIVE_EDGE, CIR_INVERSE);
    NVIC_EnableIRQ(CIR_IRQn);

    /*
     * Error bypass must be enabled for Power-down reception. A receive-error flag normally stops CIR data
     * conversion. If this happens while the CPU is in Power-down mode, the desired remote-control pattern might
     * never reach the wake-up comparison logic. Error bypass allows CIR to recover when a valid pattern appears.
     */
    CIR_EnableErrorBypass(CIR0);

    /* Start CIR receiver operation after all timing and interrupt settings have been configured. */
    CIR_Open(CIR0);
}

typedef struct
{
    char* KeyString;       /*!< Printable name of the remote-control key. */
    uint32_t IrCode0;      /*!< Decoded 32-bit IR code associated with the key. */
    uint32_t u32Len;       /*!< Number of valid bits in the IR code. */
} IR_INFO;

/* Remote-control key lookup table used only for displaying the decoded key name. */
const IR_INFO Array[] =
{
    "POWER",        0xED12BF40, CIR_DATA_BIT_COUNT,
    "IMAGE",        0xEB14BF40, CIR_DATA_BIT_COUNT,
    "0",            0xFF00BF40, CIR_DATA_BIT_COUNT,
    "1",            0xFE01BF40, CIR_DATA_BIT_COUNT,
    "2",            0xFD02BF40, CIR_DATA_BIT_COUNT,
    "3",            0xFC03BF40, CIR_DATA_BIT_COUNT,
    "4",            0xFB04BF40, CIR_DATA_BIT_COUNT,
    "5",            0xFA05BF40, CIR_DATA_BIT_COUNT,
    "6",            0xF906BF40, CIR_DATA_BIT_COUNT,
    "7",            0xF807BF40, CIR_DATA_BIT_COUNT,
    "8",            0xF708BF40, CIR_DATA_BIT_COUNT,
    "9",            0xF609BF40, CIR_DATA_BIT_COUNT,
    "TV",           0xEC13BF40, CIR_DATA_BIT_COUNT,
    "DISPLAY",      0xE916BF40, CIR_DATA_BIT_COUNT,
    "CATV",         0xAE51BF40, CIR_DATA_BIT_COUNT,
    "SLEEP",        0xE11EBF40, CIR_DATA_BIT_COUNT,
    "RETURN",       0xF30CBF40, CIR_DATA_BIT_COUNT,
    "INPUT",        0xF40BBF40, CIR_DATA_BIT_COUNT,
    "SILENT",       0xEF10BF40, CIR_DATA_BIT_COUNT,
    "VOL_UP",       0xE51ABF40, CIR_DATA_BIT_COUNT,
    "VOL_DN",       0xE11EBF40, CIR_DATA_BIT_COUNT,
    "VOL_SILENT",   0xE718BF40, CIR_DATA_BIT_COUNT,
    "CHAN_UP",      0xE41BBF40, CIR_DATA_BIT_COUNT,
    "CHAN_DN",      0xE01FBF40, CIR_DATA_BIT_COUNT,
    "REPEAT",       CIR_REPEAT_CODE, CIR_DATA_BIT_COUNT,
};


void CIR_Parsing(void)
{
    uint32_t u32IrCode;
    uint32_t i;

    do
    {
        /* Process all CIR events collected by the interrupt handler before entering Power-down mode again. */
        while(isEmpty() != 1)
        {
            u32IrCode = Pop(Queue);

            if(u32IrCode != CIR_REPEAT_CODE)
            {
                /* Search the lookup table for a readable key name corresponding to the received IR code. */
                for(i = 0; i < IR_INFO_COUNT; i = i + 1)
                {
                    if(u32IrCode == Array[i].IrCode0)
                    {
                        printf("%s Pressed\n", Array[i].KeyString);
                        break;
                    }
                }

                if(i == IR_INFO_COUNT)
                    printf("Unknown IR Code 0x%x\n", u32IrCode);
            }
            else
            {
                printf("REPEAT\n");
            }
        }

        /* No pending CIR data remains. Enter Power-down mode and wait for the next CIR wake-up event. */
        printf("Enter Power Down\n");
        PowerDownFunction();
        printf("Wake up\n\n");

        /* Keep a short observation interval after wake-up before returning to the receive loop. */
        CLK_SysTickDelay(CIR_WAKEUP_DELAY_US);
    }
    while(1);
}

int main()
{
    SYS_Init();                        /* Initialize system clocks and multi-function pins. */
    UART0_Init();                      /* Initialize UART0 for debug output. */

    printf("\n");
    printf("+------------------------------------------+\n");
    printf("|    M471 CIR Sample Code                  |\n");
    printf("+------------------------------------------+\n");

    CIR_Init();                        /* Configure CIR timing, interrupts, and Power-down wake-up conditions. */
    CIR_Parsing();                     /* Decode received CIR frames and repeatedly enter Power-down mode. */

    printf("\nCIR Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
