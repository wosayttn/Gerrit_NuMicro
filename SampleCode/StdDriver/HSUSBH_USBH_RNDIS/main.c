/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    This sample demonstrates MCU USB Host connectivity with Android USB
 *           tethering, including RNDIS device enumeration and initialization,
 *           dynamic IPv4 address assignment through the lwIP DHCP client,
 *           and access to the MCU static web dashboard using Chrome on the
 *           Android phone.
 * @note     This sample is currently not applicable to iPhone.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

#include "usbh_lib.h"
#include "usbh_rndis.h"
#include "rndis_dashboard_netif.h"

#ifdef DEBUG_ENABLE_SEMIHOST
    #error This sample cannot execute with semihost enabled
#endif

#define USE_USB_APLL1_CLOCK         1

static volatile uint32_t s_u32TickCnt;
static RNDIS_SESSION_T *s_psObservedSession;
static uint32_t s_u32ObservedGeneration;
static RNDIS_STATE_E s_eObservedState = RNDIS_STATE_DETACHED;

void SysTick_Handler(void);
void enable_sys_tick(int ticks_per_second);
void SYS_Init(void);
uint32_t get_ticks(void);

static void rndis_log_state(RNDIS_SESSION_T *session, RNDIS_STATE_E state)
{
    RNDIS_STATS_T stats;
    RNDIS_FAILURE_E failure = usbh_rndis_get_failure(session);

    if (usbh_rndis_get_stats(session, &stats) != RNDIS_OK)
    {
        printf("[RNDIS] state=%u failure=%s(%u), stats unavailable\n",
               (uint32_t)state, usbh_rndis_failure_string(failure), (uint32_t)failure);
        return;
    }

    printf("[RNDIS] state=%u failure=%s(%u) ctrl_err=%lu get=%lu get_err=%lu "
           "notif=%lu notif_err=%lu notif_bad=%lu xfer_err=%lu stale=%lu\n",
           (uint32_t)state, usbh_rndis_failure_string(failure), (uint32_t)failure,
           (unsigned long)stats.control_errors, (unsigned long)stats.get_polls,
           (unsigned long)stats.get_transport_errors, (unsigned long)stats.notification_events,
           (unsigned long)stats.notification_errors, (unsigned long)stats.notification_invalid,
           (unsigned long)stats.transfer_errors, (unsigned long)stats.stale_completions);
}

NVT_ITCM void SysTick_Handler(void)
{
    s_u32TickCnt++;
}

void enable_sys_tick(int ticks_per_second)
{
    s_u32TickCnt = 0;

    if (SysTick_Config(SystemCoreClock / (uint32_t)ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");

        while (1);
    }
}

uint32_t get_ticks(void)
{
    return s_u32TickCnt;
}

/*
 *  This function is necessary for USB Host library.
 */
void delay_us(int usec)
{
    /*
     *  Configure Timer0, clock source from HIRC_12M. Prescale 12
     */
    /* TIMER0 clock from HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_HIRC, 0);
    CLK_EnableModuleClock(TMR0_MODULE);

    TIMER_SET_PRESCALE_VALUE(TIMER0, (12 - 1));
    /* stop timer0 */
    TIMER_Stop(TIMER0);
    /* write 1 to clear for safety */
    TIMER_ClearIntFlag(TIMER0);
    TIMER_ClearWakeupFlag(TIMER0);
    /* set timer cmp value */
    TIMER_SET_CMP_VALUE(TIMER0, usec);
    /* Timer0 config to oneshot mode */
    TIMER_SET_OPMODE(TIMER0, TIMER_ONESHOT_MODE);
    /* start timer0*/
    TIMER_Start(TIMER0);

    while (TIMER_GetIntFlag(TIMER0) == 0);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);
    CLK_EnableXtalRC(CLK_SRCCTL_HIRC48MEN_Msk);

    /* Wait for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRC48MSTB_Msk);

    /* Switch SCLK clock source to PLL0 and Enable PLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

#if (USE_USB_APLL1_CLOCK)
    /* Enable APLL1 96MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HXT, 96000000, CLK_APLL1_SELECT);
#endif

    /* Enable GPIOA module clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(GPIOF_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /* Enable HSOTG module clock */
    CLK_EnableModuleClock(HSOTG0_MODULE);
    /* Select HSOTG PHY Reference clock frequency which is from HXT*/
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_24_0M);

#if (USE_USB_APLL1_CLOCK)
    /* USB Host desired input clock is 48 MHz. Set as APLL1 divided by 2 (96/2 = 48) */
    CLK_SetModuleClock(USBH0_MODULE, CLK_USBSEL_USBSEL_APLL1_DIV2, CLK_USBDIV_USBDIV(1));
#else
    /* USB Host desired input clock is 48 MHz. Set as HIRC48M divided by 1 (48/1 = 48) */
    CLK_SetModuleClock(USBH0_MODULE, CLK_USBSEL_USBSEL_HIRC48M, CLK_USBDIV_USBDIV(1));
#endif

    /* Enable USBH module clock */
    CLK_EnableModuleClock(USBH0_MODULE);
    CLK_EnableModuleClock(USBD0_MODULE);
    CLK_EnableModuleClock(OTG0_MODULE);
    /* Enable HSUSBH module clock */
    CLK_EnableModuleClock(HSUSBH0_MODULE);

    /* Set OTG as USB Host role */
    SYS->USBPHY = (0x1ul << (SYS_USBPHY_HSOTGPHYEN_Pos)) | (0x1ul << (SYS_USBPHY_HSUSBROLE_Pos)) | (0x1ul << (SYS_USBPHY_OTGPHYEN_Pos)) | (0x1 << SYS_USBPHY_USBROLE_Pos);
    delay_us(20);
    SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;
    //delay_us(20);

    /* Set Debug Uart CLK*/
    SetDebugUartCLK();
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* USB_VBUS_EN (USB 1.1 VBUS power enable pin) multi-function pin - PB.15     */
    SET_USB_VBUS_EN_PB15();

    /* USB_VBUS_ST (USB 1.1 over-current detect pin) multi-function pin - PB.14   */
    SET_USB_VBUS_ST_PB14();

    /* HSUSB_VBUS_EN (USB 2.0 VBUS power enable pin) multi-function pin - PJ.13   */
    SET_HSUSB_VBUS_EN_PJ13();

    /* HSUSB_VBUS_ST (USB 2.0 over-current detect pin) multi-function pin - PJ.12 */
    SET_HSUSB_VBUS_ST_PJ12();

    /* USB 1.1 port multi-function pin VBUS, D+, D-, and ID pins */
    SET_USB_VBUS_PA12();
    SET_USB_D_MINUS_PA13();
    SET_USB_D_PLUS_PA14();
    SET_USB_OTG_ID_PA15();

    /* Lock protected registers */
    SYS_LockReg();
}
/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int32_t main(void)
{
    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    InitDebugUart();                   /* Init DeubgUART for printf */

    enable_sys_tick(100);

    printf("\n");
    printf("+----------------------------------------------------------+\n");
    printf("| RNDIS Dashboard HTTP service on port 80                  |\n");
    printf("| DHCP-bound gate; one listener and one active client.     |\n");
    printf("| Bounded fixed pools; closes on timeout or failures.      |\n");
    printf("+----------------------------------------------------------+\n");

    usbh_rndis_init();
    usbh_core_init();

    if (usbh_rndis_core_ready() != RNDIS_OK)
    {
        printf("[RNDIS] driver registration failed\n");

        while (1)
        {
            ;
        }
    }

    usbh_memory_used();

    while (1)
    {
        RNDIS_SESSION_T *current_session;

        uint32_t current_generation = 0U;
        int32_t generation_status = RNDIS_ERR_NOT_READY;

        (void)usbh_pooling_hubs();           /* USB Host port detect polling and management */

        current_session = usbh_rndis_get_device_list();
        RndisDashboardNetifBeginIteration(current_session);

        usbh_rndis_poll();                    /* Deferred control, packet validation and resubmit */
        current_session = usbh_rndis_get_device_list();

        RndisDashboardNetifEndIteration(current_session);

        if (current_session != NULL)
        {
            generation_status = usbh_rndis_get_session_generation(current_session, &current_generation);
        }

        if ((s_psObservedSession != current_session) ||
                ((current_session != NULL) && ((generation_status != RNDIS_OK) ||
                                               (s_u32ObservedGeneration != current_generation))))
        {
            if (s_psObservedSession != NULL)
            {
                printf("[RNDIS] session detached\n");
                usbh_memory_used();
            }

            s_psObservedSession = current_session;
            s_u32ObservedGeneration = (generation_status == RNDIS_OK) ? current_generation : 0U;
            s_eObservedState = RNDIS_STATE_DETACHED;

            if (s_psObservedSession != NULL)
            {
                printf("[RNDIS] session attached\n");
                usbh_memory_used();
            }
        }

        if ((s_psObservedSession != NULL) &&
                (s_eObservedState != usbh_rndis_get_state(s_psObservedSession)))
        {
            s_eObservedState = usbh_rndis_get_state(s_psObservedSession);
            rndis_log_state(s_psObservedSession, s_eObservedState);
        }
    }
}
