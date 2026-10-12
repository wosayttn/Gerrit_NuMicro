#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Driver_SAI.h"
#include "cmsis_os2.h"                  // ARM::CMSIS:RTOS2:Keil RTX5
#include "NuMicro.h"
#include "DV_Framework.h"

//------------------------------------------------------------------------------
// <o> Driver_SAI#
#define DRV_SAI                     0

#define _ARM_Driver_SAI_(n)         Driver_SAI##n
#define  ARM_Driver_SAI_(n)         _ARM_Driver_SAI_(n)

//------------------------------------------------------------------------------
extern ARM_DRIVER_SAI ARM_Driver_SAI_(DRV_SAI);

//------------------------------------------------------------------------------
static ARM_DRIVER_SAI *drv = &ARM_Driver_SAI_(DRV_SAI);
static volatile uint32_t sai_event;
static osEventFlagsId_t event_flags;
static char msg_buf[256];

//------------------------------------------------------------------------------
static void SAI_DrvEvent(uint32_t event)
{
    sai_event |= event;
    osEventFlagsSet(event_flags, event);
}

void SAI_GetVersion(void)
{
    ARM_DRIVER_VERSION ver = drv->GetVersion();
    TEST_ASSERT((ver.api >= ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)) && (ver.drv >= ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)));
    snprintf(msg_buf, sizeof(msg_buf), "[INFO] API v%d.%d, Driver v%d.%d", ver.api >> 8, ver.api & 0xFF, ver.drv >> 8, ver.drv & 0xFF);
    TEST_MESSAGE(msg_buf);
}

void SAI_GetCapabilities(void)
{
    ARM_SAI_CAPABILITIES cap = drv->GetCapabilities();
    snprintf(msg_buf, sizeof(msg_buf), "[INFO] Async=%d, Sync=%d, Protocol I2S=%d", cap.asynchronous, cap.synchronous, cap.protocol_i2s);
    TEST_MESSAGE(msg_buf);
}

void SAI_Init_Uninit(void)
{
    TEST_ASSERT(drv->Initialize(SAI_DrvEvent) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->PowerControl(ARM_POWER_FULL) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->PowerControl(ARM_POWER_OFF) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->Uninitialize() == ARM_DRIVER_OK);
}

void SAI_Transmit_Test(void)
{
    uint32_t tx_data[8] = {0x12345678, 0xDEADBEEF, 0xAAAA5555, 0x55AAAA55, 0xFFFFFFFF, 0x0, 0x13579BDF, 0x2468ACE0};
    TEST_ASSERT(drv->Initialize(SAI_DrvEvent) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->PowerControl(ARM_POWER_FULL) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_PROTOCOL_I2S | ARM_SAI_DATA_SIZE(16), 0, 16000) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->Send(tx_data, 8) == ARM_DRIVER_OK);
    osEventFlagsWait(event_flags, ARM_SAI_EVENT_SEND_COMPLETE, osFlagsWaitAny, 200);
    TEST_ASSERT(drv->PowerControl(ARM_POWER_OFF) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->Uninitialize() == ARM_DRIVER_OK);
}

void SAI_Receive_Test(void)
{
    uint32_t rx_data[8];
    TEST_ASSERT(drv->Initialize(SAI_DrvEvent) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->PowerControl(ARM_POWER_FULL) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->Control(ARM_SAI_CONFIGURE_RX | ARM_SAI_PROTOCOL_I2S | ARM_SAI_DATA_SIZE(16), 0, 16000) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->Receive(rx_data, 8) == ARM_DRIVER_OK);
    osEventFlagsWait(event_flags, ARM_SAI_EVENT_RECEIVE_COMPLETE, osFlagsWaitAny, 200);
    TEST_ASSERT(drv->PowerControl(ARM_POWER_OFF) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->Uninitialize() == ARM_DRIVER_OK);
}

void SAI_DataSize_Test(void)
{
    uint8_t  tx_8bit[8] = {0x12, 0x34, 0x56, 0x78, 0xAB, 0xCD, 0xEF, 0x00};
    uint16_t tx_16bit[8] = {0x1234, 0x5678, 0xABCD, 0xEF00, 0x1111, 0x2222, 0x3333, 0x4444};
    uint32_t tx_32bit[8] = {0x12345678, 0xDEADBEEF, 0xAAAA5555, 0x55AAAA55, 0xFFFFFFFF, 0x0, 0x13579BDF, 0x2468ACE0};

    TEST_MESSAGE("[DATA SIZE] 8-bit Data Test");
    TEST_ASSERT(drv->Initialize(SAI_DrvEvent) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->PowerControl(ARM_POWER_FULL) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_PROTOCOL_I2S | ARM_SAI_DATA_SIZE(8), 0, 16000) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->Send(tx_8bit, 8) == ARM_DRIVER_OK);
    osEventFlagsWait(event_flags, ARM_SAI_EVENT_SEND_COMPLETE, osFlagsWaitAny, 200);
    drv->Uninitialize();

    TEST_MESSAGE("[DATA SIZE] 16-bit Data Test");
    drv->Initialize(SAI_DrvEvent);
    drv->PowerControl(ARM_POWER_FULL);
    drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_PROTOCOL_I2S | ARM_SAI_DATA_SIZE(16), 0, 16000);
    drv->Send(tx_16bit, 8);
    osEventFlagsWait(event_flags, ARM_SAI_EVENT_SEND_COMPLETE, osFlagsWaitAny, 200);
    drv->Uninitialize();

    TEST_MESSAGE("[DATA SIZE] 32-bit Data Test");
    drv->Initialize(SAI_DrvEvent);
    drv->PowerControl(ARM_POWER_FULL);
    drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_PROTOCOL_I2S | ARM_SAI_DATA_SIZE(32), 0, 16000);
    drv->Send(tx_32bit, 8);
    osEventFlagsWait(event_flags, ARM_SAI_EVENT_SEND_COMPLETE, osFlagsWaitAny, 200);
    drv->Uninitialize();
}

void SAI_TDM_Test(void)
{
    uint32_t tdm_data[16] = {0};
    TEST_MESSAGE("[TDM TEST] 4-Slot TDM Mode Test");
    TEST_ASSERT(drv->Initialize(SAI_DrvEvent) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->PowerControl(ARM_POWER_FULL) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_PROTOCOL_I2S | ARM_SAI_DATA_SIZE(16), ARM_SAI_SLOT_COUNT(4), 16000) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->Send(tdm_data, 16) == ARM_DRIVER_OK);
    osEventFlagsWait(event_flags, ARM_SAI_EVENT_SEND_COMPLETE, osFlagsWaitAny, 200);
    drv->PowerControl(ARM_POWER_OFF);
    drv->Uninitialize();
}

void SAI_Underflow_Overflow_Test(void)
{
    TEST_MESSAGE("[UNDERFLOW/OVERFLOW TEST] Testing TX underflow / RX overflow handling");
    TEST_ASSERT(drv->Initialize(SAI_DrvEvent) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->PowerControl(ARM_POWER_FULL) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_CONFIGURE_RX | ARM_SAI_PROTOCOL_I2S | ARM_SAI_DATA_SIZE(16), 0, 16000) == ARM_DRIVER_OK);

    // Trigger underflow by calling abort without sending
    TEST_ASSERT(drv->Control(ARM_SAI_ABORT_SEND, 0, 0) == ARM_DRIVER_OK);
    osEventFlagsWait(event_flags, ARM_SAI_EVENT_TX_UNDERFLOW, osFlagsWaitAny, 200);

    // Trigger overflow by calling abort receive
    TEST_ASSERT(drv->Control(ARM_SAI_ABORT_RECEIVE, 0, 0) == ARM_DRIVER_OK);
    osEventFlagsWait(event_flags, ARM_SAI_EVENT_RX_OVERFLOW, osFlagsWaitAny, 200);

    drv->PowerControl(ARM_POWER_OFF);
    drv->Uninitialize();
}

void SAI_Protocol_Switch_Test(void)
{
    TEST_MESSAGE("[PROTOCOL SWITCH] Testing protocol change I2S ? MSB ? LSB");
    uint32_t dummy_data[4] = {0x11111111, 0x22222222, 0x33333333, 0x44444444};

    TEST_ASSERT(drv->Initialize(SAI_DrvEvent) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->PowerControl(ARM_POWER_FULL) == ARM_DRIVER_OK);

    TEST_ASSERT(drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_PROTOCOL_I2S | ARM_SAI_DATA_SIZE(16), 0, 16000) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->Send(dummy_data, 4) == ARM_DRIVER_OK);
    osEventFlagsWait(event_flags, ARM_SAI_EVENT_SEND_COMPLETE, osFlagsWaitAny, 200);

    TEST_ASSERT(drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_PROTOCOL_MSB_JUSTIFIED | ARM_SAI_DATA_SIZE(16), 0, 16000) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->Send(dummy_data, 4) == ARM_DRIVER_OK);
    osEventFlagsWait(event_flags, ARM_SAI_EVENT_SEND_COMPLETE, osFlagsWaitAny, 200);

    TEST_ASSERT(drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_PROTOCOL_LSB_JUSTIFIED | ARM_SAI_DATA_SIZE(16), 0, 16000) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->Send(dummy_data, 4) == ARM_DRIVER_OK);
    osEventFlagsWait(event_flags, ARM_SAI_EVENT_SEND_COMPLETE, osFlagsWaitAny, 200);

    drv->PowerControl(ARM_POWER_OFF);
    drv->Uninitialize();
}

void SAI_MCLK_Test(void)
{
    TEST_MESSAGE("[MCLK TEST] Testing MCLK output enable/disable");
    TEST_ASSERT(drv->Initialize(SAI_DrvEvent) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->PowerControl(ARM_POWER_FULL) == ARM_DRIVER_OK);

    // Enable MCLK output
    TEST_ASSERT(drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_PROTOCOL_I2S | ARM_SAI_MCLK_PIN_OUTPUT, 0, 16000) == ARM_DRIVER_OK);

    // Disable MCLK output (default)
    TEST_ASSERT(drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_PROTOCOL_I2S | ARM_SAI_MCLK_PIN_INACTIVE, 0, 16000) == ARM_DRIVER_OK);

    drv->PowerControl(ARM_POWER_OFF);
    drv->Uninitialize();
}

void SAI_Advanced_Mode_Test(void)
{
    TEST_MESSAGE("[MODE TEST] Testing Master / Slave Mode");
    drv->Initialize(SAI_DrvEvent);
    drv->PowerControl(ARM_POWER_FULL);

    drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_MODE_MASTER, 0, 16000);
    drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_MODE_SLAVE, 0, 16000);

    drv->PowerControl(ARM_POWER_OFF);
    drv->Uninitialize();
}

void SAI_Sync_Test(void)
{
    TEST_MESSAGE("[SYNC TEST] Testing Async / Sync Modes");
    drv->Initialize(SAI_DrvEvent);
    drv->PowerControl(ARM_POWER_FULL);

    drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_ASYNCHRONOUS, 0, 16000);
    drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_SYNCHRONOUS, 0, 16000);

    drv->PowerControl(ARM_POWER_OFF);
    drv->Uninitialize();
}

void SAI_BitOrder_Test(void)
{
    TEST_MESSAGE("[BIT ORDER TEST] Testing MSB / LSB First");
    drv->Initialize(SAI_DrvEvent);
    drv->PowerControl(ARM_POWER_FULL);

    drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_MSB_FIRST, 0, 16000);
    drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_LSB_FIRST, 0, 16000);

    drv->PowerControl(ARM_POWER_OFF);
    drv->Uninitialize();
}

void SAI_MonoMode_Test(void)
{
    TEST_MESSAGE("[MONO TEST] Testing Mono Mode Enable");
    drv->Initialize(SAI_DrvEvent);
    drv->PowerControl(ARM_POWER_FULL);

    drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_MONO_MODE, 0, 16000);

    drv->PowerControl(ARM_POWER_OFF);
    drv->Uninitialize();
}

void SAI_ClockPolarity_Test(void)
{
    TEST_MESSAGE("[CLOCK POLARITY TEST] Testing Polarity 0 / 1");
    drv->Initialize(SAI_DrvEvent);
    drv->PowerControl(ARM_POWER_FULL);

    drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_CLOCK_POLARITY_0, 0, 16000);
    drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_CLOCK_POLARITY_1, 0, 16000);

    drv->PowerControl(ARM_POWER_OFF);
    drv->Uninitialize();
}

void SAI_FrameSync_Test(void)
{
    TEST_MESSAGE("[FRAME SYNC TEST] Testing FrameSync Width, Polarity, Early");
    drv->Initialize(SAI_DrvEvent);
    drv->PowerControl(ARM_POWER_FULL);

    uint32_t frame_cfg = ARM_SAI_FRAME_LENGTH(64) | ARM_SAI_FRAME_SYNC_WIDTH(8) | ARM_SAI_FRAME_SYNC_POLARITY_LOW | ARM_SAI_FRAME_SYNC_EARLY;
    drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_PROTOCOL_USER, frame_cfg, 16000);

    drv->PowerControl(ARM_POWER_OFF);
    drv->Uninitialize();
}

void SAI_Slot_Test(void)
{
    TEST_MESSAGE("[SLOT TEST] Testing Slot Count / Size / Offset");
    drv->Initialize(SAI_DrvEvent);
    drv->PowerControl(ARM_POWER_FULL);

    uint32_t slot_cfg = ARM_SAI_SLOT_COUNT(4) | ARM_SAI_SLOT_SIZE_32 | ARM_SAI_SLOT_OFFSET(2);
    drv->Control(ARM_SAI_CONFIGURE_TX | ARM_SAI_PROTOCOL_USER, slot_cfg, 16000);

    drv->PowerControl(ARM_POWER_OFF);
    drv->Uninitialize();
}

void SAI_Data_SendReceive_Test(void)
{
    uint8_t u8TxData[256] =
    {
        0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55,
        0xCC, 0xBB, 0xAA, 0x99, 0x00, 0xFF, 0xEE, 0xDD,
        0x78, 0x56, 0x34, 0x12, 0xF0, 0xDE, 0xBC, 0x9A,
        0x3C, 0x2D, 0x1E, 0x0F, 0x78, 0x69, 0x5A, 0x4B,
        0xBE, 0xBA, 0xFE, 0xCA, 0xDE, 0xC0, 0xAD, 0xDE,
        0xCE, 0xFA, 0xED, 0xFE, 0x0D, 0xD0, 0x01, 0xC0,
        0x34, 0x12, 0xCD, 0xAB, 0xBC, 0x9A, 0x78, 0x56,
        0xDF, 0x9B, 0x57, 0x13, 0xE0, 0xAC, 0x68, 0x24,
        0x0D, 0x0C, 0x0B, 0x0A, 0x2F, 0x2E, 0x1F, 0x1E,
        0x4D, 0x4C, 0x3D, 0x3C, 0x6F, 0x6E, 0x5F, 0x5E,
        0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
        0xAA, 0xAA, 0xAA, 0xAA, 0x55, 0x55, 0x55, 0x55,
        0x0F, 0x0F, 0x0F, 0x0F, 0xF0, 0xF0, 0xF0, 0xF0,
        0xA5, 0xA5, 0xA5, 0xA5, 0x5A, 0x5A, 0x5A, 0x5A,
        0x11, 0x11, 0x11, 0x11, 0x22, 0x22, 0x22, 0x22,
        0x33, 0x33, 0x33, 0x33, 0x44, 0x44, 0x44, 0x44,
        0x88, 0x88, 0x88, 0x88, 0x99, 0x99, 0x99, 0x99,
        0xBB, 0xBB, 0xBB, 0xBB, 0xCC, 0xCC, 0xCC, 0xCC,
        0xDD, 0xDD, 0xDD, 0xDD, 0xEE, 0xEE, 0xEE, 0xEE,
        0x78, 0x56, 0x34, 0x12, 0xF0, 0xDE, 0xBC, 0x9A,
        0xDF, 0x9B, 0x57, 0x13, 0xE0, 0xAC, 0x68, 0x24,
        0xBE, 0xBA, 0xFE, 0xCA, 0xDE, 0xC0, 0xAD, 0xDE,
        0xCE, 0xFA, 0xED, 0xFE, 0x0D, 0xD0, 0x01, 0xC0,
        0x34, 0x12, 0xCD, 0xAB, 0xBC, 0x9A, 0x78, 0x56,
        0x0F, 0x0F, 0x0F, 0x0F, 0xF0, 0xF0, 0xF0, 0xF0,
        0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55,
        0xCC, 0xBB, 0xAA, 0x99, 0x00, 0xFF, 0xEE, 0xDD,
        0x0D, 0x0C, 0x0B, 0x0A, 0x2F, 0x2E, 0x1F, 0x1E,
        0x4D, 0x4C, 0x3D, 0x3C, 0x6F, 0x6E, 0x5F, 0x5E,
        0xAA, 0xAA, 0xAA, 0xAA, 0x55, 0x55, 0x55, 0x55,
        0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
        0xDF, 0x9B, 0x57, 0x13, 0xE0, 0xAC, 0x68, 0x24
    };

    uint8_t u8RxData[256] = {0};

    TEST_MESSAGE("[DATA FLOW TEST] TX and RX Functional Test");

    drv->Initialize(SAI_DrvEvent);

    drv->PowerControl(ARM_POWER_FULL);

    // Basic 16-bit I2S configuration
    drv->Control(ARM_SAI_CONFIGURE_TX |
                 ARM_SAI_MODE_MASTER |
                 ARM_SAI_PROTOCOL_I2S |
                 ARM_SAI_DATA_SIZE(8) |
                 ARM_SAI_MONO_MODE |
                 ARM_SAI_LSB_FIRST |
                 ARM_SAI_MCLK_PIN_OUTPUT,
                 0,
                 48000);

    drv->Control(ARM_SAI_CONFIGURE_RX |
                 ARM_SAI_MODE_MASTER |
                 ARM_SAI_PROTOCOL_I2S |
                 ARM_SAI_DATA_SIZE(8) |
                 ARM_SAI_MONO_MODE |
                 ARM_SAI_LSB_FIRST |
                 ARM_SAI_MCLK_PIN_OUTPUT,
                 0,
                 48000);

    drv->Control(ARM_SAI_MASK_SLOTS_RX, 0x02, 0);  // only receive from Left (slot 0)

    event_flags = osEventFlagsNew(NULL);

    drv->Control(ARM_SAI_CONTROL_RX, 1, 0);
    // Enable TX/RX before transfer
    drv->Control(ARM_SAI_CONTROL_TX, 1, 0);

    drv->Receive(u8RxData, sizeof(u8RxData));

    // Start Send and Receive together
    drv->Send(u8TxData, sizeof(u8TxData));

    osEventFlagsWait(event_flags, ARM_SAI_EVENT_SEND_COMPLETE, osFlagsWaitAny, 200);
    osEventFlagsWait(event_flags, ARM_SAI_EVENT_RECEIVE_COMPLETE, osFlagsWaitAny, 200);

    // Compare transmitted and received data
    for (int i = 0; i < (int)sizeof(u8TxData); i++)
    {
        char msg[64];
        snprintf(msg, sizeof(msg), "TX[%d]=0x%08X, RX[%d]=0x%08X", i, u8TxData[i], i, u8RxData[i]);
        TEST_MESSAGE(msg);
        // If loopback is enabled externally, compare data
        // TEST_ASSERT(tx_data[i] == rx_data[i]);
    }

    drv->PowerControl(ARM_POWER_OFF);
    drv->Uninitialize();
}

void SAI_DV_Initialize(void)
{
    sai_event = 0;
    event_flags = osEventFlagsNew(NULL);
    TEST_MESSAGE("[INFO] SAI DV Initialized");
}

void SAI_DV_Uninitialize(void)
{
    osEventFlagsDelete(event_flags);
    TEST_MESSAGE("[INFO] SAI DV Uninitialized");
}

void SAI_Validation_Run(void)
{
    SAI_DV_Initialize();

    //SAI_GetVersion();
    //SAI_GetCapabilities();
    //SAI_Init_Uninit();
    //SAI_Transmit_Test();
    //SAI_Receive_Test();
    //SAI_DataSize_Test();
    //SAI_TDM_Test();
    //SAI_Underflow_Overflow_Test();
    //SAI_Protocol_Switch_Test();
    //SAI_MCLK_Test();
    //SAI_Advanced_Mode_Test();
    //SAI_Sync_Test();
    //SAI_BitOrder_Test();
    //SAI_MonoMode_Test();
    //SAI_ClockPolarity_Test();
    //SAI_FrameSync_Test();
    //SAI_Slot_Test();

    SAI_Data_SendReceive_Test();

    SAI_DV_Uninitialize();
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

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

#if (DRV_SAI == 0)
    /* Enable I2S0 module clock */
    CLK_EnableModuleClock(I2S0_MODULE);
    CLK_SetModuleClock(I2S0_MODULE, CLK_I2SSEL_I2S0SEL_HIRC, MODULE_NoMsk);
#else
    /* Enable I2S0 module clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Select source from HIRC(12MHz) */
    CLK_SetModuleClock(SPI0_MODULE, CLK_SPISEL_SPI0SEL_HIRC, MODULE_NoMsk);
#endif

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

#if (DRV_SAI == 0)
    /* Set multi-function pins for I2S0 */
    SET_I2S0_BCLK_PC4();
    SET_I2S0_MCLK_PC3();
    SET_I2S0_DI_PC2();
    SET_I2S0_DO_PC1();
    SET_I2S0_LRCK_PC0();

    /* Enable I2S0 clock pin (PC4) schmitt trigger */
    PC->SMTEN |= GPIO_SMTEN_SMTEN4_Msk;

#elif (DRV_SAI == 2)
    /* Setup SPI0 multi-function pins */
    /* PA.3 is SPI0_SS,   PA.2 is SPI0_CLK,
       PA.1 is SPI0_MISO, PA.0 is SPI0_MOSI*/
    SET_SPI0_SS_PA3();
    SET_SPI0_CLK_PA2();
    SET_SPI0_MOSI_PA0();
    SET_SPI0_MISO_PA1();

    /* PA.4 is SPI0_I2SMCLK */
    SET_SPI0_I2SMCLK_PA4();

    /* Enable I2S0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;
#endif

    /* Lock protected registers */
    SYS_LockReg();
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART for printf */
    InitDebugUart();

    printf("\nCMSIS Driver SAI Test\n");

    osKernelInitialize();                       // Initialize CMSIS-RTOS2

    osThreadNew((osThreadFunc_t)SAI_Validation_Run, NULL, NULL);      // Create validation main thread

    osKernelStart();                            // Start thread execution

    for (;;) {}
}
