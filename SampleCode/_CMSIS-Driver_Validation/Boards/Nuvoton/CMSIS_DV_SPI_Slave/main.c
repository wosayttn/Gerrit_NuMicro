#include <stdio.h>
#include <string.h>

#include "NuMicro.h"
#include "Driver_SPI.h"
#include "cmsis_os2.h"

//------------------------------------------------------------------------------
#define DRV_SPI     0
#define _ARM_Driver_SPI_(n)   Driver_SPI##n
#define ARM_Driver_SPI_(n)    _ARM_Driver_SPI_(n)

#define TEST_ASSERT(cond) \
    do { \
        if (!(cond)) { \
            printf("[ASSERT FAILED] %s at %s:%d\n", #cond, __FILE__, __LINE__); \
            return; \
        } \
    } while (0)

#define TEST_MESSAGE(msg) \
    printf("[TEST] %s\n", msg)

//------------------------------------------------------------------------------
extern ARM_DRIVER_SPI ARM_Driver_SPI_(DRV_SPI);

static ARM_DRIVER_SPI *drv = &ARM_Driver_SPI_(DRV_SPI);
static osEventFlagsId_t evt_id;
static volatile uint32_t spi_event;

void generate_random_pattern(uint8_t *pu8Data, uint32_t u32DataSize)
{
    uint32_t u32Val = 0, u32i = 0;

    /* Create Trim Pattern */
    for (u32i = 0; u32i < u32DataSize; u32i++)
    {
        u32Val = (u32i & 0x0F) ^ (u32i >> 4) ^ (u32i >> 3);

        if (u32i & 0x01)
        {
            u32Val = ~u32Val;
        }

        pu8Data[u32i] = ~(uint8_t)(u32Val ^ (u32i << 3) ^ (u32i >> 2));
    }
}

static void SPI_Callback(uint32_t event)
{
    spi_event |= event;
    osEventFlagsSet(evt_id, event);
}

void SPI_Test_Initialize(void)
{
    evt_id = osEventFlagsNew(NULL);
    TEST_ASSERT(drv->Initialize(SPI_Callback) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->PowerControl(ARM_POWER_FULL) == ARM_DRIVER_OK);
    TEST_MESSAGE("[INFO] SPI Initialized");
}

void SPI_Test_Uninitialize(void)
{
    TEST_ASSERT(drv->PowerControl(ARM_POWER_OFF) == ARM_DRIVER_OK);
    TEST_ASSERT(drv->Uninitialize() == ARM_DRIVER_OK);
    osEventFlagsDelete(evt_id);
    TEST_MESSAGE("[INFO] SPI Uninitialized");
}

void SPI_SendOnly_Test(uint8_t *au8TxData, uint32_t u32DataSize)
{
    TEST_MESSAGE("[TEST] SPI Send Only");

    drv->Control(ARM_SPI_MODE_SLAVE |
                 ARM_SPI_CPOL0_CPHA0 |
                 ARM_SPI_MSB_LSB     |
                 ARM_SPI_DATA_BITS(8),
                 1000000);

    TEST_ASSERT(drv->Send(au8TxData, u32DataSize) == ARM_DRIVER_OK);
    osEventFlagsWait(evt_id, ARM_SPI_EVENT_TRANSFER_COMPLETE, osFlagsWaitAny, 200);
}

void SPI_ReceiveOnly_Test(uint8_t *au8RxData, uint32_t u32DataSize)
{
    TEST_MESSAGE("[TEST] SPI Receive Only");

    drv->Control(ARM_SPI_MODE_SLAVE |
                 ARM_SPI_CPOL0_CPHA0 |
                 ARM_SPI_MSB_LSB     |
                 ARM_SPI_DATA_BITS(8),
                 1000000);

    TEST_ASSERT(drv->Receive(au8RxData, u32DataSize) == ARM_DRIVER_OK);
    osEventFlagsWait(evt_id, ARM_SPI_EVENT_TRANSFER_COMPLETE, osFlagsWaitAny, 200);
}

void SPI_Validation_Run(void)
{
    uint8_t au8TxData[512] = {0};
    uint8_t au8RxData[512] = {0};
    char msg[64];

    generate_random_pattern(au8TxData, sizeof(au8TxData));

    SPI_Test_Initialize();

    SPI_ReceiveOnly_Test(au8RxData, sizeof(au8RxData));
    SPI_SendOnly_Test(au8TxData, sizeof(au8TxData));

    SPI_Test_Uninitialize();

    for (int i = 0; i < 32; i++)
    {
        snprintf(msg, sizeof(msg), "RX[%d] = 0x%02X", i, au8RxData[i]);
        TEST_MESSAGE(msg);
    }
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

#if (DRV_SPI == 0)
    /* Enable SPI0 module clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Select SPI0 module clock source as PCLK1 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_SPISEL_SPI0SEL_PCLK0, MODULE_NoMsk);
#elif (DRV_SPI == 4)
    /* Enable SPI0 module clock */
    CLK_EnableModuleClock(QSPI0_MODULE);

    /* Select SPI0 module clock source as PCLK1 */
    CLK_SetModuleClock(QSPI0_MODULE, CLK_QSPISEL_QSPI0SEL_PCLK0, MODULE_NoMsk);
#elif (DRV_SPI == 6)
    /* Enable SPI0 module clock */
    CLK_EnableModuleClock(LPSPI0_MODULE);

    /* Select SPI0 module clock source as PCLK1 */
    CLK_SetModuleClock(LPSPI0_MODULE, CLK_LPSPISEL_LPSPI0SEL_PCLK4, MODULE_NoMsk);
#elif (DRV_SPI == 7)
    /* Enable SPI0 module clock */
    CLK_EnableModuleClock(USCI0_MODULE);
#elif (DRV_SPI == 8)
    /* Enable SPIM0 module clock */
    CLK_EnableModuleClock(SPIM0_MODULE);
#endif

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

#if (DRV_SPI == 0)
    /* Setup SPI0 multi-function pins */
    SET_SPI0_MOSI_PA0();
    SET_SPI0_MISO_PA1();
    SET_SPI0_CLK_PA2();
    SET_SPI0_SS_PA3();
#elif (DRV_SPI == 4)
    SET_QSPI0_MOSI0_PA0();
    SET_QSPI0_MISO0_PA1();
    SET_QSPI0_CLK_PA2();
    SET_QSPI0_SS_PA3();
#elif (DRV_SPI == 6)
    SET_LPSPI0_MOSI_PA0();
    SET_LPSPI0_MISO_PA1();
    SET_LPSPI0_CLK_PA2();
    SET_LPSPI0_SS_PA3();
#elif (DRV_SPI == 7)
    /* Set USCI0_SPI multi-function pins */
    SET_USCI0_CTL0_PB0();
    SET_USCI0_CLK_PA11();
    SET_USCI0_DAT0_PA10();
    SET_USCI0_DAT1_PA9();
#elif (DRV_SPI == 8)
    uint32_t u32SlewRate = GPIO_SLEWCTL_HIGH;

    /* Init SPIM multi-function pins */
    SET_SPIM0_CLK_PH13();
    SET_SPIM0_MISO_PJ4();
    SET_SPIM0_MOSI_PJ3();
    SET_SPIM0_D2_PJ5();
    SET_SPIM0_D3_PJ6();
    SET_SPIM0_SS_PJ7();

    PH->SMTEN |= (GPIO_SMTEN_SMTEN13_Msk);

    PJ->SMTEN |= (GPIO_SMTEN_SMTEN3_Msk |
                  GPIO_SMTEN_SMTEN4_Msk |
                  GPIO_SMTEN_SMTEN5_Msk |
                  GPIO_SMTEN_SMTEN6_Msk |
                  GPIO_SMTEN_SMTEN7_Msk);

    /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
    GPIO_SetSlewCtl(PH, BIT13, u32SlewRate);

    GPIO_SetSlewCtl(PJ, BIT3, u32SlewRate);
    GPIO_SetSlewCtl(PJ, BIT4, u32SlewRate);
    GPIO_SetSlewCtl(PJ, BIT5, u32SlewRate);
    GPIO_SetSlewCtl(PJ, BIT6, u32SlewRate);

#endif

    /* Enable SPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* USCI_SPI clock pin enable schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN11_Msk;

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

    osThreadNew((osThreadFunc_t)SPI_Validation_Run, NULL, NULL);      // Create validation main thread

    osKernelStart();                            // Start thread execution

    for (;;) {}
}
