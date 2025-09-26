#include "Driver_I2C.h"
#include "NuMicro.h"
#include <string.h>

/*
  CMSIS Driver Instance | Nuvoton Hardware Resource
  :---------------------|:--------------------------
  Driver_I2C0           | I2C0
  Driver_I2C1           | I2C1
  Driver_I2C2           | I2C2
  Driver_I2C3           | I2C3
  Driver_I2C4           | LPI2C0
  Driver_I2C5           | UI2C0
  Driver_I2C6           | I3C0
*/


extern ARM_DRIVER_I2C            Driver_I2C0;
extern ARM_DRIVER_I2C            Driver_I2C1;
extern ARM_DRIVER_I2C            Driver_I2C2;
extern ARM_DRIVER_I2C            Driver_I2C3;
extern ARM_DRIVER_I2C            Driver_I2C4;
extern ARM_DRIVER_I2C            Driver_I2C5;
extern ARM_DRIVER_I2C            Driver_I2C6;

static volatile uint32_t I2C_Event0;
static volatile uint32_t I2C_Event1;

/* I2C Signal Event function callback */
static void I2C_SignalEvent0(uint32_t event)
{
    I2C_Event0 |= event;
}

/* I2C Signal Event function callback */
static void I2C_SignalEvent1(uint32_t event)
{
    I2C_Event1 |= event;
}

static volatile uint8_t g_au8MstTxData[512] __attribute__((aligned(4)));
static volatile uint8_t g_au8MstRxData[512] __attribute__((aligned(4)));
static volatile uint8_t g_au8SlvTxData[512] __attribute__((aligned(4)));
static volatile uint8_t g_au8SlvRxData[512] __attribute__((aligned(4)));

/*
#define ARM_I2C_EVENT_TRANSFER_DONE       (1UL << 0)  ///< Master/Slave Transmit/Receive finished
#define ARM_I2C_EVENT_TRANSFER_INCOMPLETE (1UL << 1)  ///< Master/Slave Transmit/Receive incomplete transfer
#define ARM_I2C_EVENT_SLAVE_TRANSMIT      (1UL << 2)  ///< Addressed as Slave Transmitter but transmit operation is not set.
#define ARM_I2C_EVENT_SLAVE_RECEIVE       (1UL << 3)  ///< Addressed as Slave Receiver but receive operation is not set.
#define ARM_I2C_EVENT_ADDRESS_NACK        (1UL << 4)  ///< Address not acknowledged from Slave
#define ARM_I2C_EVENT_GENERAL_CALL        (1UL << 5)  ///< Slave addressed with general call address
#define ARM_I2C_EVENT_ARBITRATION_LOST    (1UL << 6)  ///< Master lost arbitration
#define ARM_I2C_EVENT_BUS_ERROR           (1UL << 7)  ///< Bus error detected (START/STOP at illegal position)
#define ARM_I2C_EVENT_BUS_CLEAR           (1UL << 8)  ///< Bus clear finished
*/

void I2C_ParseEvent(uint32_t event0, uint32_t event1)
{
    uint32_t i = 0, I2C_Event[2] = {event0, event1}, event;

    for (i = 0; i < 2; i++)
    {
        event = I2C_Event[i];
        printf(" - I2C_Event%d = %X...\n", i, event);

        if (event & ARM_I2C_EVENT_TRANSFER_DONE)
        {
            printf(" -- Master/Slave Transmit/Receive finished\n");
        }

        if (event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE)
        {
            printf(" -- Master/Slave Transmit/Receive incomplete transfer\n");
        }

        if (event & ARM_I2C_EVENT_SLAVE_TRANSMIT)
        {
            printf(" -- Addressed as Slave Transmitter but transmit operation is not set.\n");
        }

        if (event & ARM_I2C_EVENT_SLAVE_RECEIVE)
        {
            printf(" -- Addressed as Slave Receiver but receive operation is not set.\n");
        }

        if (event & ARM_I2C_EVENT_ADDRESS_NACK)
        {
            printf(" -- Address not acknowledged from Slave\n");
        }

        if (event & ARM_I2C_EVENT_GENERAL_CALL)
        {
            printf(" -- Slave addressed with general call address\n");
        }

        if (event & ARM_I2C_EVENT_ARBITRATION_LOST)
        {
            printf(" -- Master lost arbitration\n");
        }

        if (event & ARM_I2C_EVENT_BUS_ERROR)
        {
            printf(" -- Bus error detected (START/STOP at illegal position)\n");
        }

        if (event & ARM_I2C_EVENT_BUS_CLEAR)
        {
            printf(" -- Bus clear finished\n");
        }
    }
}


int32_t i2c_pair_7bit(ARM_DRIVER_I2C *I2Cdrv0, ARM_DRIVER_I2C *I2Cdrv1)
{
    uint32_t i, cnt = 0, temp;
    I2Cdrv0->Initialize(I2C_SignalEvent0);
    I2Cdrv0->PowerControl(ARM_POWER_FULL);
    I2Cdrv0->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    I2Cdrv0->Control(ARM_I2C_BUS_CLEAR, 0);
    I2Cdrv0->Control(ARM_I2C_OWN_ADDRESS, 0x14);
    I2Cdrv1->Initialize(I2C_SignalEvent1);
    I2Cdrv1->PowerControl(ARM_POWER_FULL);
    I2Cdrv1->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    I2Cdrv1->Control(ARM_I2C_OWN_ADDRESS, 0x15);
    I2C_Event0 = 0;
    I2C_Event1 = 0;
    {
        I2C_Event0 = 0;
        I2C_Event1 = 0;

        for (i = 0; i < 512; i++)
        {
            g_au8MstTxData[i] = i & 0xFF;
            g_au8SlvTxData[i] = (i * 2) & 0xFF;
        }

        memset((void *)g_au8MstRxData, 0, 512);
        memset((void *)g_au8SlvRxData, 0, 512);
        // Master Write, Slave Read
        printf("\n1. 7-Bit MasterTransmit +  SlaveReceive...\n");
        I2Cdrv1->SlaveReceive((uint8_t *)g_au8SlvRxData, 512);
        I2Cdrv0->MasterTransmit(0x15, (uint8_t *)g_au8MstTxData, 512, false);

        while ((I2C_Event0 & ARM_I2C_EVENT_TRANSFER_DONE) == 0U)
        {
            temp = I2Cdrv0->GetDataCount();

            if (temp != cnt)
            {
                printf(" - %d bytes transmitted ...\n", temp);
                cnt = temp;
            }
        }

        I2C_ParseEvent(I2C_Event0, I2C_Event1);

        //
        for (i = 0; i < 512; i++)
        {
            if (g_au8SlvRxData[i] != g_au8MstTxData[i])
            {
                printf(" - Compair Fail @ %d, TX = %d, RX = %d...\n"
                       , i
                       , g_au8MstTxData[i]
                       , g_au8SlvRxData[i]
                      );
                break;
            }
        }

        //
        printf("\n2. 7-Bit MasterReceive +  SlaveTransmit...\n");
        I2C_Event0 = 0;
        I2C_Event1 = 0;
        // Master Read, Slave Write
        I2Cdrv1->SlaveTransmit((uint8_t *)g_au8SlvTxData, 512);
        I2Cdrv0->MasterReceive(0x15, (uint8_t *)g_au8MstRxData, 512, false);

        while ((I2C_Event0 & ARM_I2C_EVENT_TRANSFER_DONE) == 0U)
        {
            temp = I2Cdrv0->GetDataCount();

            if (temp != cnt)
            {
                printf(" - %d bytes Received ...\n", temp);
                cnt = temp;
            }
        }

        I2C_ParseEvent(I2C_Event0, I2C_Event1);

        //
        for (i = 0; i < 512; i++)
        {
            if (g_au8SlvTxData[i] != g_au8MstRxData[i])
            {
                printf(" - Compair Fail @ %d, TX = %d, RX = %d...\n"
                       , i
                       , g_au8SlvTxData[i]
                       , g_au8MstRxData[i]
                      );
                break;
            }
        }
    }
    I2Cdrv0->PowerControl(ARM_POWER_OFF);
    I2Cdrv0->Uninitialize();
    I2Cdrv1->PowerControl(ARM_POWER_OFF);
    I2Cdrv1->Uninitialize();
    return 0;
}

int32_t i2c_pair_10bit(ARM_DRIVER_I2C *I2Cdrv0, ARM_DRIVER_I2C *I2Cdrv1)
{
    uint32_t i, cnt = 0, temp, tx_size = 512;
    I2Cdrv0->Initialize(I2C_SignalEvent0);
    I2Cdrv0->PowerControl(ARM_POWER_FULL);
    I2Cdrv0->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    I2Cdrv0->Control(ARM_I2C_BUS_CLEAR, 0);
    I2Cdrv0->Control(ARM_I2C_OWN_ADDRESS, 0x114 | ARM_I2C_ADDRESS_10BIT);
    I2Cdrv1->Initialize(I2C_SignalEvent1);
    I2Cdrv1->PowerControl(ARM_POWER_FULL);
    I2Cdrv1->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    I2Cdrv1->Control(ARM_I2C_OWN_ADDRESS, 0x115 | ARM_I2C_ADDRESS_10BIT);
    I2C_Event0 = 0;
    I2C_Event1 = 0;
    {
        I2C_Event0 = 0;
        I2C_Event1 = 0;

        for (i = 0; i < tx_size; i++)
        {
            g_au8MstTxData[i] = i & 0xFF;
            g_au8SlvTxData[i] = (i * 2) & 0xFF;
        }

        memset((void *)g_au8MstRxData, 0, tx_size);
        memset((void *)g_au8SlvRxData, 0, tx_size);
        // Master Write, Slave Read
        printf("\n1. 10-Bit MasterTransmit + SlaveReceive with Abort Transfer...\n");
        I2Cdrv1->SlaveReceive((uint8_t *)g_au8SlvRxData, tx_size);
        I2Cdrv0->MasterTransmit(0x115 | ARM_I2C_ADDRESS_10BIT, (uint8_t *)g_au8MstTxData, tx_size, false);

        while ((I2C_Event0 & ARM_I2C_EVENT_TRANSFER_DONE) == 0U)
        {
            temp = I2Cdrv0->GetDataCount();

            if (temp != cnt)
            {
                printf(" - %d bytes transmitted ...\n", temp);
                cnt = temp;
            }

            if (temp > 10)
            {
                I2Cdrv0->Control(ARM_I2C_ABORT_TRANSFER, 0);
                break;
            }
        }

        while ((I2C_Event0 & ARM_I2C_EVENT_TRANSFER_DONE) == 0U) {};

        temp = I2Cdrv0->GetDataCount();

        printf(" - %d bytes transmitted ...\n", temp);

        I2C_ParseEvent(I2C_Event0, I2C_Event1);

        //
        for (i = 0; i < tx_size; i++)
        {
            if (g_au8SlvRxData[i] != g_au8MstTxData[i])
            {
                printf("\n - Compair Fail @ %d, TX = %d, RX = %d...\n"
                       , i
                       , g_au8MstTxData[i]
                       , g_au8SlvRxData[i]
                      );
                break;
            }
        }

        //
        printf("\n2. 10-Bit MasterReceive +  SlaveTransmit with Abort Transfer...\n");
        I2C_Event0 = 0;
        I2C_Event1 = 0;
        // Master Read, Slave Write
        I2Cdrv1->SlaveTransmit((uint8_t *)g_au8SlvTxData, tx_size);
        I2Cdrv0->MasterReceive(0x115 | ARM_I2C_ADDRESS_10BIT, (uint8_t *)g_au8MstRxData, tx_size, false);

        while ((I2C_Event0 & ARM_I2C_EVENT_TRANSFER_DONE) == 0U)
        {
            temp = I2Cdrv0->GetDataCount();

            if (temp != cnt)
            {
                printf(" - %d bytes Received ...\n", temp);
                cnt = temp;
            }

            if (temp > 10)
            {
                I2Cdrv0->Control(ARM_I2C_ABORT_TRANSFER, 0);
                break;
            }
        }

        while ((I2C_Event0 & ARM_I2C_EVENT_TRANSFER_DONE) == 0U) {};

        temp = I2Cdrv0->GetDataCount();

        printf(" - %d bytes Received ...\n", temp);

        I2C_ParseEvent(I2C_Event0, I2C_Event1);

        //
        for (i = 0; i < tx_size; i++)
        {
            if (g_au8SlvTxData[i] != g_au8MstRxData[i])
            {
                printf("\n - Compair Fail @ %d, TX = %d, RX = %d...\n"
                       , i
                       , g_au8SlvTxData[i]
                       , g_au8MstRxData[i]
                      );
                break;
            }
        }
    }
    printf("\n\nSwitch Role ...0: Slave, 1: Master\n");
    {
        I2C_Event0 = 0;
        I2C_Event1 = 0;

        for (i = 0; i < tx_size; i++)
        {
            g_au8MstTxData[i] = i & 0xFF;
            g_au8SlvTxData[i] = (i * 2) & 0xFF;
        }

        memset((void *)g_au8MstRxData, 0, tx_size);
        memset((void *)g_au8SlvRxData, 0, tx_size);
        // Master Write, Slave Read
        printf("\n3. 10-Bit MasterTransmit +  SlaveReceive...\n");
        I2Cdrv0->SlaveReceive((uint8_t *)g_au8SlvRxData, tx_size);
        I2Cdrv1->MasterTransmit(0x114 | ARM_I2C_ADDRESS_10BIT, (uint8_t *)g_au8MstTxData, tx_size, false);

        while ((I2C_Event1 & ARM_I2C_EVENT_TRANSFER_DONE) == 0U)
        {
            temp = I2Cdrv1->GetDataCount();

            if (temp != cnt)
            {
                printf(" - %d bytes transmitted ...\n", temp);
                cnt = temp;
            }
        }

        I2C_ParseEvent(I2C_Event0, I2C_Event1);

        //
        for (i = 0; i < tx_size; i++)
        {
            if (g_au8SlvRxData[i] != g_au8MstTxData[i])
            {
                printf("\n - Compair Fail @ %d, TX = %d, RX = %d...\n"
                       , i
                       , g_au8MstTxData[i]
                       , g_au8SlvRxData[i]
                      );
                break;
            }
        }

        //
        printf("\n4. 10-Bit MasterReceive +  SlaveTransmit...\n");
        I2C_Event0 = 0;
        I2C_Event1 = 0;
        // Master Read, Slave Write
        I2Cdrv0->SlaveTransmit((uint8_t *)g_au8SlvTxData, tx_size);
        I2Cdrv1->MasterReceive(0x114 | ARM_I2C_ADDRESS_10BIT, (uint8_t *)g_au8MstRxData, tx_size, false);

        while ((I2C_Event1 & ARM_I2C_EVENT_TRANSFER_DONE) == 0U)
        {
            temp = I2Cdrv1->GetDataCount();

            if (temp != cnt)
            {
                printf(" - %d bytes Received ...\n", temp);
                cnt = temp;
            }
        }

        I2C_ParseEvent(I2C_Event0, I2C_Event1);

        //
        for (i = 0; i < tx_size; i++)
        {
            if (g_au8SlvTxData[i] != g_au8MstRxData[i])
            {
                printf("\n - Compair Fail @ %d, TX = %d, RX = %d...\n"
                       , i
                       , g_au8SlvTxData[i]
                       , g_au8MstRxData[i]
                      );
                break;
            }
        }
    }
    I2Cdrv0->PowerControl(ARM_POWER_OFF);
    I2Cdrv0->Uninitialize();
    I2Cdrv1->PowerControl(ARM_POWER_OFF);
    I2Cdrv1->Uninitialize();
    return 0;
}

int32_t i2c_pair_GCmode(ARM_DRIVER_I2C *I2Cdrv0, ARM_DRIVER_I2C *I2Cdrv1)
{
    uint32_t i, cnt = 0, temp;
    I2Cdrv0->Initialize(I2C_SignalEvent0);
    I2Cdrv0->PowerControl(ARM_POWER_FULL);
    I2Cdrv0->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    I2Cdrv0->Control(ARM_I2C_BUS_CLEAR, 0);
    I2Cdrv0->Control(ARM_I2C_OWN_ADDRESS, 0x14 | ARM_I2C_ADDRESS_GC);
    I2Cdrv1->Initialize(I2C_SignalEvent1);
    I2Cdrv1->PowerControl(ARM_POWER_FULL);
    I2Cdrv1->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    I2Cdrv1->Control(ARM_I2C_OWN_ADDRESS, 0x15 | ARM_I2C_ADDRESS_GC);
    I2C_Event0 = 0;
    I2C_Event1 = 0;

    for (i = 0; i < 512; i++)
    {
        g_au8MstTxData[i] = i & 0xFF;
    }

    memset((void *)g_au8SlvRxData, 0, 512);
    printf("\n1. GC-Mode MasterTransmit +  SlaveReceive...\n");
    I2Cdrv1->SlaveReceive((uint8_t *)g_au8SlvRxData, 512);
    I2Cdrv0->MasterTransmit(0, (uint8_t *)g_au8MstTxData, 512, false);

    while ((I2C_Event0 & ARM_I2C_EVENT_TRANSFER_DONE) == 0U)
    {
        temp = I2Cdrv0->GetDataCount();

        if (temp != cnt)
        {
            printf(" - %d bytes transmitted ...\n", temp);
            cnt = temp;
        }
    }

    I2C_ParseEvent(I2C_Event0, I2C_Event1);

    for (i = 0; i < 512; i++)
    {
        if (g_au8SlvRxData[i] != g_au8MstTxData[i])
        {
            printf("Compair Fail @ %d, TX = %d, RX = %d...\n"
                   , i
                   , g_au8MstTxData[i]
                   , g_au8SlvRxData[i]
                  );
            break;
        }
    }

    // Switch ....
    printf("\n\nSwitch Role ...0: Slave, 1: Master\n");
    I2C_Event0 = 0;
    I2C_Event1 = 0;
    memset((void *)g_au8SlvRxData, 0, 512);
    printf("\n2. GC-Mode MasterTransmit +  SlaveReceive...\n");
    I2Cdrv0->SlaveReceive((uint8_t *)g_au8SlvRxData, 512);
    I2Cdrv1->MasterTransmit(0, (uint8_t *)g_au8MstTxData, 512, false);

    while ((I2C_Event1 & ARM_I2C_EVENT_TRANSFER_DONE) == 0U)
    {
        temp = I2Cdrv1->GetDataCount();

        if (temp != cnt)
        {
            printf(" - %d bytes transmitted ...\n", temp);
            cnt = temp;
        }
    }

    I2C_ParseEvent(I2C_Event0, I2C_Event1);

    for (i = 0; i < 512; i++)
    {
        if (g_au8SlvRxData[i] != g_au8MstTxData[i])
        {
            printf("Compair Fail @ %d, TX = %d, RX = %d...\n"
                   , i
                   , g_au8MstTxData[i]
                   , g_au8SlvRxData[i]
                  );
            break;
        }
    }

    I2Cdrv0->PowerControl(ARM_POWER_OFF);
    I2Cdrv0->Uninitialize();
    I2Cdrv1->PowerControl(ARM_POWER_OFF);
    I2Cdrv1->Uninitialize();
    return 0;
}

int32_t i2c_pair(void)
{
    // Driver_I2C6 (I3C0), only support 7-Bit Mode
    i2c_pair_7bit(&Driver_I2C6, &Driver_I2C0);
    i2c_pair_7bit(&Driver_I2C1, &Driver_I2C2);
    i2c_pair_GCmode(&Driver_I2C1, &Driver_I2C2);
    i2c_pair_10bit(&Driver_I2C1, &Driver_I2C2);
    return 0;
}
