// Copyright (c) 2006-2016, Arm Limited, All Rights Reserved
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may
// not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "Driver_Storage.h"
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define TEST_ASSERT(Expr)                       if (!(Expr)) { printf("%s:%u: assertion failure\n", __FUNCTION__, __LINE__); while (1) ;}
#define TEST_ASSERT_EQUAL(expected, actual)     if ((expected) != (actual)) {printf("%s:%u: assertion failure\n", __FUNCTION__, __LINE__); while (1) ;}
#define TEST_ASSERT_NOT_EQUAL(expected, actual) if ((expected) == (actual)) {printf("%s:%u: assertion failure\n", __FUNCTION__, __LINE__); while (1) ;}

// forward declarations
void callbackHandler(int32_t status, ARM_STORAGE_OPERATION operation);
void progressStateMachine(void);

static enum
{
    NEEDS_INITIALIZATION,
    NEEDS_ERASE,
    NEEDS_PROGRAMMING,
    NEEDS_READ,
    NEEDS_VERIFICATION_FOLLOWING_READ,
    FINISHED
} state;

extern ARM_DRIVER_STORAGE ARM_Driver_Storage_(0);
ARM_DRIVER_STORAGE *drv = &ARM_Driver_Storage_(0);

#define BUFFER_SIZE     FMC_FLASH_PAGE_SIZE

static uint8_t buffer[BUFFER_SIZE];

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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

int main(int argc, char **argv)
{
    NVT_UNUSED(argc);
    NVT_UNUSED(argv);

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART for printf */
    InitDebugUart();
    printf("\nCMSIS Driver Storage Test\n");

    state = NEEDS_INITIALIZATION;
    progressStateMachine();

    while (1)
    {
        // WFE(); // optional low-power sleep
    }
}

void progressStateMachine(void)
{
    int32_t rc;
    static ARM_STORAGE_BLOCK firstBlock;

    if (!ARM_STORAGE_VALID_BLOCK(&firstBlock))
    {
        // Get the first block. This block is entered only once.
        rc = drv->GetNextBlock(NULL, &firstBlock); // get first block
        TEST_ASSERT_EQUAL(ARM_DRIVER_OK, rc);
    }

    TEST_ASSERT(ARM_STORAGE_VALID_BLOCK(&firstBlock));
    TEST_ASSERT(firstBlock.size > 0);

    switch (state)
    {
        case NEEDS_INITIALIZATION:
            rc = drv->Initialize(callbackHandler);
            TEST_ASSERT(rc >= ARM_DRIVER_OK);

            if (rc == ARM_DRIVER_OK)
            {
                TEST_ASSERT_EQUAL(1, drv->GetCapabilities().asynchronous_ops);
                state = NEEDS_ERASE;
                return; // there is pending asynchronous activity which will lead to a completion callback later.
            }

            TEST_ASSERT_EQUAL(1, rc); // synchronous completion

            rc = drv->PowerControl(ARM_POWER_FULL);
            TEST_ASSERT(rc >= ARM_DRIVER_OK);

        // intentional fall-through

        case NEEDS_ERASE:
            TEST_ASSERT(firstBlock.attributes.erase_unit > 0);
            rc = drv->Erase(firstBlock.addr, firstBlock.attributes.erase_unit);
            TEST_ASSERT(rc >= ARM_DRIVER_OK);

            if (rc == ARM_DRIVER_OK)
            {
                TEST_ASSERT_EQUAL(1, drv->GetCapabilities().asynchronous_ops);
                state = NEEDS_PROGRAMMING;
                return; // there is pending asynchronous activity which will lead to a completion callback later.
            }

            TEST_ASSERT_EQUAL(firstBlock.attributes.erase_unit, (uint32_t)rc); // synchronous completion

        // intentional fall-through

        case NEEDS_PROGRAMMING:
            TEST_ASSERT(BUFFER_SIZE >= firstBlock.attributes.erase_unit);
#define PATTERN 0xAA
            memset(buffer, 0xAA, firstBlock.attributes.erase_unit);
            rc = drv->ProgramData(firstBlock.addr, buffer, firstBlock.attributes.erase_unit);
            TEST_ASSERT(rc >= ARM_DRIVER_OK);

            if (rc == ARM_DRIVER_OK)
            {
                TEST_ASSERT_EQUAL(1, drv->GetCapabilities().asynchronous_ops);
                state = NEEDS_READ;
                return;  // there is pending asynchronous activity which will lead to a completion callback later.
            }

            TEST_ASSERT_EQUAL(firstBlock.attributes.erase_unit, (uint32_t)rc); // synchronous completion

        // intentional fall-through

        case NEEDS_READ:
            rc = drv->ReadData(firstBlock.addr, buffer, firstBlock.attributes.erase_unit);
            TEST_ASSERT(rc >= ARM_DRIVER_OK);

            if (rc == ARM_DRIVER_OK)
            {
                TEST_ASSERT_EQUAL(1, drv->GetCapabilities().asynchronous_ops);
                state = NEEDS_VERIFICATION_FOLLOWING_READ;
                return;  // there is pending asynchronous activity which will lead to a completion callback later.
            }

            TEST_ASSERT_EQUAL(firstBlock.attributes.erase_unit, (uint32_t)rc);

        // intentional fall-through

        case NEEDS_VERIFICATION_FOLLOWING_READ:
            printf("Verifying data\r\n");

            for (unsigned i = 0; i < firstBlock.attributes.erase_unit; i++)
            {
                TEST_ASSERT_EQUAL(PATTERN, buffer[i]);
            }

            state = FINISHED;
            printf("  Done\r\n");
            break;

        case FINISHED:
            break;
    } // switch (state)
}

void callbackHandler(int32_t status, ARM_STORAGE_OPERATION operation)
{
    (void)status;
    (void)operation;

    switch (operation)
    {
        case ARM_STORAGE_OPERATION_INITIALIZE:
        case ARM_STORAGE_OPERATION_READ_DATA:
        case ARM_STORAGE_OPERATION_PROGRAM_DATA:
        case ARM_STORAGE_OPERATION_ERASE:
            progressStateMachine();
            break;

        default:
            printf("[%s] unexpected callback for opcode %u with status %d\r\n", __func__, operation, status);
            break;
    }
}