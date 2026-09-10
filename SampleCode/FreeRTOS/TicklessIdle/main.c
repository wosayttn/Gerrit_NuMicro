/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show task delay with tickless idle
 *
 * @copyright (c) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/* Standard includes. */
#include "string.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "NuMicro.h"

//*** <<< Use Configuration Wizard in Context Menu >>> ***
// <o> Sleep time
#define TIME_DELAY_SLEEP  500
//*** <<< end of configuration section >>>    ***


#define tickless_task_PRIORITY  (configMAX_PRIORITIES - 2)

#define LED     PG4

static void Tickless_task(void *pvParameters);
void vParTestInitialise(void);

/*-----------------------------------------------------------*/

/*
 * Perform any application specific hardware configuration.  The clocks,
 * memory, etc. are configured before main() is called.
 */
static void prvSetupHardware(void);

/*
 * CMSIS clock configuration function.
 */
extern void SystemCoreClockUpdate(void);

/*-----------------------------------------------------------*/

int main(void)
{
    /* Prepare the hardware to run this demo. */
    prvSetupHardware();

    /* Create tickless task */
    if(xTaskCreate(Tickless_task, "Tickless_task", configMINIMAL_STACK_SIZE + 100, NULL, tickless_task_PRIORITY, NULL) != pdPASS)
    {
        printf("Task creation failed!\n");
        while(1);
    }

    printf("\r\nTick count :\n");

    /* Task Scheduler */
    vTaskStartScheduler();
    for(;;) {}
}

/* Tickless Task */
static void Tickless_task(void *pvParameters)
{
    for(;;)
    {
        printf("%d\n", xTaskGetTickCount());
        /* Toggle LED */
        LED ^= 1;
        vTaskDelay(TIME_DELAY_SLEEP);
    }
}

/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
    vParTestInitialise();
}

/*-----------------------------------------------------------*/
#if ( configCHECK_FOR_STACK_OVERFLOW > 0 )

    void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                        char * pcTaskName )
    {
        /* Check pcTaskName for the name of the offending task,
         * or pxCurrentTCB if pcTaskName has itself been corrupted. */
        ( void ) xTask;
        ( void ) pcTaskName;
    }

#endif /* #if ( configCHECK_FOR_STACK_OVERFLOW > 0 ) */