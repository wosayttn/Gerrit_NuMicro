FreeRTOS's tickless idle mode works by halting the periodic tick interrupt when the system is idle, i.e., when there are no tasks available to execute. The tick interrupt is then resumed with a corrective adjustment to the RTOS tick count value when needed. By disabling the tick interrupt, the microcontroller can conserve power by entering a deep sleep state until an interrupt is triggered, or a task needs to be transitioned to the Ready state by the RTOS kernel.

To activate the built-in tickless idle feature in FreeRTOS, you need to set the value of configUSE_TICKLESS_IDLE to 1 in the FreeRTOSConfig.h file.

This sample demonstrates how to enable tickless idle by setting configUSE_TICKLESS_IDLE to 1 in FreeRTOSConfig.h.
Monitor HCLK on PD.12 to check whether the system enters power-down mode (when the clock is off).
The sample creates a task that prints the tick count and delays for 500 ticks.
PB.1 through PB.5 can wake the system.