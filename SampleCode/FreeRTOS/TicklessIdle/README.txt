FreeRTOS's tickless idle mode works by halting the periodic tick interrupt when the system is idle, i.e., when there are no tasks available to execute. The tick interrupt is then resumed with a corrective adjustment to the RTOS tick count value when needed. By disabling the tick interrupt, the microcontroller can conserve power by entering a deep sleep state until an interrupt is triggered, or a task needs to be transitioned to the Ready state by the RTOS kernel.

To activate the built-in tickless idle feature in FreeRTOS, you need to set the value of configUSE_TICKLESS_IDLE to 1 in the FreeRTOSConfig.h file.

This sample enables tickless idle by setting configUSE_TICKLESS_IDLE to 1 in FreeRTOSConfig.h.
Monitor the HCLK/64 clock output on PA.3 to see when the CPU enters Power-down mode (the output stops).
A task prints the tick count and delays for 1000 ticks (1 second at the configured 1000 Hz tick rate).
PB.1 through PB.5 can wake the system in this sample.