#ifndef __DEBUG_PRINTF_H__
#define __DEBUG_PRINTF_H__

// -------- <<< Use Configuration Wizard in Context Menu >>> --------
#ifndef TRACK_HARDFAULT
    // <q> Enable HardFault Status Tracking
    // <i> Set to 1 to enable HardFault status tracking; set to 0 to disable.
    #define TRACK_HARDFAULT     0
#endif

#ifndef CHECK_SCU_VIOLATION
    // <q> Enable SCU violation check debug message
    // <i> Set to 1 to enable SCU violation check; set to 0 to disable.
    #define CHECK_SCU_VIOLATION 0
#endif

#ifndef DEBUG_PRINTF
    // <q> Enable Debug printf message
    // <i> Set to 1 to enable debug printf; set to 0 to disable.
    #define DEBUG_PRINTF        0
#endif
// -------- <<< End of Configuration Wizard Context Menu >>> --------

#if (DEBUG_PRINTF == 1) || (TRACK_HARDFAULT == 1) || (CHECK_SCU_VIOLATION == 1)
    /*cstat !MISRAC2012-Rule-21.6: The debug_printf macro is used for debugging purposes and is not intended for production code. */
    #include <stdio.h>
    #define debug_printf(...)   ((void)printf(__VA_ARGS__))
#else
    #define debug_printf(...)   ((void)0)
#endif

#endif /* __DEBUG_PRINTF_H__ */