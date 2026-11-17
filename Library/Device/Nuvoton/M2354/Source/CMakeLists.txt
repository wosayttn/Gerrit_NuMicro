enable_language(C ASM)

if(BSP_TOOLCHAIN STREQUAL "gcc")
    set(STARTUP_SOURCE GCC/startup_M2354.S)
    list(APPEND STARTUP_SOURCE GCC/_syscalls.c)
else()
    set(STARTUP_SOURCE ARM/startup_M2354.S)
endif()

add_library(system_startup 
    system_M2354.c
    ${STARTUP_SOURCE}
)
