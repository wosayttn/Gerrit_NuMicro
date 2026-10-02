set(FREERTOS_DIR ${BSP_DIR}/ThirdParty/FreeRTOS)
set(FREERTOS_PORT_DIR ${FREERTOS_DIR}/portable/GCC/ARM_CM33_NTZ/non_secure)
set(FREERTOS_KERNEL_SRC
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/timers.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/queue.c
)

create_project(FreeRTOS_Blinky
    ${CMAKE_CURRENT_LIST_DIR}/Blinky/main.c
    ${FREERTOS_KERNEL_SRC}
    ${FREERTOS_DIR}/portable/MemMang/heap_2.c
    ${FREERTOS_PORT_DIR}/port.c
    ${FREERTOS_PORT_DIR}/portasm.c
)
target_include_directories(FreeRTOS_Blinky PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/Blinky
    ${FREERTOS_DIR}/include
    ${FREERTOS_PORT_DIR}
)

create_project(FreeRTOS_TicklessIdle
    ${CMAKE_CURRENT_LIST_DIR}/TicklessIdle/main.c
    ${CMAKE_CURRENT_LIST_DIR}/TicklessIdle/port.c
    ${CMAKE_CURRENT_LIST_DIR}/TicklessIdle/ParTest.c
    ${FREERTOS_KERNEL_SRC}
    ${FREERTOS_DIR}/portable/MemMang/heap_4.c
)
target_include_directories(FreeRTOS_TicklessIdle PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/TicklessIdle
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/GCC/ARM_CM4F
)