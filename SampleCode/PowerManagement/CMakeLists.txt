create_project(SYS_DPDMode_Wakeup ${CMAKE_CURRENT_LIST_DIR}/SYS_DPDMode_Wakeup/main.c)
create_project(SYS_PowerDown_MinCurrent ${CMAKE_CURRENT_LIST_DIR}/SYS_PowerDown_MinCurrent/main.c)
create_project(SYS_PowerDownMode ${CMAKE_CURRENT_LIST_DIR}/SYS_PowerDownMode/main.c)
create_project(SYS_PowerMode ${CMAKE_CURRENT_LIST_DIR}/SYS_PowerMode/main.c)
create_project(SYS_SPDMode_Wakeup ${CMAKE_CURRENT_LIST_DIR}/SYS_SPDMode_Wakeup/main.c)


# they need special startup code
# Only support building with ARMCLANG
if(C_COMPILER_ID STREQUAL ARMCLANG)
    enable_language(C ASM)
    create_project_no_startup(SYS_SPDMode_WakeupAndReturn ${CMAKE_CURRENT_LIST_DIR}/SYS_SPDMode_WakeupAndReturn/main.c ${CMAKE_CURRENT_LIST_DIR}/SYS_SPDMode_WakeupAndReturn/mystartup_m3331_Keil.S ${BSP_DIR}/Library/Device/Nuvoton/m3331/Source/system_m3331.c ${RETARGET})
    create_project_no_startup(SYS_SPDMode_WakeupVTOR ${CMAKE_CURRENT_LIST_DIR}/SYS_SPDMode_WakeupVTOR/main.c ${CMAKE_CURRENT_LIST_DIR}/SYS_SPDMode_WakeupVTOR/mystartup_m3331_Keil.S ${BSP_DIR}/Library/Device/Nuvoton/m3331/Source/system_m3331.c ${RETARGET})
endif()
