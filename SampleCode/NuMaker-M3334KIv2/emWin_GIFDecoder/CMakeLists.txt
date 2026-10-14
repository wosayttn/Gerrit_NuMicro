set(EMWIN_DEMO_DIR ${CMAKE_CURRENT_LIST_DIR})
set(EMWIN_DIR      ${BSP_DIR}/ThirdParty/emWin_light)

# ============================================================
# emWin pre-built library (imported)
# ============================================================
add_library(emwin_lib STATIC IMPORTED)
if(C_COMPILER_ID STREQUAL ARMCLANG)
    set_target_properties(emwin_lib PROPERTIES
        IMPORTED_LOCATION ${EMWIN_DIR}/Lib/NUemWin_CM33_Keil.lib
    )
elseif(C_COMPILER_ID STREQUAL GCC)
    set_target_properties(emwin_lib PROPERTIES
        IMPORTED_LOCATION ${EMWIN_DIR}/Lib/libNUemWin_CM33_GNU.a
    )
endif()
target_include_directories(emwin_lib INTERFACE
    ${EMWIN_DIR}/Include
    ${EMWIN_DIR}/Config
)
# emwin_lib references FMC_Open/FMC_Close from standard_driver. Declaring it as
# an INTERFACE dependency ensures the linker sees standard_driver *after*
# emwin_lib, so those symbols are resolved on the second pass.
target_link_libraries(emwin_lib INTERFACE standard_driver)

# ============================================================
# emWin_GIFDecoder executable
# ============================================================
create_project(emWin_GIFDecoder
    ${EMWIN_DEMO_DIR}/main.c
    ${EMWIN_DEMO_DIR}/Application/DecodeGIF.c
    ${EMWIN_DEMO_DIR}/Application/lcd/ILI9341.c
    ${EMWIN_DIR}/Config/GUIConf2.c
    ${EMWIN_DIR}/Config/GUI_X.c
    ${EMWIN_DIR}/Config/LCDConf2.c
)
target_link_libraries(emWin_GIFDecoder emwin_lib)
target_compile_definitions(emWin_GIFDecoder PRIVATE
    __DEMO_320x240__
)
target_include_directories(emWin_GIFDecoder PRIVATE
    ${EMWIN_DEMO_DIR}/Application
    ${EMWIN_DEMO_DIR}/Application/lcd
    ${EMWIN_DIR}/Include
    ${EMWIN_DIR}/Config
)
