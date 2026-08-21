set(AUDIO_GUI_DIR ${CMAKE_CURRENT_LIST_DIR})

# ============================================================
# NuvotonAudioGUI_Firmware executable
# ============================================================
create_project(NuvotonAudioGUI_Firmware
    ${AUDIO_GUI_DIR}/main.c
    ${AUDIO_GUI_DIR}/src/adc_converter.c
    ${AUDIO_GUI_DIR}/src/codec_config.c
    ${AUDIO_GUI_DIR}/src/descriptors.c
    ${AUDIO_GUI_DIR}/src/HIRC_autoTrim.c
    ${AUDIO_GUI_DIR}/src/i2c_process.c
    ${AUDIO_GUI_DIR}/src/sys_init.c
    ${AUDIO_GUI_DIR}/src/usbd_audio.c
    ${AUDIO_GUI_DIR}/src/user_config.c
)
target_include_directories(NuvotonAudioGUI_Firmware PRIVATE
    ${AUDIO_GUI_DIR}
    ${AUDIO_GUI_DIR}/src
    ${BSP_DIR}/Library/Device/Nuvoton/m3331/Config
)
