create_project(Blinky ${CMAKE_CURRENT_LIST_DIR}/Blinky/main.c)
create_project(MP3_Player ${CMAKE_CURRENT_LIST_DIR}/MP3_Player/main.c ${CMAKE_CURRENT_LIST_DIR}/MP3_Player/audio_codec.c ${CMAKE_CURRENT_LIST_DIR}/MP3_Player/diskio.c ${CMAKE_CURRENT_LIST_DIR}/MP3_Player/isr.c ${CMAKE_CURRENT_LIST_DIR}/MP3_Player/mp3.c ${CMAKE_CURRENT_LIST_DIR}/MP3_Player/mp3headerparser.c ${CMAKE_CURRENT_LIST_DIR}/MP3_Player/sdglue.c)
target_link_libraries(MP3_Player fatfs_lib libmad)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/LVGL_WidgetsDemo)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/emWin_GIFDecoder)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/NuvotonAudioGUI_Firmware)