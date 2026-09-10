create_project(FwApp ${CMAKE_CURRENT_LIST_DIR}/App/main.c)
target_include_directories(FwApp PUBLIC ${BSP_DIR}/Library/NuMaker/xmodem)
target_link_libraries(FwApp xmodem_lib)

create_project(FwBackupApp ${CMAKE_CURRENT_LIST_DIR}/BackupApp/main.c)
target_include_directories(FwBackupApp PUBLIC ${BSP_DIR}/Library/NuMaker/xmodem)
target_link_libraries(FwBackupApp xmodem_lib)

create_project(FwLoader ${CMAKE_CURRENT_LIST_DIR}/Loader/main.c)
target_include_directories(FwLoader PUBLIC ${BSP_DIR}/Library/NuMaker/xmodem)
target_link_libraries(FwLoader xmodem_lib)