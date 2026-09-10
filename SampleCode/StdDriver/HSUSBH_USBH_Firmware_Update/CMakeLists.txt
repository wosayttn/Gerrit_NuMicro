set(PRJ_DIR ${CMAKE_CURRENT_LIST_DIR})

create_project(HSUSBH_USBH_Firmware_Update
    ${PRJ_DIR}/main.c
    ${PRJ_DIR}/diskio.c
    ${PRJ_DIR}/usbh_update.c
)
target_link_libraries(HSUSBH_USBH_Firmware_Update usbhost_msc fatfs_lib)
target_include_directories(HSUSBH_USBH_Firmware_Update PRIVATE
    ${PRJ_DIR}
    ${BSP_DIR}/Library/Device/Nuvoton/m3331/Config
)
