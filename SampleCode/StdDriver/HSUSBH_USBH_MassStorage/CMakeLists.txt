set(PRJ_DIR ${CMAKE_CURRENT_LIST_DIR})

create_project(HSUSBH_USBH_MassStorage
    ${PRJ_DIR}/main.c
    ${PRJ_DIR}/diskio.c
)
target_link_libraries(HSUSBH_USBH_MassStorage usbhost_msc fatfs_lib)
target_include_directories(HSUSBH_USBH_MassStorage PRIVATE
    ${PRJ_DIR}
    ${BSP_DIR}/Library/Device/Nuvoton/m3331/Config
)
