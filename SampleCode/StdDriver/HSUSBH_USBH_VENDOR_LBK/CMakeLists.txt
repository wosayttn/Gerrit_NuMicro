set(PRJ_DIR ${CMAKE_CURRENT_LIST_DIR})

create_project(HSUSBH_USBH_VENDOR_LBK
    ${PRJ_DIR}/main.c
    ${PRJ_DIR}/lbk_driver.c
)
target_link_libraries(HSUSBH_USBH_VENDOR_LBK usbhost_core)
target_include_directories(HSUSBH_USBH_VENDOR_LBK PRIVATE
    ${PRJ_DIR}
    ${BSP_DIR}/Library/Device/Nuvoton/m3331/Config
)
