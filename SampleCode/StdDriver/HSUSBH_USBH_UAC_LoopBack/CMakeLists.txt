set(PRJ_DIR ${CMAKE_CURRENT_LIST_DIR})

create_project(HSUSBH_USBH_UAC_LoopBack
    ${PRJ_DIR}/main.c
    ${PRJ_DIR}/uac_loopback.c
)
target_link_libraries(HSUSBH_USBH_UAC_LoopBack usbhost_uac)
target_include_directories(HSUSBH_USBH_UAC_LoopBack PRIVATE
    ${PRJ_DIR}
    ${BSP_DIR}/Library/Device/Nuvoton/m3331/Config
)
