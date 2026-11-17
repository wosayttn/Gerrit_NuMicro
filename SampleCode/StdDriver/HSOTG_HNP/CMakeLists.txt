set(PRJ_DIR ${CMAKE_CURRENT_LIST_DIR})

create_project(HSOTG_HNP
    ${PRJ_DIR}/main.c
    ${PRJ_DIR}/descriptors.c
    ${PRJ_DIR}/hid_mouse.c
)
target_link_libraries(HSOTG_HNP usbhost_hid)
target_include_directories(HSOTG_HNP PRIVATE
    ${PRJ_DIR}
    ${BSP_DIR}/Library/Device/Nuvoton/m3331/Config
)
