set(PRJ_DIR ${CMAKE_CURRENT_LIST_DIR})

create_project(HSUSBH_USBH_HID_Mouse_Keyboard
    ${PRJ_DIR}/main.c
    ${PRJ_DIR}/keycode_xlate.c
)
target_link_libraries(HSUSBH_USBH_HID_Mouse_Keyboard usbhost_hid)
target_include_directories(HSUSBH_USBH_HID_Mouse_Keyboard PRIVATE
    ${PRJ_DIR}
    ${BSP_DIR}/Library/Device/Nuvoton/m3331/Config
)
