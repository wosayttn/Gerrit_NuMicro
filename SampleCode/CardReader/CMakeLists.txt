create_project(USBD_CCID ${CMAKE_CURRENT_LIST_DIR}/USBD_CCID/main.c ${CMAKE_CURRENT_LIST_DIR}/USBD_CCID/Ccid_if.c ${CMAKE_CURRENT_LIST_DIR}/USBD_CCID/ccid.c ${CMAKE_CURRENT_LIST_DIR}/USBD_CCID/descriptors.c ${CMAKE_CURRENT_LIST_DIR}/USBD_CCID/sc_intf.c)

if(BSP_TOOLCHAIN STREQUAL "gcc")
    target_link_libraries(USBD_CCID PRIVATE
        "-Wl,--start-group"
        "${BSP_DIR}/Library/SmartcardLib/libsmartcard.a"
        standard_driver
        system_startup
        "-Wl,--end-group"
    )
else()
    target_link_libraries(USBD_CCID PRIVATE
        "${BSP_DIR}/Library/SmartcardLib/SmartCardLib_Keil.lib"
    )
endif()
