set(USBHOSTLIB_DIR ${CMAKE_CURRENT_LIST_DIR})

# ============================================================
# USB Host core library
# ============================================================
add_library(usbhost_core
    ${USBHOSTLIB_DIR}/src_core/ehci.c
    ${USBHOSTLIB_DIR}/src_core/ehci_iso.c
    ${USBHOSTLIB_DIR}/src_core/hub.c
    ${USBHOSTLIB_DIR}/src_core/mem_alloc.c
    ${USBHOSTLIB_DIR}/src_core/ohci.c
    ${USBHOSTLIB_DIR}/src_core/usb_core.c
)
target_include_directories(usbhost_core PUBLIC
    ${USBHOSTLIB_DIR}/inc
)

# ============================================================
# USB Host MSC (Mass Storage Class) library
# ============================================================
add_library(usbhost_msc
    ${USBHOSTLIB_DIR}/src_msc/msc_driver.c
    ${USBHOSTLIB_DIR}/src_msc/msc_xfer.c
)
target_include_directories(usbhost_msc PUBLIC
    ${USBHOSTLIB_DIR}/src_msc
)
target_link_libraries(usbhost_msc PUBLIC usbhost_core)

# ============================================================
# USB Host HID library
# ============================================================
add_library(usbhost_hid
    ${USBHOSTLIB_DIR}/src_hid/hid_core.c
    ${USBHOSTLIB_DIR}/src_hid/hid_driver.c
)
target_include_directories(usbhost_hid PUBLIC
    ${USBHOSTLIB_DIR}/src_hid
)
target_link_libraries(usbhost_hid PUBLIC usbhost_core)

# ============================================================
# USB Host UAC (Audio Class) library
# ============================================================
add_library(usbhost_uac
    ${USBHOSTLIB_DIR}/src_uac/uac_core.c
    ${USBHOSTLIB_DIR}/src_uac/uac_driver.c
    ${USBHOSTLIB_DIR}/src_uac/uac_parser.c
)
target_include_directories(usbhost_uac PUBLIC
    ${USBHOSTLIB_DIR}/src_uac
)
target_link_libraries(usbhost_uac PUBLIC usbhost_core)

# ============================================================
# USB Host CDC library
# ============================================================
add_library(usbhost_cdc
    ${USBHOSTLIB_DIR}/src_cdc/cdc_core.c
    ${USBHOSTLIB_DIR}/src_cdc/cdc_driver.c
    ${USBHOSTLIB_DIR}/src_cdc/cdc_parser.c
)
target_include_directories(usbhost_cdc PUBLIC
    ${USBHOSTLIB_DIR}/src_cdc
)
target_link_libraries(usbhost_cdc PUBLIC usbhost_core)
