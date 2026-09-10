set(HSOTG_DIR ${CMAKE_CURRENT_LIST_DIR})

# ============================================================
# HSOTG_Dual_Role_UMAS executable
# ============================================================
create_project(HSOTG_Dual_Role_UMAS
    ${HSOTG_DIR}/main.c
    ${HSOTG_DIR}/descriptors.c
    ${HSOTG_DIR}/diskio.c
    ${HSOTG_DIR}/massstorage.c
)
target_link_libraries(HSOTG_Dual_Role_UMAS usbhost_msc fatfs_lib)
target_include_directories(HSOTG_Dual_Role_UMAS PRIVATE
    ${HSOTG_DIR}
    ${BSP_DIR}/Library/Device/Nuvoton/m3331/Config
)
