create_project(ISP_CAN ${CMAKE_CURRENT_LIST_DIR}/ISP_CAN/main.c)

create_project(ISP_I2C
    ${CMAKE_CURRENT_LIST_DIR}/ISP_I2C/main.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_I2C/fmc_user.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_I2C/isp_user.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_I2C/targetdev.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_I2C/i2c_transfer.c
)
create_project(ISP_RS485
    ${CMAKE_CURRENT_LIST_DIR}/ISP_RS485/main.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_RS485/fmc_user.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_RS485/isp_user.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_RS485/targetdev.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_RS485/uart_transfer.c
)
create_project(ISP_SPI
    ${CMAKE_CURRENT_LIST_DIR}/ISP_SPI/main.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_SPI/fmc_user.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_SPI/isp_user.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_SPI/targetdev.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_SPI/spi_transfer.c
)
create_project(ISP_UART
    ${CMAKE_CURRENT_LIST_DIR}/ISP_UART/main.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_UART/fmc_user.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_UART/isp_user.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_UART/targetdev.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_UART/uart_transfer.c
)

if(C_COMPILER_ID STREQUAL ARMCLANG)
    # if the project requires user's assemble, it only support ARMLANG
    enable_language(C ASM)
    create_project_no_startup(ISP_DFU_20
        ${CMAKE_CURRENT_LIST_DIR}/ISP_DFU_20/main.c
        ${CMAKE_CURRENT_LIST_DIR}/ISP_DFU_20/dfu_transfer.c
        ${CMAKE_CURRENT_LIST_DIR}/ISP_DFU_20/fmc_user.c
        ${CMAKE_CURRENT_LIST_DIR}/ISP_DFU_20/isp_user.c
        ${CMAKE_CURRENT_LIST_DIR}/ISP_DFU_20/descriptors.c
        ${CMAKE_CURRENT_LIST_DIR}/ISP_DFU_20/targetdev.c
        ${CMAKE_CURRENT_LIST_DIR}/ISP_DFU_20/startup_m3331_user.S
        ${BSP_DIR}/Library/Device/Nuvoton/m3331/Source/system_m3331.c
    )
    target_compile_options(ISP_DFU_20 PRIVATE -Wno-pragma-pack)

    create_project_no_startup(ISP_HID_20
        ${CMAKE_CURRENT_LIST_DIR}/ISP_HID_20/main.c
        ${CMAKE_CURRENT_LIST_DIR}/ISP_HID_20/fmc_user.c
        ${CMAKE_CURRENT_LIST_DIR}/ISP_HID_20/isp_user.c
        ${CMAKE_CURRENT_LIST_DIR}/ISP_HID_20/hsusbd_user.c
        ${CMAKE_CURRENT_LIST_DIR}/ISP_HID_20/descriptors.c
        ${CMAKE_CURRENT_LIST_DIR}/ISP_HID_20/targetdev.c
        ${CMAKE_CURRENT_LIST_DIR}/ISP_HID_20/hid_transfer.c
        ${CMAKE_CURRENT_LIST_DIR}/ISP_DFU_20/startup_m3331_user.S
        ${BSP_DIR}/Library/Device/Nuvoton/m3331/Source/system_m3331.c
    )
endif()
