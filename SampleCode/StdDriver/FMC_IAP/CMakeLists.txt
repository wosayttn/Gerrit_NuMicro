enable_language(ASM)

set(LDROM_PRJ FMC_IAP_LDROM_iap)
set(LDROM_BIN ${CMAKE_CURRENT_BINARY_DIR}/fmc_ld_code.bin)

# LDROM image: linked with the LDROM scatter/linker script
add_executable(${LDROM_PRJ} ${CMAKE_CURRENT_LIST_DIR}/LDROM_iap.c ${RETARGET} ${STARTUP})
target_link_libraries(${LDROM_PRJ} standard_driver)

if(C_COMPILER_ID STREQUAL ARMCLANG)
    target_link_options(${LDROM_PRJ}
        PUBLIC
        --cpu=Cortex-M33
        --library_type=microlib
        --lto
        --scatter ${BSP_DIR}/Library/Device/Nuvoton/m3331/Config/m3331_ac6_ldrom.sct
        --entry Reset_Handler
        --map
        --datacompressor=off
        --info=inline
        --summary_stderr
        --info totals
        --load_addr_map_info
        --xref
        --callgraph
        --symbols
    )

    add_custom_command(TARGET ${LDROM_PRJ} POST_BUILD
        BYPRODUCTS ${LDROM_BIN}
        COMMAND ${ARM_FROMELF} --bin --output ${LDROM_BIN} $<TARGET_FILE:${LDROM_PRJ}>
        VERBATIM
    )
endif()

if(C_COMPILER_ID STREQUAL GCC)
    target_link_options(${LDROM_PRJ}
        PUBLIC
        -T ${BSP_DIR}/Library/Device/Nuvoton/m3331/Config/m3331_gcc_ldrom.ld
        -Xlinker
        --gc-sections
        -Wl,-Map=${LDROM_PRJ}.map
        -gdwarf-3
        -flto=auto
    )

    add_custom_command(TARGET ${LDROM_PRJ} POST_BUILD
        BYPRODUCTS ${LDROM_BIN}
        COMMAND ${ARM_OBJCOPY} -O binary $<TARGET_FILE:${LDROM_PRJ}> ${LDROM_BIN}
        VERBATIM
    )
endif()

# APROM image: embeds the LDROM binary through ap_image.S
create_project(FMC_IAP
    ${CMAKE_CURRENT_LIST_DIR}/APROM_main.c
    ${CMAKE_CURRENT_LIST_DIR}/VSCode/FMC_IAP_APROM/ap_image.S
)
target_compile_definitions(FMC_IAP PRIVATE BINFILE="${LDROM_BIN}")
set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/VSCode/FMC_IAP_APROM/ap_image.S
    PROPERTIES OBJECT_DEPENDS ${LDROM_BIN})
add_dependencies(FMC_IAP ${LDROM_PRJ})