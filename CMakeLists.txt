cmake_minimum_required(VERSION 3.21)
cmake_policy(SET CMP0123 NEW)

# Create a symbolic link from ${base_name} in the binary directory
# to the corresponding path in the source directory.
function(link_to_source base_name)
    # Get OS dependent path to use in `execute_process`
    if(CMAKE_HOST_WIN32)
        # mklink is an internal command of cmd.exe it can only work with \
        string(REPLACE "/" "\\" link "${CMAKE_CURRENT_BINARY_DIR}/${base_name}")
        string(REPLACE "/" "\\" target "${CMAKE_CURRENT_SOURCE_DIR}/${base_name}")
    else()
        set(link "${CMAKE_CURRENT_BINARY_DIR}/${base_name}")
        set(target "${CMAKE_CURRENT_SOURCE_DIR}/${base_name}")
    endif()

    if(NOT EXISTS ${link})
        if(CMAKE_HOST_UNIX)
            set(command ln -s ${target} ${link})
        else()
            if(IS_DIRECTORY ${target})
                set(command cmd.exe /c mklink /j ${link} ${target})
            else()
                set(command cmd.exe /c mklink /h ${link} ${target})
            endif()
        endif()

        execute_process(COMMAND ${command}
            RESULT_VARIABLE result
            ERROR_VARIABLE output)

        if(NOT ${result} EQUAL 0)
            message(FATAL_ERROR "Could not create symbolic link for: ${target} --> ${output}")
        endif()
    endif()
endfunction(link_to_source)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION "5")
set(CMAKE_SYSTEM_PROCESSOR ARM)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(BSP_TOOLCHAIN "armclang" CACHE STRING "Firmware toolchain (armclang or gcc)")
set_property(CACHE BSP_TOOLCHAIN PROPERTY STRINGS armclang gcc)
set(LINK_MCU_CORE "Cortex-M23")

if(BSP_TOOLCHAIN STREQUAL "armclang")
    set(ARMCLANG_ROOT "" CACHE PATH "ARM Compiler 6 installation directory")
    find_program(ARM_CC NAMES armclang HINTS "${ARMCLANG_ROOT}/bin" REQUIRED)
    find_program(ARM_CXX NAMES armclang HINTS "${ARMCLANG_ROOT}/bin" REQUIRED)
    find_program(ARM_LINK NAMES armlink HINTS "${ARMCLANG_ROOT}/bin" REQUIRED)
    find_program(ARM_FROMELF NAMES fromelf HINTS "${ARMCLANG_ROOT}/bin" REQUIRED)
    get_filename_component(TOOLCHAIN_BIN "${ARM_CC}" DIRECTORY)
    get_filename_component(TOOLCHAIN_PATH "${TOOLCHAIN_BIN}" DIRECTORY)

    set(CMAKE_C_COMPILER "${ARM_CC}" CACHE STRING "")
    set(CMAKE_CXX_COMPILER "${ARM_CXX}" CACHE STRING "")
    set(CMAKE_ASM_COMPILER "${ARM_CC}" CACHE FILEPATH "" FORCE)
    set(CMAKE_LINKER "${ARM_LINK}" CACHE STRING "")

    set(ARMCLANG_FLAGS "--target=arm-arm-none-eabi -mcpu=${LINK_MCU_CORE} -c -fno-rtti -funsigned-char -D__MICROLIB -gdwarf-3 -O2 -fno-function-sections -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality")
    set(CMAKE_C_FLAGS "-xc -std=c99 ${ARMCLANG_FLAGS}")
    set(CMAKE_CXX_FLAGS "${ARMCLANG_FLAGS}")
    set(CMAKE_ASM_FLAGS "--target=arm-arm-none-eabi -mcpu=cortex-m23 -mthumb -x assembler-with-cpp -D__MICROLIB=1")
    set(CMAKE_EXECUTABLE_SUFFIX ".axf")
elseif(BSP_TOOLCHAIN STREQUAL "gcc")
    set(ARM_GCC_ROOT "" CACHE PATH "GNU Arm Embedded Toolchain installation directory")
    find_program(GCC_CC NAMES arm-none-eabi-gcc HINTS "${ARM_GCC_ROOT}/bin" REQUIRED)
    find_program(GCC_CXX NAMES arm-none-eabi-g++ HINTS "${ARM_GCC_ROOT}/bin" REQUIRED)
    find_program(GCC_AR NAMES arm-none-eabi-ar HINTS "${ARM_GCC_ROOT}/bin" REQUIRED)
    find_program(GCC_RANLIB NAMES arm-none-eabi-ranlib HINTS "${ARM_GCC_ROOT}/bin" REQUIRED)
    find_program(GCC_LINK NAMES arm-none-eabi-ld HINTS "${ARM_GCC_ROOT}/bin" REQUIRED)
    find_program(GCC_OBJCOPY NAMES arm-none-eabi-objcopy HINTS "${ARM_GCC_ROOT}/bin" REQUIRED)
    get_filename_component(TOOLCHAIN_BIN "${GCC_CC}" DIRECTORY)
    execute_process(
        COMMAND "${GCC_CC}" -print-sysroot
        OUTPUT_VARIABLE TOOLCHAIN_PATH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        COMMAND_ERROR_IS_FATAL ANY
    )

    set(CMAKE_C_COMPILER "${GCC_CC}" CACHE STRING "")
    set(CMAKE_CXX_COMPILER "${GCC_CXX}" CACHE STRING "")
    set(CMAKE_ASM_COMPILER "${GCC_CC}" CACHE FILEPATH "" FORCE)
    set(CMAKE_AR "${GCC_AR}" CACHE FILEPATH "" FORCE)
    set(CMAKE_RANLIB "${GCC_RANLIB}" CACHE FILEPATH "" FORCE)
    set(CMAKE_LINKER "${GCC_LINK}" CACHE FILEPATH "" FORCE)

    set(GCC_CPU_FLAGS "-mcpu=cortex-m23 -mthumb")
    set(CMAKE_C_FLAGS "-std=c99 ${GCC_CPU_FLAGS} -funsigned-char -fshort-enums -ffunction-sections -fdata-sections -O2 -g3")
    set(CMAKE_CXX_FLAGS "${GCC_CPU_FLAGS} -fno-rtti -fno-exceptions -ffunction-sections -fdata-sections -O2 -g3")
    set(CMAKE_ASM_FLAGS "${GCC_CPU_FLAGS}")
    set(CMAKE_EXECUTABLE_SUFFIX ".elf")
else()
    message(FATAL_ERROR "Unsupported BSP_TOOLCHAIN '${BSP_TOOLCHAIN}'; choose armclang or gcc")
endif()

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH "${TOOLCHAIN_PATH}")

project(m2354bsp)

function(create_project PRJ)
    add_executable(${PRJ} ${ARGN})
    target_link_libraries(${PRJ} PRIVATE system_startup standard_driver)

    configure_firmware_target(${PRJ})
endfunction()

if(BSP_TOOLCHAIN STREQUAL "gcc")
    set(CMAKE_EXECUTABLE_SUFFIX ".elf")
endif()

function(configure_firmware_target PRJ)
    if(BSP_TOOLCHAIN STREQUAL "armclang")
        target_link_options(${PRJ} PRIVATE
            --cpu=${LINK_MCU_CORE}
            --library_type=microlib
            --ro-base 0x00000000
            --rw-base 0x20000000
            --entry Reset_Handler
            --first __Vectors
            --strict
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
        add_custom_command(TARGET ${PRJ} POST_BUILD
            COMMAND "${ARM_FROMELF}" --bin --output "$<TARGET_FILE_DIR:${PRJ}>/${PRJ}.bin" "$<TARGET_FILE:${PRJ}>"
            VERBATIM
        )
    else()
        set(GCC_LINKER_SCRIPT "${BSP_DIR}/Library/Device/Nuvoton/M2354/Source/GCC/gcc_arm.ld")
        target_link_options(${PRJ} PRIVATE
            -mcpu=cortex-m23
            -mthumb
            -T "${GCC_LINKER_SCRIPT}"
            -Wl,--gc-sections
            "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PRJ}.map"
        )
        add_custom_command(TARGET ${PRJ} POST_BUILD
            COMMAND "${GCC_OBJCOPY}" -O binary "$<TARGET_FILE:${PRJ}>" "$<TARGET_FILE_DIR:${PRJ}>/${PRJ}.bin"
            VERBATIM
        )
    endif()
endfunction()

function(link_crypto_firmware_target PRJ CRYPTO_DRIVER)
    target_include_directories(${PRJ} BEFORE PRIVATE
        "${NVT_PSA_FIRMWARE_OVERLAY}/include"
        "${NVT_PSA_FIRMWARE_OVERLAY}/core"
    )

    if(BSP_TOOLCHAIN STREQUAL "gcc")
        target_link_libraries(${PRJ} PRIVATE
            "-Wl,--start-group"
            tfpsacrypto
            ${CRYPTO_DRIVER}
            standard_driver
            system_startup
            "-Wl,--end-group"
        )
    else()
        target_link_libraries(${PRJ} PRIVATE
            system_startup
            standard_driver
            ${CRYPTO_DRIVER}
            tfpsacrypto
        )
    endif()
endfunction()

set(BSP_DIR ${CMAKE_CURRENT_LIST_DIR})

include_directories(${TOOLCHAIN_PATH}/include)
include_directories(${BSP_DIR}/Library/CMSIS/Include)
include_directories(${BSP_DIR}/Library/Device/Nuvoton/M2354/Include)
include_directories(${BSP_DIR}/Library/StdDriver/inc)
include_directories(${BSP_DIR}/Library/SmartcardLib/Include)

# Build library
add_subdirectory(${BSP_DIR}/Library/StdDriver/src)
add_subdirectory(${BSP_DIR}/Library/Device/Nuvoton/M2354/Source)
include(${BSP_DIR}/Library/CryptoAccelerator/tfpsacrypto.cmake)
add_subdirectory(${BSP_DIR}/Library/CryptoAccelerator)

# Build sample code
add_subdirectory(${BSP_DIR}/SampleCode/AttackDetection)
add_subdirectory(${BSP_DIR}/SampleCode/CardReader)
add_subdirectory(${BSP_DIR}/SampleCode/CortexM23)
add_subdirectory(${BSP_DIR}/SampleCode/Crypto)

# add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/CRYPTO_ECC_SM2)
# add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/CRYPTO_SHA_SM3)
# add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/CRYPTO_AES_GCM)
# add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/CRYPTO_AES_CCM)
# add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/ACMP_CompareDAC)
# add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/ACMP_CompareVBG)
# add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/CRYPTO_ECC_Demo)
