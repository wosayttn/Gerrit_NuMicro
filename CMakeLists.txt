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

set(ARMCLANG_ROOT "" CACHE PATH "ARM Compiler 6 installation directory")
find_program(ARM_CC NAMES armclang HINTS "${ARMCLANG_ROOT}/bin" REQUIRED)
find_program(ARM_CXX NAMES armclang HINTS "${ARMCLANG_ROOT}/bin" REQUIRED)
find_program(ARM_AR NAMES armar HINTS "${ARMCLANG_ROOT}/bin" REQUIRED)
find_program(ARM_LINK NAMES armlink HINTS "${ARMCLANG_ROOT}/bin" REQUIRED)
find_program(ARM_FROMELF NAMES fromelf HINTS "${ARMCLANG_ROOT}/bin" REQUIRED)
get_filename_component(TOOLCHAIN_BIN "${ARM_CC}" DIRECTORY)
get_filename_component(TOOLCHAIN_PATH "${TOOLCHAIN_BIN}" DIRECTORY)

set(CMAKE_C_COMPILER ${ARM_CC} CACHE STRING "")
set(CMAKE_CXX_COMPILER ${ARM_CXX} CACHE STRING "")
set(CMAKE_ASM_COMPILER ${ARM_CC} CACHE FILEPATH "" FORCE)
set(CMAKE_LINKER ${ARM_LINK} CACHE STRING "")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PATH})

set(LINK_MCU_CORE "Cortex-M23")

set(C_FLAGS "--target=arm-arm-none-eabi -mcpu=${LINK_MCU_CORE} -c -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -D__MICROLIB -gdwarf-3 -O2 -fno-function-sections -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality -D__MICROLIB")
set(CMAKE_C_FLAGS "-xc -std=c99  ${C_FLAGS}")
set(CMAKE_CXX_FLAGS ${C_FLAGS})
set(CMAKE_ASM_FLAGS "--target=arm-arm-none-eabi -mcpu=cortex-m23 -mthumb -x assembler-with-cpp -D__MICROLIB=1")
set(CMAKE_EXECUTABLE_SUFFIX ".axf")

project(m2354bsp)

function(create_project PRJ)
    add_executable(${PRJ} ${ARGN})
    target_link_libraries(${PRJ} system_startup standard_driver)

    target_link_options(${PRJ}
        PUBLIC
        --cpu=Cortex-M23
        --library_type=microlib
        --ro-base 0x00000000
        --rw-base 0x20000000
        --entry Reset_Handler
        --first __Vectors
        --strict
        --map
        --datacompressor=off
        --info=inline
        --entry Reset_Handler
        --summary_stderr
        --info totals
        --map
        --load_addr_map_info
        --xref
        --callgraph
        --symbols
    )

    add_custom_command(TARGET ${PRJ} POST_BUILD
        COMMAND ${ARM_FROMELF} --bin --output ${PRJ}.bin ${PRJ}.elf
        VERBATIM
    )
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
