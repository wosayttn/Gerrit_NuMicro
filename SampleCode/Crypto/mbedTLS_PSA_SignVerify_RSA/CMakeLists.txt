# PSA sample built against the Mbed TLS 4.x (_mbedtls) stack.
# The tf-psa-crypto core library is provided by the root CMake build.
set(PRJ mbedTLS_PSA_SignVerify_RSA)
add_executable(${PRJ} main.c)

target_link_libraries(${PRJ}
    system_startup
    standard_driver
    crypto_accelerator_4x
    tfpsacrypto
)

target_include_directories(${PRJ}
    PUBLIC
    ${BSP_DIR}/Library/CryptoAccelerator/src
    ${NVT_MBEDTLS_SOURCE}/include
    ${BSP_DIR}/Library/CryptoAccelerator/configs/target
    ${TFPSA_DIR}/include
    ${TFPSA_DIR}/core
    ${TFPSA_DIR}/dispatch
    ${TFPSA_DIR}/drivers/builtin/include
    ${TFPSA_DIR}/drivers/builtin/src
)

target_compile_definitions(${PRJ}
    PUBLIC
    MBEDTLS_CONFIG_FILE="mbedtls_config.h"
    TF_PSA_CRYPTO_CONFIG_FILE="crypto_config.h"
)

target_link_options(${PRJ}
   PUBLIC
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
    --entry Reset_Handler
    --summary_stderr
    --info totals
    --map
    --load_addr_map_info
    --xref
    --callgraph
    --symbols
)

add_custom_command(TARGET ${PRJ}
    POST_BUILD
   COMMAND ${ARM_FROMELF} --bin --output ${PRJ}.bin ${PRJ}.elf
   VERBATIM
)
