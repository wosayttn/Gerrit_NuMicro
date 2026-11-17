set(PRJ mbedTLS_PSA_AEAD)
add_executable(${PRJ}
    main.c
    aead_test_main.c
)

link_crypto_firmware_target(${PRJ} crypto_accelerator_4x)

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

configure_firmware_target(${PRJ})