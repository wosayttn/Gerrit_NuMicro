# PSA symmetric cipher (AES) sample built against the Mbed TLS 4.x (_mbedtls) stack.
# Sources: local main() wrapper + the cipher/AEAD test + the shared PSA example
# functions (hash_a_message, encrypt/decrypt_with_symmetric_ciphers, ..._gcm) that
# live in CryptoAccelerator/src/psa_get_started_example.c.
# The tf-psa-crypto core library is provided by the root CMake build.
set(PRJ mbedTLS_PSA_Cipher_AES)
add_executable(${PRJ}
    main.c
    symmetric_ciphers_test_main.c
    ${BSP_DIR}/Library/CryptoAccelerator/src/psa_get_started_example.c
)

link_crypto_firmware_target(${PRJ} crypto_accelerator_4x_test)

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
