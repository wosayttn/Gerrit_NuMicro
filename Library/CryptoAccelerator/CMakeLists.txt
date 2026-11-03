# Mbed TLS 4.x Nuvoton PSA transparent drivers (moved from tf-psa-crypto/drivers/nvt-drivers).
# The legacy *_alt.c files in this folder are inert under 4.x (no _ALT mechanism) and are not built.
set(CRPT_SRC
    src/nvt_crpt.c
    src/nvt_psa_sha.c
    src/nvt_psa_aes.c
    src/nvt_psa_gcm.c
    src/nvt_psa_ccm.c
    src/nvt_hash_driver_entrypoints.c
    src/nvt_cipher_driver_entrypoints.c
    src/nvt_aead_driver_entrypoints.c
    src/nvt_psa_ecc.c
    src/nvt_psa_rsa.c
    src/nvt_ecc_driver_entrypoints.c
    src/nvt_rsa_driver_entrypoints.c
)

function(configure_nvt_crypto_accelerator target)
    target_link_libraries(${target} PUBLIC nvt_crypto_configuration)
    target_include_directories(${target}
      PUBLIC
        ${TOOLCHAIN_PATH}/include
        ${BSP_DIR}/Library/CMSIS/Include
        ${BSP_DIR}/Library/StdDriver/inc
        ${BSP_DIR}/Library/Device/Nuvoton/M2354/Include
        ${BSP_DIR}/Library/CryptoAccelerator/src
        ${NVT_MBEDTLS_SOURCE}/include
        ${TFPSA_DIR}/include
        ${TFPSA_DIR}/core
        ${TFPSA_DIR}/drivers/builtin/include
        ${TFPSA_DIR}/drivers/builtin/src
    )
endfunction()

add_library(crypto_accelerator_4x ${CRPT_SRC})
configure_nvt_crypto_accelerator(crypto_accelerator_4x)

add_library(crypto_accelerator_4x_test ${CRPT_SRC})
configure_nvt_crypto_accelerator(crypto_accelerator_4x_test)
target_compile_definitions(crypto_accelerator_4x_test
    PUBLIC MBEDTLS_PSA_NVT_TEST_HOOKS)




