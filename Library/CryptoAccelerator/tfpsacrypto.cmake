set(NVT_MBEDTLS_SOURCE "${BSP_DIR}/ThirdParty/mbedtls")
set(NVT_PSA_FIRMWARE_OVERLAY
    "${CMAKE_CURRENT_LIST_DIR}/integration/firmware-overlay/tf-psa-crypto")
set(TFPSA_DIR "${NVT_MBEDTLS_SOURCE}/tf-psa-crypto")

add_library(nvt_crypto_configuration INTERFACE)
target_include_directories(nvt_crypto_configuration INTERFACE
    "${BSP_DIR}/Library/CryptoAccelerator/configs/target"
    "${BSP_DIR}/Library/CryptoAccelerator/src"
    "${NVT_PSA_FIRMWARE_OVERLAY}/include"
    "${NVT_PSA_FIRMWARE_OVERLAY}/core"
    "${NVT_MBEDTLS_SOURCE}/include"
    "${TFPSA_DIR}/include"
    "${TFPSA_DIR}/drivers/builtin/include"
    "${BSP_DIR}/Library/Device/Nuvoton/M2354/Include"
    "${BSP_DIR}/Library/CMSIS/Include"
    "${BSP_DIR}/Library/StdDriver/inc"
)
target_compile_definitions(nvt_crypto_configuration INTERFACE
    MBEDTLS_CONFIG_FILE="mbedtls_config.h"
    TF_PSA_CRYPTO_CONFIG_FILE="crypto_config.h"
    MBEDTLS_ALLOW_PRIVATE_ACCESS=
)

file(GLOB TFPSA_SRC CONFIGURE_DEPENDS
    "${TFPSA_DIR}/core/*.c"
    "${TFPSA_DIR}/platform/*.c"
    "${TFPSA_DIR}/utilities/*.c"
    "${TFPSA_DIR}/drivers/builtin/src/*.c"
)
list(APPEND TFPSA_SRC
    "${TFPSA_DIR}/extras/md.c"
    "${NVT_PSA_FIRMWARE_OVERLAY}/core/psa_crypto_driver_wrappers_no_static.c"
)
add_library(tfpsacrypto ${TFPSA_SRC})
target_link_libraries(tfpsacrypto PUBLIC nvt_crypto_configuration)
target_include_directories(tfpsacrypto PRIVATE
    "${NVT_PSA_FIRMWARE_OVERLAY}/core"
    "${TFPSA_DIR}/core"
    "${TFPSA_DIR}/dispatch"
    "${TFPSA_DIR}/drivers/builtin/src"
    "${TFPSA_DIR}/platform"
    "${TFPSA_DIR}/utilities"
    "${TFPSA_DIR}/extras"
)