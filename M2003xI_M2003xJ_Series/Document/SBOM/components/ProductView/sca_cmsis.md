# SCA Component Evidence: CMSIS

## Classification

- View: ProductView
- Component: CMSIS
- Supplier: Arm Limited
- Component version: 6.1.0
- Integration: Vendored source
- License: Apache-2.0
- PURL: `pkg:github/ARM-software/CMSIS_6@6.1.0`

## Included source paths

- `Library/CMSIS/Core/Include`
- `Library/CMSIS/Driver`
- `Library/CMSIS/RTOS2`

## Excluded source paths

- `Library/CMSIS/Core/Test`
- `Library/CMSIS/Documentation`

The excluded paths remain in the repository but are not part of the Product View runtime and development scope.

## Version evidence

### CMSIS-Core(M)

- Resolved version: 6.1.0
- Evidence path: `Library/CMSIS/Core/Include/cmsis_version.h`
- Evidence:
  - `__CM_CMSIS_VERSION_MAIN = 6`
  - `__CM_CMSIS_VERSION_SUB = 1`

### CMSIS-RTOS2 API

- Resolved version: 2.3.0
- Evidence path: `Library/CMSIS/RTOS2/Include/cmsis_os2.h`
- Evidence:
  - Revision `V2.3.0`
  - Current history entry `Version 2.3.0`
  - Project `CMSIS-RTOS2 API`

### CMSIS-Driver release

- Resolved release: 2.10.0
- Evidence path: `Library/CMSIS/Documentation/html/General/revision_history.html`
- Evidence: CMSIS release 6.1.0 identifies `CMSIS-Driver: 2.10.0`.

### Driver Common definitions

- Resolved version: 2.0
- Evidence path: `Library/CMSIS/Driver/Include/Driver_Common.h`
- Evidence:
  - Revision `V2.0`
  - Current history entry `Version 2.0`

## Driver interface API versions

- `ARM_CAN_API_VERSION`: 1.3 (`Library/CMSIS/Driver/Include/Driver_CAN.h`)
- `ARM_ETH_MAC_API_VERSION`: 2.2 (`Library/CMSIS/Driver/Include/Driver_ETH_MAC.h`)
- `ARM_ETH_PHY_API_VERSION`: 2.2 (`Library/CMSIS/Driver/Include/Driver_ETH_PHY.h`)
- `ARM_FLASH_API_VERSION`: 2.3 (`Library/CMSIS/Driver/Include/Driver_Flash.h`)
- `ARM_I2C_API_VERSION`: 2.4 (`Library/CMSIS/Driver/Include/Driver_I2C.h`)
- `ARM_MCI_API_VERSION`: 2.4 (`Library/CMSIS/Driver/Include/Driver_MCI.h`)
- `ARM_NAND_API_VERSION`: 2.4 (`Library/CMSIS/Driver/Include/Driver_NAND.h`)
- `ARM_SAI_API_VERSION`: 1.2 (`Library/CMSIS/Driver/Include/Driver_SAI.h`)
- `ARM_SPI_API_VERSION`: 2.3 (`Library/CMSIS/Driver/Include/Driver_SPI.h`)
- `ARM_STORAGE_API_VERSION`: 1.2 (`Library/CMSIS/Driver/Include/Driver_Storage.h`)
- `ARM_USART_API_VERSION`: 2.4 (`Library/CMSIS/Driver/Include/Driver_USART.h`)
- `ARM_USBD_API_VERSION`: 2.3 (`Library/CMSIS/Driver/Include/Driver_USBD.h`)
- `ARM_USBH_API_VERSION`: 2.4 (`Library/CMSIS/Driver/Include/Driver_USBH.h`)
- `ARM_WIFI_API_VERSION`: 1.1 (`Library/CMSIS/Driver/Include/Driver_WiFi.h`)

The interface API versions are recorded independently. No single interface API version is used as a substitute for the CMSIS-Driver release or Driver Common definitions version.

## License evidence

- `Library/CMSIS/Core/Include/cmsis_version.h`: Apache-2.0, 1936 bytes, SHA-256 `3B9195C5373A2A4DBFD968B083B3D7F5F446F9EC2E126F75928AC1FF4356B91E`
- `Library/CMSIS/Driver/Include/Driver_Common.h`: Apache-2.0, 2248 bytes, SHA-256 `EB32E5589B32A3160941C60B638A474C0AE23F9C63EB04143AA31908A4A70A4B`
- `Library/CMSIS/RTOS2/Include/cmsis_os2.h`: Apache-2.0, 42623 bytes, SHA-256 `0DE5E6FF219EE13BCD1B1CA2B6446D68CB56B6A11E16494E9577BB2FFDD52A70`

Each representative file contains exactly one `SPDX-License-Identifier: Apache-2.0` declaration.

## Conclusion

CMSIS is accepted as a third-party vendored source component in the Product View under Apache-2.0. The component evidence binds the summarized version and license conclusions to repository-relative source paths and file hashes.
