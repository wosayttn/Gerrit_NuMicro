# M2L31 Binary Artifact Evidence

## Classification

- SBOM View: Test Sample SBOM
- Scope: RMC sample firmware payloads and the HIDTransferTest host-build import library
- Distribution Status: Git-tracked and distributed in the BSP repository

## RMC Firmware Artifacts

The RMC sample source and build glue carry Apache-2.0 notices. The reviewed
projects identify these files as repository-built sample payloads or directly
generate/embed them. No separately distributed third-party runtime was
identified in the repository build relationships reviewed for this evidence.
The Apache-2.0 conclusion applies only to the exact paths and hashes below.

| Artifact | Size | SHA-256 | Relationship evidence |
|---|---:|---|---|
| `SampleCode/StdDriver/RMC_DualBankFwUpdate/NewFirmwareSample.bin` | 10540 | `7fd5aab425ec0a011c07e24da48e9d03969c3554038f9da3e2b4309aa29ec41f` | `RMC_DualBankFwUpdate/App` source and IAR/Keil/VSCode project files |
| `SampleCode/StdDriver/RMC_FwUpdateApplication/NewApp.bin` | 8560 | `e9c9e05d69f7285821d2933693e8fb1311709fa95d37a55fd8295aa0138d118b` | `RMC_FwUpdateApplication/App` source and IAR/Keil/VSCode project files |
| `SampleCode/StdDriver/RMC_IAP/GCC/LDROM_iap/LDROM_iap.bin` | 6560 | `3760d4f404b95d3586086637825cbcab586670a8af1ea9b131258695eb02cd38` | `.cproject` objcopy output and `GCC/ap_image.S` include |
| `SampleCode/StdDriver/RMC_IAP/IAR/Release/Exe/rmc_ld_iap.bin` | 3791 | `ea786e858d3e7e15e8235484316f3b98dc8d35072767e5b056d6156dca5c3d3a` | `rmc_ld_iap.ewp` output and `rmc_ap_main.ewp` image input |
| `SampleCode/StdDriver/RMC_IAP/KEIL/Obj/rmc_ld_iap.bin` | 3132 | `2b125703cc1236c42f6470a4abe0e0a8833b918decf4a4f4d653625040cd55d0` | `rmc_ld_iap.uvprojx` and `ap_image.s` `INCBIN` |
| `SampleCode/StdDriver/RMC_IAP/VSCode/bin/rmc_ld_iap.bin` | 5220 | `51aeb754c157f956b9c26e1c29e650d4cd91d271c55d832649fde803fe34c0f7` | LDROM output plus APROM assembly `.incbin` references |

A rebuilt artifact requires renewed review when its SHA-256, source closure,
build configuration, linked inputs, or toolchain runtime changes.

## Microsoft Windows HID Import Library

- Path: `Tool/HIDTransferTest/HIDTransferTest/hid.lib`
- Size: 12384 bytes
- SHA-256: `22bcccaed9b092f4ab2aa12e1471959ed1847296f350bb6d7390b0718fdb5ca4`
- Supplier: Microsoft Corporation
- License expression: `LicenseRef-Microsoft-Windows-SDK`

The archive contains the standard Windows HID import descriptors and imports,
including `__IMPORT_DESCRIPTOR_HID`, `HID_NULL_THUNK_DATA`, `HidD_*`, and
`HidP_*`. `HIDTransferTest.vcxproj` explicitly links `hid.lib` and
`setupapi.lib`. This identifies the distributed file as a Microsoft Windows
SDK import library used only to build the host-side test utility. The
applicable Microsoft SDK redistribution terms must remain associated with the
file; this evidence does not map those terms to an unrelated SPDX license.

## Machine-readable hash bindings

- SHA-256: `7fd5aab425ec0a011c07e24da48e9d03969c3554038f9da3e2b4309aa29ec41f`
- SHA-256: `e9c9e05d69f7285821d2933693e8fb1311709fa95d37a55fd8295aa0138d118b`
- SHA-256: `3760d4f404b95d3586086637825cbcab586670a8af1ea9b131258695eb02cd38`
- SHA-256: `ea786e858d3e7e15e8235484316f3b98dc8d35072767e5b056d6156dca5c3d3a`
- SHA-256: `2b125703cc1236c42f6470a4abe0e0a8833b918decf4a4f4d653625040cd55d0`
- SHA-256: `51aeb754c157f956b9c26e1c29e650d4cd91d271c55d832649fde803fe34c0f7`
- SHA-256: `22bcccaed9b092f4ab2aa12e1471959ed1847296f350bb6d7390b0718fdb5ca4`
