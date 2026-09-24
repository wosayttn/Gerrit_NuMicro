# M2003 C/D/E/G Test-Sample Binary Artifact Evidence

## Scope

Every Git-tracked `.bin`, `.lib`, `.a`, `.exe`, and `.dll` in this BSP is
represented by its exact BSP-relative path and SHA-256. This file supports the
three first-party firmware artifacts declared in `sca_binary-artifacts.json`.

## FMC IAP LDROM Firmware

- Version basis: `SampleCode/StdDriver/FMC_IAP/FMC_IAP_LDROM.c` declares V1.00.
- License basis: the adjacent Nuvoton source and project inputs carry or inherit
  the repository Apache-2.0 distribution license.
- IAR artifact: `SampleCode/StdDriver/FMC_IAP/IAR/FMC_IAP_LDROM.bin`, 3273
  bytes, SHA-256
  `cc2107bcf601b556a78f2f0ecd1cc9c9b7107dbb7d225859387ebe9784b7a79c`.
- Keil artifact: `SampleCode/StdDriver/FMC_IAP/KEIL/FMC_IAP_LDROM.bin`, 3200
  bytes, SHA-256
  `8edcc9433a65785d9828311e17fb46f99942fe8acf186692fd16966efd42ffbb`.
- VSCode artifact: `SampleCode/StdDriver/FMC_IAP/VSCode/bin/fmc_ld_iap.bin`,
  2780 bytes, SHA-256
  `b45a1ccb8981686794e456f272d8fd702e74cbef0d15d09160651c70f2e6fd01`.

The IAR, Keil, and VSCode project files reference the corresponding LDROM
image. These checked-in files are treated as repository build outputs. No
compiler identity or reproducible-build claim is made because the repository
does not preserve sufficient build provenance for those assertions.
