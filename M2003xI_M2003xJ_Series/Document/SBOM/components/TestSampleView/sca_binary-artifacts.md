# M2003 I/J Test-Sample Binary Artifact Evidence

## Scope

Every Git-tracked `.bin`, `.lib`, `.a`, `.exe`, and `.dll` in this BSP is
represented by its exact BSP-relative path and SHA-256. This file supports the
five first-party firmware artifacts declared in `sca_binary-artifacts.json`.

## M2003 I/J Dual-Bank Update Firmware

- Version basis: the adjacent App and Loader `main.c` files declare V1.00.
- License basis: those Nuvoton sources carry SPDX-License-Identifier:
  Apache-2.0.
- Artifact: `SampleCode/StdDriver/FMC_DualBankFwUpdate/NewFirmwareSample.bin`,
  6092 bytes, SHA-256
  `28e4b052d2828b6fc3b57ef70acb8c1d3f8419c50387c305ef763d74549ece84`.
- Relationship evidence: the App and Loader IAR, Keil, and VSCode projects
  define the firmware build context.

## M2003 I/J Firmware Update Application

- Version basis: the adjacent App and Loader `main.c` files declare V1.00.
- License basis: those Nuvoton sources carry SPDX-License-Identifier:
  Apache-2.0.
- Artifact: `SampleCode/StdDriver/FMC_FwUpdateApplication/NewApp.bin`, 5600
  bytes, SHA-256
  `e079b9f07af168810b603e579886c3f1b7afa10539aa0c0fec57d4e0e70d961b`.
- Relationship evidence: the App and Loader IAR, Keil, and VSCode projects
  define the firmware build context.

## M2003 I/J FMC IAP LDROM Firmware

- Version basis: `SampleCode/StdDriver/FMC_IAP/FMC_IAP_LDROM.c` declares V1.00.
- License basis: the adjacent Nuvoton source and project inputs carry or inherit
  the repository Apache-2.0 distribution license.
- IAR artifact: `SampleCode/StdDriver/FMC_IAP/IAR/FMC_IAP_LDROM.bin`, 2728
  bytes, SHA-256
  `7e58b7c8e7891d1a2d78b45a0be7bb099e3447cb8624ae8280c385eedb9059ce`.
- Keil artifact: `SampleCode/StdDriver/FMC_IAP/Keil/FMC_IAP_LDROM.bin`, 3324
  bytes, SHA-256
  `e4962a24e1fc23b33568473c7e854d546b63ea745738fd304228f652aa3cd325`.
- VSCode artifact: `SampleCode/StdDriver/FMC_IAP/VSCode/bin/fmc_ld_iap.bin`,
  3376 bytes, SHA-256
  `736961095c4e48aea65ef05a70380a88eefe6fefd7efb29f1abd577d1a386251`.

These checked-in files are treated as repository build outputs. No compiler
identity or reproducible-build claim is made because the repository does not
preserve sufficient build provenance for those assertions.
