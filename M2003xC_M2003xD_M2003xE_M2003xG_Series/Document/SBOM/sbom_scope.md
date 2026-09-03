# Introduction

The MCU BSP SBOM package consists of two CycloneDX SBOM files: a Product SBOM and a Test Sample SBOM.

The Product SBOM includes components that are included in, linked with, deployed to, or reasonably expected to be integrated into the final product firmware or software.

The Test Sample SBOM includes components used only for samples, demonstrations, validation, or testing and not included in the product runtime unless explicitly integrated by the user.

# Product SBOM

The Product SBOM is used for CRA compliance, product vulnerability management, and customer product integration risk assessment.

## Scope

The Product SBOM covers drivers, libraries, startup code, device support, and source components that are compiled, linked, flashed, deployed, or reasonably expected to be integrated into the product by customers.

## Included Paths

- Library/CMSIS/Core/Include
- Library/CMSIS/Driver
- Library/CMSIS/RTOS2
- Library/Device
- Library/StdDriver

## Excluded Paths

- Library/CMSIS/Core/Test
- Library/CMSIS/Documentation
- Document/
- SampleCode/

Library/CMSIS/Core/Test contains upstream CMSIS validation and test source and is excluded from the Product SBOM.

Library/CMSIS/Documentation is excluded from component scope. Specific documentation files may be referenced as supporting version evidence.

Document/ contains documentation and SBOM evidence and is not represented as a product software component.

SampleCode/ is represented separately in the Test Sample SBOM.

## Evidence

The Product SBOM scope is defined in:

- product-scope.yaml
- components/ProductView/

The CMSIS ProductView evidence represents the bundled CMSIS source subset as one CMSIS component covering CMSIS-Core, CMSIS-Driver, and CMSIS-RTOS2.

The M2003 Device source contains _syscalls.c and semihosting.h files with incomplete file-level license identification. Their SBOM treatment is documented in:

- license-review/syscalls-semihosting-license-review.md

# Test Sample SBOM

The Test Sample SBOM is used for transparent disclosure and internal or customer evaluation, but is labeled as a non-product runtime dependency.

## Scope

The Test Sample SBOM covers sample code, demonstration projects, validation code, test harnesses, and reviewed prebuilt files used by sample projects.

## Included Paths

- SampleCode/

## Excluded Paths

- Document/
- Library/

## Evidence

The Test Sample SBOM scope is defined in:

- test-sample-scope.yaml
- components/TestSampleView/

Test Sample components are not product runtime dependencies unless they are explicitly reused, redistributed, or integrated by the user.

# Release Handling

The final Product and Test Sample CycloneDX SBOM files, manifest, vulnerability scan results, checksum file, and release README are published as SVN release evidence.

The final release identifier is derived from the Git commit containing the approved SPDX corrections and SBOM evidence.
