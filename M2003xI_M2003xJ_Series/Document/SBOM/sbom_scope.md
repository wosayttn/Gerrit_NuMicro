# SBOM Scope

## BSP

- Name: M2003xI_M2003xJ_Series
- Device support root: Library/Device/Nuvoton/M2003J
- Scope model: Product View and Test Sample View

## Product View

The Product View contains source required for BSP integration and product development.

### Included paths

- Library/CMSIS/Core/Include
- Library/CMSIS/Driver
- Library/CMSIS/RTOS2
- Library/Device/Nuvoton/M2003J
- Library/StdDriver

### Excluded paths

- Library/CMSIS/Core/Test
- Library/CMSIS/Documentation
- SampleCode

CMSIS is represented as a single aggregated third-party component. The vendored subset includes CMSIS-Core headers, CMSIS-Driver, and CMSIS-RTOS2. CMSIS test and documentation content is retained in the repository but excluded from the Product View.

The Product View includes _syscalls.c and semihosting.h. Neither file contains an SPDX identifier; their exact byte content is tracked by license-review/syscalls-semihosting-license-review.md.

## Test Sample View

The Test Sample View includes:

- SampleCode

The complete SampleCode tree is retained in scope. Git-tracked firmware binaries and IDE build outputs within this tree are treated as Nuvoton sample artifacts or supporting build artifacts, not automatically as third-party software components.

The current sample artifact inventory includes:

- 10 .bin files
- 4 .axf files
- 32 .o files
- 1 .out file

All 47 identified binary or object artifacts are Git-tracked. Forty are located in ISP obj directories. Seven are outside those build-output directories and are treated as intentional sample artifact candidates.

## Component evidence

Product View third-party component evidence:

- components/ProductView/sca_cmsis.json
- components/ProductView/sca_cmsis.md

No separate Test Sample View SCA component evidence is created solely for Nuvoton sample binaries or tracked build outputs.

## Release evidence

Git stores scope, component evidence, and license review records.

The final Product and Test Sample CycloneDX SBOMs, manifest, vulnerability scan results, README, and SHA256SUMS.txt are generated after the final Git commit and stored in the corresponding SVN release directory.
