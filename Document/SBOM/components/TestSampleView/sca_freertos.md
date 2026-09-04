# FreeRTOS-Kernel Third-Party Component Evidence

## Component Identity

- Name: `FreeRTOS-Kernel`
- Supplier: Amazon Web Services, Inc. (FreeRTOS project)
- Version: `10.5.1`
- SPDX license: `MIT`
- PURL: `pkg:generic/freertos-kernel@10.5.1`
- CPE: `cpe:2.3:o:amazon:freertos:10.5.1:*:*:*:*:*:*:*`
- Integration: Vendored source for the M2U51 FreeRTOS sample projects
- SBOM view: Test Sample SBOM
- Evidence path: `ThirdParty/FreeRTOS`

## Version Evidence

- `ThirdParty/FreeRTOS/Source/manifest.yml` identifies version `v10.5.1`.
- `ThirdParty/FreeRTOS/Source/sbom.spdx` identifies package
  `FreeRTOS-Kernel`, version `v10.5.1`, and distribution URL
  `https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/v10.5.1`.
- `ThirdParty/FreeRTOS/Source/include/FreeRTOS.h` identifies
  `FreeRTOS Kernel V10.5.1`.

## License and Supplier Evidence

- `ThirdParty/FreeRTOS/License/license.txt` states that the FreeRTOS Kernel
  is released under the MIT license and records Amazon.com copyright.
- `ThirdParty/FreeRTOS/Source/sbom.spdx` records `MIT` as the package
  concluded license and Amazon Web Services as the document creator.
- Vendored kernel source files carry `SPDX-License-Identifier: MIT`.

The vendored Demo/Common files carry the same MIT permission notice. No
different license marker was found in the included FreeRTOS subtree.

## Upstream and Import Traceability

The embedded upstream SPDX document identifies the upstream tag `v10.5.1`.
The vendored files do not record an exact upstream commit, so no commit SHA is
asserted.

## Included Content

- `ThirdParty/FreeRTOS/Demo`
- `ThirdParty/FreeRTOS/License`
- `ThirdParty/FreeRTOS/Source`
- `ThirdParty/FreeRTOS/README.md`
- `ThirdParty/FreeRTOS/links_to_doc_pages_for_the_demo_projects.url`

## Canonical Component Content Hash

- Algorithm: `sha256-path-nul-content-nul-v1`
- File count: `144`
- Path base: `ThirdParty/FreeRTOS`
- Exact inventory: `Document/SBOM/aggregate-inventory/test-freertos-kernel.json`
- SHA-256: `d396d0257baa029714b1507428b4aeaed19e144b821e8677b1528737b32ed83b`

The hash covers every file below `ThirdParty/FreeRTOS` at the scan source.
Files are ordered by path relative to that path base. For each file, the
SHA-256 input contains the UTF-8 relative path, a NUL byte, the raw file bytes,
and a terminating NUL byte.

## Scope Classification

FreeRTOS is required by the FreeRTOS sample projects but is not part of the
Product SBOM runtime scope unless a customer explicitly integrates it into a
product.
