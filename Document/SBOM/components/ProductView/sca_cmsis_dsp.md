# CMSIS-DSP Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored component under `Library/CMSIS-DSP`.

## 1) Component Identity

- Component name (`name`): `CMSIS-DSP`
- Component type (`type`): `library`
- Supplier / manufacturer: `Arm Limited`
- Version: `1.10.0`
- License: `Apache-2.0`
- Evidence path: `Library/CMSIS-DSP`

## 2) Evidence for Version and License

- Version evidence: Library/CMSIS-DSP/Include/arm_math.h (@version V1.10.0; SPDX-License-Identifier: Apache-2.0); Library/CMSIS-DSP/LICENSE

## 3) Suggested BOM-Ref and purl

- `bom-ref`: `pkg:generic/cmsis-dsp@1.10.0?source=vendored&path=Library/CMSIS-DSP`
- `purl`: `pkg:generic/cmsis-dsp@1.10.0`

## 4) Compliance Notes

- Keep original upstream copyright/license notices.
- Machine-readable metadata: `ProductView/sca_cmsis_dsp.json`.
