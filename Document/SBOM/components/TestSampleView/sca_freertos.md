# FreeRTOS-Kernel Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored component under `ThirdParty/FreeRTOS`.

## 1) Component Identity

- Component name (`name`): `FreeRTOS-Kernel`
- Component type (`type`): `library`
- Supplier / manufacturer: `Amazon.com, Inc.`
- Version: `11.1.0`
- License: `MIT`
- Evidence path: `ThirdParty/FreeRTOS`
- CPE (used for vulnerability matching): `cpe:2.3:o:amazon:freertos:11.1.0:*:*:*:*:*:*:*`

## 2) Evidence for Version and License

- Version evidence: ThirdParty/FreeRTOS/manifest.yml (version v11.1.0); ThirdParty/FreeRTOS/include/FreeRTOS.h; ThirdParty/FreeRTOS/sbom.spdx

## 3) Suggested BOM-Ref and purl

- `bom-ref`: `pkg:generic/freertos-kernel@11.1.0?source=vendored&path=ThirdParty/FreeRTOS`
- `purl`: `pkg:generic/freertos-kernel@11.1.0`

## 4) Compliance Notes

- Keep original upstream copyright/license notices.
- Machine-readable metadata: `TestSampleView/sca_freertos.json`.
