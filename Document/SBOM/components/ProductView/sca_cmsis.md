# CMSIS Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored component under `Library/CMSIS`.

## 1) Component Identity

- Component name (`name`): `CMSIS`
- Component type (`type`): `library`
- Supplier / manufacturer: `Arm Limited`
- Version: `6.1.0`
- License: `Apache-2.0`
- Evidence path: `Library/CMSIS`

## 2) Evidence for Version and License

- Version evidence: Library/CMSIS/Core/Include/cmsis_version.h (__CM_CMSIS_VERSION_MAIN 6, __CM_CMSIS_VERSION_SUB 1)

## 3) Suggested BOM-Ref and purl

- `bom-ref`: `pkg:generic/cmsis@6.1.0?source=vendored&path=Library/CMSIS`
- `purl`: `pkg:generic/cmsis@6.1.0`

## 4) Compliance Notes

- Keep original upstream copyright/license notices.
- Machine-readable metadata: `ProductView/sca_cmsis.json`.
