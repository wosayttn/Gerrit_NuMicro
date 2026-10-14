# FatFs Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored component under `ThirdParty/FatFs`.

## 1) Component Identity

- Component name (`name`): `FatFs`
- Component type (`type`): `library`
- Supplier / manufacturer: `ChaN`
- Version: `R0.13a`
- License: `FatFs license (BSD-like, as declared by ChaN in source header)`
- Evidence path: `ThirdParty/FatFs`
- CPE (used for vulnerability matching): `cpe:2.3:a:elm-chan:fatfs:r0.13a:*:*:*:*:*:*:*`

## 2) Evidence for Version and License

- Version evidence: ThirdParty/FatFs/source/ff.h; ThirdParty/FatFs/source/ff.c; ThirdParty/FatFs/source/ffconf.h

## 3) Suggested BOM-Ref and purl

- `bom-ref`: `pkg:generic/fatfs@R0.13a?source=vendored&path=ThirdParty/FatFs`
- `purl`: `pkg:generic/fatfs@R0.13a`

## 4) Compliance Notes

- Keep original upstream copyright/license notices.
- Machine-readable metadata: `TestSampleView/sca_fatfs.json`.
