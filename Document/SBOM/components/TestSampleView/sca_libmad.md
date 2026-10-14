# LibMAD Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored component under `ThirdParty/LibMAD`.

## 1) Component Identity

- Component name (`name`): `LibMAD`
- Component type (`type`): `library`
- Supplier / manufacturer: `Underbit Technologies, Inc.`
- Version: `0.15.1-beta`
- License: `GPL-2.0-or-later`
- Evidence path: `ThirdParty/LibMAD`
- CPE (used for vulnerability matching): `cpe:2.3:a:underbit:libmad:0.15.1b:*:*:*:*:*:*:*`

## 2) Evidence for Version and License

- Version evidence: ThirdParty/LibMAD/inc/version.h

## 3) Suggested BOM-Ref and purl

- `bom-ref`: `pkg:generic/libmad@0.15.1-beta?source=vendored&path=ThirdParty/LibMAD`
- `purl`: `pkg:generic/libmad@0.15.1-beta`

## 4) Compliance Notes

- Keep original upstream copyright/license notices.
- Machine-readable metadata: `TestSampleView/sca_libmad.json`.
- This component is licensed under a copyleft license; review the obligations before redistributing it or linking it into a product.
