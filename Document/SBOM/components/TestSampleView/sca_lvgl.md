# LVGL Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored component under `ThirdParty/lvgl`.

## 1) Component Identity

- Component name (`name`): `LVGL`
- Component type (`type`): `library`
- Supplier / manufacturer: `LVGL Kft`
- Version: `9.4.0`
- License: `MIT`
- Evidence path: `ThirdParty/lvgl`
- CPE (used for vulnerability matching): `cpe:2.3:a:lvgl:lvgl:9.4.0:*:*:*:*:*:*:*`

## 2) Evidence for Version and License

- Version evidence: ThirdParty/lvgl/lv_version.h (LVGL_VERSION 9.4.0); ThirdParty/lvgl/LICENCE.txt (MIT licence, Copyright (c) 2025 LVGL Kft)

## 3) Suggested BOM-Ref and purl

- `bom-ref`: `pkg:generic/lvgl@9.4.0?source=vendored&path=ThirdParty/lvgl`
- `purl`: `pkg:generic/lvgl@9.4.0`

## 4) Compliance Notes

- Keep original upstream copyright/license notices.
- Machine-readable metadata: `TestSampleView/sca_lvgl.json`.
