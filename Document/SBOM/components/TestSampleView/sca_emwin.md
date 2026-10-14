# emWin Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored component under `ThirdParty/emWin_light`.

## 1) Component Identity

- Component name (`name`): `emWin`
- Component type (`type`): `library`
- Supplier / manufacturer: `SEGGER Microcontroller GmbH`
- Version: `6.46.12`
- License: `SEGGER emWin license (software licensed by SEGGER Software GmbH to Nuvoton Technology Corporation)`
- Evidence path: `ThirdParty/emWin_light`

## 2) Evidence for Version and License

- Version evidence: ThirdParty/emWin_light/Include/GUI_Version.h (GUI_VERSION 646012); ThirdParty/emWin_light/Include/GUI.h (SEGGER license header)

## 3) Suggested BOM-Ref and purl

- `bom-ref`: `pkg:generic/emwin@6.46.12?source=vendored&path=ThirdParty/emWin_light`
- `purl`: `pkg:generic/emwin@6.46.12`

## 4) Compliance Notes

- Keep original upstream copyright/license notices.
- Machine-readable metadata: `TestSampleView/sca_emwin.json`.
