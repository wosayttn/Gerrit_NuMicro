# FreeRTOS-Kernel Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored FreeRTOS component under `ThirdParty/FreeRTOS`.

## 1) Component Identity

- Component name (`name`): `FreeRTOS-Kernel`
- Component type (`type`): `library`
- Supplier / project: `FreeRTOS` (Amazon)
- Version: `v10.5.1`
- License: `MIT` (SPDX)
- Evidence path: `ThirdParty/FreeRTOS`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `ThirdParty/FreeRTOS/Source/manifest.yml`
  - `name : "FreeRTOS-Kernel"`
  - `version: "v10.5.1"`
  - `license: "MIT"`
- `ThirdParty/FreeRTOS/Source/include/FreeRTOS.h`
  - Header shows `FreeRTOS Kernel V10.5.1`
  - Header includes `SPDX-License-Identifier: MIT`
- `ThirdParty/FreeRTOS/Source/tasks.c`
  - Header shows `FreeRTOS Kernel V10.5.1`
  - Header includes `SPDX-License-Identifier: MIT`
- `ThirdParty/FreeRTOS/Source/sbom.spdx`
  - `PackageName: FreeRTOS-Kernel`
  - `PackageVersion: v10.5.1`
  - `PackageLicenseConcluded: MIT`

## 3) License Handling Guidance

For CycloneDX output, use SPDX license ID directly:

- `licenses[0].license.id = MIT`

Also preserve upstream license headers in vendored source files.

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `library`
- `name`: `FreeRTOS-Kernel`
- `version`: `10.5.1`
- `scope`: `required`
- `author`: `Amazon.com, Inc. (FreeRTOS project)`
- `description`: `FreeRTOS Kernel vendored in ThirdParty/FreeRTOS.`
- `licenses[0].license.id`: `MIT`
- `properties` (recommended custom properties):
  - `src_path = ThirdParty/FreeRTOS`
  - `integration = vendored_source`
  - `upstream_manifest = ThirdParty/FreeRTOS/Source/manifest.yml`

## 5) Suggested BOM-Ref and purl

Suggested values:

- `bom-ref`: `pkg:generic/freertos-kernel@10.5.1?source=vendored&path=ThirdParty/FreeRTOS`
- `purl`: `pkg:generic/freertos-kernel@10.5.1`

If your internal SBOM naming policy differs, keep naming consistent across all third-party components in this BSP.

## 6) CycloneDX JSON Component Example

```json
{
  "type": "library",
  "bom-ref": "pkg:generic/freertos-kernel@10.5.1?source=vendored&path=ThirdParty/FreeRTOS",
  "name": "FreeRTOS-Kernel",
  "version": "10.5.1",
  "scope": "required",
  "author": "Amazon.com, Inc. (FreeRTOS project)",
  "purl": "pkg:generic/freertos-kernel@10.5.1",
  "description": "FreeRTOS Kernel vendored in ThirdParty/FreeRTOS.",
  "licenses": [
    {
      "license": {
        "id": "MIT"
      }
    }
  ],
  "properties": [
    { "name": "src_path", "value": "ThirdParty/FreeRTOS" },
    { "name": "integration", "value": "vendored_source" },
    { "name": "upstream_manifest", "value": "ThirdParty/FreeRTOS/Source/manifest.yml" }
  ]
}
```

## 7) Compliance Notes

- Keep original upstream copyright/license notices.
- If producing a finer-grained SBOM (for example, app + RTOS + board support), represent each major unit as separate components and declare dependencies between them.

## Canonical source identity

- Version anchor: `ThirdParty/FreeRTOS/Source/include/FreeRTOS.h`
- Version-anchor SHA-256: `4fb17b1ff11fc3f82cff4e04f4546fc82ff3115ef213097fd942c3acdb657fbb`
- Aggregate algorithm: `sha256-path-nul-content-nul-v1`
- Aggregate path base: `ThirdParty/FreeRTOS`
- Tracked files: `144`
- Aggregate SHA-256: `d396d0257baa029714b1507428b4aeaed19e144b821e8677b1528737b32ed83b`
- Exact inventory: `Document/SBOM/inventory/freertos.json`
- Canonical CPE: `cpe:2.3:o:amazon:freertos:10.5.1:*:*:*:*:*:*:*`
- CPE binding: all 144 tracked paths and file hashes are identical to formal M2U51 evidence `sca_freertos.json` SHA-256 `c34bdf0b5413c57cfb65d2f9f0b8a077489b1ba296a8af4615a6ad928db563c2`. Its repository-relative comparison digest is `8b5255922f674077cbb1ea038fc5fd78adb40b013e5dbdd8a9531758c8d42113`; this establishes component identity only, not M2L31 affectedness or approval.
- License evidence: the version-anchor source header identifies `MIT`.
