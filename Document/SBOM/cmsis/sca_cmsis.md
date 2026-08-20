# CMSIS Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored Arm CMSIS component under `Library/CMSIS`.

## 1) Component Identity

- Component name (`name`): `CMSIS`
- Component type (`type`): `library`
- Supplier / project: `Arm Limited`
- Version: `6.1.0`
- License: `Apache-2.0` (SPDX)
- Evidence path: `Library/CMSIS`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `Library/CMSIS/Core/Include/cmsis_version.h`
  - Header includes `SPDX-License-Identifier: Apache-2.0`
  - `__CM_CMSIS_VERSION_MAIN` is `6U`
  - `__CM_CMSIS_VERSION_SUB` is `1U`
  - `__CA_CMSIS_VERSION_MAIN` is `6U`
  - `__CA_CMSIS_VERSION_SUB` is `1U`
- `Library/CMSIS/RTOS2/Include/cmsis_os2.h`
  - Header includes `SPDX-License-Identifier: Apache-2.0`
  - Header shows `Project: CMSIS-RTOS2 API`
  - Header shows `Version 2.3.0`
- `Library/CMSIS/Driver/Include/Driver_Common.h`
  - Header includes `SPDX-License-Identifier: Apache-2.0`
  - Header shows common driver definitions API `Version 2.0`
- `Library/CMSIS/Documentation/Overview.md`
  - Describes CMSIS as the Common Microcontroller Software Interface Standard
  - Identifies the included material as CMSIS v6 related content

## 3) License Handling Guidance

For CycloneDX output, use SPDX license ID directly:

- `licenses[0].license.id = Apache-2.0`

Also preserve upstream copyright and license headers in vendored source files.

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `library`
- `name`: `CMSIS`
- `version`: `6.1.0`
- `scope`: `required`
- `author`: `Arm Limited`
- `description`: `Arm CMSIS vendored in Library/CMSIS, including CMSIS-Core, CMSIS-Driver, CMSIS-RTOS2 API headers, templates, and documentation.`
- `licenses[0].license.id`: `Apache-2.0`
- `properties` (recommended custom properties):
  - `src_path = Library/CMSIS`
  - `integration = vendored_source`
  - `cmsis_core_version = 6.1`
  - `cmsis_rtos2_api_version = 2.3.0`
  - `cmsis_driver_common_api_version = 2.0`

## 5) Suggested BOM-Ref and purl

Suggested values:

- `bom-ref`: `pkg:generic/cmsis@6.1.0?source=vendored&path=Library/CMSIS`
- `purl`: `pkg:generic/cmsis@6.1.0`

If your internal SBOM naming policy differs, keep naming consistent across all third-party components in this BSP.

## 6) CycloneDX JSON Component Example

```json
{
  "type": "library",
  "bom-ref": "pkg:generic/cmsis@6.1.0?source=vendored&path=Library/CMSIS",
  "name": "CMSIS",
  "version": "6.1.0",
  "scope": "required",
  "author": "Arm Limited",
  "purl": "pkg:generic/cmsis@6.1.0",
  "description": "Arm CMSIS vendored in Library/CMSIS, including CMSIS-Core, CMSIS-Driver, CMSIS-RTOS2 API headers, templates, and documentation.",
  "licenses": [
    {
      "license": {
        "id": "Apache-2.0"
      }
    }
  ],
  "properties": [
    { "name": "src_path", "value": "Library/CMSIS" },
    { "name": "integration", "value": "vendored_source" },
    { "name": "cmsis_core_version", "value": "6.1" },
    { "name": "cmsis_rtos2_api_version", "value": "2.3.0" },
    { "name": "cmsis_driver_common_api_version", "value": "2.0" }
  ]
}
```

## 7) Compliance Notes

- Keep original upstream copyright/license notices.
- CMSIS in this BSP contains multiple CMSIS subareas, including Core, Driver, RTOS2 API headers, templates, and documentation.
- If producing a finer-grained SBOM, represent major CMSIS subcomponents separately when product policy requires component-level version tracking.
