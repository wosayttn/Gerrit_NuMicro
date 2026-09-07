# CMSIS Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored Arm CMSIS source subset under `Library/CMSIS`.

This CMSIS component covers only `Core/Include`, `Driver`, and `RTOS2`. `Core/Test` and `Documentation` are excluded from component scope. Selected documentation files are used only as supporting version evidence.

## 1) Component Identity

- Component name (`name`): `CMSIS`
- Component type (`type`): `library`
- Supplier / project: `Arm Limited`
- Release version: `6.1.0`
- License: `Apache-2.0` (SPDX)
- Evidence path: `Library/CMSIS`
- Integration type: `vendored_source_subset`
- Upstream project: `ARM-software/CMSIS_6`

Included paths:

- `Library/CMSIS/Core/Include`
- `Library/CMSIS/Driver`
- `Library/CMSIS/RTOS2`

Excluded paths:

- `Library/CMSIS/Core/Test`
- `Library/CMSIS/Documentation`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `Library/CMSIS/Core/Include/cmsis_version.h`
  - Header includes `SPDX-License-Identifier: Apache-2.0`
  - `__CM_CMSIS_VERSION_MAIN` is `6U`
  - `__CM_CMSIS_VERSION_SUB` is `1U`
  - CMSIS-Core version is `6.1.0`

- `Library/CMSIS/RTOS2/Include/cmsis_os2.h`
  - Header identifies the project as `CMSIS-RTOS2 API`
  - Header revision is `V2.3.0`
  - CMSIS-RTOS2 API version is `2.3.0`

- `Library/CMSIS/Driver/Include/Driver_Common.h`
  - Header includes `SPDX-License-Identifier: Apache-2.0`
  - Header revision is `V2.0`
  - Common Driver definitions API version is `2.0`

- `Library/CMSIS/Documentation/html/General/revision_history.html`
  - CMSIS Base Software release is `6.1.0`
  - CMSIS-Core release is `6.1.0`
  - CMSIS-Driver release is `2.10.0`

The CMSIS-Driver interface and template files contain individual API and implementation versions. Those values are not used as the overall CMSIS component version.

The reviewed CMSIS-Driver scope contains 36 C and header files. All 36 files contain an Apache-2.0 SPDX identifier.

## 3) License Handling Guidance

For CycloneDX output, use the SPDX license identifier directly:

- `licenses[0].license.id = Apache-2.0`

Preserve the original upstream copyright and license headers in all vendored source files.

No missing SPDX identifier was identified in the reviewed CMSIS-Driver source scope.

The CMSIS documentation directory is excluded from component scope. Its revision history is referenced only as supporting version evidence.

## 4) Suggested CycloneDX Field Mapping

- `name`: `CMSIS`
- `type`: `library`
- `version`: `6.1.0`
- `scope`: `required`
- `author`: `Arm Limited`
- `purl`: `pkg:github/ARM-software/CMSIS_6@6.1.0`
- `bom-ref`: `pkg:github/ARM-software/CMSIS_6@6.1.0?source=vendored&path=Library/CMSIS`
- `licenses[0].license.id`: `Apache-2.0`

Module-level properties:

- `cmsis_core_version = 6.1.0`
- `cmsis_rtos2_api_version = 2.3.0`
- `cmsis_driver_common_api_version = 2.0`
- `bsp:cmsis-driver-release = 2.10.0`

Source paths, included paths, excluded paths, upstream project, version evidence, and license evidence are recorded in `sca_cmsis.json`.

## 5) Compliance Notes

- Keep all original upstream copyright and license notices.
- Represent the reviewed CMSIS subset as one Product SBOM component.
- Do not create separate CMSIS-Core, CMSIS-Driver, or CMSIS-RTOS2 components for this BSP.
- Exclude `Library/CMSIS/Core/Test` from the Product SBOM.
- Exclude `Library/CMSIS/Documentation` from component scope.
- Documentation may be used only as supporting version evidence.
- Do not interpret individual Driver API versions as the CMSIS release version.
- Reopen this review if the CMSIS source subset, version evidence, or license evidence changes.
