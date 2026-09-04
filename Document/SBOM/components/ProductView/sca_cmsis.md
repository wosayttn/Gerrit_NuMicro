# CMSIS Third-Party Component Evidence

## Identity and Scope

- Name: CMSIS
- Supplier: Arm Limited
- Version: 6.1.0
- License: Apache-2.0
- PURL: `pkg:generic/cmsis@6.1.0`
- Included paths: `Library/CMSIS/Core`, `Library/CMSIS/Driver`,
  `Library/CMSIS/RTOS2`
- Excluded independent component: `Library/CMSIS/DSP`

CMSIS is a required Product-view development and runtime support library.
CMSIS-DSP is represented separately because it has its own version and release
lifecycle.

## Evidence

- `Library/CMSIS/Core/Include/cmsis_version.h` defines CMSIS-Core 6.1.
- `Library/CMSIS/RTOS2/Include/cmsis_os2.h` defines the RTOS2 API version.
- Representative source files carry
  `SPDX-License-Identifier: Apache-2.0`.
- Repository import/update commit:
  `cc5fcbbceafb1e7f28e6aff65ac6943cec8d569f`.

The distributed source does not establish an exact upstream tag or commit, so
none is asserted.

## Canonical Content Hash

- Algorithm: `sha256-path-nul-content-nul-v1`
- Tracked files: 152
- SHA-256:
  `65e0bc843cd9a72f84c8ee3447882c949f27ba505d4e39558c5e287cd1d92d5d`

Files are ordered by repository-relative POSIX path. Each path, a NUL byte, its
raw contents, and a terminating NUL byte are hashed.
