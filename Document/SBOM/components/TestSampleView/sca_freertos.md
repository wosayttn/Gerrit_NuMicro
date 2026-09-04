# FreeRTOS-Kernel Third-Party Component Evidence

## Identity and Scope

- Name: FreeRTOS-Kernel
- Supplier evidence: Amazon.com, Inc. or its affiliates
- SBOM version: 10.0.0
- License: MIT
- PURL: `pkg:generic/freertos-kernel@10.0.0`
- Evidence path: `ThirdParty/FreeRTOS`
- View: Test Sample SBOM

This vendored source is required by FreeRTOS sample projects. It is not part of
the Product runtime scope unless a user explicitly integrates it.

## Version Evidence and Limitation

Most distributed kernel and Demo/Common source headers, including
`ThirdParty/FreeRTOS/Source/include/FreeRTOS.h`, state
`FreeRTOS Kernel V10.0.0`. However,
`ThirdParty/FreeRTOS/Source/include/task.h` retains
`tskKERNEL_VERSION_NUMBER "V9.0.0"` and matching 9.0.0 numeric macros.

The SBOM uses 10.0.0 because it is the pervasive distributed file-header
identity. The conflicting local macro is preserved as a documented evidence
limitation. No exact upstream tag, commit, or CPE is asserted.

## License and Supplier Evidence

`ThirdParty/FreeRTOS/License/license.txt` states that the FreeRTOS kernel is
released under the MIT license and records Amazon.com, Inc. or its affiliates.
The kernel source headers carry the same copyright and MIT permission text.

## Canonical Content Hash

- Algorithm: `sha256-path-nul-content-nul-v1`
- Tracked files: 136
- SHA-256:
  `cc6dadc5fae4b34b21fd987a66c6224fd0699246fe3ce3c9329969fa3ac05577`

The hash covers every Git-tracked file under `ThirdParty/FreeRTOS`, ordered by
repository-relative POSIX path, with path and raw content separated and
terminated by NUL bytes.
