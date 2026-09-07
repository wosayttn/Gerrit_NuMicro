# M471 BSP SBOM Scope Definition

## Product SBOM

The Product SBOM covers software intended to support target firmware
development or integration:

- `Library/CMSIS`
- `Library/Device`
- `Library/StdDriver`

The following distributed material is excluded from Product runtime scope:

- `Document/` and `SampleCode/`
- `ThirdParty/FreeRTOS`, which is used by sample projects
- `Library/CMSIS/Documentation`
- `Library/CMSIS/CoreValidation`

CMSIS documentation and CoreValidation content are reference and validation
material, not runtime dependencies. CMSIS-Core, CMSIS-Driver, CMSIS-RTOS2, and
CMSIS-DSP are represented by evidence-backed manual components.

## Test Sample SBOM

The Test Sample SBOM covers:

- `SampleCode/`
- `ThirdParty/`

This view includes sample projects, project configuration, generated sample
firmware, host-side utilities, static sample libraries, and the vendored
FreeRTOS sources. These components are not Product runtime dependencies unless
the user explicitly integrates or redistributes them.

## Repository-Level Exclusions

The repository root build/editor configuration, `Readme.pdf`, and SBOM evidence
under `Document/SBOM` are delivery metadata rather than software components and
are outside both scan views.

## Maintenance

Review these scopes whenever a top-level directory, middleware component,
third-party source tree, host utility, or distributed binary is added or
removed.
