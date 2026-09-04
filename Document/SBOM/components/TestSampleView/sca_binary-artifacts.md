# M471 Test-Sample Binary Artifact Evidence

## Scope

Every Git-tracked `.bin`, `.lib`, `.a`, `.exe`, and `.dll` is represented by
its exact distributed path and SHA-256. This file supports the ten first-party
build artifacts declared in `sca_binary-artifacts.json`.

| Artifact group | Version basis | License and supplier basis |
| --- | --- | --- |
| FMC DualBank firmware | Adjacent `main.c` version V3.00 | Adjacent Nuvoton source and project files carry Apache-2.0 |
| FMC IAP firmware | `APROM_main.c` and `LDROM_main.c` version V1.00 | Adjacent Nuvoton source, assembly, and project files carry Apache-2.0 |
| FMC MultiWordProgram firmware | Adjacent `main.c` version V0.10 | Adjacent Nuvoton source and project files carry Apache-2.0 |
| XOMLib static libraries | `xomlib.c`, `xomlibIAR.c`, and `xomlib.h` version V3.00 | Those Nuvoton sources carry Apache-2.0 and the IAR/Keil projects name the corresponding outputs |

The checked-in firmware and libraries are treated as repository build outputs.
No compiler version or reproducible-build claim is made where the repository
does not record one.

`XOMAddr.exe` and `lua5.1.dll` are the remaining two physical binaries. They
are matched by exact SHA-256 through the canonical
`binary-license-database.json` snapshot and its Product Security-reviewed
evidence records. The database conclusion applies only to those exact hashes.

## Physical Closure

The machine-readable declarations carry each first-party artifact path, size,
SHA-256, source/project relationship evidence, supplier, version, license, and
PURL. Canonical generation additionally emits one `type=file` component per
physical path, so files with identical hashes would remain independently
closed.
