# M2003 `_syscalls.c` and `semihosting.h` License Review

## Review Scope

This review covers:

- `Library/Device/Nuvoton/M2003/Source/GCC/_syscalls.c`
- `Library/Device/Nuvoton/M2003/Source/GCC/semihosting.h`

## File Integrity Evidence

### `_syscalls.c`

- SHA-256: `C353A276489BC5CC5C7932F35C804CC27A264373E90D6409C224624AE1A08612`
- Size: 25154 bytes
- Git tracked: Yes

### `semihosting.h`

- SHA-256: `49ED6788E138763CA61041486F80CBA4AD562DFCBCA615771DC85D62F05D5A77`
- Size: 3721 bytes
- Git tracked: Yes

## Source Relationship

`_syscalls.c` directly includes `semihosting.h` at line 336.

No other reviewed C, header, or assembly file includes `semihosting.h`.
The header is therefore treated as an implementation-support file associated
with `_syscalls.c`.

## File-Level Notices

`_syscalls.c` contains the following notices:

- The file is part of the µOS++ III distribution.
- Parts of the file are from newlib sources and are stated to be issued under GPL.
- Copyright (c) 2014 Liviu Ionescu.

The available file evidence does not identify:

- the exact µOS++ or newlib revision;
- the applicable GPL version;
- whether the license is GPL-only or GPL-or-later;
- whether an exception applies; or
- the exact license boundaries between the incorporated portions.

`semihosting.h` does not contain an SPDX identifier, copyright statement,
or complete license statement.

## Cross-BSP Comparison

The M2003xC/M2003xD/M2003xE/M2003xG and M2003xI/M2003xJ `_syscalls.c`
files are byte-for-byte identical.

Shared `_syscalls.c` SHA-256:

`C353A276489BC5CC5C7932F35C804CC27A264373E90D6409C224624AE1A08612`

The corresponding `semihosting.h` files are not byte-for-byte identical:

- M2003xC/M2003xD/M2003xE/M2003xG:
  `49ED6788E138763CA61041486F80CBA4AD562DFCBCA615771DC85D62F05D5A77`
- M2003xI/M2003xJ:
  `BADB1A3D393FDC2B14900424397C770FB18D65A0E2A9E67036BF25EDA0113DB0`

The M2003 and M2U51 `_syscalls.c` files are also not byte-for-byte identical:

- M2003:
  `C353A276489BC5CC5C7932F35C804CC27A264373E90D6409C224624AE1A08612`
- M2U51:
  `F2A6DFF29C7EEB015068E34B4498B99A154A86D67ADB2F9CF1ED935D2FD3CD7E`

M2U51 may be used only as a reference for SBOM representation, not as
evidence of an identical upstream file.

## SBOM Treatment

- Do not create an independent `_syscalls.c` CycloneDX component.
- Do not create an independent `semihosting.h` CycloneDX component.
- Keep both files within the directory-level M2003 Device component.
- Do not assert an exact µOS++ or newlib version without reliable evidence.
- Do not assert a specific SPDX GPL expression from the incomplete notice.
- Preserve all file-level notices.
- Do not interpret the Device component's Apache-2.0 designation as
  relicensing the µOS++, newlib-derived, or Liviu Ionescu portions.

## Review Status

Accepted for directory-level SBOM representation with documented file-level
license limitations.

This decision applies only to SBOM representation. It does not constitute
relicensing, legal approval, or determination of the exact upstream license.

## Re-Review Conditions

Reopen this review if:

1. either reviewed file changes;
2. `_syscalls.c` becomes part of a confirmed default product build;
3. an exact µOS++ or newlib revision is identified;
4. the applicable GPL version or exception is confirmed;
5. OSS or Legal review requires a separate component; or
6. the M2003 Device SBOM treatment changes.

## Evidence Baseline

- Baseline Git commit: `27a72fba7b47a22363900ece3fbc91a5890b24dd`
- Baseline short commit: `27a72fb`
- Review date: `2026-08-25`
