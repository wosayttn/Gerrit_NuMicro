# License Review: GCC Syscalls and Semihosting Sources

## Scope

This review applies only to the following files in M2003xI_M2003xJ_Series:

- Library/Device/Nuvoton/M2003J/Source/GCC/_syscalls.c
- Library/Device/Nuvoton/M2003J/Source/GCC/semihosting.h

These files are part of the Product View under the M2003J device support source tree.

## Review reason

Neither file contains an SPDX-License-Identifier declaration. No SPDX identifier was inserted because the applicable license must not be inferred solely from file location, neighboring files, or similarity to another BSP.

The files are therefore tracked as license-review exceptions. Their exact path, length, and SHA-256 digest are recorded below so that the conclusion remains bound to the reviewed byte content.

## Reviewed files

### _syscalls.c

- Path: Library/Device/Nuvoton/M2003J/Source/GCC/_syscalls.c
- Length: 25154 bytes
- SHA-256: $(@{Path=Library/Device/Nuvoton/M2003J/Source/GCC/_syscalls.c; Exists=True; Length=25154; SHA256=C353A276489BC5CC5C7932F35C804CC27A264373E90D6409C224624AE1A08612; SpdxCount=0; CopyrightLines=Line 4: // Copyright (c) 2014 Liviu Ionescu}.SHA256)
- SPDX occurrence count: 0
- Treatment: Product View source retained for GCC runtime and system-call support; license remains subject to documented review.

### semihosting.h

- Path: Library/Device/Nuvoton/M2003J/Source/GCC/semihosting.h
- Length: 3673 bytes
- SHA-256: $(@{Path=Library/Device/Nuvoton/M2003J/Source/GCC/semihosting.h; Exists=True; Length=3673; SHA256=BADB1A3D393FDC2B14900424397C770FB18D65A0E2A9E67036BF25EDA0113DB0; SpdxCount=0; CopyrightLines=}.SHA256)
- SPDX occurrence count: 0
- Treatment: Product View source retained for GCC semihosting support; license remains subject to documented review.

## Cross-BSP handling

This review is specific to M2003xI_M2003xJ_Series.

The semihosting.h file is not assumed to be byte-for-byte identical to the corresponding file in another M2003 BSP. Each BSP must use its own path, length, and SHA-256 evidence.

## SBOM treatment

- Include both files in the Product View source scope.
- Do not create a separate third-party component solely for these files.
- Do not assign an SPDX license expression to these files without additional authoritative evidence.
- Preserve this review as supporting license evidence.
- Re-run the review if either file path, length, or SHA-256 digest changes.

## Review conclusion

The two files are accepted into the Product View as explicitly documented license-review exceptions. This record does not create, replace, or infer a license grant.
