# M2U51 `_syscalls.c` and `semihosting.h` legal review status

## Scope

The files remain distributed at source commit `00b0843b3254c1c09961d0fc5d4b1b266a547ae8` but are excluded
from the `M2U51 Device` aggregate and all runtime claims. The exact inventory and
SHA-256 values are in `excluded-source-inventory.json`.

## Status

**BLOCKED.** Repository evidence does not establish an exact µOS++/newlib
upstream revision, applicable GPL version, exception, or authoritative
redistribution terms. Historical treatment in another BSP is not legal approval
and is not used to infer a license.

Required decision: OSS/Legal must provide a hash-bound classification and
redistribution decision for both files, including the applicable SPDX expression
or approved custom license text. Until then, the files remain explicitly
distributed, excluded, inventoried, and release-blocking.
