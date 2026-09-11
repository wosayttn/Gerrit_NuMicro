# M2L31 BSP durable SBOM source scope

This durable source set is generated from Git commit
`7d75be4053bb0365b838f165216fb42328a69ae2`. Scan source, generation,
release, and distribution provenance are separate. Release identity,
distribution identity, and lifecycle commit are `null`; it makes
no formal-release or publication claim.

The Product view covers `Library/`. The Test Sample view covers
`SampleCode/`, `ThirdParty/`, and `Tool/`. Every component is reachable from
its metadata root and has a dependency row.

All Git-tracked `.a`, `.bin`, `.cat`, `.dll`, `.exe`, `.inf`, `.lib`, and `.sys`
artifacts are represented exactly once with repository-relative path,
worktree SHA-256, and Git blob SHA-1. The 13 CAT/INF artifacts are included;
their identity evidence is recorded, but their licenses remain fail-closed
external legal decisions.

CMSIS, FatFs, and FreeRTOS use
`sha256-path-nul-content-nul-v1` tracked-scope aggregates with exact
inventories. Their former single-file hashes are explicitly version anchors.

Shared verification may pass while structured external blockers remain.
The external release gate is separate and must return nonzero until all legal and
PSIRT/Product Security blockers are resolved.
