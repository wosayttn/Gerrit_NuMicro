# M2U51 BSP current SBOM

These repository-current source-review artifacts were generated from scan source
`00b0843b3254c1c09961d0fc5d4b1b266a547ae8` at generation commit
`abfceb90e21a8e33b20cebf063a1035fa5e81b5c`. Product and Test Sample BOMs are
CycloneDX 1.6. They are not a formal release or distribution package; release and
distribution commits remain null.

Product component count is 5 and Test Sample component count is 5. Firmware
components use one semantic component each with an exact path/SHA occurrence; no
duplicate file component is counted. Durable aggregate inventories and blocker
evidence are stored under `Document/SBOM/`.

The Product scan has zero matches but is identity-limited because no
authoritative first-party CPE is asserted. It is not a clean security
assessment. The Test Sample scan preserves five FreeRTOS 10.5.1 range matches:
CVE-2024-28115 (High), CVE-2026-77234 (Critical), and CVE-2026-77235,
CVE-2026-77236, CVE-2026-77237 (High). All remain `not_assessed/pending`.

Formal release remains **BLOCKED**: PSIRT/Product Security disposition,
OSS/Legal classification for the excluded syscall files, and reproducible FMC
build/link/runtime evidence are absent. Repository validation does not clear
these external blockers.
