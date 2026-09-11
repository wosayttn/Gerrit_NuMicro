# Product Security Review Status

- Candidate baseline: `7d75be4053bb0365b838f165216fb42328a69ae2`
- Product Security assessment: `not_assessed`
- Product Security approval: `pending`
- VEX: `not_published`
- `not_affected` assertions: none

Offline Grype 0.117.0 with database schema v6.1.9 and
`add-cpes-if-none=false` produced five Test Sample scanner range matches:
`CVE-2024-28115`, `CVE-2026-77234`, `CVE-2026-77235`,
`CVE-2026-77236`, and `CVE-2026-77237`. These are retained as scanner
range matches only. They are not affectedness determinations, VEX statements,
or security approvals. PSIRT/Product Security disposition remains required and
the external release gate remains blocked.
