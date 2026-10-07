# Specifications

These are unchanged publisher PDFs, stored with Git LFS. `SHA256SUMS` checks
their downloaded bytes.

| Simulator | Document | Issue | Date | Publisher download |
| --- | --- | --- | --- | --- |
| VQ1 | [VADR-TS-002](VQ1-Technical-Specification-00.02.pdf) | 00.02 | 2026-05-04 | [PDF](https://www.theaigrandprix.com/wp-content/uploads/2026/05/260508_Technical_Spec_0002.pdf) |
| VQ2 | [VADR-TS-003](VQ2-Technical-Specification-00.03.pdf) | 00.03 | 2026-06-24 | [PDF](https://www.theaigrandprix.com/wp-content/uploads/2026/06/260624_Technical_Spec_0003.pdf) |

VQ2 section 9.3 blocks `ATTITUDE`, `LOCAL_POSITION_NED`, `ODOMETRY`, and
`GATE_INFO`. The VQ1 legacy executable retains its privileged telemetry.
See [wiring](wiring.md) for what packets actually arrive and how the adapter
exposes them. A specification's physics rate is not a promise that every sensor
packet arrives at that rate.
