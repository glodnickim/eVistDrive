# TQ-06 phase 5 implementation decisions

Timestamp: 2026-09-24T20:06:18+02:00

- M820 has one G53 chain owner. Fast and supervisor phase counters are stored separately, reset to zero explicitly, and advance together modulo 10. This is an adapter decision authorized by contract section 8, matching the accepted tick-indexed Oracle; it does not claim the original firmware had a shared scheduler zero point.
- PAS runs before the chain; FSM phase 7 precedes BDE8. BDE8 executes every logical tick before D7EC phase 0 or E1E8 phase 3. Tick trace encodes executed stages as PAS=1, FSM=2, BDE8=4, D7EC=8, E1E8=16.
- Register-offset names identify private byte storage, never native MCU memory. Explicit little-endian 8/16/32-bit writes preserve overlapping publications. Fixed D4 initial bytes come from the accepted resets. Wider temporary integer arithmetic preserves Python/ARM width operations without signed C overflow. No floating point, dynamic allocation, original binary execution or PI1/PI2 is used.
- normal_permission observes BDE8 normal drive-command mode M298=2. It adds no independent PAS gate and does not alter M2AA. Native veto integration remains phase 7 work.
- 39600 differential ticks compare all 120 TickTrace fields, with all six FSM states and M2AA ceiling 6500 reached. Eight mutants are rejected. This is software evidence only, awaiting independent review.
- Reference pins were checked with the unmodified accepted reference.py loader. Its four expected hashes identify CRLF checkout bytes; governance blobs use LF. Reconstructing CRLF alone matches all four exact pins. The complete fixture was revalidated through that unmodified loader: zero mismatches. Neither governance sources nor the original golden fixture were changed.
