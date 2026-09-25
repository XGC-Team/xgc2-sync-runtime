# Transport findings (Z2b calibration)

These were measured with `crates/xgc-rt-host/tests/zenoh_impaired.rs`:
- **Transport:** zenoh 1.9.0, peer mode, UDP unicast. The class is `control`: best-effort, drop, RealTime, express.
- **Traffic:** 2000 samples × 1 KiB at 200 Hz.
- **Impairment:** the seeded relay `xgc-rt-impair`, which logs ground truth per sample.

## Results

| Profile | Relay truth | Audit | Transport discarded | OWD − injected delay, p50 / p99 |
|---|---|---|---|---|
| A: 40 ms, GE burst loss (≈2.5 % avg), 2 % dup, 2 % reorder | 54 dropped, 43 duplicated, 46 held | 100 lost, 0 dup, 0 reordered | 46, exactly the held set | 0.49 / 1.50 ms |
| B: 40 ± 10 ms jitter, no loss | 0 dropped | 683 lost (34 %), 0 reordered | 683 | 0.57 / 8.3 ms (survivors) |

**Audit correctness held in both profiles:**
- every relay drop is an audited loss;
- no dropped sample appears as received;
- every other loss is a frame the relay delivered but the transport discarded.

## What this means

1. **Zenoh best-effort discards any frame that arrives after a later one, and suppresses duplicates.** So on this transport:
   - reordering on the wire shows up as **loss**, never as a reorder count;
   - duplicates never reach the module.

   The audit's reorder and duplicate counters therefore read 0 on the `control` and `state` classes by construction. That is a property of the transport, not evidence of a clean network.
2. **Jitter larger than the send interval costs samples.** At 200 Hz, ±10 ms lost 34 %. At the TRO DMPC rate (10 Hz, 100 ms interval) the same jitter would not reorder. The exposure grows with the channel rate.
3. **OWD under jitter is a survivor statistic.** Late frames are discarded, so the audited OWD distribution under-states the network's. In profile B, the audited p50 of 37.6 ms is below the injected median of 40 ms. A claim about link delay under jitter must use the relay or netem truth, or a reliable class.
4. For DMPC, "latest wins" may be the desired semantics: an older neighbor plan that arrives after a newer one is useless. But then the **loss** figure mixes network loss with superseded frames. Only the impairment ground truth separates the two.

## Open decision (D7)

Keep `control` on best-effort with latest-wins, and report loss as "not delivered (lost or superseded)"? Or offer a sequenced-reliable class for claims that need wire-level reorder statistics? The default until decided is best-effort, with this caveat stated in every report that uses it.
