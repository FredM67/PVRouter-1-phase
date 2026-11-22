# Diverted Energy Tests

Tests for `diverted_energy.h`, the diverted energy measured on CT2. It is Robin's accounting of `Mk2_fasterControl_Full`: once per mains cycle, the energy measured on CT2 counts if it reaches the anti-creep limit (`ANTI_CREEP_LIMIT`), whole Wh are carried into a 16-bit total, and everything is reset after `DISPLAY_SHUTDOWN_IN_HOURS` without diversion. The mains cycles during which a local load or relay is forced (override pin, scheduled boost) are not counted at all, and count as cycles without diversion. `DivertedPower::watts()` gives the average diverted power over a datalog period without a division, a forced cycle counting as 0 W.

## Tests

| Test | Description |
|------|-------------|
| `test_whole_wh_carried` | With 100 units per Wh, 60 twice gives 1 Wh and keeps the remainder: a third 60 gives 1, a fourth 2 |
| `test_anti_creep_drops_noise` | With a limit of 5, 1000 cycles of 4 leave 0 Wh and the detector inactive; a cycle at the limit counts |
| `test_negative_energy_is_not_counted` | Without a limit, negative cycles (CT2 the wrong way round, noise) are not subtracted |
| `test_reset_after_inactivity` | With 10 shutdown cycles, 9 or 10 cycles without any load ON keep the total, 11 reset it, the remainder and the detector too |
| `test_total_saturates` | The total stops at 65535 Wh, never wraps |
| `test_forced_cycle_not_counted` | 10 cycles of 60 with a load forced leave 0 Wh; the same cycles not forced give 6 Wh |
| `test_forced_cycles_count_towards_reset` | Forced cycles count as cycles without diversion: a boost at night does not keep the day's total alive |
| `test_counted_energy` | The energy counted: 0 below the limit, when negative or when forced |
| `test_average_watts` | The average power of a 5 s period at 50 Hz and of a 40 s period at 60 Hz, saturated to 32767 W |

## Running

```bash
pio test -e native -f "native/test_diverted_energy"
```
