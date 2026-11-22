# Fast Control Tests

Tests for the decision on the predicted energy (`fast_control.h`), Robin Emley's fasterControl. At the first sample set of the negative half cycle, the ISR predicts the bucket at the end of the cycle: the bucket at its start plus the energy of the first half scaled to a whole cycle (`energyInBucket_long + averagePower` in Robin's code). At sample set `DECISION_SAMPLE` (3, counted from 0) of the negative half, `Switching::onNewCycle()` decides on that prediction; the zero-crossing triac drivers latch the new states at the next positive zero crossing.

In `processing.cpp`, the first half goes through the same conversion as a whole cycle (`sumP >> SUM_P_SHIFT` and the per-sample calibration table, entry for `SAMPLE_SETS_PER_CYCLE`), so the prediction and the bucket share their units.

## Tests

| Test | Description |
|------|-------------|
| `test_prediction_adds_the_first_half_as_a_whole_cycle` | 300 units over 32 of 64 sample sets: 1000 + 600 = 1600 |
| `test_prediction_without_samples` | No sample set yet: the bucket as it is |
| `test_prediction_negative_power` | Import lowers the prediction: 1000 - 400 = 600 |
| `test_decision_uses_prediction_only` | Bucket at 400 (lower half), prediction 900 (upper half): load 0 switches ON, lower threshold reset to 500, upper threshold untouched. Fed the measured 400, nothing would switch and the upper threshold would be reset instead |

## Running

```bash
pio test -e native -f "*test_fast_control*"
```
