# Fast Control Tests

Tests for the decision on the predicted energy (`fast_control.h`), Robin Emley's fasterControl. At the first sample set of the negative half cycle, the ISR predicts the bucket at the end of the cycle: the bucket at its start plus the energy of the first half scaled to a whole cycle (`energyInBucket_long + averagePower` in Robin's code). At sample set `DECISION_SAMPLE` (3, counted from 0) of the negative half, `Switching::onNewCycle()` decides on that prediction; the zero-crossing triac drivers latch the new states at the next positive zero crossing.

In `processing.cpp`, the prediction needs no division: the per-sample calibration table holds cal / n for each expected n, so twice the first half's sum through the entry for 2n sample sets is sumP x cal / n, the first half's average power as one cycle's energy, in the bucket's units (`wholeCycleIndexOf()` and the table overload of `predictEndOfCycle()`). With 2n outside the table (no proper half cycle), the bucket is kept as the prediction.

## Tests

| Test | Description |
|------|-------------|
| `test_prediction_adds_the_first_half_as_a_whole_cycle` | 300 units over 32 of 64 sample sets: 1000 + 600 = 1600 |
| `test_prediction_without_samples` | No sample set yet: the bucket as it is |
| `test_prediction_negative_power` | Import lowers the prediction: 1000 - 400 = 600 |
| `test_division_free_prediction_50Hz` | For every n with 2n in 60..68 (64 sample sets per cycle, sums halved) and sums up to full scale, both signs, two calibrations: the division-free prediction is the division form's, within the rounding of the table (2 units + 1/8192) |
| `test_division_free_prediction_60Hz` | The same with 2n in 49..57 (53 sample sets per cycle, no shift) |
| `test_division_free_index_out_of_range` | 2n below or above the table: no entry (`NO_INDEX`), the caller keeps the bucket |
| `test_decision_uses_prediction_only` | Bucket at 400 (lower half), prediction 900 (upper half): load 0 switches ON, lower threshold reset to 500, upper threshold untouched. Fed the measured 400, nothing would switch and the upper threshold would be reset instead |

## Running

```bash
pio test -e native -f "*test_fast_control*"
```
