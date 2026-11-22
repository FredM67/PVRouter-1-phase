# CT1 Filter Tests

Tests for `ct_filter.h`, the filter that offsets the high-pass behaviour of the grid CT: a low-pass of the current, added back with a gain (`lpf_gain`, `alpha` in `calibration.h`, on the old boards). It is Robin's filter of `Mk2_fasterControl_Full`, at his resolution (x256), in fixed point instead of his float maths: `coefficientsOf()` turns the float settings into fixed point at compile time, the update truncates towards zero as his float to int32 conversion does, and the output is back at x64, saturated to the int16 range.

## Tests

| Test | Description |
|------|-------------|
| `test_gain_zero_is_identity` | With a gain of 0 the sample comes back unchanged, full scale included |
| `test_full_scale_with_a_dc_residual_saturates` | After a DC residual of 6 counts, a full-scale peak saturates to `INT16_MAX` / `INT16_MIN`, sign kept, instead of wrapping |
| `test_a_small_steady_input_moves_the_state` | 4 ADC counts move the low-pass state by one x256 unit, as Robin's: the dead zone is no wider than his (at x64 it was 14 counts) |
| `test_time_constant_within_2_percent_of_robins` | After a step, the low-pass reaches 1 - 1/e of it within 2 % of -1 / ln(1 - alpha) samples, the time constant of Robin's float formula |
| `test_steady_state_within_one_count_of_robins` | One second of a 50 Hz current with a DC residual: the output stays within one ADC count (64 at x64) of Robin's code, verbatim, at every sample, and the state within one count of his at the end |

## Running

```bash
pio test -e native -f "native/test_ct_filter"
```
