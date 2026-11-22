# CT1 Filter Tests

Tests for `ct_filter.h`, the filter that offsets the high-pass behaviour of the grid CT: a low-pass of the current, added back with a gain (`lpf_gain`, `alpha` in `calibration.h`, on the old boards). It is Robin's filter of `Mk2_fasterControl_Full`, at his resolution: the firmware's DC-removed sample (x64 scale, int16) is scaled up to x256 in an int32, filtered, scaled back and saturated to the int16 range.

## Tests

| Test | Description |
|------|-------------|
| `test_gain_zero_is_identity` | With a gain of 0 the sample comes back unchanged, full scale included |
| `test_full_scale_with_a_dc_residual_saturates` | After a DC residual of 6 counts, a full-scale peak saturates to `INT16_MAX` / `INT16_MIN`, sign kept, instead of wrapping |
| `test_a_small_steady_input_moves_the_state` | 4 ADC counts move the low-pass state (at x64 the dead zone was 14 counts) |
| `test_matches_robins_formula` | One second of a 50 Hz current with a DC residual: state and output equal Robin's code, verbatim, at every sample |

## Running

```bash
pio test -e native -f "native/test_ct_filter"
```
