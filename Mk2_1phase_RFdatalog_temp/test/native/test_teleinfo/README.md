# Teleinfo Tests

Tests for `teleinfo.h`, the telemetry frame of the IoT output. The frame is streamed: `TeleInfo::writeFrameStep()` writes one line per step of the serial output (STX before the first line, ETX after the last), each line with its own checksum, from a data source given as a template parameter. The test sources cover the feature sets of the firmware: CT2 (the diverted power `D` and energy `E`, after `P`), relays, loads, temperature sensors, dual tariff, a 32-bit sample count.

The reference is the buffered `TeleInfo` class of the former firmware, its code kept in the test (`utoa()`/`ultoa()` replaced by `snprintf()`), and its `sendTelemetryData()`, with `D` and `E` after `P` and `N` sent unsigned: the streamed frame must be the same, byte for byte.

## Tests

| Test | Description |
|------|-------------|
| `test_line` | Single lines: tag and index, sign, 16- and 32-bit values, the checksum, the longest line |
| `test_golden_frame` | A frame written by hand, checksums computed by hand |
| `test_streamed_frame_equals_buffered` | The streamed frame equals the buffered one, for every feature set, typical and extreme values, invalid temperatures |
| `test_streamed_frame_32_bit_sample_count` | The same beyond 19 s, the sample count on 32 bits |
| `test_step_fits_serial_output` | A step never writes more than `MAX_STEP_LENGTH`, at most the serial output's 32 characters |
| `test_one_line_per_step` | One step per line |
| `test_absence_count_unsigned` | `N` (seconds without diverted energy) is sent unsigned: 40000 and 65535, not negative |
| `test_diverted_labels_with_ct2` | With CT2: `D` and `E` right after `P`, exactly one of each, `D1`...`Dn` (the loads' ON time) still there |
| `test_no_diverted_labels_without_ct2` | Without CT2: neither `D` nor `E`, `D1`...`Dn` still there |

## Running

```bash
pio test -e native -f "native/test_teleinfo"
```
