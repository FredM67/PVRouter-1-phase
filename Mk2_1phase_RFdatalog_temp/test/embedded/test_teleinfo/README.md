# Teleinfo Tests

Tests for `teleinfo.h`, the telemetry frame of the IoT output, on the ATmega328P (16-bit `int`): a frame is written one line per step of the serial output, STX before the first line, ETX after the last. The expected text is built independently, each checksum summed by the test. The native tests (`test/native/test_teleinfo`) check the frame byte for byte against the buffered frame of the former firmware.

## Tests

| Test | Description |
|------|-------------|
| `test_lineSize_calculation` | The size of a line (5 + tag + value), and a step fits the serial output's 32 characters |
| `test_teleinfo_line_contents` | Single lines: a 5-character temperature, 16- and 32-bit sample counts, an indexed relay state, -32768, 0 |
| `test_diverted_labels_with_ct2` | A whole frame with CT2: `D` (W) and `E` (Wh) after `P`, apart from `D1`, `D2` (the loads' ON time), an invalid temperature skipped |
| `test_no_diverted_labels_without_ct2` | The same frame without CT2: neither `D` nor `E` |

## Running

```bash
pio test -e uno_sim -f "*test_teleinfo*"
```
