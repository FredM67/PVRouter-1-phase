# ADC Chain Tests

Tests for `adc_chain.h`: the analog inputs and the ADC reference of each board, and the order in which the three channels are converted. The ADC runs free: when the interrupt of a finished conversion runs, the next conversion has already started with the previous `ADMUX`, so the `ADMUX` written now applies to the conversion after next. The interrupt therefore writes, after each conversion, the channel two steps ahead in the chain V → I_GRID → I_DIVERTED.

## Tests

| Test | Description |
|------|-------------|
| `test_pins_per_board` | The analog input of each channel: `OLD_SEG` V A3, CT1 A5, CT2 A4; `OLD_OLED` and `NEW` V A0, CT1 A1, CT2 A3 |
| `test_reference_per_board` | AVcc on the two old boards, the internal 1.1 V on `NEW` |
| `test_pipeline_order` | 30 rounds of the pipeline from a chain primed with V then I_GRID, on each board: the channel the `ADMUX` of each conversion selects is the one the interrupt believes it handles (`channelConverted()`), in the order V, I_GRID, I_DIVERTED; reference and left adjustment always set |

## Running

```bash
pio test -e native -f "native/test_adc_chain"
```
