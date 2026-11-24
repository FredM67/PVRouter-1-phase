# ADC Chain Tests

Tests for `adc_chain.h`: the analog inputs and the ADC reference of each board, and the order in which the three channels are converted. The ADC runs free: when the interrupt of a finished conversion runs, the next conversion has already started with the previous `ADMUX`, so the `ADMUX` written now applies to the conversion after next. The interrupt therefore writes, after each conversion, the channel two steps ahead in the chain V → I_GRID → I_DIVERTED.

## Tests

| Test | Description |
|------|-------------|
| `test_order_of_the_chain` | The three conversions are numbered 0 to 2 in the order V, I_GRID, I_DIVERTED |
| `test_pins_per_board` | The analog input of each channel: `OLD_SEG` V A3, CT1 A5, CT2 A4; `OLD_OLED` and `NEW` V A0, CT1 A1, CT2 A3 |
| `test_reference_per_board` | AVcc on the two old boards, the internal 1.1 V on `NEW` |
| `test_admux_of_is_reference_adlar_and_pin` | ADMUX is reference, ADLAR and pin, on each board |
| `test_after_next_of_every_position` | The channel written after a conversion is two steps ahead, wrap included (I_GRID gives V, I_DIVERTED gives I_GRID) |
| `test_admux_written_after_selects_the_channel_after_next` | `admuxWrittenAfter()` is the ADMUX of the channel after next, on each board |
| `test_channel_converted_by_round` | Round k converts channel k modulo 3 |
| `test_pipeline_order` | 30 rounds of the pipeline from a chain primed with V then I_GRID, on each board: the channel the `ADMUX` of each conversion selects is the one the interrupt believes it handles (`channelConverted()`), in the order V, I_GRID, I_DIVERTED; reference and left adjustment always set |

## Running

```bash
pio test -e native -f "native/test_adc_chain"
```
