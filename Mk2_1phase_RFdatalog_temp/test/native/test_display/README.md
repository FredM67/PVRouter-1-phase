# Display Tests

Tests for `display_core.h`, the part of the display that needs neither Arduino nor a setting: the pins of each display type (as in `Mk2_fasterControl_Full`), the port images the ADC interrupt writes to multiplex the 7-segment display, what is shown in turn, and the characters or text shown for a value.

## Tests

| Test | Description |
|------|-------------|
| `test_pins_of_each_type` | The pins of SEG (12), SEG_HW (8) and the OLED (A4, A5) as a list and as a 32-bit mask, none twice; none for NONE |
| `test_image_of_a_pin` | D0-D7 on PORTD, D8-D13 on PORTB, D14-D19 on PORTC |
| `test_seg_images` | SEG: the segments of a few characters, the DP, the digit lines (HIGH but the digit on), the blanking bits |
| `test_seg_hw_images` | SEG_HW: the BCD lines of a few characters ('-' is a blank), the DP, the 74HC138 lines, the enable line |
| `test_images_stay_within_the_pins` | No image, for any character, DP and digit, sets a bit outside the display's pins |
| `test_energy_slot` | FORC first, then OFF, then the energy, or the walking dots before any diversion (7-segment); no energy without CT2, but FORC and OFF still |
| `test_turn` | The energy slot, then each sensor, then again; temperatures only without slot; WAITING with nothing to show |
| `test_seg_energy` | "1.234" up to 9999 Wh, "12.34" above, as Full |
| `test_seg_texts` | FORC, OFF and the walking dots, as Full |
| `test_seg_temperature` | Rounded to 0.1 °C ("23.5C", "-5.2C"), whole degrees from 100 °C and below -9.9 °C, no "-0.0", "----" when invalid; no 'C' nor '-' on SEG_HW |
| `test_oled_texts` | The 5 characters of the large OLED digits for the energy and the temperature |

## Running

```bash
pio test -e native -f native/test_display
```
