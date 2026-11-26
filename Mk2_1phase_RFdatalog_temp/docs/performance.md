# Performance Characteristics

Timing of the firmware on the ATmega328P at 16 MHz, as measured in the grid simulator
([sim/README.md](../../sim/README.md)). For the flash and RAM, see [memory-layout.md](memory-layout.md).

## The ADC interrupt

The ADC is free-running: one conversion every 104 µs (13 ADC clocks at 125 kHz, 1,664 CPU
cycles), and one ISR call per conversion. Each call must end before the next conversion
completes.

Measured cycle-exactly in the grid simulator on `dev` e8a912fe, with the simulator's
configuration (`sim/configs/sim_config.sh`: three local loads, human-readable output, no
display, temperature sensing, relays or RF), ISR entry to `reti`:

| Build | Average | Longest call | Calls longer than one conversion |
|---|---|---|---|
| `OLD_SEG` (`make -C sim check`) | 347 cycles (21.7 µs) | 500 cycles (31.2 µs) | 0 |
| `NEW` (`make -C sim check-new`) | 243 cycles (15.2 µs) | 447 cycles (27.9 µs) | 0 |

On the old boards, the CT1 call runs the CT1 filter at every sample set (`ct_filter.h`: about 210
cycles in fixed point, against about 1100 with floats); it is off on `NEW`. The longest calls are
the CT2 calls that run a cycle task (`CycleTask`, `processing.cpp`): the work of the mains cycle
is spread over them, one task per call, instead of running in the call of the zero crossing.

The simulator fails a run whose longest call exceeds `isr_max` 560 cycles
(`sim/scenarios/common.scn`), so that a change that eats the headroom fails before samples are
lost.

## The ADMUX deadline

The ISR writes ADMUX last, and the value applies to the conversion after next: when the interrupt
of a finished conversion runs, the next conversion has already started (`adc_chain.h`). The write
must come at least one ADC clock (128 CPU cycles) after the start of the running conversion, as
the datasheet requires (earlier, that conversion may take the new channel), and before that
conversion ends, 1664 cycles after its start (later, the channels land one conversion late). The
simulator measures this as `ADMUX delay`, with `misfiled samples` and `ISR overruns`; in the two
runs above, ADMUX is written 135 to 504 cycles after its conversion started.

The ISR alone stays well inside this window. What brings the write close to the deadline is an
interrupt that starts late: the OneWire library keeps interrupts off for 65 to 70 µs (about 1100
cycles) for a bus reset and for each 0 written. The simulator enforces `admux_delay_max` 1620
cycles on the grid scenarios, 1480 for `check-new-temp` and 1600 for `check-seg-temp`, the
temperature builds: see `check-seg-temp` in [sim/README.md](../../sim/README.md). The loop
schedules the 1-Wire transactions for that reason (below).

## The main loop

The main loop (`loop()` in `main.cpp`) does what is not time-critical, triggered by flags the ISR
sets (`shared_var.h`):

- once per mains cycle (`b_newMainsCycle`, set once the work after the positive crossing is
  done): with temperature sensing, one short 1-Wire transaction first (`temp_schedule.h`), so that
  its slots fall among the ordinary ISR calls of the positive half; one round per datalog period:
  the conversion of every sensor (750 ms), then three transactions per sensor; the build refuses
  more sensors than fit in the period (`validation.h`);
- once per second: the per-second tasks (dual tariff, override pins and scheduled boosts, relays,
  load priorities and their rotation) and the display;
- once per datalog period (`b_datalogEventPending`, every `DATALOG_PERIOD_IN_SECONDS`, 5 s by
  default, 30 s at most): power and voltage, the relays' average, the temperatures, and the
  output;
- on every pass: the serial output in progress, as far as the transmit buffer allows (it never
  waits, `serial_output.h`), and the RF frames to the remote units.

The OLED is redrawn from the loop only when its value changes; the I2C transfer of the large
digits takes about 20 ms (`utils_oled.h`).

## Benchmarks

The embedded `*_perf` suites and `test_datalog_math_avr` count the CPU cycles of the arithmetic
helpers in simavr; CI compares them, with the flash and RAM, on every pull request. See
[Benchmark history](../test/README.md#benchmark-history).
