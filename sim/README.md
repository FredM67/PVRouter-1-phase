# grid_sim - the router on a simulated grid

`grid_sim` runs the **production firmware** (`firmware.elf`, unmodified) in [simavr](https://github.com/buserror/simavr), an ATmega328P emulator, and connects it to a simulated single-phase installation:

- the 3 ADC channels of the board - voltage, CT1 at the grid connection, CT2 on the diverted current - receive 50 Hz (or 60 Hz) waves on the board's analog inputs, computed at the exact CPU cycle of each conversion;
- each load pin drives a load of known power through a **zero-crossing triac driver**: a pin change only takes effect at the next zero crossing;
- the grid current (CT1) is the scenario's surplus minus the loads that conduct - which is what the firmware measures and regulates; the diverted current (CT2) is the sum of the local loads and relays that conduct, forced ones included - which the firmware turns into the diverted energy it reports;
- an **RFM69 radio model** sits on the SPI bus (chip select D10). Remote loads follow the frames the router sends them, received by an ideal receiver or by the **`RemoteLoadReceiver` firmware** running in more simulated AVRs. Frames can be lost on purpose;
- **DS18B20 temperature sensors** can sit on a 1-Wire bus (`ds18b20.c`): the firmware reads them as on the board, and the timing of its bus I/O against the mains cycle is measured.

No hardware, no sun: regulation, load priorities, diverted energy, ISR timing and switching instants can be checked on any Linux/WSL machine, and in CI.

## What it measures

| Figure | Meaning |
|---|---|
| ADC reference | AVCC (3.3 V on the old boards) or internal 1.1 V (new board), as the firmware selects it in `ADMUX`: the analog levels follow it |
| ADC inputs | the analog input of each channel (`channels`), and the largest swing seen on each, in mV |
| ADC conversion period | must be 1664 cycles (13 ADC clocks at /128); `grid_sim` fails otherwise |
| ADC ISR average / max | cycle-exact duration of `ISR(ADC_vect)`; the longest call, with the channel of its sample (V, I1, I2) and where that sample was taken in the mains cycle (degrees from the rising zero crossing); and the longest call per channel and half cycle: the zero-crossing work runs with the V sample just after each crossing |
| ISR overruns | ISR calls longer than one conversion (104 µs): the next sample may be taken on the wrong channel |
| misfiled samples | conversions filed under the wrong channel (or on an input of no channel): the ISR writes ADMUX for the conversion after next, so an overrun leaves one conversion on the previous channel. The first two rounds after the ADC starts are ignored |
| ADMUX delay | cycles from the start of a conversion to the ISR's ADMUX write. The datasheet only allows a change one ADC clock (128 cycles) or more after the conversion started: earlier, that conversion may take the new channel. The simulated ADC locks the channel at once, so misfiled samples cannot show it; this figure does. The latest write counts too: it must come before the next conversion starts, 1664 cycles after the interrupt was raised, or the conversion after next takes the old channel. A late write (interrupts kept off too long before the ISR, or a long ISR) is listed in the events file with how long the ISR waited for its entry |
| min sample sets | as reported by the firmware's own datalog (64 per mains cycle at 50 Hz) |
| per load | switch-ons, time on, energy, and the **switching latency**: delay from the pin change to the zero crossing where the load follows, and whether it was a rising crossing (start of the measurement window) |
| per relay | switch-ons, time on, energy, and the shortest complete ON and OFF periods |
| RF, per node | frames the router sent to it, the longest gap between two of them, air time per frame, frames lost, times its receiver lost the link |
| receiver, per remote unit | frames received, frames missed because the radio was not listening |
| grid | import / export energy |
| diverted | the energy through CT2 (every local load and relay that conducted, forced ones included), and the diverted energy and power the firmware last reported (`, diverted <W> W, <Wh> Wh` of its datalog line) |
| 1-Wire | resets, presence pulses, slot errors (a low neither a 1, a 0 nor a reset); the bursts of bus activity (slots less than 2 ms apart) and the longest; where in the mains cycle the slots that keep interrupts off for long fall: a reset (70 µs from its release to its presence sample) and a 0 written (65 µs low) |
| per DS18B20 | its conversions and scratchpad reads, and the reads made before the end of a conversion (750 ms) |
| temperatures | each `T<n>` of the datalog: in how many lines, lowest and highest |

The firmware's serial output is shown with the simulated time.

## Usage (Linux / WSL)

The scenarios describe the installation (loads, surplus, inputs) and what is expected; the board
comes from a **board file** that `board_scn.sh` writes and `grid_sim -b` reads after the scenario:
its analog inputs (`channels`), its ADC reference (`avcc`), its calibration read from
`calibration.h` (`vcal`, `pcal`), the mains, and the lowest sample-set count expected. The
Makefile writes the board files in `out/`, so the same scenarios run on every board:

| Board | Inputs | Reference | Calibration (calibration.h) |
|---|---|---|---|
| `OLD_SEG` (`make check`) | V A3, CT1 A5, CT2 A4 | AVCC 3.3 V | `f_voltageCalOldBoard`, `f_powerCalOldBoard`, `f_powerCalDivertedOldBoard` |
| `OLD_OLED` (`make check-old-oled`) | V A0, CT1 A1, CT2 A3 | AVCC 3.3 V | the same |
| `NEW` (`make check-new`) | V A0, CT1 A1, CT2 A3 | internal 1.1 V | `f_voltageCalNewBoard`, `f_powerCalPerAmp` × `f_ctRating` (CT1) and × `f_ctRatingDiverted` (CT2) |

The simulator generates the ADC counts the calibration reads back as the scenario's volts and
watts, then converts them to the analog levels of the reference the firmware selected: the
calibration gives the right watts by construction, and the summary shows the swing on each input.
With the old boards' default calibration (0.05 W per step²), 230 V is ±399 counts (±1.29 V on a
3.3 V AVCC), 1 kW on a CT ±100 counts; a CT clips beyond about 5 kW (±510 counts), and the CT1
filter of the old boards (gain 9) adds about 10 % to the current it sees. The scenarios stay
within 3 kW per CT.

`configs/sim_config.sh` sets the board and the human-readable datalog (which `grid_sim` reads) in
`config.h`, whatever its defaults, and, when asked, the CT rating in `calibration.h` and a 60 Hz
mains in `config_system.h`. It also gives `config.h` **three local loads** on D5, D6 and D7
(`configs/three_loads.sh`): the shipped `config.h` has one load, but the scenarios and the patches
of `configs/` assume three, so every simulator build has three. A build that applies a patch
without `sim_config.sh` (the remote loads and SEG_HW builds of the CI matrix) runs
`configs/three_loads.sh` first. Run it before every firmware build below, and restore the sources after
each build: `restore` below puts back every file that `sim_config.sh`, the patches of `configs/`
and the build (`version.h`) change.

Build with `PVR_SIM_BUILD` set, as CI does. The firmware prints its banner at 9600 baud before it
starts sampling, so the banner's length shifts the whole run against the scenario: a different
branch name alone moves the switching instants. With `PVR_SIM_BUILD`, the build writes the branch
name a CI checkout gives (`N/A`), and the same code gives the same result everywhere.

```bash
# 1. build the firmware (OLD_SEG)
export PVR_SIM_BUILD=1
restore() { (cd Mk2_1phase_RFdatalog_temp && git checkout config.h calibration.h config_system.h version.h); }
sh sim/configs/sim_config.sh
(cd Mk2_1phase_RFdatalog_temp && pio run -e basic)
restore

# 2. run every scenario, fail on any unmet expectation
make -C sim check
make -C sim -j$(nproc) check     # the same, scenarios in parallel

# or one scenario, with the serial output (BOARD=OLD_SEG by default, OLD_OLED, NEW)
make -C sim run SCN=scenarios/clouds.scn
```

simavr comes from the PlatformIO package `platformio/tool-simavr` (installed by `pio test -e uno_sim`, or `pio pkg install -g -t platformio/tool-simavr`). Only the libelf runtime is needed besides gcc and make. Another firmware can be given with `FW=path/to/firmware.elf`.

`make run` and `make check` also write, in `out/`:
- `<scenario>-trace.csv`: one line per mains cycle - surplus, grid power, power through CT2, loads conducting (bit mask);
- `<scenario>-events.csv`: every load change, RF frame (`RF<node>` and its payload in hex) and lost RF link, with its time.

### The other boards, 20 A CTs, 60 Hz, temperature sensing, 7-segment display

Each firmware is built in a build directory of its own:

```bash
export PVR_SIM_BUILD=1      # and restore() as above
sh sim/configs/sim_config.sh OLD_OLED
(cd Mk2_1phase_RFdatalog_temp && PLATFORMIO_BUILD_DIR=.pio/build-old-oled pio run -e basic)
restore
sh sim/configs/sim_config.sh NEW
(cd Mk2_1phase_RFdatalog_temp && PLATFORMIO_BUILD_DIR=.pio/build-new pio run -e basic)
restore
sh sim/configs/sim_config.sh NEW 20
(cd Mk2_1phase_RFdatalog_temp && PLATFORMIO_BUILD_DIR=.pio/build-new20 pio run -e basic)
restore
sh sim/configs/sim_config.sh OLD_SEG 60Hz
(cd Mk2_1phase_RFdatalog_temp && PLATFORMIO_BUILD_DIR=.pio/build-60hz pio run -e basic)
restore
sh sim/configs/sim_config.sh NEW
sed -n '/^diff --git/,$p' sim/configs/temperature.patch | patch -p1
(cd Mk2_1phase_RFdatalog_temp && PLATFORMIO_BUILD_DIR=.pio/build-new-temp pio run -e basic)
restore
sh sim/configs/sim_config.sh OLD_SEG
sed -n '/^diff --git/,$p' sim/configs/seg_hw.patch | patch -p1
sed -n '/^diff --git/,$p' sim/configs/temperature.patch | patch -p1
(cd Mk2_1phase_RFdatalog_temp && PLATFORMIO_BUILD_DIR=.pio/build-seg-temp pio run -e basic)
restore
make -C sim -j$(nproc) check check-old-oled check-new check-new20 check-60hz check-new-temp check-seg-temp
make -C sim compare-boards compare-old-oled compare-boards-20 compare-ct
```

- `check-new20`: the new board with 20 A CTs (0.065 A per step, about the old boards' resolution); `sim_config.sh NEW 20` writes the rating into `f_ctRating` and `f_ctRatingDiverted`, and the board file uses it.
- `check-60hz`: `sim_config.sh ... 60Hz` sets `SUPPLY_FREQUENCY` to 60 in `config_system.h`, and the board file a 60 Hz mains (53 sample sets per cycle): the per-cycle constants of the firmware (the Wh of the diverted energy, the datalog period, the seconds) follow the frequency.
- `compare-boards` / `compare-old-oled`: each scenario must give the same result as on `OLD_SEG`: grid import and export, and the energy of each load, within 1 % or 0.1 Wh. The boards only differ in resolution (with the 50 A CTs of `calibration.h`, the new board has 0.161 A per ADC step against 0.061 A on the old ones) and in analog inputs, which can move a switching instant by a half-cycle now and then. The diverted energy is not compared: the firmware counts whole Wh, which a few hundredths of a Wh can tip.
- `compare-boards-20`: 20 A CTs against `OLD_SEG`, within 0.5 % or 0.1 Wh; `compare-ct`: 20 A against 50 A CTs, within 1 % or 0.1 Wh.
- `check-new-temp`: `NEW` with temperature sensing on D3 (`configs/temperature.patch`, built in `.pio/build-new-temp`), and the two DS18B20 of `configs/ds18b20.scn` on the bus, at 21.50 and -3.25 °C: the datalog must report both, read after their conversion. A reset and each 0 written keep interrupts off for their 1-Wire time slot (70 and 65 µs, about 1100 cycles), and the ADC interrupt waits meanwhile.
- `check-seg-temp`: `OLD_SEG` with the 7-segment display through its pin-saving chips (`configs/seg_hw.patch`: `SEG_HW`, refreshed in the ADC interrupt with the CT1 sample, loads on D10-D12) and temperature sensing (`configs/temperature.patch`) with the DS18B20 of `configs/ds18b20.scn`, built in `.pio/build-seg-temp`. Its board file remaps the scenarios' load pins D5-D7 onto D10-D12 (`remap`). The display is refreshed on the CT1 call. It is the tightest build for the ADC deadline: an ISR call entered up to ~1200 cycles late, after a 1-Wire time slot, still has to write ADMUX within the 1664 cycles of its conversion. The work of the mains cycle is spread over the CT2 calls that follow the zero crossings (`processing.cpp`, `CycleTask`), and the firmware starts each 1-Wire transaction right after the positive crossing's work, one per mains cycle (`temp_schedule.h`): the slots fall at 28-118 deg (34-146 deg at 60 Hz), among the ordinary calls, and the conversion that starts each round runs before the datalog output, whose UART interrupts would add to the delay. On the old boards the CT1 call is never light (its filter, and here the display refresh: up to ~400 cycles to ADMUX), and a slot can delay it: ~1480 cycles seen, ~1600 possible, against ~1400 seen on `NEW`, whose ordinary calls need ~276 cycles at most (~1480 possible). Their board files enforce it: `admux_delay_max` 1600 for `check-seg-temp`, 1480 for `check-new-temp`.

### Remote loads

The scenarios in `scenarios/rf` need a firmware with remote loads: `configs/remote_loads.patch` applied to `config.h`, built in a build directory of its own (loads 3 and 4 go to remote units 1 and 2, RF nodes 15 and 16; the RFM69 takes D2 and D10 to D13).

```bash
export PVR_SIM_BUILD=1      # and restore() as above
sh sim/configs/sim_config.sh
sed -n '/^diff --git/,$p' sim/configs/remote_loads.patch | patch -p1   # or git apply --unidiff-zero
(cd Mk2_1phase_RFdatalog_temp && PLATFORMIO_BUILD_DIR=.pio/build-rf pio run -e basic)
restore
make -C sim check-rf                                   # with ideal receivers
make -C sim check-rf RX_NODES="15 16"                  # with the receiver firmware
```

The radio model implements what the LowPowerLab library uses in packet mode:
- the register file;
- the operating modes;
- a FIFO that goes on air when the chip enters TX mode.

The PacketSent flag rises after the frame's real air time at the programmed bit rate, so `send()` blocks the main loop as long as it does on the board. The channel always looks free.

A frame that has been sent reaches every other radio. A radio that is listening (RX mode) on the same network gets it in its FIFO, with PayloadReady set and DIO0 (D2, INT0) raised, so the receiving side of the library runs unchanged. A radio that is not listening misses it, as a real one would.

Each remote unit is either:
- **an ideal receiver**, when no firmware is given for its node (the default): its loads take their bit from each frame once it has been sent, and switch off when no frame has come for `rf_timeout` seconds (500 ms, like the receiver firmware);
- **the `RemoteLoadReceiver` firmware** (`grid_sim -r <node>=<receiver.elf>`), in its own simulated AVR, running in step with the router on the same clock. Its load pins (`receiver_pins`) drive its remote loads, and its serial output is shown, prefixed with `R<node>|`. Its sources are in `RemoteLoadReceiver/`, at the root of this repository. `build_receivers.sh` builds it once per node ID, since `REMOTE_NODE_ID` is a constant of its `config_rf.h`; `make check-rf RX_NODES=...` does this for the nodes given, from the sources in `RX_SRC` (`../RemoteLoadReceiver` by default).

Frames get lost in `rf_drop` windows, and at random with `rf_loss`, from a fixed seed, so the same frames are lost on every run of the same firmware. Which frames the sequence falls on still depends on the firmware's timing; a limit on the losses in a row keeps the longest gap independent of it.

### Relays

The scenarios in `scenarios/relays` need a firmware with relay diversion: `configs/relays.patch` puts two relays on D8 and D9 (surplus thresholds 500 and 1000 W, import threshold 100 W, 2 minutes minimum ON and OFF) with a 1-minute filter, override pins D4 (everything) and D13 (the relays). It is built in a build directory of its own, so the `basic` firmware of `make check` stays.

```bash
export PVR_SIM_BUILD=1      # and restore() as above
sh sim/configs/sim_config.sh
sed -n '/^diff --git/,$p' sim/configs/relays.patch | patch -p1   # or git apply --unidiff-zero
(cd Mk2_1phase_RFdatalog_temp && PLATFORMIO_BUILD_DIR=.pio/build-relays pio run -e basic)
restore
make -C sim check-relays
```

A relay's contacts follow its pin at once. The firmware decides on the grid power of each datalog line (every 5 s) through its TEMA filter, waits 1 minute after any relay change, and holds each relay for its minimum ON or OFF time - from power-up for the first switch-on. A relay scenario therefore lasts several minutes; the minimum times are longer than the settle time, so that a firmware ignoring them fails.

### In CI

The scenarios are spread over parallel jobs. `ci_targets.sh` gives the make targets of a job, from the scenarios it finds: job k of n takes every n-th scenario, starting from the k-th. A grid scenario runs on all its boards in the same job (`OLD_SEG`, `OLD_OLED`, `NEW`, `NEW` with 20 A CTs, `OLD_SEG` at 60 Hz, `NEW` with temperature sensing, `OLD_SEG` with the 7-segment display and temperature sensing), which then compares them. The workflow (`.github/workflows/build.yml`, job `grid-sim`) builds each of these firmwares as above, with `git apply --unidiff-zero` for the patches.

```bash
sim/ci_targets.sh grid 1/3      # check-clouds check-old-oled-clouds check-new-clouds check-new20-clouds check-60hz-clouds ...
sim/ci_targets.sh relays 2/3    # check-relays-relay_forced check-relays-relay_override_settle ...
```

With more scenarios than jobs, a job runs several of them in parallel. A new scenario needs no change to the workflows; more jobs, only their `part` list.

## Scenarios

Plain text, one command per line, `#` starts a comment.

| Command | Meaning |
|---|---|
| `include <file>` | read another scenario file (relative path) |
| `duration <s>` | simulated time |
| `mains <Hz> <Vrms>` | grid frequency and voltage (board file) |
| `channels V <pin> I1 <pin> I2 <pin>` | the analog input of the voltage, of CT1 (grid) and of CT2 (diverted), `A3` or `3` (board file) |
| `remap <pin> <pin>` | the firmware drives the scenario's output or reads its input on another pin: loads, inputs and pin expectations follow (board file) |
| `avcc <mV>` | AVCC, the ADC reference of the old boards (board file) |
| `vcal <V per step>` / `pcal <CT1> [<CT2>]` | `f_voltageCal` / `f_powerCal` of each CT (CT2 as CT1 when not given) (board file) |
| `load <pin> <W>` | a resistive load on a firmware output pin; CT2 measures it |
| `relay <pin> <W>` | a resistive load on a relay: it follows the pin at once, with no zero-crossing latching; CT2 measures it |
| `remote <node> <bit> <W>` | a resistive load on a remote unit, driven by bit `bit` of the frames sent to RF node `node`; not on CT2 |
| `receiver_pins <pin>...` | receiver firmware: the output pin of each payload bit, bit 0 first (`loadPins` in its `config.h`) |
| `rf_timeout <s>` | ideal receivers: link timeout (default 0.5 s) |
| `rf_drop <t0> <t1> [<node>]` | frames sent in this window (to this node, or to all) are lost |
| `rf_loss <percent> [<max>]` | share of frames lost at random, over the whole run; at most `<max>` in a row per node |
| `rf_absent` | the router's RFM69 does not answer (missing, unpowered or badly wired: every SPI read gives 0) |
| `ds18b20 <pin> <°C> <ROM code>` | a DS18B20 on the 1-Wire bus of the pin, at that temperature; its ROM code in 8 hex bytes (`28 BE 41 6B 09 00 00 A4`). Until its first conversion, it holds the power-on value 85.00 °C |
| `input <pin> <0\|1> [<t>]` | level of an input pin from time t on (default 0): the latest line at or before the current time wins; unconnected inputs read 1, like the pull-ups |
| `at <t> <W>` | surplus in W (> 0 = export) at time t; linear in between, repeat a time for a step |
| `expect <t> pin <p> on\|off` | state of an output pin at time t |
| `expect <t> remote <node> <bit> on\|off` | state of a remote load at time t |
| `expect rf_max_gap <node> <s>` | frames to the node never further apart than this, from its first frame to the end |
| `expect rf_links_lost <node> <min> <max>` | times the node's receiver lost the RF link (its loads switched off) |
| `expect switches <pin> <min> <max>` | switch-ons of a load or relay over the run |
| `expect min_on <pin> <s>` / `expect min_off <pin> <s>` | shortest complete ON period / OFF period between two ON ones, at least |
| `expect <t0> <t1> grid_avg <min> <max>` | average grid power (import > 0) over a window |
| `expect diverted_wh <min> <max>` | the diverted energy the firmware last reported, for an energy within [min, max]: it counts whole Wh, so from floor(min) to floor(max) Wh pass |
| `expect <t0> <t1> diverted_wh <min> <max>` | its increase, from the last line before t0 to the last line printed by t1, compared as it is |
| `expect [<t0> <t1>] diverted_w_avg <min> <max>` | average of the diverted power the firmware reports, over every datalog line or those printed within the window (each line averages the 5 s before it, forced cycles counting as 0 W) |
| `expect min_sample_sets <n>` | lowest sample-set count reported by the firmware (board file) |
| `expect isr_overruns <n>` | at most n ISR calls longer than one conversion |
| `expect misfiled_samples <n>` | at most n conversions filed under the wrong channel |
| `expect isr_max <cycles>` | ISR duration never above this |
| `expect admux_delay_min <cycles>` | the ISR never writes ADMUX sooner than this after its conversion started |
| `expect admux_delay_max <cycles>` | ... nor later than this (the deadline is 1664) |
| `expect clipped <n>` | at most n samples beyond the ADC's range |
| `expect ds18b20_early_reads <n>` | at most n scratchpads read before the end of their conversion |
| `expect temperature <n> <min> <max>` | every `T<n>` the datalog reports lies within [min, max], and at least one line reports it |
| `expect voltage <min> <max>` | every `V` the datalog reports lies within [min, max], and at least one line reports it (`common.scn`: 220 to 240 V on the 230 V mains of the board files) |

`scenarios/common.scn` describes the installation of the default `config.h` (three local loads on D5-D7, override pin D4) and the timing expected on every run (no misfiled sample, ADMUX written 128 cycles or more after its conversion started, no ISR overrun, no clipped sample), with margins enforced: the longest ISR call and the latest ADMUX write must stay a little above the worst measured on the builds that run them (`isr_max`, `admux_delay_max`), so that a change eating the headroom fails before samples are lost; the other scenarios include it. The ADMUX margin of the grid scenarios, 1620 cycles, leaves room for the ordinary shifts of the 1-Wire time slots against the ADC phase in the temperature builds: a failure between 1620 and the 1664-cycle deadline, with `misfiled_samples` 0, is a margin warning, not a lost sample. `scenarios/rf/common.scn` and `scenarios/relays/common.scn` do the same for the remote-loads and relay configurations. The relay and RF scenarios are those of the 3-phase router, every power halved (relay thresholds included) to stay within the range of one CT.

The firmware's datalog starts about 11 s after power-up (banner, then the start-up period), and reports every 5 s: a diverted energy is only seen once a line follows it. When the surplus comes, the energy bucket first fills up to half its 360 J (exported, 0.05 Wh), and it empties when the surplus goes: the diverted energy is the surplus × time less the export target, within those half-bucket transients.

## Limits

- **Firmware configuration**: the scenarios match the default `config.h` (3 local loads on D5-D7, override pin D4), `scenarios/rf` matches `configs/remote_loads.patch` and `scenarios/relays` matches `configs/relays.patch`. Another configuration needs its own `common.scn`.
- **Analog front end**: the counts follow the calibration, not a model of the CTs, burdens and dividers of each board; the summary shows the resulting swing on each input.
- **RF**: a frame either arrives intact or is lost; no range, interference, collisions or corrupted frames. The channel always looks free to carrier sense.
- **Ideal signals**: pure sine waves, no ADC noise, no CT phase error or high-pass behaviour (which the CT1 filter of the old boards offsets), no harmonics.
- **Power stage**: an ideal zero-crossing triac driver, resistive loads; relays switch at once, with no bounce.
- **Timing**: simavr executes the real instructions with their datasheet cycle counts; peripherals are modelled, not exact.
- **DS18B20**: Skip ROM, Match ROM, Convert T and Read Scratchpad only, with a fixed temperature, a 750 ms conversion and no parasite power. The bus is ideal: no noise, no rise time. Search ROM (the calibration mode lists the sensors with it) is not modelled.
- **ADC reference**: the board file's `avcc` cancels out (the simulator turns the counts it wants into millivolts on the reference the firmware selected, which the ADC turns back into the same counts), so a firmware that selects the wrong reference still reads the right counts: the simulator cannot catch it. The native tests of `adc_chain.h` check the ADMUX of each board.
- **ADC pipeline**: the depth of the ADMUX pipeline (written for the conversion after next) is only checked through behaviour: a wrong depth files samples under the wrong channel (`misfiled_samples`) and breaks the regulation.
- **Diverted energy bands**: the firmware counts whole Wh and `expect diverted_wh` floors both ends of its band, so a whole-run band is effectively about ±9 % around its middle on these short scenarios, wider than the ±5 % it is written with.

## simavr ADC fix

simavr's ADC does not time free-running conversions correctly: with this firmware it converts every ~256 CPU cycles instead of 1664. `avr_adc.c` is a copy of upstream's with that fix; it is linked ahead of `libsimavr.a`, so it replaces the library's ADC. `grid_sim` checks the conversion period on every run.
