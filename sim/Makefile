# grid_sim - runs the PVRouter firmware on a simulated single-phase grid (Linux / WSL)
#
#   make                build grid_sim
#   make check          run every scenario against the OLD_SEG firmware, fail on any expectation
#   make check-old-oled the same scenarios against the OLD_OLED firmware (other analog inputs)
#   make check-new      the same against the NEW firmware (internal 1.1 V reference)
#   make check-new20    the same with 20 A CTs: about the resolution of the old boards
#   make check-60hz     the same against an OLD_SEG firmware built for 60 Hz, on a 60 Hz mains
#   make check-seg-temp the same against OLD_SEG with the 7-segment display (SEG_HW, loads on D10-D12)
#                       and temperature sensing (configs/seg_hw.patch, configs/temperature.patch)
#   make check-new-temp the same against NEW with temperature sensing (configs/temperature.patch)
#   make check-rf       run scenarios/rf against the remote-loads firmware (configs/remote_loads.patch),
#                       with ideal receivers, or with the RemoteLoadReceiver firmware on the units
#                       of RX_NODES (RX_NODES="15 16", its sources in RX_SRC)
#   make check-relays   run scenarios/relays against the relay firmware (configs/relays.patch)
#   make compare-boards after 'make check check-new': each scenario must give the same result
#                       on both boards (grid, load and diverted energies within 1 % or 0.1 Wh)
#   make compare-old-oled     after 'make check check-old-oled': the same
#   make compare-boards-20    after 'make check check-new20': the same within 0.5 % or 0.1 Wh
#   make compare-ct     after 'make check-new check-new20': 20 A against 50 A CTs, within 1 % or 0.1 Wh
#   make run SCN=scenarios/surplus_step.scn [BOARD=OLD_SEG]    run one scenario, serial output shown
#
# Every scenario is a target of its own (check-<name>, check-old-oled-<name>, check-new-<name>,
# check-rf-<name>, check-relays-<name>...): with -j$(nproc), they run in parallel, and
# 'make -j$(nproc) check check-rf' runs both sets together.
#
# The board (analog inputs, ADC reference, calibration from calibration.h, mains) is given to
# grid_sim by a board file, written by board_scn.sh: the scenarios describe the installation only.
#
# The firmware is built separately, as CI does (README.md, Usage):
#   export PVR_SIM_BUILD=1
#   sh sim/configs/sim_config.sh   # human-readable output (grid_sim reads it), OLD_SEG board
#   (cd Mk2_1phase_RFdatalog_temp && pio run -e basic)
# The other firmwares are built in build directories of their own (README.md).

SIMAVR ?= $(HOME)/.platformio/packages/tool-simavr
PIO_DIR ?= ../Mk2_1phase_RFdatalog_temp/.pio
FW          ?= $(PIO_DIR)/build/basic/firmware.elf
FW_OLD_OLED ?= $(PIO_DIR)/build-old-oled/basic/firmware.elf
FW_NEW      ?= $(PIO_DIR)/build-new/basic/firmware.elf
FW_NEW20    ?= $(PIO_DIR)/build-new20/basic/firmware.elf
FW_60HZ     ?= $(PIO_DIR)/build-60hz/basic/firmware.elf
FW_SEG_TEMP ?= $(PIO_DIR)/build-seg-temp/basic/firmware.elf
FW_NEW_TEMP ?= $(PIO_DIR)/build-new-temp/basic/firmware.elf
FW_RF       ?= $(PIO_DIR)/build-rf/basic/firmware.elf
FW_RELAYS   ?= $(PIO_DIR)/build-relays/basic/firmware.elf
CT_LOW  ?= 20
CAL    ?= ../Mk2_1phase_RFdatalog_temp/calibration.h
SCN    ?= scenarios/surplus_step.scn
BOARD  ?= OLD_SEG
OUT    ?= out
# remote units running the receiver firmware in scenarios/rf (node IDs of configs/remote_loads.patch):
# none by default, the RemoteLoadReceiver sources are not in this repository
RX_NODES ?=
RX_SRC   ?= ../RemoteLoadReceiver
RX_DIR   ?= $(OUT)/receivers
RX_ARGS   = $(foreach n,$(RX_NODES),-r $(n)=$(RX_DIR)/receiver-$(n).elf)

CFLAGS  ?= -O2 -g
CFLAGS  += -std=gnu11 -I$(SIMAVR)/include/simavr
# the vendored avr_adc.o comes first: libsimavr.a's own ADC is then never linked in.
# -l:libelf.so.1 only needs the libelf runtime, not the -dev package.
LDLIBS  = $(SIMAVR)/lib/libsimavr.a -l:libelf.so.1 -lm

SCENARIOS    := $(filter-out scenarios/common.scn,$(wildcard scenarios/*.scn))
RF_SCENARIOS := $(filter-out scenarios/rf/common.scn,$(wildcard scenarios/rf/*.scn))
RELAY_SCENARIOS := $(filter-out scenarios/relays/common.scn,$(wildcard scenarios/relays/*.scn))

grid_sim: grid_sim.o rfm69.o avr_adc.o
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

grid_sim.o: grid_sim.c rfm69.h
	$(CC) $(CFLAGS) -Wall -Wextra -c -o $@ $<

rfm69.o: rfm69.c rfm69.h
	$(CC) $(CFLAGS) -Wall -Wextra -c -o $@ $<

# upstream code, built as is
avr_adc.o: avr_adc.c
	$(CC) $(CFLAGS) -w -c -o $@ $<

$(FW):
	$(error firmware not found: $(FW) - build it first: export PVR_SIM_BUILD=1, run sim/configs/sim_config.sh from the repository root, then 'pio run -e basic' (README.md, Usage))

$(FW_OLD_OLED):
	$(error firmware not found: $(FW_OLD_OLED) - see check-old-oled in README.md)

$(FW_NEW):
	$(error firmware not found: $(FW_NEW) - see check-new in README.md)

$(FW_NEW20):
	$(error firmware not found: $(FW_NEW20) - see check-new20 in README.md)

$(FW_60HZ):
	$(error firmware not found: $(FW_60HZ) - see check-60hz in README.md)

$(FW_SEG_TEMP):
	$(error firmware not found: $(FW_SEG_TEMP) - see configs/seg_hw.patch)

$(FW_NEW_TEMP):
	$(error firmware not found: $(FW_NEW_TEMP) - see configs/temperature.patch)

$(FW_RF):
	$(error firmware not found: $(FW_RF) - apply configs/remote_loads.patch, then 'PLATFORMIO_BUILD_DIR=.pio/build-rf pio run -e basic' (README.md))

$(FW_RELAYS):
	$(error firmware not found: $(FW_RELAYS) - see configs/relays.patch)

# the board files: board_scn.sh <board> <calibration.h> [<CT rating> [<frequency>]]
$(OUT)/old_seg_board.scn: board_scn.sh $(CAL)
	@mkdir -p $(OUT)
	sh board_scn.sh OLD_SEG $(CAL) > $@

$(OUT)/old_oled_board.scn: board_scn.sh $(CAL)
	@mkdir -p $(OUT)
	sh board_scn.sh OLD_OLED $(CAL) > $@

$(OUT)/new_board.scn: board_scn.sh $(CAL)
	@mkdir -p $(OUT)
	sh board_scn.sh NEW $(CAL) > $@

$(OUT)/new20_board.scn: board_scn.sh $(CAL)
	@mkdir -p $(OUT)
	sh board_scn.sh NEW $(CAL) $(CT_LOW) > $@

$(OUT)/old_seg_60hz_board.scn: board_scn.sh $(CAL)
	@mkdir -p $(OUT)
	sh board_scn.sh OLD_SEG $(CAL) '' 60 > $@

# OLD_SEG with SEG_HW: the loads of the scenarios (D5-D7) are on D10-D12 (configs/seg_hw.patch)
$(OUT)/old_seg_hw_board.scn: board_scn.sh $(CAL)
	@mkdir -p $(OUT)
	{ sh board_scn.sh OLD_SEG $(CAL); printf 'remap 5 10\nremap 6 11\nremap 7 12\n'; } > $@

# make run: the board file and the firmware of BOARD
RUN_BOARD = $(OUT)/$(shell echo $(BOARD) | tr A-Z a-z)_board.scn
RUN_FW = $(if $(filter NEW,$(BOARD)),$(FW_NEW),$(if $(filter OLD_OLED,$(BOARD)),$(FW_OLD_OLED),$(FW)))

run: grid_sim $(RUN_FW) $(RUN_BOARD)
	@mkdir -p $(OUT)
	./grid_sim -t $(OUT)/$(notdir $(SCN:.scn=))-trace.csv -e $(OUT)/$(notdir $(SCN:.scn=))-events.csv -b $(RUN_BOARD) $(RUN_FW) $(SCN)

CHECK    := $(SCENARIOS:scenarios/%.scn=check-%)
CHECK_OLD_OLED := $(SCENARIOS:scenarios/%.scn=check-old-oled-%)
CHECK_NEW := $(SCENARIOS:scenarios/%.scn=check-new-%)
CHECK_NEW20 := $(SCENARIOS:scenarios/%.scn=check-new20-%)
CHECK_60HZ := $(SCENARIOS:scenarios/%.scn=check-60hz-%)
CHECK_SEG_TEMP := $(SCENARIOS:scenarios/%.scn=check-seg-temp-%)
CHECK_NEW_TEMP := $(SCENARIOS:scenarios/%.scn=check-new-temp-%)
CHECK_RF := $(RF_SCENARIOS:scenarios/rf/%.scn=check-rf-%)
CHECK_RELAYS := $(RELAY_SCENARIOS:scenarios/relays/%.scn=check-relays-%)

check: $(CHECK)

check-old-oled: $(CHECK_OLD_OLED)

check-new: $(CHECK_NEW)

check-new20: $(CHECK_NEW20)

check-60hz: $(CHECK_60HZ)

check-seg-temp: $(CHECK_SEG_TEMP)

check-new-temp: $(CHECK_NEW_TEMP)

check-rf: $(CHECK_RF)

check-relays: $(CHECK_RELAYS)

# read the summaries of the last 'make check check-new' / 'make check check-old-oled' / ...
compare-boards:
	sh compare_boards.sh $(OUT) new 1 0.1

compare-old-oled:
	sh compare_boards.sh $(OUT) old-oled 1 0.1

compare-boards-20:
	sh compare_boards.sh $(OUT) new20 0.5 0.1

compare-ct:
	sh compare_boards.sh $(OUT) new20 1 0.1 new

# the summary is shown once the scenario is over: parallel runs don't mix their lines
# $(call scenario,<firmware, board file and receivers>,<scenario file>,<output name>)
define scenario
	@mkdir -p $(OUT)
	@./grid_sim -q -t $(OUT)/$(3)-trace.csv -e $(OUT)/$(3)-events.csv $(1) $(2) > $(OUT)/$(3)-summary.txt; \
	  status=$$?; cat $(OUT)/$(3)-summary.txt; exit $$status
endef

$(CHECK): check-%: grid_sim $(FW) $(OUT)/old_seg_board.scn
	$(call scenario,-b $(OUT)/old_seg_board.scn $(FW),scenarios/$*.scn,$*)

$(CHECK_OLD_OLED): check-old-oled-%: grid_sim $(FW_OLD_OLED) $(OUT)/old_oled_board.scn
	$(call scenario,-b $(OUT)/old_oled_board.scn $(FW_OLD_OLED),scenarios/$*.scn,old-oled-$*)

$(CHECK_NEW): check-new-%: grid_sim $(FW_NEW) $(OUT)/new_board.scn
	$(call scenario,-b $(OUT)/new_board.scn $(FW_NEW),scenarios/$*.scn,new-$*)

$(CHECK_NEW20): check-new20-%: grid_sim $(FW_NEW20) $(OUT)/new20_board.scn
	$(call scenario,-b $(OUT)/new20_board.scn $(FW_NEW20),scenarios/$*.scn,new20-$*)

$(CHECK_60HZ): check-60hz-%: grid_sim $(FW_60HZ) $(OUT)/old_seg_60hz_board.scn
	$(call scenario,-b $(OUT)/old_seg_60hz_board.scn $(FW_60HZ),scenarios/$*.scn,60hz-$*)

$(CHECK_SEG_TEMP): check-seg-temp-%: grid_sim $(FW_SEG_TEMP) $(OUT)/old_seg_hw_board.scn
	$(call scenario,-b $(OUT)/old_seg_hw_board.scn $(FW_SEG_TEMP),scenarios/$*.scn,seg_temp-$*)

$(CHECK_NEW_TEMP): check-new-temp-%: grid_sim $(FW_NEW_TEMP) $(OUT)/new_board.scn
	$(call scenario,-b $(OUT)/new_board.scn $(FW_NEW_TEMP),scenarios/$*.scn,new_temp-$*)

$(CHECK_RF): check-rf-%: grid_sim $(FW_RF) $(OUT)/old_seg_board.scn $(if $(RX_NODES),receivers)
	$(call scenario,-b $(OUT)/old_seg_board.scn $(RX_ARGS) $(FW_RF),scenarios/rf/$*.scn,rf-$*)

$(CHECK_RELAYS): check-relays-%: grid_sim $(FW_RELAYS) $(OUT)/old_seg_board.scn
	$(call scenario,-b $(OUT)/old_seg_board.scn $(FW_RELAYS),scenarios/relays/$*.scn,relays-$*)

receivers:
	bash build_receivers.sh $(RX_SRC) $(RX_DIR) $(RX_NODES)

clean:
	rm -rf grid_sim *.o $(OUT)

.PHONY: run check check-old-oled check-new check-new20 check-60hz check-seg-temp check-new-temp check-rf check-relays compare-boards compare-old-oled \
	compare-boards-20 compare-ct $(CHECK) $(CHECK_OLD_OLED) $(CHECK_NEW) $(CHECK_NEW20) $(CHECK_60HZ) $(CHECK_SEG_TEMP) $(CHECK_NEW_TEMP) $(CHECK_RF) \
	$(CHECK_RELAYS) receivers clean
