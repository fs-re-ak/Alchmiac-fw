# Host (PC) tests for the firmware: the real sources are compiled with a native
# gcc and linked against fakes.c (simulated HAL SPI/GPIO, ADS1118, ADS1299, BLE).
#
#   make            build and run the tests
#   make clean
#
# On Windows, any MinGW gcc works (e.g. the one bundled with CLion:
#   PATH="<CLion>/bin/mingw/bin:$PATH" mingw32-make).

ifeq ($(origin CC),default)
CC      := gcc
endif
FW      := ../../fw
BUILD   := build

INCLUDES := \
	-I. \
	-I$(FW)/Core/Inc \
	-I$(FW)/Drivers/CMSIS/Device/ST/STM32WBxx/Include \
	-I$(FW)/Drivers/CMSIS/Include \
	-I$(FW)/Drivers/STM32WBxx_HAL_Driver/Inc \
	-I$(FW)/Drivers/STM32WBxx_HAL_Driver/Inc/Legacy \
	-I$(FW)/Middlewares/ST/STM32_WPAN \
	-I$(FW)/Middlewares/ST/STM32_WPAN/ble \
	-I$(FW)/Middlewares/ST/STM32_WPAN/ble/core \
	-I$(FW)/Middlewares/ST/STM32_WPAN/ble/core/auto \
	-I$(FW)/Middlewares/ST/STM32_WPAN/ble/core/template \
	-I$(FW)/Middlewares/ST/STM32_WPAN/ble/svc/Inc \
	-I$(FW)/Middlewares/ST/STM32_WPAN/ble/svc/Src \
	-I$(FW)/Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread \
	-I$(FW)/Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci \
	-I$(FW)/Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl \
	-I$(FW)/Middlewares/ST/STM32_WPAN/utilities \
	-I$(FW)/STM32_WPAN/App \
	-I$(FW)/Utilities/lpm/tiny_lpm \
	-I$(FW)/Utilities/sequencer

DEFINES := -DDEBUG -DSTM32WB5Mxx -DUSE_HAL_DRIVER
# The CMSIS/HAL headers target a 32-bit ARM core: silence their warnings on the PC.
CFLAGS  := -std=gnu11 -O0 -g $(DEFINES) $(INCLUDES) -w

SRCS := test_ppg.c fakes.c $(FW)/Core/Src/ads1118.c $(FW)/STM32_WPAN/App/p2p_server_app.c

$(BUILD)/test_ppg.exe: $(SRCS) fakes.h $(FW)/Core/Inc/ads1118.h $(FW)/STM32_WPAN/App/hermes_ble.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(SRCS) -o $@

.PHONY: test clean
test: $(BUILD)/test_ppg.exe
	./$(BUILD)/test_ppg.exe

.DEFAULT_GOAL := test

clean:
	rm -rf $(BUILD)
