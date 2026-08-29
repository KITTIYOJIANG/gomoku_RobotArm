PROJECT := jibot1_stage1_safe
BUILD := build

PREFIX ?= arm-none-eabi-
CC := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE := $(PREFIX)size

CPU := -mcpu=cortex-m3 -mthumb
DEFINES := -DUSE_STDPERIPH_DRIVER -DSTM32F10X_MD -DSAFE_STAGE1
INCLUDES := \
	-ICMSIS/CM3/CoreSupport \
	-ICMSIS/CM3/DeviceSupport/ST/STM32F10x \
	-ILibraries/inc \
	-Isrc \
	-IUSER

CFLAGS := $(CPU) $(DEFINES) $(INCLUDES) -std=gnu99 -Os -g3 \
	-ffunction-sections -fdata-sections -fno-common \
	-Wall -Wextra -Wshadow -Wformat=2
ASFLAGS := $(CPU) $(DEFINES) $(INCLUDES) -x assembler-with-cpp
LDFLAGS := $(CPU) -Tstm32f103c8.ld --specs=nano.specs --specs=nosys.specs \
	-Wl,--gc-sections -Wl,-Map=$(BUILD)/$(PROJECT).map -Wl,--cref

C_SOURCES := \
	CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c \
	Libraries/src/misc.c \
	Libraries/src/stm32f10x_gpio.c \
	Libraries/src/stm32f10x_iwdg.c \
	Libraries/src/stm32f10x_rcc.c \
	Libraries/src/stm32f10x_tim.c \
	Libraries/src/stm32f10x_usart.c \
	src/z_delay.c \
	src/z_global.c \
	src/z_gpio.c \
	src/z_timer.c \
	src/z_usart.c \
	USER/stm32f10x_it.c \
	USER/safe_main.c \
	USER/arm_safety.c \
	USER/safe_console.c \
	USER/servo_bus.c

ASM_SOURCES := Libraries/Startup/gcc_ride7/startup_stm32f10x_md.s
OBJECTS := $(addprefix $(BUILD)/,$(C_SOURCES:.c=.o)) \
	$(addprefix $(BUILD)/,$(ASM_SOURCES:.s=.o))
DEPS := $(OBJECTS:.o=.d)

ifeq ($(OS),Windows_NT)
MKDIR = powershell.exe -NoProfile -Command "New-Item -ItemType Directory -Force -Path '$(dir $@)' | Out-Null"
CLEAN_BUILD = powershell.exe -NoProfile -Command "if (Test-Path -LiteralPath '$(BUILD)') { Remove-Item -LiteralPath '$(BUILD)' -Recurse -Force }"
else
MKDIR = mkdir -p $(dir $@)
CLEAN_BUILD = rm -rf $(BUILD)
endif

.PHONY: all clean size

all: $(BUILD)/$(PROJECT).elf $(BUILD)/$(PROJECT).hex $(BUILD)/$(PROJECT).bin size

$(BUILD)/$(PROJECT).elf: $(OBJECTS) stm32f103c8.ld
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD)/$(PROJECT).hex: $(BUILD)/$(PROJECT).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD)/$(PROJECT).bin: $(BUILD)/$(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/%.o: %.c
	@$(MKDIR)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/%.o: %.s
	@$(MKDIR)
	$(CC) $(ASFLAGS) -MMD -MP -c $< -o $@

size: $(BUILD)/$(PROJECT).elf
	$(SIZE) $<

clean:
	$(CLEAN_BUILD)

-include $(DEPS)
