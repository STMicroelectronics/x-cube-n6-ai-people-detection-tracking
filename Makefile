######################################
# quiet mode
######################################
V = 0
ifeq ($(V), 0)
  quiet = quiet_
else
  quiet =
endif
quiet_CC  = @echo "  CC $@"; $(CC)
quiet_LD  = @echo "  LD $@"; $(CC)
quiet_AS  = @echo "  AS $@"; $(AS)
quiet_SZ  = @echo "  SZ $@"; $(SZ)
quiet_HEX  = @echo "  HEX $@"; $(HEX)
quiet_BIN  = @echo "  BIN $@"; $(BIN)

######################################
# helpers
######################################
rwildcard=$(foreach d,$(wildcard $(1:=/*)),$(call rwildcard,$d,$2) $(filter $(subst *,%,$2),$d))

######################################
# target
######################################
TARGET = Project
 # Supported Options: VD66GY; IMX335; VD55G1; VD1943
SENSOR_LIST = IMX335 VD66GY VD55G1 VD1943

 # Supported Options: STM32N6570-DK; NUCLEO-N657X0-Q
BOARD ?= STM32N6570-DK

define is_nucleo
$(if $(filter $(BOARD),NUCLEO-N657X0-Q),1,0)
endef

######################################
# building variables
######################################
OPT = -Os -g3

#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES += Src/main.c
C_SOURCES += Src/app.c
C_SOURCES += Src/utils.c
C_SOURCES += Src/app_fuseprogramming.c
C_SOURCES += Src/stm32_lcd_ex.c
C_SOURCES += Src/stm32n6xx_it.c
C_SOURCES += Model/$(BOARD)/network.c
C_SOURCES += Model/$(BOARD)/stai_network.c
C_SOURCES += Src/app_cam.c
C_SOURCES += Src/freertos_bsp.c

# ASM sources
ASM_SOURCES =
ASM_SOURCES_S =

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
READELF = $(GCC_PATH)/$(PREFIX)readelf
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
CP = $(PREFIX)objcopy
READELF = $(PREFIX)readelf
endif
LD = $(CC)
HEX = $(CP) -O ihex
BIN = $(CP) -O binary

#######################################
# CFLAGS
#######################################
CPU = -mcpu=cortex-m55 -mcmse -mthumb
FPU = -mfpu=fpv5-d16 -mfloat-abi=hard

# mcu
MCU = $(CPU) $(FPU)

# C defines
C_DEFS += -DSTM32N657xx
C_DEFS += -DUSE_FULL_ASSERT
C_DEFS += -DUSE_FULL_LL_DRIVER
C_DEFS += -DVECT_TAB_SRAM

C_DEFS += $(foreach SENSOR, $(SENSOR_LIST), -DUSE_$(SENSOR)_SENSOR)

ifdef APP_VERSION_STRING
C_DEFS += -DAPP_VERSION_STRING=\"$(APP_VERSION_STRING)\"
endif

ifeq ($(call is_nucleo),1)
C_DEFS += -DSTM32N6570_NUCLEO_REV
else
C_DEFS += -DSTM32N6570_DK_REV
endif

# We only support single model
C_DEFS += -DTX_MAX_PARALLEL_NETWORKS=1

# C includes
# Patched files
C_INCLUDES += -IInc -IModel/$(BOARD)


ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fstack-usage -fdata-sections -ffunction-sections -fcyclomatic-complexity
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fstack-usage -fdata-sections -ffunction-sections -fcyclomatic-complexity
# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

CFLAGS += -std=gnu11

#######################################
# LDFLAGS
#######################################
# link script
ifeq ($(call is_nucleo),1)
LDSCRIPT = Gcc/STM32N657xx_nucleo.ld
else
LDSCRIPT = Gcc/STM32N657xx.ld
endif

# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) $(LDFLAGS_OTHERS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections
# Uncomment to enable %f formatted output
LDFLAGS_OTHERS += -u _printf_float
# Avoid 'build/Project.elf has a LOAD segment with RWX permissions' warning
LDFLAGS_OTHERS += -Wl,--no-warn-rwx-segments
LDFLAGS += -Wl,--print-memory-usage

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# Include mk files
#######################################
include mks/fw.mk
include mks/ai.mk
include mks/cmw.mk
include mks/freertos.mk
include mks/gcc.mk
include mks/iar.mk
# force UVCL for nucleo
ifeq ($(call is_nucleo),1)
SCR_LIB_SCREEN_ITF ?= UVCL
#SCR_LIB_SCREEN_ITF ?= SPI
else
SCR_LIB_SCREEN_ITF ?= LTDC
endif
SCR_USBX_REL_DIR := STM32Cube_FW_N6/Middlewares/ST/usbx
SCR_LIB_RTOS := FREERTOS
include Lib/screenl/scr_lib.mk
ifeq ($(call is_nucleo),0)
include Lib/tracker/tracker.mk
endif

#######################################
# build the application
#######################################
OBJECTS = $(addprefix $(BUILD_DIR)/, $(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES_S:.S=.o))


$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	$($(quiet)CC)  -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	$($(quiet)CC)  -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	$($(quiet)AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).list: $(OBJECTS)
	$(file > $@, $(OBJECTS))

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile $(BUILD_DIR)/$(TARGET).list
	$($(quiet)LD) @$(BUILD_DIR)/$(TARGET).list $(LDFLAGS) -o $@
	$($(quiet)SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$($(quiet)HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$($(quiet)BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(call rwildcard,$(BUILD_DIR),*.d)
