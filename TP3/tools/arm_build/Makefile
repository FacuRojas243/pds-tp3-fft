# Compilación del firmware FUERA de MCUXpresso (sólo para verificar que
# compila y linkea contra el SDK). En el laboratorio se usa el proyecto de
# MCUXpresso IDE; esto no hace falta para la entrega.
#
# Uso:
#   make SDK_CORE=... SDK_DEV=... SDK_BOARD=... CMSIS_CORE=... CMSIS_DSP=...
#
# Repos usados (GitHub, rama release/26.09.00):
#   nxp-mcuxpresso/mcuxsdk-core, mcux-devices-mcx, mcuxsdk-examples
#   ARM-software/CMSIS_6, ARM-software/CMSIS-DSP (v1.16.2)

DEPS       ?= $(HOME)/deps
SDK_CORE   ?= $(DEPS)/mcuxsdk-core
SDK_DEV    ?= $(DEPS)/mcux-devices-mcx/MCXN/MCXN947
SDK_BOARD  ?= $(DEPS)/mcuxsdk-examples/_boards/frdmmcxn947
CMSIS_CORE ?= $(DEPS)/CMSIS_6/CMSIS/Core/Include
CMSIS_DSP  ?= $(DEPS)/CMSIS-DSP

SRC_DIR := ../../source
OUT     := out

CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size

CPUFLAGS := -mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard
DEFS     := -DCPU_MCXN947VDF_cm33_core0 -DNDEBUG
DRV      := $(SDK_CORE)/drivers

INCS := -I$(SRC_DIR) -I$(SDK_DEV) -I$(SDK_DEV)/../periph -I$(SDK_DEV)/drivers -I$(SDK_BOARD) \
        -I$(DRV)/common -I$(DRV)/ctimer -I$(DRV)/gpio -I$(DRV)/lpadc \
        -I$(DRV)/lpflexcomm -I$(DRV)/lpflexcomm/lpuart -I$(DRV)/mcx_spc \
        -I$(DRV)/vref_1 -I$(DRV)/port -I$(CMSIS_CORE) \
        -I$(CMSIS_DSP)/Include -I$(CMSIS_DSP)/PrivateInclude

CFLAGS  := $(CPUFLAGS) $(DEFS) $(INCS) -O2 -g -std=gnu11 -Wall -Wextra \
           -Wno-unused-parameter -ffunction-sections -fdata-sections
LDFLAGS := $(CPUFLAGS) -T$(SDK_DEV)/gcc/MCXN947_cm33_core0_flash.ld \
           -Wl,--gc-sections -Wl,-Map=$(OUT)/tp3.map --specs=nano.specs --specs=nosys.specs

APP_SRCS := $(SRC_DIR)/main.c $(SRC_DIR)/fft_proc.c $(SRC_DIR)/frame_tx.c $(SRC_DIR)/app_pins.c
SDK_SRCS := $(SDK_DEV)/system_MCXN947_cm33_core0.c $(SDK_DEV)/drivers/fsl_clock.c \
            $(SDK_DEV)/drivers/fsl_reset.c $(SDK_BOARD)/clock_config.c \
            $(DRV)/common/fsl_common.c $(DRV)/common/fsl_common_arm.c \
            $(DRV)/ctimer/fsl_ctimer.c $(DRV)/gpio/fsl_gpio.c $(DRV)/lpadc/fsl_lpadc.c \
            $(DRV)/lpflexcomm/fsl_lpflexcomm.c $(DRV)/lpflexcomm/lpuart/fsl_lpuart.c \
            $(DRV)/mcx_spc/fsl_spc.c $(DRV)/vref_1/fsl_vref.c
DSP_SRCS := $(CMSIS_DSP)/Source/TransformFunctions/TransformFunctions.c \
            $(CMSIS_DSP)/Source/CommonTables/CommonTables.c \
            $(CMSIS_DSP)/Source/ComplexMathFunctions/ComplexMathFunctions.c \
            $(CMSIS_DSP)/Source/StatisticsFunctions/StatisticsFunctions.c \
            $(CMSIS_DSP)/Source/BasicMathFunctions/BasicMathFunctions.c \
            $(CMSIS_DSP)/Source/SupportFunctions/SupportFunctions.c \
            $(CMSIS_DSP)/Source/FastMathFunctions/FastMathFunctions.c
ASM_SRCS := $(SDK_DEV)/gcc/startup_MCXN947_cm33_core0.S

OBJS := $(addprefix $(OUT)/,$(notdir $(APP_SRCS:.c=.o) $(SDK_SRCS:.c=.o) $(DSP_SRCS:.c=.o) $(ASM_SRCS:.S=.o)))
vpath %.c $(sort $(dir $(APP_SRCS) $(SDK_SRCS) $(DSP_SRCS)))
vpath %.S $(dir $(ASM_SRCS))

all: $(OUT)/tp3.elf

$(OUT):
	mkdir -p $@

$(OUT)/%.o: %.c | $(OUT)
	$(CC) $(CFLAGS) -c $< -o $@

$(OUT)/%.o: %.S | $(OUT)
	$(CC) $(CPUFLAGS) $(DEFS) -c $< -o $@

$(OUT)/tp3.elf: $(OBJS)
	$(CC) $(LDFLAGS) $^ -lm -o $@
	$(OBJCOPY) -O binary $@ $(OUT)/tp3.bin
	$(SIZE) $@

clean:
	rm -rf $(OUT)

.PHONY: all clean
