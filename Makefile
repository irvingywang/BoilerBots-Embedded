######################################
# BoilerBots Embedded Build System
######################################

# ======== PROJECT CONFIGURATION ========
# Robot project to build
ROBOT_PROJECT ?= Swerve-Standard
TARGET = $(ROBOT_PROJECT)

# Board configuration
BOARD = typec
CONTROL_BASE = control-base
BOARD_BASE = $(CONTROL_BASE)/${BOARD}-board

# Build configuration
DEBUG = 1
OPT = -Og
BUILD_DIR = build/$(ROBOT_PROJECT)

# 
ifeq ($(BOARD), typec)
	STARTUP_POSTFIX = stm32f407xx
	LINK_SCRIPT_PREFIX = STM32F407IGHx
	BOARD_C_DEF = STM32F407xx
endif

ifeq ($(BOARD), mc02)
	STARTUP_POSTFIX = stm32h723xx
	LINK_SCRIPT_PREFIX = STM32H723VGTx
	BOARD_C_DEF = STM32H723xx
endif

# Find available robot projects
ROBOT_PROJECTS := $(patsubst %/src,%,$(wildcard */src))

# ======== TOOLCHAIN SETUP ========
# Compiler and tools
PREFIX = arm-none-eabi-
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

# ======== MCU FLAGS ========
ifeq ($(BOARD), typec)
	CPU = -mcpu=cortex-m4
	FPU = -mfpu=fpv4-sp-d16
	FLOAT-ABI = -mfloat-abi=hard
endif

ifeq ($(BOARD), mc02)
	CPU = -mcpu=cortex-m7
	FPU = -mfpu=fpv5-d16
	FLOAT-ABI = -mfloat-abi=hard
endif

# CPU and FPU settings
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# ======== COMPILER FLAGS ========
# ASM flags
AS_DEFS = 
AS_INCLUDES = -I$(BOARD_BASE)/Core/Inc
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -fdata-sections -ffunction-sections

# C flags
C_DEFS = -DUSE_HAL_DRIVER -D$(BOARD_C_DEF)
C_INCLUDES = \
-I$(BOARD_BASE)/Core/Inc \
-I$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/include \
-I$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
-I$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-I$(BOARD_BASE)/Drivers/CMSIS/Include \
-I$(CONTROL_BASE)/algo/inc \
-I$(CONTROL_BASE)/devices/inc \
-I$(CONTROL_BASE)/bsp/inc \
-I$(ROBOT_PROJECT)/inc

# typec HAL includes
ifeq ($(BOARD),typec)
C_INCLUDES += \
	-I$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Inc \
	-I$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy \
	-I$(BOARD_BASE)/Drivers/CMSIS/Device/ST/STM32F4xx/Include
endif

# mc02 HAL includes
ifeq ($(BOARD),mc02)
C_INCLUDES += \
	-I$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Inc \
	-I$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy \
	-I$(BOARD_BASE)/Drivers/CMSIS/Device/ST/STM32H7xx/Include

# include USB
C_INCLUDES += \
	-I$(BOARD_BASE)/USB_DEVICE/App \
	-I$(BOARD_BASE)/USB_DEVICE/Target \
	-I$(BOARD_BASE)/Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
	-I$(BOARD_BASE)/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
endif

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -fmessage-length=0 -Werror
ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# Linker flags
LDSCRIPT = $(BOARD_BASE)/$(LINK_SCRIPT_PREFIX)_FLASH.ld
LIBS = -lc -lm -lnosys
LIBDIR = 
LDFLAGS = $(MCU) -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -flto -Wl,--print-memory-usage -u _printf_float

# ======== TERMINAL COLORS ========
COLOR_RESET = \033[0m
COLOR_RED = \033[31m
COLOR_GREEN = \033[32m
COLOR_YELLOW = \033[33m
COLOR_BLUE = \033[34m
COLOR_MAGENTA = \033[35m
COLOR_CYAN = \033[36m
COLOR_BOLD = \033[1m

# ======== SOURCE FILES ========
# C sources
C_SOURCES = \
$(BOARD_BASE)/Core/Src/main.c \
$(BOARD_BASE)/Core/Src/gpio.c \
$(BOARD_BASE)/Core/Src/freertos.c \
$(BOARD_BASE)/Core/Src/dma.c \
$(BOARD_BASE)/Core/Src/spi.c \
$(BOARD_BASE)/Core/Src/tim.c \
$(BOARD_BASE)/Core/Src/usart.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/list.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/queue.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/timers.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
$(wildcard $(CONTROL_BASE)/algo/src/*.c) \
$(wildcard $(CONTROL_BASE)/bsp/src/*.c) \
$(wildcard $(CONTROL_BASE)/devices/src/*.c) \
$(wildcard $(ROBOT_PROJECT)/src/*.c)

# typec HAL sources
ifeq ($(BOARD), typec)
C_SOURCES += \
	$(BOARD_BASE)/Core/Src/can.c \
	$(BOARD_BASE)/Core/Src/i2c.c

C_SOURCES += \
	$(BOARD_BASE)/Core/Src/stm32f4xx_it.c \
	$(BOARD_BASE)/Core/Src/stm32f4xx_hal_msp.c \
	$(BOARD_BASE)/Core/Src/stm32f4xx_hal_timebase_tim.c \
	$(BOARD_BASE)/Core/Src/system_stm32f4xx.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
	$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c

ASM_SOURCES = $(BOARD_BASE)/startup_stm32f407xx.s
endif

# mc02 HAL sources
ifeq ($(BOARD), mc02)
# CANFD on mc02
C_SOURCES += \
	$(BOARD_BASE)/Core/Src/canfd.c \
	$(BOARD_BASE)/Core/Src/octospi.c \

C_SOURCES += \
	$(BOARD_BASE)/Core/Src/stm32h7xx_it.c \
	$(BOARD_BASE)/Core/Src/stm32h7xx_hal_msp.c \
	$(BOARD_BASE)/Core/Src/stm32h7xx_hal_timebase_tim.c \
	$(BOARD_BASE)/Core/Src/system_stm32h7xx.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_can.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ramfunc.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c \
	$(BOARD_BASE)/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c \
	$(BOARD_BASE)/USB_DEVICE/App/usb_device.c \
	$(BOARD_BASE)/USB_DEVICE/App/usbd_desc.c \
	$(BOARD_BASE)/USB_DEVICE/App/usbd_cdc_if.c \
	$(BOARD_BASE)/USB_DEVICE/Target/usbd_conf.c \
	Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c \

ASM_SOURCES = $(BOARD_BASE)/startup_stm32h723xx.s
endif


# ======== BUILD RULES ========
# List of object files
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# Default target: build all
all: print_info 
	@echo "${COLOR_YELLOW}Compiling...${COLOR_RESET}"
	@$(MAKE) --no-print-directory $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin
	@echo "${COLOR_GREEN}${COLOR_BOLD}Build successful!${COLOR_RESET}"

# Project info
print_info:
	@echo "${COLOR_CYAN}${COLOR_BOLD}Building robot project: ${COLOR_MAGENTA}$(ROBOT_PROJECT)${COLOR_CYAN}"
	@echo "${COLOR_CYAN}${COLOR_BOLD}Output directory: ${COLOR_MAGENTA}$(BUILD_DIR)/${COLOR_RESET}"

# Build C files
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@ || \
	(echo "${COLOR_RED}${COLOR_BOLD}Error compiling $<${COLOR_RESET}" && exit 1)

# Build ASM files
$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@$(AS) -c $(CFLAGS) $< -o $@ || \
	(echo "${COLOR_RED}${COLOR_BOLD}Error assembling $<${COLOR_RESET}" && exit 1)

# Link ELF
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	@echo "${COLOR_YELLOW}Linking...${COLOR_RESET}"
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@ || \
	(echo "${COLOR_RED}${COLOR_BOLD}Error linking $@${COLOR_RESET}" && exit 1)
	@$(SZ) $@

# Generate binary
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf | $(BUILD_DIR)
	@$(BIN) $< $@
	@echo "${COLOR_CYAN}Generated binary: $@${COLOR_RESET}"

$(BUILD_DIR):
	@mkdir -p $@

# Clean build files
clean:
	@echo "${COLOR_YELLOW}Cleaning build directory: build${COLOR_RESET}"
	@rm -rf build
	@echo "${COLOR_GREEN}Clean complete${COLOR_RESET}"

# Clean for Windows
clean_powershell:
	@echo "Cleaning build directory: build"
	@if exist build rmdir /s /q build
	@echo "Clean complete"

# ======== FLASH COMMANDS ========
# OpenOCD commands
OPENOCD_CMSIS_CMD = -f $(BOARD_BASE)/config/openocd_cmsis_dap.cfg -c init -c halt -c "program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset" -c "reset run" -c shutdown
OPENOCD_STLINK_CMD = -f $(BOARD_BASE)/config/openocd_stlink.cfg -c init -c halt -c "program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset" -c "reset run" -c shutdown

# Flash for Unix systems
flash:
	@echo "${COLOR_CYAN}${COLOR_BOLD}Attempting to flash device...${COLOR_RESET}"
	@openocd -d2 $(OPENOCD_CMSIS_CMD) && \
	(echo "${COLOR_GREEN}${COLOR_BOLD}[Success] Device programmed using CMSIS-DAP.${COLOR_RESET}") || \
	(echo "${COLOR_YELLOW}${COLOR_BOLD}[Warning] Trying STLink...${COLOR_RESET}" && \
	openocd -d2 $(OPENOCD_STLINK_CMD) && \
	(echo "${COLOR_GREEN}${COLOR_BOLD}[Success] Device programmed using STLink.${COLOR_RESET}") || \
	(echo "${COLOR_RED}${COLOR_BOLD}[Error] Flash failed. Check connections.${COLOR_RESET}"))

# Flash for Windows
flash_powershell:
	@echo "Attempting to use CMSIS-DAP..."
	@openocd $(OPENOCD_CMSIS_CMD) && \
	(powershell Write-Host -ForegroundColor Green [Success]: "Successfully programmed the device using CMSIS-DAP.") || \
	(powershell Write-Host -ForegroundColor Yellow [Warning]: "Failed to connect using CMSIS-DAP. Attempting to use STLink..." && \
	openocd $(OPENOCD_STLINK_CMD) && \
	(powershell Write-Host -ForegroundColor Green [Success]: "Successfully programmed the device using STLink.") || \
	(powershell Write-Host -ForegroundColor Yellow [Warning]: "Failed to connect using both CMSIS-DAP and STLink. Please check your connections and try again."))

# Show sources (debug)
print_sources:
	@echo "C sources:" $(C_SOURCES)
	@echo "ASM sources:" $(ASM_SOURCES)

# Include dependency files
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
