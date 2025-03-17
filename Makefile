######################################
# target
######################################
# Use robot project name for the target name
ROBOT_PROJECT ?= Swerve-Standard
TARGET = $(ROBOT_PROJECT)

BOARD = typec
CONTROL_BASE = control-base
BOARD_BASE = $(CONTROL_BASE)/${BOARD}-board-base

ifeq ($(BOARD), typec)
	STARTUP_POSTFIX = stm32f407xx
	LINK_SCRIPT_PREFIX = STM32F407IGHx
	BOARD_C_DEF = STM32F407xx
endif

# Find all robot projects (directories that contain a src subdirectory)
ROBOT_PROJECTS := $(patsubst %/src,%,$(wildcard */src))

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build/$(ROBOT_PROJECT)

######################################
# source
######################################
# C sources
C_SOURCES =  \
$(BOARD_BASE)/Core/Src/main.c \
$(BOARD_BASE)/Core/Src/gpio.c \
$(BOARD_BASE)/Core/Src/freertos.c \
$(BOARD_BASE)/Core/Src/can.c \
$(BOARD_BASE)/Core/Src/dma.c \
$(BOARD_BASE)/Core/Src/i2c.c \
$(BOARD_BASE)/Core/Src/spi.c \
$(BOARD_BASE)/Core/Src/tim.c \
$(BOARD_BASE)/Core/Src/usart.c \
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
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
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
$(wildcard $(ROBOT_PROJECT)/src/*.c) \

# ASM sources
ASM_SOURCES =  \
$(BOARD_BASE)/startup_stm32f407xx.s


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
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb -mthumb-interwork $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-D$(BOARD_C_DEF)

# AS includes
AS_INCLUDES =  \
-I$(BOARD_BASE)/Core/Inc

# C includes
C_INCLUDES =  \
-I$(BOARD_BASE)/Core/Inc \
-I$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Inc \
-I$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-I$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/include \
-I$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
-I$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-I$(BOARD_BASE)/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
-I$(BOARD_BASE)/Drivers/CMSIS/Include \
-I$(CONTROL_BASE)/algo/inc \
-I$(CONTROL_BASE)/devices/inc \
-I$(CONTROL_BASE)/bsp/inc \
-I$(ROBOT_PROJECT)/inc \

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -fmessage-length=0 -Werror

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = $(BOARD_BASE)/$(LINK_SCRIPT_PREFIX)_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -flto -Wl,--print-memory-usage -u _printf_float

# Color definitions for terminal output
COLOR_RESET = \033[0m
COLOR_RED = \033[31m
COLOR_GREEN = \033[32m
COLOR_YELLOW = \033[33m
COLOR_BLUE = \033[34m
COLOR_MAGENTA = \033[35m
COLOR_CYAN = \033[36m
COLOR_BOLD = \033[1m

# default action: build all
all: print_info 
	@echo "${COLOR_YELLOW}Compiling...${COLOR_RESET}"
	@$(MAKE) --no-print-directory $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin
	@echo "${COLOR_GREEN}${COLOR_BOLD}Build successful!${COLOR_RESET}"

# Add a print statement to debug which sources are being compiled
print_info:
	@echo "${COLOR_CYAN}${COLOR_BOLD}Building robot project: ${COLOR_MAGENTA}$(ROBOT_PROJECT)${COLOR_CYAN}"
	@echo "${COLOR_CYAN}${COLOR_BOLD}Output directory: ${COLOR_MAGENTA}$(BUILD_DIR)/"

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@ || \
	(echo "${COLOR_RED}${COLOR_BOLD}Error compiling $<${COLOR_RESET}" && exit 1)

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@$(AS) -c $(CFLAGS) $< -o $@ || \
	(echo "${COLOR_RED}${COLOR_BOLD}Error assembling $<${COLOR_RESET}" && exit 1)

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	@echo "${COLOR_YELLOW}Linking...${COLOR_RESET}"
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@ || \
	(echo "${COLOR_RED}${COLOR_BOLD}Error linking $@${COLOR_RESET}" && exit 1)
	@$(SZ) $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf | $(BUILD_DIR)
	@$(BIN) $< $@
	@echo "${COLOR_CYAN}Generated binary: $@${COLOR_RESET}"

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.bin | $(BUILD_DIR)
	@$(BIN) $< $@	
	
$(BUILD_DIR):
	@mkdir $@		

#######################################
# clean up
#######################################
clean:
	@echo "${COLOR_YELLOW}Cleaning build directory: $(BUILD_DIR)${COLOR_RESET}"
	@rm -rf $(BUILD_DIR)
	@echo "${COLOR_GREEN}Clean complete${COLOR_RESET}"

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

#######################################
# download task
#######################################

# Windows (Powershell)
ECHO_WARNING_POWERSHELL=powershell Write-Host -ForegroundColor Yellow [Warning]:
ECHO_SUCCESS_POWERSHELL=powershell Write-Host -ForegroundColor Green [Success]:

flash_powershell:
	@echo "Attempting to use CMSIS-DAP..."
	@openocd -f config/openocd_cmsis_dap.cfg -c init -c halt -c "program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset" -c "reset run" -c shutdown && \
	($(ECHO_SUCCESS_POWERSHELL) "Successfully programmed the device using CMSIS-DAP.") || \
	($(ECHO_WARNING_POWERSHELL) "Failed to connect using CMSIS-DAP. Attempting to use STLink..." && \
	openocd -f config/openocd_stlink.cfg -c init -c halt -c "program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset" -c "reset run" -c shutdown && \
	($(ECHO_SUCCESS_POWERSHELL) "Successfully programmed the device using STLink.") || \
	($(ECHO_WARNING_POWERSHELL) "Failed to connect using both CMSIS-DAP and STLink. Please check your connections and try again."))


# Unix-Like (Linux, MacOS)
ECHO_WARNING=echo "\033[33m[Warning]\033[0m"
ECHO_SUCCESS=echo "\033[32m[Success]\033[0m"

flash:
	@echo "${COLOR_CYAN}${COLOR_BOLD}Attempting to flash device...${COLOR_RESET}"
	@openocd -d2 -f config/openocd_cmsis_dap.cfg -c init -c halt -c "program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset" -c "reset run" -c shutdown && \
	(echo "${COLOR_GREEN}${COLOR_BOLD}[Success] Device programmed using CMSIS-DAP.${COLOR_RESET}") || \
	(echo "${COLOR_YELLOW}${COLOR_BOLD}[Warning] Trying STLink...${COLOR_RESET}" && \
	openocd -d2 -f config/openocd_stlink.cfg -c init -c halt -c "program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset" -c "reset run" -c shutdown && \
	(echo "${COLOR_GREEN}${COLOR_BOLD}[Success] Device programmed using STLink.${COLOR_RESET}") || \
	(echo "${COLOR_RED}${COLOR_BOLD}[Error] Flash failed. Check connections.${COLOR_RESET}"))

print_sources:
	@echo "C sources:" $(C_SOURCES)
	@echo "ASM sources:" $(ASM_SOURCES)

# *** EOF ***
