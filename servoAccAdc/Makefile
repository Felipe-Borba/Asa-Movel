# ##############################################################################
# MCU Makefile essentials
# ##############################################################################

# ==============================================================================
# Function definitions
# ==============================================================================

rwildcard								= $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2) $(filter $(subst *,%,$2),$d))
filter-out-any							= $(foreach v,$(2),$(if $(findstring $(1),$(v)),,$(v)))

# ==============================================================================
# Environment variables
# ==============================================================================

# ------------------------------------------------------------------------------
# Received through application -------------------------------------------------

# BUILD_DIR								?= build
# BUILD_NAME								?= application
# COMPILER_DEBUG							?= 1
# COMPILER_OPT							?= -Og
# COMPILER_PATH							?= 
# COMPILER_STD							?= c++11
# DEFINES_ASM								?= 
# DEFINES_C									?= USE_HAL_DRIVER STM32F091xC
# DEFINES_CPP								?= USE_HAL_DRIVER STM32F091xC
# LINKER_FILE							?= STM32F091RCTx_FLASH.ld
# MCU_DEVICE_NAME							?= STM32F103xB
# EXCLUDE_PATH							?= 
# FUNSAPE_PATH							?= 
# USER_FLAGS_ASM						?= 
# USER_FLAGS_C							?= 
# USER_FLAGS_CPP						?= 

# ------------------------------------------------------------------------------
# Fixed value - DO NOT CHANGE --------------------------------------------------

MCU_AVR_CODE_OFFSET						= 0
MCU_ARM_STM32_CODE_OFFSET				= 300

# ------------------------------------------------------------------------------
# Calculated based on received variables ---------------------------------------

MCU_DEVICE_CODE							= 0
include Makefile-AVR.mk
include Makefile-ARM-STM32.mk

ifeq ($(MCU_DEVICE_CODE),0)
    $(error No supported device found. Verify the MCU_DEVICE_NAME environment variable!)
endif

# ==============================================================================
# Compiler exacutables
# ==============================================================================

# ------------------------------------------------------------------------------
# Evaluate compiler prefix based on MCU platform -------------------------------

ifeq ($(MCU_DEVICE_PLATFORM),AVR)
    COMPILER_PREFIX = avr-
else ifeq  ($(MCU_DEVICE_PLATFORM),ARM_STM32)
    COMPILER_PREFIX = arm-none-eabi-
endif

# ------------------------------------------------------------------------------
# Evaluate executables names based on compiler path ----------------------------

ifneq ($(COMPILER_PATH),)
    COMPILER_PATH	:= $(COMPILER_PATH)/
endif

AR		:= $(COMPILER_PATH)$(COMPILER_PREFIX)ar
AS		:= $(COMPILER_PATH)$(COMPILER_PREFIX)gcc -x assembler-with-cpp
BIN		:= $(COMPILER_PATH)$(COMPILER_PREFIX)objcopy -O binary -S
CC		:= $(COMPILER_PATH)$(COMPILER_PREFIX)gcc
CCPP	:= $(COMPILER_PATH)$(COMPILER_PREFIX)g++
CP		:= $(COMPILER_PATH)$(COMPILER_PREFIX)objcopy
HEX		:= $(COMPILER_PATH)$(COMPILER_PREFIX)objcopy -O ihex
SZ		:= $(COMPILER_PATH)$(COMPILER_PREFIX)size

# ==============================================================================
# Search for files
# ==============================================================================

# ------------------------------------------------------------------------------
# Declaring variables
# ------------------------------------------------------------------------------

CODE_HEADERS_ASM						= 
CODE_HEADERS_H							= 
CODE_HEADERS_HPP						= 
CODE_HEADERS_HXX						= 
CODE_HEADERS_ALL						= 
CODE_SOURCES_ASM						= 
CODE_SOURCES_S							= 
CODE_SOURCES_AXX						= 
CODE_SOURCES_C							= 
CODE_SOURCES_CPP						= 
CODE_SOURCES_CXX						= 
FUNSAPE_HEADERS_ASM						= 
FUNSAPE_HEADERS_H						= 
FUNSAPE_HEADERS_HPP						= 
FUNSAPE_HEADERS_HXX						= 
FUNSAPE_HEADERS_ALL						= 
FUNSAPE_SOURCES_ASM						= 
FUNSAPE_SOURCES_S						= 
FUNSAPE_SOURCES_AXX						= 
FUNSAPE_SOURCES_C						= 
FUNSAPE_SOURCES_CPP						= 
FUNSAPE_SOURCES_CXX						= 

# ------------------------------------------------------------------------------
# Main code
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------
# Main code - ASM header files -------------------------------------------------

CODE_HEADERS_ASM		:= $(sort $(dir $(call rwildcard, , *.S)))
CODE_HEADERS_ASM		:= $(sort $(call filter-out-any,_hide/,$(CODE_HEADERS_ASM)))
CODE_HEADERS_ASM		:= $(sort $(call filter-out-any,doc/,$(CODE_HEADERS_ASM)))
CODE_HEADERS_ASM		:= $(sort $(call filter-out-any,Release/,$(CODE_HEADERS_ASM)))
CODE_HEADERS_ASM		:= $(sort $(call filter-out-any,temp/,$(CODE_HEADERS_ASM)))

# ------------------------------------------------------------------------------
# Main code - H header files ---------------------------------------------------

CODE_HEADERS_H			:= $(sort $(dir $(call rwildcard, , *.h)))
CODE_HEADERS_H			:= $(sort $(call filter-out-any,_hide/,$(CODE_HEADERS_H)))
CODE_HEADERS_H			:= $(sort $(call filter-out-any,doc/,$(CODE_HEADERS_H)))
CODE_HEADERS_H			:= $(sort $(call filter-out-any,Release/,$(CODE_HEADERS_H)))
CODE_HEADERS_H			:= $(sort $(call filter-out-any,temp/,$(CODE_HEADERS_H)))

# ------------------------------------------------------------------------------
# Main cide - HPP header files -------------------------------------------------

CODE_HEADERS_HPP		:= $(sort $(dir $(call rwildcard, , *.hpp)))
CODE_HEADERS_HPP		:= $(sort $(call filter-out-any,_hide/,$(CODE_HEADERS_HPP)))
CODE_HEADERS_HPP		:= $(sort $(call filter-out-any,doc/,$(CODE_HEADERS_HPP)))
CODE_HEADERS_HPP		:= $(sort $(call filter-out-any,Release/,$(CODE_HEADERS_HPP)))
CODE_HEADERS_HPP		:= $(sort $(call filter-out-any,temp/,$(CODE_HEADERS_HPP)))

# ------------------------------------------------------------------------------
# Main code - HXX header files -------------------------------------------------

CODE_HEADERS_HXX		:= $(sort $(CODE_HEADERS_H) $(CODE_HEADERS_HPP))

# ------------------------------------------------------------------------------
# Main code - All header files -------------------------------------------------

CODE_HEADERS_ALL		:= $(sort $(CODE_HEADERS_HXX) $(CODE_HEADERS_ASM))

# ------------------------------------------------------------------------------
# Main code - Assembly source files --------------------------------------------

CODE_SOURCES_S			:= $(sort $(call rwildcard, , *.s))
CODE_SOURCES_S			:= $(sort $(call filter-out-any,_hide/,$(CODE_SOURCES_S)))
CODE_SOURCES_S			:= $(sort $(call filter-out-any,doc/,$(CODE_SOURCES_S)))
CODE_SOURCES_S			:= $(sort $(call filter-out-any,Release/,$(CODE_SOURCES_S)))
CODE_SOURCES_S			:= $(sort $(call filter-out-any,temp/,$(CODE_SOURCES_S)))
CODE_SOURCES_ASM		:= $(sort $(call rwildcard, , *.asm))
CODE_SOURCES_ASM		:= $(sort $(call filter-out-any,_hide/,$(CODE_SOURCES_ASM)))
CODE_SOURCES_ASM		:= $(sort $(call filter-out-any,doc/,$(CODE_SOURCES_ASM)))
CODE_SOURCES_ASM		:= $(sort $(call filter-out-any,Release/,$(CODE_SOURCES_ASM)))
CODE_SOURCES_ASM		:= $(sort $(call filter-out-any,temp/,$(CODE_SOURCES_ASM)))

# ------------------------------------------------------------------------------
# Main code - AXX source files -------------------------------------------------

CODE_SOURCES_AXX		:= $(sort $(CODE_SOURCES_ASM) $(CODE_SOURCES_S))

# ------------------------------------------------------------------------------
# Main code - C source files ---------------------------------------------------

CODE_SOURCES_C			:= $(sort $(call rwildcard, , *.c))
CODE_SOURCES_C			:= $(sort $(call filter-out-any,_hide/,$(CODE_SOURCES_C)))
CODE_SOURCES_C			:= $(sort $(call filter-out-any,doc/,$(CODE_SOURCES_C)))
CODE_SOURCES_C			:= $(sort $(call filter-out-any,Release/,$(CODE_SOURCES_C)))
CODE_SOURCES_C			:= $(sort $(call filter-out-any,temp/,$(CODE_SOURCES_C)))

# ------------------------------------------------------------------------------
# Main code - CPP source files -------------------------------------------------

CODE_SOURCES_CPP		:= $(sort $(call rwildcard, , *.cpp))
CODE_SOURCES_CPP		:= $(sort $(call filter-out-any,_hide/,$(CODE_SOURCES_CPP)))
CODE_SOURCES_CPP		:= $(sort $(call filter-out-any,doc/,$(CODE_SOURCES_CPP)))
CODE_SOURCES_CPP		:= $(sort $(call filter-out-any,Release/,$(CODE_SOURCES_CPP)))
CODE_SOURCES_CPP		:= $(sort $(call filter-out-any,temp/,$(CODE_SOURCES_CPP)))

# ------------------------------------------------------------------------------
# Main code - CXX source files -------------------------------------------------

CODE_SOURCES_CXX		:= $(sort $(CODE_SOURCES_C) $(CODE_SOURCES_CPP))

# ------------------------------------------------------------------------------
# FunSAPE Library
# ------------------------------------------------------------------------------

ifneq ($(FUNSAPE_PATH),)
    # --------------------------------------------------------------------------
    # FunSAPE Library - ASM header files ---------------------------------------

    FUNSAPE_HEADERS_ASM		:= $(sort $(dir $(call rwildcard, $(FUNSAPE_PATH), *.S))
    FUNSAPE_HEADERS_ASM		:= $(sort $(call filter-out-any,_hide/,$(FUNSAPE_HEADERS_ASM)))
    FUNSAPE_HEADERS_ASM		:= $(sort $(call filter-out-any,doc/,$(FUNSAPE_HEADERS_ASM)))
    FUNSAPE_HEADERS_ASM		:= $(sort $(call filter-out-any,Release/,$(FUNSAPE_HEADERS_ASM)))
    FUNSAPE_HEADERS_ASM		:= $(sort $(call filter-out-any,temp/,$(FUNSAPE_HEADERS_ASM)))

    # --------------------------------------------------------------------------
    # FunSAPE Library - H header files -----------------------------------------

    FUNSAPE_HEADERS_H		:= $(sort $(dir $(call rwildcard, $(FUNSAPE_PATH), *.h)))
    FUNSAPE_HEADERS_H		:= $(sort $(call filter-out-any,_hide/,$(FUNSAPE_HEADERS_H)))
    FUNSAPE_HEADERS_H		:= $(sort $(call filter-out-any,doc/,$(FUNSAPE_HEADERS_H)))
    FUNSAPE_HEADERS_H		:= $(sort $(call filter-out-any,Release/,$(FUNSAPE_HEADERS_H)))
    FUNSAPE_HEADERS_H		:= $(sort $(call filter-out-any,temp/,$(FUNSAPE_HEADERS_H)))

    # --------------------------------------------------------------------------
    # FunSAPE Library - HPP header files ---------------------------------------

    FUNSAPE_HEADERS_HPP		:= $(sort $(dir $(call rwildcard, $(FUNSAPE_PATH), *.hpp)))
    FUNSAPE_HEADERS_HPP		:= $(sort $(call filter-out-any,_hide/,$(FUNSAPE_HEADERS_HPP)))
    FUNSAPE_HEADERS_HPP		:= $(sort $(call filter-out-any,doc/,$(FUNSAPE_HEADERS_HPP)))
    FUNSAPE_HEADERS_HPP		:= $(sort $(call filter-out-any,Release/,$(FUNSAPE_HEADERS_HPP)))
    FUNSAPE_HEADERS_HPP		:= $(sort $(call filter-out-any,temp/,$(FUNSAPE_HEADERS_HPP)))

    # --------------------------------------------------------------------------
    # FunSAPE Library - HXX header files ---------------------------------------

    FUNSAPE_HEADERS_HXX		:= $(sort $(FUNSAPE_HEADERS_H) $(FUNSAPE_HEADERS_HPP))

    # --------------------------------------------------------------------------
    # FunSAPE Library - All header files ---------------------------------------

    FUNSAPE_HEADERS_ALL		:= $(sort $(FUNSAPE_HEADERS_HXX) $(FUNSAPE_HEADERS_ASM))

    # --------------------------------------------------------------------------
    # FunSAPE Library - Assembly source files ----------------------------------

    FUNSAPE_SOURCES_S		:= $(sort $(call rwildcard, $(FUNSAPE_PATH), *.s))
    FUNSAPE_SOURCES_S		:= $(sort $(call filter-out-any,_hide/,$(FUNSAPE_SOURCES_S)))
    FUNSAPE_SOURCES_S		:= $(sort $(call filter-out-any,doc/,$(FUNSAPE_SOURCES_S)))
    FUNSAPE_SOURCES_S		:= $(sort $(call filter-out-any,Release/,$(FUNSAPE_SOURCES_S)))
    FUNSAPE_SOURCES_S		:= $(sort $(call filter-out-any,temp/,$(FUNSAPE_SOURCES_S)))
    FUNSAPE_SOURCES_ASM		:= $(sort $(call rwildcard, $(FUNSAPE_PATH), *.asm))
    FUNSAPE_SOURCES_ASM		:= $(sort $(call filter-out-any,_hide/,$(FUNSAPE_SOURCES_ASM)))
    FUNSAPE_SOURCES_ASM		:= $(sort $(call filter-out-any,doc/,$(FUNSAPE_SOURCES_ASM)))
    FUNSAPE_SOURCES_ASM		:= $(sort $(call filter-out-any,Release/,$(FUNSAPE_SOURCES_ASM)))
    FUNSAPE_SOURCES_ASM		:= $(sort $(call filter-out-any,temp/,$(FUNSAPE_SOURCES_ASM)))

    # --------------------------------------------------------------------------
    # FunSAPE Library - AXX source files ---------------------------------------

    FUNSAPE_SOURCES_AXX		:= $(sort $(FUNSAPE_SOURCES_ASM) $(FUNSAPE_SOURCES_S))

    # --------------------------------------------------------------------------
    # FunSAPE Library - C source files -----------------------------------------

    FUNSAPE_SOURCES_C		:= $(sort $(call rwildcard, $(FUNSAPE_PATH), *.c))
    FUNSAPE_SOURCES_C		:= $(sort $(call filter-out-any,_hide/,$(FUNSAPE_SOURCES_C)))
    FUNSAPE_SOURCES_C		:= $(sort $(call filter-out-any,doc/,$(FUNSAPE_SOURCES_C)))
    FUNSAPE_SOURCES_C		:= $(sort $(call filter-out-any,Release/,$(FUNSAPE_SOURCES_C)))
    FUNSAPE_SOURCES_C		:= $(sort $(call filter-out-any,temp/,$(FUNSAPE_SOURCES_C)))

    # --------------------------------------------------------------------------
    # FunSAPE Library - CPP source files ---------------------------------------

    FUNSAPE_SOURCES_CPP		:= $(sort $(call rwildcard, $(FUNSAPE_PATH), *.cpp))
    FUNSAPE_SOURCES_CPP		:= $(sort $(call filter-out-any,_hide/,$(FUNSAPE_SOURCES_CPP)))
    FUNSAPE_SOURCES_CPP		:= $(sort $(call filter-out-any,doc/,$(FUNSAPE_SOURCES_CPP)))
    FUNSAPE_SOURCES_CPP		:= $(sort $(call filter-out-any,Release/,$(FUNSAPE_SOURCES_CPP)))
    FUNSAPE_SOURCES_CPP		:= $(sort $(call filter-out-any,temp/,$(FUNSAPE_SOURCES_CPP)))

    # --------------------------------------------------------------------------
    # FunSAPE Library - CXX source files ---------------------------------------

    FUNSAPE_SOURCES_CXX		:= $(sort $(FUNSAPE_SOURCES_C) $(FUNSAPE_SOURCES_CPP))
endif

HEADERS_ASM				:= $(sort $(FUNSAPE_HEADERS_ASM) $(CODE_HEADERS_ASM))
HEADERS_H				:= $(sort $(FUNSAPE_HEADERS_H) $(CODE_HEADERS_H))
HEADERS_HPP				:= $(sort $(FUNSAPE_HEADERS_HPP) $(CODE_HEADERS_HPP))
HEADERS_HXX				:= $(sort $(FUNSAPE_HEADERS_HXX) $(CODE_HEADERS_HXX))
SOURCES_ASM				:= $(sort $(FUNSAPE_SOURCES_ASM) $(CODE_SOURCES_ASM))
SOURCES_S				:= $(sort $(FUNSAPE_SOURCES_S) $(CODE_SOURCES_S))
SOURCES_AXX				:= $(sort $(FUNSAPE_SOURCES_AXX) $(CODE_SOURCES_AXX))
SOURCES_C				:= $(sort $(FUNSAPE_SOURCES_C) $(CODE_SOURCES_C))
SOURCES_CPP				:= $(sort $(FUNSAPE_SOURCES_CPP) $(CODE_SOURCES_CPP))
SOURCES_CXX				:= $(sort $(FUNSAPE_SOURCES_CXX) $(CODE_SOURCES_CXX))

# ==============================================================================
# Create flags
# ==============================================================================

# ------------------------------------------------------------------------------
# Include flags ----------------------------------------------------------------

INCLUDES_ASM		:= $(addprefix -I, $(HEADERS_ASM))
INCLUDES_H			:= $(addprefix -I, $(HEADERS_H))
INCLUDES_HPP		:= $(addprefix -I, $(HEADERS_HPP))
INCLUDES_HXX		:= $(sort $(INCLUDES_HPP) $(INCLUDES_H))
INCLUDES_ALL		:= $(sort $(INCLUDES_ASM) $(INCLUDES_HXX))

ifneq ($(USER_DEFINES_ASM),)
    USER_DEFINES_ASM	:= $(addprefix -D,$(USER_DEFINES_ASM))
endif
DEFINES_ASM			:= $(sort $(USER_DEFINES_ASM))
# -D$(MCU_DEVICE_MACRO)
ifneq ($(USER_DEFINES_C),)
    USER_DEFINES_C	:= $(addprefix -D,$(USER_DEFINES_C))
endif
DEFINES_C			:= $(sort $(USER_DEFINES_C) -D$(MCU_DEVICE_MACRO))
ifneq ($(USER_DEFINES_CPP),)
    USER_DEFINES_CPP	:= $(addprefix -D,$(USER_DEFINES_CPP))
endif
DEFINES_CPP			:= $(sort $(USER_DEFINES_CPP) -D$(MCU_DEVICE_MACRO))
DEFINES_CXX			:= $(sort $(DEFINES_C) $(DEFINES_CPP))

# ------------------------------------------------------------------------------
# MCU compiler flags -----------------------------------------------------------

ifeq ($(MCU_DEVICE_PLATFORM),AVR)
    FLAG_MCU				= -mmcu=$(MCU_DEVICE_NAME)
else ifeq ($(MCU_DEVICE_PLATFORM),ARM_STM32)
    FLAG_MCU				= -mcpu=$(MCU_DEVICE_CORE_ID) -mthumb $(MCU_DEVICE_CORE_FPU) $(MCU_DEVICE_CORE_FLOAT_ABI)
endif

# ------------------------------------------------------------------------------
# ASM compiler flags -----------------------------------------------------------

FLAGS_ASM					= $(FLAG_MCU)
ifneq ($(COMPILER_STD),)
    FLAGS_ASM				+= -std=$(COMPILER_STD)
endif
FLAGS_ASM					+= $(COMPILER_OPT)
FLAGS_ASM					+= $(DEFINES_ASM)
FLAGS_ASM					+= $(INCLUDES_AXX)
FLAGS_ASM					+= -Wall -fdata-sections -ffunction-sections
# -fno-exceptions -fno-rtti
FLAGS_ASM					+= $(USER_FLAGS_ASM)

# ------------------------------------------------------------------------------
# C compiler flags -------------------------------------------------------------

FLAGS_C						= $(FLAG_MCU)
ifneq ($(COMPILER_STD),)
    FLAGS_C					+= -std=$(COMPILER_STD_C)
endif
FLAGS_C						+= $(COMPILER_OPT)
FLAGS_C						+= $(DEFINES_C)
FLAGS_C						+= $(INCLUDES_HXX)
FLAGS_C						+= -Wall -fdata-sections -ffunction-sections
FLAGS_C						+= $(USER_FLAGS_C)
ifeq ($(COMPILER_DEBUG),1)
    FLAGS_C					+= -g -gdwarf-2
endif
FLAGS_C						+= -MMD -MP -MF"$(@:%.o=%.d)"

# ------------------------------------------------------------------------------
# CPP compiler flags -----------------------------------------------------------

FLAGS_CPP					= $(FLAG_MCU)
ifneq ($(COMPILER_STD),)
    FLAGS_CPP				+= -std=$(COMPILER_STD_CPP)
endif
FLAGS_CPP					+= $(COMPILER_OPT)
FLAGS_CPP					+= $(DEFINES_CPP)
FLAGS_CPP					+= $(INCLUDES_HXX)
FLAGS_CPP					+= -Wall -fdata-sections -ffunction-sections -fno-exceptions -fno-rtti
FLAGS_CPP					+= $(USER_FLAGS_CPP)
ifeq ($(COMPILER_DEBUG),1)
    FLAGS_CPP				+= -g -gdwarf-2
endif
FLAGS_CPP					+= -MMD -MP -MF"$(@:%.o=%.d)"

# ------------------------------------------------------------------------------
# CXX compiler flags -----------------------------------------------------------

FLAGS_CXX					= $(FLAG_MCU)
ifneq ($(COMPILER_STD),)
    FLAGS_CXX				+= -std=$(COMPILER_STD_CPP)
endif
FLAGS_CXX					+= $(COMPILER_OPT)
FLAGS_CXX					+= $(DEFINES_CXX)
FLAGS_CXX					+= $(INCLUDES_HXX)
FLAGS_CXX					+= -Wall -fdata-sections -ffunction-sections -fno-exceptions -fno-rtti
FLAGS_CXX					+= $(USER_FLAGS_CPP)
ifeq ($(COMPILER_DEBUG),1)
    FLAGS_CPP				+= -g -gdwarf-2
endif
FLAGS_CPP					+= -MMD -MP -MF"$(@:%.o=%.d)"

# ------------------------------------------------------------------------------
# Linker flags -----------------------------------------------------------------

ifneq ($(USER_LIBS),)
    USER_LIBS					:= $(addprefix -l,$(USER_LIBS))
endif

FLAGS_LINKER					= $(FLAG_MCU)
ifneq ($(LINKER_FILE),)
    FLAGS_LINKER				+=-T$(LINKER_FILE)
endif
ifeq ($(MCU_DEVICE_PLATFORM),ARM_STM32)
    FLAGS_LINKER				+= -specs=nano.specs
endif
FLAGS_LINKER					+=$(LIBRARY_PATH)
ifeq ($(MCU_DEVICE_PLATFORM),ARM_STM32)
    FLAGS_LINKER				+= -lc -lm -lnosys
endif
FLAGS_LINKER					+= $(USER_LIBS)
FLAGS_LINKER					+= -Wl,--gc-sections -Wl,-Map=$(BUILD_DIR)/$(BUILD_NAME).map,--cref

# ==============================================================================
# Detecting computer support
# ==============================================================================

# ------------------------------------------------------------------------------
# Operational system and CPU architecture --------------------------------------

USED_OS									:=
USED_ARC								:=
### Windows
ifeq ($(OS),Windows_NT)
    USED_OS								:= Windows
### Windows - AMD64
    ifeq ($(PROCESSOR_ARCHITECTURE),AMD64)
        USED_ARC						:= AMD64
    endif
### Windows - I86
    ifeq ($(PROCESSOR_ARCHITECTURE),x86)
        USED_ARC						:= I86
    endif
### Windows - Unknown
    ifeq ($(USED_ARC),)
        USED_ARC						:= unknown architecture
    endif
else
    UNAME_S								:= $(shell uname -s)
### Linux
    ifeq ($(UNAME_S),Linux)
        USED_OS							:= Linux
    endif
### OSX
    ifeq ($(UNAME_S),Darwin)
        USED_OS							:= OSX
    endif
### Linux/OSX - AMD64
    UNAME_P								:= $(shell uname -p)
    ifeq ($(UNAME_P),x86_64)
        USED_ARC 						:= AMD64
    endif
### Linux/OSX - I86
    ifneq ($(filter %86,$(UNAME_P)),)
        USED_ARC						:= I86
    endif
### Linux/OSX - ARM
    ifneq ($(filter arm%,$(UNAME_P)),)
        USED_ARC						:= ARM
    endif
### Linux/OSX - Unknown
    ifeq ($(USED_ARC),)
        USED_ARC						:= unknown architecture
    endif
endif

# ------------------------------------------------------------------------------
# Detect what shell is used ----------------------------------------------------

USED_SHELL								:=
ifeq ($(findstring cmd.exe,$(SHELL)),cmd.exe)
    USED_SHELL							:= cmd
    PRINT_EMPTY_LINE					:= @echo.
else
    USED_SHELL							:= bash
    PRINT_EMPTY_LINE					:= @echo
endif

##############################################
##############################################

ifeq ($(COMPILE_C_AS_CPP),1)
    CC							:= $(CCPP)
    FLAGS_C						:= $(FLAGS_CPP)
endif

# ==============================================================================
# Build recipes
# ==============================================================================

.PHONY: compile size program cleanCompile cleanCompileProgram clean compileProgram compileCleanProgram cleanCompileCleanProgram doNothing

# ------------------------------------------------------------------------------
# Compile recipe
# ------------------------------------------------------------------------------

compile: $(BUILD_DIR)/$(BUILD_NAME).elf $(BUILD_DIR)/$(BUILD_NAME).hex $(BUILD_DIR)/$(BUILD_NAME).bin

# List of C objects
OBJECTS			+= $(addprefix $(BUILD_DIR)/,$(notdir $(SOURCES_C:.c=.o)))
vpath %.c $(sort $(dir $(SOURCES_C)))

# List of CPP objects
OBJECTS			+= $(addprefix $(BUILD_DIR)/,$(notdir $(SOURCES_CPP:.cpp=.opp)))
vpath %.cpp $(sort $(dir $(SOURCES_CPP)))

# List of Assembly objects
OBJECTS			+= $(addprefix $(BUILD_DIR)/,$(notdir $(SOURCES_S:.s=.o)))
vpath %.s $(sort $(dir $(SOURCES_S)))

# List of Assembly objects
OBJECTS			+= $(addprefix $(BUILD_DIR)/,$(notdir $(SOURCES_ASM:.asm=.o)))
vpath %.asm $(sort $(dir $(SOURCES_ASM)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(FLAGS_C) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.opp: %.cpp Makefile | $(BUILD_DIR)
	$(CCPP) -c $(FLAGS_CPP) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(FLAGS_ASM) $< -o $@

$(BUILD_DIR)/%.o: %.asm Makefile | $(BUILD_DIR)
	$(AS) -c $(FLAGS_ASM) $< -o $@

$(BUILD_DIR)/$(BUILD_NAME).elf: $(OBJECTS) Makefile
	$(CCPP) $(OBJECTS) $(FLAGS_LINKER) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@
	@echo 
	@echo 

$(BUILD_DIR):
	mkdir $@

# ------------------------------------------------------------------------------
# Clean recipe
# ------------------------------------------------------------------------------

clean:
	$(PRINT_EMPTY_LINE)
	@echo ================================================================================
	@echo Removing auxiliary files...
ifeq ($(USED_SHELL),cmd)
	@del /f /s /q "$(BUILD_DIR)\\*.d" 2> NUL
	@del /f /s /q "$(BUILD_DIR)\\*.lst" 2> NUL
	@del /f /s /q "$(BUILD_DIR)\\*.o" 2> NUL
	@del /f /s /q "$(BUILD_DIR)\\*.opp" 2> NUL
endif
ifeq ($(USED_SHELL),bash)
	@rm -r -f -v $(BUILD_DIR)/*.d 2> NUL
	@rm -r -f -v $(BUILD_DIR)/*.lst 2> NUL
	@rm -r -f -v $(BUILD_DIR)/*.o 2> NUL
	@rm -r -f -v $(BUILD_DIR)/*.opp 2> NUL
endif
	@echo Auxiliary files removed!
	@echo ================================================================================
	$(PRINT_EMPTY_LINE)
	@echo 

# ------------------------------------------------------------------------------
# Program recipe
# ------------------------------------------------------------------------------

program:
	$(PRINT_EMPTY_LINE)
	@echo ================================================================================
	@echo Loading program...
ifeq ($(MCU_DEVICE_PLATFORM),ARM_STM32)
	ST-LINK_CLI -c SWD UR -P $(BUILD_DIR)/$(BUILD_NAME).hex -V -HardRst PULSE=2000 -Rst
else ifeq ($(MCU_DEVICE_PLATFORM),AVR)
	avrdude -C $(PROGRAMMER_CONFIG_FILE) -v -p $(MCU_DEVICE_NAME) -c $(PROGRAMMER_TYPE) -P $(PROGRAMMER_PORT) -b$(PROGRAMMER_BAUDRATE) -D -Uflash:w:$(BUILD_DIR)/$(BUILD_NAME).hex:i
endif
	@echo Program loaded!
	@echo ================================================================================
	$(PRINT_EMPTY_LINE)
	@echo 
	@echo 
	@echo 

# ------------------------------------------------------------------------------
# Compound recipes
# ------------------------------------------------------------------------------

cleanCompile:				clean compile

compileProgram:				compile program

cleanCompileProgram:		clean compile program

compileCleanProgram:		compile clean program

cleanCompileCleanProgram:	clean compile clean program

# ------------------------------------------------------------------------------
# Information recipe
# ------------------------------------------------------------------------------

doNothing:
	$(info $(USED_OS))
	$(info $(USED_ARC))
	$(info $(USED_SHELL))
