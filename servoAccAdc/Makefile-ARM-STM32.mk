# ##############################################################################
# MCU Makefile essentials - ARM STM32 device detection
# ##############################################################################

ifeq ($(MCU_DEVICE_CODE), 0)
    ifeq ($(MCU_DEVICE_NAME),STM32F103xB)
        MCU_DEVICE_CODE					= ($(MCU_ARM_STM32_CODE_OFFSET) + 1)
        MCU_DEVICE_CORE_FLOAT_ABI		= 
        MCU_DEVICE_CORE_FPU				= 
        MCU_DEVICE_CORE_ID				= cortex-m3
        MCU_DEVICE_MACRO				= STM32F103xB
        MCU_DEVICE_PLATFORM				= ARM_STM32
    else ifeq ($(MCU_DEVICE_NAME),OTHER)
        MCU_DEVICE_CODE					= ($(MCU_ARM_STM32_CODE_OFFSET) + 2)
        MCU_DEVICE_CORE_FLOAT_ABI		= 
        MCU_DEVICE_CORE_FPU				= 
        MCU_DEVICE_CORE_ID				= cortex-
        MCU_DEVICE_MACRO				= 
        MCU_DEVICE_PLATFORM				= ARM_STM32
    endif
endif
