set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

# Resolve absolute tools for both VS Code and terminal builds. No global PATH edit.
# ARM_GNU_TOOLCHAIN, when set, points at a GNU Arm bin directory.
file(GLOB _stm32_bundle_bins "$ENV{LOCALAPPDATA}/stm32cube/bundles/gnu-tools-for-stm32/*/bin")
list(SORT _stm32_bundle_bins COMPARE NATURAL ORDER DESCENDING)
find_program(ARM_GCC_EXECUTABLE NAMES arm-none-eabi-gcc
    HINTS "$ENV{ARM_GNU_TOOLCHAIN}" ${_stm32_bundle_bins})
if(NOT ARM_GCC_EXECUTABLE)
    message(FATAL_ERROR "GNU Arm compiler not found. Set ARM_GNU_TOOLCHAIN to its bin directory, or install the STM32Cube GNU tools bundle.")
endif()
get_filename_component(_arm_bin "${ARM_GCC_EXECUTABLE}" DIRECTORY)
find_program(ARM_GXX_EXECUTABLE NAMES arm-none-eabi-g++ HINTS "${_arm_bin}" NO_DEFAULT_PATH REQUIRED)
find_program(ARM_OBJCOPY_EXECUTABLE NAMES arm-none-eabi-objcopy HINTS "${_arm_bin}" NO_DEFAULT_PATH REQUIRED)
find_program(ARM_SIZE_EXECUTABLE NAMES arm-none-eabi-size HINTS "${_arm_bin}" NO_DEFAULT_PATH REQUIRED)
set(CMAKE_C_COMPILER "${ARM_GCC_EXECUTABLE}")
set(CMAKE_ASM_COMPILER "${ARM_GCC_EXECUTABLE}")
set(CMAKE_CXX_COMPILER "${ARM_GXX_EXECUTABLE}")
set(CMAKE_LINKER "${ARM_GXX_EXECUTABLE}")
set(CMAKE_OBJCOPY "${ARM_OBJCOPY_EXECUTABLE}")
set(CMAKE_SIZE "${ARM_SIZE_EXECUTABLE}")

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F407XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
