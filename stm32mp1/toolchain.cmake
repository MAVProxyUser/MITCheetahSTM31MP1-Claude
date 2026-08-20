# CMake toolchain file: cross-compile Cheetah-Software for the Octavo OSD32MP1
# (STM32MP157, dual Cortex-A7, armv7l hard-float) running OpenSTLinux.
#
# Uses the messense macos-cross-toolchains build of arm-unknown-linux-gnueabihf
# (gcc 15.2, glibc 2.28 base -> runs on the board's glibc 2.31). We dynamically
# link glibc but STATICALLY link libstdc++/libgcc so the binary does not depend
# on the board's older gcc-9 C++ runtime.
#
# Usage:  cmake -DCMAKE_TOOLCHAIN_FILE=stm32mp1/toolchain.cmake -DSTM32MP1_BUILD=ON ..

set(CMAKE_SYSTEM_NAME      Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Toolchain triplet. Override with -DCROSS_PREFIX=... or $CROSS_PREFIX if needed.
if(DEFINED ENV{CROSS_PREFIX})
  set(CROSS_PREFIX $ENV{CROSS_PREFIX})
endif()
if(NOT DEFINED CROSS_PREFIX)
  set(CROSS_PREFIX arm-unknown-linux-gnueabihf)
endif()

find_program(_CROSS_CC  ${CROSS_PREFIX}-gcc)
find_program(_CROSS_CXX ${CROSS_PREFIX}-g++)
if(NOT _CROSS_CC OR NOT _CROSS_CXX)
  message(FATAL_ERROR
    "Cross toolchain '${CROSS_PREFIX}' not found in PATH.\n"
    "Install it with:  brew install arm-unknown-linux-gnueabihf")
endif()

set(CMAKE_C_COMPILER   ${_CROSS_CC})
set(CMAKE_CXX_COMPILER ${_CROSS_CXX})

# Cortex-A7 with NEON (vfpv4), hard-float ABI. Replaces the desktop's -march=native.
set(STM32MP1_ARCH_FLAGS "-mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard")
set(CMAKE_C_FLAGS_INIT   "${STM32MP1_ARCH_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${STM32MP1_ARCH_FLAGS}")

# Self-contained C++ runtime; glibc stays dynamic (2.28 <= board 2.31).
set(_STM32MP1_LINK "-static-libstdc++ -static-libgcc")
set(CMAKE_EXE_LINKER_FLAGS_INIT    "${_STM32MP1_LINK}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${_STM32MP1_LINK}")

# Resolve the sysroot from the compiler and confine find_* to it (+ the source tree).
execute_process(COMMAND ${CMAKE_C_COMPILER} -print-sysroot
                OUTPUT_VARIABLE _CROSS_SYSROOT OUTPUT_STRIP_TRAILING_WHITESPACE)
if(_CROSS_SYSROOT)
  set(CMAKE_SYSROOT ${_CROSS_SYSROOT})
  set(CMAKE_FIND_ROOT_PATH ${_CROSS_SYSROOT})
endif()
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
