cmake_minimum_required(VERSION 3.5)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

set(CMAKE_COLOR_MAKEFILE   ON)
#execute_process(COMMAND ../scripts/make_types.sh)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#set(CMAKE_VERBOSE_MAKEFILE ON)

option(MINI_CHEETAH_BUILD "use compiler flags for mini cheetah computer" OFF)
option(STM32MP1_BUILD "cross-compile for the Octavo OSD32MP1 (Cortex-A7 armv7l)" OFF)
set(BUILD_TYPE_RELEASE TRUE)

option(NO_SIM "Do not build simulator" OFF)

# The STM32MP1 port has no QT/display and only builds the robot control path.
if(STM32MP1_BUILD)
  set(NO_SIM ON)
endif()

if(STM32MP1_BUILD)
  # Cross build for the Octavo OSD32MP1 dev board. Cortex-A7 + NEON replaces
  # -march=native; -Werror is dropped (newer cross-gcc emits new warnings on
  # this ~2019 codebase); libstdc++/libgcc are linked statically (see toolchain.cmake).
  # THIS_COM is empty so THIS_COM "config/..." resolves relative to the run dir
  # (the deploy package), where run.sh cd's before launching.
  SET (THIS_COM "" )
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h)
  set(STM32MP1_ARCH_FLAGS "-mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard")
  # Eigen assumes x86-style tolerance of misaligned SIMD; ARMv7 NEON faults on it
  # (the kernel cannot fix up NEON alignment traps). Disable Eigen's alignment
  # requirement so it emits unaligned NEON loads. MUST be identical in every TU
  # (ABI: it changes sizeof of aligned Eigen members) -> a clean rebuild is required.
  set(STM32MP1_EIGEN_FLAGS "-DEIGEN_MAX_ALIGN_BYTES=0 -DEIGEN_MAX_STATIC_ALIGN_BYTES=0")
  set(CMAKE_CXX_FLAGS "-O3 -no-pie -g0 ${STM32MP1_ARCH_FLAGS} ${STM32MP1_EIGEN_FLAGS} -Wall -Wextra \
  -Wno-unused-parameter -Wno-unused-variable -Wno-sign-compare")
  set(CMAKE_C_FLAGS "-O3 -g0 ${STM32MP1_ARCH_FLAGS} -std=gnu99 -I.")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -no-pie -static-libstdc++ -static-libgcc")
  # Vendored Eigen 3.4.0, visible to every subdirectory (JCQP, common, robot, ...)
  # so <eigen3/Eigen/...> resolves without a system Eigen install.
  include_directories("${CMAKE_CURRENT_SOURCE_DIR}/third-party/eigen")
  message("**** STM32MP1 (Cortex-A7) cross build enabled ****")
elseif(MINI_CHEETAH_BUILD)
  SET (THIS_COM "../" )
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h)
  set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb -Wall \
  -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
  -Wshadow -Wsign-promo -Werror")
  set(CMAKE_C_FLAGS "-O3  -ggdb -std=gnu99 -I.")
  message("**** Mini-Cheetah build enabled ****")
else(MINI_CHEETAH_BUILD)
  SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h)

  if(CMAKE_SYSTEM_NAME MATCHES Linux)
    set(CMAKE_CXX_FLAGS "-O3 -no-pie -march=native -ggdb -Wall \
    -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
    -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
    -Wshadow -Wsign-promo -Werror")
  elseif(APPLE)
    set(CMAKE_CXX_FLAGS "-O3 -march=native -ggdb -Wall \
    -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
    -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
    -Wshadow -Wsign-promo")
    include_directories("/usr/local/include/")   # lcm includes
  endif()

  set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")
  message("**** Mini-Cheetah build disabled ****")
endif(MINI_CHEETAH_BUILD)

set(CMAKE_CXX_STANDARD 14)

#find_package(lcm)

# robot/CMakeLists selects a trimmed source list under STM32MP1_BUILD (no EtherCAT/
# VectorNav/SPI/operator-LCM) and also builds the jpos_ctrl executable there.
add_subdirectory(robot)
add_subdirectory(third-party)
add_subdirectory(common)

if(NO_SIM)

else(NO_SIM)
  add_subdirectory(sim)
endif()

if(NOT STM32MP1_BUILD)
  add_subdirectory(user)
  add_subdirectory(rc_test)
endif()
