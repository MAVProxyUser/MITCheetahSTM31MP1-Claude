if(STM32MP1_BUILD)
  # ---- Trimmed headless port: Unitree RS485 + CAN IMU, no SPI/EtherCAT/VectorNav ----
  include_directories(${CMAKE_BINARY_DIR})
  include_directories(${PROJECT_SOURCE_DIR})
  include_directories("include/")
  include_directories("./")
  include_directories("../common/include/")
  include_directories("../third-party")
  include_directories("../third-party/ParamHandler")
  include_directories("../lcm-types/cpp")
  # Null LCM shim resolves both <lcm-cpp.hpp> and <lcm/lcm-cpp.hpp>
  include_directories("${PROJECT_SOURCE_DIR}/stm32mp1/lcm_shim")
  include_directories("${PROJECT_SOURCE_DIR}/stm32mp1/lcm_shim/lcm")

  set(STM32MP1_ROBOT_SRC
    src/RobotRunner.cpp
    src/JPosInitializer.cpp
    src/Stm32mp1HardwareBridge.cpp
    src/rt/rt_gazebo.cpp)
  if(NOT STM32MP1_HOST)
    # RS485 motors and CAN IMU are Linux-only (termios2/BOTHER, SocketCAN);
    # the host build talks to Gazebo over UDP and never needs them.
    list(APPEND STM32MP1_ROBOT_SRC src/rt/rt_unitree.cpp src/rt/rt_can_imu.cpp)
  endif()

  add_library(robot STATIC ${STM32MP1_ROBOT_SRC})
  target_link_libraries(robot biomimetics pthread)

  # Hardware robot executable (JPos controller for bring-up).
  add_executable(jpos_ctrl
    ${PROJECT_SOURCE_DIR}/stm32mp1/robot_main.cpp
    ${PROJECT_SOURCE_DIR}/user/JPos_Controller/JPos_Controller.cpp)
  target_include_directories(jpos_ctrl PRIVATE ${PROJECT_SOURCE_DIR}/user/JPos_Controller)
  target_link_libraries(jpos_ctrl robot biomimetics dynacore_param_handler pthread)

  # Gazebo SITL executable (same controller, UDP backend to Gazebo on the workstation).
  add_executable(jpos_ctrl_sim
    ${PROJECT_SOURCE_DIR}/gazebo/sim_main.cpp
    ${PROJECT_SOURCE_DIR}/user/JPos_Controller/JPos_Controller.cpp)
  target_include_directories(jpos_ctrl_sim PRIVATE ${PROJECT_SOURCE_DIR}/user/JPos_Controller)
  target_link_libraries(jpos_ctrl_sim robot biomimetics dynacore_param_handler pthread)

  # Stand controller against Gazebo (holds a Go1 stance; validates the joint map).
  add_executable(stand_sim
    ${PROJECT_SOURCE_DIR}/gazebo/stand_main.cpp
    ${PROJECT_SOURCE_DIR}/gazebo/StandController.cpp)
  target_include_directories(stand_sim PRIVATE ${PROJECT_SOURCE_DIR}/gazebo)
  target_link_libraries(stand_sim robot biomimetics dynacore_param_handler pthread)

  # Joint-space creep-gait walk controller against Gazebo (walk A->B).
  add_executable(walk_sim
    ${PROJECT_SOURCE_DIR}/gazebo/walk_main.cpp
    ${PROJECT_SOURCE_DIR}/gazebo/WalkController.cpp)
  target_include_directories(walk_sim PRIVATE ${PROJECT_SOURCE_DIR}/gazebo)
  target_link_libraries(walk_sim robot biomimetics dynacore_param_handler pthread)

  # Dynamic trot (Raibert footholds + attitude feedback); the speed gait.
  add_executable(trot_sim
    ${PROJECT_SOURCE_DIR}/gazebo/trot_main.cpp
    ${PROJECT_SOURCE_DIR}/gazebo/TrotController.cpp
    ${PROJECT_SOURCE_DIR}/gazebo/WaypointNav.cpp)
  target_include_directories(trot_sim PRIVATE ${PROJECT_SOURCE_DIR}/gazebo)
  target_link_libraries(trot_sim robot biomimetics dynacore_param_handler pthread)

  # Statically-stable crawl with CoM shift (abstract convention; BRIDGE_CONV=mit).
  add_executable(static_gait_sim
    ${PROJECT_SOURCE_DIR}/gazebo/static_gait_main.cpp
    ${PROJECT_SOURCE_DIR}/gazebo/StaticGaitController.cpp
    ${PROJECT_SOURCE_DIR}/gazebo/WaypointNav.cpp)
  target_include_directories(static_gait_sim PRIVATE ${PROJECT_SOURCE_DIR}/gazebo)
  target_link_libraries(static_gait_sim robot biomimetics dynacore_param_handler pthread)

  # Find the co-deployed third-party .so's next to the binary (deploy = copy the folder).
  set_target_properties(jpos_ctrl jpos_ctrl_sim stand_sim walk_sim static_gait_sim trot_sim PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN")

else()
  include_directories(${CMAKE_BINARY_DIR})
  include_directories(${PROJECT_SOURCE_DIR})
  include_directories("include/")
  include_directories("./")
  include_directories("../common/include/")
  include_directories("../third-party")
  include_directories("../third-party/ParamHandler")
  include_directories("../third-party/vectornav/include")
  include_directories("../third-party/vectornav/include/vn")
  include_directories("../third-party/lord_imu/Include")

  include_directories("../lcm-types/cpp")
  include_directories("/usr/local/include/lcm/")   # lcm includes
  include_directories("../third-party/SOEM/osal")
  include_directories("../third-party/SOEM/osal/linux")
  include_directories("../third-party/SOEM/oshw")
  include_directories("../third-party/SOEM/oshw/linux")

  file(GLOB sources "src/*.cpp" "src/rt/*.c"  "src/rt/*.cpp")

  add_library(robot SHARED ${sources})

  target_link_libraries(robot biomimetics pthread lcm inih dynacore_param_handler lord_imu soem)
  if(CMAKE_SYSTEM_NAME MATCHES Linux)
  target_link_libraries(robot libvnc rt)
  endif()
endif()
