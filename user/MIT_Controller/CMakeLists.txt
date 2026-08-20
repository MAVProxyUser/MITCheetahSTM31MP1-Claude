include_directories("./")
include_directories("./Controllers")
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../common/FootstepPlanner")
include_directories("../../third-party/")
include_directories(SYSTEM "../../third-party/qpOASES/include")
include_directories("../../third-party/ParamHandler")

include_directories("../../lcm-types/cpp")
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("/usr/local/include/eigen3")   # lcm includes
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(Controllers/WBC)
add_subdirectory(Controllers/WBC_Ctrl)
add_subdirectory(Controllers/VisionMPC)

if(STM32MP1_BUILD)
  # Port: mit_ctrl_sim runs the MIT_Controller against Gazebo (UDP backend). Exclude
  # main.cpp (needs main_helper/SimulationBridge) and the sublib sources (built as libs).
  file(GLOB MIT_SRC
    "FSM_States/*.cpp"
    "Controllers/convexMPC/*.cpp"
    "Controllers/BalanceController/*.cpp"
    "Controllers/BackFlip/*.cpp")
  add_executable(mit_ctrl_sim
    ${MIT_SRC}
    MIT_Controller.cpp
    ${PROJECT_SOURCE_DIR}/stm32mp1/gazebo/mit_sim_main.cpp)
  target_link_libraries(mit_ctrl_sim robot biomimetics dynacore_param_handler
    qpOASES Goldfarb_Optimizer osqp WBC_Ctrl VisionMPC pthread)
  set_target_properties(mit_ctrl_sim PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE INSTALL_RPATH "$ORIGIN")
else()
  file(GLOB_RECURSE sources
  "./*.cpp"
  "FSM_States/*.cpp"
  "Controllers/BalanceController/*.cpp"
  "Controllers/convexMPC/*.cpp")
  add_executable(mit_ctrl ${sources} MIT_Controller.cpp main.cpp)
  target_link_libraries(mit_ctrl robot biomimetics)
  target_link_libraries(mit_ctrl dynacore_param_handler qpOASES)
  target_link_libraries(mit_ctrl Goldfarb_Optimizer osqp)
  target_link_libraries(mit_ctrl WBC_Ctrl)
  target_link_libraries(mit_ctrl VisionMPC)
endif()
