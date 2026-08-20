/*!
 * @file robot_main.cpp
 * @brief Entry point for the STM32MP1 headless robot program.
 *
 * Bypasses main_helper (which pulls in the simulator and the SPI/EtherCAT bridges)
 * and drives the Unitree/CAN port bridge directly.
 *
 *   jpos_ctrl [robot_yaml] [user_yaml]
 * defaults: ./stm32mp1-defaults.yaml  ./jpos-user-parameters.yaml
 */
#include <string>

#include "Stm32mp1HardwareBridge.h"
#include "JPos_Controller.hpp"

int main(int argc, char** argv) {
  std::string robot_yaml = (argc > 1) ? argv[1] : "stm32mp1-defaults.yaml";
  std::string user_yaml  = (argc > 2) ? argv[2] : "jpos-user-parameters.yaml";

  printf("[stm32mp1] JPos controller | robot=%s user=%s\n",
         robot_yaml.c_str(), user_yaml.c_str());

  JPos_Controller* ctrl = new JPos_Controller();
  Stm32mp1HardwareBridge bridge(ctrl, robot_yaml, user_yaml);
  bridge.run();
  return 0;
}
