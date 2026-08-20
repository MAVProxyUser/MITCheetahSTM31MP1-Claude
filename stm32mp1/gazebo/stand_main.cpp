/*!
 * @file stand_main.cpp
 * @brief STM32MP1 Cheetah StandController against a Gazebo (Go1) SITL over UDP.
 *   stand_sim [peer_ip] [robot_yaml]
 */
#include <string>
#include <cstdio>

#include "Stm32mp1HardwareBridge.h"
#include "StandController.hpp"

static std::string g_peer;

int main(int argc, char** argv) {
  g_peer                 = (argc > 1) ? argv[1] : "127.0.0.1";
  std::string robot_yaml = (argc > 2) ? argv[2] : "stm32mp1-defaults.yaml";

  printf("[stand] Gazebo SITL | peer=%s robot=%s\n", g_peer.c_str(), robot_yaml.c_str());

  StandController* ctrl = new StandController();
  Stm32mp1HardwareBridge bridge(ctrl, robot_yaml, "",
                                Stm32mp1HardwareBridge::Backend::GAZEBO);
  GazeboUdpConfig g;
  g.peer_addr   = g_peer.c_str();
  g.cmd_port    = 9100;
  g.sensor_port = 9101;
  bridge.setGazebo(g);

  bridge.run();
  return 0;
}
