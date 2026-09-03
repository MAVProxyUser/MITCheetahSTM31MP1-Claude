/*!
 * @file trot_main.cpp
 * @brief STM32MP1 dynamic trot against a Gazebo (Go1) SITL.
 *   trot_sim [peer_ip] [robot_yaml]
 * Run the bridge with BRIDGE_CONV=mit (abstract joint convention).
 */
#include <string>
#include <cstdio>

#include "Stm32mp1HardwareBridge.h"
#include "TrotController.hpp"

int main(int argc, char** argv) {
  std::string peer       = (argc > 1) ? argv[1] : "127.0.0.1";
  std::string robot_yaml = (argc > 2) ? argv[2] : "stm32mp1-defaults.yaml";

  printf("[trot] Gazebo SITL | peer=%s robot=%s\n", peer.c_str(), robot_yaml.c_str());

  TrotController* ctrl = new TrotController();
  Stm32mp1HardwareBridge bridge(ctrl, robot_yaml, "",
                                Stm32mp1HardwareBridge::Backend::GAZEBO);
  GazeboUdpConfig g;
  g.peer_addr = peer.c_str();
  g.cmd_port = 9100;
  g.sensor_port = 9101;
  bridge.setGazebo(g);
  bridge.run();
  return 0;
}
