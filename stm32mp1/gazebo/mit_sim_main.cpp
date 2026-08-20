/*!
 * @file mit_sim_main.cpp
 * @brief STM32MP1 MIT_Controller (convex MPC + WBC) against a Gazebo (Go1) SITL.
 *   mit_ctrl_sim [peer_ip] [robot_yaml] [user_yaml]
 * Runs the full locomotion FSM; drive it with an RC/gamepad source or the FSM's
 * default. This is the path toward OpenPilot-style waypoint navigation.
 */
#include <string>
#include <cstdio>

#include "Stm32mp1HardwareBridge.h"
#include "MIT_Controller.hpp"

static std::string g_peer;

int main(int argc, char** argv) {
  g_peer                 = (argc > 1) ? argv[1] : "127.0.0.1";
  std::string robot_yaml = (argc > 2) ? argv[2] : "stm32mp1-defaults.yaml";
  std::string user_yaml  = (argc > 3) ? argv[3] : "mc-mit-ctrl-user-parameters.yaml";

  printf("[mit-sim] Gazebo SITL | peer=%s robot=%s user=%s\n",
         g_peer.c_str(), robot_yaml.c_str(), user_yaml.c_str());

  MIT_Controller* ctrl = new MIT_Controller();
  Stm32mp1HardwareBridge bridge(ctrl, robot_yaml, user_yaml,
                                Stm32mp1HardwareBridge::Backend::GAZEBO);
  GazeboUdpConfig g;
  g.peer_addr   = g_peer.c_str();
  g.cmd_port    = 9100;
  g.sensor_port = 9101;
  bridge.setGazebo(g);

  bridge.run();
  return 0;
}
