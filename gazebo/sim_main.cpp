/*!
 * @file sim_main.cpp
 * @brief STM32MP1 Cheetah controller against a Gazebo (Go1) SITL over UDP.
 *
 *   jpos_ctrl_sim [peer_ip] [robot_yaml] [user_yaml]
 *
 * peer_ip = the workstation running cheetah_gazebo_bridge.py + Gazebo (default
 * 127.0.0.1). The controller sends impedance commands to peer_ip:9100 and
 * receives sensor/joint packets on :9101.
 */
#include <string>
#include <cstdio>

#include "Stm32mp1HardwareBridge.h"
#include "JPos_Controller.hpp"

static std::string g_peer;   // must outlive the bridge (holds the c_str)

int main(int argc, char** argv) {
  g_peer                = (argc > 1) ? argv[1] : "127.0.0.1";
  std::string robot_yaml = (argc > 2) ? argv[2] : "stm32mp1-defaults.yaml";
  std::string user_yaml  = (argc > 3) ? argv[3] : "jpos-user-parameters.yaml";

  printf("[sim] Gazebo SITL | peer=%s robot=%s user=%s\n",
         g_peer.c_str(), robot_yaml.c_str(), user_yaml.c_str());

  JPos_Controller* ctrl = new JPos_Controller();
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
