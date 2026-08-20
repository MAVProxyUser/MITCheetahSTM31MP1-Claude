/*!
 * @file rt_gazebo.h
 * @brief UDP link to a Gazebo (gz-harmonic) simulation of the robot.
 *
 * SITL backend that replaces rt_unitree + rt_can_imu: instead of RS485 motors and
 * a CAN IMU, sensor + joint state arrive over UDP from cheetah_gazebo_bridge.py
 * (running next to Gazebo on the workstation), and joint impedance commands go back
 * the same way. It fills the same spi_data_t / VectorNavData the control loop uses,
 * so RobotRunner is unchanged.
 *
 * Wire layout (little-endian, packed): see sim_sensor_packet / sim_command_packet.
 * Joints are flattened Cheetah-order: index = leg*3 + joint, leg 0..3 (FR,FL,RR,RL),
 * joint 0=abad,1=hip,2=knee.
 */
#ifndef PROJECT_RT_GAZEBO_H
#define PROJECT_RT_GAZEBO_H

#ifdef linux

#include <stdint.h>
#include <spi_command_t.hpp>
#include <spi_data_t.hpp>
#include "SimUtilities/IMUTypes.h"   // VectorNavData

#pragma pack(push, 1)

//! Gazebo bridge -> controller: IMU + joint feedback + (future) baro/GPS.
struct sim_sensor_packet {
  uint32_t magic;        // SIM_SENSOR_MAGIC
  uint32_t seq;
  float    accel[3];     // m/s^2, body frame
  float    gyro[3];      // rad/s, body frame
  float    quat[4];      // x, y, z, w
  float    q[12];        // joint position, flat leg*3+joint
  float    qd[12];       // joint velocity
  // reserved for waypoint-nav work (Cheetah does not consume these yet)
  float    baro_alt;     // m
  float    baro_pressure;// Pa
  double   gps_lat;      // deg
  double   gps_lon;      // deg
  float    gps_alt;      // m
  float    gps_vel[3];   // NED m/s
};

//! controller -> Gazebo bridge: per-joint impedance command (the bridge runs the motor PD).
struct sim_command_packet {
  uint32_t magic;        // SIM_COMMAND_MAGIC
  uint32_t seq;
  float    q_des[12];
  float    qd_des[12];
  float    kp[12];
  float    kd[12];
  float    tau_ff[12];
};

#pragma pack(pop)

static const uint32_t SIM_SENSOR_MAGIC  = 0x53454E53u;  // 'SENS'
static const uint32_t SIM_COMMAND_MAGIC = 0x434D4443u;  // 'CMDC'

struct GazeboUdpConfig {
  const char* peer_addr = "127.0.0.1";  //!< workstation running the bridge/Gazebo
  int         cmd_port  = 9100;         //!< we send commands here
  int         sensor_port = 9101;       //!< we bind + receive sensors here
};

//! Baro + GPS from the sim. Not consumed by Cheetah yet; available for future
//! waypoint navigation via gazebo_get_aux().
struct SimAuxSensors {
  float  baro_alt = 0, baro_pressure = 0;
  double gps_lat = 0, gps_lon = 0;
  float  gps_alt = 0, gps_vel[3] = {0, 0, 0};
};

/*! Open the UDP socket and start the receiver thread that keeps *imu_out and the
 *  internal joint feedback current. Returns 0 on success. */
int  init_gazebo(const GazeboUdpConfig& cfg, VectorNavData* imu_out);

/*! Send the impedance command, then copy the latest joint feedback into *data.
 *  Mirrors unitree_send_receive so the bridge's motor task is unchanged. */
void gazebo_send_receive(spi_command_t* command, spi_data_t* data);

void  gazebo_close();
float gazebo_sensor_hz();   //!< measured inbound sensor rate (diagnostics)

/*! Latest baro + GPS from the sim (for future waypoint nav; Cheetah ignores these today). */
void  gazebo_get_aux(SimAuxSensors* out);

#endif  // linux
#endif  // PROJECT_RT_GAZEBO_H
