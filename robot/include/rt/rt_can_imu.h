/*!
 * @file rt_can_imu.h
 * @brief IMU over SocketCAN (DroneCAN) for the STM32MP1 port.
 *
 * Replaces rt_vectornav. The board's IMU is no longer on I2C/SPI; gyro+accel arrive
 * as two single-frame DroneCAN messages on can0 (the "compact stream" from the OP
 * Revo Redux node, ~400 Hz, raw MPU-9150 counts). A background thread reads can0,
 * decodes those two messages, runs a Madgwick AHRS on the MP1 to synthesise the
 * quaternion Cheetah's VectorNavOrientationEstimator needs, and continuously updates
 * a VectorNavData the control loop reads.
 *
 * Compact stream (single frame, int16[3] little-endian, from node imu_node_id):
 *   msg 20500  gyro   raw counts, +-2000 dps FS  -> dps  = raw / 16.4
 *   msg 20501  accel  raw counts, +-2 g FS       -> m/s2 = raw * 9.80665 / 16384
 *
 * VectorNavData.quat is stored [x, y, z, w] (the estimator reads quat[3] as w).
 */
#ifndef PROJECT_RT_CAN_IMU_H
#define PROJECT_RT_CAN_IMU_H

#ifdef linux

#include <stdint.h>
#include "SimUtilities/IMUTypes.h"   // VectorNavData

struct CanImuConfig {
  const char* interface   = "can0";
  int         imu_node_id = -1;       //!< DroneCAN source node; -1 = accept any node
                                      //!< publishing the compact stream (allocator-agnostic;
                                      //!< only the IMU node emits 20500/20501). Observed live: node 123.
  uint16_t    msg_gyro    = 20500;    //!< int16[3] LE raw counts
  uint16_t    msg_accel   = 20501;    //!< int16[3] LE raw counts

  // Raw-count -> physical. MPU-9150: +-2000 dps (16.4 LSB/dps), +-2 g (16384 LSB/g).
  float gyro_lsb_to_rads  = (1.0f / 16.4f) * 0.017453292519943295f;  // -> rad/s
  float accel_lsb_to_ms2  = 9.80665f / 16384.0f;                     // -> m/s^2

  // Axis remap into Cheetah body frame: out[i] = axis_sign[i] * in[axis_map[i]].
  // MUST match the physical IMU mounting -- the single most likely sign-bug source.
  int   axis_map[3]  = {0, 1, 2};
  float axis_sign[3] = {1.f, 1.f, 1.f};

  float madgwick_beta = 0.1f;         //!< AHRS gain (rad/s); larger = trust accel more / faster settle
  bool  use_kernel_filter = true;     //!< install CAN_RAW filters for the two msg ids
};

/*! Open the CAN interface and start the reader/AHRS thread that keeps *out current.
 *  Returns 0 on success, negative on failure. *out must outlive the driver. */
int  init_can_imu(const CanImuConfig& cfg, VectorNavData* out);

/*! Stop the thread and close the socket. */
void can_imu_close();

/*! Measured message rates (Hz), for diagnostics/bring-up. */
float can_imu_gyro_hz();
float can_imu_accel_hz();

#endif  // linux
#endif  // PROJECT_RT_CAN_IMU_H
