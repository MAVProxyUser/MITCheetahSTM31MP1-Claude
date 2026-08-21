/*!
 * @file rt_unitree.h
 * @brief RS485 communication to Unitree joint motors (A1 / B1 family).
 *
 * Drop-in replacement for rt_spi on the STM32MP1 port: instead of talking to the
 * MIT spine boards over SPI, it drives Unitree motors over one or more RS485 buses
 * and fills the exact same spi_command_t / spi_data_t structures the rest of the
 * stack (RobotRunner, LegController) already uses. Legs/joints are indexed the MIT
 * way: leg 0..3, joint 0=abad, 1=hip, 2=knee.
 *
 * The Unitree wire protocol (motor_msg_A1B1.h) and CRC (crc32.h) are vendored under
 * third-party/unitree_motor_sdk. The serial transport here is written for the MP1:
 * arbitrary baud via termios2/BOTHER and hardware RS485 driver-enable via TIOCSRS485.
 */
#ifndef PROJECT_RT_UNITREE_H
#define PROJECT_RT_UNITREE_H

// (decls portable; impl Linux-only: declarations visible on every platform;
//  the .cpp is only compiled for Linux targets)

#include <stdint.h>
#include <spi_command_t.hpp>
#include <spi_data_t.hpp>

//! Motor family. Selects baud, wire protocol nuances, and a default gear ratio.
enum class UnitreeMotorType { A1, B1 };

/*!
 * Per-joint mapping and motor<->joint conversion.
 *   joint value = sign * (motor value / gear) + offset      (position)
 * gear, sign and offset MUST be validated per robot on the bench.
 */
struct UnitreeJointMap {
  int   bus     = 0;     //!< index into UnitreeConfig::buses
  int   motor_id = 0;    //!< Unitree motor id on that bus
  float gear    = 9.1f;  //!< gear reduction (A1 ~9.1)
  float sign    = 1.f;   //!< +1 / -1 joint direction relative to the motor
  float offset  = 0.f;   //!< joint angle (rad) when the motor sits at its encoder zero
  bool  present = true;  //!< false -> skip (motor not wired yet)
};

//! One RS485 bus == one serial device fronted by a fast RS485 transceiver (hardware DE).
struct UnitreeBusConfig {
  const char* device = nullptr;  //!< e.g. "/dev/ttySTM1"
  int         baud   = 4000000;  //!< A1 = 4 Mbps, B1 = 6 Mbps
};

//! Whole-robot configuration: up to 4 buses and the 4x3 joint map.
struct UnitreeConfig {
  UnitreeMotorType motorType = UnitreeMotorType::A1;
  int              num_buses = 4;
  UnitreeBusConfig buses[4];
  UnitreeJointMap  joint[4][3];       //!< [leg][abad/hip/knee]
  bool             use_rs485_hw_de = true;   //!< TIOCSRS485 hardware driver-enable
  int              read_timeout_us = 2000;   //!< per-motor response timeout
};

/*! Sensible bring-up default: A1, one bus per leg, ids 0/1/2 = abad/hip/knee. */
UnitreeConfig unitree_default_config();

/*! Open + configure every bus. Returns 0 on success, negative on failure. */
int init_unitree(const UnitreeConfig& cfg);

/*! One command/feedback cycle across all present motors (mirrors spi_send_receive). */
void unitree_send_receive(spi_command_t* command, spi_data_t* data);

/*! Send a zero-torque / brake command to every motor (safe stop). */
void unitree_stop_all();

void unitree_close();

spi_data_t*    get_spi_data();
spi_command_t* get_spi_command();

  // linux
#endif  // PROJECT_RT_UNITREE_H
