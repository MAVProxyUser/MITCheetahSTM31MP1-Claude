/*!
 * unitree_probe — single-motor RS485 bring-up test for the STM32MP1 port.
 *
 * Opens ONE bus, pings ONE Unitree A1/B1 motor with a zero-torque (safe) command,
 * and prints the decoded joint feedback + round-trip success rate. Use this to
 * validate the RS485 wiring, transceiver, baud and hardware DE before wiring the
 * driver into the control loop.
 *
 * Build (cross): see stm32mp1/tools/build_tools.sh
 * Run (on board):  ./unitree_probe /dev/ttySTM1 4000000 0
 *                  ./unitree_probe <device> <baud> <motor_id> [cycles]
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include "rt/rt_unitree.h"

static void sleep_us(long us) {
  struct timespec ts{us / 1000000, (us % 1000000) * 1000};
  nanosleep(&ts, nullptr);
}

int main(int argc, char** argv) {
  const char* dev = (argc > 1) ? argv[1] : "/dev/ttySTM1";
  int baud        = (argc > 2) ? atoi(argv[2]) : 4000000;
  int motor_id    = (argc > 3) ? atoi(argv[3]) : 0;
  int cycles      = (argc > 4) ? atoi(argv[4]) : 200;

  printf("unitree_probe: %s @ %d baud, motor id %d, %d cycles\n", dev, baud, motor_id, cycles);

  UnitreeConfig cfg;                 // start from empty, mark a single motor present
  cfg.motorType = (baud >= 6000000) ? UnitreeMotorType::B1 : UnitreeMotorType::A1;
  cfg.num_buses = 1;
  cfg.buses[0].device = dev;
  cfg.buses[0].baud   = baud;
  for (int leg = 0; leg < 4; ++leg)
    for (int j = 0; j < 3; ++j) cfg.joint[leg][j].present = false;
  cfg.joint[0][0].present  = true;   // probe uses leg0/abad slot
  cfg.joint[0][0].bus      = 0;
  cfg.joint[0][0].motor_id = motor_id;
  cfg.joint[0][0].gear     = 1.0f;   // report raw motor units for bring-up
  cfg.joint[0][0].sign     = 1.0f;
  cfg.joint[0][0].offset   = 0.0f;

  if (init_unitree(cfg) != 0) { printf("init_unitree failed\n"); return 1; }

  spi_command_t* cmd  = get_spi_command();
  spi_data_t*    data = get_spi_data();
  memset(cmd, 0, sizeof(*cmd));      // zero torque, zero gains == safe / freewheel

  int ok = 0;
  for (int i = 0; i < cycles; ++i) {
    unitree_send_receive(cmd, data);
    bool answered = (data->spi_driver_status == 0);
    if (answered) ok++;
    if (i < 5 || i % 50 == 0) {
      printf("cycle %4d: %s  q=%+8.4f  qd=%+8.4f  (status=%d)\n",
             i, answered ? "OK " : "no reply",
             data->q_abad[0], data->qd_abad[0], data->spi_driver_status);
    }
    sleep_us(2000);                  // ~500 Hz
  }

  printf("done: %d/%d replies (%.1f%%)\n", ok, cycles, 100.0 * ok / cycles);
  unitree_close();
  return 0;
}
