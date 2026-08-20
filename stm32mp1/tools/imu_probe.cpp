/*!
 * imu_probe — CAN IMU bring-up test for the STM32MP1 port.
 *
 * Reads the DroneCAN compact IMU stream off can0, runs the AHRS, and prints
 * gyro (rad/s), accel (m/s^2), the synthesised quaternion, and message rates.
 * Use it to confirm the bus is up, the node id / message ids are right, the axis
 * mapping/signs are sane (tilt the board and watch roll/pitch), and the rate is
 * enough for the control loop.
 *
 *   ./imu_probe [can_iface] [imu_node_id] [seconds]
 *   ./imu_probe can0 124 0        # 0 = run until Ctrl-C
 *
 * Bring the interface up first (board-config), e.g.:
 *   ip link set can0 up type can bitrate 1000000
 */
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <ctime>
#include "rt/rt_can_imu.h"

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int) { g_stop = 1; }

static void sleep_ms(long ms) {
  struct timespec ts{ms / 1000, (ms % 1000) * 1000000};
  nanosleep(&ts, nullptr);
}

int main(int argc, char** argv) {
  const char* iface = (argc > 1) ? argv[1] : "can0";
  int node          = (argc > 2) ? atoi(argv[2]) : -1;   // -1 = any node publishing the stream
  int seconds       = (argc > 3) ? atoi(argv[3]) : 0;   // 0 = forever

  signal(SIGINT, on_sig);

  CanImuConfig cfg;
  cfg.interface = iface;
  cfg.imu_node_id = node;

  VectorNavData v;
  for (int i = 0; i < 3; ++i) { v.gyro[i] = 0; v.accelerometer[i] = 0; }
  v.quat[0] = 0; v.quat[1] = 0; v.quat[2] = 0; v.quat[3] = 1;

  if (init_can_imu(cfg, &v) != 0) { printf("init_can_imu failed\n"); return 1; }
  printf("reading %s node %d ... (Ctrl-C to stop)\n", iface, node);

  double elapsed = 0;
  while (!g_stop && (seconds == 0 || elapsed < seconds)) {
    sleep_ms(250);
    elapsed += 0.25;
    printf("gyro[% 7.3f % 7.3f % 7.3f] rad/s  acc[% 7.3f % 7.3f % 7.3f] m/s2  "
           "quat(xyzw)[% .3f % .3f % .3f % .3f]  gyro=%.0fHz acc=%.0fHz\n",
           v.gyro[0], v.gyro[1], v.gyro[2],
           v.accelerometer[0], v.accelerometer[1], v.accelerometer[2],
           v.quat[0], v.quat[1], v.quat[2], v.quat[3],
           can_imu_gyro_hz(), can_imu_accel_hz());
  }

  can_imu_close();
  return 0;
}
