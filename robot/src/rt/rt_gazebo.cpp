/*!
 * @file rt_gazebo.cpp
 * @brief UDP link to a Gazebo simulation. See rt_gazebo.h.
 */
#ifdef linux

#include "rt/rt_gazebo.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#include <atomic>
#include <mutex>
#include <thread>

namespace {

GazeboUdpConfig    g_cfg;
VectorNavData*     g_imu = nullptr;
int                g_sock = -1;
struct sockaddr_in g_peer;      // where commands are sent
std::thread        g_thread;
std::atomic<bool>  g_run{false};
std::mutex         g_mtx;

spi_data_t         g_data;      // latest joint feedback (guarded by g_mtx)
SimAuxSensors      g_aux;       // latest baro/gps (guarded by g_mtx)
SimTruth           g_truth;     // latest sim ground truth (guarded by g_mtx)
std::atomic<uint32_t> g_cmd_seq{0};
std::atomic<float> g_hz{0.f};

inline double now_s() {
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

void unpack_sensor(const sim_sensor_packet& p) {
  std::lock_guard<std::mutex> lk(g_mtx);
  for (int leg = 0; leg < 4; ++leg) {
    g_data.q_abad[leg]  = p.q[leg * 3 + 0];
    g_data.q_hip[leg]   = p.q[leg * 3 + 1];
    g_data.q_knee[leg]  = p.q[leg * 3 + 2];
    g_data.qd_abad[leg] = p.qd[leg * 3 + 0];
    g_data.qd_hip[leg]  = p.qd[leg * 3 + 1];
    g_data.qd_knee[leg] = p.qd[leg * 3 + 2];
  }
  g_data.spi_driver_status = 0;
  if (g_imu) {
    for (int i = 0; i < 3; ++i) { g_imu->accelerometer[i] = p.accel[i]; g_imu->gyro[i] = p.gyro[i]; }
    for (int i = 0; i < 4; ++i) g_imu->quat[i] = p.quat[i];   // [x,y,z,w]
  }
  g_aux.baro_alt = p.baro_alt; g_aux.baro_pressure = p.baro_pressure;
  g_aux.gps_lat = p.gps_lat; g_aux.gps_lon = p.gps_lon; g_aux.gps_alt = p.gps_alt;
  for (int i = 0; i < 3; ++i) g_aux.gps_vel[i] = p.gps_vel[i];
  for (int i = 0; i < 3; ++i) { g_truth.pos[i] = p.truth_pos[i]; g_truth.vworld[i] = p.truth_vworld[i]; }
  for (int i = 0; i < 4; ++i) g_truth.quat[i] = p.truth_quat[i];
}

void reader_loop() {
  double win = now_s(); int cnt = 0;
  while (g_run.load()) {
    sim_sensor_packet p;
    ssize_t n = recv(g_sock, &p, sizeof(p), 0);
    if (n == (ssize_t)sizeof(p) && p.magic == SIM_SENSOR_MAGIC) {
      unpack_sensor(p);
      cnt++;
      double t = now_s();
      if (t - win >= 1.0) { g_hz.store(cnt / (float)(t - win)); cnt = 0; win = t; }
    }
    // recv timeout (SO_RCVTIMEO) lets us poll g_run for clean shutdown
  }
}

}  // namespace

int init_gazebo(const GazeboUdpConfig& cfg, VectorNavData* imu_out) {
  g_cfg = cfg;
  g_imu = imu_out;
  memset(&g_data, 0, sizeof(g_data));

  g_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (g_sock < 0) { printf("[rt_gazebo] socket failed: %s\n", strerror(errno)); return -1; }

  struct sockaddr_in bind_addr;
  memset(&bind_addr, 0, sizeof(bind_addr));
  bind_addr.sin_family = AF_INET;
  bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  bind_addr.sin_port = htons(cfg.sensor_port);
  if (bind(g_sock, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
    printf("[rt_gazebo] bind(%d) failed: %s\n", cfg.sensor_port, strerror(errno));
    close(g_sock); g_sock = -1; return -1;
  }
  struct timeval tv = {0, 100000};   // 100 ms
  setsockopt(g_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  memset(&g_peer, 0, sizeof(g_peer));
  g_peer.sin_family = AF_INET;
  g_peer.sin_port = htons(cfg.cmd_port);
  if (inet_pton(AF_INET, cfg.peer_addr, &g_peer.sin_addr) != 1) {
    printf("[rt_gazebo] bad peer addr %s\n", cfg.peer_addr);
    close(g_sock); g_sock = -1; return -1;
  }

  g_run.store(true);
  g_thread = std::thread(reader_loop);
  printf("[rt_gazebo] UDP up: recv sensors on :%d, send commands to %s:%d\n",
         cfg.sensor_port, cfg.peer_addr, cfg.cmd_port);
  return 0;
}

void gazebo_send_receive(spi_command_t* command, spi_data_t* data) {
  // pack + send the impedance command
  sim_command_packet c;
  memset(&c, 0, sizeof(c));
  c.magic = SIM_COMMAND_MAGIC;
  c.seq = g_cmd_seq.fetch_add(1);
  for (int leg = 0; leg < 4; ++leg) {
    c.q_des[leg*3+0]  = command->q_des_abad[leg];
    c.q_des[leg*3+1]  = command->q_des_hip[leg];
    c.q_des[leg*3+2]  = command->q_des_knee[leg];
    c.qd_des[leg*3+0] = command->qd_des_abad[leg];
    c.qd_des[leg*3+1] = command->qd_des_hip[leg];
    c.qd_des[leg*3+2] = command->qd_des_knee[leg];
    c.kp[leg*3+0]     = command->kp_abad[leg];
    c.kp[leg*3+1]     = command->kp_hip[leg];
    c.kp[leg*3+2]     = command->kp_knee[leg];
    c.kd[leg*3+0]     = command->kd_abad[leg];
    c.kd[leg*3+1]     = command->kd_hip[leg];
    c.kd[leg*3+2]     = command->kd_knee[leg];
    c.tau_ff[leg*3+0] = command->tau_abad_ff[leg];
    c.tau_ff[leg*3+1] = command->tau_hip_ff[leg];
    c.tau_ff[leg*3+2] = command->tau_knee_ff[leg];
  }
  if (g_sock >= 0)
    sendto(g_sock, &c, sizeof(c), 0, (struct sockaddr*)&g_peer, sizeof(g_peer));

  // copy the latest feedback out
  std::lock_guard<std::mutex> lk(g_mtx);
  *data = g_data;
}

void gazebo_close() {
  g_run.store(false);
  if (g_thread.joinable()) g_thread.join();
  if (g_sock >= 0) { close(g_sock); g_sock = -1; }
}

float gazebo_sensor_hz() { return g_hz.load(); }

void gazebo_get_truth(SimTruth* out) {
  std::lock_guard<std::mutex> lk(g_mtx);
  *out = g_truth;
}

void gazebo_get_aux(SimAuxSensors* out) {
  std::lock_guard<std::mutex> lk(g_mtx);
  *out = g_aux;
}

#endif  // linux
