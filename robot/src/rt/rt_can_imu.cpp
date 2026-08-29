/*!
 * @file rt_can_imu.cpp
 * @brief IMU over SocketCAN (DroneCAN compact stream) + Madgwick AHRS for the STM32MP1 port.
 * See rt_can_imu.h.
 */
#ifdef linux

#include "rt/rt_can_imu.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#include <atomic>
#include <mutex>
#include <thread>

namespace {

CanImuConfig      g_cfg;
VectorNavData*    g_out = nullptr;
int               g_sock = -1;
std::thread       g_thread;
std::atomic<bool> g_run{false};
std::mutex        g_mtx;

// Madgwick state, quaternion [w,x,y,z].
float q0 = 1.f, q1 = 0.f, q2 = 0.f, q3 = 0.f;
float g_beta = 0.06f;

// latest raw physical samples (already unit-converted + axis-remapped)
float g_gyro[3]  = {0, 0, 0};
float g_accel[3] = {0, 0, 0};

// rate counters
std::atomic<float> g_gyro_hz{0.f}, g_accel_hz{0.f};

inline double now_s() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// Standard Madgwick IMU-only update (gyro rad/s, accel any unit -> normalised).
void madgwick(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float recipNorm, s0, s1, s2, s3, qDot1, qDot2, qDot3, qDot4;
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  if (!(ax == 0.f && ay == 0.f && az == 0.f)) {
    recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
    float _2q0 = 2*q0, _2q1 = 2*q1, _2q2 = 2*q2, _2q3 = 2*q3;
    float _4q0 = 4*q0, _4q1 = 4*q1, _4q2 = 4*q2, _8q1 = 8*q1, _8q2 = 8*q2;
    float q0q0 = q0*q0, q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;
    s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
    s1 = _4q1*q3q3 - _2q3*ax + 4*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
    s2 = 4*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
    s3 = 4*q1q1*q3 - _2q1*ax + 4*q2q2*q3 - _2q2*ay;
    recipNorm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;
    qDot1 -= g_beta * s0; qDot2 -= g_beta * s1; qDot3 -= g_beta * s2; qDot4 -= g_beta * s3;
  }
  q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;
  recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

inline int16_t le16(const uint8_t* p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

// Apply raw int16[3] -> physical (scale) -> Cheetah body frame (axis remap/sign).
void remap(const int16_t raw[3], float scale, float out[3]) {
  float in[3] = {raw[0] * scale, raw[1] * scale, raw[2] * scale};
  for (int i = 0; i < 3; ++i) out[i] = g_cfg.axis_sign[i] * in[g_cfg.axis_map[i]];
}

void reader_loop() {
  double last_gyro_t = now_s();
  double win_t = last_gyro_t;
  int gyro_cnt = 0, accel_cnt = 0;
  bool have_accel = false;

  while (g_run.load()) {
    struct can_frame f;
    ssize_t n = read(g_sock, &f, sizeof(f));
    if (n < (ssize_t)sizeof(struct can_frame)) {
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) continue;  // recv timeout
      continue;
    }
    if (!(f.can_id & CAN_EFF_FLAG)) continue;                 // DroneCAN uses 29-bit ids
    uint32_t id = f.can_id & CAN_EFF_MASK;
    uint16_t msg_type = (id >> 8) & 0xFFFF;
    uint8_t  src_node = id & 0x7F;
    if ((id >> 7) & 1) continue;                              // service frame, not a message
    if (g_cfg.imu_node_id >= 0 && src_node != g_cfg.imu_node_id) continue;
    if (f.can_dlc < 6) continue;                              // need 3x int16

    if (msg_type == g_cfg.msg_gyro) {
      int16_t raw[3] = {le16(f.data + 0), le16(f.data + 2), le16(f.data + 4)};
      float gyro[3];  remap(raw, g_cfg.gyro_lsb_to_rads, gyro);
      double t = now_s();
      float dt = (float)(t - last_gyro_t);
      last_gyro_t = t;
      if (dt <= 0.f || dt > 0.1f) dt = 0.0025f;               // guard first/again samples
      std::lock_guard<std::mutex> lk(g_mtx);
      g_gyro[0] = gyro[0]; g_gyro[1] = gyro[1]; g_gyro[2] = gyro[2];
      madgwick(g_gyro[0], g_gyro[1], g_gyro[2],
               have_accel ? g_accel[0] : 0.f,
               have_accel ? g_accel[1] : 0.f,
               have_accel ? g_accel[2] : 0.f, dt);
      if (g_out) {
        g_out->gyro[0] = g_gyro[0]; g_out->gyro[1] = g_gyro[1]; g_out->gyro[2] = g_gyro[2];
        g_out->quat[0] = q1; g_out->quat[1] = q2; g_out->quat[2] = q3; g_out->quat[3] = q0; // [x,y,z,w]
        g_out->valid = true;   // a real IMU frame has landed (see IMUTypes.h)
      }
      gyro_cnt++;
    } else if (msg_type == g_cfg.msg_accel) {
      int16_t raw[3] = {le16(f.data + 0), le16(f.data + 2), le16(f.data + 4)};
      float accel[3]; remap(raw, g_cfg.accel_lsb_to_ms2, accel);
      std::lock_guard<std::mutex> lk(g_mtx);
      g_accel[0] = accel[0]; g_accel[1] = accel[1]; g_accel[2] = accel[2];
      have_accel = true;
      if (g_out) { g_out->accelerometer[0]=accel[0]; g_out->accelerometer[1]=accel[1]; g_out->accelerometer[2]=accel[2]; }
      accel_cnt++;
    }

    double t = now_s();
    if (t - win_t >= 1.0) {
      g_gyro_hz.store(gyro_cnt / (float)(t - win_t));
      g_accel_hz.store(accel_cnt / (float)(t - win_t));
      gyro_cnt = accel_cnt = 0; win_t = t;
    }
  }
}

}  // namespace

int init_can_imu(const CanImuConfig& cfg, VectorNavData* out) {
  g_cfg = cfg;
  g_out = out;
  g_beta = cfg.madgwick_beta;
  q0 = 1.f; q1 = q2 = q3 = 0.f;

  g_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (g_sock < 0) { printf("[rt_can_imu] socket(PF_CAN) failed: %s\n", strerror(errno)); return -1; }

  struct ifreq ifr;
  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, cfg.interface, IFNAMSIZ - 1);
  if (ioctl(g_sock, SIOCGIFINDEX, &ifr) < 0) {
    printf("[rt_can_imu] interface %s not found: %s (is it up? 'ip link set %s up type can ...')\n",
           cfg.interface, strerror(errno), cfg.interface);
    close(g_sock); g_sock = -1; return -1;
  }
  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (cfg.use_kernel_filter) {
    struct can_filter filt[2];
    filt[0].can_id   = ((uint32_t)cfg.msg_gyro  << 8) | CAN_EFF_FLAG;
    filt[0].can_mask = (0xFFFFu << 8) | CAN_EFF_FLAG;          // match message-type id only
    filt[1].can_id   = ((uint32_t)cfg.msg_accel << 8) | CAN_EFF_FLAG;
    filt[1].can_mask = (0xFFFFu << 8) | CAN_EFF_FLAG;
    setsockopt(g_sock, SOL_CAN_RAW, CAN_RAW_FILTER, filt, sizeof(filt));
  }
  // 100 ms recv timeout so the thread can be stopped cleanly.
  struct timeval tv = {0, 100000};
  setsockopt(g_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  if (bind(g_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    printf("[rt_can_imu] bind(%s) failed: %s\n", cfg.interface, strerror(errno));
    close(g_sock); g_sock = -1; return -1;
  }

  g_run.store(true);
  g_thread = std::thread(reader_loop);
  printf("[rt_can_imu] reading %s: gyro msg %u / accel msg %u from node %d\n",
         cfg.interface, cfg.msg_gyro, cfg.msg_accel, cfg.imu_node_id);
  return 0;
}

void can_imu_close() {
  g_run.store(false);
  if (g_thread.joinable()) g_thread.join();
  if (g_sock >= 0) { close(g_sock); g_sock = -1; }
}

float can_imu_gyro_hz()  { return g_gyro_hz.load(); }
float can_imu_accel_hz() { return g_accel_hz.load(); }

#endif  // linux
