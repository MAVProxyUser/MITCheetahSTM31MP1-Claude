/*!
 * @file rt_unitree.cpp
 * @brief RS485 communication to Unitree joint motors (A1/B1) for the STM32MP1 port.
 *
 * See rt_unitree.h. This fills the same spi_command_t / spi_data_t the MIT stack uses,
 * so RobotRunner / LegController are unchanged. Legs 0..3, joints 0=abad,1=hip,2=knee.
 *
 * TRANSPORT (MP1-specific): arbitrary baud via termios2/BOTHER, half-duplex with the
 * UART's hardware RS485 driver-enable (TIOCSRS485). We must NOT include <termios.h>
 * together with <asm/termbits.h>; termios2 gives us the custom-baud path the classic
 * termios API cannot.
 *
 * PROTOCOL: Unitree A1/B1 low-level frame (third-party/unitree_motor_sdk/motor_msg_A1B1.h),
 * CRC32 from crc32.h. Command = 34 B (MasterComdDataV3), feedback = 78 B (ServoComdDataV3).
 *
 * VALIDATE ON HARDWARE: the field scalings (T x256, W x128, Pos x16384/2pi, K_P x2048,
 * K_W x1024), the FOC mode value, and the per-joint gear/sign/offset are taken from the
 * public headers / common practice and must be checked against a live motor before load.
 */
#ifdef linux

#include "rt/rt_unitree.h"

#include <asm/termbits.h>   // struct termios2, BOTHER, TCSETS2/TCGETS2, flag bits
#include <linux/serial.h>   // struct serial_rs485, SER_RS485_*
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "unitree_motor_sdk/motor_msg_A1B1.h"
#include "unitree_motor_sdk/crc32.h"

// A1/B1 joint working mode (FOC/servo). Verify against queryMotorMode() for your firmware.
static const uint8_t A1_MODE_FOC   = 10;
static const uint8_t A1_MODE_BRAKE = 0;

static UnitreeConfig g_cfg;
static int           g_fd[4] = {-1, -1, -1, -1};
static spi_command_t g_command;
static spi_data_t    g_data;

// ---- serial bus setup -------------------------------------------------------

static int set_custom_baud(int fd, int baud) {
  struct termios2 tio;
  if (ioctl(fd, TCGETS2, &tio) < 0) return -1;
  tio.c_cflag &= ~CBAUD;
  tio.c_cflag |= BOTHER | CS8 | CLOCAL | CREAD;   // 8 data bits, ignore modem lines, enable rx
  tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);    // no parity, 1 stop bit, no HW flow control
  tio.c_iflag = 0;                                 // raw input, no xon/xoff, no CR/NL mapping
  tio.c_oflag = 0;                                 // raw output
  tio.c_lflag = 0;                                 // non-canonical, no echo/signals
  tio.c_ispeed = baud;
  tio.c_ospeed = baud;
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 0;                             // non-blocking; we time out with poll()
  if (ioctl(fd, TCSETS2, &tio) < 0) return -1;
  return 0;
}

static int set_rs485(int fd, bool enable) {
  struct serial_rs485 rs485;
  memset(&rs485, 0, sizeof(rs485));
  if (enable) {
    // Hardware drives RTS as the transceiver DE: asserted while transmitting,
    // released for the reply. Turnaround delays in ms (0 = let the UART decide).
    rs485.flags = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND;
    rs485.flags &= ~SER_RS485_RTS_AFTER_SEND;
    rs485.delay_rts_before_send = 0;
    rs485.delay_rts_after_send  = 0;
  } else {
    rs485.flags = 0;
  }
  if (ioctl(fd, TIOCSRS485, &rs485) < 0) {
    // Not fatal: some setups use an auto-direction transceiver or a GPIO instead.
    printf("[rt_unitree] TIOCSRS485 not supported on this port (errno %d); "
           "assuming auto-direction transceiver\n", errno);
    return 1;
  }
  return 0;
}

static int open_bus(const UnitreeBusConfig& bus) {
  int fd = open(bus.device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    printf("[rt_unitree] failed to open %s: %s\n", bus.device, strerror(errno));
    return -1;
  }
  if (set_custom_baud(fd, bus.baud) < 0) {
    printf("[rt_unitree] failed to set baud %d on %s: %s\n", bus.baud, bus.device, strerror(errno));
    close(fd);
    return -1;
  }
  if (g_cfg.use_rs485_hw_de) set_rs485(fd, true);
  return fd;
}

// ---- one motor transaction (A1/B1) -----------------------------------------

struct MotorCmdVals { float tau, dq, q, kp, kd; uint8_t mode; };
struct MotorFbVals  { float tau, dq, q; int temp; int err; bool ok; };

static void write_all(int fd, const void* buf, size_t n) {
  const uint8_t* p = (const uint8_t*)buf;
  size_t sent = 0;
  while (sent < n) {
    ssize_t w = write(fd, p + sent, n - sent);
    if (w > 0) sent += (size_t)w;
    else if (w < 0 && errno != EAGAIN && errno != EINTR) break;
  }
}

// Read exactly n bytes or time out (us). Returns bytes read.
static size_t read_exact(int fd, void* buf, size_t n, int timeout_us) {
  uint8_t* p = (uint8_t*)buf;
  size_t got = 0;
  struct pollfd pfd = {fd, POLLIN, 0};
  while (got < n) {
    int pr = poll(&pfd, 1, (timeout_us + 999) / 1000);   // ms granularity
    if (pr <= 0) break;                                  // timeout or error
    ssize_t r = read(fd, p + got, n - got);
    if (r > 0) got += (size_t)r;
    else if (r < 0 && errno != EAGAIN && errno != EINTR) break;
  }
  return got;
}

static MotorFbVals txrx_a1(int fd, int id, const MotorCmdVals& c, int timeout_us) {
  MotorFbVals fb = {0, 0, 0, 0, 0, false};

  MasterComdDataV3 pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.head.start[0] = 0xFE;
  pkt.head.start[1] = 0xEE;
  pkt.head.motorID  = (unsigned char)id;
  pkt.Mdata.mode      = c.mode;
  pkt.Mdata.ModifyBit = 0xFF;   // do not modify stored motor params
  pkt.Mdata.ReadBit   = 0;
  pkt.Mdata.T   = (q15_t)(c.tau * 256.0f);
  pkt.Mdata.W   = (q15_t)(c.dq  * 128.0f);
  pkt.Mdata.Pos = (int32_t)(c.q * (16384.0f / (2.0f * (float)M_PI)));
  pkt.Mdata.K_P = (q15_t)(c.kp * 2048.0f);
  pkt.Mdata.K_W = (q15_t)(c.kd * 1024.0f);
  // CRC32 over all complete 32-bit words except the trailing CRC word.
  pkt.CRCdata.u32 = crc32_core((uint32_t*)&pkt, (uint32_t)((sizeof(pkt) >> 2) - 1));

  write_all(fd, &pkt, sizeof(pkt));

  ServoComdDataV3 r;
  size_t got = read_exact(fd, &r, sizeof(r), timeout_us);
  if (got != sizeof(r)) return fb;                                  // timeout / short read
  if (r.head.start[0] != 0xFE || r.head.start[1] != 0xEE) return fb;
  uint32_t crc = crc32_core((uint32_t*)&r, (uint32_t)((sizeof(r) >> 2) - 1));
  if (crc != r.CRCdata.u32) return fb;                              // corrupt frame

  fb.tau  = r.Mdata.T   / 256.0f;
  fb.dq   = r.Mdata.W   / 128.0f;
  fb.q    = r.Mdata.Pos * (2.0f * (float)M_PI / 16384.0f);
  fb.temp = r.Mdata.Temp;
  fb.err  = r.Mdata.MError;
  fb.ok   = true;
  return fb;
}

// ---- public API -------------------------------------------------------------

UnitreeConfig unitree_default_config() {
  UnitreeConfig cfg;
  cfg.motorType = UnitreeMotorType::A1;
  cfg.num_buses = 4;
  static const char* devs[4] = {"/dev/ttySTM1", "/dev/ttySTM2", "/dev/ttySTM3", "/dev/ttySTM4"};
  for (int b = 0; b < 4; ++b) { cfg.buses[b].device = devs[b]; cfg.buses[b].baud = 4000000; }
  const float g = 9.1f;   // A1 gear ratio
  for (int leg = 0; leg < 4; ++leg)
    for (int j = 0; j < 3; ++j) {
      cfg.joint[leg][j].bus = leg;      // one bus per leg
      cfg.joint[leg][j].motor_id = j;   // 0=abad, 1=hip, 2=knee
      cfg.joint[leg][j].gear = g;
      cfg.joint[leg][j].sign = 1.f;
      cfg.joint[leg][j].offset = 0.f;
      cfg.joint[leg][j].present = true;
    }
  return cfg;
}

int init_unitree(const UnitreeConfig& cfg) {
  g_cfg = cfg;
  memset(&g_command, 0, sizeof(g_command));
  memset(&g_data, 0, sizeof(g_data));
  int opened = 0;
  for (int b = 0; b < g_cfg.num_buses && b < 4; ++b) {
    if (!g_cfg.buses[b].device) continue;
    g_fd[b] = open_bus(g_cfg.buses[b]);
    if (g_fd[b] >= 0) {
      opened++;
      printf("[rt_unitree] bus %d: %s @ %d baud OK\n", b, g_cfg.buses[b].device, g_cfg.buses[b].baud);
    }
  }
  if (opened == 0) { printf("[rt_unitree] no buses opened\n"); return -1; }
  return 0;
}

// Access the per-joint arrays uniformly: index 0=abad, 1=hip, 2=knee.
static inline const float* cmd_q  (const spi_command_t* c, int j){ return j==0?c->q_des_abad:j==1?c->q_des_hip:c->q_des_knee; }
static inline const float* cmd_qd (const spi_command_t* c, int j){ return j==0?c->qd_des_abad:j==1?c->qd_des_hip:c->qd_des_knee; }
static inline const float* cmd_kp (const spi_command_t* c, int j){ return j==0?c->kp_abad:j==1?c->kp_hip:c->kp_knee; }
static inline const float* cmd_kd (const spi_command_t* c, int j){ return j==0?c->kd_abad:j==1?c->kd_hip:c->kd_knee; }
static inline const float* cmd_tau(const spi_command_t* c, int j){ return j==0?c->tau_abad_ff:j==1?c->tau_hip_ff:c->tau_knee_ff; }
static inline float* dat_q (spi_data_t* d, int j){ return j==0?d->q_abad:j==1?d->q_hip:d->q_knee; }
static inline float* dat_qd(spi_data_t* d, int j){ return j==0?d->qd_abad:j==1?d->qd_hip:d->qd_knee; }

void unitree_send_receive(spi_command_t* command, spi_data_t* data) {
  int failures = 0;
  for (int leg = 0; leg < 4; ++leg) {
    for (int j = 0; j < 3; ++j) {
      const UnitreeJointMap& m = g_cfg.joint[leg][j];
      if (!m.present || m.bus < 0 || m.bus >= 4 || g_fd[m.bus] < 0) continue;

      const float g = m.gear, s = m.sign, o = m.offset;
      // joint -> motor (see header): 1/s == s for s in {+1,-1}
      MotorCmdVals c;
      c.q   = g * s * (cmd_q(command, j)[leg] - o);
      c.dq  = g * s *  cmd_qd(command, j)[leg];
      c.tau = s * cmd_tau(command, j)[leg] / g;
      c.kp  = cmd_kp(command, j)[leg] / (g * g);
      c.kd  = cmd_kd(command, j)[leg] / (g * g);
      c.mode = A1_MODE_FOC;

      MotorFbVals fb = txrx_a1(g_fd[m.bus], m.motor_id, c, g_cfg.read_timeout_us);
      if (fb.ok) {
        // motor -> joint
        dat_q(data, j)[leg]  = s * (fb.q  / g) + o;
        dat_qd(data, j)[leg] = s * (fb.dq / g);
      } else {
        failures++;
      }
    }
  }
  data->spi_driver_status = failures;   // 0 == all motors answered this cycle
}

void unitree_stop_all() {
  for (int b = 0; b < g_cfg.num_buses && b < 4; ++b) {
    if (g_fd[b] < 0) continue;
    for (int leg = 0; leg < 4; ++leg)
      for (int j = 0; j < 3; ++j) {
        const UnitreeJointMap& m = g_cfg.joint[leg][j];
        if (m.present && m.bus == b) {
          MotorCmdVals c = {0, 0, 0, 0, 0, A1_MODE_BRAKE};
          txrx_a1(g_fd[b], m.motor_id, c, g_cfg.read_timeout_us);
        }
      }
  }
}

void unitree_close() {
  unitree_stop_all();
  for (int b = 0; b < 4; ++b) if (g_fd[b] >= 0) { close(g_fd[b]); g_fd[b] = -1; }
}

spi_data_t*    get_spi_data()    { return &g_data; }
spi_command_t* get_spi_command() { return &g_command; }

#endif  // linux
