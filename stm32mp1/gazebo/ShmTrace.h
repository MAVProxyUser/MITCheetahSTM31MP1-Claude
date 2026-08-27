/*! @file ShmTrace.h
 *  @brief Lock-free, single-writer shared-memory ring buffer for per-tick
 *  control-loop tracing, built to root-cause the fleet-only fast-fall
 *  mystery (sector:15:3 tipping over within seconds, only in a 3-dog
 *  fleet, with clean control-loop timing - so the cause is somewhere in
 *  the CONTROL DATA itself, not a scheduling stall, and the existing
 *  low-rate printf logging (1/50 ticks at best, gated behind env vars
 *  because a full-rate printf would itself perturb the very timing this
 *  project has repeatedly found to matter) has never had enough
 *  resolution to catch it.
 *
 *  Per direct instruction: shared memory, a ring buffer, cheap enough to
 *  run every control tick (500 Hz) with no syscalls and no formatting on
 *  the hot path, a SEPARATE process reaps it (never the control loop
 *  itself - a writer that also has to drain its own buffer is not a
 *  cheap writer any more), and a persistent archive/"oracle" of confirmed
 *  crashes gets built from it so failures accumulate evidence instead of
 *  each one vanishing when the process exits.
 *
 *  Design:
 *   - POSIX shm_open()/mmap(), one named segment per dog
 *     ("/cheetah_trace_<instance>"), sized for RING_CAPACITY fixed-size
 *     records (65536 @ 98 bytes = ~6.4 MB - about 131 s of history at
 *     500 Hz, comfortably more than any fall takes to develop).
 *   - Single producer (the control loop / nav thread in THIS process),
 *     lock-free: an atomic monotonic write_seq, incremented AFTER the
 *     record at slot (seq % capacity) is fully written (release order),
 *     so any reader that observes write_seq == N is guaranteed record
 *     N-1 is complete. A reader can fall behind (the ring wraps under
 *     it) - that only costs OLD history, never corrupts a read, and for
 *     "what led up to this crash" the recent end is what matters.
 *   - Every field is a POD scalar the Python reaper can parse with a
 *     fixed struct.unpack format string - no pointers, no STL, nothing
 *     that only means something inside this process.
 */
#ifndef CHEETAH_SHM_TRACE_H
#define CHEETAH_SHM_TRACE_H

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace shmtrace {

// Packed explicitly - this layout crosses the C++/Python boundary via a
// raw byte-for-byte struct.unpack, so compiler-inserted padding would
// silently desync the two sides. 98 bytes: comfortably small, keeps 65536 records to ~6.4 MB.
#pragma pack(push, 1)
struct Record {
  double   t;            // elapsed mission time, seconds
  uint64_t seq;           // this record's sequence number (redundant with
                          // the ring index, kept so a reader can detect a
                          // wrap-induced gap rather than misread one)
  char     tag[20];       // short site/event name, NUL-padded (e.g.
                          // "tick", "ESTOP", "FALL", "recover_wait",
                          // "recover_ok", "recover_fail")
  float    roll, pitch, yaw;      // rpy, radians
  float    wx, wy, wz;            // omegaBody, rad/s
  float    vx, vy, vz;            // vBody, m/s
  float    z;                     // position[2], m
  float    period_ms;             // THIS tick's measured control period -
                                   // the periodic printf only ever reports
                                   // a min/max over thousands of ticks,
                                   // which is exactly what would hide a
                                   // single-tick hitch that only matters
                                   // because of what it lands on
  float    contact[4];            // contactEstimate, per leg, 0..1
  uint8_t  op_mode;                // FSM_OperatingMode: 0 NORMAL,
                                   // 1 TRANSITIONING, 2 ESTOP, 3 EDAMP
  uint8_t  finite;                 // 1 if the state estimate was finite
                                   // this tick, 0 if the NaN guard fired
};
#pragma pack(pop)
// 98 bytes - measured directly with a standalone compile after hand-
// summing the field widths got it wrong twice in a row. Not required to
// be any particular round number since #pragma pack(1) already guarantees
// no compiler padding; the number only matters because the Python
// reaper's struct.unpack format string has to match it exactly.
static_assert(sizeof(Record) == 98, "Record layout must stay in sync with the Python reaper's struct format - update BOTH sides together");

constexpr uint32_t RING_CAPACITY = 65536;

struct Header {
  std::atomic<uint64_t> write_seq;
  uint32_t capacity;
  uint32_t record_size;
  uint32_t magic;         // sanity check for the reaper - catches "attached
                          // to a stale/foreign segment" before it silently
                          // decodes garbage as floats
  uint32_t writer_pid;    // identifies WHICH open() of this segment name a
                          // reader is looking at - see the long comment on
                          // this field's use in shm_reaper.py's watch().
                          // Costs nothing: fits in the 4 bytes Header was
                          // already padded to (24 measured vs 20 summed,
                          // from write_seq's 8-byte atomic alignment).
};
constexpr uint32_t MAGIC = 0x43484554;  // "CHET"

struct Ring {
  Header header;
  Record records[RING_CAPACITY];
};

// One writer per process. Lazily opened on first log() call so a process
// that never logs never pays for the mmap at all; the segment name is
// derived from SIM_INSTANCE so each dog in a fleet gets its own, matching
// the same per-dog namespacing convention the UDP ports and gz topics
// already use.
class Writer {
 public:
  static Writer& instance() {
    static Writer w;
    return w;
  }

  void log(const char* tag, double t, float roll, float pitch, float yaw,
           float wx, float wy, float wz, float vx, float vy, float vz,
           float z, float period_ms, uint8_t op_mode, uint8_t finite,
           const float contact[4]) {
    ensure_open();
    if (!ring_) return;
    const uint64_t seq = ring_->header.write_seq.load(std::memory_order_relaxed);
    Record& r = ring_->records[seq % RING_CAPACITY];
    r.t = t;
    r.seq = seq;
    std::memset(r.tag, 0, sizeof(r.tag));
    std::strncpy(r.tag, tag, sizeof(r.tag) - 1);
    r.roll = roll; r.pitch = pitch; r.yaw = yaw;
    r.wx = wx; r.wy = wy; r.wz = wz;
    r.vx = vx; r.vy = vy; r.vz = vz;
    r.z = z;
    r.period_ms = period_ms;
    for (int i = 0; i < 4; ++i) r.contact[i] = contact[i];
    r.op_mode = op_mode;
    r.finite = finite;
    // Publish LAST - see the file header's ordering note.
    ring_->header.write_seq.store(seq + 1, std::memory_order_release);
  }

  ~Writer() {
    if (ring_) munmap(ring_, sizeof(Ring));
    if (fd_ >= 0) close(fd_);
    // Deliberately NOT shm_unlink()ing here: the whole point is for the
    // reaper (a separate process) to read this AFTER the writer exits,
    // e.g. a crash that _exit()s mid-write. The launcher/reaper unlinks
    // once it has archived what it needs (see shm_reaper.py).
  }

 private:
  Writer() = default;
  Writer(const Writer&) = delete;

  void ensure_open() {
    if (opened_) return;
    opened_ = true;   // only try once per process, even on failure
    const char* inst = getenv("SIM_INSTANCE");
    char name[64];
    snprintf(name, sizeof(name), "/cheetah_trace_%s", inst ? inst : "0");
    // UNLINK FIRST. Two real failure modes if a same-named segment from
    // an earlier run is still sitting in shared memory (POSIX shm
    // deliberately outlives the process that created it - that is what
    // lets the reaper read a crash after the writer _exit()s, but it
    // means a NEW run's O_CREAT just reopens the OLD segment instead of
    // getting a fresh one): (1) on macOS specifically, ftruncate() on a
    // POSIX shm object can only succeed ONCE for that object's entire
    // lifetime - a second call, even to the identical size, fails EINVAL,
    // which silently disabled tracing for an entire run before this was
    // caught; (2) even where ftruncate re-succeeds, a reader that attaches
    // to an old run's already-populated ring before this run's first
    // write lands would read genuinely stale data and misattribute it to
    // the wrong run. Unlinking first guarantees O_CREAT actually creates.
    shm_unlink(name);
    fd_ = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (fd_ < 0) {
      perror("[shmtrace] shm_open failed - tracing disabled for this process");
      return;
    }
    if (ftruncate(fd_, sizeof(Ring)) != 0) {
      perror("[shmtrace] ftruncate failed - tracing disabled");
      close(fd_); fd_ = -1;
      return;
    }
    void* mem = mmap(nullptr, sizeof(Ring), PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (mem == MAP_FAILED) {
      perror("[shmtrace] mmap failed - tracing disabled");
      close(fd_); fd_ = -1;
      return;
    }
    ring_ = reinterpret_cast<Ring*>(mem);
    ring_->header.write_seq.store(0, std::memory_order_relaxed);
    ring_->header.capacity = RING_CAPACITY;
    ring_->header.record_size = sizeof(Record);
    ring_->header.magic = MAGIC;
    ring_->header.writer_pid = static_cast<uint32_t>(getpid());
    printf("[shmtrace] tracing to %s (%u records, %.1f MB)\n",
           name, RING_CAPACITY, (double)sizeof(Ring) / 1e6);
  }

  bool opened_ = false;
  int fd_ = -1;
  Ring* ring_ = nullptr;
};

// Free-function convenience wrapper - every call site just says
// shmtrace::log(...), matching the terseness a per-tick call site needs.
inline void log(const char* tag, double t, float roll, float pitch, float yaw,
                 float wx, float wy, float wz, float vx, float vy, float vz,
                 float z, float period_ms, uint8_t op_mode, uint8_t finite,
                 const float contact[4]) {
  Writer::instance().log(tag, t, roll, pitch, yaw, wx, wy, wz, vx, vy, vz,
                          z, period_ms, op_mode, finite, contact);
}

}  // namespace shmtrace

#endif  // CHEETAH_SHM_TRACE_H
