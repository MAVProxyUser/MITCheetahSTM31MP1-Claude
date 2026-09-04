/*! @file ShmTrace.h
 *  @brief Lock-free, single-writer shared-memory ring buffers for
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
 *  each one vanishing when the process exits. Per a SECOND direct
 *  instruction, every printf/fprintf debug call site in RobotRunner.cpp
 *  and mit_sim_main.cpp is now ALSO routed through here rather than
 *  stdio, so printf is fully retired as a debug mechanism from those two
 *  files - see the TEXT RING section below for why that needed a second,
 *  separate ring rather than overloading the numeric Record above.
 *
 *  Design (numeric/tick ring):
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
 *
 *  Design (text ring, for the printf replacement):
 *   - A SECOND, separate named segment per dog
 *     ("/cheetah_trace_text_<instance>") of fixed-size, pre-FORMATTED
 *     message records - not a generic structured-log redesign of Record
 *     above. The application-level printf call sites this replaces
 *     (nav progress, mission result, gait changes, FALL/STALL/recover,
 *     estimator debug, ...) each carry their own unrelated set of
 *     arguments (waypoint index, N/E, a gait name string, a corner
 *     radius, ...) that do not fit any one fixed numeric schema the way
 *     the tick ring's per-tick physical state does; formatting each
 *     message ONCE at the call site (a single vsnprintf into a stack
 *     buffer, same cost printf already paid for its OWN formatting, just
 *     without the stdio buffering/locking/potential-flush after it) and
 *     storing the finished string is far simpler than inventing a
 *     type-tagged variadic wire format for a Python reader to decode.
 *   - `logf()` below intentionally mirrors printf's own signature
 *     (format string + varargs) so every existing call site converts by
 *     replacing the function name and dropping the trailing "\n" (the
 *     text-tailing bridge re-adds it when it appends a record to the
 *     log file downstream tooling already parses - see
 *     shm_reaper.py's tail_text_to_log()).
 *   - Same lock-free single-writer / release-ordered write_seq scheme
 *     as the tick ring, and the same Header layout (capacity/record_size
 *     tell a generic reader which ring it is looking at).
 */
#ifndef CHEETAH_SHM_TRACE_H
#define CHEETAH_SHM_TRACE_H

#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <cstdlib>   // getenv/atoi for the run_id stamp
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
  float    kin_z;                 // THE DETECTOR'S OWN HEIGHT: -min over
                                   // legs of (rBody^T (hip + p))[2]. This is
                                   // what RobotRunner's fall test actually
                                   // consumes (it prefers this over the
                                   // estimate), and until 2026-09-03 nothing
                                   // logged it - so 438 archived falls could
                                   // say the ESTIMATOR's z leads the real
                                   // body by 0.86 s (OPEN-26) and could not
                                   // say whether the DETECTOR's height does.
  float    foot_z[4];             // per-leg (rBody^T (hip + p))[2], the terms
                                   // kin_z is the min of. Carried because the
                                   // foot-buckling hypothesis for OPEN-26
                                   // predicts the error tracks leg-geometry
                                   // change, and a min alone cannot show that.
  float    foot_fz[4];            // per-leg WORLD-FRAME FOOT SPEED, m/s (the
                                   // name is now historical - it began as a
                                   // force field; see RobotRunner.cpp for why
                                   // no force signal exists here). This is the
                                   // sensorless contact test: a planted foot
                                   // is stationary in the world. Added
                                   // 2026-09-04 because OPEN-26 found
                                   // the KF fully trusting a foot 0.165 m off
                                   // the ground, because there IS no contact
                                   // estimation - ContactEstimator::run()
                                   // copies the gait SCHEDULE. Whether a
                                   // force-based contact signal would have
                                   // told planted from airborne has to be
                                   // measured, not assumed. It cannot be, in
                                   // this build: SpiData has no torque field
                                   // and tauEstimate is assembled from
                                   // commands[], so nothing here observes what
                                   // a foot actually feels. See the long note
                                   // at the call site in RobotRunner.cpp. What
                                   // this field carries is what the MPC
                                   // believed it was pushing with.
  float    track_err[4];          // per-leg |pDes - p|, m: how far each foot
                                   // is from where the controller ASKED it to
                                   // be. Added 2026-09-04 to split OPEN-26 in
                                   // two: during the fold the legs retract by
                                   // 0.285 m against a 0.288 m standing
                                   // height, and nothing so far says whether
                                   // the controller COMMANDS that tuck (a
                                   // planning/MPC fault) or the legs go there
                                   // despite the command (a torque/tracking
                                   // fault). Those are different bugs with
                                   // different fixes and no measurement here
                                   // has yet distinguished them.
  uint8_t  op_mode;                // FSM_OperatingMode: 0 NORMAL,
                                   // 1 TRANSITIONING, 2 ESTOP, 3 EDAMP
  uint8_t  finite;                 // 1 if the state estimate was finite
                                   // this tick, 0 if the NaN guard fired
};
#pragma pack(pop)
// 150 bytes (98 -> 118 kin_z+foot_z, -> 134 foot_fz, -> 150 track_err) -
// measured directly with a standalone compile after hand-
// summing the field widths got it wrong twice in a row. Not required to
// be any particular round number since #pragma pack(1) already guarantees
// no compiler padding; the number only matters because the Python
// reaper's struct.unpack format string has to match it exactly.
static_assert(sizeof(Record) == 150, "Record layout must stay in sync with the Python reaper's struct format - update BOTH sides together");

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
  uint32_t run_id;        // THE conductor's run number ($SIM_RUN_ID), stamped
                          // into shared memory itself so run identity does not
                          // live only in a log line another process can append
                          // to. Per direct instruction: "the SHM should tag the
                          // run number and the conductor and logs and everything
                          // else should all use the same one for no confusion."
                          // It is already the same number in server.py's
                          // orchestration log, in each ctrl log's [RUNID] stamp
                          // and in every archived filename - this closes the
                          // last gap, the ring itself.
                          //
                          // Load-bearing, not decorative: the reaper had only
                          // writer_pid to tell "my controller" from "a dead
                          // one's leftovers", and a pid is reused by the OS and
                          // means nothing to a human reading a log. A run id is
                          // unique per launch, monotonic, and is the SAME token
                          // the operator already sees in the panel - so a
                          // mismatch is both checkable in code and obvious to a
                          // person. (A stale ring replayed into a fresh log
                          // produced a false PASS earlier tonight.)
  uint32_t _pad;          // keep write_seq's 8-byte alignment EXPLICIT rather
                          // than implicit, so the Python side's struct format
                          // stays an exact mirror instead of relying on the
                          // compiler's padding choice.
};
static_assert(sizeof(Header) == 32,
              "Header layout must stay in sync with shm_reaper.py's HEADER_FMT "
              "- update BOTH sides together");
constexpr uint32_t MAGIC = 0x43484554;  // "CHET"

struct Ring {
  Header header;
  Record records[RING_CAPACITY];
};

// ---- text ring: the printf replacement -----------------------------------
#pragma pack(push, 1)
struct TextRecord {
  double   t;         // elapsed mission time, seconds - same clock as Record
  uint64_t seq;
  char     msg[200];   // NUL-padded, pre-formatted (vsnprintf'd at the call
                       // site). 200 comfortably covers the longest existing
                       // call site's expansion (the [mission] gait-change
                       // line, ~150 chars worst case with a long gait name).
};
#pragma pack(pop)
// 216 bytes (8+8+200), measured consistent with pack(1)'s no-padding
// guarantee - unlike Record this was designed to a round input size rather
// than measured after the fact, but the Python side still asserts it.
static_assert(sizeof(TextRecord) == 216, "TextRecord layout must stay in sync with the Python reaper's struct format - update BOTH sides together");

constexpr uint32_t TEXT_RING_CAPACITY = 8192;
// Lower rate than the tick ring by design (event lines, not 500 Hz physical
// state) - 8192 is generous headroom (many minutes at any real event rate)
// at a total size (~1.8 MB) that costs nothing to keep mapped.

struct TextRing {
  Header header;
  TextRecord records[TEXT_RING_CAPACITY];
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
           const float contact[4], const float* kin5 = nullptr,
           const float* fz4 = nullptr, const float* te4 = nullptr) {
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
    // kin5 = {kin_z, foot_z[0..3]}. Optional so the recovery/stall call
    // sites that have no leg data stay untouched and write zeros.
    r.kin_z = kin5 ? kin5[0] : 0.f;
    for (int i = 0; i < 4; ++i) r.foot_z[i] = kin5 ? kin5[1 + i] : 0.f;
    for (int i = 0; i < 4; ++i) r.foot_fz[i] = fz4 ? fz4[i] : 0.f;
    for (int i = 0; i < 4; ++i) r.track_err[i] = te4 ? te4[i] : 0.f;
    r.op_mode = op_mode;
    r.finite = finite;
    // Publish LAST - see the file header's ordering note.
    ring_->header.write_seq.store(seq + 1, std::memory_order_release);
  }

  // The printf replacement: same call-site shape as printf (format string +
  // varargs), formatted ONCE here via vsnprintf into the record itself - no
  // stdio buffering/locking, no possible fflush stall, and (unlike printf)
  // this never blocks even if the eventual consumer (the tail-to-log bridge)
  // is slow or not running at all - a lagging reader just loses old ring
  // history, the writer never waits on it. Takes the va_list directly (not
  // `...`) because C has no way to forward one variadic call's `...` into
  // another - both the member-call convenience below and the free function
  // at the bottom of this file extract their own va_list and land here.
  void vlogf(double t, const char* fmt, va_list args) {
    ensure_text_open();
    if (!text_ring_) return;
    const uint64_t seq = text_ring_->header.write_seq.load(std::memory_order_relaxed);
    TextRecord& r = text_ring_->records[seq % TEXT_RING_CAPACITY];
    r.t = t;
    r.seq = seq;
    vsnprintf(r.msg, sizeof(r.msg), fmt, args);
    // Publish LAST - same ordering note as log() above.
    text_ring_->header.write_seq.store(seq + 1, std::memory_order_release);
  }

  void logf(double t, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vlogf(t, fmt, args);
    va_end(args);
  }

  ~Writer() {
    if (ring_) munmap(ring_, sizeof(Ring));
    if (fd_ >= 0) close(fd_);
    if (text_ring_) munmap(text_ring_, sizeof(TextRing));
    if (text_fd_ >= 0) close(text_fd_);
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
    ring_->header.run_id = static_cast<uint32_t>(
        getenv("SIM_RUN_ID") ? atoi(getenv("SIM_RUN_ID")) : 0);
    ring_->header._pad = 0;
    printf("[shmtrace] tracing to %s (%u records, %.1f MB)\n",
           name, RING_CAPACITY, (double)sizeof(Ring) / 1e6);
  }

  void ensure_text_open() {
    if (text_opened_) return;
    text_opened_ = true;
    const char* inst = getenv("SIM_INSTANCE");
    char name[64];
    snprintf(name, sizeof(name), "/cheetah_trace_text_%s", inst ? inst : "0");
    // Same shm_unlink()-first reasoning as ensure_open() above - a stale
    // segment from an earlier run on this instance number must not silently
    // reuse a not-fresh ftruncate(), and a reader must never see a mix of
    // this run's and a previous run's text.
    shm_unlink(name);
    text_fd_ = shm_open(name, O_CREAT | O_RDWR, 0666);
    if (text_fd_ < 0) {
      perror("[shmtrace] text shm_open failed - text logging disabled for this process");
      return;
    }
    if (ftruncate(text_fd_, sizeof(TextRing)) != 0) {
      perror("[shmtrace] text ftruncate failed - text logging disabled");
      close(text_fd_); text_fd_ = -1;
      return;
    }
    void* mem = mmap(nullptr, sizeof(TextRing), PROT_READ | PROT_WRITE, MAP_SHARED, text_fd_, 0);
    if (mem == MAP_FAILED) {
      perror("[shmtrace] text mmap failed - text logging disabled");
      close(text_fd_); text_fd_ = -1;
      return;
    }
    text_ring_ = reinterpret_cast<TextRing*>(mem);
    text_ring_->header.write_seq.store(0, std::memory_order_relaxed);
    text_ring_->header.capacity = TEXT_RING_CAPACITY;
    text_ring_->header.record_size = sizeof(TextRecord);
    text_ring_->header.magic = MAGIC;
    text_ring_->header.writer_pid = static_cast<uint32_t>(getpid());
    text_ring_->header.run_id = static_cast<uint32_t>(
        getenv("SIM_RUN_ID") ? atoi(getenv("SIM_RUN_ID")) : 0);
    text_ring_->header._pad = 0;
    // Deliberately no printf here (unlike ensure_open() above): this path
    // exists SPECIFICALLY to retire printf from the application's own
    // call sites, so announcing its own success via printf would be the
    // one line this whole mechanism failed to convert. The tick ring's
    // init line stays on printf as a bootstrap diagnostic for the tracing
    // system itself, which cannot log its own failure through itself.
  }

  bool opened_ = false;
  int fd_ = -1;
  Ring* ring_ = nullptr;
  bool text_opened_ = false;
  int text_fd_ = -1;
  TextRing* text_ring_ = nullptr;
};

// Free-function convenience wrapper - every call site just says
// shmtrace::log(...), matching the terseness a per-tick call site needs.
inline void log(const char* tag, double t, float roll, float pitch, float yaw,
                 float wx, float wy, float wz, float vx, float vy, float vz,
                 float z, float period_ms, uint8_t op_mode, uint8_t finite,
                 const float contact[4], const float* kin5 = nullptr,
           const float* fz4 = nullptr, const float* te4 = nullptr) {
  Writer::instance().log(tag, t, roll, pitch, yaw, wx, wy, wz, vx, vy, vz,
                          z, period_ms, op_mode, finite, contact, kin5, fz4, te4);
}

// The printf replacement call sites actually use: shmtrace::logf(t, "...",
// args...) drops in wherever printf("...\n", args...) used to be - same
// format string (minus the trailing newline, which the downstream
// tail-to-log bridge adds back) and same argument list, just with an
// explicit elapsed-time as the first argument (mirroring log() above)
// since a formatted message string carries no separate time field of its
// own for the archive/bridge to sort or correlate against the tick ring by.
__attribute__((format(printf, 2, 3)))
inline void logf(double t, const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  Writer::instance().vlogf(t, fmt, args);
  va_end(args);
}

}  // namespace shmtrace

#endif  // CHEETAH_SHM_TRACE_H
