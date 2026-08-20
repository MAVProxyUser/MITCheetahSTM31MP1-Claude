/*!
 * Null LCM shim for the STM32MP1 headless port.
 *
 * Stock LCM needs glib, which is painful to cross-compile (and unnecessary for a
 * headless robot running from YAML config). The Cheetah code only ever uses LCM as
 * lcm::LCM with publish()/subscribe()/handle()/good() and lcm::ReceiveBuffer in
 * handler signatures -- it never encodes/decodes messages directly. So this shim
 * provides that exact surface with no-op implementations: publishes go nowhere,
 * subscriptions never fire, handle() blocks briefly. The full stack compiles and
 * links with its LCM calls intact and runs standalone.
 *
 * To add real networked operator tooling later, replace this with a minimal
 * LCM-UDPM transport (the udpm wire format is small and glib-free) -- no changes
 * to the robot code required.
 */
#ifndef STM32MP1_LCM_SHIM_CPP_HPP
#define STM32MP1_LCM_SHIM_CPP_HPP

#include <string>
#include <ctime>

namespace lcm {

// Opaque receive-buffer type used in subscription handler signatures.
class ReceiveBuffer {
 public:
  const void* data = nullptr;
  uint32_t data_size = 0;
  int64_t recv_utime = 0;
};

// Returned by subscribe(); nothing to configure in the null transport.
class Subscription {
 public:
  void setQueueCapacity(int) {}
};

class LCM {
 public:
  explicit LCM(std::string /*lcm_url*/ = "") {}
  ~LCM() {}

  //! Always "good" so initialisation proceeds.
  bool good() const { return true; }
  int  getFileno() const { return -1; }

  //! Publish is a no-op: never touches the message, so POD types (no encode) work.
  template <class MessageType>
  int publish(const std::string& /*channel*/, const MessageType* /*msg*/) { return 0; }

  //! Subscribe records nothing; handlers never fire in the null transport.
  template <class MessageType, class MessageHandlerClass>
  Subscription* subscribe(
      const std::string& /*channel*/,
      void (MessageHandlerClass::*/*handler*/)(const ReceiveBuffer*, const std::string&, const MessageType*),
      MessageHandlerClass* /*context*/) {
    return &sub_;
  }

  //! No messages ever arrive; sleep briefly so callers' spin loops don't busy-wait.
  int handle() {
    struct timespec ts{0, 5 * 1000 * 1000};  // 5 ms
    nanosleep(&ts, nullptr);
    return 0;
  }
  int handleTimeout(int timeout_ms) {
    struct timespec ts{timeout_ms / 1000, (long)(timeout_ms % 1000) * 1000 * 1000};
    nanosleep(&ts, nullptr);
    return 0;
  }

 private:
  Subscription sub_;
};

}  // namespace lcm

#endif  // STM32MP1_LCM_SHIM_CPP_HPP
