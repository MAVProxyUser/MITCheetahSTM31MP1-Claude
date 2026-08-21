/*!
 * @file PeriodicTask.cpp
 * @brief Implementation of a periodic function running in a separate thread.
 * Periodic tasks have a task manager, which measure how long they take to run.
 */
#ifdef linux
 #include <sys/timerfd.h>
#endif

#include <unistd.h>
#include <chrono>
#include <cmath>
#include <thread>

#include "Utilities/PeriodicTask.h"
#include "Utilities/Timer.h"
#include "Utilities/Utilities_print.h"


/*!
 * Construct a new task within a TaskManager
 * @param taskManager : Parent task manager
 * @param period : how often to run
 * @param name : name of task
 */
PeriodicTask::PeriodicTask(PeriodicTaskManager* taskManager, float period,
                           std::string name)
    : _period(period), _name(name) {
  taskManager->addTask(this);
}

/*!
 * Begin running task
 */
void PeriodicTask::start() {
  if (_running) {
    printf("[PeriodicTask] Tried to start %s but it was already running!\n",
           _name.c_str());
    return;
  }
  init();
  _running = true;
  _thread = std::thread(&PeriodicTask::loopFunction, this);
}

/*!
 * Stop running task
 */
void PeriodicTask::stop() {
  if (!_running) {
    printf("[PeriodicTask] Tried to stop %s but it wasn't running!\n",
           _name.c_str());
    return;
  }
  _running = false;
  printf("[PeriodicTask] Waiting for %s to stop...\n", _name.c_str());
  _thread.join();
  printf("[PeriodicTask] Done!\n");
  cleanup();
}

/*!
 * If max period is more than 30% over desired period, it is slow
 */
bool PeriodicTask::isSlow() {
  return _maxPeriod > _period * 1.3f || _maxRuntime > _period;
}

/*!
 * Reset max statistics
 */
void PeriodicTask::clearMax() {
  _maxPeriod = 0;
  _maxRuntime = 0;
}

/*!
 * Print the status of this task in the table format
 */
void PeriodicTask::printStatus() {
  if (!_running) return;
  if (isSlow()) {
    printf_color(PrintColor::Red, "|%-20s|%6.4f|%6.4f|%6.4f|%6.4f|%6.4f\n",
                 _name.c_str(), _lastRuntime, _maxRuntime, _period,
                 _lastPeriodTime, _maxPeriod);
  } else {
    printf("|%-20s|%6.4f|%6.4f|%6.4f|%6.4f|%6.4f\n", _name.c_str(),
           _lastRuntime, _maxRuntime, _period, _lastPeriodTime, _maxPeriod);
  }
}

/*!
 * Call the task in a timed loop.  Uses a timerfd
 */
void PeriodicTask::loopFunction() {
#ifdef linux
  auto timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
#endif
  int seconds = (int)_period;
  int nanoseconds = (int)(1e9 * std::fmod(_period, 1.f));

  Timer t;

#ifdef linux
  itimerspec timerSpec;
  timerSpec.it_interval.tv_sec = seconds;
  timerSpec.it_value.tv_sec = seconds;
  timerSpec.it_value.tv_nsec = nanoseconds;
  timerSpec.it_interval.tv_nsec = nanoseconds;

  timerfd_settime(timerFd, 0, &timerSpec, nullptr);
#else
  // NON-LINUX (Mac-first host build): upstream guards the timerfd calls but
  // leaves NO wait at all here, so the "periodic" task free-runs. Measured on
  // macOS: the 500 Hz control loop spun at ~1.9 MHz - 3700x real time - which
  // advances every gait timer, MPC segment counter and integrator at nonsense
  // rates and drops the robot on its face within a metre. Sleep to an ABSOLUTE
  // deadline (not a relative sleep, which accumulates the runtime as drift).
  const auto periodNs =
      std::chrono::nanoseconds((long long)((double)_period * 1e9));
  auto nextWake = std::chrono::steady_clock::now();
#endif
  unsigned long long missed = 0;

  printf("[PeriodicTask] Start %s (%d s, %d ns)\n", _name.c_str(), seconds,
         nanoseconds);
  while (_running) {
    _lastPeriodTime = (float)t.getSeconds();
    t.start();
    run();
    _lastRuntime = (float)t.getSeconds();
#ifdef linux
    int m = read(timerFd, &missed, sizeof(missed));
    (void)m;
#else
    nextWake += periodNs;
    auto now = std::chrono::steady_clock::now();
    if (nextWake < now) {
      // Overran the period. Re-base rather than spinning to "catch up", which
      // is what timerfd reports as a missed tick.
      ++missed;
      nextWake = now;
    } else {
      std::this_thread::sleep_until(nextWake);
    }
#endif
    _maxPeriod = std::max(_maxPeriod, _lastPeriodTime);
    _maxRuntime = std::max(_maxRuntime, _lastRuntime);
  }
  printf("[PeriodicTask] %s has stopped!\n", _name.c_str());
}

PeriodicTaskManager::~PeriodicTaskManager() {}

/*!
 * Add a new task to a task manager
 */
void PeriodicTaskManager::addTask(PeriodicTask* task) {
  _tasks.push_back(task);
}

/*!
 * Print the status of all tasks and rest max statistics
 */
void PeriodicTaskManager::printStatus() {
  printf("\n----------------------------TASKS----------------------------\n");
  printf("|%-20s|%-6s|%-6s|%-6s|%-6s|%-6s\n", "name", "rt", "rt-max", "T-des",
         "T-act", "T-max");
  printf("-----------------------------------------------------------\n");
  for (auto& task : _tasks) {
    task->printStatus();
    task->clearMax();
  }
  printf("-------------------------------------------------------------\n\n");
}

/*!
 * Print only the slow tasks
 */
void PeriodicTaskManager::printStatusOfSlowTasks() {
  for (auto& task : _tasks) {
    if (task->isSlow()) {
      task->printStatus();
      task->clearMax();
    }
  }
}

/*!
 * Stop all tasks
 */
void PeriodicTaskManager::stopAll() {
  for (auto& task : _tasks) {
    task->stop();
  }
}
