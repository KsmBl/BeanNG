// A few helpers for the parts of a physics step that fall apart into independent pieces. Two thousand steps a second leave
// no time for waking a thread each time, so a helper spins for a moment after its work and only then goes to sleep.
#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <vector>
#if defined(__x86_64__)
#include <immintrin.h>
#endif

class Workers {
 public:
  static constexpr int kMost = 4;  // with the caller
  static Workers& get() {
    static Workers w;
    return w;
  }
  int threads() const { return int(helpers_.size()) + 1; }

  // Calls fn(task, slot) once for every task. Slot says which thread it is on, below threads(); slot 0 is the caller's.
  template <class F>
  void run(int tasks, F&& fn) {
    if (helpers_.empty() || tasks < 2) {
      for (int i = 0; i < tasks; i++) fn(i, 0);
      return;
    }
    using Fn = std::remove_reference_t<F>;
    call_ = [](void* c, int task, int slot) { (*static_cast<Fn*>(c))(task, slot); };
    context_ = &fn;
    tasks_ = tasks;
    next_.store(0, std::memory_order_relaxed);
    pending_.store(int(helpers_.size()), std::memory_order_relaxed);
    generation_.fetch_add(1, std::memory_order_release);
    if (sleeping_.load(std::memory_order_acquire) > 0) {
      std::lock_guard<std::mutex> lock(mutex_);
      wake_.notify_all();
    }
    work(0);
    while (pending_.load(std::memory_order_acquire) > 0) relax();
  }

 private:
  Workers() {
    int want = 0;
    const int cores = int(std::thread::hardware_concurrency());
    // Half of them are likely to be the other half of a core, and a game may be running beside the server.
    if (cores >= 4) want = std::min(cores / 2 - 1, kMost - 1);
    if (const char* e = std::getenv("BEANNG_THREADS")) want = std::atoi(e) - 1;
    want = want < 0 ? 0 : (want > kMost - 1 ? kMost - 1 : want);
    for (int i = 0; i < want; i++) helpers_.emplace_back([this, i] { helperLoop(i + 1); });
  }
  ~Workers() {
    quit_.store(true);
    generation_.fetch_add(1);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      wake_.notify_all();
    }
    for (std::thread& t : helpers_) t.join();
  }
  static void relax() {
#if defined(__x86_64__)
    _mm_pause();
#else
    std::this_thread::yield();
#endif
  }
  void work(int slot) {
    for (;;) {
      const int task = next_.fetch_add(1, std::memory_order_relaxed);
      if (task >= tasks_) return;
      call_(context_, task, slot);
    }
  }
  void helperLoop(int slot) {
    unsigned long seen = 0;
    for (;;) {
      int spins = 0;
      while (generation_.load(std::memory_order_acquire) == seen) {
        relax();
        if (++spins < 6000) continue;
        // Nothing for a while, which is about 200 microseconds of spinning.
        sleeping_.fetch_add(1, std::memory_order_acq_rel);
        {
          std::unique_lock<std::mutex> lock(mutex_);
          wake_.wait(lock, [&] { return generation_.load(std::memory_order_acquire) != seen; });
        }
        sleeping_.fetch_sub(1, std::memory_order_acq_rel);
        spins = 0;
      }
      seen = generation_.load(std::memory_order_acquire);
      if (quit_.load()) return;
      work(slot);
      pending_.fetch_sub(1, std::memory_order_acq_rel);
    }
  }

  std::vector<std::thread> helpers_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::atomic<unsigned long> generation_{0};
  std::atomic<int> next_{0}, pending_{0}, sleeping_{0};
  std::atomic<bool> quit_{false};
  void (*call_)(void*, int, int) = nullptr;
  void* context_ = nullptr;
  int tasks_ = 0;
};
