/**
 * @file src/common/include/display_device/retry_scheduler.h
 * @brief Declarations for the RetryScheduler.
 */
#pragma once

// system includes
#include <algorithm>
#include <condition_variable>
#include <functional>
#include <memory>
#include <thread>

// local includes
#include "logging.h"

namespace display_device {
  /**
   * @brief A convenience class for stopping the RetryScheduler.
   *
   * It is conceptually similar to `std::stop_token` except that it also uses
   * RAII to perform a cleanup. This allows to return void types
   * in the RetryScheduler without a hassle.
   */
  class SchedulerStopToken final {
  public:
    /**
     * @brief Default constructor.
     * @param cleanup Function to be executed once the destructor is called (object goes out of scope).
     */
    explicit SchedulerStopToken(std::function<void()> cleanup);

    /**
     * @brief Deleted copy constructor.
     */
    SchedulerStopToken(const SchedulerStopToken &) = delete;

    /**
     * @brief Deleted copy operator.
     */
    SchedulerStopToken &operator=(const SchedulerStopToken &) = delete;

    /**
     * @brief Executes cleanup logic if scheduler stop was requested.
     */
    ~SchedulerStopToken();

    /**
     * @brief Request the scheduler to be stopped.
     */
    void requestStop();

    /**
     * @brief Check if stop was requested.
     * @return True if stop was requested, false otherwise.
     */
    [[nodiscard]] bool stopRequested() const;

  private:
    bool m_stop_requested {false};
    std::function<void()> m_cleanup;
  };

  namespace detail {
    /**
     * @brief Given that we know that we are dealing with a function,
     *        check if it is an optional function (like std::function<...>) or other callable.
     */
    template<class FunctionT>
    concept OptionalFunction = requires(FunctionT exec_fn) {
      static_cast<bool>(exec_fn);
    };

    /**
     * @brief A convenience template struct helper for adding const to the type.
     */
    template<class T, bool AddConst>
    struct AutoConst;

    template<class T>
    struct AutoConst<T, false> {
      using type = T;
    };

    template<class T>
    struct AutoConst<T, true> {
      using type = std::add_const_t<T>;
    };

    /**
     * @brief A convenience template helper for adding const to the type.
     */
    template<class T, bool AddConst>
    using auto_const_t = typename AutoConst<T, AddConst>::type;

    /**
     * @brief Check if the function signature matches the acceptable signature for RetryScheduler::execute
     *        without a stop token.
     */
    template<class T, class FunctionT>
    concept ExecuteWithoutStopToken = requires(FunctionT exec_fn, T &value) { exec_fn(value); };

    /**
     * @brief Check if the function signature matches the acceptable signature for RetryScheduler::execute
     *        with a stop token.
     */
    template<class T, class FunctionT>
    concept ExecuteWithStopToken = requires(FunctionT exec_fn, T &value, SchedulerStopToken &token) {
      exec_fn(value, token);
    };

    /**
     * @brief Check if the function signature matches the acceptable signature for RetryScheduler::execute.
     */
    template<class T, class FunctionT>
    concept ExecuteCallbackLike = ExecuteWithoutStopToken<T, FunctionT> || ExecuteWithStopToken<T, FunctionT>;
  }  // namespace detail

  /**
   * @brief Scheduler options to be used when scheduling executor function.
   */
  struct SchedulerOptions {
    /**
     * @brief Defines the executor's execution logic when it is scheduled.
     */
    enum class Execution {
      Immediate,  ///< Executor is executed in the calling thread immediately and scheduled afterward.
      ImmediateWithSleep,  ///< The first sleep duration is TAKEN from `m_sleep_durations` and the calling thread is put to sleep. Once awoken, follows by same logic as `Immediate`.
      ScheduledOnly  ///< Executor is executed in the thread only.
    };

    std::vector<std::chrono::milliseconds> m_sleep_durations;  ///< Specifies for long the scheduled thread sleeps before invoking executor. Last duration is reused indefinitely.
    Execution m_execution {Execution::Immediate};  ///< Executor's execution logic.
  };

  /**
   * @brief A wrapper class around an interface that provides a thread-safe access to the
   *        interface and allows to schedule arbitrary logic for it to retry until it succeeds.
   * @note The scheduler is designed to only schedule 1 callback at a time, until it is either
   *       replaced or stopped.
   */
  template<class T>
  class RetryScheduler final {
  public:
    /**
     * @brief Default constructor.
     * @param iface Interface to be passed around to the executor functions.
     */
    explicit RetryScheduler(std::unique_ptr<T> iface):
        m_iface {iface ? std::move(iface) : throw std::logic_error {"Nullptr interface provided in RetryScheduler!"}},
        m_thread {[this]() {
          std::unique_lock lock {m_mutex};
          while (m_keep_alive) {
            m_syncing_thread = false;
            if (auto duration {takeNextDuration(m_sleep_durations)}; duration > std::chrono::milliseconds::zero()) {
              // We're going to sleep until manually woken up or the time elapses.
              m_sleep_cv.wait_for(lock, duration, [this]() {
                return m_syncing_thread;
              });
            } else {
              // We're going to sleep until manually woken up.
              m_sleep_cv.wait(lock, [this]() {
                return m_syncing_thread;
              });
            }

            if (m_syncing_thread) {
              // Thread was waken up to sync sleep time or to be stopped.
              continue;
            }

            try {
              SchedulerStopToken scheduler_stop_token {[&]() {
                clearThreadLoopUnlocked();
              }};
              m_retry_function(*m_iface, scheduler_stop_token);
              continue;
            } catch (const std::exception &error) {
              DD_LOG(error) << "Exception thrown in the RetryScheduler thread. Stopping scheduler. Error:\n"
                            << error.what();
            }

            clearThreadLoopUnlocked();
          }
        }} {
    }

    /**
     * @brief A destructor that gracefully shuts down the thread.
     */
    ~RetryScheduler() {
      {
        std::lock_guard lock {m_mutex};
        m_keep_alive = false;
        syncThreadUnlocked();
      }

      m_thread.join();
    }

    /**
     * @brief Schedule an interface executor function to be executed at specified intervals.
     * @param exec_fn Provides thread-safe access to the interface for executing arbitrary logic.
     *                It accepts a `stop_token` as a second parameter which can be used to stop
     *                the scheduler.
     * @param options Options for the scheduler.
     * @note Previously scheduled executor is replaced by a new one!
     * @examples
     * std::unique_ptr<SettingsManagerInterface> iface = getIface(...);
     * RetryScheduler<SettingsManagerInterface> scheduler{std::move(iface)};
     *
     * scheduler.schedule([](SettingsManagerInterface& iface, SchedulerStopToken& stop_token){
     *   if (iface.revertSettings()) {
     *     stop_token.requestStop();
     *   }
     * }, { .m_sleep_durations = { 50ms, 10ms });
     * @examples_end
     */
    void schedule(std::function<void(T &, SchedulerStopToken &stop_token)> exec_fn, const SchedulerOptions &options) {
      if (!exec_fn) {
        throw std::logic_error {"Empty callback function provided in RetryScheduler::schedule!"};
      }

      if (options.m_sleep_durations.empty()) {
        throw std::logic_error {"At least 1 sleep duration must be specified in RetryScheduler::schedule!"};
      }

      if (std::ranges::any_of(options.m_sleep_durations, [&](const auto &duration) {
            return duration == std::chrono::milliseconds::zero();
          })) {
        throw std::logic_error {"All of the durations specified in RetryScheduler::schedule must be larger than a 0!"};
      }

      std::lock_guard lock {m_mutex};
      SchedulerStopToken stop_token {[&]() {
        stopUnlocked();
      }};

      // We are catching the exception here instead of propagating to have
      // similar try...catch login as in the scheduler thread.
      try {
        auto sleep_durations = options.m_sleep_durations;
        if (options.m_execution != SchedulerOptions::Execution::ScheduledOnly) {
          if (options.m_execution == SchedulerOptions::Execution::ImmediateWithSleep) {
            std::this_thread::sleep_for(takeNextDuration(sleep_durations));
          }

          exec_fn(*m_iface, stop_token);
        }

        if (!stop_token.stopRequested()) {
          m_retry_function = std::move(exec_fn);
          m_sleep_durations = std::move(sleep_durations);
          syncThreadUnlocked();
        }
      } catch (const std::exception &error) {
        stop_token.requestStop();
        DD_LOG(error) << "Exception thrown in the RetryScheduler::schedule. Stopping scheduler. Error:\n"
                      << error.what();
      }
    }

    /**
     * @brief A non-const variant of the `executeImpl` method. See it for details.
     */
    template<class FunctionT>
    auto execute(FunctionT &&exec_fn) {
      return executeImpl(*this, std::forward<FunctionT>(exec_fn));
    }

    /**
     * @brief A const variant of the `executeImpl` method. See it for details.
     */
    template<class FunctionT>
    auto execute(FunctionT &&exec_fn) const {
      return executeImpl(*this, std::forward<FunctionT>(exec_fn));
    }

    /**
     * @brief Check whether anything is scheduled for execution.
     * @return True if something is scheduled, false otherwise.
     */
    [[nodiscard]] bool isScheduled() const {
      return static_cast<bool>(m_retry_function);
    }

    /**
     * @brief Stop the scheduled function - will no longer be execute once THIS method returns.
     */
    void stop() {
      std::lock_guard lock {m_mutex};
      stopUnlocked();
    }

  private:
    static std::chrono::milliseconds takeNextDuration(std::vector<std::chrono::milliseconds> &durations) {
      if (durations.size() > 1) {
        const auto front_it {std::begin(durations)};
        const auto front_value {*front_it};
        durations.erase(front_it);
        return front_value;
      }

      return durations.empty() ? std::chrono::milliseconds::zero() : durations.back();
    }

    /**
     * @brief Execute arbitrary logic using the provided interface in a thread-safe manner.
     * @param self A reference to *this.
     * @param exec_fn Provides thread-safe access to the interface for executing arbitrary logic.
     *                Acceptable function signatures are:
     *                  - AnyReturnType(T &);
     *                  - AnyReturnType(T &, SchedulerStopToken& stop_token),
     *                    `stop_token` is an optional parameter that allows to stop scheduler during
     *                    the same call.
     * @return Return value from the executor callback.
     * @note This method is not to be used directly. Intead the `execute` method is to be used.
     * @examples
     * std::unique_ptr<SettingsManagerInterface> iface = getIface(...);
     * RetryScheduler<SettingsManagerInterface> scheduler{std::move(iface)};
     * const std::string device_id { "MY_DEVICE_ID" };
     *
     * // Without stop token:
     * const auto display_name = scheduler.execute([&](SettingsManagerInterface& iface) {
     *   return iface.getDisplayName(device_id);
     * });
     *
     * // With stop token:
     * scheduler.schedule([](SettingsManagerInterface& iface, SchedulerStopToken& stop_token) {
     *   if (iface.revertSettings()) {
     *     stop_token.requestStop();
     *   }
     * });
     *
     * scheduler.execute([&](SettingsManagerInterface& iface, SchedulerStopToken& stop_token) {
     *   if (true) {
     *     // Some condition was met and we no longer want to revert settings!
     *     stop_token.requestStop();
     *   }
     * });
     * @examples_end
     */
    static auto executeImpl(auto &self, auto &&exec_fn)
      requires detail::ExecuteCallbackLike<T, decltype(exec_fn)>
    {
      using FunctionT = decltype(exec_fn);
      constexpr bool IsConst = std::is_const_v<std::remove_reference_t<decltype(self)>>;

      if constexpr (detail::OptionalFunction<FunctionT>) {
        if (!exec_fn) {
          throw std::logic_error {"Empty callback function provided in RetryScheduler::execute!"};
        }
      }

      std::lock_guard lock {self.m_mutex};
      detail::auto_const_t<std::decay_t<T>, IsConst> &iface_ref {*self.m_iface};
      if constexpr (detail::ExecuteWithStopToken<T, FunctionT>) {
        detail::auto_const_t<SchedulerStopToken, IsConst> stop_token {[&self]() {
          if constexpr (!IsConst) {
            self.stopUnlocked();
          }
        }};
        return std::forward<FunctionT>(exec_fn)(iface_ref, stop_token);
      } else {
        return std::forward<FunctionT>(exec_fn)(iface_ref);
      }
    }

    /**
     * @brief Clear the necessary data so that the thread will go into a deep sleep.
     */
    void clearThreadLoopUnlocked() {
      m_sleep_durations = {};
      m_retry_function = nullptr;
    }

    /**
     * @brief Manually wake up the thread for synchronization.
     */
    void syncThreadUnlocked() {
      m_syncing_thread = true;
      m_sleep_cv.notify_one();
    }

    /**
     * @brief Stop the scheduled function.
     */
    void stopUnlocked() {
      if (isScheduled()) {
        clearThreadLoopUnlocked();
        syncThreadUnlocked();
      }
    }

    std::unique_ptr<T> m_iface; /**< Interface to be passed around to the executor functions. */
    std::vector<std::chrono::milliseconds> m_sleep_durations; /**< Sleep times for the timer. */
    std::function<void(T &, SchedulerStopToken &)> m_retry_function {nullptr}; /**< Function to be executed until it succeeds. */

    mutable std::mutex m_mutex {}; /**< A mutex for synchronizing thread and "external" access. */
    std::condition_variable m_sleep_cv {}; /**< Condition variable for waking up thread. */
    bool m_syncing_thread {false}; /**< Safeguard for the condition variable to prevent sporadic thread wake-ups. */
    bool m_keep_alive {true}; /**< When set to false, scheduler thread will exit. */

    // Always the last in the list so that all the members are already initialized!
    std::thread m_thread; /**< A scheduler thread. */
  };
}  // namespace display_device
