/**
 * @file src/common/logging.cpp
 * @brief Definitions for the logging utility.
 */
#if !defined(_MSC_VER) && !defined(_POSIX_THREAD_SAFE_FUNCTIONS)
  #define _POSIX_THREAD_SAFE_FUNCTIONS  // For localtime_r
#endif

// class header include
#include "display_device/logging.h"

// system includes
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>

namespace display_device {
  namespace {
    std::tm threadSafeLocaltime(const std::time_t &time) {
#if defined(_MSC_VER)  // MSVCRT (2005+): std::localtime is threadsafe
      const auto tm_ptr {std::localtime(&time)};
#else  // POSIX
      std::tm buffer;
      const auto tm_ptr {localtime_r(&time, &buffer)};
#endif  // _MSC_VER
      if (tm_ptr) {
        return *tm_ptr;
      }
      return {};
    }
  }  // namespace

  Logger &Logger::get() {
    static Logger instance;  // GCOVR_EXCL_BR_LINE for some reason...
    return instance;
  }

  void Logger::setLogLevel(const LogLevel log_level) {
    m_enabled_log_level = log_level;
  }

  bool Logger::isLogLevelEnabled(LogLevel log_level) const {
    const auto log_level_v {static_cast<std::underlying_type_t<LogLevel>>(log_level)};
    const auto enabled_log_level_v {static_cast<std::underlying_type_t<LogLevel>>(m_enabled_log_level)};
    return log_level_v >= enabled_log_level_v;
  }

  void Logger::setCustomCallback(Callback callback) {
    m_custom_callback = std::move(callback);
  }

  void Logger::write(const LogLevel log_level, std::string value) {
    if (!isLogLevelEnabled(log_level)) {
      return;
    }

    if (m_custom_callback) {
      m_custom_callback(log_level, std::move(value));
      return;
    }

    std::stringstream stream;
    {
      // Time (limited by GCC 10, so it's not pretty...)
      {
        const auto now {std::chrono::system_clock::now()};
        const auto now_ms {std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch())};
        const auto now_s {std::chrono::duration_cast<std::chrono::seconds>(now_ms)};

        const std::time_t time {std::chrono::system_clock::to_time_t(now)};
        const auto localtime {threadSafeLocaltime(time)};
        const auto now_decimal_part {now_ms - now_s};

        const auto old_flags {stream.flags()};  // Save formatting flags so that they can be restored...
        stream << std::put_time(&localtime, "[%Y-%m-%d %H:%M:%S.") << std::setfill('0') << std::setw(3) << now_decimal_part.count() << "] ";
        stream.flags(old_flags);
      }

      // Log level
      switch (log_level) {  // GCOVR_EXCL_BR_LINE for when there is no case match...
        case LogLevel::verbose:
          stream << "VERBOSE: ";
          break;
        case LogLevel::debug:
          stream << "DEBUG:   ";
          break;
        case LogLevel::info:
          stream << "INFO:    ";
          break;
        case LogLevel::warning:
          stream << "WARNING: ";
          break;
        case LogLevel::error:
          stream << "ERROR:   ";
          break;
        case LogLevel::fatal:
          stream << "FATAL:   ";
          break;
      }

      // Value
      stream << value;
    }

    static std::mutex log_mutex;
    std::lock_guard lock {log_mutex};
    std::cout << stream.rdbuf() << std::endl;
  }

  Logger::Logger():
      m_enabled_log_level {LogLevel::info} {
  }

  LogWriter::LogWriter(const Logger::LogLevel log_level):
      m_log_level {log_level} {}

  LogWriter::~LogWriter() {
    Logger::get().write(m_log_level, m_buffer.str());
  }
}  // namespace display_device
