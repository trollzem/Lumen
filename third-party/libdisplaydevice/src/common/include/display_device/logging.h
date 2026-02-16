/**
 * @file src/common/include/display_device/logging.h
 * @brief Declarations for the logging utility.
 */
#pragma once

// system includes
#include <functional>
#include <sstream>
#include <string>

namespace display_device {
  /**
   * @brief A singleton class for logging or re-routing logs.
   *
   * This class is not meant to be used directly (only for configuration).
   * Instead, the MACRO below should be used throughout the code for logging.
   *
   * @note A lazy-evaluated, correctly-destroyed, thread-safe singleton pattern is used here (https://stackoverflow.com/a/1008289).
   */
  class Logger {
  public:
    /**
     * @brief Defines the possible log levels.
     * @note Level implicitly includes all other levels below it.
     * @note All levels are in lower-case on purpose to fit the "BOOST_LOG(info)" style.
     */
    enum class LogLevel {
      verbose = 0,  ///< Verbose level
      debug,  ///< Debug level
      info,  ///< Info level
      warning,  ///< Warning level
      error,  ///< Error level
      fatal  ///< Fatal level
    };

    /**
     * @brief Defines the callback type for log data re-routing.
     */
    using Callback = std::function<void(LogLevel, std::string)>;

    /**
     * @brief Get the singleton instance.
     * @returns Singleton instance for the class.
     * @examples
     * Logger& logger { Logger::get() };
     * @examples_end
     */
    static Logger &get();

    /**
     * @brief Set the log level for the logger.
     * @param log_level New level to be used.
     * @examples
     * Logger::get().setLogLevel(Logger::LogLevel::Info);
     * @examples_end
     */
    void setLogLevel(LogLevel log_level);

    /**
     * @brief Check if log level is currently enabled.
     * @param log_level Log level to check.
     * @returns True if log level is enabled.
     * @examples
     * const bool is_enabled { Logger::get().isLogLevelEnabled(Logger::LogLevel::Info) };
     * @examples_end
     */
    [[nodiscard]] bool isLogLevelEnabled(LogLevel log_level) const;

    /**
     * @brief Set custom callback for writing the logs.
     * @param callback New callback to be used or nullptr to reset to the default.
     * @examples
     * Logger::get().setCustomCallback([](const LogLevel level, std::string value){
     *    // write to file or something
     * });
     * @examples_end
     */
    void setCustomCallback(Callback callback);

    /**
     * @brief Write the string to the output (via callback) if the log level is enabled.
     * @param log_level Log level to be checked and (probably) written.
     * @param value A copy of the string to be written.
     * @examples
     * Logger::get().write(Logger::LogLevel::Info, "Hello World!");
     * @examples_end
     */
    void write(LogLevel log_level, std::string value);

    /**
     * @brief A deleted copy constructor for singleton pattern.
     * @note Public to ensure better compiler error message.
     */
    Logger(Logger const &) = delete;

    /**
     * @brief A deleted assignment operator for singleton pattern.
     * @note Public to ensure better compiler error message.
     */
    void operator=(Logger const &) = delete;

  private:
    /**
     * @brief A private constructor to ensure the singleton pattern.
     */
    explicit Logger();

    LogLevel m_enabled_log_level; /**< The currently enabled log level. */
    Callback m_custom_callback; /**< Custom callback to pass log data to. */
  };

  /**
   * @brief A helper class for accumulating output via the stream operator and then writing it out at once.
   */
  class LogWriter {
  public:
    /**
     * @brief Constructor scoped writer utility.
     * @param log_level Level to be used when writing out the output.
     */
    explicit LogWriter(Logger::LogLevel log_level);

    /**
     * @brief Write out the accumulated output.
     */
    virtual ~LogWriter();

    /**
     * @brief Stream value to the buffer.
     * @param value Arbitrary value to be written to the buffer.
     * @returns Reference to the writer utility for chaining the operators.
     */
    template<class T>
    LogWriter &operator<<(T &&value) {
      m_buffer << std::forward<T>(value);
      return *this;
    }

  private:
    Logger::LogLevel m_log_level; /**< Log level to be used. */
    std::ostringstream m_buffer; /**< Buffer to hold all the output. */
  };
}  // namespace display_device

/**
 * @brief Helper MACRO that disables output string computation if log level is not enabled.
 * @examples
 * DD_LOG(info) << "Hello World!" << " " << 123;
 * DD_LOG(error) << "OH MY GAWD!";
 * @examples_end
 */
#define DD_LOG(level) \
  for (bool is_enabled {display_device::Logger::get().isLogLevelEnabled(display_device::Logger::LogLevel::level)}; is_enabled; is_enabled = false) \
  display_device::LogWriter(display_device::Logger::LogLevel::level)
