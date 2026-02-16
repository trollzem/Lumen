// local includes
#include "display_device/logging.h"
#include "fixtures/fixtures.h"

namespace {
  // Specialized TEST macro(s) for this test file
#define TEST_S(...) DD_MAKE_TEST(TEST, LoggingTest, __VA_ARGS__)
}  // namespace

TEST_S(LogLevelVerbose) {
  using level = display_device::Logger::LogLevel;
  auto &logger {display_device::Logger::get()};

  logger.setLogLevel(level::verbose);

  EXPECT_EQ(logger.isLogLevelEnabled(level::verbose), true);
  EXPECT_EQ(logger.isLogLevelEnabled(level::debug), true);
  EXPECT_EQ(logger.isLogLevelEnabled(level::info), true);
  EXPECT_EQ(logger.isLogLevelEnabled(level::warning), true);
  EXPECT_EQ(logger.isLogLevelEnabled(level::error), true);
  EXPECT_EQ(logger.isLogLevelEnabled(level::fatal), true);
}

TEST_S(LogLevelDebug) {
  using level = display_device::Logger::LogLevel;
  auto &logger {display_device::Logger::get()};

  logger.setLogLevel(level::debug);

  EXPECT_EQ(logger.isLogLevelEnabled(level::verbose), false);
  EXPECT_EQ(logger.isLogLevelEnabled(level::debug), true);
  EXPECT_EQ(logger.isLogLevelEnabled(level::info), true);
  EXPECT_EQ(logger.isLogLevelEnabled(level::warning), true);
  EXPECT_EQ(logger.isLogLevelEnabled(level::error), true);
  EXPECT_EQ(logger.isLogLevelEnabled(level::fatal), true);
}

TEST_S(LogLevelInfo) {
  using level = display_device::Logger::LogLevel;
  auto &logger {display_device::Logger::get()};

  logger.setLogLevel(level::info);

  EXPECT_EQ(logger.isLogLevelEnabled(level::verbose), false);
  EXPECT_EQ(logger.isLogLevelEnabled(level::debug), false);
  EXPECT_EQ(logger.isLogLevelEnabled(level::info), true);
  EXPECT_EQ(logger.isLogLevelEnabled(level::warning), true);
  EXPECT_EQ(logger.isLogLevelEnabled(level::error), true);
  EXPECT_EQ(logger.isLogLevelEnabled(level::fatal), true);
}

TEST_S(LogLevelWarning) {
  using level = display_device::Logger::LogLevel;
  auto &logger {display_device::Logger::get()};

  logger.setLogLevel(level::warning);

  EXPECT_EQ(logger.isLogLevelEnabled(level::verbose), false);
  EXPECT_EQ(logger.isLogLevelEnabled(level::debug), false);
  EXPECT_EQ(logger.isLogLevelEnabled(level::info), false);
  EXPECT_EQ(logger.isLogLevelEnabled(level::warning), true);
  EXPECT_EQ(logger.isLogLevelEnabled(level::error), true);
  EXPECT_EQ(logger.isLogLevelEnabled(level::fatal), true);
}

TEST_S(LogLevelError) {
  using level = display_device::Logger::LogLevel;
  auto &logger {display_device::Logger::get()};

  logger.setLogLevel(level::error);

  EXPECT_EQ(logger.isLogLevelEnabled(level::verbose), false);
  EXPECT_EQ(logger.isLogLevelEnabled(level::debug), false);
  EXPECT_EQ(logger.isLogLevelEnabled(level::info), false);
  EXPECT_EQ(logger.isLogLevelEnabled(level::warning), false);
  EXPECT_EQ(logger.isLogLevelEnabled(level::error), true);
  EXPECT_EQ(logger.isLogLevelEnabled(level::fatal), true);
}

TEST_S(LogLevelFatal) {
  using level = display_device::Logger::LogLevel;
  auto &logger {display_device::Logger::get()};

  logger.setLogLevel(level::fatal);

  EXPECT_EQ(logger.isLogLevelEnabled(level::verbose), false);
  EXPECT_EQ(logger.isLogLevelEnabled(level::debug), false);
  EXPECT_EQ(logger.isLogLevelEnabled(level::info), false);
  EXPECT_EQ(logger.isLogLevelEnabled(level::warning), false);
  EXPECT_EQ(logger.isLogLevelEnabled(level::error), false);
  EXPECT_EQ(logger.isLogLevelEnabled(level::fatal), true);
}

TEST_S(DefaultLogger) {
  using level = display_device::Logger::LogLevel;
  auto &logger {display_device::Logger::get()};

  const auto write_and_get_cout {[this, &logger](level level, std::string value) -> std::string {
    m_cout_buffer.str(std::string {});  // reset the buffer
    logger.write(level, std::move(value));
    return m_cout_buffer.str();
  }};

  logger.setLogLevel(level::verbose);
  // clang-format off
  EXPECT_TRUE(testRegex(write_and_get_cout(level::verbose, "Hello World!"), R"(\[\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}.\d{3}\] VERBOSE: Hello World!\n)"));
  EXPECT_TRUE(testRegex(write_and_get_cout(level::debug, "Hello World!"),   R"(\[\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}.\d{3}\] DEBUG:   Hello World!\n)"));
  EXPECT_TRUE(testRegex(write_and_get_cout(level::info, "Hello World!"),    R"(\[\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}.\d{3}\] INFO:    Hello World!\n)"));
  EXPECT_TRUE(testRegex(write_and_get_cout(level::warning, "Hello World!"), R"(\[\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}.\d{3}\] WARNING: Hello World!\n)"));
  EXPECT_TRUE(testRegex(write_and_get_cout(level::error, "Hello World!"),   R"(\[\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}.\d{3}\] ERROR:   Hello World!\n)"));
  EXPECT_TRUE(testRegex(write_and_get_cout(level::fatal, "Hello World!"),   R"(\[\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}.\d{3}\] FATAL:   Hello World!\n)"));
  // clang-format on
}

TEST_S(CustomCallback) {
  using level = display_device::Logger::LogLevel;
  using level_t = std::underlying_type_t<level>;
  auto &logger {display_device::Logger::get()};

  std::string output;
  logger.setLogLevel(level::verbose);
  logger.setCustomCallback([&output](const level level, const std::string &value) {
    output = std::to_string(static_cast<level_t>(level)) + " " + value;
  });

  logger.write(level::verbose, "Hello World!");
  EXPECT_EQ(output, "0 Hello World!");
  EXPECT_TRUE(m_cout_buffer.str().empty());

  logger.write(level::debug, "Hello World!");
  EXPECT_EQ(output, "1 Hello World!");
  EXPECT_TRUE(m_cout_buffer.str().empty());

  logger.write(level::info, "Hello World!");
  EXPECT_EQ(output, "2 Hello World!");
  EXPECT_TRUE(m_cout_buffer.str().empty());

  logger.write(level::warning, "Hello World!");
  EXPECT_EQ(output, "3 Hello World!");
  EXPECT_TRUE(m_cout_buffer.str().empty());

  logger.write(level::error, "Hello World!");
  EXPECT_EQ(output, "4 Hello World!");
  EXPECT_TRUE(m_cout_buffer.str().empty());

  logger.write(level::fatal, "Hello World!");
  EXPECT_EQ(output, "5 Hello World!");
  EXPECT_TRUE(m_cout_buffer.str().empty());
}

TEST_S(WriteMethodRespectsLogLevel, DefaultLogger) {
  using level = display_device::Logger::LogLevel;
  auto &logger {display_device::Logger::get()};

  EXPECT_TRUE(m_cout_buffer.str().empty());

  logger.setLogLevel(level::error);
  logger.write(level::info, "Hello World!");
  EXPECT_TRUE(m_cout_buffer.str().empty());

  logger.setLogLevel(level::info);
  logger.write(level::info, "Hello World!");
  EXPECT_FALSE(m_cout_buffer.str().empty());
}

TEST_S(WriteMethodRespectsLogLevel, CustomCallback) {
  using level = display_device::Logger::LogLevel;
  auto &logger {display_device::Logger::get()};

  bool callback_invoked {false};
  logger.setCustomCallback([&callback_invoked](auto, auto) {
    callback_invoked = true;
  });

  logger.setLogLevel(level::error);
  logger.write(level::info, "Hello World!");
  EXPECT_EQ(callback_invoked, false);

  logger.setLogLevel(level::info);
  logger.write(level::info, "Hello World!");
  EXPECT_EQ(callback_invoked, true);
}

TEST_S(LogMacroDisablesStreamChain) {
  using level = display_device::Logger::LogLevel;
  auto &logger {display_device::Logger::get()};

  bool output_logged {false};
  logger.setCustomCallback([&output_logged](auto, auto) {
    output_logged = true;
  });

  bool some_function_invoked {false};
  const auto some_function {[&some_function_invoked]() {
    some_function_invoked = true;
    return "some string";
  }};

  logger.setLogLevel(level::error);
  DD_LOG(info) << some_function();
  EXPECT_EQ(output_logged, false);
  EXPECT_EQ(some_function_invoked, false);

  logger.setLogLevel(level::info);
  DD_LOG(info) << some_function();
  EXPECT_EQ(output_logged, true);
  EXPECT_EQ(some_function_invoked, true);
}
