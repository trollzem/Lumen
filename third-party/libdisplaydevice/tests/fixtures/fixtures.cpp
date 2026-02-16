// header include
#include "fixtures/fixtures.h"

// system includes
#include <regex>

void BaseTest::SetUp() {
  if (const auto skip_reason {skipTest()}; !skip_reason.empty()) {
    m_test_skipped_at_setup = true;
    GTEST_SKIP() << skip_reason;
  }

  if (isOutputSuppressed()) {
    // See https://stackoverflow.com/a/58369622/11214013
    m_sbuf = std::cout.rdbuf();  // save cout buffer (std::cout)
    std::cout.rdbuf(m_cout_buffer.rdbuf());  // redirect cout to buffer (std::cout)
  }

  // Set the default log level, before the test starts. Will default to verbose in case nothing was specified.
  display_device::Logger::get().setLogLevel(getDefaultLogLevel().value_or(display_device::Logger::LogLevel::verbose));
}

void BaseTest::TearDown() {
  if (m_test_skipped_at_setup) {
    // We are not using the IsSkipped() state here. Here we are skipping
    // teardown, because we have skipped the setup entirely, but during normal
    // skips we still want to do teardown.
    return;
  }

  // reset the callback to avoid potential leaks
  display_device::Logger::get().setCustomCallback(nullptr);

  // Restore cout buffer and print the suppressed output out in case we have failed :/
  if (isOutputSuppressed()) {
    std::cout.rdbuf(m_sbuf);
    m_sbuf = nullptr;

    const auto test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    if (test_info && test_info->result()->Failed()) {
      std::cout << std::endl
                << "Test failed: " << test_info->name() << std::endl
                << std::endl
                << "Captured cout:" << std::endl
                << m_cout_buffer.str() << std::endl;
    }
  }
}

const std::vector<std::string> &BaseTest::getArgs() const {
  static const auto args {::testing::internal::GetArgvs()};
  return args;
}

std::optional<std::string> BaseTest::getArgWithMatchingPattern(const std::string &pattern, bool remove_match) const {
  const auto &args {getArgs()};
  if (!args.empty()) {
    const std::regex re_pattern {pattern};

    // We are skipping the first arg which is always binary name/path.
    for (auto it {std::next(std::begin(args))}; it != std::end(args); ++it) {
      if (std::smatch match; std::regex_search(*it, match, re_pattern)) {
        return remove_match ? std::regex_replace(*it, re_pattern, "") : *it;
      }
    }
  }

  return std::nullopt;
}

bool BaseTest::isOutputSuppressed() const {
  return true;
}

bool BaseTest::isSystemTest() const {
  return false;
}

std::string BaseTest::skipTest() const {
  if (isSystemTest()) {
    const static bool is_system_test_skippable {
      []() {
        const auto value {getEnv("SKIP_SYSTEM_TESTS")};
        return value == "1";
      }()
    };

    if (is_system_test_skippable) {
      return "Skipping, this system test is disabled via SKIP_SYSTEM_TESTS=1 env.";
    }
  }
  return {};
}

std::optional<display_device::Logger::LogLevel> BaseTest::getDefaultLogLevel() const {
  const static auto default_log_level {
    []() -> std::optional<display_device::Logger::LogLevel> {
      const auto value {getEnv("LOG_LEVEL")};
      if (value == "verbose") {
        return display_device::Logger::LogLevel::verbose;
      }
      if (value == "debug") {
        return display_device::Logger::LogLevel::debug;
      }
      if (value == "info") {
        return display_device::Logger::LogLevel::info;
      }
      if (value == "warning") {
        return display_device::Logger::LogLevel::warning;
      }
      if (value == "error") {
        return display_device::Logger::LogLevel::error;
      }

      return std::nullopt;
    }()
  };

  return default_log_level;
}
