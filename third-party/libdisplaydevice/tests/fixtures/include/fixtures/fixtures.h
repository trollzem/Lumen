#pragma once

// system includes
#include <boost/preprocessor/comparison/equal.hpp>
#include <boost/preprocessor/control/if.hpp>
#include <boost/preprocessor/seq/fold_left.hpp>
#include <boost/preprocessor/variadic/to_seq.hpp>
#include <filesystem>
#include <gtest/gtest.h>

// local includes
#include "display_device/logging.h"
#include "test_utils.h"

// Undefine the original TEST macro
#undef TEST

// Redefine TEST to use our BaseTest class, to automatically use our BaseTest fixture
#define TEST(test_case_name, test_name) \
  GTEST_TEST_(test_case_name, test_name, ::BaseTest, ::testing::internal::GetTypeId<::BaseTest>())

// Helper macros for concatenating macro variables with an underscore separator (https://stackoverflow.com/questions/74505380/how-to-concatenate-join-va-args-with-delimiters-separators)
#define DD_CAT_OP_(index, state, elem) BOOST_PP_CAT(state, BOOST_PP_CAT(_, elem))
#define DD_CAT_SEQ_(seq) BOOST_PP_SEQ_FOLD_LEFT(DD_CAT_OP_, BOOST_PP_SEQ_HEAD(seq), BOOST_PP_SEQ_TAIL(seq))
#define DD_CAT_VA_MULTIPLE_(...) DD_CAT_SEQ_(BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__))
#define DD_CAT_VA_SINGLE_(...) __VA_ARGS__
#define DD_CAT_VA_ARGS_(...) BOOST_PP_IF(BOOST_PP_EQUAL(BOOST_PP_VARIADIC_SIZE(__VA_ARGS__), 1), DD_CAT_VA_SINGLE_, DD_CAT_VA_MULTIPLE_)(__VA_ARGS__)

// A macro for making the actual test macro.
// Usage example:
//   For normal tests:
//    #define TEST_S(...) DD_MAKE_TEST(TEST, SomeTestSuite, __VA_ARGS__)
//   For tests with fixtures:
//    #define TEST_F_S(...) DD_MAKE_TEST(TEST_F, SomeTestFixture, __VA_ARGS__)
#define DD_MAKE_TEST(test_macro, test_suite_name, ...) test_macro(test_suite_name, DD_CAT_VA_ARGS_(__VA_ARGS__))

/**
 * @brief Base class for tests.
 *
 * This class provides a base test fixture for all tests.
 */
class BaseTest: public ::testing::Test {
protected:
  ~BaseTest() override = default;

  void SetUp() override;

  void TearDown() override;

  /**
   * @brief Get available command line arguments.
   * @return Command line args from GTest.
   */
  [[nodiscard]] virtual const std::vector<std::string> &getArgs() const;

  /**
   * @brief Get the command line argument that matches the pattern.
   * @param pattern Pattern to look for.
   * @param remove_match Specify if the matched pattern should be removed before returning argument.
   * @return Matching command line argument or null optional if nothing matched.
   */
  [[nodiscard]] virtual std::optional<std::string> getArgWithMatchingPattern(const std::string &pattern, bool remove_match) const;

  /**
   * @brief Check if the test output is to be redirected and printed out only if test fails.
   * @return True if output is to be suppressed, false otherwise.
   * @note It is useful for suppressing noise in automatic tests, but not so much in manual ones.
   */
  [[nodiscard]] virtual bool isOutputSuppressed() const;

  /**
   * @brief Check if the test interacts/modifies with the system settings.
   * @returns True if it does, false otherwise.
   * @note By setting SKIP_SYSTEM_TESTS=1 env, these tests will be skipped (useful during development).
   */
  [[nodiscard]] virtual bool isSystemTest() const;

  /**
   * @brief Skip the test by specifying the reason.
   * @returns A non-empty string (reason) if test needs to be skipped, empty string otherwise.
   */
  [[nodiscard]] virtual std::string skipTest() const;

  /**
   * @brief Get the default log level for the test base.
   * @returns A log level set in the env OR null optional if fallback should be used (verbose).
   * @note By setting LOG_LEVEL=<level> env you can change the level (e.g. LOG_LEVEL=error).
   */
  [[nodiscard]] virtual std::optional<display_device::Logger::LogLevel> getDefaultLogLevel() const;

  std::stringstream m_cout_buffer; /**< Stores the cout in case the output is suppressed. */

private:
  std::streambuf *m_sbuf {nullptr}; /**< Stores the handle to the original cout stream. */
  bool m_test_skipped_at_setup {false}; /**< Indicates whether the SetUp method was skipped. */
};
