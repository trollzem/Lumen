// standard includes
#include <array>
#include <filesystem>

// lib includes
#include <gtest/gtest.h>

// test includes
#include "tests/utils.h"

// Undefine the original TEST macro
#undef TEST

// Redefine TEST to use our BaseTest class, to automatically use our BaseTest fixture
#define TEST(test_case_name, test_name) \
  GTEST_TEST_(test_case_name, test_name, ::BaseTest, ::testing::internal::GetTypeId<::BaseTest>())

/**
 * @brief Base class for tests.
 *
 * This class provides a base test fixture for all tests.
 *
 * ``cout``, ``stderr``, and ``stdout`` are redirected to a buffer, and the buffer is printed if the test fails.
 *
 * @todo Retain the color of the original output.
 */
class BaseTest: public ::testing::Test {
protected:
  // https://stackoverflow.com/a/58369622/11214013

  // we can possibly use some internal googletest functions to capture stdout and stderr, but I have not tested this
  // https://stackoverflow.com/a/33186201/11214013

  BaseTest():
      sbuf {nullptr},
      pipe_stdout {nullptr},
      pipe_stderr {nullptr} {
    // intentionally empty
  }

  ~BaseTest() override = default;

  void SetUp() override {
    // todo: only run this one time, instead of every time a test is run
    // see: https://stackoverflow.com/questions/2435277/googletest-accessing-the-environment-from-a-test
    // get command line args from the test executable
    testArgs = ::testing::internal::GetArgvs();

    // then get the directory of the test executable
    // std::string path = ::testing::internal::GetArgvs()[0];
    testBinary = testArgs[0];

    // get the directory of the test executable
    testBinaryDir = std::filesystem::path(testBinary).parent_path();

    // If testBinaryDir is empty or `.` then set it to the current directory
    // maybe some better options here: https://stackoverflow.com/questions/875249/how-to-get-current-directory
    if (testBinaryDir.empty() || testBinaryDir.string() == ".") {
      testBinaryDir = std::filesystem::current_path();
    }

    sbuf = std::cout.rdbuf();  // save cout buffer (std::cout)
    std::cout.rdbuf(cout_buffer.rdbuf());  // redirect cout to buffer (std::cout)
  }

  void TearDown() override {
    std::cout.rdbuf(sbuf);  // restore cout buffer

    // get test info
    const ::testing::TestInfo *const test_info = ::testing::UnitTest::GetInstance()->current_test_info();

    if (test_info->result()->Failed()) {
      std::cout << std::endl
                << "Test failed: " << test_info->name() << std::endl
                << std::endl
                << "Captured cout:" << std::endl
                << cout_buffer.str() << std::endl
                << "Captured stdout:" << std::endl
                << stdout_buffer.str() << std::endl
                << "Captured stderr:" << std::endl
                << stderr_buffer.str() << std::endl;
    }

    sbuf = nullptr;  // clear sbuf
    if (pipe_stdout) {
      pclose(pipe_stdout);
      pipe_stdout = nullptr;
    }
    if (pipe_stderr) {
      pclose(pipe_stderr);
      pipe_stderr = nullptr;
    }
  }

  // functions and variables
  std::vector<std::string> testArgs;  // CLI arguments used
  std::filesystem::path testBinary;  // full path of this binary
  std::filesystem::path testBinaryDir;  // full directory of this binary
  std::stringstream cout_buffer;  // declare cout_buffer
  std::stringstream stdout_buffer;  // declare stdout_buffer
  std::stringstream stderr_buffer;  // declare stderr_buffer
  std::streambuf *sbuf;
  FILE *pipe_stdout;
  FILE *pipe_stderr;

  int exec(const char *cmd) {
    std::array<char, 128> buffer {};
    pipe_stdout = popen((std::string(cmd) + " 2>&1").c_str(), "r");
    pipe_stderr = popen((std::string(cmd) + " 2>&1").c_str(), "r");
    if (!pipe_stdout || !pipe_stderr) {
      throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe_stdout) != nullptr) {
      stdout_buffer << buffer.data();
    }
    while (fgets(buffer.data(), buffer.size(), pipe_stderr) != nullptr) {
      stderr_buffer << buffer.data();
    }
    int returnCode = pclose(pipe_stdout);
    pipe_stdout = nullptr;
    if (returnCode != 0) {
      std::cout << "Error: " << stderr_buffer.str() << std::endl
                << "Return code: " << returnCode << std::endl;
    }
    return returnCode;
  }
};

class LinuxTest: public BaseTest {
protected:
  void SetUp() override {
#ifndef __linux__
    GTEST_SKIP_("Skipping, this test is for Linux only.");
#endif
  }

  void TearDown() override {
    BaseTest::TearDown();
  }
};

class MacOSTest: public BaseTest {
protected:
  void SetUp() override {
#if !defined(__APPLE__) || !defined(__MACH__)
    GTEST_SKIP_("Skipping, this test is for macOS only.");
#endif
  }

  void TearDown() override {
    BaseTest::TearDown();
  }
};

class WindowsTest: public BaseTest {
protected:
  void SetUp() override {
#ifndef _WIN32
    GTEST_SKIP_("Skipping, this test is for Windows only.");
#endif
    BaseTest::SetUp();
  }

  void TearDown() override {
    BaseTest::TearDown();
  }
};
