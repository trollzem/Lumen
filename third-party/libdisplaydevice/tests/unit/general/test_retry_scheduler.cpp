// system includes
#include <gmock/gmock.h>

// local includes
#include "display_device/retry_scheduler.h"
#include "fixtures/fixtures.h"

namespace {
  using namespace std::chrono_literals;

  // Convenience keywords for GMock
  using ::testing::HasSubstr;

  // A dummy struct for the tests
  struct TestIface {
    std::vector<int> m_durations;

    void nonConstMethod() { /* noop */ }

    void constMethod() const { /* noop */ }
  };

  // Some threads wake up a little earlier than expected, so we round the lower bound to
  // 99% of the expected value to stabilize the tests
  int roundTo99(int value) {
    return static_cast<int>(std::round(value * 0.99));
  }

  // Test fixture(s) for this file
  class RetrySchedulerTest: public BaseTest {
  public:
    display_device::RetryScheduler<TestIface> m_impl {std::make_unique<TestIface>()};
  };

  // Specialized TEST macro(s) for this test file
#define TEST_F_S(...) DD_MAKE_TEST(TEST_F, RetrySchedulerTest, __VA_ARGS__)
}  // namespace

TEST_F_S(NullptrInterfaceProvided) {
  EXPECT_THAT([]() {
    const display_device::RetryScheduler<TestIface> scheduler(nullptr);
  },
              ThrowsMessage<std::logic_error>(HasSubstr("Nullptr interface provided in RetryScheduler!")));
}

TEST_F_S(Schedule, NullptrCallbackProvided) {
  EXPECT_THAT([&]() {
    m_impl.schedule(nullptr, {.m_sleep_durations = {0ms}});
  },
              ThrowsMessage<std::logic_error>(HasSubstr("Empty callback function provided in RetryScheduler::schedule!")));
}

TEST_F_S(Schedule, NoDurations) {
  EXPECT_THAT([&]() {
    m_impl.schedule([](auto, auto &) {
    },
                    {.m_sleep_durations = {}});
  },
              ThrowsMessage<std::logic_error>(HasSubstr("At least 1 sleep duration must be specified in RetryScheduler::schedule!")));
}

TEST_F_S(Schedule, ZeroDuration) {
  EXPECT_THAT([&]() {
    m_impl.schedule([](auto, auto &) {
    },
                    {.m_sleep_durations = {0ms}});
  },
              ThrowsMessage<std::logic_error>(HasSubstr("All of the durations specified in RetryScheduler::schedule must be larger than a 0!")));
}

TEST_F_S(Schedule, SchedulingDurations) {
  // Note: in this test we care that the delay is not less than the requested one, but we
  //       do not really have an upper ceiling...

  const auto schedule_and_get_average_delays {[&](const std::vector<std::chrono::milliseconds> &durations) {
    m_impl.execute([](TestIface &iface) {
      iface.m_durations.clear();
    });

    std::optional<std::chrono::high_resolution_clock::time_point> prev;
    m_impl.schedule([&durations, &prev](TestIface &iface, auto &stop_token) {
      auto now = std::chrono::high_resolution_clock::now();
      if (prev) {
        iface.m_durations.push_back(static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now - *prev).count()));
        if (iface.m_durations.size() == durations.size()) {
          stop_token.requestStop();
        }
      }

      prev = now;
    },
                    {.m_sleep_durations = durations});

    while (m_impl.isScheduled()) {
      std::this_thread::sleep_for(1ms);
    }

    return m_impl.execute([](TestIface &iface) {
      int sum {0};
      for (const auto timing : iface.m_durations) {
        sum += timing;
      }
      return sum / iface.m_durations.size();
    });
  }};

  EXPECT_GE(schedule_and_get_average_delays({10ms, 10ms, 10ms, 10ms, 10ms, 10ms, 10ms, 10ms, 10ms, 10ms}), 10);
  EXPECT_GE(schedule_and_get_average_delays({50ms, 50ms, 50ms, 50ms, 50ms, 50ms, 50ms, 50ms, 50ms, 50ms}), 50);
  EXPECT_GE(schedule_and_get_average_delays({10ms, 20ms, 30ms, 40ms, 50ms, 10ms, 50ms, 10ms, 50ms, 10ms}), 28);
}

TEST_F_S(Schedule, SchedulerInteruptAndReplacement) {
  int counter_a {0};
  m_impl.schedule([&counter_a](auto, auto &) {
    counter_a++;
  },
                  {.m_sleep_durations = {5ms}});

  while (counter_a < 3) {
    std::this_thread::sleep_for(1ms);
  }

  int counter_a_last_value {0};
  int counter_b {0};
  m_impl.schedule([&counter_a_last_value, &counter_a, &counter_b](auto, auto &) {
    std::this_thread::sleep_for(15ms);
    counter_a_last_value = counter_a;
    counter_b++;
  },
                  {.m_sleep_durations = {1ms}});

  while (counter_b < 3) {
    std::this_thread::sleep_for(1ms);
  }

  // Counter A no longer incremented since the function was replaced
  EXPECT_EQ(counter_a_last_value, counter_a);

  // Stop the scheduler to avoid SEGFAULTS
  m_impl.stop();
}

TEST_F_S(Schedule, StoppedImmediately) {
  m_impl.schedule([&](auto, auto &stop_token) {
    stop_token.requestStop();
  },
                  {.m_sleep_durations = {1000ms}});

  EXPECT_FALSE(m_impl.isScheduled());
}

TEST_F_S(Schedule, Execution, Immediate) {
  const auto default_duration {500ms};
  const auto calling_thread_id {std::this_thread::get_id()};
  std::optional<std::thread::id> first_call_scheduler_thread_id;
  std::optional<std::thread::id> second_call_scheduler_thread_id;

  int first_call_delay {-1};
  int second_call_delay {-1};
  auto prev = std::chrono::high_resolution_clock::now();
  m_impl.schedule([&](auto, auto &stop_token) {
    const auto now = std::chrono::high_resolution_clock::now();
    const auto duration = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now - prev).count());
    prev = now;

    if (!first_call_scheduler_thread_id) {
      first_call_delay = duration;
      first_call_scheduler_thread_id = std::this_thread::get_id();
      return;
    }

    second_call_delay = duration;
    second_call_scheduler_thread_id = std::this_thread::get_id();
    stop_token.requestStop();
  },
                  {.m_sleep_durations = {default_duration * 2, default_duration}, .m_execution = display_device::SchedulerOptions::Execution::Immediate});

  while (m_impl.isScheduled()) {
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_GE(first_call_delay, roundTo99(0));
  EXPECT_LT(first_call_delay, default_duration.count());
  EXPECT_GE(second_call_delay, roundTo99(default_duration.count() * 2));
  EXPECT_LT(second_call_delay, default_duration.count() * 3);

  EXPECT_TRUE(first_call_scheduler_thread_id);
  EXPECT_TRUE(second_call_scheduler_thread_id);

  EXPECT_EQ(*first_call_scheduler_thread_id, calling_thread_id);
  EXPECT_NE(*first_call_scheduler_thread_id, *second_call_scheduler_thread_id);
}

TEST_F_S(Schedule, Execution, ImmediateWithSleep) {
  const auto default_duration {500ms};
  const auto calling_thread_id {std::this_thread::get_id()};
  std::optional<std::thread::id> first_call_scheduler_thread_id;
  std::optional<std::thread::id> second_call_scheduler_thread_id;

  int first_call_delay {-1};
  int second_call_delay {-1};
  auto prev = std::chrono::high_resolution_clock::now();
  m_impl.schedule([&](auto, auto &stop_token) {
    const auto now = std::chrono::high_resolution_clock::now();
    const auto duration = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now - prev).count());
    prev = now;

    if (!first_call_scheduler_thread_id) {
      first_call_delay = duration;
      first_call_scheduler_thread_id = std::this_thread::get_id();
      return;
    }

    second_call_delay = duration;
    second_call_scheduler_thread_id = std::this_thread::get_id();
    stop_token.requestStop();
  },
                  {.m_sleep_durations = {default_duration * 2, default_duration}, .m_execution = display_device::SchedulerOptions::Execution::ImmediateWithSleep});

  while (m_impl.isScheduled()) {
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_GE(first_call_delay, roundTo99(default_duration.count() * 2));
  EXPECT_LT(first_call_delay, default_duration.count() * 3);
  EXPECT_GE(second_call_delay, roundTo99(default_duration.count()));
  EXPECT_LT(second_call_delay, default_duration.count() * 2);

  EXPECT_TRUE(first_call_scheduler_thread_id);
  EXPECT_TRUE(second_call_scheduler_thread_id);

  EXPECT_EQ(*first_call_scheduler_thread_id, calling_thread_id);
  EXPECT_NE(*first_call_scheduler_thread_id, *second_call_scheduler_thread_id);
}

TEST_F_S(Schedule, Execution, ScheduledOnly) {
  const auto default_duration {500ms};
  const auto calling_thread_id {std::this_thread::get_id()};
  std::optional<std::thread::id> first_call_scheduler_thread_id;
  std::optional<std::thread::id> second_call_scheduler_thread_id;

  int first_call_delay {-1};
  int second_call_delay {-1};
  auto prev = std::chrono::high_resolution_clock::now();
  m_impl.schedule([&](auto, auto &stop_token) {
    const auto now = std::chrono::high_resolution_clock::now();
    const auto duration = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now - prev).count());
    prev = now;

    if (!first_call_scheduler_thread_id) {
      first_call_delay = duration;
      first_call_scheduler_thread_id = std::this_thread::get_id();
      return;
    }

    second_call_delay = duration;
    second_call_scheduler_thread_id = std::this_thread::get_id();
    stop_token.requestStop();
  },
                  {.m_sleep_durations = {default_duration * 2, default_duration}, .m_execution = display_device::SchedulerOptions::Execution::ScheduledOnly});

  while (m_impl.isScheduled()) {
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_GE(first_call_delay, roundTo99(default_duration.count() * 2));
  EXPECT_LT(first_call_delay, default_duration.count() * 3);
  EXPECT_GE(second_call_delay, roundTo99(default_duration.count()));
  EXPECT_LT(second_call_delay, default_duration.count() * 2);

  EXPECT_TRUE(first_call_scheduler_thread_id);
  EXPECT_TRUE(second_call_scheduler_thread_id);

  EXPECT_NE(*first_call_scheduler_thread_id, calling_thread_id);
  EXPECT_EQ(*first_call_scheduler_thread_id, *second_call_scheduler_thread_id);
}

TEST_F_S(Schedule, ExceptionThrown, DuringImmediateCall) {
  auto &logger {display_device::Logger::get()};

  int counter_a {0};
  m_impl.schedule([&](auto, auto &) {
    counter_a++;
  },
                  {.m_sleep_durations = {1ms}});
  while (counter_a < 3) {
    std::this_thread::sleep_for(1ms);
  }

  std::string output;
  logger.setCustomCallback([&output](auto, const std::string &value) {
    output = value;
  });

  EXPECT_TRUE(m_impl.isScheduled());
  m_impl.schedule([&](auto, auto &) {
    throw std::runtime_error("Get rekt!");
  },
                  {.m_sleep_durations = {1ms}});
  EXPECT_FALSE(m_impl.isScheduled());
  EXPECT_EQ(output, "Exception thrown in the RetryScheduler::schedule. Stopping scheduler. Error:\nGet rekt!");

  // Verify that scheduler still works
  int counter_b {0};
  m_impl.schedule([&](auto, auto &) {
    counter_b++;
  },
                  {.m_sleep_durations = {1ms}});
  while (counter_b < 3) {
    std::this_thread::sleep_for(1ms);
  }

  // Stop the scheduler to avoid SEGFAULTS
  m_impl.stop();
}

TEST_F_S(Schedule, ExceptionThrown, DuringScheduledCall) {
  auto &logger {display_device::Logger::get()};

  std::string output;
  logger.setCustomCallback([&output](auto, const std::string &value) {
    output = value;
  });

  bool first_call {true};
  EXPECT_EQ(output, "");
  m_impl.schedule([&](auto, auto &) {
    if (!first_call) {
      throw std::runtime_error("Get rekt!");
    }
    first_call = false;
  },
                  {.m_sleep_durations = {1ms}});

  while (m_impl.isScheduled()) {
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_EQ(output, "Exception thrown in the RetryScheduler thread. Stopping scheduler. Error:\nGet rekt!");

  // Verify that scheduler still works
  int counter {0};
  m_impl.schedule([&](auto, auto &) {
    counter++;
  },
                  {.m_sleep_durations = {1ms}});
  while (counter < 3) {
    std::this_thread::sleep_for(1ms);
  }

  // Stop the scheduler to avoid SEGFAULTS
  m_impl.stop();
}

TEST_F_S(Execute, NonConst, NullptrCallbackProvided) {
  EXPECT_THAT([this]() {
    auto &non_const_impl {m_impl};
    non_const_impl.execute(std::function<void(TestIface &)> {});
  },
              ThrowsMessage<std::logic_error>(HasSubstr("Empty callback function provided in RetryScheduler::execute!")));
}

TEST_F_S(Execute, Const, NullptrCallbackProvided) {
  // Note: this test verifies that non-const method is invoked from const method.
  EXPECT_THAT([this]() {
    auto &const_impl {m_impl};
    const_impl.execute(std::function<void(TestIface &)> {});
  },
              ThrowsMessage<std::logic_error>(HasSubstr("Empty callback function provided in RetryScheduler::execute!")));
}

TEST_F_S(Execute, SchedulerNotStopped) {
  int counter {0};
  m_impl.schedule([&](auto, auto &) {
    counter++;
  },
                  {.m_sleep_durations = {1ms}});
  while (counter < 3) {
    std::this_thread::sleep_for(1ms);
  }

  int counter_before_sleep {0};
  int counter_after_sleep {0};
  m_impl.execute([&](auto, auto &) {
    counter_before_sleep = counter;
    std::this_thread::sleep_for(15ms);
    counter_after_sleep = counter;
  });

  while (counter <= counter_after_sleep) {
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_EQ(counter_before_sleep, counter_after_sleep);
  EXPECT_GT(counter, counter_after_sleep);

  // Stop the scheduler to avoid SEGFAULTS
  m_impl.stop();
}

TEST_F_S(Execute, SchedulerStopped) {
  int counter {0};
  m_impl.schedule([&](auto, auto &) {
    counter++;
  },
                  {.m_sleep_durations = {1ms}});
  while (counter < 3) {
    std::this_thread::sleep_for(1ms);
  }

  int counter_before_sleep {0};
  int counter_after_sleep {0};
  m_impl.execute([&](auto, auto &stop_token) {
    counter_before_sleep = counter;
    std::this_thread::sleep_for(15ms);
    counter_after_sleep = counter;
    stop_token.requestStop();
  });

  EXPECT_FALSE(m_impl.isScheduled());
  EXPECT_EQ(counter_before_sleep, counter_after_sleep);
  EXPECT_EQ(counter, counter_after_sleep);
}

TEST_F_S(Execute, SchedulerStopped, ExceptThatItWasNotRunning) {
  EXPECT_FALSE(m_impl.isScheduled());
  m_impl.execute([](auto, auto &stop_token) {
    stop_token.requestStop();
  });
  EXPECT_FALSE(m_impl.isScheduled());

  int counter {0};
  m_impl.schedule([&](auto, auto &) {
    counter++;
  },
                  {.m_sleep_durations = {1ms}});
  while (counter < 3) {
    std::this_thread::sleep_for(1ms);
  }

  // Stop the scheduler to avoid SEGFAULTS
  m_impl.stop();
}

TEST_F_S(Execute, ExceptionThrown) {
  int counter {0};
  m_impl.schedule([&](auto, auto &) {
    counter++;
  },
                  {.m_sleep_durations = {1ms}});
  while (counter < 3) {
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_THAT([&]() {
    m_impl.execute([](auto) {
      throw std::runtime_error("Get rekt!");
    });
  },
              ThrowsMessage<std::runtime_error>(HasSubstr("Get rekt!")));

  EXPECT_TRUE(m_impl.isScheduled());

  // Stop the scheduler to avoid SEGFAULTS
  m_impl.stop();
}

TEST_F_S(Execute, ExceptionThrown, BeforeStopToken) {
  int counter {0};
  m_impl.schedule([&](auto, auto &) {
    counter++;
  },
                  {.m_sleep_durations = {1ms}});
  while (counter < 3) {
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_THAT([&]() {
    m_impl.execute([](auto, auto &stop_token) {
      throw std::runtime_error("Get rekt!");
      stop_token.requestStop();
    });
  },
              ThrowsMessage<std::runtime_error>(HasSubstr("Get rekt!")));

  EXPECT_TRUE(m_impl.isScheduled());

  // Stop the scheduler to avoid SEGFAULTS
  m_impl.stop();
}

TEST_F_S(Execute, ExceptionThrown, AfterStopToken) {
  int counter {0};
  m_impl.schedule([&](auto, auto &) {
    counter++;
  },
                  {.m_sleep_durations = {1ms}});
  while (counter < 3) {
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_THAT([&]() {
    m_impl.execute([](auto, auto &stop_token) {
      stop_token.requestStop();
      throw std::runtime_error("Get rekt!");
    });
  },
              ThrowsMessage<std::runtime_error>(HasSubstr("Get rekt!")));

  EXPECT_FALSE(m_impl.isScheduled());
}

TEST_F_S(Execute, ConstVsNonConst, WithoutStopToken) {
  const auto const_callback = [](const TestIface &iface) {
    iface.constMethod();
  };
  const auto non_const_callback = [](TestIface &iface) {
    iface.nonConstMethod();
  };
  const auto const_callback_auto = [](const auto &iface) {
    iface.constMethod();
  };
  const auto non_const_callback_auto = [](auto &iface) {
    iface.nonConstMethod();
  };

  // Verify it compiles with non-const
  auto &non_const_impl {m_impl};
  non_const_impl.execute(const_callback);
  non_const_impl.execute(non_const_callback);
  non_const_impl.execute(const_callback_auto);
  non_const_impl.execute(non_const_callback_auto);

  // Verify it compiles with const (commented out code will not compile)
  const auto &const_impl {m_impl};
  const_impl.execute(const_callback);
  // const_impl.execute(non_const_callback);
  const_impl.execute(const_callback_auto);
  // const_impl.execute(non_const_callback_auto);
}

TEST_F_S(Execute, ConstVsNonConst, WithStopToken) {
  const auto const_const_callback = [](const TestIface &iface, const display_device::SchedulerStopToken &token) {
    iface.constMethod();
    (void) token.stopRequested();
  };
  const auto const_non_const_callback = [](const TestIface &iface, display_device::SchedulerStopToken &token) {
    iface.constMethod();
    token.requestStop();
  };
  const auto non_const_const_callback = [](TestIface &iface, const display_device::SchedulerStopToken &token) {
    iface.nonConstMethod();
    (void) token.stopRequested();
  };
  const auto non_const_non_const_callback = [](TestIface &iface, display_device::SchedulerStopToken &token) {
    iface.nonConstMethod();
    token.requestStop();
  };
  const auto const_const_callback_auto = [](const auto &iface, const auto &token) {
    iface.constMethod();
    (void) token.stopRequested();
  };
  const auto const_non_const_callback_auto = [](const auto &iface, auto &token) {
    iface.constMethod();
    token.requestStop();
  };
  const auto non_const_const_callback_auto = [](auto &iface, const auto &token) {
    iface.nonConstMethod();
    (void) token.stopRequested();
  };
  const auto non_const_non_const_callback_auto = [](auto &iface, auto &token) {
    iface.nonConstMethod();
    token.requestStop();
  };

  // Verify it compiles with non-const
  auto &non_const_impl {m_impl};
  non_const_impl.execute(const_const_callback);
  non_const_impl.execute(const_non_const_callback);
  non_const_impl.execute(non_const_const_callback);
  non_const_impl.execute(non_const_non_const_callback);
  non_const_impl.execute(const_const_callback_auto);
  non_const_impl.execute(const_non_const_callback_auto);
  non_const_impl.execute(non_const_const_callback_auto);
  non_const_impl.execute(non_const_non_const_callback_auto);

  // Verify it compiles with const (commented out code will not compile)
  const auto &const_impl {m_impl};
  const_impl.execute(const_const_callback);
  // const_impl.execute(const_non_const_callback);
  // const_impl.execute(non_const_const_callback);
  // const_impl.execute(non_const_non_const_callback);
  const_impl.execute(const_const_callback_auto);
  // const_impl.execute(const_non_const_callback_auto);
  // const_impl.execute(non_const_const_callback_auto);
  // const_impl.execute(non_const_non_const_callback_auto);
}

TEST_F_S(Stop) {
  EXPECT_FALSE(m_impl.isScheduled());
  m_impl.stop();
  EXPECT_FALSE(m_impl.isScheduled());

  int counter {0};
  m_impl.schedule([&](auto, auto &) {
    counter++;
  },
                  {.m_sleep_durations = {1ms}});
  while (counter < 3) {
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_TRUE(m_impl.isScheduled());
  m_impl.stop();
  EXPECT_FALSE(m_impl.isScheduled());
}

TEST_F_S(ThreadCleanupInDestructor) {
  int counter {0};
  {
    display_device::RetryScheduler<TestIface> scheduler {std::make_unique<TestIface>()};

    scheduler.schedule([&](auto, auto &) {
      counter++;
    },
                       {.m_sleep_durations = {1ms}});
    while (counter < 3) {
      std::this_thread::sleep_for(1ms);
    }
  }

  const int counter_before_sleep {counter};
  std::this_thread::sleep_for(100ms);
  const int counter_after_sleep {counter};

  EXPECT_EQ(counter_before_sleep, counter_after_sleep);
}

TEST_F_S(SchedulerStopToken, DestructorNoThrow) {
  EXPECT_NO_THROW({
    display_device::SchedulerStopToken token {[]() {
    }};
    token.requestStop();
  });
  EXPECT_NO_THROW({
    display_device::SchedulerStopToken token {[]() {
    }};
  });
  EXPECT_NO_THROW({
    display_device::SchedulerStopToken token {{}};
    token.requestStop();
  });
  EXPECT_NO_THROW({
    display_device::SchedulerStopToken token {{}};
  });
}
