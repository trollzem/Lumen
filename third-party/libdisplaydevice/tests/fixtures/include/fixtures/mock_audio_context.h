#pragma once

// system includes
#include <gmock/gmock.h>

// local includes
#include "display_device/audio_context_interface.h"

namespace display_device {
  class MockAudioContext: public AudioContextInterface {
  public:
    MOCK_METHOD(bool, capture, (), (override));
    MOCK_METHOD(bool, isCaptured, (), (const, override));
    MOCK_METHOD(void, release, (), (override));
  };
}  // namespace display_device
