/**
 * @file src/platform/macos/microphone.mm
 * @brief Definitions for microphone/audio capture on macOS.
 * @details Supports:
 *   - System audio capture via ScreenCaptureKit (macOS 12.3+, preferred)
 *   - BlackHole virtual audio device (fallback for system audio)
 *   - Physical microphone capture via AVFoundation
 */
// local includes
#include "src/config.h"
#include "src/logging.h"
#include "src/platform/common.h"
#include "src/platform/macos/av_audio.h"
#include "src/platform/macos/sc_audio.h"

namespace platf {
  using namespace std::literals;

  /**
   * @brief Microphone capture using AVFoundation (for physical mics and BlackHole)
   */
  struct av_mic_t: public mic_t {
    AVAudio *av_audio_capture {};

    ~av_mic_t() override {
      [av_audio_capture release];
    }

    capture_e sample(std::vector<float> &sample_in) override {
      auto sample_size = sample_in.size();

      uint32_t length = 0;
      void *byteSampleBuffer = TPCircularBufferTail(&av_audio_capture->audioSampleBuffer, &length);

      int wait_count = 0;
      while (length < sample_size * sizeof(float)) {
        [av_audio_capture.samplesArrivedSignal lock];
        [av_audio_capture.samplesArrivedSignal waitUntilDate:[NSDate dateWithTimeIntervalSinceNow:0.1]];
        [av_audio_capture.samplesArrivedSignal unlock];
        byteSampleBuffer = TPCircularBufferTail(&av_audio_capture->audioSampleBuffer, &length);
        // After 500ms of no data, return timeout so the caller can check shutdown_event
        if (++wait_count > 5 && length < sample_size * sizeof(float)) {
          return capture_e::timeout;
        }
      }

      const float *sampleBuffer = (float *) byteSampleBuffer;
      std::vector<float> vectorBuffer(sampleBuffer, sampleBuffer + sample_size);

      std::copy_n(std::begin(vectorBuffer), sample_size, std::begin(sample_in));

      TPCircularBufferConsume(&av_audio_capture->audioSampleBuffer, (uint32_t) sample_size * sizeof(float));

      return capture_e::ok;
    }
  };

  /**
   * @brief System audio capture using ScreenCaptureKit (macOS 12.3+)
   */
  struct sc_mic_t: public mic_t {
    SCAudioCapture *sc_audio_capture API_AVAILABLE(macos(12.3)) {};

    ~sc_mic_t() override {
      if (@available(macOS 12.3, *)) {
        [sc_audio_capture release];
      }
    }

    capture_e sample(std::vector<float> &sample_in) override {
      if (@available(macOS 12.3, *)) {
        auto sample_size = sample_in.size();

        uint32_t length = 0;
        TPCircularBuffer *buffer = [sc_audio_capture getAudioBuffer];
        void *byteSampleBuffer = TPCircularBufferTail(buffer, &length);

        int wait_count = 0;
        while (length < sample_size * sizeof(float)) {
          if (!sc_audio_capture.isCapturing) {
            return capture_e::error;
          }
          [sc_audio_capture.samplesArrivedSignal lock];
          [sc_audio_capture.samplesArrivedSignal waitUntilDate:[NSDate dateWithTimeIntervalSinceNow:0.1]];
          [sc_audio_capture.samplesArrivedSignal unlock];
          byteSampleBuffer = TPCircularBufferTail(buffer, &length);
          // After 500ms of no data, return timeout so the caller can check shutdown_event
          if (++wait_count > 5 && length < sample_size * sizeof(float)) {
            return capture_e::timeout;
          }
        }

        const float *sampleBuffer = (float *) byteSampleBuffer;
        std::vector<float> vectorBuffer(sampleBuffer, sampleBuffer + sample_size);

        std::copy_n(std::begin(vectorBuffer), sample_size, std::begin(sample_in));

        TPCircularBufferConsume(buffer, (uint32_t) sample_size * sizeof(float));

        return capture_e::ok;
      }
      return capture_e::error;
    }
  };

  struct macos_audio_control_t: public audio_control_t {
    AVCaptureDevice *audio_capture_device {};

  public:
    int set_sink(const std::string &sink) override {
      BOOST_LOG(warning) << "audio_control_t::set_sink() unimplemented: "sv << sink;
      return 0;
    }

    std::unique_ptr<mic_t> microphone(const std::uint8_t *mapping, int channels, std::uint32_t sample_rate, std::uint32_t frame_size, bool continuous_audio) override {
      const char *audio_sink = "";

      if (!config::audio.sink.empty()) {
        audio_sink = config::audio.sink.c_str();
      }

      // Check if user wants system audio or has no sink specified
      bool want_system_audio = config::audio.sink.empty() ||
                               strcasecmp(audio_sink, "system") == 0 ||
                               strcasecmp(audio_sink, "desktop") == 0 ||
                               strcasecmp(audio_sink, "screencapturekit") == 0;

      // Try ScreenCaptureKit for system audio (macOS 12.3+)
      if (want_system_audio) {
        if (@available(macOS 12.3, *)) {
          if ([SCAudioCapture isAvailable]) {
            BOOST_LOG(info) << "Attempting native system audio capture via ScreenCaptureKit..."sv;

            auto sc_mic = std::make_unique<sc_mic_t>();
            sc_mic->sc_audio_capture = [[SCAudioCapture alloc] init];

            if ([sc_mic->sc_audio_capture startCaptureWithSampleRate:sample_rate channels:channels] == 0) {
              BOOST_LOG(info) << "System audio capture enabled via ScreenCaptureKit!"sv;
              return sc_mic;
            } else {
              BOOST_LOG(warning) << "ScreenCaptureKit audio capture failed, falling back to other methods"sv;
              [sc_mic->sc_audio_capture release];
            }
          }
        }
      }

      // Fall back to AVFoundation-based capture (microphones, BlackHole, etc.)
      auto mic = std::make_unique<av_mic_t>();

      // Try to find the specified audio source
      audio_capture_device = [AVAudio findMicrophone:[NSString stringWithUTF8String:audio_sink]];

      // If no specific sink configured or the configured one isn't found, try to auto-detect BlackHole
      if (audio_capture_device == nullptr && (config::audio.sink.empty() || strlen(audio_sink) == 0 || want_system_audio)) {
        NSArray<NSString *> *availableInputs = [AVAudio microphoneNames];

        // Look for BlackHole as it enables system audio capture
        for (NSString *name in availableInputs) {
          if ([name containsString:@"BlackHole"]) {
            BOOST_LOG(info) << "Auto-detected BlackHole audio device for system audio capture: "sv << [name UTF8String];
            audio_capture_device = [AVAudio findMicrophone:name];
            break;
          }
        }
      }

      if (audio_capture_device == nullptr) {
        BOOST_LOG(error) << "opening microphone '"sv << audio_sink << "' failed. Please set a valid input source in the Sunshine config."sv;
        BOOST_LOG(error) << "Available inputs:"sv;

        for (NSString *name in [AVAudio microphoneNames]) {
          BOOST_LOG(error) << "\t"sv << [name UTF8String];
        }

        // Check macOS version and provide appropriate guidance
        if (@available(macOS 12.3, *)) {
          BOOST_LOG(info) << "Tip: For system audio capture, ensure Screen Recording permission is granted in System Settings."sv;
          BOOST_LOG(info) << "     Set audio_sink = system in your config to use ScreenCaptureKit."sv;
        } else {
          BOOST_LOG(info) << "Tip: To capture system audio on older macOS, install BlackHole (brew install blackhole-2ch)"sv;
          BOOST_LOG(info) << "     Then set it as the output device in System Settings, or use a Multi-Output Device."sv;
        }

        return nullptr;
      }

      mic->av_audio_capture = [[AVAudio alloc] init];

      if ([mic->av_audio_capture setupMicrophone:audio_capture_device sampleRate:sample_rate frameSize:frame_size channels:channels]) {
        BOOST_LOG(error) << "Failed to setup microphone."sv;
        return nullptr;
      }

      return mic;
    }

    bool is_sink_available(const std::string &sink) override {
      // Check for ScreenCaptureKit availability
      if (sink.empty() || sink == "system" || sink == "desktop" || sink == "screencapturekit") {
        if (@available(macOS 12.3, *)) {
          return [SCAudioCapture isAvailable];
        }
      }

      // Check for physical input devices
      return [AVAudio findMicrophone:[NSString stringWithUTF8String:sink.c_str()]] != nil;
    }

    std::optional<sink_t> sink_info() override {
      sink_t sink;

      // Report available audio sources
      if (@available(macOS 12.3, *)) {
        if ([SCAudioCapture isAvailable]) {
          sink.host = "System Audio (ScreenCaptureKit)";
        }
      }

      return sink;
    }
  };

  std::unique_ptr<audio_control_t> audio_control() {
    return std::make_unique<macos_audio_control_t>();
  }
}  // namespace platf
