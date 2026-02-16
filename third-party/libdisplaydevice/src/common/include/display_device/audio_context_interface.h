/**
 * @file src/common/include/display_device/audio_context_interface.h
 * @brief Declarations for the AudioContextInterface.
 */
#pragma once

namespace display_device {
  /**
   * @brief A class for capturing associated audio context (settings, info or whatever).
   *
   * Some of the display devices have audio devices associated with them.
   * Turning off and on the devices will not necessarily restore them as the default
   * audio devices for the system.
   */
  class AudioContextInterface {
  public:
    /**
     * @brief Default virtual destructor.
     */
    virtual ~AudioContextInterface() = default;

    /**
     * @brief Capture audio context for currently active devices.
     * @returns True if the contexts could be captured, false otherwise.
     * @examples
     * AudioContextInterface* iface = getIface(...);
     * const auto result { iface->capture() };
     * @examples_end
     */
    [[nodiscard]] virtual bool capture() = 0;

    /**
     * @brief Check if the context is already captured.
     * @returns True if the context is captured, false otherwise.
     * @examples
     * AudioContextInterface* iface = getIface(...);
     * const auto result { iface->isCaptured() };
     * @examples_end
     */
    [[nodiscard]] virtual bool isCaptured() const = 0;

    /**
     * @brief Release captured audio context for the devices (if any).
     * @examples
     * AudioContextInterface* iface = getIface(...);
     * const auto result { iface->release() };
     * @examples_end
     */
    virtual void release() = 0;
  };
}  // namespace display_device
