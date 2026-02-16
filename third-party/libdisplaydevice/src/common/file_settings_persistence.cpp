/**
 * @file src/common/file_settings_persistence.cpp
 * @brief Definitions for persistent file settings.
 */
// class header include
#include "display_device/file_settings_persistence.h"

// system includes
#include <algorithm>
#include <fstream>
#include <iterator>

// local includes
#include "display_device/logging.h"

namespace display_device {
  FileSettingsPersistence::FileSettingsPersistence(std::filesystem::path filepath):
      m_filepath {std::move(filepath)} {
    if (m_filepath.empty()) {
      throw std::runtime_error {"Empty filename provided for FileSettingsPersistence!"};
    }
  }

  bool FileSettingsPersistence::store(const std::vector<std::uint8_t> &data) {
    try {
      std::ofstream stream {m_filepath, std::ios::binary | std::ios::trunc};
      if (!stream) {
        DD_LOG(error) << "Failed to open " << m_filepath << " for writing!";
        return false;
      }

      std::copy(std::begin(data), std::end(data), std::ostreambuf_iterator<char> {stream});
      return true;
    } catch (const std::exception &error) {
      DD_LOG(error) << "Failed to write to " << m_filepath << "! Error:\n"
                    << error.what();
      return false;
    }
  }

  std::optional<std::vector<std::uint8_t>> FileSettingsPersistence::load() const {
    if (std::error_code error_code; !std::filesystem::exists(m_filepath, error_code)) {
      if (error_code) {
        DD_LOG(error) << "Failed to load " << m_filepath << "! Error:\n"
                      << "[" << error_code.value() << "] " << error_code.message();
        return std::nullopt;
      }

      return std::vector<std::uint8_t> {};
    }

    try {
      std::ifstream stream {m_filepath, std::ios::binary};
      if (!stream) {
        DD_LOG(error) << "Failed to open " << m_filepath << " for reading!";
        return std::nullopt;
      }

      return std::vector<std::uint8_t> {std::istreambuf_iterator<char> {stream}, std::istreambuf_iterator<char> {}};
    } catch (const std::exception &error) {
      DD_LOG(error) << "Failed to read " << m_filepath << "! Error:\n"
                    << error.what();
      return std::nullopt;
    }
  }

  bool FileSettingsPersistence::clear() {
    // Return valud does not matter since we check the error code in case the file could NOT be removed.
    std::error_code error_code;
    std::filesystem::remove(m_filepath, error_code);

    if (error_code) {
      DD_LOG(error) << "Failed to remove " << m_filepath << "! Error:\n"
                    << "[" << error_code.value() << "] " << error_code.message();
      return false;
    }

    return true;
  }
}  // namespace display_device
