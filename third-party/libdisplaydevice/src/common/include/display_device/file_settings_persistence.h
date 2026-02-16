/**
 * @file src/common/include/display_device/file_settings_persistence.h
 * @brief Declarations for persistent file settings.
 */
#pragma once

// system includes
#include <filesystem>

// local includes
#include "settings_persistence_interface.h"

namespace display_device {
  /**
   * @brief Implementation of the SettingsPersistenceInterface,
   *        that saves/loads the persistent settings to/from the file.
   */
  class FileSettingsPersistence: public SettingsPersistenceInterface {
  public:
    /**
     * Default constructor. Does not perform any operations on the file yet.
     * @param filepath A non-empty filepath. Throws on empty.
     */
    explicit FileSettingsPersistence(std::filesystem::path filepath);

    /**
     * Store the data in the file specified in constructor.
     * @warning The method does not create missing directories!
     * @see SettingsPersistenceInterface::store for more details.
     */
    [[nodiscard]] bool store(const std::vector<std::uint8_t> &data) override;

    /**
     * Read the data from the file specified in constructor.
     * @note If file does not exist, an empty data list will be returned instead of null optional.
     * @see SettingsPersistenceInterface::load for more details.
     */
    [[nodiscard]] std::optional<std::vector<std::uint8_t>> load() const override;

    /**
     * Remove the file specified in constructor (if it exists).
     * @see SettingsPersistenceInterface::clear for more details.
     */
    [[nodiscard]] bool clear() override;

  private:
    std::filesystem::path m_filepath;
  };
}  // namespace display_device
