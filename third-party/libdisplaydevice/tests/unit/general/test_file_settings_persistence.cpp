// system includes
#include <fstream>
#include <gmock/gmock.h>

// local includes
#include "display_device/file_settings_persistence.h"
#include "fixtures/fixtures.h"

namespace {
  // Convenience keywords for GMock
  using ::testing::HasSubstr;

  // Test fixture(s) for this file
  class FileSettingsPersistenceTest: public BaseTest {
  public:
    ~FileSettingsPersistenceTest() override {
      std::filesystem::remove(m_filepath);
    }

    display_device::FileSettingsPersistence &getImpl(const std::filesystem::path &filepath = "testfile.ext") {
      if (!m_impl) {
        m_filepath = filepath;
        m_impl = std::make_unique<display_device::FileSettingsPersistence>(m_filepath);
      }

      return *m_impl;
    }

  private:
    std::filesystem::path m_filepath;
    std::unique_ptr<display_device::FileSettingsPersistence> m_impl;
  };

  // Specialized TEST macro(s) for this test file
#define TEST_F_S(...) DD_MAKE_TEST(TEST_F, FileSettingsPersistenceTest, __VA_ARGS__)
}  // namespace

TEST_F_S(EmptyFilenameProvided) {
  EXPECT_THAT([]() {
    const display_device::FileSettingsPersistence persistence {{}};
  },
              ThrowsMessage<std::runtime_error>(HasSubstr("Empty filename provided for FileSettingsPersistence!")));
}

TEST_F_S(Store, NewFileCreated) {
  const std::filesystem::path filepath {"myfile.ext"};
  const std::vector<std::uint8_t> data {0x00, 0x01, 0x02, 0x04, 'S', 'O', 'M', 'E', ' ', 'D', 'A', 'T', 'A'};

  EXPECT_FALSE(std::filesystem::exists(filepath));
  EXPECT_TRUE(getImpl(filepath).store(data));
  EXPECT_TRUE(std::filesystem::exists(filepath));

  std::ifstream stream {filepath, std::ios::binary};
  std::vector<std::uint8_t> file_data {std::istreambuf_iterator<char> {stream}, std::istreambuf_iterator<char> {}};
  EXPECT_EQ(file_data, data);
}

TEST_F_S(Store, FileOverwritten) {
  const std::filesystem::path filepath {"myfile.ext"};
  const std::vector<std::uint8_t> data1 {0x00, 0x01, 0x02, 0x04, 'S', 'O', 'M', 'E', ' ', 'D', 'A', 'T', 'A', ' ', '1'};
  const std::vector<std::uint8_t> data2 {0x00, 0x01, 0x02, 0x04, 'S', 'O', 'M', 'E', ' ', 'D', 'A', 'T', 'A', ' ', '2'};

  {
    std::ofstream file {filepath, std::ios_base::binary};
    std::copy(std::begin(data1), std::end(data1), std::ostreambuf_iterator<char> {file});
  }

  EXPECT_TRUE(std::filesystem::exists(filepath));
  EXPECT_TRUE(getImpl(filepath).store(data2));
  EXPECT_TRUE(std::filesystem::exists(filepath));

  std::ifstream stream {filepath, std::ios::binary};
  std::vector<std::uint8_t> file_data {std::istreambuf_iterator<char> {stream}, std::istreambuf_iterator<char> {}};
  EXPECT_EQ(file_data, data2);
}

TEST_F_S(Store, FilepathWithDirectory) {
  const std::filesystem::path filepath {"somedir/myfile.ext"};
  const std::vector<std::uint8_t> data {0x00, 0x01, 0x02, 0x04, 'S', 'O', 'M', 'E', ' ', 'D', 'A', 'T', 'A'};

  EXPECT_FALSE(std::filesystem::exists(filepath));
  EXPECT_FALSE(getImpl(filepath).store(data));
  EXPECT_FALSE(std::filesystem::exists(filepath));
}

TEST_F_S(Load, NoFileAvailable) {
  EXPECT_EQ(getImpl().load(), std::vector<std::uint8_t> {});
}

TEST_F_S(Load, FileRead) {
  const std::filesystem::path filepath {"myfile.ext"};
  const std::vector<std::uint8_t> data {0x00, 0x01, 0x02, 0x04, 'S', 'O', 'M', 'E', ' ', 'D', 'A', 'T', 'A'};

  {
    std::ofstream file {filepath, std::ios_base::binary};
    std::copy(std::begin(data), std::end(data), std::ostreambuf_iterator<char> {file});
  }

  EXPECT_EQ(getImpl(filepath).load(), data);
}

TEST_F_S(Clear, NoFileAvailable) {
  EXPECT_TRUE(getImpl().clear());
}

TEST_F_S(Clear, FileRemoved) {
  const std::filesystem::path filepath {"myfile.ext"};
  {
    std::ofstream file {filepath};
    file << "some data";
  }

  EXPECT_TRUE(std::filesystem::exists(filepath));
  EXPECT_TRUE(getImpl(filepath).clear());
  EXPECT_FALSE(std::filesystem::exists(filepath));
}
