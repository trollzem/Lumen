// local includes
#include "display_device/windows/settings_utils.h"
#include "display_device/windows/win_api_layer.h"
#include "display_device/windows/win_api_utils.h"
#include "display_device/windows/win_display_device.h"
#include "fixtures/fixtures.h"
#include "utils/comparison.h"
#include "utils/helpers.h"
#include "utils/mock_win_api_layer.h"

namespace {
  // Convenience keywords for GMock
  using ::testing::_;
  using ::testing::HasSubstr;
  using ::testing::InSequence;
  using ::testing::Return;
  using ::testing::StrictMock;

  // Test fixture(s) for this file
  class WinDisplayDeviceGeneral: public BaseTest {
  public:
    std::shared_ptr<display_device::WinApiLayer> m_layer {std::make_shared<display_device::WinApiLayer>()};
    display_device::WinDisplayDevice m_win_dd {m_layer};
  };

  class WinDisplayDeviceGeneralMocked: public BaseTest {
  public:
    std::shared_ptr<StrictMock<display_device::MockWinApiLayer>> m_layer {std::make_shared<StrictMock<display_device::MockWinApiLayer>>()};
    display_device::WinDisplayDevice m_win_dd {m_layer};
  };

  // Specialized TEST macro(s) for this test file
#define TEST_F_S(...) DD_MAKE_TEST(TEST_F, WinDisplayDeviceGeneral, __VA_ARGS__)
#define TEST_F_S_MOCKED(...) DD_MAKE_TEST(TEST_F, WinDisplayDeviceGeneralMocked, __VA_ARGS__)

  // Additional convenience global const(s)
  const UINT32 FLAGS {SDC_VALIDATE | SDC_USE_DATABASE_CURRENT};
}  // namespace

TEST_F_S(NullptrLayerProvided) {
  EXPECT_THAT([]() {
    const auto win_dd {display_device::WinDisplayDevice {nullptr}};
  },
              ThrowsMessage<std::logic_error>(HasSubstr("Nullptr provided for WinApiLayerInterface in WinDisplayDevice!")));
}

TEST_F_S(IsApiAccessAvailable) {
  // There is no real way to make it return false without logging out or do something complicated :/
  EXPECT_TRUE(m_win_dd.isApiAccessAvailable());
}

TEST_F_S_MOCKED(IsApiAccessAvailable) {
  EXPECT_CALL(*m_layer, setDisplayConfig(std::vector<DISPLAYCONFIG_PATH_INFO> {}, std::vector<DISPLAYCONFIG_MODE_INFO> {}, FLAGS))
    .Times(1)
    .WillOnce(Return(ERROR_SUCCESS));
  EXPECT_CALL(*m_layer, getErrorString(ERROR_SUCCESS))
    .Times(1)
    .WillRepeatedly(Return("ErrorDesc"));

  EXPECT_TRUE(m_win_dd.isApiAccessAvailable());
}

TEST_F_S_MOCKED(IsApiAccessAvailable, FailedToSetDisplayConfig) {
  EXPECT_CALL(*m_layer, setDisplayConfig(std::vector<DISPLAYCONFIG_PATH_INFO> {}, std::vector<DISPLAYCONFIG_MODE_INFO> {}, FLAGS))
    .Times(1)
    .WillOnce(Return(ERROR_ACCESS_DENIED));
  EXPECT_CALL(*m_layer, getErrorString(ERROR_ACCESS_DENIED))
    .Times(1)
    .WillRepeatedly(Return("ErrorDesc"));

  EXPECT_FALSE(m_win_dd.isApiAccessAvailable());
}

TEST_F_S(EnumAvailableDevices) {
  // Note: we can't verify live data precisely, so just basic check
  // is performed regarding active vs. inactive devices

  const auto available_devices {getAvailableDevices(*m_layer, false)};
  ASSERT_TRUE(available_devices);

  const auto enum_devices {m_win_dd.enumAvailableDevices()};
  ASSERT_EQ(available_devices->size(), enum_devices.size());

  const auto topology {display_device::win_utils::flattenTopology(m_win_dd.getCurrentTopology())};
  for (const auto &device_id : *available_devices) {
    auto enum_it {std::find_if(std::begin(enum_devices), std::end(enum_devices), [&device_id](const auto &entry) {
      return entry.m_device_id == device_id;
    })};

    ASSERT_TRUE(enum_it != std::end(enum_devices));
    EXPECT_EQ(enum_it->m_info.has_value(), topology.contains(device_id));
  }
}

TEST_F_S_MOCKED(EnumAvailableDevices) {
  const auto pam_active_and_inactive {[]() {
    auto pam {ut_consts::PAM_3_ACTIVE};
    pam->m_paths.at(0).targetInfo.refreshRate.Denominator = 0;
    pam->m_paths.at(2).flags &= ~DISPLAYCONFIG_PATH_ACTIVE;
    return pam;
  }()};

  InSequence sequence;
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(pam_active_and_inactive))
    .RetiresOnSaturation();

  for (int i = 1; i <= 3; ++i) {
    EXPECT_CALL(*m_layer, getMonitorDevicePath(_))
      .Times(1)
      .WillOnce(Return("Path" + std::to_string(i)))
      .RetiresOnSaturation();
    EXPECT_CALL(*m_layer, getDeviceId(_))
      .Times(1)
      .WillOnce(Return("DeviceId" + std::to_string(i)))
      .RetiresOnSaturation();
    EXPECT_CALL(*m_layer, getDisplayName(_))
      .Times(1)
      .WillOnce(Return("DisplayName" + std::to_string(i)))
      .RetiresOnSaturation();
  }

  EXPECT_CALL(*m_layer, getFriendlyName(_))
    .Times(1)
    .WillOnce(Return("FriendlyName1"))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getDisplayName(_))
    .Times(1)
    .WillOnce(Return("DisplayName1"))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getEdid(_))
    .Times(1)
    .WillOnce(Return(std::vector<std::byte> {}))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getDisplayScale(_, _))
    .Times(1)
    .WillOnce(Return(std::nullopt))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getHdrState(_))
    .Times(1)
    .WillOnce(Return(std::nullopt))
    .RetiresOnSaturation();

  EXPECT_CALL(*m_layer, getFriendlyName(_))
    .Times(1)
    .WillOnce(Return("FriendlyName2"))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getDisplayName(_))
    .Times(1)
    .WillOnce(Return("DisplayName2"))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getEdid(_))
    .Times(1)
    .WillOnce(Return(ut_consts::DEFAULT_EDID))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getDisplayScale(_, _))
    .Times(1)
    .WillOnce(Return(display_device::Rational {175, 100}))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getHdrState(_))
    .Times(1)
    .WillOnce(Return(display_device::HdrState::Enabled))
    .RetiresOnSaturation();

  EXPECT_CALL(*m_layer, getFriendlyName(_))
    .Times(1)
    .WillOnce(Return("FriendlyName3"))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getEdid(_))
    .Times(1)
    .WillOnce(Return(std::vector<std::byte> {}))
    .RetiresOnSaturation();

  const display_device::EnumeratedDeviceList expected_list {
    {"DeviceId1",
     "DisplayName1",
     "FriendlyName1",
     std::nullopt,
     display_device::EnumeratedDevice::Info {
       {1920, 1080},
       display_device::Rational {0, 1},
       display_device::Rational {0, 1},
       true,
       {0, 0},
       std::nullopt
     }},
    {"DeviceId2",
     "DisplayName2",
     "FriendlyName2",
     ut_consts::DEFAULT_EDID_DATA,
     display_device::EnumeratedDevice::Info {
       {1920, 2160},
       display_device::Rational {175, 100},
       display_device::Rational {119995, 1000},
       false,
       {1921, 0},
       display_device::HdrState::Enabled
     }},
    {"DeviceId3",
     "",
     "FriendlyName3",
     std::nullopt,
     std::nullopt}
  };
  EXPECT_EQ(m_win_dd.enumAvailableDevices(), expected_list);
}

TEST_F_S_MOCKED(EnumAvailableDevices, MissingSourceModes) {
  auto pam_missing_modes {ut_consts::PAM_3_ACTIVE};
  pam_missing_modes->m_paths.resize(2);
  pam_missing_modes->m_modes.resize(1);

  InSequence sequence;
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(pam_missing_modes))
    .RetiresOnSaturation();

  for (int i = 1; i <= 2; ++i) {
    EXPECT_CALL(*m_layer, getMonitorDevicePath(_))
      .Times(1)
      .WillOnce(Return("Path" + std::to_string(i)))
      .RetiresOnSaturation();
    EXPECT_CALL(*m_layer, getDeviceId(_))
      .Times(1)
      .WillOnce(Return("DeviceId" + std::to_string(i)))
      .RetiresOnSaturation();
    EXPECT_CALL(*m_layer, getDisplayName(_))
      .Times(1)
      .WillOnce(Return("DisplayName" + std::to_string(i)))
      .RetiresOnSaturation();
  }

  EXPECT_CALL(*m_layer, getFriendlyName(_))
    .Times(1)
    .WillOnce(Return("FriendlyName1"))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getDisplayName(_))
    .Times(1)
    .WillOnce(Return("DisplayName1"))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getEdid(_))
    .Times(1)
    .WillOnce(Return(std::vector<std::byte> {}))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getDisplayScale(_, _))
    .Times(1)
    .WillOnce(Return(std::nullopt))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getHdrState(_))
    .Times(1)
    .WillOnce(Return(std::nullopt))
    .RetiresOnSaturation();

  EXPECT_CALL(*m_layer, getFriendlyName(_))
    .Times(1)
    .WillOnce(Return("FriendlyName2"))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getDisplayName(_))
    .Times(1)
    .WillOnce(Return("DisplayName2"))
    .RetiresOnSaturation();
  EXPECT_CALL(*m_layer, getEdid(_))
    .Times(1)
    .WillOnce(Return(ut_consts::DEFAULT_EDID))
    .RetiresOnSaturation();

  const display_device::EnumeratedDeviceList expected_list {
    {"DeviceId1",
     "DisplayName1",
     "FriendlyName1",
     std::nullopt,
     display_device::EnumeratedDevice::Info {
       {1920, 1080},
       display_device::Rational {0, 1},
       display_device::Rational {120, 1},
       true,
       {0, 0},
       std::nullopt
     }},
    {"DeviceId2",
     "DisplayName2",
     "FriendlyName2",
     ut_consts::DEFAULT_EDID_DATA,
     std::nullopt}
  };
  EXPECT_EQ(m_win_dd.enumAvailableDevices(), expected_list);
}

TEST_F_S_MOCKED(EnumAvailableDevices, FailedToGetDisplayData) {
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_NULL));

  EXPECT_EQ(m_win_dd.enumAvailableDevices(), display_device::EnumeratedDeviceList {});
}

TEST_F_S_MOCKED(EnumAvailableDevices, FailedToCollectSourceData) {
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::All))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_EMPTY));

  EXPECT_EQ(m_win_dd.enumAvailableDevices(), display_device::EnumeratedDeviceList {});
}

TEST_F_S(GetDisplayName) {
  const auto all_devices {m_layer->queryDisplayConfig(display_device::QueryType::All)};
  ASSERT_TRUE(all_devices);

  for (const auto &path : all_devices->m_paths) {
    const auto device_id {m_layer->getDeviceId(path)};
    const auto display_name {m_win_dd.getDisplayName(device_id)};
    const auto active_path {display_device::win_utils::getActivePath(*m_layer, device_id, all_devices->m_paths)};

    if (&path == active_path) {
      EXPECT_EQ(display_name, m_layer->getDisplayName(path));
    } else {
      EXPECT_EQ(display_name, active_path ? m_layer->getDisplayName(*active_path) : std::string {});
    }
  }
}

TEST_F_S_MOCKED(GetDisplayName) {
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_3_ACTIVE));
  EXPECT_CALL(*m_layer, getMonitorDevicePath(_))
    .Times(1)
    .WillOnce(Return("PathX"));
  EXPECT_CALL(*m_layer, getDeviceId(_))
    .Times(1)
    .WillOnce(Return("DeviceId1"));
  EXPECT_CALL(*m_layer, getDisplayName(_))
    .Times(2)
    .WillOnce(Return("DisplayName1"))
    .WillOnce(Return("DisplayName1"));

  EXPECT_EQ(m_win_dd.getDisplayName("DeviceId1"), "DisplayName1");
}

TEST_F_S_MOCKED(GetDisplayName, EmptyDeviceId) {
  EXPECT_EQ(m_win_dd.getDisplayName(""), std::string {});
}

TEST_F_S_MOCKED(GetDisplayName, FailedToGetDisplayData) {
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_NULL));

  EXPECT_EQ(m_win_dd.getDisplayName("DeviceId1"), std::string {});
}

TEST_F_S_MOCKED(GetDisplayName, FailedToGetActivePath) {
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_EMPTY));

  EXPECT_EQ(m_win_dd.getDisplayName("DeviceId1"), std::string {});
}

TEST_F_S_MOCKED(GetDisplayName, EmptyDisplayName) {
  EXPECT_CALL(*m_layer, queryDisplayConfig(display_device::QueryType::Active))
    .Times(1)
    .WillOnce(Return(ut_consts::PAM_3_ACTIVE));
  EXPECT_CALL(*m_layer, getMonitorDevicePath(_))
    .Times(1)
    .WillOnce(Return("PathX"));
  EXPECT_CALL(*m_layer, getDeviceId(_))
    .Times(1)
    .WillOnce(Return("DeviceId1"));
  EXPECT_CALL(*m_layer, getDisplayName(_))
    .Times(2)
    .WillOnce(Return("DisplayName1"))
    .WillOnce(Return(""));

  EXPECT_EQ(m_win_dd.getDisplayName("DeviceId1"), std::string {});
}
