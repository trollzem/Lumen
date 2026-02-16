// test includes
#include "tests/conftest.cpp"

#if defined(_WIN32) || defined(_WIN64)
  #define TRAY_WINAPI 1
#elif defined(__linux__) || defined(linux) || defined(__linux)
  #define TRAY_APPINDICATOR 1
#elif defined(__APPLE__) || defined(__MACH__)
  #define TRAY_APPKIT 1
#endif

// local includes
#include "src/tray.h"

#if TRAY_APPINDICATOR
  #define TRAY_ICON1 "mail-message-new"
  #define TRAY_ICON2 "mail-message-new"
#elif TRAY_APPKIT
  #define TRAY_ICON1 "icon.png"
  #define TRAY_ICON2 "icon.png"
#elif TRAY_WINAPI
  #define TRAY_ICON1 "icon.ico"
  #define TRAY_ICON2 "icon.ico"
#endif

class TrayTest: public BaseTest {
protected:
  static struct tray testTray;

  // Static arrays for submenus
  static struct tray_menu submenu7_8[];
  static struct tray_menu submenu5_6[];
  static struct tray_menu submenu_second[];
  static struct tray_menu submenu[];

  // Non-static member functions
  static void hello_cb(struct tray_menu *item) {
    // Mock implementation
  }

  static void toggle_cb(struct tray_menu *item) {
    item->checked = !item->checked;
    tray_update(&testTray);
  }

  static void quit_cb(struct tray_menu *item) {
    tray_exit();
  }

  static void submenu_cb(struct tray_menu *item) {
    // Mock implementation
    tray_update(&testTray);
  }

  void SetUp() override {
    testTray.icon = TRAY_ICON1;
    testTray.tooltip = "TestTray";
    testTray.menu = submenu;
  }

  void TearDown() override {
    // Clean up any resources if needed
  }
};

// Define the static arrays
struct tray_menu TrayTest::submenu7_8[] = {
  {.text = "7", .cb = submenu_cb},
  {.text = "-"},
  {.text = "8", .cb = submenu_cb},
  {.text = nullptr}
};
struct tray_menu TrayTest::submenu5_6[] = {
  {.text = "5", .cb = submenu_cb},
  {.text = "6", .cb = submenu_cb},
  {.text = nullptr}
};
struct tray_menu TrayTest::submenu_second[] = {
  {.text = "THIRD", .submenu = submenu7_8},
  {.text = "FOUR", .submenu = submenu5_6},
  {.text = nullptr}
};
struct tray_menu TrayTest::submenu[] = {
  {.text = "Hello", .cb = hello_cb},
  {.text = "Checked", .checked = 1, .checkbox = 1, .cb = toggle_cb},
  {.text = "Disabled", .disabled = 1},
  {.text = "-"},
  {.text = "SubMenu", .submenu = submenu_second},
  {.text = "-"},
  {.text = "Quit", .cb = quit_cb},
  {.text = nullptr}
};
struct tray TrayTest::testTray = {
  .icon = TRAY_ICON1,
  .tooltip = "TestTray",
  .menu = submenu
};

TEST_F(TrayTest, TestTrayInit) {
  int result = tray_init(&testTray);
  EXPECT_EQ(result, 0);  // make sure return value is 0
}

TEST_F(TrayTest, TestTrayLoop) {
  int result = tray_loop(1);
  EXPECT_EQ(result, 0);  // make sure return value is 0
}

TEST_F(TrayTest, TestTrayUpdate) {
  // check the initial values
  EXPECT_EQ(testTray.icon, TRAY_ICON1);
  EXPECT_EQ(testTray.tooltip, "TestTray");

  // update the values
  testTray.icon = TRAY_ICON2;
  testTray.tooltip = "TestTray2";
  tray_update(&testTray);
  EXPECT_EQ(testTray.icon, TRAY_ICON2);
  EXPECT_EQ(testTray.tooltip, "TestTray2");

  // put back the original values
  testTray.icon = TRAY_ICON1;
  testTray.tooltip = "TestTray";
  tray_update(&testTray);
  EXPECT_EQ(testTray.icon, TRAY_ICON1);
  EXPECT_EQ(testTray.tooltip, "TestTray");
}

TEST_F(TrayTest, TestToggleCallback) {
  bool initialCheckedState = testTray.menu[1].checked;
  toggle_cb(&testTray.menu[1]);
  EXPECT_EQ(testTray.menu[1].checked, !initialCheckedState);
}

TEST_F(TrayTest, TestTrayExit) {
  tray_exit();
  // TODO: Check the state after tray_exit
}
