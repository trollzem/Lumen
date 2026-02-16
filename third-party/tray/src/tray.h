/**
 * @file src/tray.h
 * @brief Definition of the tray API.
 */
#ifndef TRAY_H
#define TRAY_H

#ifdef __cplusplus
extern "C" {
#endif

  /**
   * @brief Tray menu item.
   */
  struct tray_menu;

  /**
   * @brief Tray icon.
   */
  struct tray {
    const char *icon;  ///< Icon to display.
    const char *tooltip;  ///< Tooltip to display.
    const char *notification_icon;  ///< Icon to display in the notification.
    const char *notification_text;  ///< Text to display in the notification.
    const char *notification_title;  ///< Title to display in the notification.
    void (*notification_cb)();  ///< Callback to invoke when the notification is clicked.
    struct tray_menu *menu;  ///< Menu items.
    const int iconPathCount;  ///< Number of icon paths.
    const char *allIconPaths[];  ///< Array of icon paths.
  };

  /**
   * @brief Tray menu item.
   */
  struct tray_menu {
    const char *text;  ///< Text to display.
    int disabled;  ///< Whether the item is disabled.
    int checked;  ///< Whether the item is checked.
    int checkbox;  ///< Whether the item is a checkbox.

    void (*cb)(struct tray_menu *);  ///< Callback to invoke when the item is clicked.
    void *context;  ///< Context to pass to the callback.

    struct tray_menu *submenu;  ///< Submenu items.
  };

  /**
   * @brief Create tray icon.
   * @param tray The tray to initialize.
   * @return 0 on success, -1 on error.
   */
  int tray_init(struct tray *tray);

  /**
   * @brief Run one iteration of the UI loop.
   * @param blocking Whether to block the call or not.
   * @return 0 on success, -1 if tray_exit() was called.
   */
  int tray_loop(int blocking);

  /**
   * @brief Update the tray icon and menu.
   * @param tray The tray to update.
   */
  void tray_update(struct tray *tray);

  /**
   * @brief Terminate UI loop.
   */
  void tray_exit(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* TRAY_H */
