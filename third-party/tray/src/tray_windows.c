/**
 * @file src/tray_windows.c
 * @brief System tray implementation for Windows.
 */
// standard includes
#include <windows.h>
// clang-format off
// build fails if shellapi.h is included before windows.h
#include <shellapi.h>
// clang-format on

// local includes
#include "tray.h"

#define WM_TRAY_CALLBACK_MESSAGE (WM_USER + 1)  ///< Tray callback message.
#define WC_TRAY_CLASS_NAME "TRAY"  ///< Tray window class name.
#define ID_TRAY_FIRST 1000  ///< First tray identifier.

/**
 * @brief Icon information.
 */
struct icon_info {
  const char *path;  ///< Path to the icon
  HICON icon;  ///< Regular icon
  HICON large_icon;  ///< Large icon
  HICON notification_icon;  ///< Notification icon
};

/**
 * @brief Icon type.
 */
enum IconType {
  REGULAR = 1,  ///< Regular icon
  LARGE,  ///< Large icon
  NOTIFICATION  ///< Notification icon
};

static WNDCLASSEX wc;
static NOTIFYICONDATAW nid;
static HWND hwnd;
static HMENU hmenu = NULL;
static void (*notification_cb)() = 0;
static UINT wm_taskbarcreated;

static struct icon_info *icon_infos;
static int icon_info_count;

static LRESULT CALLBACK _tray_wnd_proc(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam) {
  switch (msg) {
    case WM_CLOSE:
      DestroyWindow(hwnd);
      return 0;
    case WM_DESTROY:
      PostQuitMessage(0);
      return 0;
    case WM_TRAY_CALLBACK_MESSAGE:
      if (lparam == WM_LBUTTONUP || lparam == WM_RBUTTONUP) {
        POINT p;
        GetCursorPos(&p);
        SetForegroundWindow(hwnd);
        WORD cmd = TrackPopupMenu(hmenu, TPM_LEFTALIGN | TPM_RIGHTBUTTON | TPM_RETURNCMD | TPM_NONOTIFY, p.x, p.y, 0, hwnd, NULL);
        SendMessage(hwnd, WM_COMMAND, cmd, 0);
        return 0;
      } else if (lparam == NIN_BALLOONUSERCLICK && notification_cb != NULL) {
        notification_cb();
      }
      break;
    case WM_COMMAND:
      if (wparam >= ID_TRAY_FIRST) {
        MENUITEMINFO item = {
          .cbSize = sizeof(MENUITEMINFO),
          .fMask = MIIM_ID | MIIM_DATA,
        };
        if (GetMenuItemInfo(hmenu, wparam, FALSE, &item)) {
          struct tray_menu *menu = (struct tray_menu *) item.dwItemData;
          if (menu != NULL && menu->cb != NULL) {
            menu->cb(menu);
          }
        }
        return 0;
      }
      break;
  }

  if (msg == wm_taskbarcreated) {
    Shell_NotifyIconW(NIM_ADD, &nid);
    return 0;
  }

  return DefWindowProc(hwnd, msg, wparam, lparam);
}

static HMENU _tray_menu(struct tray_menu *m, UINT *id) {
  HMENU hmenu = CreatePopupMenu();
  for (; m != NULL && m->text != NULL; m++, (*id)++) {
    if (strcmp(m->text, "-") == 0) {
      InsertMenuW(hmenu, *id, MF_SEPARATOR, TRUE, L"");
    } else {
      MENUITEMINFOW item;
      memset(&item, 0, sizeof(item));
      item.cbSize = sizeof(MENUITEMINFOW);
      item.fMask = MIIM_ID | MIIM_TYPE | MIIM_STATE | MIIM_DATA;
      item.fType = 0;
      item.fState = 0;
      if (m->submenu != NULL) {
        item.fMask = item.fMask | MIIM_SUBMENU;
        item.hSubMenu = _tray_menu(m->submenu, id);
      }
      if (m->disabled) {
        item.fState |= MFS_DISABLED;
      }
      if (m->checked) {
        item.fState |= MFS_CHECKED;
      }
      item.wID = *id;

      // Convert UTF-8 text to UTF-16 (wide string)
      int wide_size = MultiByteToWideChar(CP_UTF8, 0, m->text, -1, NULL, 0);
      wchar_t *wide_text = (wchar_t *) malloc(wide_size * sizeof(wchar_t));
      if (wide_text == NULL) {
        DestroyMenu(hmenu);
        return NULL;
      }
      MultiByteToWideChar(CP_UTF8, 0, m->text, -1, wide_text, wide_size);

      item.dwTypeData = wide_text;
      item.dwItemData = (ULONG_PTR) m;

      InsertMenuItemW(hmenu, *id, TRUE, &item);
      // Free the allocated wide string
      free(wide_text);
    }
  }
  return hmenu;
}

/**
 * @brief Create icon information.
 * @param path Path to the icon.
 * @return Icon information.
 */
struct icon_info _create_icon_info(const char *path) {
  struct icon_info info;
  info.path = strdup(path);

  // These must be separate invocations otherwise Windows may opt to only return large or small icons.
  // MSDN does not explicitly state this anywhere, but it has been observed on some machines.
  ExtractIconEx(path, 0, &info.large_icon, NULL, 1);
  ExtractIconEx(path, 0, NULL, &info.icon, 1);

  info.notification_icon = LoadImageA(NULL, path, IMAGE_ICON, GetSystemMetrics(SM_CXICON) * 2, GetSystemMetrics(SM_CYICON) * 2, LR_LOADFROMFILE);
  return info;
}

/**
 * @brief Initialize icon cache.
 * @param paths Paths to the icons.
 * @param count Number of paths.
 */
void _init_icon_cache(const char **paths, int count) {
  icon_info_count = count;
  icon_infos = malloc(sizeof(struct icon_info) * icon_info_count);

  for (int i = 0; i < count; ++i) {
    icon_infos[i] = _create_icon_info(paths[i]);
  }
}

/**
 * @brief Destroy icon cache.
 */
void _destroy_icon_cache() {
  for (int i = 0; i < icon_info_count; ++i) {
    DestroyIcon(icon_infos[i].icon);
    DestroyIcon(icon_infos[i].large_icon);
    DestroyIcon(icon_infos[i].notification_icon);
    free((void *) icon_infos[i].path);
  }

  free(icon_infos);
  icon_infos = NULL;
  icon_info_count = 0;
}

/**
 * @brief Fetch cached icon.
 * @param icon_record Icon record.
 * @param icon_type Icon type.
 * @return Icon.
 */
HICON _fetch_cached_icon(struct icon_info *icon_record, enum IconType icon_type) {
  switch (icon_type) {
    case REGULAR:
      return icon_record->icon;
    case LARGE:
      return icon_record->large_icon;
    case NOTIFICATION:
      return icon_record->notification_icon;
  }
}

/**
 * @brief Fetch icon.
 * @param path Path to the icon.
 * @param icon_type Icon type.
 * @return Icon.
 */
HICON _fetch_icon(const char *path, enum IconType icon_type) {
  // Find a cached icon by path
  for (int i = 0; i < icon_info_count; ++i) {
    if (strcmp(icon_infos[i].path, path) == 0) {
      return _fetch_cached_icon(&icon_infos[i], icon_type);
    }
  }

  // Expand cache, fetch, and retry
  icon_info_count += 1;
  icon_infos = realloc(icon_infos, sizeof(struct icon_info) * icon_info_count);
  int index = icon_info_count - 1;
  icon_infos[icon_info_count - 1] = _create_icon_info(path);

  return _fetch_cached_icon(&icon_infos[icon_info_count - 1], icon_type);
}

int tray_init(struct tray *tray) {
  wm_taskbarcreated = RegisterWindowMessage("TaskbarCreated");

  _init_icon_cache(tray->allIconPaths, tray->iconPathCount);

  memset(&wc, 0, sizeof(wc));
  wc.cbSize = sizeof(WNDCLASSEX);
  wc.lpfnWndProc = _tray_wnd_proc;
  wc.hInstance = GetModuleHandle(NULL);
  wc.lpszClassName = WC_TRAY_CLASS_NAME;
  if (!RegisterClassEx(&wc)) {
    return -1;
  }

  hwnd = CreateWindowEx(0, WC_TRAY_CLASS_NAME, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  if (hwnd == NULL) {
    return -1;
  }
  UpdateWindow(hwnd);

  memset(&nid, 0, sizeof(nid));
  nid.cbSize = sizeof(NOTIFYICONDATAW);
  nid.hWnd = hwnd;
  nid.uID = 0;
  nid.uFlags = NIF_ICON | NIF_MESSAGE;
  nid.uCallbackMessage = WM_TRAY_CALLBACK_MESSAGE;
  Shell_NotifyIconW(NIM_ADD, &nid);

  tray_update(tray);
  return 0;
}

int tray_loop(int blocking) {
  MSG msg;
  if (blocking) {
    GetMessage(&msg, hwnd, 0, 0);
  } else {
    PeekMessage(&msg, hwnd, 0, 0, PM_REMOVE);
  }
  if (msg.message == WM_QUIT) {
    return -1;
  }
  TranslateMessage(&msg);
  DispatchMessage(&msg);
  return 0;
}

void tray_update(struct tray *tray) {
  UINT id = ID_TRAY_FIRST;
  HMENU prevmenu = hmenu;
  hmenu = _tray_menu(tray->menu, &id);
  SendMessage(hwnd, WM_INITMENUPOPUP, (WPARAM) hmenu, 0);

  HICON icon = _fetch_icon(tray->icon, REGULAR);
  HICON largeIcon = tray->notification_icon != 0 ? _fetch_icon(tray->notification_icon, NOTIFICATION) : _fetch_icon(tray->icon, LARGE);

  if (icon != NULL) {
    nid.hIcon = icon;
  }

  if (largeIcon != 0) {
    nid.hBalloonIcon = largeIcon;
    nid.dwInfoFlags = NIIF_USER | NIIF_LARGE_ICON;
  }
  if (tray->tooltip != 0 && strlen(tray->tooltip) > 0) {
    MultiByteToWideChar(CP_UTF8, 0, tray->tooltip, -1, nid.szTip, sizeof(nid.szTip) / sizeof(wchar_t));
    nid.uFlags |= NIF_TIP;
  }
  QUERY_USER_NOTIFICATION_STATE notification_state;
  HRESULT ns = SHQueryUserNotificationState(&notification_state);
  int can_show_notifications = ns == S_OK && notification_state == QUNS_ACCEPTS_NOTIFICATIONS;
  if (can_show_notifications == 1 && tray->notification_title != 0 && strlen(tray->notification_title) > 0) {
    MultiByteToWideChar(CP_UTF8, 0, tray->notification_title, -1, nid.szInfoTitle, sizeof(nid.szInfoTitle) / sizeof(wchar_t));
    nid.uFlags |= NIF_INFO;
  } else if ((nid.uFlags & NIF_INFO) == NIF_INFO) {
    nid.szInfoTitle[0] = L'\0';
  }
  if (can_show_notifications == 1 && tray->notification_text != 0 && strlen(tray->notification_text) > 0) {
    MultiByteToWideChar(CP_UTF8, 0, tray->notification_text, -1, nid.szInfo, sizeof(nid.szInfo) / sizeof(wchar_t));
  } else if ((nid.uFlags & NIF_INFO) == NIF_INFO) {
    nid.szInfo[0] = L'\0';
  }
  if (can_show_notifications == 1 && tray->notification_cb != NULL) {
    notification_cb = tray->notification_cb;
  }

  Shell_NotifyIconW(NIM_MODIFY, &nid);

  if (prevmenu != NULL) {
    DestroyMenu(prevmenu);
  }
}

void tray_exit(void) {
  Shell_NotifyIconW(NIM_DELETE, &nid);
  SendMessage(hwnd, WM_CLOSE, 0, 0);
  _destroy_icon_cache();
  if (hmenu != 0) {
    DestroyMenu(hmenu);
  }
  UnregisterClass(WC_TRAY_CLASS_NAME, GetModuleHandle(NULL));
}
