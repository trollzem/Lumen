/**
 * @file src/tray_linux.c
 * @brief System tray implementation for Linux.
 */
// standard includes
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

// lib includes
#ifdef TRAY_AYATANA_APPINDICATOR
  #include <libayatana-appindicator/app-indicator.h>
#elif TRAY_LEGACY_APPINDICATOR
  #include <libappindicator/app-indicator.h>
#endif
#ifndef IS_APP_INDICATOR
  #define IS_APP_INDICATOR APP_IS_INDICATOR  ///< Define IS_APP_INDICATOR for app-indicator compatibility.
#endif
#include <libnotify/notify.h>
#define TRAY_APPINDICATOR_ID "tray-id"  ///< Tray appindicator ID.

// local includes
#include "tray.h"

static bool async_update_pending = false;
static pthread_cond_t async_update_cv = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t async_update_mutex = PTHREAD_MUTEX_INITIALIZER;

static AppIndicator *indicator = NULL;
static int loop_result = 0;
static NotifyNotification *currentNotification = NULL;

static void _tray_menu_cb(GtkMenuItem *item, gpointer data) {
  (void) item;
  struct tray_menu *m = (struct tray_menu *) data;
  m->cb(m);
}

static GtkMenuShell *_tray_menu(struct tray_menu *m) {
  GtkMenuShell *menu = (GtkMenuShell *) gtk_menu_new();
  for (; m != NULL && m->text != NULL; m++) {
    GtkWidget *item;
    if (strcmp(m->text, "-") == 0) {
      item = gtk_separator_menu_item_new();
    } else {
      if (m->submenu != NULL) {
        item = gtk_menu_item_new_with_label(m->text);
        gtk_menu_item_set_submenu(GTK_MENU_ITEM(item), GTK_WIDGET(_tray_menu(m->submenu)));
      } else if (m->checkbox) {
        item = gtk_check_menu_item_new_with_label(m->text);
        gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(item), !!m->checked);
      } else {
        item = gtk_menu_item_new_with_label(m->text);
      }
      gtk_widget_set_sensitive(item, !m->disabled);
      if (m->cb != NULL) {
        g_signal_connect(item, "activate", G_CALLBACK(_tray_menu_cb), m);
      }
    }
    gtk_widget_show(item);
    gtk_menu_shell_append(menu, item);
  }
  return menu;
}

int tray_init(struct tray *tray) {
  if (gtk_init_check(0, NULL) == FALSE) {
    return -1;
  }
  notify_init("tray-icon");
  indicator = app_indicator_new(TRAY_APPINDICATOR_ID, tray->icon, APP_INDICATOR_CATEGORY_APPLICATION_STATUS);
  if (indicator == NULL || !IS_APP_INDICATOR(indicator)) {
    return -1;
  }
  app_indicator_set_status(indicator, APP_INDICATOR_STATUS_ACTIVE);
  tray_update(tray);
  return 0;
}

int tray_loop(int blocking) {
  gtk_main_iteration_do(blocking);
  return loop_result;
}

static gboolean tray_update_internal(gpointer user_data) {
  struct tray *tray = user_data;

  if (indicator != NULL && IS_APP_INDICATOR(indicator)) {
    app_indicator_set_icon_full(indicator, tray->icon, tray->icon);
    // GTK is all about reference counting, so previous menu should be destroyed
    // here
    app_indicator_set_menu(indicator, GTK_MENU(_tray_menu(tray->menu)));
  }
  if (tray->notification_text != 0 && strlen(tray->notification_text) > 0 && notify_is_initted()) {
    if (currentNotification != NULL && NOTIFY_IS_NOTIFICATION(currentNotification)) {
      notify_notification_close(currentNotification, NULL);
      g_object_unref(G_OBJECT(currentNotification));
    }
    const char *notification_icon = tray->notification_icon != NULL ? tray->notification_icon : tray->icon;
    currentNotification = notify_notification_new(tray->notification_title, tray->notification_text, notification_icon);
    if (currentNotification != NULL && NOTIFY_IS_NOTIFICATION(currentNotification)) {
      if (tray->notification_cb != NULL) {
        notify_notification_add_action(currentNotification, "default", "Default", NOTIFY_ACTION_CALLBACK(tray->notification_cb), NULL, NULL);
      }
      notify_notification_show(currentNotification, NULL);
    }
  }

  // Unwait any pending tray_update() calls
  pthread_mutex_lock(&async_update_mutex);
  async_update_pending = false;
  pthread_cond_broadcast(&async_update_cv);
  pthread_mutex_unlock(&async_update_mutex);
  return G_SOURCE_REMOVE;
}

void tray_update(struct tray *tray) {
  // Perform the tray update on the tray loop thread, but block
  // in this thread to ensure none of the strings stored in the
  // tray icon struct go out of scope before the callback runs.

  if (g_main_context_is_owner(g_main_context_default())) {
    // Invoke the callback directly if we're on the loop thread
    tray_update_internal(tray);
  } else {
    // If there's already an update pending, wait for it to complete
    // and claim the next pending update slot.
    pthread_mutex_lock(&async_update_mutex);
    while (async_update_pending) {
      pthread_cond_wait(&async_update_cv, &async_update_mutex);
    }
    async_update_pending = true;
    pthread_mutex_unlock(&async_update_mutex);

    // Queue the update callback to the tray thread
    g_main_context_invoke(NULL, tray_update_internal, tray);

    // Wait for the callback to run
    pthread_mutex_lock(&async_update_mutex);
    while (async_update_pending) {
      pthread_cond_wait(&async_update_cv, &async_update_mutex);
    }
    pthread_mutex_unlock(&async_update_mutex);
  }
}

static gboolean tray_exit_internal(gpointer user_data) {
  if (currentNotification != NULL && NOTIFY_IS_NOTIFICATION(currentNotification)) {
    int v = notify_notification_close(currentNotification, NULL);
    if (v == TRUE) {
      g_object_unref(G_OBJECT(currentNotification));
    }
  }
  notify_uninit();
  return G_SOURCE_REMOVE;
}

void tray_exit(void) {
  // Wait for any pending callbacks to complete
  pthread_mutex_lock(&async_update_mutex);
  while (async_update_pending) {
    pthread_cond_wait(&async_update_cv, &async_update_mutex);
  }
  pthread_mutex_unlock(&async_update_mutex);

  // Perform cleanup on the main thread
  loop_result = -1;
  g_main_context_invoke(NULL, tray_exit_internal, NULL);
}
