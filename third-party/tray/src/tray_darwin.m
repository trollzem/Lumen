/**
 * @file src/tray_darwin.m
 * @brief System tray implementation for macOS.
 */
// standard includes
#include <string.h>

// lib includes
#include <Cocoa/Cocoa.h>

// local includes
#include "tray.h"

/**
 * @class AppDelegate
 * @brief The application delegate that handles menu actions.
 */
@interface AppDelegate: NSObject <NSApplicationDelegate>
/**
 * @brief Callback function for menu item actions.
 * @param sender The object that sent the action message.
 * @return void
 */
- (IBAction)menuCallback:(id)sender;
@end

@implementation AppDelegate {
}

- (IBAction)menuCallback:(id)sender {
  struct tray_menu *m = [[sender representedObject] pointerValue];
  if (m != NULL && m->cb != NULL) {
    m->cb(m);
  }
}

@end

static NSApplication *app;
static NSStatusBar *statusBar;
static NSStatusItem *statusItem;

#define QUIT_EVENT_SUBTYPE 0x0DED  ///< NSEvent subtype used to signal exit.

static NSMenu *_tray_menu(struct tray_menu *m) {
  NSMenu *menu = [[NSMenu alloc] init];
  [menu setAutoenablesItems:FALSE];

  for (; m != NULL && m->text != NULL; m++) {
    if (strcmp(m->text, "-") == 0) {
      [menu addItem:[NSMenuItem separatorItem]];
    } else {
      NSMenuItem *menuItem = [[NSMenuItem alloc]
        initWithTitle:[NSString stringWithUTF8String:m->text]
               action:@selector(menuCallback:)
        keyEquivalent:@""];
      [menuItem setEnabled:(m->disabled ? FALSE : TRUE)];
      [menuItem setState:(m->checked ? 1 : 0)];
      [menuItem setRepresentedObject:[NSValue valueWithPointer:m]];
      [menu addItem:menuItem];
      if (m->submenu != NULL) {
        [menu setSubmenu:_tray_menu(m->submenu) forItem:menuItem];
      }
    }
  }
  return menu;
}

int tray_init(struct tray *tray) {
  AppDelegate *delegate = [[AppDelegate alloc] init];
  app = [NSApplication sharedApplication];
  [app setDelegate:delegate];
  statusBar = [NSStatusBar systemStatusBar];
  statusItem = [statusBar statusItemWithLength:NSVariableStatusItemLength];
  tray_update(tray);
  [app activateIgnoringOtherApps:TRUE];
  return 0;
}

int tray_loop(int blocking) {
  NSDate *until = (blocking ? [NSDate distantFuture] : [NSDate distantPast]);
  NSEvent *event = [app nextEventMatchingMask:ULONG_MAX
                                    untilDate:until
                                       inMode:[NSString stringWithUTF8String:"kCFRunLoopDefaultMode"]
                                      dequeue:TRUE];
  if (event) {
    if (event.type == NSEventTypeApplicationDefined && event.subtype == QUIT_EVENT_SUBTYPE) {
      return -1;
    }

    [app sendEvent:event];
  }
  return 0;
}

void tray_update(struct tray *tray) {
  NSImage *image = [[NSImage alloc] initWithContentsOfFile:[NSString stringWithUTF8String:tray->icon]];
  NSSize size = NSMakeSize(16, 16);
  [image setSize:NSMakeSize(16, 16)];
  statusItem.button.image = image;
  [statusItem setMenu:_tray_menu(tray->menu)];
}

void tray_exit(void) {
  NSEvent *event = [NSEvent otherEventWithType:NSEventTypeApplicationDefined
                                      location:NSMakePoint(0, 0)
                                 modifierFlags:0
                                     timestamp:0
                                  windowNumber:0
                                       context:nil
                                       subtype:QUIT_EVENT_SUBTYPE
                                         data1:0
                                         data2:0];
  [app postEvent:event atStart:FALSE];
}
