/**
 * @file src/platform/macos/virtual_display.m
 * @brief CGVirtualDisplay-based virtual display management for macOS 14+.
 *
 * Spawns a helper subprocess (vd_helper) to create and hold the virtual display.
 * This avoids process-level state in Sunshine (TCC, frameworks, etc.) that prevents
 * CGVirtualDisplay from registering with WindowServer when created in-process.
 *
 * This file is compiled with ARC (-fobjc-arc). See macos.cmake.
 */
#import <Foundation/Foundation.h>
#import <CoreGraphics/CoreGraphics.h>
#include <pthread.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include "virtual_display.h"

extern char **environ;

// State protected by mutex
static pthread_mutex_t vd_mutex = PTHREAD_MUTEX_INITIALIZER;
static pid_t vd_helper_pid = 0;
static uint32_t vd_display_id = 0;

// Get the path to the vd_helper binary (same directory as sunshine)
static NSString *helperPath(void) {
  NSString *mainExe = [[NSBundle mainBundle] executablePath];
  if (!mainExe) {
    // Fallback: use /proc/self or _NSGetExecutablePath
    char buf[4096];
    uint32_t size = sizeof(buf);
    if (_NSGetExecutablePath(buf, &size) == 0) {
      mainExe = [[NSString stringWithUTF8String:buf] stringByResolvingSymlinksInPath];
    }
  }
  if (!mainExe) return nil;
  NSString *dir = [mainExe stringByDeletingLastPathComponent];
  return [dir stringByAppendingPathComponent:@"vd_helper"];
}

uint32_t virtual_display_create(int width, int height, int fps) {
  pthread_mutex_lock(&vd_mutex);

  // Destroy existing display first
  if (vd_helper_pid > 0) {
    NSLog(@"[Sunshine] Killing existing vd_helper (pid=%d, display=%u) before creating new one",
          vd_helper_pid, vd_display_id);
    kill(vd_helper_pid, SIGTERM);
    int status;
    waitpid(vd_helper_pid, &status, 0);
    vd_helper_pid = 0;
    vd_display_id = 0;
  }

  NSString *helper = helperPath();
  if (!helper) {
    NSLog(@"[Sunshine] Could not determine vd_helper path");
    pthread_mutex_unlock(&vd_mutex);
    return 0;
  }

  if (![[NSFileManager defaultManager] isExecutableFileAtPath:helper]) {
    NSLog(@"[Sunshine] vd_helper not found at: %@", helper);
    pthread_mutex_unlock(&vd_mutex);
    return 0;
  }

  NSLog(@"[Sunshine] Spawning vd_helper: %@ %d %d %d", helper, width, height, fps);

  // Set up pipe for reading displayID from child's stdout
  int pipefd[2];
  if (pipe(pipefd) != 0) {
    NSLog(@"[Sunshine] pipe() failed: %s", strerror(errno));
    pthread_mutex_unlock(&vd_mutex);
    return 0;
  }

  // Build argv
  char widthStr[16], heightStr[16], fpsStr[16];
  snprintf(widthStr, sizeof(widthStr), "%d", width);
  snprintf(heightStr, sizeof(heightStr), "%d", height);
  snprintf(fpsStr, sizeof(fpsStr), "%d", fps);

  const char *argv[] = {
    [helper fileSystemRepresentation],
    widthStr,
    heightStr,
    fpsStr,
    NULL
  };

  // Set up posix_spawn file actions: child's stdout → pipe write end
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, pipefd[1], STDOUT_FILENO);
  posix_spawn_file_actions_addclose(&actions, pipefd[0]);
  posix_spawn_file_actions_addclose(&actions, pipefd[1]);

  pid_t pid;
  int err = posix_spawn(&pid, argv[0], &actions, NULL, (char *const *)argv, environ);
  posix_spawn_file_actions_destroy(&actions);

  // Close write end in parent
  close(pipefd[1]);

  if (err != 0) {
    NSLog(@"[Sunshine] posix_spawn failed: %s", strerror(err));
    close(pipefd[0]);
    pthread_mutex_unlock(&vd_mutex);
    return 0;
  }

  NSLog(@"[Sunshine] vd_helper spawned (pid=%d)", pid);

  // Read displayID from child's stdout (with timeout)
  char buf[64] = {0};
  ssize_t n = 0;
  fd_set readfds;
  struct timeval tv;
  tv.tv_sec = 10;
  tv.tv_usec = 0;
  FD_ZERO(&readfds);
  FD_SET(pipefd[0], &readfds);

  int sel = select(pipefd[0] + 1, &readfds, NULL, NULL, &tv);
  if (sel > 0) {
    n = read(pipefd[0], buf, sizeof(buf) - 1);
  }
  close(pipefd[0]);

  if (n <= 0) {
    NSLog(@"[Sunshine] vd_helper produced no output, killing");
    kill(pid, SIGTERM);
    waitpid(pid, NULL, 0);
    pthread_mutex_unlock(&vd_mutex);
    return 0;
  }

  uint32_t displayID = (uint32_t)strtoul(buf, NULL, 10);
  if (displayID == 0) {
    NSLog(@"[Sunshine] vd_helper returned displayID=0, killing");
    kill(pid, SIGTERM);
    waitpid(pid, NULL, 0);
    pthread_mutex_unlock(&vd_mutex);
    return 0;
  }

  vd_helper_pid = pid;
  vd_display_id = displayID;

  NSLog(@"[Sunshine] Virtual display %u created via vd_helper (pid=%d)", displayID, pid);

  // Write display ID to file so launch scripts can find the virtual display
  [@(displayID).stringValue writeToFile:@"/tmp/sunshine_vd_id" atomically:YES encoding:NSUTF8StringEncoding error:nil];

  // Verify from parent process too
  CGDirectDisplayID activeDisplays[32];
  uint32_t displayCount = 0;
  if (CGGetActiveDisplayList(32, activeDisplays, &displayCount) == kCGErrorSuccess) {
    BOOL found = NO;
    for (uint32_t i = 0; i < displayCount; i++) {
      if (activeDisplays[i] == displayID) { found = YES; break; }
    }
    NSLog(@"[Sunshine] Parent sees display %u: %@ in CGGetActiveDisplayList (%u total)",
          displayID, found ? @"FOUND" : @"NOT found", displayCount);
  }

  pthread_mutex_unlock(&vd_mutex);
  return displayID;
}

void virtual_display_destroy(void) {
  pthread_mutex_lock(&vd_mutex);

  if (vd_helper_pid > 0) {
    uint32_t old_id = vd_display_id;
    pid_t old_pid = vd_helper_pid;
    NSLog(@"[Sunshine] Destroying virtual display %u (killing vd_helper pid=%d)", old_id, old_pid);
    kill(old_pid, SIGTERM);
    // Non-blocking wait — child might take a moment
    int status;
    for (int i = 0; i < 10; i++) {
      if (waitpid(old_pid, &status, WNOHANG) != 0) break;
      usleep(100000); // 100ms
    }
    vd_helper_pid = 0;
    vd_display_id = 0;
    NSLog(@"[Sunshine] Destroyed virtual display %u", old_id);
    [[NSFileManager defaultManager] removeItemAtPath:@"/tmp/sunshine_vd_id" error:nil];
  }

  pthread_mutex_unlock(&vd_mutex);
}

uint32_t virtual_display_get_id(void) {
  pthread_mutex_lock(&vd_mutex);
  uint32_t result = vd_display_id;
  pthread_mutex_unlock(&vd_mutex);
  return result;
}
