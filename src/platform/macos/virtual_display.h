/**
 * @file src/platform/macos/virtual_display.h
 * @brief Declarations for CGVirtualDisplay-based virtual display management on macOS.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Create a virtual display with the specified resolution and refresh rate.
 * @param width Display width in pixels.
 * @param height Display height in pixels.
 * @param fps Refresh rate in Hz.
 * @return The CGDirectDisplayID of the created display, or 0 on failure.
 */
uint32_t virtual_display_create(int width, int height, int fps);

/**
 * @brief Destroy the currently active virtual display.
 */
void virtual_display_destroy(void);

/**
 * @brief Get the display ID of the currently active virtual display.
 * @return The CGDirectDisplayID, or 0 if no virtual display is active.
 */
uint32_t virtual_display_get_id(void);

#ifdef __cplusplus
}
#endif
