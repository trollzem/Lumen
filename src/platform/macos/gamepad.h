/**
 * @file src/platform/macos/gamepad.h
 * @brief Virtual gamepad support for macOS.
 * @details Provides gamepad input emulation via:
 *   - Keyboard/Mouse emulation (default, no drivers needed)
 *   - foohid virtual HID (optional, requires kext installation)
 */
#pragma once

#import <Foundation/Foundation.h>
#import <Carbon/Carbon.h>
#import <CoreGraphics/CoreGraphics.h>

// Gamepad button mapping to keyboard keys
typedef struct {
    int a_key;           // A button -> key
    int b_key;           // B button -> key
    int x_key;           // X button -> key
    int y_key;           // Y button -> key
    int lb_key;          // Left bumper -> key
    int rb_key;          // Right bumper -> key
    int start_key;       // Start -> key
    int back_key;        // Back/Select -> key
    int lstick_key;      // Left stick click -> key
    int rstick_key;      // Right stick click -> key
    int dpad_up_key;     // D-pad up -> key
    int dpad_down_key;   // D-pad down -> key
    int dpad_left_key;   // D-pad left -> key
    int dpad_right_key;  // D-pad right -> key
} GamepadKeyMapping;

// Default mapping (common gaming keys)
static const GamepadKeyMapping kDefaultGamepadMapping = {
    .a_key = kVK_Space,        // A -> Space (jump/action)
    .b_key = kVK_ANSI_E,       // B -> E (interact)
    .x_key = kVK_ANSI_R,       // X -> R (reload)
    .y_key = kVK_ANSI_F,       // Y -> F (use)
    .lb_key = kVK_ANSI_Q,      // LB -> Q
    .rb_key = kVK_Tab,         // RB -> Tab
    .start_key = kVK_Escape,   // Start -> Escape
    .back_key = kVK_Tab,       // Back -> Tab
    .lstick_key = kVK_Shift,   // L3 -> Shift (sprint)
    .rstick_key = kVK_ANSI_V,  // R3 -> V (melee)
    .dpad_up_key = kVK_ANSI_1,     // D-pad -> number keys (weapon select)
    .dpad_down_key = kVK_ANSI_3,
    .dpad_left_key = kVK_ANSI_4,
    .dpad_right_key = kVK_ANSI_2,
};

@interface MacOSGamepad : NSObject

@property (nonatomic, assign) int gamepadIndex;
@property (nonatomic, assign) BOOL isConnected;
@property (nonatomic, assign) GamepadKeyMapping keyMapping;
@property (nonatomic, assign) float leftStickDeadzone;
@property (nonatomic, assign) float rightStickDeadzone;
@property (nonatomic, assign) float mouseSensitivity;
@property (nonatomic, assign) CGEventSourceRef eventSource;

// Button state tracking
@property (nonatomic, assign) uint16_t buttonState;
@property (nonatomic, assign) int16_t leftStickX;
@property (nonatomic, assign) int16_t leftStickY;
@property (nonatomic, assign) int16_t rightStickX;
@property (nonatomic, assign) int16_t rightStickY;
@property (nonatomic, assign) uint8_t leftTrigger;
@property (nonatomic, assign) uint8_t rightTrigger;

+ (BOOL)isAvailable;

- (instancetype)initWithIndex:(int)index;
- (void)updateState:(uint16_t)buttons
         leftStickX:(int16_t)lsX
         leftStickY:(int16_t)lsY
        rightStickX:(int16_t)rsX
        rightStickY:(int16_t)rsY
        leftTrigger:(uint8_t)lt
       rightTrigger:(uint8_t)rt;
- (void)disconnect;

@end
