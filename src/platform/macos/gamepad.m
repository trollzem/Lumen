/**
 * @file src/platform/macos/gamepad.m
 * @brief Virtual gamepad implementation for macOS.
 * @details Emulates gamepad input via keyboard and mouse events.
 */
#import "gamepad.h"

// Gamepad button bit flags (matching Moonlight/Sunshine protocol)
#define BUTTON_DPAD_UP      0x0001
#define BUTTON_DPAD_DOWN    0x0002
#define BUTTON_DPAD_LEFT    0x0004
#define BUTTON_DPAD_RIGHT   0x0008
#define BUTTON_START        0x0010
#define BUTTON_BACK         0x0020
#define BUTTON_LEFT_STICK   0x0040
#define BUTTON_RIGHT_STICK  0x0080
#define BUTTON_LB           0x0100
#define BUTTON_RB           0x0200
#define BUTTON_HOME         0x0400
#define BUTTON_A            0x1000
#define BUTTON_B            0x2000
#define BUTTON_X            0x4000
#define BUTTON_Y            0x8000

@implementation MacOSGamepad

+ (BOOL)isAvailable {
    // Keyboard/mouse emulation is always available
    return YES;
}

- (instancetype)initWithIndex:(int)index {
    self = [super init];
    if (self) {
        self.gamepadIndex = index;
        self.isConnected = YES;
        self.keyMapping = kDefaultGamepadMapping;
        self.leftStickDeadzone = 0.15f;
        self.rightStickDeadzone = 0.15f;
        self.mouseSensitivity = 15.0f;
        self.buttonState = 0;
        self.leftStickX = 0;
        self.leftStickY = 0;
        self.rightStickX = 0;
        self.rightStickY = 0;
        self.leftTrigger = 0;
        self.rightTrigger = 0;

        self.eventSource = CGEventSourceCreate(kCGEventSourceStateHIDSystemState);

        NSLog(@"[MacOSGamepad] Gamepad %d connected (keyboard/mouse emulation mode)", index);
    }
    return self;
}

- (void)dealloc {
    if (self.eventSource) {
        CFRelease(self.eventSource);
    }
    [super dealloc];
}

- (void)disconnect {
    self.isConnected = NO;
    NSLog(@"[MacOSGamepad] Gamepad %d disconnected", self.gamepadIndex);
}

- (float)normalizeStickValue:(int16_t)value {
    // Normalize from -32768..32767 to -1.0..1.0
    return (float)value / 32767.0f;
}

- (void)pressKey:(int)keyCode {
    CGEventRef keyDown = CGEventCreateKeyboardEvent(self.eventSource, keyCode, true);
    CGEventPost(kCGHIDEventTap, keyDown);
    CFRelease(keyDown);
}

- (void)releaseKey:(int)keyCode {
    CGEventRef keyUp = CGEventCreateKeyboardEvent(self.eventSource, keyCode, false);
    CGEventPost(kCGHIDEventTap, keyUp);
    CFRelease(keyUp);
}

- (void)handleButtonChange:(uint16_t)buttonMask keyCode:(int)keyCode wasPressed:(BOOL)wasPressed isPressed:(BOOL)isPressed {
    if (isPressed && !wasPressed) {
        [self pressKey:keyCode];
    } else if (!isPressed && wasPressed) {
        [self releaseKey:keyCode];
    }
}

- (void)moveMouse:(float)deltaX deltaY:(float)deltaY {
    if (fabs(deltaX) < 0.01f && fabs(deltaY) < 0.01f) {
        return;
    }

    CGEventRef moveEvent = CGEventCreateMouseEvent(
        self.eventSource,
        kCGEventMouseMoved,
        CGPointZero,
        kCGMouseButtonLeft
    );

    // Set relative movement
    CGEventSetIntegerValueField(moveEvent, kCGMouseEventDeltaX, (int)(deltaX * self.mouseSensitivity));
    CGEventSetIntegerValueField(moveEvent, kCGMouseEventDeltaY, (int)(deltaY * self.mouseSensitivity));

    CGEventPost(kCGHIDEventTap, moveEvent);
    CFRelease(moveEvent);
}

- (void)mouseButton:(CGMouseButton)button pressed:(BOOL)pressed {
    CGEventType eventType;

    if (button == kCGMouseButtonLeft) {
        eventType = pressed ? kCGEventLeftMouseDown : kCGEventLeftMouseUp;
    } else if (button == kCGMouseButtonRight) {
        eventType = pressed ? kCGEventRightMouseDown : kCGEventRightMouseUp;
    } else {
        eventType = pressed ? kCGEventOtherMouseDown : kCGEventOtherMouseUp;
    }

    // Get current mouse position
    CGEventRef posEvent = CGEventCreate(self.eventSource);
    CGPoint mousePos = CGEventGetLocation(posEvent);
    CFRelease(posEvent);

    CGEventRef clickEvent = CGEventCreateMouseEvent(
        self.eventSource,
        eventType,
        mousePos,
        button
    );

    CGEventPost(kCGHIDEventTap, clickEvent);
    CFRelease(clickEvent);
}

- (void)updateState:(uint16_t)buttons
         leftStickX:(int16_t)lsX
         leftStickY:(int16_t)lsY
        rightStickX:(int16_t)rsX
        rightStickY:(int16_t)rsY
        leftTrigger:(uint8_t)lt
       rightTrigger:(uint8_t)rt {

    if (!self.isConnected) {
        return;
    }

    uint16_t oldButtons = self.buttonState;

    // Handle button presses
    [self handleButtonChange:BUTTON_A keyCode:self.keyMapping.a_key
                 wasPressed:(oldButtons & BUTTON_A) isPressed:(buttons & BUTTON_A)];
    [self handleButtonChange:BUTTON_B keyCode:self.keyMapping.b_key
                 wasPressed:(oldButtons & BUTTON_B) isPressed:(buttons & BUTTON_B)];
    [self handleButtonChange:BUTTON_X keyCode:self.keyMapping.x_key
                 wasPressed:(oldButtons & BUTTON_X) isPressed:(buttons & BUTTON_X)];
    [self handleButtonChange:BUTTON_Y keyCode:self.keyMapping.y_key
                 wasPressed:(oldButtons & BUTTON_Y) isPressed:(buttons & BUTTON_Y)];
    [self handleButtonChange:BUTTON_LB keyCode:self.keyMapping.lb_key
                 wasPressed:(oldButtons & BUTTON_LB) isPressed:(buttons & BUTTON_LB)];
    [self handleButtonChange:BUTTON_RB keyCode:self.keyMapping.rb_key
                 wasPressed:(oldButtons & BUTTON_RB) isPressed:(buttons & BUTTON_RB)];
    [self handleButtonChange:BUTTON_START keyCode:self.keyMapping.start_key
                 wasPressed:(oldButtons & BUTTON_START) isPressed:(buttons & BUTTON_START)];
    [self handleButtonChange:BUTTON_BACK keyCode:self.keyMapping.back_key
                 wasPressed:(oldButtons & BUTTON_BACK) isPressed:(buttons & BUTTON_BACK)];
    [self handleButtonChange:BUTTON_LEFT_STICK keyCode:self.keyMapping.lstick_key
                 wasPressed:(oldButtons & BUTTON_LEFT_STICK) isPressed:(buttons & BUTTON_LEFT_STICK)];
    [self handleButtonChange:BUTTON_RIGHT_STICK keyCode:self.keyMapping.rstick_key
                 wasPressed:(oldButtons & BUTTON_RIGHT_STICK) isPressed:(buttons & BUTTON_RIGHT_STICK)];

    // D-pad
    [self handleButtonChange:BUTTON_DPAD_UP keyCode:self.keyMapping.dpad_up_key
                 wasPressed:(oldButtons & BUTTON_DPAD_UP) isPressed:(buttons & BUTTON_DPAD_UP)];
    [self handleButtonChange:BUTTON_DPAD_DOWN keyCode:self.keyMapping.dpad_down_key
                 wasPressed:(oldButtons & BUTTON_DPAD_DOWN) isPressed:(buttons & BUTTON_DPAD_DOWN)];
    [self handleButtonChange:BUTTON_DPAD_LEFT keyCode:self.keyMapping.dpad_left_key
                 wasPressed:(oldButtons & BUTTON_DPAD_LEFT) isPressed:(buttons & BUTTON_DPAD_LEFT)];
    [self handleButtonChange:BUTTON_DPAD_RIGHT keyCode:self.keyMapping.dpad_right_key
                 wasPressed:(oldButtons & BUTTON_DPAD_RIGHT) isPressed:(buttons & BUTTON_DPAD_RIGHT)];

    self.buttonState = buttons;

    // Handle left stick -> mouse movement
    float leftX = [self normalizeStickValue:lsX];
    float leftY = [self normalizeStickValue:lsY];

    if (fabs(leftX) > self.leftStickDeadzone || fabs(leftY) > self.leftStickDeadzone) {
        // Apply deadzone
        if (fabs(leftX) <= self.leftStickDeadzone) leftX = 0;
        if (fabs(leftY) <= self.leftStickDeadzone) leftY = 0;

        [self moveMouse:leftX deltaY:leftY];
    }

    self.leftStickX = lsX;
    self.leftStickY = lsY;
    self.rightStickX = rsX;
    self.rightStickY = rsY;

    // Handle triggers -> mouse buttons
    // Left trigger = right click, Right trigger = left click
    BOOL ltPressed = lt > 127;
    BOOL rtPressed = rt > 127;
    BOOL wasLtPressed = self.leftTrigger > 127;
    BOOL wasRtPressed = self.rightTrigger > 127;

    if (ltPressed != wasLtPressed) {
        [self mouseButton:kCGMouseButtonRight pressed:ltPressed];
    }
    if (rtPressed != wasRtPressed) {
        [self mouseButton:kCGMouseButtonLeft pressed:rtPressed];
    }

    self.leftTrigger = lt;
    self.rightTrigger = rt;
}

@end
