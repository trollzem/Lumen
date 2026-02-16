# inputtino

An easy to use virtual input library for Linux built on top of `uinput`, `evdev` and `uhid`.  
Currently in use by [Wolf](https://github.com/games-on-whales/wolf), [Sunshine](https://github.com/LizardByte/Sunshine) and [Moonshine](https://github.com/hgaiser/moonshine)

Supports:

- Keyboard
- Mouse
- Touchscreen
- Trackpad
- Pen tablet
- Joypad
    - Correctly emulates Xbox, PS5 or Nintendo joypads
    - Supports callbacks on Rumble events
    - Gyro, Acceleration, Touchpad, Adaptive triggers, LED and battery status fully supported when creating a virtual DualSense joypad (with full support without Steam Input for games that are compatible with DualSense)

Interested in how the joypad works under the hood? Checkout these blog posts:
- [When uinput Isn’t Enough: Virtualizing a DualSense controller](https://abeltra.me/blog/inputtino-uhid-1/)
- [Creating a Virtual DualSense Controller via UHID](https://abeltra.me/blog/inputtino-uhid-2/)
- [Beyond USB: Improving Virtual Controller Support in Linux Games](https://abeltra.me/blog/inputtino-uhid-3/)

A special thanks goes to [@hgaiser](https://github.com/hgaiser) for all the help in yak shaving the DualSense implementation.

## Include in a C++ project

If using `Cmake` it's as simple as

```cmake
FetchContent_Declare(
        inputtino
        GIT_REPOSITORY https://github.com/games-on-whales/inputtino.git
        GIT_TAG <GIT_SHA_OR_TAG>)
FetchContent_MakeAvailable(inputtino)
target_link_libraries(<your_project_name> PUBLIC inputtino::libinputtino)
```

### Example usage

```c++
#include <inputtino/input.hpp>

auto joypad = Joypad::create(Joypad::PS, Joypad::RUMBLE | Joypad::ANALOG_TRIGGERS);

joypad->set_stick(Joypad::LS, 1000, 2000);
joypad->set_pressed_buttons(Joypad::X | Joypad::DPAD_RIGHT);

auto rumble_data = std::make_shared<std::pair<int, int>>();
joypad.set_on_rumble([rumble_data](int low_freq, int high_freq) {
    rumble_data->first = low_freq;
    rumble_data->second = high_freq;
});
```

For more examples you can look at the unit tests under `tests/`: Joypads have been tested using `SDL2` other input
devices have been tested with `libinput`.

The main interface is easily accessible
under [include/inputtino/input.hpp](include/inputtino/input.hpp)

## Using the Python bindings

Checkout the instructions in [bindings/python/](bindings/python/); example usage:

```python
from inputtino import Mouse, MouseButton

# Initialize mouse device
mouse = Mouse()

# Move mouse
mouse.move(100, 50)  # Move right 100, down 50
mouse.move_abs(500, 300, 1920, 1080)  # Move to absolute position

# Click operations
mouse.click(MouseButton.LEFT)
mouse.click(MouseButton.RIGHT, duration=0.5)  # Hold for 0.5 seconds

# Scrolling
mouse.scroll_vertical(120)  # Scroll up
mouse.scroll_horizontal(-120)  # Scroll left
```

## Using the Rust bindings

After building and installing `inputtino` you can use the Rust bindings by adding the following to your `Cargo.toml`:

```toml
[dependencies]
inputtino = { git = "https://github.com/games-on-whales/inputtino.git", branch="stable" }
```

Example usage:

```rust
let device = DeviceDefinition::new(
        "Rusty Keyboard",
        0xAB,
        0xCD,
        0xEF,
        "Rusty Keyboard Phys",
        "Rusty Keyboard Uniq",
    );
let keyboard = Keyboard::new(&device).unwrap();

keyboard.press_key(0x41); // KEY_A
keyboard.release_key(0x41);
```

See the [tests](bindings/rust/inputtino/tests) for more examples.