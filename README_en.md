[日本語](README.md)

# MeganeMouse: A Glasses-Mounted Head Mouse

![MeganeMouse: A Glasses-Mounted Head Mouse](MeganeMouse.jpeg)

This glasses-mounted head mouse provides only cursor movement functionality from typical mouse operations. You can combine it with regular mouse/trackpad/external switches for clicking operations as needed.

- **Development**: Shigeru Kobayashi (+ Claude Code, Gemini)
- **Development Collaboration**: Hideo Arai

> I am facing ALS (Amyotrophic Lateral Sclerosis), a progressive intractable disease that progressively weakens muscles throughout my body, leading to a loss of movement. As the nature of my disability continuously changes over time, the "just right" method of support must also constantly evolve. Last winter, operating a keyboard became difficult, and by this spring, using a mouse with my hands also became challenging. With movement remaining in my head and neck, I was searching for a "just right" way to operate my computer and tablet. Eye-tracking devices, which allow computer operation using only eye movements, are the mainstream option for when ALS progresses further and movement becomes more limited, but it's still too early for me to use one. Existing products were available that could detect my current head movements and substitute all mouse functions. I tried them, but they were too multifunctional, causing frequent malfunctions, and ultimately weren't a good fit for me. I was in a bind! In this day and age, being unable to receive and send information via digital devices leads to social isolation. When I consulted with Kobayashi-san, he proposed: "Let's try making a glasses-type device with simplified functionality, limited to just moving the pointer. You can handle clicking with the fingers that you can still move slightly. Instead of incorporating all mouse operations, let's narrow down the functions and entrust the rest to the physical abilities Arai-san still has... How about something like that?" "That's exactly it! That's what I was hoping for! (laughs)" I replied. Rather than just adding functions, he created the "just right" solution by subtracting them.
> 
> As a development collaborator, I sincerely hope that these "Kobayashi Glasses" will be a "just right" benefit for others facing the same challenges I did. Incidentally, I am inputting this very text stress-free using the "Kobayashi Glasses" to control the pointer almost as freely as I would with my hands, combined with voice input and an on-screen virtual keyboard. The perceived speed feels almost the same as before I lost the use of my hands. (Written October 27, 2025)
> 
> Hideo Arai, Physical Performer, Dance Artist

[![Arai-san operating the "Kobayashi Mouse"](https://img.youtube.com/vi/oqdMLz64TT0/maxresdefault.jpg)](https://youtu.be/oqdMLz64TT0?cc_load_policy=1&cc_lang_pref=en)

## Features

- **Head Movement Control**: Convert head movements to precise cursor movement (no clicking)
- **Dual Connection Modes**: USB HID (device name: `AtomS3R`) and Bluetooth HID (device name: `MeganeMouse-BT`) with runtime switching
- **Flexible Device Orientation**: 5-way mounting configuration (Left-V/H, Right-V/H, Back-V)
- **WiFi Configuration**: Easy setup and adjustment via web interface
- **Customizable Response**: Adjustable sensitivity, response curves, and movement balance
- **Mounting Angle Correction**: 3D rotation compensation for imperfect device mounting
- **Visual Status Display**: LCD shows connection status and device information

## Hardware Requirements

- **M5Stack [AtomS3R](https://docs.m5stack.com/en/core/AtomS3R)**: Development board
- **USB-C cable**: For power and data connection
- **Glasses**: With wide, flat temples (arms) to provide a stable mounting surface for the double-sided tape
- **Double-sided tape**: Such as 3M [KRG-15](https://www.scotch.jp/3M/ja_JP/p/d/v101693093/), ultra-strong yet removable
- **M5Stack [Tail Bat](https://docs.m5stack.com/en/atom/tailbat)**: Battery for Atom series (optional, for battery operation)

## Quick Start

### 1. Hardware Setup

1. Mount the device (AtomS3R) on glasses using double-sided tape
2. Connect the device to your PC via USB-C cable

### 2. Firmware Upload

#### Using Arduino IDE

1. Install [Arduino IDE](https://www.arduino.cc/en/software/)
2. Open `MeganeMouse.ino` in Arduino IDE
3. Install board: `M5Stack`
4. Install required libraries:
   - `M5AtomS3` (including library dependencies)
   - `NimBLE-Arduino` (for Bluetooth support)
5. Select board: `M5AtomS3R`
6. Upload the code

#### Using M5Burner

1. Create an account at [M5Stack Community](https://community.m5stack.com/)
2. Install [M5Burner](https://docs.m5stack.com/en/uiflow/m5burner/intro)
3. Click the :bust_in_silhouette: button at the top of the screen and log in with your registered email and password
4. Select `ATOMS3` tab on the left side of the window
5. Enter `MeganeMouse` in the search box at the top of the window and select this project's firmware
6. Press the `Download` button to download the firmware
7. Click the `Burn` button, select the appropriate COM port in the displayed screen, and click the `Start` button to flash the firmware

### 3. Initial Setup

1. Power on the device. The LCD will show a blue screen (power on), and then turn yellow to begin the calibration process (see below).
2. Calibration Process:
   1. Yellow screen: Motion detection (wait for 3 seconds of device stillness)
   2. Red screen: Active calibration (with progress display)
   3. Green screen: Calibration successful OR Orange screen: Calibration failed (using stored calibration values)
3. The device automatically saves calibration values and uses them as fallback

### 4. Configuration

1. Long press the main button (which is the device's screen) for 2+ seconds to enable WiFi (disabled by default for power saving)
2. Connect to WiFi network: Network name is `MeganeMouse-WiFi`, password is `MeganeMouse`
3. Access `192.168.4.1` in your browser
4. Configure connection method (USB/Bluetooth), device orientation, and other settings as needed on the displayed web page
5. Click `Apply Settings | 設定を反映` button on the web page to apply settings
6. Long press the main button to disable WiFi (device automatically restarts when connection method is changed)

Functions available from the main web page:

- `View Current Status | 状態と動作情報を表示`: Display current device status and operation information
- `Reset to Defaults | 初期値に戻す`: Reset all settings to defaults
- `Recalibrate | 再校正する`: Recalibrate (execute when device is in a stable state)

## Configuration Options

### Sensitivity Levels

Choose the overall responsiveness of the head mouse:

- **Low | 低**: Slower, more precise movements - ideal for detailed work requiring accuracy
- **Medium | 中**: Balanced speed and precision (default) - suitable for general use
- **High | 高**: Faster, more responsive movements - optimal for fast navigation and gaming

### Device Orientation

Select the option that matches how you've mounted the AtomS3R on your glasses.

If mounted on the LEFT temple (arm):

- Left-V (Default): Choose this if the device is vertical (USB-C port points down).
- Left-H: Choose this if the device is horizontal (USB-C port points backward).

If mounted on the RIGHT temple (arm):

- Right-V: Choose this if the device is vertical (USB-C port points down).
- Right-H: Choose this if the device is horizontal (USB-C port points backward).

If mounted on the BACK of the head (e.g., on a strap):

- Back-V: Choose this if the device is vertical (USB-C port points down).

V (Vertical) means the USB-C port points downwards (relative to the glasses). H (Horizontal) means the USB-C port points backward.

### Connection Mode

Connection mode selection:

- **USB HID**: Wired connection via USB (shortest latency)
- **Bluetooth HID**: Wireless connection via Bluetooth Low Energy
- Device restarts after mode changes for proper initialization

### Response Curve Settings

The response curve system provides natural, adaptive mouse control by adjusting sensitivity based on your movement patterns.

#### Base Curve Shape

Controls the overall sensitivity curve.

- Setting smaller values (0.1-0.5) provides more linear response for precise control.
- Setting larger values (1.0-2.0) creates more aggressive acceleration effects for easier fast movements.
- Start with the default 0.2 and increase the value if you want to enhance the speed improvement effect when moving your head quickly.

#### Adaptation Strength

Determines how much the curve adapts to your movement patterns.

- Set to 0.0 for consistent response regardless of speed.
- Set to 1.0-2.0 for more responsive reactions during fast movements or to keep delicate positioning smooth during slow movements.
- The default 0.5 provides balanced settings that enhance fast movement effects while preventing slow movement sensitivity from becoming excessive.

#### Parameter Relationship

#### Recommended Combinations

- Precise control: Base Curve Shape: 0.2, Adaptation Strength: 0.3
- Balanced (default): Base Curve Shape: 0.2, Adaptation Strength: 0.5
- Gaming/fast: Base Curve Shape: 0.8, Adaptation Strength: 1.2

### Mounting Angle Correction (units in degrees)

Configure these if you want to correct the device mounting angle (default values are all 0):

- **Pitch Correction**: Compensate for forward/backward tilt (+: when tilted backward, -: when tilted forward)
- **Roll Correction**: Compensate for left/right tilt (+: when tilted right, -: when tilted left)
- **Yaw Correction**: Compensate for rotational misalignment (+: when rotated clockwise, -: when rotated counter-clockwise)

### Movement Balance (units in %)

Configure these if you want to fine-tune movement sensitivity in each direction (default values are all 0):

- **Horizontal Balance**: Adjust left/right movement sensitivity (+: enhance right, -: enhance left)
- **Vertical Balance**: Adjust up/down movement sensitivity (+: enhance down, -: enhance up)

## Troubleshooting

### Cursor Moving Erratically

- Confirm the device is securely attached to glasses
- Change sensitivity to a lower value
- Use the web interface `Recalibrate | 再校正する` button to recalibrate

### Cursor Not Moving

- **USB Mode**: Use a USB cable capable of data communication and confirm the device is recognized by PC/tablet
- **Bluetooth Mode**: Ensure device is paired and LCD displays "Connected"
- Confirm device orientation is set correctly
- Use the web interface `Recalibrate | 再校正する` button to recalibrate

### Bluetooth Connection Issues

- Look for "MeganeMouse-BT" in Bluetooth device list
- Confirm device LCD displays "ADVERTISING" or "CONNECTED"
- Try unpairing and re-pairing the device
- **Clear all pairings**: Hold button for 3+ seconds while powering on to reset Bluetooth pairings
- Restart device if connection fails

### Poor Responsiveness

- Try different sensitivity levels
- Adjust response curve settings
- Check mounting angle corrections
- Verify device orientation matches physical mounting

### Can't Connect to WiFi

- Long press button to enable WiFi (screen flashes cyan)
- Look for access point with SSID `MeganeMouse-WiFi`
- Enter password `MeganeMouse`
- Try restarting the device

## Developer Information

### Tested Combinations

- Arduino IDE: v2.3.6
- M5Stack (board): v3.2.3
- M5AtomS3 (library): v1.0.2
- NimBLE-Arduino (library): v.2.3.6

### System Architecture

- **Update Rate**: 50Hz (20ms intervals)
- **Calibration**: Motion detection, progress display, automatic retry (up to 3 attempts), persistent storage, fallback system
- **Filtering**: Speed-adaptive IIR filter with dynamic alpha values
- **Coordinate System**: 3D rotation matrices for mounting correction

### Performance Specifications

- **Resolution**: Sub-degree head movement detection
- **Latency**: ~20ms end-to-end processing time
- **Stability**: Automatic drift compensation
- **Range**: ±45° mounting angle correction

### Debug Mode

To enable debug output instead of cursor control:

1. Change `#define DEBUG_MODE 0` to `#define DEBUG_MODE 1`
2. Recompile and upload
3. Open Serial Monitor at 115200 baud to see debug information

**Note**: The M5AtomS3R's USB controller can only operate as one type of device at a time (e.g., a USB HID Mouse or a Serial/Debug port, but not both). When HID is enabled (`#define DEBUG_MODE 0`), serial output via USB is disabled. You must modify the code to select the mode.

### Customization

Key parameters can be adjusted in the `Config` namespace:

- Sensitivity divisors
- Calibration thresholds
- Filter parameters
- Update intervals

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## Support

For issues, questions, or contributions, please open an issue on the project repository.
