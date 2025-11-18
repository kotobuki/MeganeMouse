# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MeganeMouse is a head-controlled mouse device for the M5Stack AtomS3R development board. It converts head movements to cursor movement using the built-in gyroscope sensor, designed for individuals with limited hand mobility (particularly ALS patients). The device provides cursor movement via head tracking and supports an optional external switch for left mouse button clicks.

**Platform**: Arduino/ESP32 (M5Stack AtomS3R)
**Language**: C++ (Arduino framework)
**Hardware**:
- M5Stack AtomS3R with built-in 6-axis IMU sensor
- Optional external switch on GPIO pin 1 for left mouse button

## Building and Uploading

### Arduino IDE Method

1. Open `MeganeMouse.ino` in Arduino IDE
2. Install the M5Stack board support package (version 3.2.3 tested)
3. Install required libraries:
   - `M5AtomS3` (v1.0.2) - includes all M5 dependencies
   - `NimBLE-Arduino` (v2.3.6) - for Bluetooth HID support
4. Select board: **M5AtomS3R**
5. Connect device via USB-C cable
6. Upload the sketch

### M5Burner Method (Pre-compiled Firmware)

M5Burner is the official M5Stack firmware flashing tool that can download and flash pre-compiled binaries.

1. Download and install M5Burner from M5Stack docs
2. Create/login to M5Stack Community account
3. Search for "MeganeMouse" firmware
4. Download and burn to device

## Key Architecture Concepts

### Dual HID Mode Architecture

The device supports **two mutually exclusive HID modes** controlled by `#define DEBUG_MODE`:

- **DEBUG_MODE = 0** (Default): HID mouse mode (USB or Bluetooth)
  - USB HID Mouse via `USBHIDMouse` library
  - Bluetooth HID Mouse via `NimBLE-Arduino` library
  - Serial debugging is DISABLED (ESP32 USB controller limitation)

- **DEBUG_MODE = 1**: Debug mode
  - Serial output enabled at 115200 baud
  - Mouse functionality disabled
  - Used for sensor debugging and calibration tuning

**Important**: The M5AtomS3R USB controller can only function as ONE device type at a time (either HID Mouse OR Serial/Debug port). To switch modes, you must modify the code, recompile, and upload.

### Connection Modes (Runtime Switchable)

When in HID mode (DEBUG_MODE = 0), users can switch between:

- **USB HID**: Wired connection via USB-C (lowest latency)
- **Bluetooth HID**: Wireless BLE connection (device name: `MeganeMouse-BT`)

Mode switching is done via the WiFi web interface and requires a device restart.

### External Switch Support

An external switch can be connected to **GPIO pin 1** to provide left mouse button functionality:

- **Hardware**: Connect switch between GPIO 1 and GND
- **Configuration**: Pin initialized with `INPUT_PULLUP` (active LOW)
- **Operation**: Switch pressed = button down, switch released = button up
- **USB HID**: Uses `Mouse.press()` and `Mouse.release()` for state changes
- **Bluetooth HID**: Button state encoded in HID report byte 2, bit 0

**State Tracking**:
- `switchIsPressed`: Current switch state (updated every 20ms)
- `switchWasPressed`: Previous state for edge detection

**Status Display**: LCD shows real-time status bar at bottom:
- Format: `USB | SW:ON` or `BT | SW:  ` (blank when not pressed)
- Updates immediately on state change or every 100ms
- Hidden when WiFi configuration is active

### Calibration System

The device performs **automatic gyroscope calibration** on boot:

1. **Motion Detection Phase** (Yellow screen): Waits for device to be stationary for 3 consecutive checks
2. **Calibration Phase** (Red screen): Collects 64 samples to calculate gyro bias values
3. **Result Phase**:
   - Green = Success (new calibration saved)
   - Orange = Failed (using stored fallback calibration)
   - Red = Failed with no backup

Calibration values are persisted in ESP32 flash memory (Preferences library) and used as fallback.

### Sensor Data Flow

```
Raw IMU Data (rad/s)
  ↓
Calibration Bias Removal
  ↓
Convert to deg/s
  ↓
3D Mounting Angle Correction (Pitch/Roll/Yaw rotation matrices)
  ↓
Axis Mapping (based on device orientation: LEFT_V, LEFT_H, RIGHT_V, RIGHT_H, BACK_V)
  ↓
Speed-Based IIR Filtering (adaptive smoothing)
  ↓
Response Curve Application (quadratic, speed-adaptive)
  ↓
Sensitivity Scaling
  ↓
Balance Adjustment
  ↓
Speed Limiting
  ↓
HID Mouse Movement (USB or Bluetooth)
```

### Device Orientation System

Supports 5 mounting positions on glasses:

- **LEFT_V**: Left temple, USB-C port pointing down (default)
- **LEFT_H**: Left temple, USB-C port pointing backward
- **RIGHT_V**: Right temple, USB-C port pointing down
- **RIGHT_H**: Right temple, USB-C port pointing backward
- **BACK_V**: Back of head (strap), USB-C port pointing down

Each orientation remaps gyroscope axes (gx, gy, gz) to horizontal/vertical cursor movement.

### WiFi Configuration Interface

- WiFi is **disabled by default** for power saving
- Long press button (2+ seconds) to toggle WiFi on/off
- Creates AP: SSID=`MeganeMouse-WiFi`, Password=`MeganeMouse`
- Web interface at `192.168.4.1` for all configuration
- Settings saved to ESP32 Preferences (flash memory)

### Adaptive Filtering & Response

**Speed-Based IIR Filter**:
- Low-pass filter with dynamic alpha value
- Slow movement (< 2 deg/s): Heavy filtering (alpha = 0.4)
- Fast movement (> 50 deg/s): Light filtering (alpha = 0.9)
- Prevents jitter during precise positioning while allowing fast navigation

**Response Curve System**:
- Base curve: Quadratic response `output = input * (1 + k * |input|)`
- Adaptation strength: Modifies k based on movement speed
- Provides natural acceleration for fast movements

## Configuration Parameters

All tunable parameters are centralized in the `Config` namespace (lines 82-134):

**Timing**:
- `UPDATE_INTERVAL_MS`: 20ms (50Hz update rate)
- `CALIBRATION_TIMEOUT_MS`: 10 seconds
- `BT_RESET_HOLD_DURATION_MS`: 3 seconds

**Calibration**:
- `CALIBRATION_SAMPLES`: 64 samples
- `CALIBRATION_MAX_STD_DEVIATION`: 90.0 rad/s

**Motion Detection Thresholds**:
- `MOTION_STATIONARY_THRESHOLD`: 2.0 deg/s
- `MOTION_SLOW_THRESHOLD`: 10.0 deg/s
- `MOTION_FAST_THRESHOLD`: 50.0 deg/s

**Sensitivity Divisors**:
- `SENS_HIGH_DIVISOR`: 100.0
- `SENS_MEDIUM_DIVISOR`: 200.0
- `SENS_LOW_DIVISOR`: 400.0

**User-Configurable Defaults** (saved to Preferences):
- Base curve shape (0.1-2.0, default 0.2)
- Adaptation strength (0.0-2.0, default 0.5)
- Mounting angle corrections (±45°)
- Movement balance (±50%)

## Important Code Locations

- **Main loop**: Line 1517 - 50Hz update cycle
- **External switch configuration**: Line 63 - SWITCH_PIN constant (GPIO 1)
- **Switch state reading**: Line 1536 - Read switch every 20ms
- **Status bar display**: Line 1113 - updateStatusBar() function
- **USB HID button handling**: Line 1580-1585 - Press/release on state change
- **Bluetooth HID button**: Line 1266 - sendBluetoothMouseMove() with button parameter
- **Calibration function**: Line 513 - Enhanced calibration with progress display
- **Mounting angle correction**: Line 300 - 3D rotation matrix application
- **Axis mapping**: Line 1553 - Device orientation handling
- **Web server handlers**: Line 750 - Configuration interface
- **Bluetooth HID initialization**: Line 1187 - NimBLE setup
- **Speed-based IIR filter**: Line 205 - Adaptive filtering class
- **Response curve**: Line 330 - Quadratic response function

## Common Development Tasks

### Testing Sensor Readings

```cpp
// Change line 35:
#define DEBUG_MODE 0  →  #define DEBUG_MODE 1

// Recompile and upload
// Open Serial Monitor at 115200 baud
```

### Adjusting Sensitivity

Modify sensitivity divisors in `Config` namespace (lines 118-120) or adjust via web interface.

### Tuning Calibration

Adjust `CALIBRATION_SAMPLES` (line 103) or `CALIBRATION_MAX_STD_DEVIATION` (line 104).

### Modifying Filter Behavior

Edit `SpeedBasedIIRFilter` class parameters (lines 212-215):
- `MIN_ALPHA`: Filtering for slow movement
- `MAX_ALPHA`: Filtering for fast movement
- `SPEED_LOW/HIGH`: Speed thresholds

### Adding New Device Orientations

1. Add enum value to `DeviceOrientation` (line 74)
2. Add case in axis mapping switch statement (line 1553)
3. Update web interface HTML (line 846)

### Changing External Switch Pin

To use a different GPIO pin for the external switch:

```cpp
// Change line 63:
const int SWITCH_PIN = 1;  →  const int SWITCH_PIN = <new_pin>;
```

Ensure the chosen pin supports INPUT_PULLUP and is not used by other peripherals.

### Customizing Status Bar Display

Modify `updateStatusBar()` function (line 1113) to change:
- Display format and text
- Update frequency (DISPLAY_UPDATE_INTERVAL_MS, line 297)
- Position on screen
- Connection status indicators

## Bluetooth Pairing Management

- Hold button on boot for 3+ seconds: Clear all Bluetooth pairings (only works in Bluetooth mode)
- Uses NimBLE bonding storage
- Pairing reset useful when switching between computers

## Performance Specifications

- **Update Rate**: 50Hz (20ms intervals)
- **Latency**: ~20ms end-to-end processing
- **Resolution**: Sub-degree head movement detection
- **Mounting Tolerance**: ±45° angle correction
- **Stability**: Automatic drift compensation via calibration

## State Management

All persistent settings stored in ESP32 Preferences (flash):
- User settings (sensitivity, orientation, balance, etc.)
- Calibration values (gyro bias, standard deviation)
- Connection mode (USB vs Bluetooth)
- WiFi configuration state

Load settings: Line 359
Save settings: Line 406
Reset to defaults: Line 423

## Display Color Codes

The LCD screen uses colors to indicate device state:

- **Blue**: Power on
- **Yellow**: Calibration motion detection
- **Red**: Active calibration
- **Green**: Calibration success / BT paired
- **Orange**: Calibration failed (using old values)
- **Purple**: Clearing Bluetooth pairings
- **Cyan**: Bluetooth advertising
- **Black**: Normal operation (WiFi off) - shows status bar at bottom
- **White text**: WiFi enabled or status bar text

**Status Bar** (bottom of screen, normal operation):
- Shows connection mode: `USB`, `BT`, `BT*` (not connected), or `DBG`
- Shows switch state: `SW:ON` (pressed) or `SW:  ` (not pressed)
- Example: `USB | SW:ON`

## Web Interface Endpoints

- `/` - Main configuration form
- `/status` - JSON-like status information
- `/reset` - Reset all settings to defaults
- `/recalibrate` - Trigger manual recalibration

## Tested Hardware/Software Versions

- Arduino IDE: v2.3.6
- M5Stack board package: v3.2.3
- M5AtomS3 library: v1.0.2
- NimBLE-Arduino library: v2.3.6

## License

GNU General Public License v3.0 - See LICENSE file for full terms.
