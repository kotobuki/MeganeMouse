/*
 * MeganeMouse
 * 
 * A head-controlled mouse device using the M5AtomS3R's built-in gyroscope sensor,
 * intentionally providing only cursor movement functionality (no clicking).
 * 
 * This device was developed in close collaboration with Hideo Arai,
 * based on his search for a "just right" input method while living with ALS.
 * 
 * Key Features:
 * - Dual HID Connection: USB and Bluetooth (with runtime switching)
 * - WiFi Configuration: Web interface for settings, calibration, and adjustments
 * - Adaptive Response: Speed-based filtering and customizable response curves
 * - Flexible Mounting: 5-way orientation and 3D angle correction
 * 
 * Development: Shigeru Kobayashi (+ Claude Code, Gemini)
 * Development Collaboration: Hideo Arai
 * 
 * Copyright (C) 2025 Shigeru Kobayashi
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#define DEBUG_MODE 0  // 0: HID mode, 1: debug mode

#include <M5Unified.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#if DEBUG_MODE == 0
#include <USB.h>
#include <USBHIDMouse.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#endif

#if DEBUG_MODE == 0
USBHIDMouse Mouse;
NimBLEHIDDevice* nimbleHid;
NimBLECharacteristic* nimbleInputMouse;
NimBLEServer* nimbleServer = nullptr;
#endif

// ===== WiFi Configuration =====
const char* AP_SSID = "MeganeMouse-WiFi";
const char* AP_PASSWORD = "MeganeMouse";  // Hardcoded password (minimum 8 characters)
const char* BT_DEVICE_NAME = "MeganeMouse-BT";
WebServer server(80);
Preferences preferences;

// ===== External Switch Configuration =====
const int SWITCH_PIN = 1;  // External switch for left mouse button

// ===== Mathematical Constants =====
#ifndef RAD_TO_DEG
static constexpr float RAD_TO_DEG = 180.0f / M_PI;  // Precise conversion factor
#endif
#ifndef DEG_TO_RAD
static constexpr float DEG_TO_RAD = M_PI / 180.0f;  // For angle conversions
#endif

// ===== Enums =====
enum SensitivityLevel { SENS_LOW,
                        SENS_MEDIUM,
                        SENS_HIGH };
enum DeviceOrientation { LEFT_V,    // Side Left - Battery Vertical
                         LEFT_H,    // Side Left - Battery Horizontal
                         RIGHT_V,   // Side Right - Battery Vertical
                         RIGHT_H,   // Side Right - Battery Horizontal
                         BACK_V };  // Back - Battery Vertical

// ===== Configuration Structure =====
// All adjustable parameters in one place for easy tuning
namespace Config {
// ===== Timing Constants =====
static constexpr int UPDATE_INTERVAL_MS = 20;  // 50Hz
static constexpr unsigned long CALIBRATION_TIMEOUT_MS = 10000;
static constexpr unsigned long BT_RESET_HOLD_DURATION_MS = 3000;
static constexpr unsigned long SERIAL_INIT_DELAY_MS = 1000;
static constexpr unsigned long DEBUG_INTERVAL_MS = 1000;

// ===== Default Values =====
static constexpr float DEFAULT_BASE_K = 0.2f;
static constexpr float DEFAULT_ADAPT_STRENGTH = 0.5f;
static constexpr SensitivityLevel DEFAULT_SENSITIVITY = SENS_MEDIUM;
static constexpr DeviceOrientation DEFAULT_ORIENTATION = LEFT_V;
static constexpr bool DEFAULT_USE_BLUETOOTH_MOUSE = false;
static constexpr float DEFAULT_PITCH_CORRECTION = 0.0f;
static constexpr float DEFAULT_ROLL_CORRECTION = 0.0f;
static constexpr float DEFAULT_YAW_CORRECTION = 0.0f;
static constexpr float DEFAULT_HORIZONTAL_BALANCE = 0.0f;
static constexpr float DEFAULT_VERTICAL_BALANCE = 0.0f;

// ===== Calibration Parameters =====
static constexpr int CALIBRATION_SAMPLES = 64;
static constexpr float CALIBRATION_MAX_STD_DEVIATION = 90.0f;

// ===== Motion Detection Thresholds =====
static constexpr float MOTION_STATIONARY_THRESHOLD = 2.0f;
static constexpr float MOTION_SLOW_THRESHOLD = 10.0f;
static constexpr float MOTION_FAST_THRESHOLD = 50.0f;

// ===== Response Curve Parameters =====
static constexpr float RESPONSE_NORMALIZER = 100.0f;
static constexpr float RESPONSE_VERY_SLOW_THRESHOLD = 5.0f;
static constexpr float RESPONSE_MEDIUM_THRESHOLD = 20.0f;
static constexpr float RESPONSE_MAX_SPEED_FACTOR = 2.0f;

// ===== Sensitivity Divisors =====
static constexpr float SENS_HIGH_DIVISOR = 100.0f;
static constexpr float SENS_MEDIUM_DIVISOR = 200.0f;
static constexpr float SENS_LOW_DIVISOR = 400.0f;

// ===== Input Validation Ranges =====
static constexpr float BASE_K_MIN = 0.1f;
static constexpr float BASE_K_MAX = 2.0f;
static constexpr float ADAPT_STRENGTH_MIN = 0.0f;
static constexpr float ADAPT_STRENGTH_MAX = 2.0f;
static constexpr float MOUNTING_ANGLE_MIN = -45.0f;
static constexpr float MOUNTING_ANGLE_MAX = 45.0f;
static constexpr float BALANCE_MIN = -50.0f;
static constexpr float BALANCE_MAX = 50.0f;

// ===== Motion Limits =====
static constexpr int MAX_MOUSE_SPEED = 100;
};

// Configurable parameters
float baseK = Config::DEFAULT_BASE_K;                  // Base curve shape
float adaptStrength = Config::DEFAULT_ADAPT_STRENGTH;  // Adaptation range
float horizontalBalance = 0.0f;                        // 0% = balanced, >0% = enhance right, <0% = enhance left
float verticalBalance = 0.0f;                          // 0% = balanced, >0% = enhance down, <0% = enhance up
float mountingAngleX = 0.0f;                           // Pitch correction in degrees
float mountingAngleY = 0.0f;                           // Roll correction in degrees
float mountingAngleZ = 0.0f;                           // Yaw correction in degrees

// ===== Data Structures =====
struct GyroCalibration {
  float biasX, biasY, biasZ;        // Average values when stationary
  float stdDevX, stdDevY, stdDevZ;  // Standard deviations
  bool isValid;
};

// ===== Simple Motion Tracker =====
// Tracks motion speed for adaptive response
class SimpleMotionTracker {
private:
  float currentSpeedX;
  float currentSpeedY;
  float smoothedSpeedX;
  float smoothedSpeedY;

  static constexpr float SPEED_SMOOTHING = 0.3f;  // Low-pass filter for speed estimation

public:
  SimpleMotionTracker()
    : currentSpeedX(0), currentSpeedY(0),
      smoothedSpeedX(0), smoothedSpeedY(0) {}

  void updateSpeed(float x, float y) {
    currentSpeedX = abs(x);
    currentSpeedY = abs(y);

    // Smooth the speed estimates to avoid jumpy behavior
    smoothedSpeedX = smoothedSpeedX * (1 - SPEED_SMOOTHING) + currentSpeedX * SPEED_SMOOTHING;
    smoothedSpeedY = smoothedSpeedY * (1 - SPEED_SMOOTHING) + currentSpeedY * SPEED_SMOOTHING;
  }

  float getSpeedX() const {
    return smoothedSpeedX;
  }
  float getSpeedY() const {
    return smoothedSpeedY;
  }

  // Simple motion state detection
  bool isStationary() const {
    return smoothedSpeedX < Config::MOTION_STATIONARY_THRESHOLD && smoothedSpeedY < Config::MOTION_STATIONARY_THRESHOLD;
  }

  bool isSlowMoving() const {
    return (smoothedSpeedX < Config::MOTION_SLOW_THRESHOLD && smoothedSpeedY < Config::MOTION_SLOW_THRESHOLD) && !isStationary();
  }

  bool isFastMoving() const {
    return smoothedSpeedX > Config::MOTION_FAST_THRESHOLD || smoothedSpeedY > Config::MOTION_FAST_THRESHOLD;
  }

  void reset() {
    currentSpeedX = currentSpeedY = 0;
    smoothedSpeedX = smoothedSpeedY = 0;
  }
};

// ===== Speed-based IIR Filter =====
// Dynamic first-order IIR filter that adapts based on input speed
class SpeedBasedIIRFilter {
private:
  float previousOutput;
  float previousInput;
  bool initialized;

  // Filter parameters
  static constexpr float MIN_ALPHA = 0.4f;    // Heavy filtering for slow/stationary
  static constexpr float MAX_ALPHA = 0.9f;    // Light filtering for fast movement
  static constexpr float SPEED_LOW = 2.0f;    // Below this is considered slow
  static constexpr float SPEED_HIGH = 50.0f;  // Above this is considered fast

public:
  SpeedBasedIIRFilter()
    : previousOutput(0), previousInput(0), initialized(false) {}

  float filter(float newValue) {
    if (!initialized) {
      initialized = true;
      previousOutput = newValue;
      previousInput = newValue;
      return newValue;
    }

    // Calculate speed (rate of change)
    float speed = abs(newValue);

    // Calculate alpha based on speed
    // Maps speed to alpha: slow movement = low alpha (more filtering)
    //                     fast movement = high alpha (less filtering)
    float alpha;
    if (speed < SPEED_LOW) {
      alpha = MIN_ALPHA;
    } else if (speed > SPEED_HIGH) {
      alpha = MAX_ALPHA;
    } else {
      // Linear interpolation between MIN and MAX
      float t = (speed - SPEED_LOW) / (SPEED_HIGH - SPEED_LOW);
      alpha = MIN_ALPHA + t * (MAX_ALPHA - MIN_ALPHA);
    }

    // Apply IIR filter: output = α * input + (1-α) * previous_output
    float output = alpha * newValue + (1.0f - alpha) * previousOutput;

    // Update state
    previousOutput = output;
    previousInput = newValue;

    return output;
  }

  void reset() {
    previousOutput = 0;
    previousInput = 0;
    initialized = false;
  }

  float getCurrentOutput() const {
    return previousOutput;
  }
};

// ===== Global Variables =====
GyroCalibration calibration = { 0, 0, 0, 0, 0, 0, false };
SpeedBasedIIRFilter xFilter;
SpeedBasedIIRFilter yFilter;
SimpleMotionTracker motionTracker;

// Settings
SensitivityLevel currentSensitivity = SENS_MEDIUM;
DeviceOrientation currentOrientation = LEFT_V;  // Default to Side Left - Battery Vertical
bool mouseEnabled = true;

// Timing
unsigned long lastUpdateTime = 0;
unsigned long lastDebugTime = 0;

// WiFi control
bool wifiEnabled = false;  // Start with WiFi disabled to save power

// Mouse mode control
bool useBluetoothMouse = false;  // false = USB HID, true = Bluetooth HID (NimBLE)

// External button state
bool switchIsPressed = false;
bool switchWasPressed = false;

// Display update control
unsigned long lastDisplayUpdate = 0;
static constexpr unsigned long DISPLAY_UPDATE_INTERVAL_MS = 100;  // Update display every 100ms

// ===== Mounting Angle Correction =====
// Apply 3D rotation to correct for device mounting angle
void applyMountingCorrection(float& gx, float& gy, float& gz) {
  // Convert mounting angles to radians
  float angleX_rad = mountingAngleX * DEG_TO_RAD;
  float angleY_rad = mountingAngleY * DEG_TO_RAD;
  float angleZ_rad = mountingAngleZ * DEG_TO_RAD;

  // Pre-calculate sin/cos values
  float sx = sin(angleX_rad), cx = cos(angleX_rad);
  float sy = sin(angleY_rad), cy = cos(angleY_rad);
  float sz = sin(angleZ_rad), cz = cos(angleZ_rad);

  // Apply rotation matrices in order: Z (yaw) -> Y (roll) -> X (pitch)
  // This order is chosen for intuitive adjustment:
  // - First correct yaw (rotation around vertical when standing)
  // - Then correct roll (tilt left/right)
  // - Finally correct pitch (tilt forward/back)

  // Store original values
  float gx_orig = gx;
  float gy_orig = gy;
  float gz_orig = gz;

  // Apply Z rotation (yaw correction)
  float gx_z = gx_orig * cz - gy_orig * sz;
  float gy_z = gx_orig * sz + gy_orig * cz;
  float gz_z = gz_orig;

  // Apply Y rotation (roll correction)
  float gx_y = gx_z * cy + gz_z * sy;
  float gy_y = gy_z;
  float gz_y = -gx_z * sy + gz_z * cy;

  // Apply X rotation (pitch correction)
  gx = gx_y;
  gy = gy_y * cx - gz_y * sx;
  gz = gy_y * sx + gz_y * cx;
}

// ===== Simple Response Curve =====
// Quadratic response curve for natural feeling
float applyResponseCurve(float input, float k) {
  // Simple quadratic curve: output = input * (1 + k * |input|)
  // k controls the curvature (0 = linear, higher = more curved)
  float absInput = abs(input);
  float factor = 1.0f + k * absInput / Config::RESPONSE_NORMALIZER;  // Normalize for reasonable range
  return input * factor;
}

// ===== Simple Dynamic Response =====
// Adjust response based on speed (no complex variance calculation)
float calculateDynamicResponse(float speed, float baseK, float adaptStrength) {
  // Simple speed-based adaptation
  // Low speed: reduce k for more linear response
  // High speed: increase k for more aggressive response

  if (speed < Config::RESPONSE_VERY_SLOW_THRESHOLD) {
    // Very slow movement: more linear response
    return baseK * (0.5f + 0.5f * adaptStrength);
  } else if (speed < Config::RESPONSE_MEDIUM_THRESHOLD) {
    // Medium speed: normal response
    return baseK;
  } else {
    // Fast movement: enhanced response
    float speedFactor = min(Config::RESPONSE_MAX_SPEED_FACTOR, speed / Config::RESPONSE_MEDIUM_THRESHOLD);
    return baseK * (1.0f + adaptStrength * (speedFactor - 1.0f) * 0.5f);
  }
}

// ===== Preferences Functions =====
void loadSettings() {
  preferences.begin("meganeMouse", false);

  // Load smooth response parameters
  baseK = preferences.getFloat("baseK", Config::DEFAULT_BASE_K);
  adaptStrength = preferences.getFloat("adaptStr", Config::DEFAULT_ADAPT_STRENGTH);

  // Load mounting angle corrections
  mountingAngleX = preferences.getFloat("mountX", Config::DEFAULT_PITCH_CORRECTION);
  mountingAngleY = preferences.getFloat("mountY", Config::DEFAULT_ROLL_CORRECTION);
  mountingAngleZ = preferences.getFloat("mountZ", Config::DEFAULT_YAW_CORRECTION);

  // Load other settings
  currentSensitivity = (SensitivityLevel)preferences.getInt("sensitivity", (int)Config::DEFAULT_SENSITIVITY);
  currentOrientation = (DeviceOrientation)preferences.getInt("orientation", (int)Config::DEFAULT_ORIENTATION);
  horizontalBalance = preferences.getFloat("hBalance", Config::DEFAULT_HORIZONTAL_BALANCE);
  verticalBalance = preferences.getFloat("vBalance", Config::DEFAULT_VERTICAL_BALANCE);
  useBluetoothMouse = preferences.getBool("btMouse", Config::DEFAULT_USE_BLUETOOTH_MOUSE);

  // Ensure values are within valid ranges
  baseK = constrain(baseK, 0.1f, 2.0f);
  adaptStrength = constrain(adaptStrength, 0.0f, 2.0f);
  mountingAngleX = constrain(mountingAngleX, -45.0f, 45.0f);
  mountingAngleY = constrain(mountingAngleY, -45.0f, 45.0f);
  mountingAngleZ = constrain(mountingAngleZ, -45.0f, 45.0f);
  horizontalBalance = constrain(horizontalBalance, -50.0f, 50.0f);
  verticalBalance = constrain(verticalBalance, -50.0f, 50.0f);
  currentSensitivity = (SensitivityLevel)constrain(currentSensitivity, 0, 2);
  currentOrientation = (DeviceOrientation)constrain(currentOrientation, 0, 4);

  preferences.end();

  Serial.println("Settings loaded from flash:");
  Serial.printf("  Base K: %.2f\n", baseK);
  Serial.printf("  Adapt Strength: %.2f\n", adaptStrength);
  Serial.printf("  Mounting Angles: X=%.1f deg, Y=%.1f deg, Z=%.1f deg\n",
                mountingAngleX, mountingAngleY, mountingAngleZ);
  Serial.printf("  Sensitivity: %s\n",
                currentSensitivity == SENS_HIGH ? "HIGH" : currentSensitivity == SENS_MEDIUM ? "MEDIUM"
                                                                                             : "LOW");
  const char* orientationNames[] = { "Left-V", "Left-H", "Right-V", "Right-H", "Back-V" };
  Serial.printf("  Orientation: %s\n", orientationNames[currentOrientation]);
  Serial.printf("  Horizontal Balance: %.0f%%\n", horizontalBalance);
  Serial.printf("  Vertical Balance: %.0f%%\n", verticalBalance);
  Serial.printf("  Mouse Mode: %s\n", useBluetoothMouse ? "Bluetooth" : "USB HID");
}

void saveSettings() {
  preferences.begin("meganeMouse", false);
  preferences.putFloat("baseK", baseK);
  preferences.putFloat("adaptStr", adaptStrength);
  preferences.putFloat("mountX", mountingAngleX);
  preferences.putFloat("mountY", mountingAngleY);
  preferences.putFloat("mountZ", mountingAngleZ);
  preferences.putFloat("hBalance", horizontalBalance);
  preferences.putFloat("vBalance", verticalBalance);
  preferences.putInt("sensitivity", (int)currentSensitivity);
  preferences.putInt("orientation", (int)currentOrientation);
  preferences.putBool("btMouse", useBluetoothMouse);
  preferences.end();

  Serial.println("Settings saved to flash");
}

void resetSettings() {
  baseK = Config::DEFAULT_BASE_K;
  adaptStrength = Config::DEFAULT_ADAPT_STRENGTH;
  mountingAngleX = 0.0f;
  mountingAngleY = 0.0f;
  mountingAngleZ = 0.0f;
  horizontalBalance = 0.0f;
  verticalBalance = 0.0f;
  currentSensitivity = Config::DEFAULT_SENSITIVITY;
  currentOrientation = LEFT_V;

  saveSettings();
}

// ===== Calibration Support Functions =====
// Load stored calibration values from preferences
void loadStoredCalibration() {
  preferences.begin("meganeMouse", false);
  calibration.biasX = preferences.getFloat("calBiasX", 0.0f);
  calibration.biasY = preferences.getFloat("calBiasY", 0.0f);
  calibration.biasZ = preferences.getFloat("calBiasZ", 0.0f);
  calibration.stdDevX = preferences.getFloat("calStdX", 0.0f);
  calibration.stdDevY = preferences.getFloat("calStdY", 0.0f);
  calibration.stdDevZ = preferences.getFloat("calStdZ", 0.0f);
  calibration.isValid = preferences.getBool("calValid", false);
  preferences.end();

  if (calibration.isValid) {
    Serial.println("Loaded stored calibration values:");
    Serial.printf("  Bias: X=%.2f, Y=%.2f, Z=%.2f\n",
                  calibration.biasX, calibration.biasY, calibration.biasZ);
    Serial.printf("  StdDev: X=%.2f, Y=%.2f, Z=%.2f\n",
                  calibration.stdDevX, calibration.stdDevY, calibration.stdDevZ);
  }
}

// Save calibration values to preferences
void saveCalibrationToPreferences() {
  preferences.begin("meganeMouse", false);
  preferences.putFloat("calBiasX", calibration.biasX);
  preferences.putFloat("calBiasY", calibration.biasY);
  preferences.putFloat("calBiasZ", calibration.biasZ);
  preferences.putFloat("calStdX", calibration.stdDevX);
  preferences.putFloat("calStdY", calibration.stdDevY);
  preferences.putFloat("calStdZ", calibration.stdDevZ);
  preferences.putBool("calValid", calibration.isValid);
  preferences.end();

  Serial.println("Calibration values saved to preferences");
}

// Check if device is still enough for calibration
bool isDeviceStillForCalibration() {
  const int CHECK_SAMPLES = 20;
  const float MAX_MOVEMENT_THRESHOLD = 2.0f;         // rad/s - increased from 0.1 to account for sensor noise
  const float MAX_INDIVIDUAL_AXIS_THRESHOLD = 1.0f;  // rad/s per axis

  float totalMovement = 0;
  float maxAxisMovement = 0;

  for (int i = 0; i < CHECK_SAMPLES; i++) {
    M5.Imu.update();
    auto imuData = M5.Imu.getImuData();

    float gx = abs(imuData.gyro.x);
    float gy = abs(imuData.gyro.y);
    float gz = abs(imuData.gyro.z);

    float movement = gx + gy + gz;
    totalMovement += movement;

    // Track maximum movement on any single axis
    float maxCurrentAxis = max(gx, max(gy, gz));
    maxAxisMovement = max(maxAxisMovement, maxCurrentAxis);

    delay(25);  // 25ms between samples for faster checking
  }

  float averageMovement = totalMovement / CHECK_SAMPLES;

  // Debug output to help tune thresholds
  Serial.printf("Stillness check: avg=%.3f rad/s, max_axis=%.3f rad/s (thresholds: %.1f, %.1f)\n",
                averageMovement, maxAxisMovement, MAX_MOVEMENT_THRESHOLD, MAX_INDIVIDUAL_AXIS_THRESHOLD);

  // Device is still if both average movement and max single-axis movement are below thresholds
  return (averageMovement < MAX_MOVEMENT_THRESHOLD) && (maxAxisMovement < MAX_INDIVIDUAL_AXIS_THRESHOLD);
}

// ===== Enhanced Calibration Function =====
// This function implements enhanced calibration with motion detection and progress display
void performCalibration() {
  Serial.println("\n=== Starting Enhanced Calibration ===");

  // Load any existing calibration values as fallback
  GyroCalibration storedCalibration = calibration;
  loadStoredCalibration();
  GyroCalibration fallbackCalibration = calibration;

  // Reset filters and motion tracker
  xFilter.reset();
  yFilter.reset();
  motionTracker.reset();

  // Step 1: Motion Detection - Wait for device to be still
  Serial.println("Step 1: Waiting for device to be still...");
  M5.Display.fillScreen(YELLOW);
  M5.Display.setTextColor(BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(0, 10);
  M5.Display.printf("CAL.\n");
  M5.Display.printf("Pls. keep\ndev. STILL\n");
  M5.Display.printf("Detecting\nmotion...");
  M5.update();

  bool deviceStill = false;
  int stillnessCheckCount = 0;
  int totalStillnessAttempts = 0;
  const int REQUIRED_STILLNESS_CHECKS = 3;  // Reduced to 3 checks since each check now takes 500ms
  const int MAX_STILLNESS_ATTEMPTS = 10;    // After 10 attempts (5 seconds), skip stillness check

  while (!deviceStill && totalStillnessAttempts < MAX_STILLNESS_ATTEMPTS) {
    totalStillnessAttempts++;

    if (isDeviceStillForCalibration()) {
      stillnessCheckCount++;
      Serial.printf("Stillness check %d/%d passed\n", stillnessCheckCount, REQUIRED_STILLNESS_CHECKS);

      if (stillnessCheckCount >= REQUIRED_STILLNESS_CHECKS) {
        deviceStill = true;
      }
    } else {
      stillnessCheckCount = 0;  // Reset if movement detected
      Serial.printf("Movement detected, restarting stillness check (attempt %d/%d)\n",
                    totalStillnessAttempts, MAX_STILLNESS_ATTEMPTS);
    }

    // Update display with progress
    M5.Display.fillScreen(YELLOW);
    M5.Display.setTextColor(BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(0, 10);
    M5.Display.printf("CAL.\n");
    M5.Display.printf("Keep STILL\n");
    M5.Display.printf("Progress:\n%d/%d", stillnessCheckCount, REQUIRED_STILLNESS_CHECKS);
    // Show auto-skip countdown when past halfway point to prepare user for timeout
    if (totalStillnessAttempts > MAX_STILLNESS_ATTEMPTS / 2) {
      M5.Display.printf("\nAuto-skip in %d", MAX_STILLNESS_ATTEMPTS - totalStillnessAttempts);
    }
    M5.update();

    delay(500);
  }

  if (totalStillnessAttempts >= MAX_STILLNESS_ATTEMPTS) {
    Serial.println("Stillness detection timeout - proceeding with calibration anyway");
    M5.Display.fillScreen(ORANGE);
    M5.Display.setTextColor(BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(0, 10);
    M5.Display.printf("SENSOR\nNOISY\n");
    M5.Display.printf("Proceeding\nanyway");
    M5.update();
    delay(1000);
  }

  // Step 2: Perform Calibration with Progress Display
  Serial.println("Step 2: Performing calibration...");
  M5.Display.fillScreen(RED);
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(0, 10);
  M5.Display.printf("KEEP STILL\nCal...\n");
  M5.update();

  unsigned long startTime = millis();
  bool calibrationSuccessful = false;
  int attemptCount = 0;
  const int MAX_ATTEMPTS = 3;

  // Keep trying until we get stable readings, reach max attempts, or timeout
  while (!calibrationSuccessful && attemptCount < MAX_ATTEMPTS && (millis() - startTime < Config::CALIBRATION_TIMEOUT_MS)) {

    attemptCount++;
    Serial.printf("Calibration attempt %d/%d\n", attemptCount, MAX_ATTEMPTS);

    // Statistical variables for calculating mean and standard deviation
    double sumX = 0, sumY = 0, sumZ = 0;
    double sumX2 = 0, sumY2 = 0, sumZ2 = 0;

    // Collect samples with progress display
    for (int i = 0; i < Config::CALIBRATION_SAMPLES; i++) {
      M5.Imu.update();
      auto imuData = M5.Imu.getImuData();

      float gx = imuData.gyro.x;
      float gy = imuData.gyro.y;
      float gz = imuData.gyro.z;

      // Accumulate for mean calculation
      sumX += gx;
      sumY += gy;
      sumZ += gz;

      // Accumulate squares for standard deviation calculation
      sumX2 += gx * gx;
      sumY2 += gy * gy;
      sumZ2 += gz * gz;

      // Update progress display every 8 samples
      if (i % 8 == 0) {
        int progress = (i * 100) / Config::CALIBRATION_SAMPLES;
        M5.Display.fillScreen(RED);
        M5.Display.setTextColor(WHITE);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(0, 10);
        M5.Display.printf("KEEP STILL\nCal.ing...\n");
        M5.Display.printf("Try %d/%d\n", attemptCount, MAX_ATTEMPTS);
        M5.Display.printf("Done: %d%%", progress);
        M5.update();
      }

      delay(5);  // Small delay between samples
    }

    // Calculate means (these will be our bias values)
    float meanX = sumX / Config::CALIBRATION_SAMPLES;
    float meanY = sumY / Config::CALIBRATION_SAMPLES;
    float meanZ = sumZ / Config::CALIBRATION_SAMPLES;

    // Calculate standard deviations
    // Formula: sqrt(E[X²] - E[X]²)
    float stdDevX = sqrt(sumX2 / Config::CALIBRATION_SAMPLES - meanX * meanX);
    float stdDevY = sqrt(sumY2 / Config::CALIBRATION_SAMPLES - meanY * meanY);
    float stdDevZ = sqrt(sumZ2 / Config::CALIBRATION_SAMPLES - meanZ * meanZ);

    // Check if the device was really stationary
    // Low standard deviation means the readings were consistent
    if (stdDevX < Config::CALIBRATION_MAX_STD_DEVIATION && stdDevY < Config::CALIBRATION_MAX_STD_DEVIATION && stdDevZ < Config::CALIBRATION_MAX_STD_DEVIATION) {

      // Save calibration data
      calibration.biasX = meanX;
      calibration.biasY = meanY;
      calibration.biasZ = meanZ;
      calibration.stdDevX = stdDevX;
      calibration.stdDevY = stdDevY;
      calibration.stdDevZ = stdDevZ;
      calibration.isValid = true;
      calibrationSuccessful = true;

      // Save to preferences
      saveCalibrationToPreferences();

      Serial.println("Calibration successful!");
      Serial.printf("New Bias: X=%.2f, Y=%.2f, Z=%.2f\n",
                    calibration.biasX, calibration.biasY, calibration.biasZ);
      Serial.printf("StdDev: X=%.2f, Y=%.2f, Z=%.2f\n",
                    calibration.stdDevX, calibration.stdDevY, calibration.stdDevZ);
    } else {
      Serial.printf("Movement detected during calibration (StdDev: X=%.2f, Y=%.2f, Z=%.2f)\n",
                    stdDevX, stdDevY, stdDevZ);

      // Show movement detected message
      M5.Display.fillScreen(ORANGE);
      M5.Display.setTextColor(BLACK);
      M5.Display.setTextSize(2);
      M5.Display.setCursor(0, 10);
      M5.Display.printf("MOVEMENT\nDETECTED\n");
      M5.Display.printf("Attempt %d/%d\nfailed", attemptCount, MAX_ATTEMPTS);
      M5.update();
      delay(1000);
    }
  }

  // Step 3: Handle results and fallback
  if (!calibrationSuccessful) {
    Serial.println("Calibration failed after maximum attempts");

    // Use fallback calibration if available
    if (fallbackCalibration.isValid) {
      calibration = fallbackCalibration;
      Serial.println("Using stored calibration values as fallback");
      Serial.printf("Fallback Bias: X=%.2f, Y=%.2f, Z=%.2f\n",
                    calibration.biasX, calibration.biasY, calibration.biasZ);
    } else {
      Serial.println("No stored calibration available - using zero bias");
      calibration.biasX = 0;
      calibration.biasY = 0;
      calibration.biasZ = 0;
      calibration.isValid = false;
    }
  }

  // Step 4: Visual feedback - green for success, orange for fallback, red for failure
  if (calibrationSuccessful) {
    M5.Display.fillScreen(GREEN);
    M5.Display.setTextColor(BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(0, 20);
    M5.Display.printf("CAL.\nSUCCESSFUL");
    M5.update();
    Serial.println("Calibration completed successfully");
  } else if (fallbackCalibration.isValid) {
    M5.Display.fillScreen(ORANGE);
    M5.Display.setTextColor(BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(0, 10);
    M5.Display.printf("CAL.\nFAILED\n");
    M5.Display.printf("Using old\nvalues");
    M5.update();
    Serial.println("Using fallback calibration");
  } else {
    M5.Display.fillScreen(RED);
    M5.Display.setTextColor(WHITE);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(0, 10);
    M5.Display.printf("CAL.\nFAILED\n");
    M5.Display.printf("No backup\navailable");
    M5.update();
    Serial.println("Calibration failed completely");
  }

  delay(2000);  // Show result for 2 seconds
  M5.Display.fillScreen(BLACK);
  M5.update();
}

// ===== Web Server Handlers =====
void handleRoot() {
  if (server.method() == HTTP_POST) {
    // Process form submission
    if (server.hasArg("baseK") && server.hasArg("adaptStr") && server.hasArg("sensitivity")) {
      float newBaseK = server.arg("baseK").toFloat();
      float newAdaptStr = server.arg("adaptStr").toFloat();
      String sensStr = server.arg("sensitivity");
      float newHBalance = server.arg("hBalance").toFloat();
      float newVBalance = server.arg("vBalance").toFloat();
      float newMountX = server.arg("mountX").toFloat();
      float newMountY = server.arg("mountY").toFloat();
      float newMountZ = server.arg("mountZ").toFloat();

      // Validate and constrain values
      baseK = constrain(newBaseK, Config::BASE_K_MIN, Config::BASE_K_MAX);
      adaptStrength = constrain(newAdaptStr, Config::ADAPT_STRENGTH_MIN, Config::ADAPT_STRENGTH_MAX);
      horizontalBalance = constrain(newHBalance, Config::BALANCE_MIN, Config::BALANCE_MAX);
      verticalBalance = constrain(newVBalance, Config::BALANCE_MIN, Config::BALANCE_MAX);
      mountingAngleX = constrain(newMountX, Config::MOUNTING_ANGLE_MIN, Config::MOUNTING_ANGLE_MAX);
      mountingAngleY = constrain(newMountY, Config::MOUNTING_ANGLE_MIN, Config::MOUNTING_ANGLE_MAX);
      mountingAngleZ = constrain(newMountZ, Config::MOUNTING_ANGLE_MIN, Config::MOUNTING_ANGLE_MAX);

      if (sensStr == "low") currentSensitivity = SENS_LOW;
      else if (sensStr == "medium") currentSensitivity = SENS_MEDIUM;
      else if (sensStr == "high") currentSensitivity = SENS_HIGH;

      // Process orientation setting
      if (server.hasArg("orientation")) {
        String orientStr = server.arg("orientation");
        if (orientStr == "left-v") currentOrientation = LEFT_V;
        else if (orientStr == "left-h") currentOrientation = LEFT_H;
        else if (orientStr == "right-v") currentOrientation = RIGHT_V;
        else if (orientStr == "right-h") currentOrientation = RIGHT_H;
        else if (orientStr == "back-v") currentOrientation = BACK_V;
      }

      // Check for mouse mode change
      bool modeChanged = false;
      if (server.hasArg("connectionMode")) {
        bool newBluetoothMode = (server.arg("connectionMode") == "bluetooth");
        if (newBluetoothMode != useBluetoothMouse) {
          useBluetoothMouse = newBluetoothMode;
          modeChanged = true;
        }
      }

      saveSettings();

      // Handle mode change (requires restart)
      if (modeChanged) {
        showModeChangeMessage(useBluetoothMouse);
        ESP.restart();
      }

      // Redirect to avoid form resubmission
      server.sendHeader("Location", "/");
      server.send(303);
      return;
    }
  }

  // Generate HTML form with minimal mobile-friendly CSS
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='utf-8' name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>MeganeMouse Settings</title>";
  html += "<style>";
  html += "*{box-sizing:border-box;}";
  html += "body{font-family:sans-serif;font-size:18px;margin:20px;padding:0;line-height:1.6;}";
  html += "input[type=number]{width:100%;height:44px;font-size:18px;margin:5px 0;padding:0 10px;}";
  html += "input[type=radio]{width:20px;height:20px;margin-right:10px;}";
  html += "input[type=submit]{width:100%;height:50px;font-size:20px;margin:20px 0;}";
  html += "label{font-size:18px;}";
  html += "a{display:block;padding:10px 0;font-size:18px;}";
  html += ".info{font-size:14px;color:#666;margin:5px 0;}";
  html += ".section{border-top:2px solid #ddd;margin-top:20px;padding-top:15px;}";
  html += "</style>";
  html += "</head><body>";
  html += "<h1>Settings | 設定</h1>";
  html += "<p><b>Note:</b> Scroll to the bottom of the page to save changes with the <b>Apply Settings</b> button.</p>";
  html += "<p><b>ご注意：</b>設定の保存は、ページ最下部の<b>設定を反映</b>ボタンで行ってください。</p>";
  html += "<form method='post'>";

  html += "<div class='section'><h2>Sensitivity | 感度</h2>";
  html += "<p><label><input type='radio' name='sensitivity' value='low'";
  if (currentSensitivity == SENS_LOW) html += " checked";
  html += ">Low | 低</label></p>";

  html += "<p><label><input type='radio' name='sensitivity' value='medium'";
  if (currentSensitivity == SENS_MEDIUM) html += " checked";
  html += ">Medium | 中</label></p>";

  html += "<p><label><input type='radio' name='sensitivity' value='high'";
  if (currentSensitivity == SENS_HIGH) html += " checked";
  html += ">High | 高</label></p>";
  html += "</div>";

  html += "<div class='section'><h2>Device Orientation | デバイス方向</h2>";
  // html += "<p>Mounting Position | 取り付け位置:<br>";
  html += "<select name='orientation' style='width:100%;height:44px;font-size:18px;'>";

  html += "<option value='left-v'";
  if (currentOrientation == LEFT_V) html += " selected";
  html += ">Left-V | 左・ポート下向き</option>";

  html += "<option value='left-h'";
  if (currentOrientation == LEFT_H) html += " selected";
  html += ">Left-H | 左・ポート後ろ向き</option>";

  html += "<option value='right-v'";
  if (currentOrientation == RIGHT_V) html += " selected";
  html += ">Right-V | 右・ポート下向き</option>";

  html += "<option value='right-h'";
  if (currentOrientation == RIGHT_H) html += " selected";
  html += ">Right-H | 右・ポート後ろ向き</option>";

  html += "<option value='back-v'";
  if (currentOrientation == BACK_V) html += " selected";
  html += ">Back-V | 後・ポート下向き</option>";

  html += "</select></p>";
  html += "<span class='info'>V: USB-C port points down</span><br>";
  html += "<span class='info'>H: USB-C port points backward</span><br>";
  html += "<span class='info'>V: USB-Cポートが下向き</span><br>";
  html += "<span class='info'>H: USB-Cポートが後ろ向き</span>";
  html += "</div>";

  html += "<div class='section'><h2>Connection Mode | 接続モード</h2>";
  html += "<p><label><input type='radio' name='connectionMode' value='usb'";
  if (!useBluetoothMouse) html += " checked";
  html += ">USB HID</label></p>";

  html += "<p><label><input type='radio' name='connectionMode' value='bluetooth'";
  if (useBluetoothMouse) html += " checked";
  html += ">Bluetooth HID</label></p>";
  html += "<span class='info'>Changing the mode will cause a restart when the settings are applied.</span><br>";
  html += "<span class='info'>モードを変更すると設定反映時に再起動します。</span>";
  html += "</div>";

  html += "<div class='section'><h2>Response Curve</h2>";
  html += "<span class='info'>Adjusts sensitivity based on movement speed for natural control.</span><br>";
  html += "<span class='info'>移動速度に応じて感度を調整し、自然な操作性を実現します。</span>";
  html += "<p>Base Curve Shape (0.1-2.0):<br>";
  html += "<input type='number' name='baseK' value='" + String(baseK, 1) + "' min='0.1' max='2.0' step='0.1'></p>";
  html += "<span class='info'>Controls acceleration. Higher = more acceleration (less linear). Default: 0.2</span><br>";
  html += "<span class='info'>加速の度合い。値が大きいほど加速が強くなります。初期値: 0.2</span></p>";

  html += "<p>Adaptation Strength (0.0-2.0):<br>";
  html += "<input type='number' name='adaptStr' value='" + String(adaptStrength, 1) + "' min='0.0' max='2.0' step='0.1'></p>";
  html += "<span class='info'>How much the curve adapts to movement speed. 0 = no adaptation. Default: 0.5</span><br>";
  html += "<span class='info'>移動速度への適応度。0 = 適応なし。初期値: 0.5</span></p>";

  html += "<div class='section'><h2>Mounting Angle Correction</h2>";
  html += "<span class='info'>If the device is not perfectly aligned when mounted on your glasses, use these settings to compensate.</span><br>";
  html += "<span class='info'>デバイスがメガネに正しく取り付けられていない場合、ここで補正します。</span>";

  html += "<p>Pitch Correction (°):<br>";
  html += "<input type='number' name='mountX' value='" + String((int)round(mountingAngleX)) + "' min='-45' max='45' step='1'>";
  html += "<span class='info'>Compensates for forward/backward tilt. + = tilted backward, - = tilted forward</span><br>";
  html += "<span class='info'>前後の傾きを補正。+ = 後、- = 前</span></p>";

  html += "<p>Roll Correction (°):<br>";
  html += "<input type='number' name='mountY' value='" + String((int)round(mountingAngleY)) + "' min='-45' max='45' step='1'>";
  html += "<span class='info'>Compensates for left/right tilt. + = tilted right, - = tilted left</span><br>";
  html += "<span class='info'>左右の傾きを補正。+ = 右、- = 左</span></p>";

  html += "<p>Yaw Correction (°):<br>";
  html += "<input type='number' name='mountZ' value='" + String((int)round(mountingAngleZ)) + "' min='-45' max='45' step='1'>";
  html += "<span class='info'>Compensates for rotation. + = rotated clockwise, - = rotated counter-clockwise</span><br>";
  html += "<span class='info'>回転のずれを補正。+ = 時計回り、- = 反時計回り</span></p>";
  html += "</div>";

  html += "<div class='section'><h2>Movement Balance</h2>";
  html += "<span class='info'>Fine-tune movement sensitivity in each direction.</span><br>";
  html += "<span class='info'>各方向の動きの感度を微調整します。</span>";
  html += "<p>Horizontal Balance (%):<br>";
  html += "<input type='number' name='hBalance' value='" + String((int)round(horizontalBalance)) + "' min='-50' max='50' step='1'>";
  html += "<span class='info'>0: balanced L/R, +: enhance right, -: enhance left</span><br>";
  html += "<span class='info'>0: 左右均等, +: 右を強める, -: 左を強める</span></p>";

  html += "<p>Vertical Balance (%):<br>";
  html += "<input type='number' name='vBalance' value='" + String((int)round(verticalBalance)) + "' min='-50' max='50' step='1'>";
  html += "<span class='info'>0: balanced U/D, +: enhance down, -: enhance up</span><br>";
  html += "<span class='info'>0: 上下均等, +: 下を強める, -: 上を強める</span></p>";
  html += "</div>";

  html += "<input type='submit' value='Apply Settings | 設定を反映'>";
  html += "</form>";

  html += "<hr>";
  html += "<a href='/status'>View Current Status | 状態と動作情報を表示</a>";
  html += "<a href='/reset'>Reset to Defaults | 初期値に戻す</a>";
  html += "<a href='/recalibrate'>Recalibrate | 再校正する</a>";

  html += "</body></html>";

  // Add cache-control headers to prevent browser caching
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.send(200, "text/html", html);
}

void handleStatus() {
  String status = "MeganeMouse Status\n\n";
  status += "Current Settings:\n";
  status += "  Base K: " + String(baseK, 2) + "\n";
  status += "  Adapt Strength: " + String(adaptStrength, 2) + "\n";
  status += "  Sensitivity: ";
  status += (currentSensitivity == SENS_HIGH ? "HIGH" : currentSensitivity == SENS_MEDIUM ? "MEDIUM"
                                                                                          : "LOW");
  status += "\n";
  status += "  Horizontal Balance: " + String(horizontalBalance, 0) + "%\n";
  status += "  Vertical Balance: " + String(verticalBalance, 0) + "%\n";
  status += "  Mounting Angles: X=" + String(mountingAngleX, 0) + " deg, Y=" + String(mountingAngleY, 0) + " deg, Z=" + String(mountingAngleZ, 0) + " deg\n\n";

  status += "Motion State:\n";
  status += "  Speed X: " + String(motionTracker.getSpeedX(), 1) + " deg/s\n";
  status += "  Speed Y: " + String(motionTracker.getSpeedY(), 1) + " deg/s\n";
  status += "  State: ";
  if (motionTracker.isStationary()) status += "Stationary";
  else if (motionTracker.isSlowMoving()) status += "Slow Moving";
  else if (motionTracker.isFastMoving()) status += "Fast Moving";
  else status += "Normal Moving";
  status += "\n\n";

  status += "Calibration Status:\n";
  status += "  Valid: " + String(calibration.isValid ? "Yes" : "No") + "\n";
  if (calibration.isValid) {
    status += "  Bias X: " + String(calibration.biasX, 2) + "\n";
    status += "  Bias Y: " + String(calibration.biasY, 2) + "\n";
    status += "  Bias Z: " + String(calibration.biasZ, 2) + "\n";
    status += "  StdDev X: " + String(calibration.stdDevX, 2) + "\n";
    status += "  StdDev Y: " + String(calibration.stdDevY, 2) + "\n";
    status += "  StdDev Z: " + String(calibration.stdDevZ, 2) + "\n";
  }

  server.send(200, "text/plain", status);
}

void handleReset() {
  // Reset to default values
  resetSettings();

  // Reset filters and motion tracker
  xFilter.reset();
  yFilter.reset();
  motionTracker.reset();

  // Redirect to main page to show updated values
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleRecalibrate() {
  // Reset filters and motion tracker before calibration
  xFilter.reset();
  yFilter.reset();
  motionTracker.reset();

  // Perform calibration
  performCalibration();

  server.send(200, "text/plain", "Recalibration completed");
}

void handleNotFound() {
  server.send(404, "text/plain", "404: Not Found");
}

// ===== WiFi Control Functions =====
void enableWiFi() {
  if (!wifiEnabled) {
    Serial.println("Enabling WiFi...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    Serial.printf("SSID: %s\n", AP_SSID);
    Serial.printf("Password: %s\n", AP_PASSWORD);

    // Setup and start web server
    server.on("/", handleRoot);
    server.on("/status", handleStatus);
    server.on("/reset", handleReset);
    server.on("/recalibrate", handleRecalibrate);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("Web server started");

    wifiEnabled = true;

    // Display WiFi information on LCD (keep showing)
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextColor(WHITE);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(0, 10);
    M5.Display.printf("WiFi: ON\n");
    M5.Display.setTextSize(1);
    M5.Display.setCursor(0, 30);
    M5.Display.printf("SSID:\n");
    M5.Display.printf("%s\n", AP_SSID);
    M5.Display.printf("Pass: %s\n", AP_PASSWORD);
    M5.Display.printf("Web: %s\n", IP.toString().c_str());
    M5.update();
  }
}

void disableWiFi() {
  if (wifiEnabled) {
    Serial.println("Disabling WiFi...");
    server.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    wifiEnabled = false;

    // Clear display and show WiFi off status
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextColor(WHITE);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(0, 10);
    M5.Display.printf("WiFi: OFF");
    M5.update();
    delay(1000);  // Show for 1 second

    // Restore status bar
    updateStatusBar();
  }
}

void toggleWiFi() {
  if (wifiEnabled) {
    disableWiFi();
  } else {
    enableWiFi();
  }
}

void showModeChangeMessage(bool newBluetoothMode) {
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(0, 10);
  M5.Display.printf("Switch to\n");
  M5.Display.printf("%s mode\n", newBluetoothMode ? "Bluetooth" : "USB HID");
  M5.Display.printf("Restarting...");
  M5.update();
  delay(2000);  // Show message for 2 seconds
}

// ===== Status Bar Display =====
void updateStatusBar() {
  // Skip if WiFi is enabled (WiFi screen takes priority)
  if (wifiEnabled) {
    return;
  }

  // Clear the screen and draw status bar at bottom
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(WHITE);

  // Prepare status text
  String statusText = "";

#if DEBUG_MODE == 0
  // Show connection mode
  if (useBluetoothMouse) {
    #if defined(nimbleConnected)
    if (nimbleConnected) {
      statusText = "BT";
    } else {
      statusText = "BT*";  // * indicates not connected
    }
    #else
    statusText = "BT";
    #endif
  } else {
    statusText = "USB";
  }
#else
  statusText = "DBG";  // Debug mode
#endif

  // Add switch state
  statusText += " | SW:";
  statusText += switchIsPressed ? "ON" : "  ";  // ON when pressed, blank when not

  // Calculate centered position (128 pixel width, each char is ~6 pixels wide)
  int textWidth = statusText.length() * 6;
  int xPos = (128 - textWidth) / 2;

  // Position at bottom (128 pixel height, leave 2 pixels from bottom)
  int yPos = 128 - 8 - 2;

  M5.Display.setCursor(xPos, yPos);
  M5.Display.print(statusText);
  M5.update();
}

#if DEBUG_MODE == 0
// NimBLE connection tracking
bool nimbleConnected = false;

// ===== Bluetooth Pairing Reset Function =====
void clearBluetoothPairings() {
  Serial.println("Clearing all Bluetooth pairings...");

  // Show clearing message on LCD
  M5.Display.fillScreen(PURPLE);
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(0, 10);
  M5.Display.printf("CLEARING\nBT PAIRS\n");
  M5.Display.printf("Please\nwait...");
  M5.update();

  // Clear NimBLE bonding information
  int dev_num = NimBLEDevice::getNumBonds();
  Serial.printf("Found %d bonded devices to clear\n", dev_num);

  if (dev_num > 0) {
    NimBLEDevice::deleteAllBonds();
    Serial.println("All Bluetooth pairings cleared");
  } else {
    Serial.println("No Bluetooth pairings found to clear");
  }

  // Show completion message
  M5.Display.fillScreen(GREEN);
  M5.Display.setTextColor(BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(0, 10);
  M5.Display.printf("BT PAIRS\nCLEARED\n");
  M5.Display.printf("Release\nbutton");
  M5.update();

  // Wait for button release
  while (M5.BtnA.isPressed()) {
    M5.update();
    delay(50);
  }

  delay(1000);  // Show message for 1 second after button release
  M5.Display.fillScreen(BLACK);
  M5.update();

  Serial.println("Bluetooth pairing reset completed");
}

class MeganeMouseServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
    nimbleConnected = true;
    Serial.printf("NimBLE client connected: %s\n", connInfo.getAddress().toString().c_str());

    // Show connection status on LCD
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextColor(GREEN);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(0, 10);
    M5.Display.printf("Bluetooth\nCONNECTED\n");
    M5.Display.printf("Mouse: BT\n");
    M5.update();
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) {
    nimbleConnected = false;
    Serial.printf("NimBLE client disconnected: %s (reason: %d)\n",
                  connInfo.getAddress().toString().c_str(), reason);

    // Show disconnection status on LCD
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextColor(YELLOW);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(0, 10);
    M5.Display.printf("Bluetooth\nDISCONN'D\n");
    M5.Display.printf("Mouse: BT\n");
    M5.update();

    // Restart advertising
    delay(1000);
    NimBLEDevice::startAdvertising();
    Serial.println("NimBLE advertising restarted");
  }
};

void initBluetoothMouse() {
  NimBLEDevice::init(BT_DEVICE_NAME);
  nimbleServer = NimBLEDevice::createServer();
  nimbleServer->setCallbacks(new MeganeMouseServerCallbacks());

  // Set security settings for better compatibility
  NimBLEDevice::setSecurityAuth(true, false, true);

  nimbleHid = new NimBLEHIDDevice(nimbleServer);
  nimbleInputMouse = nimbleHid->getInputReport(1);  // Report ID 1 for proper HID protocol

  // NimBLE HID mouse descriptor with X/Y before buttons (proven working format)
  const uint8_t reportMap[] = {
    0x05, 0x01,  // UsagePage(Generic Desktop[1])
    0x09, 0x02,  // UsageId(Mouse[2])
    0xA1, 0x01,  // Collection(Application)
    0x85, 0x01,  //     ReportId(1)
    0x09, 0x01,  //     UsageId(Pointer[1])
    0xA1, 0x00,  //     Collection(Physical)
    0x09, 0x30,  //         UsageId(X[48])
    0x09, 0x31,  //         UsageId(Y[49])
    0x15, 0x80,  //         LogicalMinimum(-128)
    0x25, 0x7F,  //         LogicalMaximum(127)
    0x95, 0x02,  //         ReportCount(2)
    0x75, 0x08,  //         ReportSize(8)
    0x81, 0x06,  //         Input(Data, Variable, Relative)
    0x05, 0x09,  //         UsagePage(Button[9])
    0x19, 0x01,  //         UsageIdMin(Button 1[1])
    0x29, 0x03,  //         UsageIdMax(Button 3[3])
    0x15, 0x00,  //         LogicalMinimum(0)
    0x25, 0x01,  //         LogicalMaximum(1)
    0x95, 0x03,  //         ReportCount(3)
    0x75, 0x01,  //         ReportSize(1)
    0x81, 0x02,  //         Input(Data, Variable, Absolute)
    0xC0,        //     EndCollection()
    0x95, 0x01,  //     ReportCount(1)
    0x75, 0x05,  //     ReportSize(5)
    0x81, 0x03,  //     Input(Constant, Variable, Absolute)
    0xC0,        // EndCollection()
  };

  nimbleHid->setReportMap((uint8_t*)reportMap, sizeof(reportMap));
  nimbleHid->startServices();

  // Set device info for better OS compatibility
  nimbleHid->setManufacturer("M5Stack");
  nimbleHid->setPnp(0x02, 0x05ac, 0x820a, 0x0110);
  nimbleHid->setHidInfo(0x01, 0x11);  // HID version 1.11

  // Start advertising
  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->setName(BT_DEVICE_NAME);
  advertising->addServiceUUID(nimbleHid->getHidService()->getUUID());
  advertising->setAppearance(0x03C2);  // HID Mouse appearance
  advertising->start();

  nimbleHid->setBatteryLevel(100);

  Serial.println("NimBLE HID mouse initialized");
  Serial.println("Waiting for Bluetooth connection...");

  // Show advertising status on LCD
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextColor(CYAN);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(0, 10);
  M5.Display.printf("Bluetooth\nADVERTISIN\n");
  M5.Display.printf("MeganeMouse-BT\n");
  M5.Display.printf("Ready pair");
  M5.update();
}

void sendBluetoothMouseMove(int x, int y, bool leftButton) {
  if (nimbleHid != nullptr && nimbleInputMouse != nullptr && nimbleConnected) {
    // Create 3-byte NimBLE mouse report: [x, y, buttons]
    int8_t report[3];
    report[0] = (int8_t)constrain(x, -128, 127);  // X movement
    report[1] = (int8_t)constrain(y, -128, 127);  // Y movement
    report[2] = leftButton ? 0x01 : 0x00;         // Bit 0 = left button

    // Debug output
    if (x != 0 || y != 0 || leftButton) {
      Serial.printf("NimBLE Mouse: X=%d, Y=%d, Btn=%d, Report=[%02X %02X %02X]\n",
                    x, y, leftButton, report[0], report[1], report[2]);
    }

    nimbleInputMouse->setValue((uint8_t*)report, sizeof(report));
    nimbleInputMouse->notify();
  } else {
    if (x != 0 || y != 0 || leftButton) {
      Serial.printf("NimBLE Mouse blocked: hid=%p, input=%p, connected=%d\n",
                    nimbleHid, nimbleInputMouse, nimbleConnected);
    }
  }
}
#endif

// ===== Apply Sensitivity Scaling =====
int applySensitivity(float value) {
  float divisor;

  switch (currentSensitivity) {
    case SENS_HIGH:
      divisor = Config::SENS_HIGH_DIVISOR;
      break;
    case SENS_MEDIUM:
      divisor = Config::SENS_MEDIUM_DIVISOR;
      break;
    case SENS_LOW:
      divisor = Config::SENS_LOW_DIVISOR;
      break;
  }

  // Divide by sensitivity factor (larger divisor = lower sensitivity)
  return round(value / divisor);
}

// ===== Apply Balance Adjustment =====
int applyBalance(int value, float balance) {
  // balance: -50 to +50
  // 0: no adjustment
  // positive: enhance positive direction movement
  // negative: enhance negative direction movement

  if (value == 0) return 0;

  // Calculate balance factor (0% centered, adjusted based on 100)
  float factor = (100.0f + balance) / 100.0f;

  // Horizontal: positive = right, negative = left
  // Vertical: positive = down, negative = up
  if ((value > 0 && balance > 0) || (value < 0 && balance < 0)) {
    // Same sign: enhance movement
    return round(value * factor);
  } else if ((value > 0 && balance < 0) || (value < 0 && balance > 0)) {
    // Opposite sign: reduce movement
    return round(value / factor);
  }

  return value;
}

// ===== Setup Function =====
void setup() {
  // Initialize M5AtomS3R
  auto cfg = M5.config();
  cfg.internal_imu = true;
  M5.begin(cfg);

  Serial.begin(115200);
  delay(Config::SERIAL_INIT_DELAY_MS);  // Give serial time to initialize

  Serial.println("\n========================================");
  Serial.println("MeganeMouse - USB & Bluetooth");
  Serial.println("========================================\n");

  // Initialize external switch pin
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  Serial.printf("External switch initialized on pin %d\n", SWITCH_PIN);

  // Check for Bluetooth pairing reset (button held on boot)
  M5.update();  // Update button state
  if (M5.BtnA.isPressed()) {
    Serial.println("Button A pressed on boot - checking for Bluetooth pairing reset...");

    // Show initial message
    M5.Display.fillScreen(ORANGE);
    M5.Display.setTextColor(BLACK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(0, 10);
    M5.Display.printf("Hold btn\nto clear\nBT pairs");
    M5.update();

    // Wait for button to be held for 3 seconds
    unsigned long startTime = millis();
    bool buttonHeld = true;

    while (millis() - startTime < Config::BT_RESET_HOLD_DURATION_MS && buttonHeld) {
      M5.update();
      if (!M5.BtnA.isPressed()) {
        buttonHeld = false;
      }
      delay(50);
    }

    if (buttonHeld && M5.BtnA.isPressed()) {
      // Button held for 3+ seconds - clear Bluetooth pairings
#if DEBUG_MODE == 0
      // Load settings first to check if in Bluetooth mode
      loadSettings();

      if (useBluetoothMouse) {
        // Initialize NimBLE first to access bonding functions
        NimBLEDevice::init(BT_DEVICE_NAME);
        clearBluetoothPairings();
        // Deinitialize to restart fresh later
        NimBLEDevice::deinit(true);
      } else {
        // Show message that BT reset is only available in BT mode
        M5.Display.fillScreen(RED);
        M5.Display.setTextColor(WHITE);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(0, 10);
        M5.Display.printf("BT RESET\nONLY in\nBT MODE");
        M5.update();
        delay(2000);
        M5.Display.fillScreen(BLACK);
        M5.update();
      }
#else
      // In debug mode, show that BT reset is not available
      M5.Display.fillScreen(RED);
      M5.Display.setTextColor(WHITE);
      M5.Display.setTextSize(2);
      M5.Display.setCursor(0, 10);
      M5.Display.printf("BT RESET\nN/A in\nDEBUG");
      M5.update();
      delay(2000);
      M5.Display.fillScreen(BLACK);
      M5.update();
#endif
    } else {
      // Button not held long enough
      Serial.println("Button not held long enough for Bluetooth reset");
      M5.Display.fillScreen(BLACK);
      M5.update();
    }
  }

  // Load saved settings
  loadSettings();

  // Load any stored calibration values before performing new calibration
  loadStoredCalibration();

  // Initialize mouse based on saved mode
#if DEBUG_MODE == 0
  if (useBluetoothMouse) {
    initBluetoothMouse();
  } else {
    USB.begin();
    Mouse.begin();
  }
#endif

  // WiFi is disabled by default to save power
  // Long press button to enable WiFi configuration
  Serial.println("WiFi disabled by default (long press button to enable)");

  // Initial visual feedback
  M5.Display.fillScreen(BLUE);
  M5.update();
  delay(500);

  // Perform initial calibration
  performCalibration();

  // Display initial status bar
  updateStatusBar();

  Serial.println("\nReady!");
  Serial.println("Controls:");
  Serial.println("  Long press button (2+ sec): Toggle WiFi");
  Serial.println("  Hold button on boot (3+ sec): Clear Bluetooth pairings");
  Serial.println("Speed-based adaptive filtering active");
  Serial.printf("WiFi network name: '%s' (currently disabled)\n", AP_SSID);
}

// ===== Main Loop =====
void loop() {
  M5.update();

  // Handle web server (only if WiFi is enabled)
  if (wifiEnabled) {
    server.handleClient();
  }

  // Handle button presses
  if (M5.BtnA.wasHold()) {
    // Long press: Toggle WiFi
    toggleWiFi();
  }

  // Update at fixed interval
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime < Config::UPDATE_INTERVAL_MS) {
    return;
  }
  lastUpdateTime = currentTime;

  // Read external switch state (active LOW with pull-up)
  switchIsPressed = (digitalRead(SWITCH_PIN) == LOW);

  // Update status bar display when switch state changes or periodically
  if (switchIsPressed != switchWasPressed || (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL_MS)) {
    updateStatusBar();
    lastDisplayUpdate = currentTime;
  }

  // Skip if calibration is invalid or mouse is disabled
  if (!calibration.isValid || !mouseEnabled) {
    return;
  }

  // Read IMU data
  M5.Imu.update();
  auto imuData = M5.Imu.getImuData();

  // Apply calibration bias
  float gx = imuData.gyro.x - calibration.biasX;
  float gy = imuData.gyro.y - calibration.biasY;
  float gz = imuData.gyro.z - calibration.biasZ;

  // Convert from rad/s to deg/s for easier understanding
  gx *= RAD_TO_DEG;
  gy *= RAD_TO_DEG;
  gz *= RAD_TO_DEG;

  // Apply mounting angle correction BEFORE selecting axes
  // This compensates for the device not being perfectly aligned
  applyMountingCorrection(gx, gy, gz);

  // Select axes based on device orientation
  float horizontalRotation, verticalRotation;

  switch (currentOrientation) {
    case LEFT_V:                 // Side Left - Battery Vertical
      horizontalRotation = -gy;  // Yaw becomes horizontal
      verticalRotation = gz;     // Roll becomes vertical
      break;
    case LEFT_H:                 // Side Left - Battery Horizontal
      horizontalRotation = -gx;  // Pitch becomes horizontal
      verticalRotation = gz;     // Roll becomes vertical
      break;
    case RIGHT_V:                // Side Right - Battery Vertical
      horizontalRotation = -gy;  // Yaw becomes horizontal
      verticalRotation = -gz;    // Inverted roll becomes vertical
      break;
    case RIGHT_H:               // Side Right - Battery Horizontal
      horizontalRotation = gx;  // Inverted pitch becomes horizontal
      verticalRotation = -gz;   // Inverted roll becomes vertical
      break;
    case BACK_V:                 // Back - Battery Vertical
      horizontalRotation = -gy;  // Yaw becomes horizontal
      verticalRotation = -gx;    // Inverted pitch becomes vertical
      break;
    default:
      // Fallback to LEFT_V
      horizontalRotation = -gy;
      verticalRotation = gz;
      break;
  }

  // Update motion tracker with current speeds
  motionTracker.updateSpeed(horizontalRotation, verticalRotation);

  // Apply noise filtering using speed-based IIR filter
  horizontalRotation = xFilter.filter(horizontalRotation);
  verticalRotation = yFilter.filter(verticalRotation);

  // Get current motion speeds for dynamic response
  float speedX = motionTracker.getSpeedX();
  float speedY = motionTracker.getSpeedY();

  // Calculate dynamic response parameters
  float dynamicKX = calculateDynamicResponse(speedX, baseK, adaptStrength);
  float dynamicKY = calculateDynamicResponse(speedY, baseK, adaptStrength);

  // Apply response curve for natural feel
  float smoothedX = applyResponseCurve(horizontalRotation, dynamicKX);
  float smoothedY = applyResponseCurve(verticalRotation, dynamicKY);

  // Apply sensitivity scaling
  int mouseX = applySensitivity(smoothedX);
  int mouseY = applySensitivity(smoothedY);

  // Apply balance adjustment
  mouseX = applyBalance(mouseX, horizontalBalance);
  mouseY = applyBalance(mouseY, verticalBalance);

  // Apply speed limits to prevent runaway cursor
  mouseX = constrain(mouseX, -Config::MAX_MOUSE_SPEED, Config::MAX_MOUSE_SPEED);
  mouseY = constrain(mouseY, -Config::MAX_MOUSE_SPEED, Config::MAX_MOUSE_SPEED);

  // Send mouse movement and button state
#if DEBUG_MODE == 0
  if (useBluetoothMouse) {
    // Bluetooth HID: send movement and button state together
    sendBluetoothMouseMove(mouseX, mouseY, switchIsPressed);
  } else {
    // USB HID: handle movement and button separately
    if (mouseX != 0 || mouseY != 0) {
      Mouse.move(mouseX, mouseY);
    }

    // Handle button press/release
    if (switchIsPressed && !switchWasPressed) {
      Mouse.press(MOUSE_LEFT);
    } else if (!switchIsPressed && switchWasPressed) {
      Mouse.release(MOUSE_LEFT);
    }
  }

  // Update previous button state
  switchWasPressed = switchIsPressed;
#endif

#if DEBUG_MODE == 1
  // Debug output
  if (currentTime - lastDebugTime > Config::DEBUG_INTERVAL_MS) {
    Serial.println("\n----- Debug Info -----");
    Serial.printf("Raw Gyro: X=%.1f, Y=%.1f, Z=%.1f deg/s\n",
                  gx, gy, gz);
    Serial.printf("Filtered: H=%.1f, V=%.1f deg/s\n",
                  horizontalRotation, verticalRotation);
    Serial.printf("Motion Speed: X=%.1f, Y=%.1f deg/s\n",
                  speedX, speedY);
    Serial.printf("Dynamic K: X=%.2f, Y=%.2f (Base=%.2f)\n",
                  dynamicKX, dynamicKY, baseK);
    Serial.printf("Smoothed: X=%.1f, Y=%.1f\n",
                  smoothedX, smoothedY);
    Serial.printf("Mouse: X=%d, Y=%d\n", mouseX, mouseY);
    Serial.printf("Sensitivity: %s\n",
                  currentSensitivity == SENS_HIGH ? "HIGH" : currentSensitivity == SENS_MEDIUM ? "MEDIUM"
                                                                                               : "LOW");
    Serial.printf("Balance: H=%.0f%%, V=%.0f%%\n", horizontalBalance, verticalBalance);
    Serial.printf("Motion State: %s\n",
                  motionTracker.isStationary() ? "Stationary" : motionTracker.isSlowMoving() ? "Slow"
                                                              : motionTracker.isFastMoving() ? "Fast"
                                                                                             : "Normal");
    Serial.printf("WiFi Status: %s\n", wifiEnabled ? "ENABLED" : "DISABLED");
    lastDebugTime = currentTime;
  }
#endif
}