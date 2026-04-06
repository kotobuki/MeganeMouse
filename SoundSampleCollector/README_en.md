# MeganeMouse Sound Sample Collector

Sample collection tool for adding sound-based click functionality to [MeganeMouse](https://github.com/kotobuki/MeganeMouse).

Records continuous audio (default 60 seconds) from an attached microphone (PDM mic or Atomic Echo Base) and saves it as a single WAV file (16kHz / 16bit / mono) on a PC via USB Serial. The resulting file can be uploaded to Edge Impulse, where it can be split into individual samples for model training.

## Hardware requirements

- **M5Stack AtomS3R**
- **USB-C cable** (must support data transfer)
- **Microphone** (choose one of the following):
  - **PDM Microphone** (e.g., Unit Mini PDM connected to Port A)
  - **M5Stack Atomic Echo Base** (attached to the bottom of the AtomS3R)

## Verified environment

| Component             | Version |
| --------------------- | ------- |
| Arduino IDE           | 2.3.8   |
| M5Stack board package | 3.3.7   |
| M5Unified             | 0.2.13  |
| Python                | 3.14.3  |
| pyserial              | 3.5     |

## Setup

### Arduino

1. Install the following library in Arduino IDE:
   - `M5Unified` (including its dependencies)
2. Board settings:
   - Board: `M5AtomS3R` (M5Stack board package)
   - PSRAM: `OPI PSRAM`
   - USB CDC On Boot: `Enabled`
3. Open `SoundSampleCollector.ino` and set the hardware toggle at the top of the file:
   - For PDM Microphone: `#define USE_PDM_MIC 1`
   - For Atomic Echo Base: `#define USE_PDM_MIC 0`
4. Compile and upload the sketch.

> **Note:** The `M5EchoBase` library is **not used**. This sketch records audio using the M5Unified `M5.Mic` API. If the ES8311 codec is used, it is initialized via direct I2C register writes. This avoids the I2S driver conflict that occurs when the legacy driver (used by M5EchoBase) is linked alongside the new driver (used by M5Unified).

### PC

```bash
pip install pyserial
```

## Usage

### 1. Start the receiver script on the PC

```bash
python receive_samples.py --port /dev/cu.usbmodem* --label click_left # macOS
python receive_samples.py --port /dev/ttyACM0 --label click_left      # Linux
python receive_samples.py --port COM3 --label click_left              # Windows
```

### 2. Continuous recording

This sketch uses a **continuous recording mode**. A single button (i.e., AtomS3R's screen) click starts a 60-second (default) recording session. After recording, the WAV file is automatically transferred to the PC.

**Button operations:**

| Action         | While idle            | While recording |
| -------------- | --------------------- | --------------- |
| **Click**      | Start recording       | Stop early      |
| **Long press** | Cycle microphone gain | -               |

**Recording flow:**

```
Click → Countdown* → Record 60s continuously → Auto transfer → Done
                          ↑
             Click during recording to stop early
             (only the recorded portion is transferred)
```

> **Note on Countdown Behavior:**
>
> - **When using PDM Microphone:** Because there is no built-in speaker, the countdown is visual only (numbers flashing on the LCD).
> - **When using Atomic Echo Base:** The built-in speaker plays an audible 3-2-1 beep sequence along with the LCD display.

### 3. Splitting in Edge Impulse

1. Upload the recorded WAV file to Edge Impulse
2. Use Edge Impulse's "Split sample" feature to divide the continuous recording into individual samples
3. Delete any unsuitable segments manually

This workflow eliminates the need to produce sounds in precise synchronization with a countdown. Users can produce sounds at their own pace and organize the data later in Edge Impulse.

### 4. LCD display

- **Navy**: Idle (shows total sample count, gain level, recording duration)
- **Navy + large number**: Countdown (3 → 2 → 1)
- **Red**: Recording (shows elapsed seconds and progress bar)
- **Orange**: Transferring (progress bar)
- **Green**: Done

### 5. Output folder structure

```
samples/
  click_left/
    click_left.0001.wav
    click_left.0002.wav
    ...
  click_right/
    click_right.0001.wav
    ...
  noise/
    noise.0001.wav
    ...
```

Each WAV file is 16kHz / 16bit / mono.

## Recording tips

### Example Workflow

1. Run `receive_samples.py` specifying your target class (e.g., `--label click_left`).
2. Click the button on the AtomS3R to start recording.
3. For 60 seconds, repeatedly make the sound for the target class at your own pace.
4. Wait for the automatic data transfer to complete.
5. Once you have collected enough samples for the first class, stop `receive_samples.py` and repeat the process for other classes.
6. Upload the WAV files to Edge Impulse -> Split sample -> Delete unnecessary segments.

### Recording noise samples

The `noise` class is particularly easy to record. Start the recording and simply leave the device running for 60 seconds. Ambient sound is captured automatically. Record in multiple environments (quiet room, near a TV, outdoors, etc.) to improve model robustness.

### Changing recording duration

If 60 seconds is too long or too short, change the `CONTINUOUS_DURATION_SEC` value in the source code:

```cpp
#define CONTINUOUS_DURATION_SEC  60  // Change to 30, 120, etc.
```

The maximum duration is limited by PSRAM capacity (8MB on AtomS3R). Up to ~200 seconds is feasible.

## Technical details

### Architecture

**With PDM Microphone (`USE_PDM_MIC 1`)**

```
M5Unified ─── Display (M5.Display)
          ├── Button  (M5.BtnA)
          └── Mic     (M5.Mic)     ← Recording (I2S: PDM interface)
                 │
            PDM Mic (Connected to Port A, etc.)
```

**With Atomic Echo Base / ES8311 (`USE_PDM_MIC 0`)**

```
M5Unified ─── Display (M5.Display)
          ├── Button  (M5.BtnA)
          ├── Speaker (M5.Speaker) ← Beep tones (shares I2S, exclusive with Mic)
          └── Mic     (M5.Mic)     ← Recording  (shares I2S, exclusive with Speaker)
                 │
            I2S (GPIO 5,6,7,8) ← Switched between Speaker and Mic
                 │
            ES8311 codec ← Direct I2C control (GPIO 38,39, addr 0x18)
              ├── DAC → NS4150B amplifier → Speaker
              └── ADC ← MEMS microphone
```

### How continuous recording works

The 60-second recording is internally executed as 1-second chunks. M5.Mic's ring buffer (128ms) retains audio data between chunks, so no audio is lost. The LCD update time (~10ms) between chunks is bridged by the ring buffer — accumulated samples are consumed at the start of the next chunk.

### Memory usage

Allocate a recording buffer from the PSRAM. Since other libraries and components also use the PSRAM, not all of the available PSRAM shown in the table can be used.

| Duration | recording buffer size | PSRAM remaining |
| -------- | --------------------- | --------------- |
| 30s      | 960 KB                | ~7 MB           |
| 60s      | 1.88 MB               | ~6 MB           |
| 120s     | 3.75 MB               | ~4 MB           |
| 200s     | 6.25 MB               | ~1.7 MB         |

### ES8311 clock configuration (When using Atomic Echo Base)

The Atomic Echo Base has no MCLK pin. The ES8311 derives its internal clock from BCLK, using the coefficient table entry `{1024000, 16000}`.

## Troubleshooting

### Recorded audio is silent

- **When using PDM Mic**: Check the wiring of the DAT and CLK pins.
- **When using Atomic Echo Base**: Verify that the initialization log shows `ES8311 found at 0x18`.
- Try increasing microphone gain (long-press to cycle while idle).

### Transfer takes a long time

With USB CDC, the baud rate setting is virtual. Transferring 60 seconds (~1.92 MB) typically takes a few tens of seconds.

### Out of memory

Decrease `CONTINUOUS_DURATION_SEC`. The AtomS3R has 8 MB of PSRAM, but M5Unified and other libraries also consume memory.
