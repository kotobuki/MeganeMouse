# MeganeMouse Sound Inference Tester

A testing tool to verify and evaluate the performance of your trained Edge Impulse inference model before integrating the sound-based click functionality into the main MeganeMouse firmware.

It performs continuous inference on audio captured from the microphone and outputs detailed information, including class probabilities and processing times (DSP and classification), to the Serial Monitor.

## Features

- **Dual-Core (FreeRTOS) Architecture:** Audio capture and machine learning inference are handled by dedicated tasks on Core 0, leaving Core 1 completely free for the main loop (UI updates and future BLE/mouse control).
- **Real-time Monitoring:** Use the Serial Monitor to observe click detection probabilities and noise classification in real-time.

## Hardware Requirements

- **M5Stack AtomS3R**
- **USB-C cable** (must support data transfer)
- **Microphone** (choose one of the following):
  - **PDM Microphone** (e.g., Unit Mini PDM connected to Port A)
  - **M5Stack Atomic Echo Base** (attached to the bottom of the AtomS3R)

## Setup

### 1. Library Preparation
1. Build and download your trained model as an **Arduino Library** from the Edge Impulse dashboard.
2. In the Arduino IDE, go to `Sketch` > `Include Library` > `Add .ZIP Library...` and install the downloaded ZIP file.
3. Open `SoundInferenceTester.ino` and update the included header file name (e.g., `<sound-click_inferencing.h>`) to match your specific model's name.

### 2. [IMPORTANT] Improving Responsiveness (Changing Slice Count)
To prevent missing extremely short sudden sounds like clicks and to drastically reduce detection latency, it is highly recommended to increase the number of inference "slices" per window.

**WARNING:** Do not attempt to override this value using `#define` in the `.ino` file. To prevent fatal memory corruption during compilation, you MUST edit the Edge Impulse library source code directly.

1. Open your Arduino libraries folder (e.g., `Documents/Arduino/libraries/Your_Model_Name/`).
2. Open `src/model-parameters/model_metadata.h` in a text editor.
3. Find the following line and change the value from `4` to `8`, then save the file:

   ```c
   #define EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW    8
   ```

(Note: If you re-download and overwrite the library from Edge Impulse in the future, this setting will be reset and must be changed again.)

### 3. Hardware Configuration and Flashing

Set the USE_PDM_MIC macro at the top of the code according to your hardware (1 for PDM, 0 for Echo Base), and flash the code to your AtomS3R.

## Reading the Serial Monitor

Open the Serial Monitor at 115200 bps. You will see continuous output similar to the following:

```text
[noise:100% click:0% z_openset:0%] (DSP:5ms Cls:2ms)
[noise:0% click:100% z_openset:0%] (DSP:5ms Cls:2ms)
Loop recvd click: click (100%)
```

- The numbers inside `[ ... ]` represent the probability (0-100%) for each class at that moment.
- `(DSP:5ms Cls:2ms)` indicates the time taken for feature extraction (DSP processing) and the classification (inference) respectively.
- `Loop recvd click:` indicates that the inference result on Core 0 exceeded the threshold and the click event was successfully transmitted to the main loop on Core 1.
