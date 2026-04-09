# MeganeMouse Sound Inference Tester

A testing tool to verify and evaluate the performance of your trained Edge Impulse inference model before integrating the sound-based click functionality into the main MeganeMouse firmware.

It performs continuous inference on audio captured from the microphone and outputs detailed information, including class probabilities and processing times (DSP and classification), to the Serial Monitor.

## Features

- **Dual-Core (FreeRTOS) Architecture:** Audio capture and machine learning inference are handled by dedicated tasks on Core 0, leaving Core 1 completely free for the main loop (UI updates and future BLE/mouse control).
- **Real-time Monitoring:** Use the Serial Monitor to observe click detection probabilities and noise classification in real-time.
- **Robust Click Detection:** Combines consecutive frame confirmation (a click is only registered after the same non-noise class exceeds the threshold for 2 consecutive inference frames) with a 500ms time-based debounce to suppress false positives while maintaining responsiveness.

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

Set the `USE_PDM_MIC` macro at the top of the code according to your hardware (`1` for PDM, `0` for Echo Base), and flash the code to your AtomS3R.

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

## Troubleshooting

### Error: `AllocateTensors() failedERR: Failed to run classifier (-3)`

#### Reason

When running inference on the ESP32-S3 with hardware optimization (ESP-NN) enabled, the neural network requires an additional "scratch buffer" to accelerate vector calculations. This required working memory often significantly exceeds (by tens of kilobytes) the default "Tensor Arena" size estimated by Edge Impulse, leading to an allocation failure.

#### Solution

Manually increase the allocated Tensor Arena size by modifying **two header files** in the exported Edge Impulse library.

#### Step 1: Increase the global maximum arena limit

- File: `src/model-parameters/model_metadata.h`
- Increase the value of `EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE` to a safely large number (e.g., `90000` to `150000`).

```c
// Before (The exact number varies depending on your model)
#define EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE     21683

// After (Provide enough overhead, e.g., ~90KB - 150KB)
#define EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE     90000
```

#### Step 2: Increase the specific model's memory allocation

- File: `src/model-parameters/model_variables.h`
- Add an extra margin (e.g., `+ 65000`) to the `.arena_size` assignment for your model.

```c
// Before
const ei_config_tflite_graph_t ei_config_graph_... = {
    // ...
    .arena_size = tflite_learn_..._arena_size
};

// After (Add padding like + 65000)
const ei_config_tflite_graph_t ei_config_graph_... = {
    // ...
    .arena_size = tflite_learn_..._arena_size + 65000
};
```

#### Step 3: Force a clean recompile (Crucial)

The Arduino IDE aggressively caches compiled library files. To ensure your header modifications are applied, add a blank line to your main `.ino` sketch and save it before clicking the "Upload" (Compile) button.

#### [Advanced] How to Profile the Exact Memory Usage (Arena Size)

The exact memory required by hardware optimizations (like ESP-NN for ESP32-S3) is dynamically allocated and cannot be perfectly estimated beforehand. If you need to strictly minimize the RAM footprint, you can output the exact bytes consumed by the inference engine. You can then use this exact number plus a small margin (e.g., +2048 bytes) to optimally set the limits in `model_metadata.h` and `model_variables.h`.

The file to modify is `src/edge-impulse-sdk/classifier/inferencing_engines/tflite_micro.h`
(Note: If you exported the model with the "EON Compiler" enabled, modify `tflite_eon.h` instead.)

Search for `AllocateTensors()` in the file. Add a debug print statement (`ei_printf`) immediately after the allocation success check to print the `arena_used_bytes()`.

```cpp
        // Before (Existing code)
        TfLiteStatus allocate_status = interpreter->AllocateTensors();
        if (allocate_status != kTfLiteOk) {
            ei_printf("ERR: Failed to allocate TFLite arena (error code %d)\n", allocate_status);
            return EI_IMPULSE_TFLITE_ARENA_ALLOC_FAILED;
        }

        // After (Add the following two lines)
        ei_printf("DEBUG: TFLite Arena used bytes: %d\n", 
                  (int)interpreter->arena_used_bytes());
```

After saving your main `.ino` sketch file and recompiling, the exact bytes consumed will be printed to the Serial Monitor on the first inference, like `DEBUG: TFLite Arena used bytes: 84287`. Once you have confirmed the value, remove or comment out the added debug code.
