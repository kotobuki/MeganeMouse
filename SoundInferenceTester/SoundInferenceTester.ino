/* Edge Impulse Continuous Inference — Phase 3 Readiness Version
 *
 * Architecture:
 * - Core 0 (PRO_CPU): Audio Capture Task (High PRI) & ML Inference Task (Low PRI)
 * - Core 1 (APP_CPU): Arduino setup() & loop() ( BLE & Mouse Main Loop )
 * * Communication:
 * - Core 0 -> Core 1: FreeRTOS Queue (ClickEvent)
 */

#define USE_PDM_MIC 1  // 0 = ES8311 (Atomic Echo Base), 1 = PDM Mic

#define EIDSP_QUANTIZE_FILTERBANK 0

#include <M5Unified.h>

#if !USE_PDM_MIC
#include <Wire.h>
#endif

// ============================================================================
// IMPORTANT: How to improve detection responsiveness (Latency reduction)
// ============================================================================
// To improve the detection responsiveness for short sudden sounds like clicks,
// it is highly recommended to increase the number of slices per model window
// (e.g., from 4 to 8).
// * WARNING: DO NOT override EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW using
//  #define in this .ino file. Because the Arduino IDE compiles the main sketch
//  and the Edge Impulse library independently, defining it here will cause a
//  fatal memory mismatch between the sketch and the library, leading to memory
//  corruption and WDT crashes.
// * To safely change the slice count, you MUST directly edit the following header
//  file inside your exported Edge Impulse library folder:
//  -> src/model-parameters/model_metadata.h
// * Example modification in model_metadata.h:
//  #define EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW 8
// ============================================================================

#include <sound-click_inferencing.h>

// ============================================================
// Hardware & Audio Configuration
// ============================================================

#if USE_PDM_MIC
#define PDM_DAT_PIN 2
#define PDM_CLK_PIN 1
// PDM Software magnification (Software Gain)
uint8_t currentGainMultiplier = 8;  // x8. Adjust as needed (1, 2, 4, 8)
#else
#define ECHO_I2S_DIN 7
#define ECHO_I2S_WS 6
#define ECHO_I2S_BCK 8
#define ECHO_I2C_SDA 38
#define ECHO_I2C_SCL 39
#define ES8311_ADDR 0x18
// ES8311 ADC register gain (Hardware Gain)
uint8_t currentGainReg = 7;  // e.g., 1,3,5,7 (+24dB to +42dB)
#endif

// ML Settings
#define CLICK_THRESHOLD 0.7f
#define CLICK_DEBOUNCE_MS 500
// Audio chunk size. Must be a common divisor of EI classifier slice size.
#define AUDIO_CHUNK_SAMPLES EI_CLASSIFIER_SLICE_SIZE
#define CLICK_INFER_CLASS_NAME_MAX_LEN 32

// ============================================================
// Globals & Handlers
// ============================================================

typedef struct {
  int16_t* buffer;
  uint8_t buf_ready;
  uint32_t buf_count;
  uint32_t n_samples;
} inference_t;

static inference_t inference;
// Intermediate buffer between M5Unified Ring Buffer and inference.buffer
static signed short sampleBuffer[AUDIO_CHUNK_SAMPLES];

volatile bool record_status = false;
volatile bool capture_task_stopped = true;  // For safe task termination sync

TaskHandle_t capture_task_handle = NULL;
TaskHandle_t inference_task_handle = NULL;

// ============================================================
// FreeRTOS Queue and Event Definition (Inter-Core Communication)
// ============================================================

typedef struct {
  char class_name[CLICK_INFER_CLASS_NAME_MAX_LEN];
  float probability;
  uint32_t timestamp_ms;
} ClickEvent_t;

QueueHandle_t click_queue = NULL;
#define QUEUE_LENGTH 5  // Maximum pending events

// ============================================================
// ES8311 Hardware Controls (Only compiled if not using PDM)
// ============================================================
#if !USE_PDM_MIC
bool es8311_write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ES8311_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

uint8_t es8311_read_reg(uint8_t reg) {
  Wire.beginTransmission(ES8311_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ES8311_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

bool es8311_init_codec(uint8_t mic_gain) {
  Wire.beginTransmission(ES8311_ADDR);
  if (Wire.endTransmission() != 0) return false;

  // Reset sequence
  es8311_write_reg(0x00, 0x1F);
  delay(20);
  es8311_write_reg(0x00, 0x00);
  es8311_write_reg(0x00, 0x80);

  // Clock and Format configuration
  es8311_write_reg(0x01, 0xBF);
  es8311_write_reg(0x06, es8311_read_reg(0x06) & ~0x20);  // BCK source, clk enabled

  uint8_t reg02 = es8311_read_reg(0x02);
  es8311_write_reg(0x02, (reg02 & 0x07) | (0 << 5) | (2 << 3));  // Ratio coeff
  es8311_write_reg(0x03, 0x10);
  es8311_write_reg(0x04, 0x10);
  es8311_write_reg(0x05, 0x00);
  es8311_write_reg(0x06, (es8311_read_reg(0x06) & 0xE0) | 3);
  es8311_write_reg(0x07, es8311_read_reg(0x07) & 0xC0);
  es8311_write_reg(0x08, 0xFF);

  es8311_write_reg(0x00, es8311_read_reg(0x00) & 0xBF);
  es8311_write_reg(0x09, 0x0C);  // Slave, I2S, 16-bit
  es8311_write_reg(0x0A, 0x0C);

  // Power up and Mic configuration
  es8311_write_reg(0x0D, 0x01);  // Power up
  es8311_write_reg(0x0E, 0x02);
  es8311_write_reg(0x12, 0x00);  // ADC / DAC clk enabled
  es8311_write_reg(0x13, 0x10);
  es8311_write_reg(0x1C, 0x6A);  // ADC / DAC configuration
  es8311_write_reg(0x37, 0x08);
  es8311_write_reg(0x32, 0xBF);
  es8311_write_reg(0x31, 0x60);  // Mute DAC to prevent noise

  // Microphone setup
  es8311_write_reg(0x14, 0x1A);
  es8311_write_reg(0x17, 0xBF);
  es8311_write_reg(0x16, mic_gain);  // Set Hardware Mic Gain

  return true;
}
#endif

// ============================================================
// Mic Initialization
// ============================================================
void startMic() {
  M5.Mic.end();

#if !USE_PDM_MIC
  M5.Speaker.end();  // Stop speaker to release shared I2S resource
#endif

  auto mic_cfg = M5.Mic.config();
  mic_cfg.sample_rate = 16000;
  mic_cfg.stereo = false;

#if USE_PDM_MIC
  // PDM Configuration: Software Gain and Pins
  mic_cfg.magnification = currentGainMultiplier;  // Apply software gain multiplier
  mic_cfg.pin_data_in = PDM_DAT_PIN;
  mic_cfg.pin_ws = PDM_CLK_PIN;
  mic_cfg.pin_bck = -1;  // Not used
#else
  // ES8311 Configuration: Hardware Gain used (Fixed magnification), I2S Pins
  mic_cfg.magnification = 16;
  mic_cfg.pin_data_in = ECHO_I2S_DIN;
  mic_cfg.pin_ws = ECHO_I2S_WS;
  mic_cfg.pin_bck = ECHO_I2S_BCK;
#endif

  mic_cfg.pin_mck = -1;
  mic_cfg.over_sampling = 1;
  mic_cfg.dma_buf_len = 256;
  mic_cfg.dma_buf_count = 8;
  mic_cfg.task_priority = 2;  // Priority for M5GFX (higher than Core0 tasks)

  M5.Mic.config(mic_cfg);
  M5.Mic.begin();

  delay(100);

  // Dummy read to prime ADC pipeline
  static int16_t dummy[256];
  M5.Mic.record(dummy, 256, 16000);
  while (M5.Mic.isRecording()) { delay(1); }
}

// ============================================================
// Core 0 Task: Audio Capture
// ============================================================
static void capture_samples(void* arg) {
  ei_printf("Capture task started on Core %d\n", xPortGetCoreID());
  capture_task_stopped = false;

  while (record_status) {
    // 1. Record 1 slice (e.g., 1000 samples) from the M5Unified buffer
    M5.Mic.record(sampleBuffer, AUDIO_CHUNK_SAMPLES, 16000);
    while (M5.Mic.isRecording()) { vTaskDelay(1); }

    // 2. Copy directly to the inference buffer
    for (int i = 0; i < AUDIO_CHUNK_SAMPLES; i++) {
      inference.buffer[i] = sampleBuffer[i];
    }

    // 3. Send a notification signal to the inference task that data is ready
    if (inference_task_handle != NULL) {
      xTaskNotifyGive(inference_task_handle);
    }
  }

  // Safe idle loop when the task is stopped
  capture_task_stopped = true;
  while (1) { vTaskDelay(10); }
}

// ============================================================
// Core 0 Task: Continuous Inference
// ============================================================
static void inference_task(void* arg) {
  ei_printf("Inference task started on Core %d\n", xPortGetCoreID());

  // Log intervals
  static int log_counter = 0;
  // Log once per full window
  int log_interval = EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW;
  if (log_interval < 1) log_interval = 1;

  uint32_t last_click_detect_time = 0;

  // State variables for smoothing (consecutive frame detection)
  int last_valid_class_idx = -1;
  int consecutive_count = 0;
  const int REQUIRED_CONSECUTIVE_FRAMES = 2;  // Number of consecutive detections required to confirm an event

  while (1) {
    if (!record_status) {
      vTaskDelay(10);
      continue;
    }

    // Wait here until a notification is received from the audio capture task
    // indicating that 1 slice (e.g., 1000 samples) of data is ready.
    // This prevents Core 0 from looping unnecessarily (100% load) and avoids WDT errors.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Setup the signal object to fetch audio data
    signal_t signal;
    signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
    signal.get_data = &microphone_audio_signal_get_data;

    ei_impulse_result_t result = { 0 };
    // Pass signal as the 1st argument, and false (disable debug output) as the 3rd argument
    EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, false);

    if (r != EI_IMPULSE_OK) {
      ei_printf("ERR: Failed to run classifier (%d)\n", r);
      vTaskDelay(1000);
      continue;
    }

    // Find max probability class
    float max_val = 0.0f;
    int max_idx = -1;
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
      if (result.classification[ix].value > max_val) {
        max_val = result.classification[ix].value;
        max_idx = ix;
      }
    }

    if (max_idx == -1) continue;  // Should not happen with non-empty model

    const char* label = result.classification[max_idx].label;
    bool is_noise = (strcmp(label, "background_noise") == 0) || (strcmp(label, "impulsive_noise") == 0);

    // Consecutive detection (smoothing) + time-based debounce
    if (max_val >= CLICK_THRESHOLD) {
      // Same class detected consecutively as last time
      if (max_idx == last_valid_class_idx) {
        consecutive_count++;
      } else {
        // Different class detected — reset and start count from 1
        last_valid_class_idx = max_idx;
        consecutive_count = 1;
      }

      // Proceed only when the count reaches exactly the required number (prevents spam)
      if (consecutive_count == REQUIRED_CONSECUTIVE_FRAMES) {
        // Check if it's a valid non-noise event (click, long_press, etc.)
        if (!is_noise) {
          uint32_t now = millis();
          // Also apply the existing time-based debounce (prevents rapid-fire events)
          if (now - last_click_detect_time >= CLICK_DEBOUNCE_MS) {
            last_click_detect_time = now;

            // --- Core 0 -> Core 1 Communication using Queue ---
            if (click_queue != NULL) {
              ClickEvent_t event;
              strlcpy(event.class_name, label, CLICK_INFER_CLASS_NAME_MAX_LEN);
              event.probability = max_val;
              event.timestamp_ms = now;

              // Send event, do not block if queue is full
              if (xQueueSend(click_queue, &event, 0) != pdPASS) {
                ei_printf("ERR: Click queue full, event dropped.\n");
              }
            }
          }
        }
      }
    } else {
      // No class exceeded CLICK_THRESHOLD — fully reset state
      last_valid_class_idx = -1;
      consecutive_count = 0;
    }

    // Periodic log
    if (++log_counter >= log_interval) {
      log_counter = 0;
      ei_printf("[");
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("%s:%.0f%%", result.classification[ix].label, result.classification[ix].value * 100);
        if (ix < EI_CLASSIFIER_LABEL_COUNT - 1) ei_printf(" ");
      }
      ei_printf("] (DSP:%dms Cls:%dms)\n", result.timing.dsp, result.timing.classification);
    }
  }
}

// ============================================================
// Edge Impulse Controls & Callbacks
// ============================================================
static int microphone_audio_signal_get_data(size_t offset, size_t length, float* out_ptr) {
  // Copies samples from intermediate buffer (wrapping aware)
  numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);
  return 0;
}

bool microphone_inference_start(uint32_t n_samples) {
  // n_samples is EI_CLASSIFIER_RAW_SAMPLE_COUNT (e.g., 16000)
  inference.buffer = (int16_t*)malloc(n_samples * sizeof(int16_t));
  if (inference.buffer == NULL) return false;

  inference.buf_count = 0;
  inference.n_samples = n_samples;
  inference.buf_ready = 0;

  record_status = true;

  // Pin capture task to Core 0 (PRI 10, high priority)
  xTaskCreatePinnedToCore(
    capture_samples,
    "CaptureSamples",
    4096, NULL,
    10,  // Higher priority than inference task
    &capture_task_handle,
    0  // Core 0 (PRO_CPU)
  );
  return true;
}

void microphone_inference_end() {
  record_status = false;

  // Safely wait for task loop to exit
  while (!capture_task_stopped) { delay(10); }

  // Delete the task via Handle
  if (capture_task_handle != NULL) {
    vTaskDelete(capture_task_handle);
    capture_task_handle = NULL;
  }

  free(inference.buffer);
}

// ============================================================
// Setup (Core 1)
// ============================================================
void setup() {
  auto cfg = M5.config();
  cfg.internal_spk = false;
  cfg.internal_mic = false;
  cfg.external_spk = false;
  M5.begin(cfg);

  Serial.begin(115200);
  delay(2000);

  ei_printf("\nMeganeMouse Sound Click Phase 3 readiness\n");

  M5.Display.setFont(&fonts::Font0);
  M5.Display.setTextSize(2.0);
  M5.Display.fillScreen(TFT_NAVY);
  M5.Display.setTextColor(TFT_WHITE);
  M5.Display.setCursor(10, 50);
  M5.Display.print("READY");

  // --- Create Click Event Queue ---
  click_queue = xQueueCreate(QUEUE_LENGTH, sizeof(ClickEvent_t));
  if (click_queue == NULL) {
    ei_printf("ERR: Failed to create click queue.\n");
    return;
  }

#if !USE_PDM_MIC
  Wire.begin(ECHO_I2C_SDA, ECHO_I2C_SCL);
  if (!es8311_init_codec(currentGainReg)) {
    ei_printf("WAR: ES8311 not found.\n");
  }
#endif

  startMic();
  run_classifier_init();

  // Start Audio Capture Task on Core 0 (PRI 10)
  if (!microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT)) {
    ei_printf("ERR: Failed to setup audio sampling\n");
    return;
  }

  // Start ML Inference Task on Core 0 (PRI 5, lower priority)
  xTaskCreatePinnedToCore(
    inference_task,
    "InferenceTask",
    8192 * 4,  // ML task needs large stack
    NULL,
    5,
    &inference_task_handle,
    0  // Core 0 (PRO_CPU)
  );

  delay(100);
  ei_printf("Main loop running on Core %d\n", xPortGetCoreID());
}

// ============================================================
// Loop (Core 1) — Fully free for BLE and Main Mouse Control!
// ============================================================
void loop() {
  M5.update();

  // --- Receive Click Events from Core 0 via Queue (Non-blocking check) ---
  ClickEvent_t received_click;
  if (click_queue != NULL) {
    // xQueueReceive checks queue and copies data if available (copies struct data, do not block)
    if (xQueueReceive(click_queue, &received_click, 0) == pdPASS) {

      // Phase 3 main loop receives click data here!
      ei_printf("Loop recvd click: %s (%.0f%%)\n", received_click.class_name, received_click.probability * 100);

      // Visual update (Perform on Core 1 APP CPU)
      M5.Display.fillScreen(TFT_DARKGREEN);
      M5.Display.setCursor(10, 50);
      M5.Display.printf("%s\n%.0f%%", received_click.class_name, received_click.probability * 100);

      // Simulate mouse click hold or BLE send
      delay(CLICK_DEBOUNCE_MS / 2);

      M5.Display.fillScreen(TFT_NAVY);
    }
  }

  // Phase 3: Add MeganeMouse BLE mouse control and IMU reading here.

  delay(10);  // Loop rate
}