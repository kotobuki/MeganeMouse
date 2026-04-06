/*
 * SoundSampleCollector.ino
 *
 * MeganeMouse Sound Click — Phase 1: Sample Collection Firmware
 * Supports both PDM Microphone and ES8311 Codec via compile-time switch.
 *
 * Records continuous audio (default 60 seconds) and sends the entire 
 * recording as a single WAV file over USB Serial.
 *
 * Hardware Toggle:
 * Change 'USE_PDM_MIC' below to switch between hardware configurations.
 */

#define USE_PDM_MIC 1  // Set to 1 for PDM Mic, 0 for ES8311 Codec

#include <M5Unified.h>
#if !USE_PDM_MIC
#include <Wire.h>
#endif
#include "mbedtls/base64.h"

// ============================================================
// General Configuration
// ============================================================

#define SAMPLE_RATE 16000
#define SAMPLES_PER_SECOND SAMPLE_RATE
#define CONTINUOUS_DURATION_SEC 60

#define TOTAL_SAMPLES (SAMPLES_PER_SECOND * CONTINUOUS_DURATION_SEC)
#define TOTAL_PCM_SIZE (TOTAL_SAMPLES * 2)
#define WAV_HEADER_SIZE 44

#define B64_RAW_CHUNK 57

// ============================================================
// Hardware-Specific Configuration
// ============================================================

#if USE_PDM_MIC

// --- PDM Mic Settings ---
#define PDM_DAT_PIN 2
#define PDM_CLK_PIN 1

#define NUM_GAIN_LEVELS 5
const uint8_t GAIN_VALUES[NUM_GAIN_LEVELS] = { 1, 2, 4, 8, 16 };
const char* GAIN_LABELS[NUM_GAIN_LEVELS] = { "x1", "x2", "x4", "x8", "x16" };

#else

// --- ES8311 Settings ---
#define ECHO_I2S_DIN 7
#define ECHO_I2S_DOUT 5
#define ECHO_I2S_WS 6
#define ECHO_I2S_BCK 8
#define ECHO_I2C_SDA 38
#define ECHO_I2C_SCL 39

#define ES8311_ADDR 0x18
#define ES8311_REG00_RESET 0x00
#define ES8311_REG01_CLK_MGR 0x01
#define ES8311_REG02_CLK_MGR 0x02
#define ES8311_REG03_CLK_MGR 0x03
#define ES8311_REG04_CLK_MGR 0x04
#define ES8311_REG05_CLK_MGR 0x05
#define ES8311_REG06_CLK_MGR 0x06
#define ES8311_REG07_CLK_MGR 0x07
#define ES8311_REG08_CLK_MGR 0x08
#define ES8311_REG09_SDP_IN 0x09
#define ES8311_REG0A_SDP_OUT 0x0A
#define ES8311_REG0D_SYSTEM 0x0D
#define ES8311_REG0E_SYSTEM 0x0E
#define ES8311_REG12_SYSTEM 0x12
#define ES8311_REG13_SYSTEM 0x13
#define ES8311_REG14_SYSTEM 0x14
#define ES8311_REG16_ADC 0x16
#define ES8311_REG17_ADC 0x17
#define ES8311_REG1C_ADC 0x1C
#define ES8311_REG31_DAC 0x31
#define ES8311_REG32_DAC 0x32
#define ES8311_REG37_DAC 0x37

#define NUM_GAIN_LEVELS 4
const uint8_t GAIN_VALUES[NUM_GAIN_LEVELS] = { 1, 3, 5, 7 };
const char* GAIN_LABELS[NUM_GAIN_LEVELS] = { "6dB", "18dB", "30dB", "42dB" };

#define BEEP_FREQ_COUNTDOWN 440
#define BEEP_DURATION_MS 100
#define BEEP_INTERVAL_MS 1000
#define SPEAKER_VOLUME 200
#define I2S_SETTLE_MS 160
#define MIC_SWITCH_OVERHEAD_MS 80

#endif

// ============================================================
// Globals
// ============================================================

int16_t* recBuffer = nullptr;
int totalRecords = 0;
int currentGain = 1;
bool recording = false;

// ============================================================
// ES8311 Specific Functions
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
  if (Wire.endTransmission() != 0) {
    Serial.println("  ES8311 not found at 0x18!");
    return false;
  }
  Serial.println("  ES8311 found at 0x18");

  // Reset
  es8311_write_reg(ES8311_REG00_RESET, 0x1F);
  delay(20);
  es8311_write_reg(ES8311_REG00_RESET, 0x00);
  es8311_write_reg(ES8311_REG00_RESET, 0x80);

  // Clock: BCLK source, all clocks enabled
  es8311_write_reg(ES8311_REG01_CLK_MGR, 0xBF);
  uint8_t reg06 = es8311_read_reg(ES8311_REG06_CLK_MGR);
  reg06 &= ~0x20;
  es8311_write_reg(ES8311_REG06_CLK_MGR, reg06);

  // Coefficients for {1024000, 16000}
  uint8_t reg02 = es8311_read_reg(ES8311_REG02_CLK_MGR);
  reg02 &= 0x07;
  reg02 |= (0) << 5;
  reg02 |= (2) << 3;
  es8311_write_reg(ES8311_REG02_CLK_MGR, reg02);
  es8311_write_reg(ES8311_REG03_CLK_MGR, 0x10);
  es8311_write_reg(ES8311_REG04_CLK_MGR, 0x10);
  es8311_write_reg(ES8311_REG05_CLK_MGR, 0x00);
  reg06 = es8311_read_reg(ES8311_REG06_CLK_MGR);
  reg06 &= 0xE0;
  reg06 |= 3;
  es8311_write_reg(ES8311_REG06_CLK_MGR, reg06);
  uint8_t reg07 = es8311_read_reg(ES8311_REG07_CLK_MGR);
  reg07 &= 0xC0;
  es8311_write_reg(ES8311_REG07_CLK_MGR, reg07);
  es8311_write_reg(ES8311_REG08_CLK_MGR, 0xFF);

  // Format: Slave, I2S, 16-bit
  uint8_t reg00 = es8311_read_reg(ES8311_REG00_RESET);
  reg00 &= 0xBF;
  es8311_write_reg(ES8311_REG00_RESET, reg00);
  es8311_write_reg(ES8311_REG09_SDP_IN, 0x0C);
  es8311_write_reg(ES8311_REG0A_SDP_OUT, 0x0C);

  // Power up
  es8311_write_reg(ES8311_REG0D_SYSTEM, 0x01);
  es8311_write_reg(ES8311_REG0E_SYSTEM, 0x02);
  es8311_write_reg(ES8311_REG12_SYSTEM, 0x00);
  es8311_write_reg(ES8311_REG13_SYSTEM, 0x10);

  // ADC / DAC
  es8311_write_reg(ES8311_REG1C_ADC, 0x6A);
  es8311_write_reg(ES8311_REG37_DAC, 0x08);
  es8311_write_reg(ES8311_REG32_DAC, 0xBF);

  // Mute DAC to prevent noise when not playing
  es8311_write_reg(ES8311_REG31_DAC, 0x60);

  // Microphone
  es8311_write_reg(ES8311_REG14_SYSTEM, 0x1A);
  es8311_write_reg(ES8311_REG17_ADC, 0xBF);
  es8311_write_reg(ES8311_REG16_ADC, mic_gain);

  return true;
}

void es8311_set_mic_gain(uint8_t gain) {
  es8311_write_reg(ES8311_REG16_ADC, gain);
}

void es8311_dac_mute(bool mute) {
  uint8_t reg31 = es8311_read_reg(ES8311_REG31_DAC);
  if (mute) reg31 |= 0x60;
  else reg31 &= ~0x60;
  es8311_write_reg(ES8311_REG31_DAC, reg31);
}

void startSpeaker() {
  M5.Mic.end();
  es8311_dac_mute(false);

  auto spk_cfg = M5.Speaker.config();
  spk_cfg.pin_data_out = ECHO_I2S_DOUT;
  spk_cfg.pin_bck = ECHO_I2S_BCK;
  spk_cfg.pin_ws = ECHO_I2S_WS;
  spk_cfg.pin_mck = -1;
  spk_cfg.sample_rate = SAMPLE_RATE;
  spk_cfg.i2s_port = I2S_NUM_0;
  spk_cfg.magnification = 16;
  M5.Speaker.config(spk_cfg);
  M5.Speaker.begin();
  M5.Speaker.setVolume(SPEAKER_VOLUME);

  // Prime I2S TX pipeline
  M5.Speaker.tone(0, 1);
  delay(50);
}

#endif  // !USE_PDM_MIC

// ============================================================
// Common Audio Initialization
// ============================================================

void startMic() {
  M5.Mic.end();  // Required for config reload

#if !USE_PDM_MIC
  M5.Speaker.end();
  es8311_dac_mute(true);
#endif

  auto mic_cfg = M5.Mic.config();
  mic_cfg.sample_rate = SAMPLE_RATE;
  mic_cfg.over_sampling = 1;
  mic_cfg.dma_buf_len = 256;
  mic_cfg.dma_buf_count = 8;
  mic_cfg.task_priority = 2;

#if USE_PDM_MIC
  mic_cfg.stereo = false;
  mic_cfg.pin_data_in = PDM_DAT_PIN;
  mic_cfg.pin_ws = PDM_CLK_PIN;
  mic_cfg.pin_bck = -1;
  mic_cfg.pin_mck = -1;
  mic_cfg.magnification = GAIN_VALUES[currentGain];
#else
  mic_cfg.pin_data_in = ECHO_I2S_DIN;
  mic_cfg.pin_bck = ECHO_I2S_BCK;
  mic_cfg.pin_ws = ECHO_I2S_WS;
  mic_cfg.pin_mck = -1;
  mic_cfg.i2s_port = I2S_NUM_0;
  mic_cfg.magnification = 16;  // Gain handled via I2C for ES8311
#endif

  M5.Mic.config(mic_cfg);
  M5.Mic.begin();

#if USE_PDM_MIC
  delay(100);
#else
  delay(I2S_SETTLE_MS);
#endif

  // Dummy read to prime ADC pipeline
  static int16_t dummy[256];
  M5.Mic.record(dummy, 256, SAMPLE_RATE);
  while (M5.Mic.isRecording()) { delay(1); }
}

// ============================================================
// Display helpers
// ============================================================

void drawCentered(const char* text, int y, float textSize) {
  M5.Display.setTextSize(textSize);
  int w = M5.Display.textWidth(text);
  M5.Display.setCursor((128 - w) / 2, y);
  M5.Display.print(text);
}

void updateDisplay() {
  M5.Display.fillScreen(TFT_NAVY);
  M5.Display.setTextColor(TFT_WHITE);
  drawCentered("Sample Collector", 8, 1.0);

  char buf[32];
  snprintf(buf, sizeof(buf), "Count:%d", totalRecords);
  drawCentered(buf, 35, 2.0);

  snprintf(buf, sizeof(buf), "Gain:%s", GAIN_LABELS[currentGain]);
  drawCentered(buf, 68, 1.0);

  snprintf(buf, sizeof(buf), "%ds recording", CONTINUOUS_DURATION_SEC);
  drawCentered(buf, 90, 1.0);

  drawCentered("Click:Start", 108, 1.0);
}

void showRecording(int elapsed, int total) {
  M5.Display.fillScreen(TFT_RED);
  M5.Display.setTextColor(TFT_WHITE);
  drawCentered("REC", 10, 2.0);

  char buf[32];
  snprintf(buf, sizeof(buf), "%d / %d s", elapsed, total);
  drawCentered(buf, 60, 2.0);

  int pct = elapsed * 100 / total;
  M5.Display.drawRect(14, 92, 100, 12, TFT_WHITE);
  M5.Display.fillRect(16, 94, 96 * pct / 100, 8, TFT_WHITE);

  drawCentered("Click:Stop", 115, 1.0);
}

void showSending(int pct) {
  M5.Display.fillScreen(TFT_ORANGE);
  M5.Display.setTextColor(TFT_WHITE);
  drawCentered("SENDING", 15, 2.0);

  M5.Display.drawRect(14, 55, 100, 16, TFT_WHITE);
  M5.Display.fillRect(16, 57, 96 * pct / 100, 12, TFT_WHITE);

  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", pct);
  drawCentered(buf, 80, 2.0);
}

void showDone(int seconds) {
  M5.Display.fillScreen(TFT_DARKGREEN);
  M5.Display.setTextColor(TFT_WHITE);
  drawCentered("DONE!", 20, 3.0);

  char buf[32];
  snprintf(buf, sizeof(buf), "%ds saved", seconds);
  drawCentered(buf, 70, 1.5);

  snprintf(buf, sizeof(buf), "#%d", totalRecords);
  drawCentered(buf, 100, 1.5);
}

// ============================================================
// WAV Format and Serial Transfer
// ============================================================

void buildWAVHeader(uint8_t* h, uint32_t dataSize) {
  uint32_t sr = SAMPLE_RATE;
  uint16_t ch = 1, bps = 16;
  uint32_t byteRate = sr * ch * (bps / 8);
  uint16_t blockAlign = ch * (bps / 8);
  uint32_t fileSize = dataSize + 36;

  h[0] = 'R';
  h[1] = 'I';
  h[2] = 'F';
  h[3] = 'F';
  h[4] = fileSize;
  h[5] = fileSize >> 8;
  h[6] = fileSize >> 16;
  h[7] = fileSize >> 24;
  h[8] = 'W';
  h[9] = 'A';
  h[10] = 'V';
  h[11] = 'E';
  h[12] = 'f';
  h[13] = 'm';
  h[14] = 't';
  h[15] = ' ';
  h[16] = 16;
  h[17] = 0;
  h[18] = 0;
  h[19] = 0;
  h[20] = 1;
  h[21] = 0;
  h[22] = ch;
  h[23] = 0;
  h[24] = sr;
  h[25] = sr >> 8;
  h[26] = sr >> 16;
  h[27] = sr >> 24;
  h[28] = byteRate;
  h[29] = byteRate >> 8;
  h[30] = byteRate >> 16;
  h[31] = byteRate >> 24;
  h[32] = blockAlign;
  h[33] = 0;
  h[34] = bps;
  h[35] = 0;
  h[36] = 'd';
  h[37] = 'a';
  h[38] = 't';
  h[39] = 'a';
  h[40] = dataSize;
  h[41] = dataSize >> 8;
  h[42] = dataSize >> 16;
  h[43] = dataSize >> 24;
}

void sendWAVOverSerial(int recordedSeconds) {
  uint32_t pcm_size = (uint32_t)recordedSeconds * SAMPLES_PER_SECOND * 2;
  uint32_t wav_size = WAV_HEADER_SIZE + pcm_size;

  totalRecords++;
  Serial.printf(">>>WAV_START:sample:%04d\n", totalRecords);

  uint8_t header[WAV_HEADER_SIZE];
  buildWAVHeader(header, pcm_size);

  char b64line[80];
  size_t olen;
  mbedtls_base64_encode((unsigned char*)b64line, sizeof(b64line), &olen, header, WAV_HEADER_SIZE);
  b64line[olen] = '\0';
  Serial.println(b64line);

  uint8_t* pcm = (uint8_t*)recBuffer;
  size_t offset = 0;
  int lastPct = -1;

  while (offset < pcm_size) {
    size_t chunk = min((size_t)B64_RAW_CHUNK, (size_t)(pcm_size - offset));
    mbedtls_base64_encode((unsigned char*)b64line, sizeof(b64line), &olen, pcm + offset, chunk);
    b64line[olen] = '\0';
    Serial.println(b64line);
    offset += chunk;

    int pct = (int)((uint64_t)offset * 100 / pcm_size);
    if (pct != lastPct) {
      lastPct = pct;
      showSending(pct);
    }
  }

  Serial.println("<<<WAV_END");
  Serial.flush();

  Serial.printf("  Transferred: %d bytes (%ds WAV)\n", wav_size, recordedSeconds);
}

// ============================================================
// Countdown Sequence
// Returns true if cancel was requested
// ============================================================

bool runCountdown() {
#if !USE_PDM_MIC
  startSpeaker();
#endif

  for (int i = 3; i > 0; i--) {
    M5.Display.fillScreen(TFT_NAVY);
    M5.Display.setTextColor(TFT_WHITE);
    M5.Display.setTextSize(4);

    char n[4];
    snprintf(n, sizeof(n), "%d", i);
    int w = M5.Display.textWidth(n);
    M5.Display.setCursor((128 - w) / 2, 20);
    M5.Display.print(n);

    M5.Display.setTextSize(1);
    char buf[32];
    snprintf(buf, sizeof(buf), "%ds recording", CONTINUOUS_DURATION_SEC);
    drawCentered(buf, 95, 1.0);
    drawCentered("Click:Cancel", 115, 1.0);

    Serial.printf("  %d...\n", i);

#if USE_PDM_MIC
    // PDM: Visual-only countdown, wait 1 second
    uint32_t start = millis();
    while (millis() - start < 1000) {
      M5.update();
      if (M5.BtnA.wasClicked()) return true;
      delay(5);
    }
#else
    // ES8311: Audible countdown
    M5.Speaker.tone(BEEP_FREQ_COUNTDOWN, BEEP_DURATION_MS);
    uint32_t waitDuration = (i == 1) ? BEEP_INTERVAL_MS - MIC_SWITCH_OVERHEAD_MS : BEEP_INTERVAL_MS;

    uint32_t start = millis();
    while (millis() - start < waitDuration) {
      M5.update();
      if (M5.BtnA.wasClicked()) {
        M5.Speaker.stop();
        M5.Speaker.end();
        return true;
      }
      delay(5);
    }
#endif
  }

#if !USE_PDM_MIC
  M5.Speaker.stop();
  M5.Speaker.end();
#endif

  return false;
}

// ============================================================
// Continuous Recording Session
// ============================================================

void recordSession() {
  recording = true;
  Serial.println();
  Serial.printf("=== Recording session: duration=%ds ===\n", CONTINUOUS_DURATION_SEC);

  if (runCountdown()) {
    Serial.println("  Cancelled during countdown");
    recording = false;
    startMic();
    updateDisplay();
    return;
  }

  startMic();

  int recordedSeconds = 0;
  bool stopped = false;

  for (int sec = 0; sec < CONTINUOUS_DURATION_SEC; sec++) {
    int16_t* dest = recBuffer + (sec * SAMPLES_PER_SECOND);
    showRecording(sec, CONTINUOUS_DURATION_SEC);

    M5.Mic.record(dest, SAMPLES_PER_SECOND, SAMPLE_RATE);

    while (M5.Mic.isRecording()) {
      M5.update();
      if (M5.BtnA.wasClicked()) {
        stopped = true;
      }
      delay(5);
    }

    recordedSeconds = sec + 1;

    if (stopped) {
      Serial.printf("  Stopped early at %d seconds\n", recordedSeconds);
      break;
    }
  }

  showRecording(recordedSeconds, CONTINUOUS_DURATION_SEC);

  if (recordedSeconds > 0) {
    Serial.printf("  Recording complete: %d seconds\n", recordedSeconds);
    sendWAVOverSerial(recordedSeconds);
    showDone(recordedSeconds);
    delay(1500);
  }

  recording = false;
  updateDisplay();
  Serial.printf("=== Session ended ===\n");
}

// ============================================================
// Setup
// ============================================================

void setup() {
  auto cfg = M5.config();
  cfg.internal_spk = false;
  cfg.internal_mic = false;
  cfg.external_spk = false;
  M5.begin(cfg);

  Serial.begin(115200);
  delay(2000);
  Serial.println();
  Serial.println("=== MeganeMouse Sound Sample Collector ===");
  Serial.printf("    Continuous mode: %d seconds per recording\n", CONTINUOUS_DURATION_SEC);

  Serial.printf("[1/5] M5Unified OK, board:%d\n", (int)M5.getBoard());
  M5.Display.setFont(&fonts::Font0);
  M5.Display.setTextWrap(false);
  M5.Display.fillScreen(TFT_NAVY);
  M5.Display.setTextColor(TFT_WHITE);
  drawCentered("INIT...", 50, 2.0);

  Serial.printf("[2/5] PSRAM: total=%d free=%d\n", ESP.getPsramSize(), ESP.getFreePsram());
  if (ESP.getPsramSize() == 0) {
    Serial.println("  ERROR: No PSRAM!");
    while (1) delay(1000);
  }

  recBuffer = (int16_t*)heap_caps_malloc(TOTAL_PCM_SIZE, MALLOC_CAP_SPIRAM);
  if (!recBuffer) {
    Serial.println("  ERROR: Allocation failed!");
    while (1) delay(1000);
  }

#if !USE_PDM_MIC
  Serial.println("[3/5] I2C for ES8311...");
  Wire.begin(ECHO_I2C_SDA, ECHO_I2C_SCL);
  delay(50);

  Serial.println("[4/5] ES8311 codec...");
  if (!es8311_init_codec(GAIN_VALUES[currentGain])) {
    Serial.println("  WARNING: ES8311 init failed!");
  }
#else
  Serial.println("[3/5] (I2C skipped for PDM config)");
  Serial.println("[4/5] (Codec init skipped for PDM config)");
#endif

  Serial.println("[5/5] M5.Mic init...");
  startMic();
  Serial.printf("  Mic: enabled=%s\n", M5.Mic.isEnabled() ? "true" : "false");

  updateDisplay();
  Serial.println("[READY] Click=Record, LongPress=Gain");
}

// ============================================================
// Loop
// ============================================================

void loop() {
  M5.update();

  if (recording) return;

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "REC") {
      recordSession();
    }
  }

  if (M5.BtnA.wasClicked()) {
    recordSession();
  }

  if (M5.BtnA.wasHold()) {
    currentGain = (currentGain + 1) % NUM_GAIN_LEVELS;

#if USE_PDM_MIC
    // Re-initialize mic to apply M5Unified software gain
    startMic();
#else
    // Apply hardware gain to codec directly
    es8311_set_mic_gain(GAIN_VALUES[currentGain]);
#endif

    Serial.printf("GAIN_CHANGED:%s\n", GAIN_LABELS[currentGain]);
    M5.Display.fillScreen(TFT_NAVY);
    M5.Display.setTextColor(TFT_WHITE);
    drawCentered("MIC GAIN", 30, 2.0);
    drawCentered(GAIN_LABELS[currentGain], 65, 3.0);
    delay(800);
    updateDisplay();
  }

  delay(10);
}
