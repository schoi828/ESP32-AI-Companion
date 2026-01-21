/*
 * Project ChatGPT Client For ESP32
 * Description: For HTTPS connection using WiFiClientSecure
 * Author: Eric Nam
 * Date: 07-17-2024
 */

// Working ElevenLabs main.cpp

#include <Arduino.h>

// FreeNove ESP32S3 Audio ElevenLabs Speech-to-Text

#include "driver/i2s.h"
#include "driver/gpio.h"
#include <Arduino.h>
#include "FS.h"
#include "SD_MMC.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <ChatGPT.hpp>
#include <LiquidCrystal_I2C.h>
#include <ESP32_AI_Connect.h>  // Main library for AI API connections
#include <NewPing.h>
//#include <I2CScanner.h>

#define TRIGGER_PIN 2
#define ECHO_PIN 21
#define MAX_DISTANCE 200 

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// WiFi credentials
const char* ssid = "와이파이 아이디";
const char* password = "와이파이 비밀번호";
// ElevenLabs API configuration
const char* elevenlabs_api_key = "키 입력";
const char* elevenlabs_stt_url = "https://api.elevenlabs.io/v1/speech-to-text";
// WiFiClientSecure client;
const char* apiKey= "OpenAI 키 입력";
ESP32_AI_Connect aiClient("openai", apiKey, "gpt-5.2");

LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD I2C address
#define WAV_FILE_NAME "recording"
#define SAMPLE_RATE 16000U
#define SAMPLE_BITS 16
#define WAV_HEADER_SIZE 44
#define VOLUME_GAIN 1
// Scaling from 24-in-32 I2S to 16-bit: increase to lower level, decrease to raise
#define I2S_SHIFT_BITS 12

// I2S Configuration for INMP441 on Freenove ESP32S3 WROOM (legacy I2S API)
#define I2S_NUM I2S_NUM_0
#define I2S_SCK_GPIO (gpio_num_t)12   // BCLK
#define I2S_WS_GPIO  (gpio_num_t)11   // LRCLK/WS
#define I2S_SD_GPIO  (gpio_num_t)10   // DOUT from mic

#define TOUCH_PIN 1 // T1 on your Freenove board (capacitive touch)
bool isPressed = false;

// Global variables
bool recording_active = false;
String last_transcription = "";
bool wifi_connected = false;
String current_recording_file = "";

// ===== FUNCTION DECLARATIONS =====
bool connectToWiFi();
bool init_i2s_legacy();
void deinit_i2s_legacy();
void cleanupOldRecordings();
void record_wav_streaming();
void process_recording();
String send_to_elevenlabs_stt(String filename);
void generate_wav_header(uint8_t* wav_header, uint32_t wav_size, uint32_t sample_rate);
bool mountSDMMC();

// ===== IMPLEMENTATION =====
String fit16(const String &s) {
  // LCD에는 보통 ASCII가 안정적. 필요하면 여기서 추가 정리 가능.
  if (s.length() <= 32) return s;       // 16x2 = 최대 32
  return s.substring(0, 32);            // 넘치면 잘라내기
}

void lcdPrint16x2(const String &msg) {
  lcd.clear();
  String t = fit16(msg);

  // 첫 줄(0~15)
  lcd.setCursor(0, 0);
  lcd.print(t.substring(0, min(16, (int)t.length())));

  // 둘째 줄(16~31)
  if (t.length() > 16) {
    lcd.setCursor(0, 1);
    lcd.print(t.substring(16, min(32, (int)t.length())));
  }
}

bool init_i2s_legacy() {
  Serial.println("Initializing I2S (legacy API) for INMP441...");

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,

    //.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // INMP441 L/R tied to GND -> Left channel
    // INMP441 uses standard I2S (Philips) format on ESP-IDF legacy API
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 6,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  if (i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL) != ESP_OK) {
    Serial.println("Failed to install I2S driver");
    return false;
  }

  i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = (int)I2S_SCK_GPIO,
    .ws_io_num = (int)I2S_WS_GPIO,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = (int)I2S_SD_GPIO
  };
  if (i2s_set_pin(I2S_NUM, &pin_config) != ESP_OK) {
    Serial.println("Failed to set I2S pins");
    i2s_driver_uninstall(I2S_NUM);
    return false;
  }

  // Ensure clock is set: 16kHz, 32-bit, mono
  if (i2s_set_clk(I2S_NUM, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO) != ESP_OK) {
    Serial.println("Failed to set I2S clock");
    i2s_driver_uninstall(I2S_NUM);
    return false;
  }

  Serial.println("I2S (legacy) initialized successfully");
  return true;
}

void deinit_i2s_legacy() {
  i2s_driver_uninstall(I2S_NUM);
}

bool connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    wifi_connected = true;
    return true;
  } else {
    Serial.println("\nWiFi connection failed");
    wifi_connected = false;
    return false;
  }
}

void setup() {
  Serial.begin(115200);
  // Capacitive touch does not require pinMode
  Serial.printf("ARDUINO=%d.%d.%d\n", ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
Serial.println(ESP.getSdkVersion());
  lcd.init();
  lcd.noBacklight(); // 시작은 꺼두고
	lcd.setCursor(0,0);		    		// 0번째 줄 0번째 셀부터 입력하게 합니다.
  lcd.print("Waitng for touch");

  if (!init_i2s_legacy()) {
    Serial.println("I2S init failed!");
    while (1)
      ;
  }

  // Mount built-in SD using robust helper (Freenove pins, 1-bit, freq fallbacks)
  if (!mountSDMMC()) {
    Serial.println("ERROR: Could not mount built-in SD (SD_MMC). Check card (FAT32), insert, and power-cycle.");
    while (1) { delay(1000); }
  }

  cleanupOldRecordings();
  connectToWiFi();
}

void loop() {
  delay(1000);
  Serial.print("ping");
  int distance = sonar.ping_cm();
  Serial.print("Distance: ");
  Serial.println(distance);
  if (distance < 20) {
    lcd.backlight();
    lcdPrint16x2("Hold my hand");

    unsigned long start = millis();
    const unsigned long waitMs = 5000; // 5초 동안 터치 기다림

    while (millis() - start < waitMs) {
      uint32_t touchVal = touchRead(TOUCH_PIN);
      bool touched = (touchVal > 50000); // 방향은 보드에 맞게 조정

      if (touched) {
        Serial.println("Touch pressed → start recording");
        lcd.clear();
        lcd.print("Recording...");
        record_wav_streaming();
        process_recording();
        break; // 한 번 녹음하면 대기 종료
      }
      delay(20); // 폴링 간격
    }
    delay(5000);
  }
  lcd.clear();
  lcd.noBacklight();

    //delay(50);
}

void record_wav_streaming() {
  const uint32_t max_record_time = 30;  // sec

  String filename = "/" + String(WAV_FILE_NAME) + "_" + String(millis()) + ".wav";
  current_recording_file = filename;

  File file = SD_MMC.open(filename.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file");
    current_recording_file = "";
    return;
  }

  uint8_t wav_header[WAV_HEADER_SIZE];
  generate_wav_header(wav_header, 0, SAMPLE_RATE);
  file.write(wav_header, WAV_HEADER_SIZE);

  // Allocate RX buffer for 32-bit I2S samples and TX buffer for 16-bit WAV PCM
  const size_t rx_bytes = 512; // multiple of 4
  int32_t* rx_buffer = (int32_t*)malloc(rx_bytes);
  int16_t* tx_buffer = (int16_t*)malloc(rx_bytes / 2);
  if (!rx_buffer || !tx_buffer) {
    if (rx_buffer) free(rx_buffer);
    if (tx_buffer) free(tx_buffer);
    return;
  }

  recording_active = true;
  size_t total_bytes = 0;
  unsigned long startTime = millis();

  Serial.println("Recording...");

  while ((touchRead(TOUCH_PIN) > 50000) && (millis() - startTime < max_record_time * 1000)) {
    size_t bytes_read = 0;
    if (i2s_read(I2S_NUM, (void*)rx_buffer, rx_bytes, &bytes_read, pdMS_TO_TICKS(100)) != ESP_OK) continue;

    size_t samples_read = bytes_read / sizeof(int32_t);
    for (size_t i = 0; i < samples_read; i++) {
      int32_t s = rx_buffer[i];
      // Convert 24-bit-in-32-bit I2S to 16-bit PCM; adjust shift/gain as needed
      int32_t v = s >> I2S_SHIFT_BITS; // scale down
      v = v << VOLUME_GAIN;
      if (v > 32767) v = 32767;
      if (v < -32768) v = -32768;
      tx_buffer[i] = (int16_t)v;
    }

    size_t bytes_to_write = samples_read * sizeof(int16_t);
    file.write((uint8_t*)tx_buffer, bytes_to_write);
    total_bytes += bytes_to_write;
  }

  recording_active = false;
  free(rx_buffer);
  free(tx_buffer);

  file.seek(0);
  generate_wav_header(wav_header, total_bytes, SAMPLE_RATE);
  file.write(wav_header, WAV_HEADER_SIZE);
  file.close();

  Serial.printf("Recording finished: %s (%d bytes)\n", filename.c_str(), total_bytes);
}

void process_recording() {
  if (current_recording_file.isEmpty()) return;
  
  Serial.printf("Sending %s to ElevenLabs...\n", current_recording_file.c_str());
  String transcription = send_to_elevenlabs_stt(current_recording_file);

  if (transcription.length()) {
    lcd.print("STT processing");
    Serial.println("Transcription:");
    Serial.println(transcription);
    last_transcription = transcription;
    String result;
    Serial.println("\n\n[ChatGPT] - Asking a Text Question");
    String prompt = "The following is a user's voice command in Korean: \"" + transcription + "\".\n" +
    "Respond ONLY in English.\n" +
    "**Make sure that your answer must be 30 characters or fewer.** You may freely answer as long as it stays within the character limit.\n" +
    "If the question can be answered with yes or no, respond with ONLY \"yes\" or \"no\".\n" +
    "If you do not know the answer, or if the information is insufficient, respond with \"I don't know\".\n" +
    "If the speech is unclear, respond with exactly: \"Audio unclear\".\n"
    "Do not add any extra words, explanations, or punctuation.";
    
    aiClient.setChatTemperature(1.0);       // Set response creativity (0.0-2.0)
    aiClient.setChatMaxTokens(1000);         // Limit response length (in tokens)
    
    // Send a test message to the AI and get response
    Serial.println("\nSending message to AI...");
    lcd.clear();
    lcd.print("Talking to AI");
    String response = aiClient.chat(prompt);
    
    // Print the AI's response
    Serial.println("\nAI Response:");
    Serial.println(response);
    lcdPrint16x2(response);

  } else {
    Serial.println("STT failed");
    lcd.clear();
    lcd.print("STT failed");
  }

  current_recording_file = "";
}

String send_to_elevenlabs_stt(String filename) {
  uint32_t t_start = millis();
  
  if (!wifi_connected || WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, cannot send to STT");
    return "";
  }

  File file = SD_MMC.open(filename.c_str());
  if (!file) {
    Serial.println("Failed to open audio file");
    return "";
  }

  size_t file_size = file.size();
  if (file_size > 500000) {
    Serial.println("File too large for STT request (>500KB)");
    file.close();
    return "";
  }

  uint8_t* audio_data = (uint8_t*)malloc(file_size);
  if (!audio_data) {
    Serial.println("Failed to allocate memory for audio data!");
    file.close();
    return "";
  }
  size_t bytesRead = file.read(audio_data, file_size);
  file.close();

  uint32_t t_file_loaded = millis();

  HTTPClient http;
  if (!http.begin(elevenlabs_stt_url)) {
    Serial.println("Failed to initialize HTTP connection");
    free(audio_data);
    return "";
  }

  http.setTimeout(30000);
  http.setConnectTimeout(10000);
  http.addHeader("xi-api-key", elevenlabs_api_key);

  String boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";
  http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);

  String body_start = "--" + boundary + "\r\n";
  body_start += "Content-Disposition: form-data; name=\"model_id\"\r\n\r\n";
  body_start += "scribe_v1\r\n";
  // Force English to avoid wrong language auto-detection
  body_start += "--" + boundary + "\r\n";
  body_start += "Content-Disposition: form-data; name=\"language_code\"\r\n\r\n";
  body_start += "ko\r\n";
  body_start += "--" + boundary + "\r\n";
  body_start += "Content-Disposition: form-data; name=\"file\"; filename=\"audio.wav\"\r\n";
  body_start += "Content-Type: audio/wav\r\n\r\n";

  String body_end = "\r\n--" + boundary + "--\r\n";
  size_t total_size = body_start.length() + file_size + body_end.length();
  uint8_t* complete_body = (uint8_t*)malloc(total_size);

  memcpy(complete_body, body_start.c_str(), body_start.length());
  memcpy(complete_body + body_start.length(), audio_data, file_size);
  memcpy(complete_body + body_start.length() + file_size, body_end.c_str(), body_end.length());

  free(audio_data);

  uint32_t t_request_prepared = millis();

  Serial.println("Sending request to ElevenLabs STT...");

  // Start timer just before POST
  uint32_t t_request_sent = millis();
  int httpResponseCode = http.POST(complete_body, total_size);
  // Stop timer after response received
  uint32_t t_response_received = millis();

  free(complete_body);

  String transcription = "";
  String response = http.getString();

  uint32_t t_response_parsed = millis();

  if (httpResponseCode == 200) {
    Serial.println("HTTP 200 OK");
    DynamicJsonDocument doc(8192);
    DeserializationError err = deserializeJson(doc, response);
    if (err) {
      Serial.print("JSON parse failed: ");
      Serial.println(err.c_str());
    } else {
      if (doc["text"].is<String>()) {
        transcription = doc["text"].as<String>();
      } else {
        JsonArray output = doc["output"].as<JsonArray>();
        if (!output.isNull()) {
          for (JsonVariant entry : output) {
            JsonArray content = entry["content"].as<JsonArray>();
            if (content.isNull()) continue;

            for (JsonVariant chunk : content) {
              const char* type = chunk["type"] | "";
              if (strcmp(type, "output_text") != 0) continue;

              const char* text = chunk["text"] | "";
              if (text == nullptr || text[0] == '\0') continue;

              if (transcription.length() > 0) {
                transcription += ' ';
              }
              transcription += String(text);
            }
          }
        }
      }
    }
  } else {
    Serial.printf("HTTP Error: %d\n", httpResponseCode);
    Serial.println("Response: " + response);
  }

  http.end();

  // Print detailed timing information (similar to Deepgram implementation)
  Serial.println("---------------------------------------------------");
  Serial.printf("-> Audio File [%s] size: %d bytes\n", filename.c_str(), file_size);
  Serial.printf("-> Latency File Loading [t_file_loaded]:     %.3f sec\n", (float)(t_file_loaded - t_start) / 1000);
  Serial.printf("-> Latency Request Preparation:              %.3f sec\n", (float)(t_request_prepared - t_file_loaded) / 1000);
  Serial.printf("-> Latency ElevenLabs STT Response:          %.3f sec\n", (float)(t_response_received - t_request_sent) / 1000);
  Serial.printf("-> Latency Response Parsing:                 %.3f sec\n", (float)(t_response_parsed - t_response_received) / 1000);
  Serial.printf("=> TOTAL Duration [sec]: .................... %.3f sec\n", (float)(t_response_parsed - t_start) / 1000);
  Serial.printf("=> Server response length [bytes]: %d\n", response.length());
  Serial.printf("=> Transcription: [%s]\n", transcription.c_str());
  Serial.println("---------------------------------------------------");

  return transcription;
}

void generate_wav_header(uint8_t* wav_header, uint32_t wav_size, uint32_t sample_rate) {
  uint32_t file_size = wav_size + WAV_HEADER_SIZE - 8;
  uint32_t byte_rate = sample_rate * SAMPLE_BITS / 8;

  const uint8_t header[] = {
    'R',
    'I',
    'F',
    'F',
    file_size,
    file_size >> 8,
    file_size >> 16,
    file_size >> 24,
    'W',
    'A',
    'V',
    'E',
    'f',
    'm',
    't',
    ' ',
    0x10,
    0x00,
    0x00,
    0x00,
    0x01,
    0x00,
    0x01,
    0x00,
    sample_rate,
    sample_rate >> 8,
    sample_rate >> 16,
    sample_rate >> 24,
    byte_rate,
    byte_rate >> 8,
    byte_rate >> 16,
    byte_rate >> 24,
    0x02,
    0x00,
    0x10,
    0x00,
    'd',
    'a',
    't',
    'a',
    wav_size,
    wav_size >> 8,
    wav_size >> 16,
    wav_size >> 24,
  };
  memcpy(wav_header, header, sizeof(header));
}

bool mountSDMMC() {
  Serial.println("Mounting SD_MMC (built-in) in 1-bit mode on Freenove pins ...");

  auto try_mount = [](int clk, int cmd, int d0) -> bool {
    SD_MMC.end();
    SD_MMC.setPins(clk, cmd, d0);
    Serial.printf("Trying SD_MMC clk=%d cmd=%d d0=%d @ DEFAULT freq...\n", clk, cmd, d0);
    if (SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5)) return true;
    Serial.println("DEFAULT freq failed. Trying ~26MHz...");
    SD_MMC.end();
    SD_MMC.setPins(clk, cmd, d0);
    // Note: Arduino-ESP32 v2.x exposes SDMMC_FREQ_26M/52M/DEFAULT/PROBING (not 10M/5M)
    if (SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_26M, 5)) return true;
    Serial.println("~26MHz failed. Trying PROBING (400kHz)...");
    SD_MMC.end();
    SD_MMC.setPins(clk, cmd, d0);
    if (SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_PROBING, 5)) return true;
    return false;
  };

  // Per your reference: CMD=38, CLK=39, D0=40
  if (!try_mount(39, 38, 40)) {
    Serial.println("clk=39 cmd=38 d0=40 failed. Trying d0=41 as fallback...");
    if (!try_mount(39, 38, 41)) {
      Serial.println("All SD_MMC mount attempts failed.");
      return false;
    }
    Serial.println("SD_MMC mounted using d0=41");
  } else {
    Serial.println("SD_MMC mounted using d0=40");
  }

  // Print card info
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return false;
  }
  Serial.print("Card type: ");
  if (cardType == CARD_MMC) Serial.println("MMC");
  else if (cardType == CARD_SD) Serial.println("SD");
  else if (cardType == CARD_SDHC) Serial.println("SDHC");
  else Serial.println("UNKNOWN");

  uint64_t cardSize = SD_MMC.cardSize() / (1024ULL * 1024ULL);
  Serial.printf("Card size: %llu MB\n", cardSize);

  // List root to verify FS
  File root = SD_MMC.open("/");
  if (!root) {
    Serial.println("Failed to open root directory");
    return false;
  }
  File f = root.openNextFile();
  Serial.println("Root directory:");
  while (f) {
    Serial.printf("  %s (%u bytes)\n", f.name(), (unsigned)f.size());
    f = root.openNextFile();
  }
  root.close();
  Serial.println("Built-in SD (SD_MMC) initialized");
  return true;
}

void cleanupOldRecordings() {
  File root = SD_MMC.open("/");
  File file = root.openNextFile();
  while (file) {
    String filename = file.name();
    if (filename.startsWith(WAV_FILE_NAME) && filename.endsWith(".wav")) {
      file.close();
      SD_MMC.remove("/" + filename);
    } else {
      file.close();
    }
    file = root.openNextFile();
  }
  root.close();
}