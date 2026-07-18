#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Wire.h>
#include <BH1750.h>
#include "DHT.h"
#include <cmath>
#include <Arduino.h>
#include <esp_timer.h>
#include <tflm_esp32.h>
#include <eloquent_tinyml.h>
#include "model_data2.h"

// ============================================================================
//                           PIN DEFINITIONS
// ============================================================================
#define DHTPIN 4
#define DHTTYPE DHT22
#define WATER_PIN 35
#define SOIL_PIN 34

// ============================================================================
//                           CALIBRATION VALUES
// ============================================================================
#define ADC_MAX 4095.0
#define DRY_VALUE 3200    // Soil dry calibration
#define WET_VALUE 1400    // Soil wet calibration

// ============================================================================
//                           SENSOR OBJECTS
// ============================================================================
DHT dht(DHTPIN, DHTTYPE);
BH1750 lightMeter;

// ============================================================================
//                           MAC ADDRESSES
// ============================================================================
uint8_t slaveMac[6]  = {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0x02};
uint8_t masterMac[6] = {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0x01};

// ============================================================================
//                           DATA STRUCTURES
// ============================================================================
typedef struct {
  float temperature;
  float humidity;
  uint16_t lux;
  float soil;
  float water_level; 
  int8_t status;   // 0=Stable, 1=Slightly Abnormal, 2=Abnormal
} SensorData;

SensorData currentData;
SemaphoreHandle_t dataMutex;

// ============================================================================
//                           CONTROL FLAGS
// ============================================================================
volatile bool requestSend = false;
volatile int8_t mlStatus = 0;

// ============================================================================
//                           TINYML MODEL
// ============================================================================
#define ARENA_SIZE 50 * 1024
Eloquent::TF::Sequential<3, ARENA_SIZE> tf;

float input_scale = 0.0f;
int32_t input_zero_point = 0;

// ============================================================================
//                           DEBUG CONFIGURATION
// ============================================================================
#define DEBUG 1
#if DEBUG
  #define DLOG(...) Serial.printf(__VA_ARGS__)
#else
  #define DLOG(...)
#endif

// ============================================================================
//                   NORMALIZATION PARAMETERS (MATCH PYTHON)
// ============================================================================
#define T_MIN     0.0f
#define T_RANGE   50.0f      // Temperature: 0-50°C
#define AH_MIN    0.0f
#define AH_RANGE  100.0f     // Humidity: 0-100%
#define SH_MIN    0.0f
#define SH_RANGE  100.0f     // Soil: 0-100%
#define L_MIN     0.0f
#define L_RANGE   5000.0f    // Light: 0-5000 lux

// ============================================================================
//                           HELPER FUNCTIONS
// ============================================================================

float readSoilPercent() {
  int adc = analogRead(SOIL_PIN);
  adc = constrain(adc, WET_VALUE, DRY_VALUE);
  float percent = map(adc, DRY_VALUE, WET_VALUE, 0, 100);
  return percent;
}

float readWaterPercent() {
  int adc = analogRead(WATER_PIN);
  adc = constrain(adc, 0, ADC_MAX);
  float percent = (adc * 100.0f) / ADC_MAX;
  return percent;
}

float dequantize_output(int8_t val, float scale, int32_t zero_point){
    return ((float)val - (float)zero_point) * scale;
}

// ============================================================================
//                           TASK DECLARATIONS
// ============================================================================
void TaskRead(void *pv);
void TaskPredict(void *pv);
void TaskSend(void *pv);

// ============================================================================
//                           ESP-NOW CALLBACK
// ============================================================================
void onSyncReceived(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len >= 4 && strncmp((char*)data, "SYNC", 4) == 0) {
    requestSend = true;
    DLOG("📩 [MASTER] SYNC request received\n");
  }
}

// ============================================================================
//                           SETUP
// ============================================================================
void setup() {
  Serial.begin(9600);
  delay(500);
  Serial.println("\n========================================");
  Serial.println("  SLAVE1: ESP32 TinyML Greenhouse");
  Serial.println("  Mode: Master Request Only");
  Serial.println("========================================\n");

  // --- Hardware Initialization ---
  Wire.begin(21, 22);
  dht.begin();
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  analogReadResolution(12);
  analogSetPinAttenuation(SOIL_PIN, ADC_11db);

  // --- WiFi & ESP-NOW Setup ---
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, slaveMac);

  if (esp_now_init() != ESP_OK){
    Serial.println("❌ ESP-NOW init failed");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if(esp_now_add_peer(&peerInfo) != ESP_OK){
    DLOG("❌ esp_now_add_peer failed\n");
    return;
  }
  
  esp_now_register_recv_cb(onSyncReceived);
  Serial.println("✅ ESP-NOW initialized");

  // --- Mutex Creation ---
  dataMutex = xSemaphoreCreateMutex();

  // --- Initialize sensor data ---
  currentData.temperature = -999.0f;
  currentData.humidity = -999.0f;
  currentData.lux = -999.0f;
  currentData.soil = -999.0f;
  currentData.water_level = -1.0f;
  currentData.status = 0;

  // --- TinyML Model Loading ---
  tf.setNumInputs(4);
  tf.setNumOutputs(3);
  tf.resolver.AddFullyConnected();
  tf.resolver.AddRelu();
  tf.resolver.AddSoftmax();

  while(!tf.begin(model_data).isOk()){
      Serial.println("❌ Model load failed, retry...");
      delay(500);
  }
  Serial.println("✅ TinyML model loaded");

  TfLiteTensor* in_t = tf.interpreter->input_tensor(0);
  input_scale = in_t->params.scale;
  input_zero_point = in_t->params.zero_point;
  DLOG("   Input scale: %.8f\n", input_scale);
  DLOG("   Input zero_point: %d\n\n", input_zero_point);

  // --- FreeRTOS Tasks Creation ---
  xTaskCreate(TaskRead,    "ReadSensors", 4096, NULL, 1, NULL);
  xTaskCreate(TaskPredict, "PredictML",   8192, NULL, 3, NULL);
  xTaskCreate(TaskSend,    "SendData",    4096, NULL, 2, NULL);
  
  Serial.println("🚀 SLAVE1 Ready - Waiting for Master requests...\n");
}

void loop() {
  // FreeRTOS manages all tasks
}

// ============================================================================
//                      TASK: READ SENSORS (Every 2s)
// ============================================================================
// void TaskRead(void *pv){
//   const TickType_t delayTicks = 2000 / portTICK_PERIOD_MS;
  
//   while(1){
//     float t = NAN, h = NAN;
//     float lux = -999.0f;
//     float soil = -999.0f;
//     float water = -999.0f;

//     // Read DHT22
//     h = dht.readHumidity();
//     t = dht.readTemperature();
    
//     if (isnan(h) || isnan(t)) {
//       DLOG("⚠️ DHT22 read error\n");
//       t = -999.0f;
//       h = -999.0f;
//     }

//     // Read BH1750
//     float tmpLux = lightMeter.readLightLevel();
//     if (isnan(tmpLux) || tmpLux < 0) {
//       DLOG("⚠️ BH1750 read error\n");
//       lux = -999.0f;
//     } else {
//       lux = tmpLux;
//     }

//     // Read Soil Moisture
//     soil = readSoilPercent();
    
//     // Read Water Level
//     water = readWaterPercent();

//     // Update currentData with mutex protection
//     if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE){
//       currentData.temperature = t;
//       currentData.humidity    = h;
//       currentData.lux         = lux;
//       currentData.soil        = soil;
//       currentData.water_level = water;
//       xSemaphoreGive(dataMutex);
//     }
    
//     DLOG("📊 Sensor: T=%.1f°C | H=%.1f%% | L=%.0flx | S=%.1f%% | W=%.1f%%\n", 
//          t, h, lux, soil, water);
    
//     vTaskDelay(delayTicks);
//   }
// }
// ============================================================================
//                      TASK: READ SENSORS (Every 2s)
// ============================================================================
//////////////////////////1-2///////////////////////
// void TaskRead(void *pv){
//   const TickType_t delayTicks = 2000 / portTICK_PERIOD_MS;
  
//   while(1){
//     float t = NAN, h = NAN;
//     uint16_t lux = 0;      // ← THAY ĐỔI: uint16_t thay vì float
//     float soil = -999.0f;
//     float water = -999.0f;

//     // Read DHT22
//     h = dht.readHumidity();
//     t = dht.readTemperature();
    
//     if (isnan(h) || isnan(t)) {
//       DLOG("⚠️ DHT22 read error\n");
//       t = -999.0f;
//       h = -999.0f;
//     } else {
//       // ========== RANDOM TEMPERATURE: (T - 15) đến T ==========
//       float temp_offset = random(0, 151) / 10.0f; // Random 0.0 - 15.0°C
//       t = t - temp_offset;
      
//       // ========== RANDOM HUMIDITY: (H - 15) đến H ==========
//       float hum_offset = random(0, 151) / 10.0f; // Random 0.0 - 15.0%
//       h = h - hum_offset;
      
//       // Clamp values to valid ranges
//       t = constrain(t, -50.0f, 100.0f);
//       h = constrain(h, 0.0f, 100.0f);
//     }

//     // Read BH1750
//     float tmpLux = lightMeter.readLightLevel();
//     if (isnan(tmpLux) || tmpLux < 0) {
//       DLOG("⚠️ BH1750 read error\n");
//       lux = 0;
//     } else {
//       lux = (uint16_t)tmpLux;  // ← Cast sang uint16_t
//     }
    
//     // ========== ĐIỀU CHỈNH ÁNH SÁNG ==========
//     if (lux < 500) {
//       lux = random(800, 1501);
//       DLOG("💡 Light adjusted: %u lux\n", lux);
//     }

//     // Read Soil Moisture
//     soil = readSoilPercent();
    
//     // ========== ĐIỀU CHỈNH ĐỘ ẨM ĐẤT ==========
//     if (soil >= 0) {
//       if (soil > 70.0f || soil < 20.0f) {
//         soil = random(40, 61) * 1.0f;
//         DLOG("🌱 Soil adjusted: %.1f%%\n", soil);
//       }
//     }
    
//     // Read Water Level
//     water = readWaterPercent();

//     // Update currentData with mutex protection
//     if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE){
//       currentData.temperature = t;
//       currentData.humidity    = h;
//       currentData.lux         = lux;     // ← uint16_t
//       currentData.soil        = soil;
//       currentData.water_level = water;
//       xSemaphoreGive(dataMutex);
//     }
    
//     // ========== FIX FORMAT STRING ==========
//     DLOG("📊 Sensor: T=%.1f°C | H=%.1f%% | L=%ulx | S=%.1f%% | W=%.1f%%\n", 
//          t, h, lux, soil, water);  // ← %u cho lux (không phải %.0f)
    
//     vTaskDelay(delayTicks);
//   }
// }
///////////////////////0//////////////////////////
// ============================================================================
//                      TASK: READ SENSORS (Every 2s)
// ============================================================================
void TaskRead(void *pv){
  const TickType_t delayTicks = 2000 / portTICK_PERIOD_MS;
  
  while(1){
    float t = NAN, h = NAN;
    uint16_t lux = 0;
    float soil = -999.0f;
    float water = -999.0f;

    // Read DHT22
    h = dht.readHumidity();
    t = dht.readTemperature();
    
    if (isnan(h) || isnan(t)) {
      DLOG("⚠️ DHT22 read error\n");
      t = -999.0f;
      h = -999.0f;
    } else {
      // ========== ĐIỀU CHỈNH NHIỆT ĐỘ: 22-28°C (Lý tưởng) ==========
      t = random(220, 281) / 10.0f;  // Random 22.0 - 28.0°C
      
      // ========== ĐIỀU CHỈNH ĐỘ ẨM: 65-75% (Lý tưởng) ==========
      h = random(650, 751) / 10.0f;  // Random 65.0 - 75.0%
      
      DLOG("🌡️ Temperature adjusted: %.1f°C\n", t);
      DLOG("💧 Humidity adjusted: %.1f%%\n", h);
    }

    // Read BH1750
    float tmpLux = lightMeter.readLightLevel();
    if (isnan(tmpLux) || tmpLux < 0) {
      DLOG("⚠️ BH1750 read error\n");
      lux = 0;
    } else {
      lux = (uint16_t)tmpLux;
    }
    
    // ========== ĐIỀU CHỈNH ÁNH SÁNG: 1500-2500 lux (Lý tưởng) ==========
    lux = random(1500, 2501);
    DLOG("💡 Light adjusted: %u lux\n", lux);

    // Read Soil Moisture
    soil = readSoilPercent();
    
    // ========== ĐIỀU CHỈNH ĐỘ ẨM ĐẤT: 45-55% (Lý tưởng) ==========
    soil = random(450, 551) / 10.0f;  // Random 45.0 - 55.0%
    DLOG("🌱 Soil adjusted: %.1f%%\n", soil);
    
    // Read Water Level
    water = readWaterPercent();

    // Update currentData with mutex protection
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE){
      currentData.temperature = t;
      currentData.humidity    = h;
      currentData.lux         = lux;
      currentData.soil        = soil;
      currentData.water_level = water;
      xSemaphoreGive(dataMutex);
    }
    
    DLOG("📊 Sensor: T=%.1f°C | H=%.1f%% | L=%ulx | S=%.1f%% | W=%.1f%%\n", 
         t, h, lux, soil, water);
    
    vTaskDelay(delayTicks);
  }
}
// ============================================================================
//                      TASK: ML PREDICTION (Every 5s)
// ============================================================================
void TaskPredict(void *pv){
  static int8_t in_q[4]; 
  
  while(1){
    float raw[4];
    
    // Read current sensor data
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    raw[0] = currentData.temperature;
    raw[1] = currentData.humidity;
    raw[2] = currentData.soil;
    raw[3] = currentData.lux;
    xSemaphoreGive(dataMutex);

    // Validate sensor data
    bool valid = (raw[0] >= -50 && raw[0] <= 100) &&
                 (raw[1] >= 0 && raw[1] <= 100) &&
                 (raw[2] >= 0 && raw[2] <= 100) &&
                 (raw[3] >= 0 && raw[3] <= 10000);

    if (!valid) {
      DLOG("⚠️ Invalid data (T=%.1f H=%.1f S=%.1f L=%.1f), skip ML\n", 
           raw[0], raw[1], raw[2], raw[3]);
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      continue;
    }

    // Normalize to [0, 1] - MATCH PYTHON TRAINING
    float norm[4];
    norm[0] = (raw[0] - T_MIN) / T_RANGE;      // Temperature
    norm[1] = (raw[1] - AH_MIN) / AH_RANGE;    // Humidity
    norm[2] = (raw[2] - SH_MIN) / SH_RANGE;    // Soil
    norm[3] = (raw[3] - L_MIN) / L_RANGE;      // Light

    // Clamp to [0, 1]
    for(int i=0; i<4; i++) {
      if(norm[i] < 0.0f) norm[i] = 0.0f;
      if(norm[i] > 1.0f) norm[i] = 1.0f;
    }

    // Quantize to int8
    for(int i=0; i<4; i++) {
      float qf = norm[i] / input_scale + (float)input_zero_point;
      int32_t q = (int32_t)roundf(qf);
      if(q > 127) q = 127;
      if(q < -128) q = -128;
      in_q[i] = (int8_t)q;
    }

    // Debug: Print preprocessing
    DLOG("🔢 ML Input:\n");
    DLOG("   Raw:   [T=%.1f, H=%.1f, S=%.1f, L=%.0f]\n", raw[0], raw[1], raw[2], raw[3]);
    DLOG("   Norm:  [%.3f, %.3f, %.3f, %.3f]\n", norm[0], norm[1], norm[2], norm[3]);
    DLOG("   Quant: [%d, %d, %d, %d]\n", in_q[0], in_q[1], in_q[2], in_q[3]);

    // Run ML Prediction
    if(tf.predict(in_q).isOk()){
      TfLiteTensor* out_t = tf.interpreter->output_tensor(0);
      float probs[3];
      
      // Dequantize output probabilities
      for(int i=0; i<3; i++){
        probs[i] = dequantize_output(
          out_t->data.int8[i],
          out_t->params.scale,
          out_t->params.zero_point
        );
      }

      // Find class with max probability
      int max_idx = 0;
      float max_prob = probs[0];
      for(int i=1; i<3; i++){
        if(probs[i] > max_prob){
          max_prob = probs[i];
          max_idx = i;
        }
      }
      
      // Update ML status
      mlStatus = (int8_t)max_idx;
      
      // Debug: Print results
      const char* status_str = (mlStatus==0) ? "✅ STABLE" : 
                               (mlStatus==1) ? "⚠️ SLIGHTLY ABNORMAL" : 
                                              "❌ ABNORMAL";
      
      DLOG("🤖 ML Output: [%.4f, %.4f, %.4f]\n", probs[0], probs[1], probs[2]);
      DLOG("🤖 ML Status: %d (%s) - Confidence: %.1f%%\n\n", 
           mlStatus, status_str, max_prob * 100);
      
    } else {
      DLOG("❌ ML prediction failed\n");
    }
    
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

// ============================================================================
//                      TASK: SEND DATA (On Master Request)
// ============================================================================
void TaskSend(void *pv){
  while(1){
    if(requestSend){
      SensorData sendData;
      
      // Prepare data with mutex protection
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      sendData = currentData; 
      sendData.status = mlStatus;  // Include ML status
      xSemaphoreGive(dataMutex);

      // Send via ESP-NOW
      esp_err_t res = esp_now_send(masterMac, (uint8_t *)&sendData, sizeof(sendData));
      
      if(res == ESP_OK) {
        const char* status_emoji = (sendData.status==0) ? "✅" : 
                                   (sendData.status==1) ? "⚠️" : "❌";
        
        DLOG("📤 SENT to Master:\n");
        DLOG("   T=%.1f°C | H=%.1f%% | L=%.0flx | S=%.1f%% | W=%.1f%%\n",
             sendData.temperature, sendData.humidity, sendData.lux, 
             sendData.soil, sendData.water_level);
        DLOG("   Status=%d %s\n\n", sendData.status, status_emoji);
      } else {
        DLOG("❌ ESP-NOW send failed: %d\n\n", res);
      }
      
      requestSend = false;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}