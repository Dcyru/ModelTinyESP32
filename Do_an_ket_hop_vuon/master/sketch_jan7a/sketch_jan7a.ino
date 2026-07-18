#include <esp_wifi.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include "ModbusMaster_DeltaSlave.h"
#include <HTTPClient.h>
#include <time.h>

// ============================================================================
//                           CONFIGURATION
// ============================================================================

#define PLC_CYCLE_MS      1000    // PLC read/write cycle (1s)
#define HTTP_INTERVAL_MS  5000    // HTTP POST interval (5s)
#define ESPNOW_REQ_MS     60000   // ESP-NOW request interval (60s)
#define ESPNOW_RETRY_MS   3000    // ESP-NOW retry delay
#define ESPNOW_MAX_RETRY  2       // Max retry count

// ============================================================================
//                          WIFI & SERVER
// ============================================================================
const char* WIFI_SSID       = "DucHuy1";
const char* WIFI_PASS       = "123456789.";
const char* SERVER_SENSOR   = "http://192.168.1.20:3000/api/sensors";
const char* SERVER_SETTINGS = "http://192.168.1.20:3000/api/settings";

// ============================================================================
//                          MAC ADDRESSES
// ============================================================================
static const uint8_t MASTER_MAC[6] = {0x24,0x6F,0x28,0xAB,0xCD,0x01};
static const uint8_t SLAVE1_MAC[6] = {0x24,0x6F,0x28,0xAB,0xCD,0x02};
static const uint8_t SLAVE2_MAC[6] = {0x24,0x6F,0x28,0xAB,0xCD,0x03};

// ============================================================================
//                          MODBUS RTU CONFIG
// ============================================================================
#define RXD2_PIN   16
#define TXD2_PIN   17
#define DE_PIN     4
#define BAUDRATE   9600
#define SLAVE_ID   1

// PLC Register Addresses (Delta DVP format)
#define D0_ADDR 0  // D0
#define D100_ADDR  100   // D100 = 4097 + 100
#define M17_ADDR   17     // M17 starting address
#define ESPNOW_REQ_INTERVAL 30000


ModbusMaster_DeltaSlave modbus;

// ============================================================================
//                      DATA STRUCTURES
// ============================================================================

// --- ESP-NOW Sensor Data ---
typedef struct __attribute__((packed)) {
  float temperature;
  float humidity;
  float lux;
  float soil;  
  float water_level; 
  int8_t status;      // ML status from SLAVE
} SensorData;

typedef struct {
  // ===== GHI ESP32 -> PLC : D100 - D112 =====
  int16_t  temperature_x10;   // D100
  uint16_t light;             // D101
  uint16_t water_level;       // D102
  int16_t  humidity_x10;      // D103
  int16_t  soil_zone1_x10;    // D104
  int16_t  soil_zone2_x10;    // D105
  int16_t  soil_avg_x10;      // D106

  uint16_t day;               // D107
  uint16_t month;             // D108
  uint16_t year;              // D109
  uint16_t hour;              // D110
  uint16_t minute;            // D111
  uint16_t second;            // D112

  // ===== ĐỌC PLC -> ESP32 : D0 - D19 =====
  int16_t  set_temperature;   // D0
  uint16_t pump_on_threshold; // D1
  uint16_t pump_off_threshold;// D2
  uint16_t set_light;         // D3

  uint16_t t1_hour;           // D4
  uint16_t t1_minute;         // D5
  uint16_t t1_second;         // D6

  uint16_t t2_hour;           // D7
  uint16_t t2_minute;         // D8
  uint16_t t2_second;         // D9

  int16_t  set_soil_avg;      // D10
  int16_t  set_soil_zone1;    // D11
  int16_t  set_soil_zone2;    // D12

  uint16_t t3_hour;           // D13
  uint16_t t3_minute;         // D14
  uint16_t t3_second;         // D15

  uint16_t t4_hour;           // D16
  uint16_t t4_minute;         // D17
  uint16_t t4_second;         // D18

  uint16_t set_light_threshold;// D19
  uint8_t status_zone;   // ML status tổng
  bool    alert;         // cảnh báo
} PLCDataRegisters;


// ================= PLC COILS (M) =================
typedef struct {
  // -------- PLC → ESP32 (READ) : M0 – M10 --------
  bool fan_on;        // M0  trạng thái quạt
  bool pump_on;       // M1  bơm nước
  bool pump2_on;      // M2  bơm tưới
  bool valve1_on;     // M3  van KV1
  bool valve2_on;     // M4  van KV2
  bool light_on;      // M5  đèn
  bool mixer_on;      // M6  trộn phân
  bool ac_on;         // M7  sưởi / AC
  bool buzzer_on;     // M8  còi
  bool auto_mode;     // M9  auto
  bool manual_mode;   // M10 manual
  // -------- ESP32 → PLC (WRITE) : M17 – M19 -------
  bool abnormal;        // M17
  bool slightly_abnormal; // M18
  bool stable;          // M19
} PLCCoils;

/* Cấu trúc dữ liệu gửi lên Server - Sensors
 */
typedef struct {
  String   device_id;
  float    temperature;
  float    humidity;
  float    soil_moisture;        // Trung bình 2 khu vực
  float    light;
  bool     pump_on;
  bool     pump2_on;
  bool     light_on;
  bool     fan_on;
  bool     ac_on;                // heater
  bool     valve1_on;
  bool     valve2_on;
  bool     mixer_on;
  uint8_t  water_level;
  float    soil_zone1;
  float    soil_zone2;
  uint8_t  status_zone;
  bool     pump_auto_mode;       // Tự động suy ra từ auto_mode/manual_mode
  bool     light_auto_mode;
  bool     fan_auto_mode;
  bool     alert;
} SServerSensorData;

/**
 * Cấu trúc dữ liệu gửi lên Server - Settings
 */
typedef struct {
  String        device_id;
  float         target_temp;
  float         target_humidity;
  float         target_light;
  float         water_low_threshold;
  float         water_high_threshold;
  float         target_soil_zone1;
  float         target_soil_zone2;
  float         target_light_to_pump2;
  String        forbid_pump_start;    // Format: "HHMMSS"
  String        forbid_pump_end;
  String        light_start_time;
  String        light_end_time;
  unsigned long ts;
} Server_SettingsData_t;

// ============================================================================
//                        GLOBAL STATE
// ============================================================================

// ESP-NOW State
SensorData slave1Data, slave2Data;
bool slave1Received = false;
bool slave2Received = false;
bool slave1Valid = false;
bool slave2Valid = false;
int8_t ml1 = -1;
int8_t ml2 = -1;
int8_t mlTotal = -1;
unsigned long lastESPNowRequest = 0;
unsigned long lastESPNowRetry = 0;
uint8_t espnowRetryCount = 0;

// PLC State
PLCDataRegisters plcData = {};
PLCCoils plcCoils = {};
bool plcDataReady = false;

// HTTP State
unsigned long lastHttpSend = 0;

// Task Handles
TaskHandle_t modbusTaskHandle = NULL;

// ============================================================================
//                      HELPER FUNCTIONS
// ============================================================================

void syncNTPTime() {
  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("⏳ Syncing NTP time");
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.print(".");
    delay(500);
  }
  Serial.println(" ✅ Time synced");
}

void updateTimeRegisters() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    plcData.year   = timeinfo.tm_year + 1900;
    plcData.month  = timeinfo.tm_mon + 1;
    plcData.day    = timeinfo.tm_mday;
    plcData.hour   = timeinfo.tm_hour;
    plcData.minute = timeinfo.tm_min;
    plcData.second = timeinfo.tm_sec;
  }
}

// ============================================================================
//                      ESP-NOW FUNCTIONS
// ============================================================================

void requestESPNowData() {
  const char msg[] = "SYNC";
  esp_now_send(SLAVE1_MAC, (uint8_t*)msg, sizeof(msg) - 1);
  esp_now_send(SLAVE2_MAC, (uint8_t*)msg, sizeof(msg) - 1);
  Serial.println("📡 [ESP-NOW] Request sent to SLAVE1 & SLAVE2");
}
// Hàm cập nhật PLC coils từ ML_TOTAL
void updateMLCoilsFromMLTotal() {
    plcCoils.abnormal = false;          // M17
    plcCoils.slightly_abnormal = false; // M18
    plcCoils.stable = false;            // M19

    if (mlTotal == 0) {
        plcCoils.stable = true;         // Ổn định
    }
    else if (mlTotal == 1) {
        plcCoils.slightly_abnormal = true; // Hơi bất thường
    }
    else {
        plcCoils.abnormal = true;       // Bất thường
    }
}


// -------------------- ESP-NOW Receive --------------------
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len < sizeof(SensorData)) return;

    auto *received = (SensorData*)data;
    uint8_t slaveNum = info->src_addr[5];

    bool valid = (received->temperature >= -50 && received->temperature <= 100) &&
                 (received->humidity >= 0 && received->humidity <= 100) &&
                 (received->lux >= 0) &&
                 (received->soil >= 0 && received->soil <= 100);

    if (slaveNum == 0x02) {
        slave1Data = *received;
        slave1Received = true;
        slave1Valid = valid;
        ml1 = received->status;
    }
    else if (slaveNum == 0x03) {
        slave2Data = *received;
        slave2Received = true;
        slave2Valid = valid;
        ml2 = received->status;
    }

    if (slave1Received && slave2Received) {
        float avgT = (slave1Data.temperature + slave2Data.temperature) * 0.5f;
        float avgH = (slave1Data.humidity + slave2Data.humidity) * 0.5f;
        uint32_t avgL = (uint32_t)(slave1Data.lux + slave2Data.lux) * 0.5f;

        bool allValid = slave1Valid && slave2Valid;

        // ===== SOIL =====
        plcData.soil_zone1_x10 = slave1Valid ? (float)(slave1Data.soil * 10) : 0;
        plcData.soil_zone2_x10 = slave2Valid ? (float)(slave2Data.soil * 10) : 0;
        plcData.soil_avg_x10   = allValid
                                ? (int16_t)(((slave1Data.soil + slave2Data.soil) * 0.5f) * 10)
                                : 0;

        // ===== ML =====
        mlTotal = max(ml1, ml2);
        updateMLCoilsFromMLTotal();

        // ===== ENV =====
        plcData.temperature_x10 = allValid ? (int16_t)(avgT * 10) : 0;
        plcData.humidity_x10    = allValid ? (int16_t)(avgH * 10) : 0;
        plcData.light           = allValid ? (uint16_t)(avgL * 10): 0;
        // ===== WATER LEVEL từ SLAVE 1 =====
        plcData.water_level = slave1Valid ? (uint32_t)(slave1Data.water_level*10) : 0;
        // ===== DEBUG =====
        Serial.printf(
        "T%.1f H%.1f L%.0f | S[%.1f %.1f]=%.1f | W=%.0f | ML%d\n",
        avgT, avgH, avgL,
        slave1Valid ? slave1Data.soil : 0.0f,
        slave2Valid ? slave2Data.soil : 0.0f,
        allValid ? (slave1Data.soil + slave2Data.soil) * 0.5f : 0.0f,
        plcData.water_level,
        mlTotal
        );

        // reset flags
        slave1Received = false;
        slave2Received = false;
    }
}


// ============================================================================
//                      MODBUS PLC FUNCTIONS
// ============================================================================

bool writeSensorDataToPLC() {
  updateTimeRegisters();

   uint16_t writeBuf[13] = {
    (uint16_t)plcData.temperature_x10, // D100
    plcData.light,                     // D101
    plcData.water_level,               // D102
    (uint16_t)plcData.humidity_x10,    // D103
    (uint16_t)plcData.soil_zone1_x10,  // D104
    (uint16_t)plcData.soil_zone2_x10,  // D105
    (uint16_t)plcData.soil_avg_x10,    // D106
    plcData.day,                       // D107
    plcData.month,                     // D108
    plcData.year,                      // D109
    plcData.hour,                      // D110
    plcData.minute,                    // D111
    plcData.second                     // D112
  };
  bool ok = modbus.writeMultiple(SLAVE_ID, D100_ADDR, writeBuf, 13);
  
  if (ok) {
    Serial.printf("✅ [PLC WRITE] T=%d H=%d L=%u KV1=%d KV2=%d ALL=%d W=%d Time=%04d-%02d-%02d %02d:%02d:%02d\n",
                  plcData.temperature_x10, plcData.humidity_x10, plcData.light,
                  plcData.soil_zone1_x10, plcData.soil_zone2_x10, plcData.soil_avg_x10, plcData.water_level,
                  plcData.year, plcData.month, plcData.day,
                  plcData.hour, plcData.minute, plcData.second);
  } else {
    Serial.println("❌ [PLC WRITE] FAILED");
  }
  
  return ok;
}

bool writeCoilsToPLC() {
  bool ok = true;

  ok &= modbus.writeSingleM(SLAVE_ID, 17, plcCoils.abnormal);           // M17
  ok &= modbus.writeSingleM(SLAVE_ID, 18, plcCoils.slightly_abnormal);  // M18
  ok &= modbus.writeSingleM(SLAVE_ID, 19, plcCoils.stable);             // M19

  if (ok) {
    Serial.printf("✅ [PLC M WRITE] M17(ABN)=%d M18(MID)=%d M19(OK)=%d\n",
                  plcCoils.abnormal,
                  plcCoils.slightly_abnormal,
                  plcCoils.stable);
  } else {
    Serial.println("❌ [PLC M WRITE] FAILED");
  }

  return ok;
}

bool readDataFromPLC() {
  // Read D100-D120 (21 registers)
  uint16_t readBuf[21];
  bool ok = modbus.readMultiple(SLAVE_ID, D0_ADDR, readBuf, 20);
  
  if (ok) {
    // ================== D0 – D3 ==================
    plcData.set_temperature = (int16_t)readBuf[0];   // D0 : Nhiệt độ cài đặt
    plcData.pump_on_threshold = readBuf[1];          // D1 : Ngưỡng bật bơm
    plcData.pump_off_threshold = readBuf[2];         // D2 : Ngưỡng tắt bơm
    plcData.set_light = readBuf[3];                  // D3 : Ánh sáng cài đặ
    // ================== D4 – D6 : KHUNG GIỜ 1 ==================
    plcData.t1_hour   = readBuf[4];   // D4 : Giờ
    plcData.t1_minute = readBuf[5];   // D5 : Phút
    plcData.t1_second = readBuf[6];   // D6 : Giây
    // ================== D7 – D9 : KHUNG GIỜ 2 ==================
    plcData.t2_hour   = readBuf[7];   // D7 : Giờ
    plcData.t2_minute = readBuf[8];   // D8 : Phút
    plcData.t2_second = readBuf[9];   // D9 : Giây
    // ================== D10 – D12 : ĐỘ ẨM ĐẤT ==================
    plcData.set_soil_avg   = (int16_t)readBuf[10]; // D10 : Độ ẩm đất TB
    plcData.set_soil_zone1 = (int16_t)readBuf[11]; // D11 : Độ ẩm đất KV1
    plcData.set_soil_zone2 = (int16_t)readBuf[12]; // D12 : Độ ẩm đất KV2
    // ================== D13 – D15 : KHUNG GIỜ 3 ==================
    plcData.t3_hour   = readBuf[13];  // D13 : Giờ
    plcData.t3_minute = readBuf[14];  // D14 : Phút
    plcData.t3_second = readBuf[15];  // D15 : Giây
    // ================== D16 – D18 : KHUNG GIỜ 4 ==================
    plcData.t4_hour   = readBuf[16];  // D16 : Giờ
    plcData.t4_minute = readBuf[17];  // D17 : Phút
    plcData.t4_second = readBuf[18];  // D18 : Giây
    // ================== D19 ==================
    plcData.set_light_threshold = readBuf[19]; // D19 : Ngưỡng ánh sáng

    Serial.printf(
  "📖 [PLC READ]\n"
  "SetTemp=%d | PumpON=%u | PumpOFF=%u | SetLight=%u\n"
  "T1=%02u:%02u:%02u | T2=%02u:%02u:%02u\n"
  "SoilAVG=%d | Z1=%d | Z2=%d\n"
  "T3=%02u:%02u:%02u | T4=%02u:%02u:%02u\n"
  "LightThreshold=%u\n",
  plcData.set_temperature,        // D0
  plcData.pump_on_threshold,      // D1
  plcData.pump_off_threshold,     // D2
  plcData.set_light,              // D3
  plcData.t1_hour, plcData.t1_minute, plcData.t1_second,   // D4–D6
  plcData.t2_hour, plcData.t2_minute, plcData.t2_second,   // D7–D9
  plcData.set_soil_avg,            // D10
  plcData.set_soil_zone1,          // D11
  plcData.set_soil_zone2,          // D12
  plcData.t3_hour, plcData.t3_minute, plcData.t3_second,   // D13–D15
  plcData.t4_hour, plcData.t4_minute, plcData.t4_second,   // D16–D18
  plcData.set_light_threshold      // D19
);
    plcDataReady = true;
  } else {
    Serial.println("❌ [PLC READ] FAILED");
  }
  return ok;
}

bool readCoilsFromPLC() {
bool coilBuf[11];
bool ok = modbus.readMultiM(SLAVE_ID, 0, coilBuf, 11);

  
  if (ok) {
plcCoils.fan_on       = coilBuf[0];
plcCoils.pump_on      = coilBuf[1];
plcCoils.pump2_on     = coilBuf[2];
plcCoils.valve1_on    = coilBuf[3];
plcCoils.valve2_on    = coilBuf[4];
plcCoils.light_on     = coilBuf[5];
plcCoils.mixer_on     = coilBuf[6];
plcCoils.ac_on        = coilBuf[7];
plcCoils.buzzer_on    = coilBuf[8];
plcCoils.auto_mode    = coilBuf[9];
plcCoils.manual_mode = coilBuf[10];


    Serial.printf(
    "📖 [PLC M READ] FAN=%d WATER=%d IRRIG=%d KV1=%d KV2=%d LIGHT=%d MIX=%d HEAT=%d BUZ=%d AUTO=%d MAN=%d\n",
    plcCoils.fan_on,
  plcCoils.pump_on,
  plcCoils.pump2_on,
  plcCoils.valve1_on,
  plcCoils.valve2_on,
  plcCoils.light_on,
  plcCoils.mixer_on,
  plcCoils.ac_on,
  plcCoils.buzzer_on,
  plcCoils.auto_mode,
  plcCoils.manual_mode
  );
  } else {
    Serial.println("❌ [PLC COILS] READ FAILED");
  }

  return ok;
}
unsigned long lastSettingsSend = 0;  // ← THÊM DÒNG NÀY
#define SETTINGS_INTERVAL_MS 30000  // 30s gửi 1 lần

// ============================================================================
//                      HTTP SETTINGS FUNCTION
// ============================================================================
void sendSettingsToServer() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ [HTTP] WiFi not connected");
    return;
  }

  HTTPClient http;
  http.begin(SERVER_SETTINGS);
  http.addHeader("Content-Type", "application/json");

  // Chuẩn bị time strings theo format HHMMSS
  char t1[7], t2[7], t3[7], t4[7];
  sprintf(t1, "%02u%02u%02u", plcData.t1_hour, plcData.t1_minute, plcData.t1_second);
  sprintf(t2, "%02u%02u%02u", plcData.t2_hour, plcData.t2_minute, plcData.t2_second);
  sprintf(t3, "%02u%02u%02u", plcData.t3_hour, plcData.t3_minute, plcData.t3_second);
  sprintf(t4, "%02u%02u%02u", plcData.t4_hour, plcData.t4_minute, plcData.t4_second);

  // Lấy Unix timestamp (giây)
  struct tm timeinfo;
  time_t now_ts = 0;
  if (getLocalTime(&timeinfo)) {
    now_ts = mktime(&timeinfo);
  }

  String json = "{";
  json += "\"device_id\":\"ESP32_FARM_02\",";
  json += "\"target_temp\":" + String(plcData.set_temperature / 10.0, 1) + ",";
  json += "\"target_humidity\":"+ String(plcData.set_soil_avg / 10.0, 1) + ",";  // Nếu không có để 0.0
  json += "\"target_light\":" + String((float)plcData.set_light, 1) + ",";
  json += "\"water_low_threshold\":" + String((float)plcData.pump_on_threshold, 1) + ",";
  json += "\"water_high_threshold\":" + String((float)plcData.pump_off_threshold, 1) + ",";
  json += "\"target_soil_zone1\":" + String(plcData.set_soil_zone1 / 10.0, 1) + ",";
  json += "\"target_soil_zone2\":" + String(plcData.set_soil_zone2 / 10.0, 1) + ",";
  json += "\"target_light_to_pump2\":" + String((float)plcData.set_light_threshold, 1) + ",";
  json += "\"forbid_pump_start\":\"" + String(t1) + "\",";
  json += "\"forbid_pump_end\":\"" + String(t2) + "\",";
  json += "\"light_start_time\":\"" + String(t3) + "\",";
  json += "\"light_end_time\":\"" + String(t4) + "\",";
  json += "\"ts\":" + String(now_ts);
  json += "}";

  int code = http.POST(json);

  if (code > 0) {
    Serial.printf("📤 [HTTP] POST /settings → %d\n", code);
    if (code != 200) {
      Serial.println("Response: " + http.getString());
    }
  } else {
    Serial.printf("❌ [HTTP] Settings POST failed: %s\n",
                  http.errorToString(code).c_str());
  }

  http.end();
}
// ============================================================================
//                      HTTP SERVER FUNCTIONS
// ============================================================================

SServerSensorData prepareServerData() {
  SServerSensorData data;

  data.device_id = "ESP32_FARM_02";

  // ===== SENSOR =====
  data.temperature   = (plcData.temperature_x10) / 10.0f;
  data.humidity      = plcData.humidity_x10 / 10.0f;
  data.light         = (float)plcData.light;

  data.soil_zone1    = plcData.soil_zone1_x10 / 10.0f;
  data.soil_zone2    = plcData.soil_zone2_x10 / 10.0f;
  data.soil_moisture = (data.soil_zone1 + data.soil_zone2) / 2.0f;

  // ===== ACTUATOR STATE (từ PLC) =====
  data.pump_on   = (plcCoils.pump_on != 0);
  data.pump2_on  = (plcCoils.pump2_on != 0);
  data.light_on  = (plcCoils.light_on != 0);
  data.fan_on    = (plcCoils.fan_on != 0);
  data.ac_on = (plcCoils.ac_on != 0);
  data.valve1_on = (plcCoils.valve1_on != 0);
  data.valve2_on = (plcCoils.valve2_on != 0);
  data.mixer_on  = (plcCoils.mixer_on != 0);

  // ===== STATUS =====
  data.water_level = (uint32_t)slave1Data.water_level;
  data.status_zone = (uint8_t)max(ml1, ml2);
  data.alert       = (plcData.alert);

  // ===== AUTO / MANUAL (SUY RA) =====
  bool isAuto = (plcCoils.auto_mode == 1 && plcCoils.manual_mode == 0);

  data.pump_auto_mode  = isAuto;
  data.light_auto_mode = isAuto;
  data.fan_auto_mode   = isAuto;

  return data;
}


void sendSensorDataToServer() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ [HTTP] WiFi not connected");
    return;
  }

SServerSensorData data = prepareServerData();


  HTTPClient http;
  http.begin(SERVER_SENSOR);
  http.addHeader("Content-Type", "application/json");

  String json = "{";
  json += "\"device_id\":\"" + data.device_id + "\",";

  json += "\"temperature\":" + String(data.temperature, 1) + ",";
  json += "\"humidity\":" + String(data.humidity, 1) + ",";
  json += "\"soil_moisture\":" + String(data.soil_moisture, 1) + ",";
  json += "\"light\":" + String(data.light, 1) + ",";

  json += "\"pump_on\":"  + String(data.pump_on  ? "true" : "false") + ",";
  json += "\"pump2_on\":" + String(data.pump2_on ? "true" : "false") + ",";
  json += "\"light_on\":" + String(data.light_on ? "true" : "false") + ",";
  json += "\"fan_on\":"   + String(data.fan_on   ? "true" : "false") + ",";
  json += "\"ac_on\":"    + String(data.ac_on    ? "true" : "false") + ",";
  json += "\"valve1_on\":" + String(data.valve1_on ? "true" : "false") + ",";
  json += "\"valve2_on\":" + String(data.valve2_on ? "true" : "false") + ",";
  json += "\"mixer_on\":"  + String(data.mixer_on  ? "true" : "false") + ",";

  json += "\"water_level\":" + String(data.water_level) + ",";
  json += "\"soil_zone1\":"  + String(data.soil_zone1, 1) + ",";
  json += "\"soil_zone2\":"  + String(data.soil_zone2, 1) + ",";
  json += "\"status_zone\":" + String(data.status_zone) + ",";

  json += "\"pump_auto_mode\":"  + String(data.pump_auto_mode  ? "true" : "false") + ",";
  json += "\"light_auto_mode\":" + String(data.light_auto_mode ? "true" : "false") + ",";
  json += "\"fan_auto_mode\":"   + String(data.fan_auto_mode   ? "true" : "false") + ",";

  json += "\"alert\":" + String(data.alert ? "true" : "false");
  json += "}";

  int code = http.POST(json);

  if (code > 0) {
    Serial.printf("📤 [HTTP] POST /sensors → %d\n", code);
    if (code != 200) {
      Serial.println("Response: " + http.getString());
    }
  } else {
    Serial.printf("❌ [HTTP] POST failed: %s\n",
                  http.errorToString(code).c_str());
  }

  http.end();
}

// ============================================================================
//                      FREERTOS TASK - MODBUS
// ============================================================================

void TaskModbus(void *param) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  for (;;) {
    // 1. Write sensor data + time to PLC
    writeSensorDataToPLC();
    //vTaskDelay(pdMS_TO_TICKS(100));
    // 2. Write coils (ML status + auto modes)
    writeCoilsToPLC();
    //vTaskDelay(pdMS_TO_TICKS(100));
    // 3. Read actuator states + sensors from PLC
    readDataFromPLC();
    //vTaskDelay(pdMS_TO_TICKS(100));
    // 4. Read coils status
    readCoilsFromPLC();
    
    // Precise timing
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PLC_CYCLE_MS));
  }
}

// ============================================================================
//                          SETUP
// ============================================================================

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("\n\n========================================");
  Serial.println("  ESP32 Master - PLC Priority Edition");
  Serial.println("========================================\n");

  // --- WiFi Connection ---
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, MASTER_MAC);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  Serial.print("📶 Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" ✅ Connected");
  Serial.println("   IP: " + WiFi.localIP().toString());

  // --- NTP Time Sync ---
  syncNTPTime();

  // --- ESP-NOW Initialization ---
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed!");
    return;
  }

  esp_now_peer_info_t peer = {};
  peer.channel = 0;
  peer.encrypt = 0;

  memcpy(peer.peer_addr, SLAVE1_MAC, 6);
  esp_now_add_peer(&peer);
  memcpy(peer.peer_addr, SLAVE2_MAC, 6);
  esp_now_add_peer(&peer);

  esp_now_register_recv_cb(onDataRecv);
  Serial.println("✅ ESP-NOW initialized");

  // --- Modbus RS485 Initialization ---
  Serial2.begin(BAUDRATE, SERIAL_8E1, RXD2_PIN, TXD2_PIN);
  modbus.begin(Serial2, RXD2_PIN, TXD2_PIN, DE_PIN, BAUDRATE);
  modbus.setRetries(2);
  modbus.setTimeout(200);
  Serial.println("✅ Modbus RTU initialized");

  // --- FreeRTOS Task ---
  xTaskCreatePinnedToCore(
    TaskModbus,
    "ModbusTask",
    4096,
    NULL,
    2,              // Priority
    &modbusTaskHandle,
    1               // Core 1
  );
  Serial.println("✅ Modbus task started on Core 1");

  // --- Initial ESP-NOW Request ---
  requestESPNowData();
  lastESPNowRequest = millis();

  Serial.println("\n🚀 System ready!\n");
}

// ============================================================================
//                          MAIN LOOP
// ============================================================================

void loop() {
  unsigned long now = millis();

  // ===== ESP-NOW REQUEST CYCLE =====
  if (now - lastESPNowRequest >= ESPNOW_REQ_INTERVAL) {
    slave1Received = slave2Received = false;
    slave1Valid = slave2Valid = false;
    espnowRetryCount = 0;

    requestESPNowData();
    lastESPNowRequest = now;
    lastESPNowRetry = now;
  }

  // ===== ESP-NOW RETRY LOGIC =====
  bool ok1 = slave1Received && slave1Valid;
  bool ok2 = slave2Received && slave2Valid;

  if ((!ok1 || !ok2) && espnowRetryCount < ESPNOW_MAX_RETRY) {
    if (now - lastESPNowRetry >= ESPNOW_RETRY_MS) {
      espnowRetryCount++;
      Serial.printf("🔄 [ESP-NOW] Retry %d/%d\n", espnowRetryCount, ESPNOW_MAX_RETRY);
      requestESPNowData();
      lastESPNowRetry = now;
    }
  }
// ===== HTTP SEND SENSOR DATA CYCLE (5s) =====
  if (now - lastHttpSend >= HTTP_INTERVAL_MS) {
    sendSensorDataToServer();
    lastHttpSend = now;
  }

  // ===== HTTP SEND SETTINGS CYCLE (30s) =====
  if (now - lastSettingsSend >= SETTINGS_INTERVAL_MS) {
    sendSettingsToServer();
    lastSettingsSend = now;
  }

  // Minimal delay for WDT
  vTaskDelay(1);
}