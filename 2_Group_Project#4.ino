// Code by Pongthorn Boonrod and Phayuth Ittisajja

// --- Core ESP32 & Time Libraries ---
#include <WiFi.h>
#include <time.h>
#include "esp_sntp.h"


// --- Libraries for Sensors & WebServer ---
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include "Adafruit_SHT4x.h"
#include <Adafruit_Sensor.h>
#include <WebServer.h>


// --- (!!) ADDED Libraries for Encryption (!!) ---
#include <mbedtls/aes.h>    // (AES ใช้งานได้อยู่แล้ว)
#include <mbedtls/sha256.h> // (SHA-256 ใช้งานได้อยู่แล้ว)
#include <string.h>         // For memset/memcpy


// --- Mutex Handles ---
SemaphoreHandle_t i2cMutex = NULL;
SemaphoreHandle_t dataMutex = NULL;


// --- Sensor Objects ---
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
Adafruit_SHT4x sht4 = Adafruit_SHT4x();


// --- (!!) TWO Servers Running (!!) ---
WebServer server(80);         // Server 1: HTTP Web page
WiFiServer aesServer(10000);    // <-- ADDED: Server 2: Encrypted TCP (Port 10000)


// --- (!!) ADDED: CLASSROOM DH Parameters (!!) ---
unsigned long long P = 23; // เลข Prime
unsigned long long G = 5;  // เลข Generator


// --- (!!) UPDATED: 3-Sensor Data Structure (144 bytes) (!!) ---
struct SensorData {
  // BMP280 (40 bytes)
  float bmp_temp;
  float pressure_hpa;
  char bmp_timestamp[32];
  // MPU6050 (56 bytes)
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  char mpu_timestamp[32];
  // SHT4x (40 bytes)
  float sht_temp;
  float sht_humidity;
  char sht_timestamp[32];
  // (!!) ADDED PADDING (!!)
  // (136 bytes + 8 bytes padding = 144 bytes)
  char padding[8];
};
SensorData g_sensorData; // Shared global data


// --- WiFi & Time Config ---
const char *ssid     = "GalaxyA"; // <-- แก้ WiFi
const char *password = "hiphatt65"; // <-- แก้รหัสผ่าน
const long gmtOffset_sec = 7 * 3600;
const int daylightOffset_sec = 0;


// --- Task Handles ---
TaskHandle_t hPrintTask = NULL;
TaskHandle_t hBMP280Task = NULL;
TaskHandle_t hMPU6050Task = NULL;
TaskHandle_t hSHT4xTask = NULL;
TaskHandle_t hWebServerTask = NULL;
TaskHandle_t hEncryptedServerTask = NULL; // <-- ADDED


// --- Function Prototypes ---
void taskReadBMP280(void *pv);
void taskReadMPU6050(void *pv);
void taskReadSHT4x(void *pv);
void taskWebServer(void *pv);
void taskEncryptedServer(void *pv); // <-- ADDED
void handleRoot();


// --- (!!) ADDED: "Classroom" Math Function (!!) ---
unsigned long long power(unsigned long long base, unsigned long long exp, unsigned long long mod) {
    unsigned long long res = 1;
    base = base % mod;
    while (exp > 0) {
        if (exp % 2 == 1) res = (res * base) % mod;
        exp = exp >> 1;
        base = (base * base) % mod;
    }
    return res;
}


//=============================================================================
// SETUP
//=============================================================================
void setup()
{
    Serial.begin(115200);
    Serial.println("\n[Main] Booting...");


    // Create mutexes
    i2cMutex = xSemaphoreCreateMutex();
    dataMutex = xSemaphoreCreateMutex();
   
    // Initialize global data
    if(xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      strcpy(g_sensorData.bmp_timestamp, "N/A");
      strcpy(g_sensorData.mpu_timestamp, "N/A");
      strcpy(g_sensorData.sht_timestamp, "N/A");
      xSemaphoreGive(dataMutex);
    }


    // Start I2C
    Wire.begin(41, 40); // SDA=41, SCL=40
   
    // Initialize sensors (3 sensors)
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      if (!bmp.begin(0x76)) Serial.println("[Sensor] BMP280 not found!");
      if (!mpu.begin()) Serial.println("[Sensor] MPU6050 not found!");
      if (!sht4.begin()) Serial.println("[Sensor] SHT4x not found!");
      xSemaphoreGive(i2cMutex);
    }


    // Connect to WiFi
    Serial.println("[Main] Connecting WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("[WebServer] HTTP IP: ");
    Serial.println(WiFi.localIP());


    // --- (!!) Start BOTH Servers (!!) ---
    server.on("/", handleRoot);
    server.begin();          // Start HTTP Server (Port 80)
    aesServer.begin();         // <-- ADDED: Start AES TCP Server (Port 10000)
    Serial.println("[Main] HTTP Server (Port 80) and AES Server (Port 10000) started.");


    // Setup NTP (Stable version)
    configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org");
    sntp_set_sync_interval(0); // <-- (!!) FIX (!!)
    sntp_restart(); // Sync once


    // Create FreeRTOS tasks
    Serial.println("[Main] Creating tasks...");
    xTaskCreate(taskPrint, "PrintTask", 4096, NULL, 1, &hPrintTask);
    xTaskCreate(taskReadBMP280, "BMP280Task", 4096, NULL, 1, &hBMP280Task);
    xTaskCreate(taskReadMPU6050, "MPU6050Task", 8192, NULL, 1, &hMPU6050Task);
    xTaskCreate(taskReadSHT4x, "SHT4xTask", 4096, NULL, 1, &hSHT4xTask);
    xTaskCreate(taskWebServer, "WebServerTask", 8192, NULL, 2, &hWebServerTask);
   
    // <-- (!!) ADDED: Create AES Server Task (!!) -->
    xTaskCreate(taskEncryptedServer, "EncSrvTask", 4096, NULL, 2, &hEncryptedServerTask);
   
    Serial.println("[Main] All tasks created. Setup complete.");
}


//=============================================================================
// MAIN LOOP (IDLE)
//=============================================================================
void loop()
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}


//=============================================================================
// (TASKS 1-4: Project 3 - Unchanged)
//=============================================================================


// --- Task: Print current time ---
void taskPrint(void *pv) {
  (void)pv;
  char timeBuffer[32];
  struct tm timeinfo;
  for (;;) {
      if (getLocalTime(&timeinfo, 5000)) {
          strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
          Serial.printf("[Time] %s\n", timeBuffer);
      } else {
          Serial.println("[Time] Failed to obtain time");
      }
      vTaskDelay(pdMS_TO_TICKS(10000));
  }
}


// --- Task: Read BMP280 ---
void taskReadBMP280(void *pv) {
  (void)pv;
  struct tm timeinfo;
  for(;;) {
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
          float temp = bmp.readTemperature();
          float pres = bmp.readPressure() / 100.0F;
          xSemaphoreGive(i2cMutex);


          if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
              g_sensorData.bmp_temp = temp;
              g_sensorData.pressure_hpa = pres;
              if (getLocalTime(&timeinfo, 1000)) {
                  strftime(g_sensorData.bmp_timestamp, 32, "%Y-%m-%d %H:%M:%S", &timeinfo);
              }
              xSemaphoreGive(dataMutex);
          }
      } else {
          Serial.println("[BMP] Failed to get I2C Mutex");
      }
      vTaskDelay(pdMS_TO_TICKS(5000));
  }
}


// --- Task: Read MPU6050 ---
void taskReadMPU6050(void *pv) {
  (void)pv;
  sensors_event_t a, g, temp;
  struct tm timeinfo;
  for(;;) {
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
          mpu.getEvent(&a, &g, &temp);
          xSemaphoreGive(i2cMutex);


          if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
              g_sensorData.accel_x = a.acceleration.x;
              g_sensorData.accel_y = a.acceleration.y;
              g_sensorData.accel_z = a.acceleration.z;
              g_sensorData.gyro_x = g.gyro.x;
              g_sensorData.gyro_y = g.gyro.y;
              g_sensorData.gyro_z = g.gyro.z;
              if (getLocalTime(&timeinfo, 1000)) {
                  strftime(g_sensorData.mpu_timestamp, 32, "%Y-%m-%d %H:%M:%S", &timeinfo);
              }
              xSemaphoreGive(dataMutex);
          }
      } else {
          Serial.println("[MPU] Failed to get I2C Mutex");
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


// --- Task: Read SHT4x ---
void taskReadSHT4x(void *pv) {
  (void)pv;
  sensors_event_t humidity, temp;
  struct tm timeinfo;
  for(;;) {
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
          sht4.getEvent(&humidity, &temp);
          xSemaphoreGive(i2cMutex);


          if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
              g_sensorData.sht_temp = temp.temperature;
              g_sensorData.sht_humidity = humidity.relative_humidity;
              if (getLocalTime(&timeinfo, 1000)) {
                  strftime(g_sensorData.sht_timestamp, 32, "%Y-%m-%d %H:%M:%S", &timeinfo);
              }
              xSemaphoreGive(dataMutex);
          }
      } else {
          Serial.println("[SHT] Failed to get I2C Mutex");
      }
      vTaskDelay(pdMS_TO_TICKS(5000));
  }
}


// --- Task: Run web server (HTTP) ---
void taskWebServer(void *pv) {
  (void)pv;
  Serial.println("[WebServer] Task started.");
  for(;;) {
      server.handleClient();
      vTaskDelay(pdMS_TO_TICKS(10));
  }
}


// --- Handle "/" request (HTTP) ---
void handleRoot() {
  char html[2048];
  SensorData localData;


  // Copy global data safely
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      localData = g_sensorData;
      xSemaphoreGive(dataMutex);
  } else {
      server.send(503, "text/plain", "Service Unavailable (Mutex lock)");
      return;
  }


  // Generate HTML
  snprintf(html, sizeof(html),
      "<!DOCTYPE html><html><head>"
      "<title>Cucumber RS - Sensor Hub</title>"
      "<meta http-equiv='refresh' content='5'>"
      "<style>"
      "body { font-family: Arial, sans-serif; background: #f0f0f0; margin: 20px; }"
      "h1 { color: #333; text-align: center; }"
      ".container { max-width: 800px; margin: auto; background: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }"
      ".sensor-group { margin-bottom: 20px; border: 1px solid #ddd; border-radius: 5px; padding: 15px; }"
      "h2 { color: #0056b3; border-bottom: 2px solid #0056b3; padding-bottom: 5px; }"
      ".data { font-size: 1.1em; line-height: 1.6; }"
      ".timestamp { font-size: 0.9em; color: #777; }"
      "</style></head>"
      "<body><div class='container'>"
      "<h1>Cucumber RS - Sensor Hub</h1>"


      // BMP280 Group
      "<div class='sensor-group'><h2>BMP280 (Pressure/Temp)</h2>"
      "<div class='data'>"
      "Temperature: %.2f &deg;C<br/>"
      "Pressure: %.2f hPa"
      "</div><div class='timestamp'>Last Update: %s</div></div>"


      // MPU6050 Group
      "<div class='sensor-group'><h2>MPU6050 (Accel/Gyro)</h2>"
      "<div class='data'>"
      "Acceleration (X, Y, Z): %.2f, %.2f, %.2f m/s^2<br/>"
      "Gyroscope (X, Y, Z): %.2f, %.2f, %.2f rad/s"
      "</div><div class='timestamp'>Last Update: %s</div></div>"


      // SHT4x Group
      "<div class='sensor-group'><h2>SHT4x (Temp/Humidity)</h2>"
      "<div class='data'>"
      "Temperature: %.2f &deg;C<br/>"
      "Humidity: %.2f %%"
      "</div><div class='timestamp'>Last Update: %s</div></div>"


      "</div></body></html>",
      // BMP Data
      localData.bmp_temp, localData.pressure_hpa, localData.bmp_timestamp,
      // MPU Data
      localData.accel_x, localData.accel_y, localData.accel_z,
      localData.gyro_x, localData.gyro_y, localData.gyro_z,
      localData.mpu_timestamp,
      // SHT Data
      localData.sht_temp, localData.sht_humidity, localData.sht_timestamp
  );
 
  server.send(200, "text/html", html);
}




//=============================================================================
// (!!) TASK 6: NEW "Classroom" AES ENCRYPTED SERVER (!!)
//=============================================================================
void taskEncryptedServer(void *pv)
{
    (void)pv;
    Serial.println("[AES Server] Task started (Port 10000).");
   
    const int DATA_SIZE = sizeof(SensorData); // 144 bytes
   
    for(;;) // Loop forever to accept new clients
    {
        WiFiClient client = aesServer.available();
        if (client) {
            Serial.println("\n[AES Server] Client connected!");


            // --- 1. ทำ DH Handshake (ฉบับห้องเรียน) ---
            unsigned long long private_b = (esp_random() % 18) + 2; // สุ่ม 2-20
            unsigned long long public_B = power(G, private_b, P);
            Serial.printf("[DH] Server Private (b) = %llu, Public (B) = %llu\n", private_b, public_B);


            unsigned long long public_A = 0;
            unsigned long long shared_S = 0;


            // ข. รอรับ Public Key (A) จาก Client (1 byte)
            Serial.println("[DH] Waiting for Client Public Key (A)...");
            while(!client.available()) {
                delay(10);
                if (!client.connected()) {
                    Serial.println("Client disconnected before handshake.");
                    goto client_cleanup;
                }
            }
            public_A = client.read();
            Serial.printf("[DH] Received Client Public (A) = %llu\n", public_A);


            // ค. ส่ง Public Key (B) ของเรากลับไป (1 byte)
            client.write((uint8_t)public_B);
            Serial.println("[DH] Sent Server Public (B).");


            // ง. คำนวณกุญแจลับร่วม
            shared_S = power(public_A, private_b, P);
            Serial.printf("[DH] Shared Secret (S) = %llu. Handshake complete.\n", shared_S);


            // --- 2. ยืด (Derive) กุญแจ AES ด้วย SHA-256 ---
            unsigned char derived_aes_key[32]; // <--- กุญแจ AES อยู่ในนี้
            unsigned char shared_s_bytes[8];
           
            memcpy(shared_s_bytes, &shared_S, 8);
            mbedtls_sha256(shared_s_bytes, 8, derived_aes_key, 0);
            Serial.println("[AES] Derived key from SHA-256.");
           
            // (ผมลบโค้ดพิมพ์ Key ตรงนี้ ออกไปย้ายไว้ใน Loop แทน)


            // --- 3. เริ่ม Loop ส่งข้อมูล (ใช้กุญแจที่เพิ่งสร้าง) ---
            while (client.connected()) {
               
                // --- 3a. คัดลอกข้อมูล Sensor (ใช้ Mutex) ---
                SensorData localData;
                if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                    localData = g_sensorData;
                    xSemaphoreGive(dataMutex);
                } else {
                    Serial.println("[AES Server] Failed to get Data Mutex!");
                    vTaskDelay(pdMS_TO_TICKS(5000));
                    continue;
                }
               
                // --- 3b. เตรียมเข้ารหัส ---
                unsigned char iv[16];
                unsigned char iv_copy_for_sending[16];
                unsigned char encrypted_output[DATA_SIZE];
               
                for(int i=0; i<16; i++) { iv[i] = esp_random() % 256; }
                memcpy(iv_copy_for_sending, iv, 16);


                // --- 3c. เข้ารหัส (144 bytes) ---
                mbedtls_aes_context aes;
                mbedtls_aes_setkey_enc(&aes, derived_aes_key, 256); // ใช้กุญแจ DH
                mbedtls_aes_crypt_cbc(
                    &aes, MBEDTLS_AES_ENCRYPT,
                    DATA_SIZE, iv,
                    (unsigned char*)&localData,
                    encrypted_output
                );
                mbedtls_aes_free(&aes);


                // --- 3d. ส่ง (IV 16 bytes + Data 144 bytes) ---
                client.write(iv_copy_for_sending, 16);
                client.write(encrypted_output, DATA_SIZE);
               
                Serial.println("[AES Server] Sent 16-byte IV + 144-byte Encrypted Sensor Data");


                // --- (!!) NEW: พิมพ์ Key ทุกรอบที่ส่งข้อมูล (!!) ---
                Serial.print("[AES] (Current) Derived Key (Hex): ");
                for(int i=0; i<32; i++) {
                    if(derived_aes_key[i] < 0x10) {
                        Serial.print("0"); // พิมพ์ 0 นำหน้า
                    }
                    Serial.print(derived_aes_key[i], HEX);
                }
                Serial.println();
                // ---------------------------------------------------
               
                vTaskDelay(pdMS_TO_TICKS(5000)); // รอ 5 วินาที
            }
           
            client_cleanup:
            Serial.println("[AES Server] Client disconnected.");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
