// ==============================
//  ESP32 Sensor Hub - Refactored
// ==============================

#include <WiFi.h>
#include <time.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include "Adafruit_SHT4x.h"
#include <Adafruit_Sensor.h>
#include <WebServer.h>
#include <mbedtls/aes.h>
#include <mbedtls/sha256.h>
#include <string.h>

// --- Mutex Handles ---
SemaphoreHandle_t mutexI2C = NULL;
SemaphoreHandle_t mutexData = NULL;

// --- Sensor Objects ---
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
Adafruit_SHT4x sht4;

// --- Servers ---
WebServer httpServer(80);
WiFiServer tcpServer(10000);

// --- Classroom DH Parameters ---
const uint64_t DH_P = 23;
const uint64_t DH_G = 5;

// --- Sensor Data Struct (144 bytes) ---
struct SensorSnapshot {
    float bmp_temp;
    float bmp_pressure;
    char bmp_time[32];

    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    char mpu_time[32];

    float sht_temp;
    float sht_humidity;
    char sht_time[32];

    char padding[8]; // Total 144 bytes
};
SensorSnapshot sharedData;

// --- WiFi & Time Settings ---
const char* WIFI_SSID = "GalaxyA";
const char* WIFI_PASS = "hiphatt65";
const long GMT_OFFSET = 7 * 3600;
const int DST_OFFSET = 0;

// --- Task Handles ---
TaskHandle_t tPrint = NULL;
TaskHandle_t tBMP = NULL;
TaskHandle_t tMPU = NULL;
TaskHandle_t tSHT = NULL;
TaskHandle_t tHTTP = NULL;
TaskHandle_t tTCP = NULL;

// ===============================
// Helper: Modular Exponentiation
// ===============================
uint64_t modExp(uint64_t base, uint64_t exp, uint64_t mod) 
{
    uint64_t result = 1;
    base %= mod;
    while(exp > 0) 
    {
        if(exp & 1) result = (result * base) % mod;
        exp >>= 1;
        base = (base * base) % mod;
    }
    return result;
}

// ===============================
// Setup
// ===============================
void setup() 
{
    Serial.begin(115200);
    Serial.println("\n[Main] Booting...");

    mutexI2C = xSemaphoreCreateMutex();
    mutexData = xSemaphoreCreateMutex();

    // Init sharedData
    if(xSemaphoreTake(mutexData, pdMS_TO_TICKS(1000)) == pdTRUE) 
    {
        strcpy(sharedData.bmp_time, "N/A");
        strcpy(sharedData.mpu_time, "N/A");
        strcpy(sharedData.sht_time, "N/A");
        xSemaphoreGive(mutexData);
    }

    // I2C
    Wire.begin(41, 40);

    if(xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if(!bmp.begin(0x76)) Serial.println("[Sensor] BMP280 not found!");
        if(!mpu.begin()) Serial.println("[Sensor] MPU6050 not found!");
        if(!sht4.begin()) Serial.println("[Sensor] SHT4x not found!");
        xSemaphoreGive(mutexI2C);
    }

    // WiFi
    Serial.println("[Main] Connecting WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while(WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nWiFi connected");
    Serial.printf("[HTTP] IP: %s\n", WiFi.localIP().toString().c_str());

    // HTTP & TCP Servers
    httpServer.on("/", [](){ handleRoot(); });
    httpServer.begin();
    tcpServer.begin();
    Serial.println("[Main] Servers started.");

    // Time
    configTime(GMT_OFFSET, DST_OFFSET, "pool.ntp.org");
    sntp_set_sync_interval(0);
    sntp_restart();

    // Create Tasks
    xTaskCreate(taskPrintTime, "PrintTime", 4096, NULL, 1, &tPrint);
    xTaskCreate(taskBMP280, "BMP280Task", 4096, NULL, 1, &tBMP);
    xTaskCreate(taskMPU6050, "MPU6050Task", 8192, NULL, 1, &tMPU);
    xTaskCreate(taskSHT4x, "SHT4xTask", 4096, NULL, 1, &tSHT);
    xTaskCreate(taskHTTPServer, "HTTPServer", 8192, NULL, 2, &tHTTP);
    xTaskCreate(taskEncryptedTCP, "EncryptedTCP", 4096, NULL, 2, &tTCP);

    Serial.println("[Main] Setup complete.");
}

void loop() 
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// ===============================
// Tasks
// ===============================
void taskPrintTime(void* pv) {
    (void)pv;
    char buf[32];
    struct tm tinfo;
    for(;;) {
        if(getLocalTime(&tinfo, 5000)) {
            strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tinfo);
            Serial.printf("[Time] %s\n", buf);
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void taskBMP280(void* pv) 
{
    (void)pv;
    struct tm tinfo;
    for(;;) {
        if(xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(1000)) == pdTRUE) {
            float temp = bmp.readTemperature();
            float pres = bmp.readPressure() / 100.0F;
            xSemaphoreGive(mutexI2C);

            if(xSemaphoreTake(mutexData, pdMS_TO_TICKS(1000)) == pdTRUE) {
                sharedData.bmp_temp = temp;
                sharedData.bmp_pressure = pres;
                if(getLocalTime(&tinfo, 1000))
                    strftime(sharedData.bmp_time, 32, "%Y-%m-%d %H:%M:%S", &tinfo);
                xSemaphoreGive(mutexData);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void taskMPU6050(void* pv) 
{
    (void)pv;
    sensors_event_t a, g, temp;
    struct tm tinfo;
    while(true)
    {
        if(xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(1000)) == pdTRUE) {
            mpu.getEvent(&a, &g, &temp);
            xSemaphoreGive(mutexI2C);

            if(xSemaphoreTake(mutexData, pdMS_TO_TICKS(1000)) == pdTRUE) {
                sharedData.accel_x = a.acceleration.x;
                sharedData.accel_y = a.acceleration.y;
                sharedData.accel_z = a.acceleration.z;
                sharedData.gyro_x = g.gyro.x;
                sharedData.gyro_y = g.gyro.y;
                sharedData.gyro_z = g.gyro.z;
                if(getLocalTime(&tinfo, 1000))
                    strftime(sharedData.mpu_time, 32, "%Y-%m-%d %H:%M:%S", &tinfo);
                xSemaphoreGive(mutexData);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void taskSHT4x(void* pv) 
{
    (void)pv;
    sensors_event_t humidity, temp;
    struct tm tinfo;
    while(true) 
    {
        if(xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(1000)) == pdTRUE) 
        {
            sht4.getEvent(&humidity, &temp);
            xSemaphoreGive(mutexI2C);

            if(xSemaphoreTake(mutexData, pdMS_TO_TICKS(1000)) == pdTRUE) 
            {
                sharedData.sht_temp = temp.temperature;
                sharedData.sht_humidity = humidity.relative_humidity;
                if(getLocalTime(&tinfo, 1000))
                    strftime(sharedData.sht_time, 32, "%Y-%m-%d %H:%M:%S", &tinfo);
                xSemaphoreGive(mutexData);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ===============================
// HTTP Server
// ===============================
void taskHTTPServer(void* pv) 
{
    (void)pv;
    while(true) 
    {
        httpServer.handleClient();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void handleRoot() 
{
    char html[2048];
    SensorSnapshot snapshot;

    if(xSemaphoreTake(mutexData, pdMS_TO_TICKS(1000)) == pdTRUE) 
    {
        snapshot = sharedData;
        xSemaphoreGive(mutexData);
    } 
    else 
    {
        httpServer.send(503, "text/plain", "Service Unavailable");
        return;
    }

    snprintf(html, sizeof(html),
        "<html><head><meta http-equiv='refresh' content='5'></head><body>"
        "<h1>Sensor Hub</h1>"
        "<p>BMP280 Temp: %.2f C, Pressure: %.2f hPa (%s)</p>"
        "<p>MPU6050 Accel: %.2f, %.2f, %.2f m/s², Gyro: %.2f, %.2f, %.2f rad/s (%s)</p>"
        "<p>SHT4x Temp: %.2f C, Humidity: %.2f %% (%s)</p>"
        "</body></html>",
        snapshot.bmp_temp, snapshot.bmp_pressure, snapshot.bmp_time,
        snapshot.accel_x, snapshot.accel_y, snapshot.accel_z,
        snapshot.gyro_x, snapshot.gyro_y, snapshot.gyro_z, snapshot.mpu_time,
        snapshot.sht_temp, snapshot.sht_humidity, snapshot.sht_time
    );

    httpServer.send(200, "text/html", html);
}

// ===============================
// Encrypted TCP Server (Classroom DH + AES)
// ===============================
void taskEncryptedTCP(void* pv) {
    (void)pv;
    const size_t DATA_LEN = sizeof(SensorSnapshot);

    while(true) 
    {
        WiFiClient client = tcpServer.available();
        if(client) {
            Serial.println("[TCP] Client connected.");

            // --- DH Handshake ---
            uint64_t privB = (esp_random() % 18) + 2;
            uint64_t pubB = modExp(DH_G, privB, DH_P);
            Serial.printf("[DH] Server B=%llu\n", pubB);

            while(!client.available()) { delay(10); if(!client.connected()) goto end_client; }
            uint64_t pubA = client.read();
            client.write((uint8_t)pubB);

            uint64_t sharedS = modExp(pubA, privB, DH_P);
            Serial.printf("[DH] Shared S=%llu\n", sharedS);

            unsigned char aesKey[32], sBytes[8];
            memcpy(sBytes, &sharedS, 8);
            mbedtls_sha256(sBytes, 8, aesKey, 0);

            // --- Send Encrypted Data Loop ---
            while(client.connected()) 
            {
                SensorSnapshot localSnap;
                if(xSemaphoreTake(mutexData, pdMS_TO_TICKS(1000)) == pdTRUE) 
                {
                    localSnap = sharedData;
                    xSemaphoreGive(mutexData);
                } 
                else { vTaskDelay(pdMS_TO_TICKS(5000)); continue; }

                unsigned char iv[16]; for(int i=0;i<16;i++) iv[i]=esp_random()%256;
                unsigned char encrypted[DATA_LEN];

                mbedtls_aes_context aes;
                mbedtls_aes_setkey_enc(&aes, aesKey, 256);
                mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, DATA_LEN, iv, (unsigned char*)&localSnap, encrypted);
                mbedtls_aes_free(&aes);

                client.write(iv, 16);
                client.write(encrypted, DATA_LEN);

                Serial.println("[TCP] Sent encrypted sensor data.");
                vTaskDelay(pdMS_TO_TICKS(5000));
            }

            end_client:
            Serial.println("[TCP] Client disconnected.");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
