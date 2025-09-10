#include <WiFi.h>
#include "time.h"
#include "esp_sntp.h"

const char *ssid = "YOUR SSID";
const char *password = "PASSWORD";

const char *ntpServer1 = "pool.ntp.org";
const char *ntpServer2 = "time.nist.gov";
const long gmtOffset_sec = 3600*7; // Thailand is GMT+7
const int daylightOffset_sec = 0;

// Print current local time
void printLocalTime() 
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) 
  {
    Serial.println("No time available (yet)");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

// Task 1: print clock every second
void Thread_1(void *parameter) 
{
  while(1)
  {
    printLocalTime();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Task 2: update time from NTP every minute
void Thread_2(void *parameter) 
{
  while(1) 
  {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
    Serial.println("Time updated from NTP server.");
    vTaskDelay(60000 / portTICK_PERIOD_MS);
  }
}

// Callback function (gets called when time adjusts via NTP)
void timeavailable(struct timeval *t) 
{
  Serial.println("Got time adjustment from NTP!");
  printLocalTime();
}

void setup() 
{
  Serial.begin(115200);
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  esp_sntp_servermode_dhcp(1);  // (optional)

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("CONNECTED");

  sntp_set_time_sync_notification_cb(timeavailable);

  // Initial time sync
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
  printLocalTime();

  // Create tasks (Thread_2 has higher priority)
  #if CONFIG_FREERTOS_UNICORE
    #define RUNNING_CORE 0
  #else
    #define RUNNING_CORE 1
  #endif

  xTaskCreatePinnedToCore(Thread_1, "ClockDisplay", 4096, NULL, 1, NULL, RUNNING_CORE);
  xTaskCreatePinnedToCore(Thread_2, "NTPUpdate", 4096, NULL, 2, NULL, RUNNING_CORE);
}

void loop() {}
