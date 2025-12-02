//----------library Wifi and time----------
#include <WiFi.h>
#include "time.h"
#include "SPIFFS.h"

//--------------library Sensor--------------
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>

//--------Wifi and password-----------
const char* ssid     = "YOUR SSID";
const char* password = "PASSWORD";

//---------I2C Pin------------------------
const int SDApin = 41;
const int SCLpin = 40;

//-----Set web server port number to 80-----
WiFiServer server(80);

//-----Variable to store the HTTP request------
String header;

Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;

//------------------Server time-------------------
unsigned long currentTime = millis();
unsigned long previousTime = 0;
const long timeoutTime = 2000;

// Timezone
const char* ntpServer = "pool.ntp.org";
const int   GMT = 7 ;
const long  gmtOffset_sec = (GMT)*3600;
const int   daylightOffset_sec = 3600;

// TWeb server file
String INDEX = "..//WEB_index.html"

//------Sensor variable------------------------------
float bmp_temp = 0.0;
float pressure = 0.0;
float height = 0.0;

float accel_x = 0.0;
float accel_y = 0.0;
float accel_z = 0.0;
float gyro_x = 0.0;
float gyro_y = 0.0;
float gyro_z = 0.0;
float mpu_temp = 0.0;


// ------------------- Function to load HTML from SPIFFS -------------------
String loadHTML(const char* path) 
{
    File file = SPIFFS.open(path, "r");
    if (!file) 
    {
        Serial.println("Failed to open HTML file");
        return "";
    }

    String html = file.readString();
    file.close();
    return html;
}

// ------------------- Replace sensor values ------------------------
String getTimeString() 
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) 
    return "Time Sync Failed";

  char timeString[60];
  strftime(timeString, sizeof(timeString), "%H:%M:%S %A, %B %d %Y", &timeinfo);
  return String(timeString);
}

String processHTML(const String& html) 
{
  String tempHtml = html;

  // BMP280
  tempHtml.replace("%BMP_TEMP%", String(bmp_temp, 2));
  tempHtml.replace("%PRESSURE%", String(pressure, 2));
  tempHtml.replace("%HEIGHT%", String(height, 2));

  // MPU6050
  tempHtml.replace("%ACCEL_X%", String(accel_x, 2));
  tempHtml.replace("%ACCEL_Y%", String(accel_y, 2));
  tempHtml.replace("%ACCEL_Z%", String(accel_z, 2));
  tempHtml.replace("%GYRO_X%", String(gyro_x, 2));
  tempHtml.replace("%GYRO_Y%", String(gyro_y, 2));
  tempHtml.replace("%GYRO_Z%", String(gyro_z, 2));
  tempHtml.replace("%MPU_TEMP%", String(mpu_temp, 2));

  // Time
  tempHtml.replace("%TIME%", getTimeString());
  return tempHtml;
}
// ------------------------------------
//               SETUP
// ------------------------------------
void setup() 
{
  Serial.begin(115200);

  // ----- MOUNT SPIFFS -----
  if(!SPIFFS.begin(true))
    Serial.println("SPIFFS Mount Failed");
  else 
    Serial.println("SPIFFS Mounted Successfully");

  // --------------Wifi Setup-------------------------
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }

  // Time setup
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();

  //---------MPU6050 Connection------------------------------
  Serial.println("--- MPU-6050 Test on Cucumber Board ---");
  Wire.begin(SDApin, SCLpin);
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU-6050 chip");
    Serial.println("Restarting");
    ESP.restart();
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);  
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);        
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 
  Serial.println("MPU-6050 Initialized Successfully");

  //---------BMP280 Connection----------------------------
  if (!bmp.begin(0x76)) {
    Serial.println("Failed to initialize BMP280 sensor");
    Serial.println("Restarting");
    ESP.restart();
  }
  Serial.println("BMP280 Initialized Successfully ");

  // ---------------- CREATE TASKS ----------------
  xTaskCreate(web_server, "web_server", 8192, NULL, 3, NULL);
  xTaskCreate(ntp_sync,   "NTP_Sync",  6144, NULL, 2, NULL);
  xTaskCreate(bmp_reading,"bmp_reading",4096,NULL,1,NULL);
  xTaskCreate(mpu_reading,"mpu_reading",4096,NULL,1,NULL);
}

void loop() 
{
  delay(10);
}


// -------------------------------------------
//                 TASKS
// -------------------------------------------

//--------------Web Server-------------------------------
void web_server(void *parameter)
{
    while(true)
    {
        WiFiClient client = server.available();
        if (client) {
            currentTime = millis();
            previousTime = currentTime;
            Serial.println("New Client.");
            String currentLine = "";

            while (client.connected() && currentTime - previousTime <= timeoutTime) 
            {
                currentTime = millis();
                if (client.available()) 
                {
                    char c = client.read();
                    header += c;

                    if (c == '\n') 
                    {
                        if (currentLine.length() == 0) 
                        {

                            client.println("HTTP/1.1 200 OK");
                            client.println("Content-type:text/html");
                            client.println("Connection: close");
                            client.println();

                            // LOAD + REPLACE HTML
                            String html = loadHTML(INDEX);
                            String processed = processHTML(html);
                            client.print(processed);
                 
                            break;
                        } 

                        else 
                        {
                            currentLine = "";
                        }
                    } 

                    else if (c != '\r') {
                        currentLine += c;
                    }
                }
            }

            header = "";
            client.stop();
            Serial.println("Client disconnected.");
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

//-----NTP server Sync-----------------------------------
void ntp_sync(void *parameter)
{
    while(true)
    {
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        vTaskDelay(60000/portTICK_PERIOD_MS);
    }
}

//--------BMP280 Sensor Reading--------------------
void bmp_reading(void *parameter)
{
    while(true)
    {
        bmp_temp = bmp.readTemperature();
        pressure = bmp.readPressure() / 100.0F;
        height = bmp.readAltitude(1013.25);
       
        vTaskDelay(10000/portTICK_PERIOD_MS);
    }
}

//--------MPU6050 Sensor Reading--------------------
void mpu_reading(void *parameter)
{
    while(true)
    {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        accel_x = a.acceleration.x;
        accel_y = a.acceleration.y;
        accel_z = a.acceleration.z;

        gyro_x = g.gyro.x;
        gyro_y = g.gyro.y;
        gyro_z = g.gyro.z;

        mpu_temp = temp.temperature;
       
        vTaskDelay(10000/portTICK_PERIOD_MS);
    }
}
