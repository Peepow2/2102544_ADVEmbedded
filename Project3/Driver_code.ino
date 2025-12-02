//----------library Wifi and time----------
#include <WiFi.h>
#include "time.h"
//--------------library Sensor--------------
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
//--------Wifi and password-----------
const char* ssid     = "Your SSID";
const char* password = "Your PASSWORD";
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

// Timezone and Daylight saving and time stamp
const char* ntpServer = "pool.ntp.org";
const int   GMT = 7 ;
const long  gmtOffset_sec = (GMT)*3600;
const int   daylightOffset_sec = 3600;
//------Sensor variable------------------------------
// BMP280
float bmp_temp = 0.0;
float pressure = 0.0;
float height = 0.0;
// MPU6050
float accel_x = 0.0;
float accel_y = 0.0;
float accel_z = 0.0;
float gyro_x = 0.0;
float gyro_y = 0.0;
float gyro_z = 0.0;
float mpu_temp = 0.0;

// ------------------- HTML Template for web server-------------------
const char* htmlTemplate = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta http-equiv="refresh" content="5">
  <title>Sensor Data</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; margin: 0; padding-top: 20px; background-color: #f4f4f9; }
    h1 { color: #333; }
    table { margin: 20px auto; background: #fff; padding: 10px; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }
    caption { font-size: 1.2em; margin-bottom: 5px; font-weight: bold; }
    td { padding: 6px 12px; font-size: 1.1em; }
    .red { color: #dc3545; }
    .green { color: #28a745; }
    .blue { color: #007bff; }
  </style>
</head>
<body>
  <h1>Sensor Data</h1>
  <p>Last updated: %TIME%</p>
  <p>Refresh every 5 seconds.</p>

  <table>
    <caption>BMP280 Temperature and Pressure</caption>
    <tr><td>Temperature</td><td><span class="red">%BMP_TEMP% °C</span></td></tr>
    <tr><td>Pressure</td><td><span class="blue">%PRESSURE% hPa</span></td></tr>
    <tr><td>Height</td><td><span class="green">%HEIGHT% m</span></td></tr>
  </table>

  <table>
    <caption>MPU6050 Accelerometer Gyroscope and Temperature</caption>
    <tr><td>X Acceleration</td><td><span class="red">%ACCEL_X% m/s²</span></td></tr>
    <tr><td>Y Acceleration</td><td><span class="blue">%ACCEL_Y% m/s²</span></td></tr>
    <tr><td>Z Acceleration</td><td><span class="green">%ACCEL_Z% m/s²</span></td></tr>
    <tr><td>X Rotation</td><td><span class="red">%GYRO_X% rad/s</span></td></tr>
    <tr><td>Y Rotation</td><td><span class="blue">%GYRO_Y% rad/s</span></td></tr>
    <tr><td>Z Rotation</td><td><span class="green">%GYRO_Z% rad/s</span></td></tr>
    <tr><td>Temperature</td><td><span class="green">%MPU_TEMP% °C</span></td></tr>
  </table>

</body>
</html>
)rawliteral";
// ----------------------------------------------------------------------
// -------------------For replace current sensor value in web server------------------------
String processHTML(const String& html) {
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
//-----Convert Time to string to replace in web server------------
String getTimeString() {
  struct tm timeinfo;
  //Sync time
  if (!getLocalTime(&timeinfo)) {
    return "Time Sync Failed";
  }
  //Put time in char array
  char timeString[60]; // สร้าง Buffer ขนาดใหญ่พอสมควร (60 ตัวอักษร)
  strftime(timeString, sizeof(timeString), "%H:%M:%S %A, %B %d %Y", &timeinfo);
 
  //COnvert char into string
  return String(timeString);
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // --------------Wifi Setup-------------------------
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  //------------------Config Time-----------------------
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);


  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();


  //---------MPU6050 Connection------------------------------
  Serial.println("--- MPU-6050 Test on Cucumber Board ---");
  //Begin I2C
  Wire.begin(SDApin, SCLpin);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU-6050 chip");
    while (1) {
      delay(100);
    }
    }
  Serial.println("MPU-6050 Initialized Successfully");
  //Set Range of reading
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);  
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);        
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);    
 
  Serial.println("--------------------------------------");
  //---------BMP280 Connection----------------------------
  Serial.println("--- BMP280 Test on Cucumber Board ---");
  if (!bmp.begin(0x76)) {
    Serial.println("Failed to initialize BMP280 sensor");
    while (1) {
      delay(100);
    }
  }
  Serial.println("BMP280 Initialized Successfully ");
  Serial.println("--------------------------------------");
  //----------------------------------------------------------------------------------------------------------
  //---------------Create Task---------------------------------
  //-------Web Server--------------------------
   xTaskCreate(
  web_server, //Function
  "web_server",//Name
  8192,//Stack size
  NULL,//Parameter
  3, //Highest Priority
  NULL//Task handle
  );
  //---------NTP Server Task-----------------------
  xTaskCreate(
  ntp_sync, //Function
  "NTP_Sync",//Name
  6144,//Stack size
  NULL,//Parameter
  2, //Second Priority
  NULL//Task handle
  );
  //--------------BMP280  reading---------------
  xTaskCreate(
  bmp_reading, //Function
  "bmp_reading",//Name
  4096,//Stack size
  NULL,//Parameter
  1, //Lowest Priority
  NULL//Task handle
  );
  //---------------MPU6050 Reading-------------------
  xTaskCreate(
  mpu_reading, //Function
  "mpu_reading",//Name
  4096,//Stack size
  NULL,//Parameter
  1, //Lowest Priority
  NULL//Task handle
  );
}
void loop() {
  // put your main code here, to run repeatedly:
  //Only delay for 10 ms
  delay(10);
}
//---------Task------------------------------------------
//--------------Web Server-------------------------------
void web_server(void *parameter)
{
    for(;;){
    //Start respone to HTTP request
    WiFiClient client = server.available();
    if (client) {
        currentTime = millis();
        previousTime = currentTime;
        Serial.println("New Client.");
        String currentLine = "";


        // Check connection and timeout loop
        while (client.connected() && currentTime - previousTime <= timeoutTime) {
            currentTime = millis();
            if (client.available()) {
                char c = client.read();
                header += c;
                if (c == '\n') {
                    if (currentLine.length() == 0) {
                       
                        // Send HTTP Headers
                        client.println("HTTP/1.1 200 OK");
                        client.println("Content-type:text/html");
                        client.println("Connection: close");
                        client.println();


                        // Send HTTP with the update sensor value
                        client.print(processHTML(htmlTemplate));
                        break;
                    } else {
                        currentLine = "";
                    }
                } else if (c != '\r') {
                    currentLine += c;
                }
            }
        }
        header = "";
        // Disconnect the web server
        client.stop();
        Serial.println("Client disconnected.");
        Serial.println("");
    }
    vTaskDelay(10/portTICK_PERIOD_MS); //Pause this task for 10 ms
  }
}


//-----NTP server Sync-----------------------------------
void ntp_sync(void *parameter){
    for(;;){
    Serial.println("Config Time with NTP Server...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); // Synchronize time
 
    Serial.println("Synchronization Complete");
    vTaskDelay(60000/portTICK_PERIOD_MS); //Sync every 60 s
  }
}
//--------BMP280 Sensor Reading--------------------
void bmp_reading(void *parameter){
    for(;;){
    Serial.println("Reading BMP280...");
    bmp_temp = bmp.readTemperature(); //Temperature
    pressure = bmp.readPressure() / 100.0F; // Pressure (Pa to hPa)
    height = bmp.readAltitude(1013.25); // Height calculated from pressure at sea level
   
    vTaskDelay(10000/portTICK_PERIOD_MS); //Read the sensor every 10 s
  }
}


//--------MPU6050 Sensor Reading--------------------
void mpu_reading(void *parameter){
    for(;;){
      Serial.println("Reading MPU6050...");
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
 
      accel_x = a.acceleration.x; // X Acceleration
      accel_y = a.acceleration.y; // Y Acceleration
      accel_z = a.acceleration.z; // Z Acceleration
      gyro_x = g.gyro.x; // X Rotation
      gyro_y = g.gyro.y; // Y Rotation
      gyro_z = g.gyro.z; // Z Rotation
      mpu_temp = temp.temperature; // MPU Temperature
       
    vTaskDelay(10000/portTICK_PERIOD_MS); //Read the sensor every 10 s
  }
}

