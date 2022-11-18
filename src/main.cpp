#include <Arduino.h>
#include "DFRobot_ESP_PH_WITH_ADC.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "Adafruit_ADS1X15.h"
#include "EEPROM.h"
#include "Platform.h"
#include "App_Common.h"
#include <ArduinoJson.h>
#include "secrets.h"

#if !(defined(ESP8266) || defined(ESP32))
#error This code is intended to run on the ESP8266 or ESP32 platform! Please check your Tools->Board setting.
#endif

#if (ESP8266)
#include <ESP8266WiFi.h>
#elif (ESP32)
#include <WiFi.h>
#endif

#define ASYNC_HTTPS_REQUEST_GENERIC_VERSION_MIN_TARGET "AsyncHTTPSRequest_Generic v2.2.1"
#define ASYNC_HTTPS_REQUEST_GENERIC_VERSION_MIN 2002001

// Level from 0-4
#define ASYNC_HTTPS_DEBUG_PORT Serial

#define _ASYNC_TCP_SSL_LOGLEVEL_ 1
#define _ASYNC_HTTPS_LOGLEVEL_ 1

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include <AsyncHTTPSRequest_Generic.h> // https://github.com/khoih-prog/AsyncHTTPSRequest_Generic

AsyncHTTPSRequest request;

int status; // the Wifi radio's status

// Use larger queue size if necessary for large data transfer. Default is 512 bytes if not defined here
//#define ASYNC_QUEUE_LENGTH     512

// Use larger priority if necessary. Default is 10 if not defined here. Must be > 4 or adjusted to 4
//#define CONFIG_ASYNC_TCP_PRIORITY   (12)


//NTP settings
const char *ntpServer1 = "ntp.np.edu.sg";
const char *ntpServer2 = "pool.ntp.org";
const long gmtOffset_sec = 28800;
const int daylightOffset_sec = 0;

//Temp sensor settings
#define ONE_WIRE_BUS 15
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define LEDR 4 // Red LED ESP32 Pin 22

//ph sensor settings
DFRobot_ESP_PH_WITH_ADC ph;
#define ESPADC 4096.0   // the esp Analog Digital Convertion value
#define ESPVOLTAGE 5000 // the esp voltage supply value
#define PH_PIN 35       // the esp gpio data pin number
Adafruit_ADS1115 ads;

//LCD display settings
/* Global used for buffer optimization */
Gpu_Hal_Context_t host, *phost;

float voltage, phValue, temperature = 25;

// Millis for non-blocking
unsigned long startMillis; // some global variables available anywhere in the program
unsigned long Temp_startMillis;
unsigned long Ph_startMillis;
unsigned long Send_startMillis;
unsigned long currentMillis;
unsigned long tempMillis;
unsigned long phMillis;
unsigned long sendMillis;

unsigned long feedPeriod = 5000;  // the value is a number of milliseconds
unsigned long motorPeriod = 1000; // the value is a number of milliseconds
unsigned long tempPeriod = 10000; // the value is a number of milliseconds
unsigned long phPeriod = 10000;   // the value is a number of milliseconds

const int motor1pin1 = 33;
const int motor1pin2 = 25;
const int IRpin = 13;
bool flag = true;
int Count;
int Mode = 1;

//Time settings
struct tm timeinfo;
char strTime[51];

//JSON settings
StaticJsonDocument<192> Doc;
String sendJson;

float readTemperature()
{
  // add your code here to get the temperature from your temperature sensor
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

void IRAM_ATTR IRsensor()
{
  Count++;
  Mode = 1;
}

void showOnLcd()
{
  Gpu_CoCmd_Dlstart(phost);
  App_WrCoCmd_Buffer(phost, CLEAR(1, 1, 1));
  Gpu_CoCmd_Text(phost, 30, 55, 29, 0, "pH Level:");
  Gpu_CoCmd_Text(phost, 30, 121, 29, 0, "Temperature:");
  Gpu_CoCmd_Number(phost, 200, 55, 29, 0, int(phValue));
  Gpu_CoCmd_Number(phost, 200, 121, 29, 0, int(temperature));
  App_WrCoCmd_Buffer(phost, GPU_DISPLAY());
  Gpu_CoCmd_Swap(phost);
  App_Flush_Co_Buffer(phost);
  Gpu_Hal_WaitCmdfifo_empty(phost);
}

void sendRequest(String sendData)
{
  static bool requestOpenResult;

  if (request.readyState() == readyStateUnsent || request.readyState() == readyStateDone)
  {
    // requestOpenResult = request.open("GET", "https://worldtimeapi.org/api/timezone/Europe/London.txt");
    // requestOpenResult = request.open("GET", "https://worldtimeapi.org/api/timezone/America/Toronto.txt");
    requestOpenResult = request.open("POST", firebase_url);

    if (requestOpenResult)
    {
      // Only send() if open() returns true, or crash
      // request.send("{\"testing\":\"true\"}");
      request.send(sendData);
    }
    else
    {
      Serial.println(F("Can't send bad request"));
    }
  }
  else
  {
    Serial.println(F("Can't send request"));
  }
}

void requestCB(void *optParm, AsyncHTTPSRequest *request, int readyState)
{
  (void)optParm;

  if (readyState == readyStateDone)
  {
    AHTTPS_LOGDEBUG0(F("\n**************************************\n"));
    AHTTPS_LOGDEBUG1(F("Response Code = "), request->responseHTTPString());

    if (request->responseHTTPcode() == 200)
    {
      Serial.println(F("\n**************************************"));
      Serial.println(request->responseText());
      Serial.println(F("**************************************"));
    }

    request->setDebug(false);
  }
}

String getTime()
{
  if (!getLocalTime(&timeinfo))
  { // if did not get time
    Serial.println("Failed to obtain time");
    return "Failed to obtain time";
  }
  else
  {
    // Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    strftime(strTime, 50, "%Y-%m-%d %H:%M:%S", &timeinfo);
    return strTime;
  }
}

void setup()
{

  Serial.begin(115200);

  attachInterrupt(13, IRsensor, FALLING);
  startMillis = millis();      // initial start time
  Temp_startMillis = millis(); // initial start time

  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);

  pinMode(32, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(IRpin, INPUT);

  // Controlling spin direction of motors:
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  analogWrite(32, 255); // ENA pin`

  EEPROM.begin(32); // needed EEPROM.begin to store calibration k in eeprom
  ph.begin();
  sensors.begin();
  ads.setGain(GAIN_ONE);
  ads.begin();

  phost = &host;
  /* Init HW Hal */
  App_Common_Init(&host);

  Serial.print(F("\nStarting AsyncHTTPSRequest_ESP using "));
  Serial.println(ARDUINO_BOARD);

#if defined(ESP32)
  Serial.println(ASYNC_TCP_SSL_VERSION);
#else
  // Serial.println(ESPASYNC_TCP_SSL_VERSION);
#endif

  Serial.println(ASYNC_HTTPS_REQUEST_GENERIC_VERSION);

#if defined(ASYNC_HTTPS_REQUEST_GENERIC_VERSION_MIN)
  if (ASYNC_HTTPS_REQUEST_GENERIC_VERSION_INT < ASYNC_HTTPS_REQUEST_GENERIC_VERSION_MIN)
  {
    Serial.print(F("Warning. Must use this example on Version equal or later than : "));
    Serial.println(ASYNC_HTTPS_REQUEST_GENERIC_VERSION_MIN_TARGET);
  }
#endif

  WiFi.mode(WIFI_STA);

  WiFi.begin(ssid, password);

  Serial.print(F("Connecting to WiFi SSID: "));
  Serial.println(ssid);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.print(F("\nAsyncHTTPSRequest @ IP : "));
  Serial.println(WiFi.localIP());

  request.setDebug(false);

  request.onReadyStateChange(requestCB);

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1);
}

void loop()
{
  currentMillis = millis();
  tempMillis = millis();
  phMillis = millis();

  if (tempMillis - Temp_startMillis >= tempPeriod)
  {
    temperature = readTemperature(); // read your temperature sensor to execute temperature compensation
    Serial.print("Temperature:");
    Serial.print(temperature, 1);
    Serial.println("^C");
    Temp_startMillis = tempMillis;
    showOnLcd();
    sendJson = ""; // clear string
    Doc.clear();   // release memory used by JsonObject
    Doc["time"] = getTime();
    Doc["value"] = temperature;
    serializeJson(Doc, sendJson);
    sendRequest(sendJson);
  }
  if (phMillis - Ph_startMillis >= phPeriod)
  {
    // temperature = readTemperature();                     //needed for temperature compensation below
    // voltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE;  // read the voltage
    // Serial.println(voltage);
    // phValue = ph.readPH(voltage, temperature);  // convert voltage to pH with temperature compensation
    // phValue = random(10);
    phValue = map(analogRead(PH_PIN), 0, 4096, 0, 14);
    Serial.print("pH:");
    Serial.println(phValue, 4);
    Ph_startMillis = phMillis;
    showOnLcd();
  }
  if ((currentMillis - startMillis >= feedPeriod) && flag == true && Mode == 1)
  {
    digitalWrite(motor1pin2, LOW);
    digitalWrite(LEDR, LOW);
    flag = false;
    startMillis = currentMillis;
  }
  else if ((currentMillis - startMillis >= motorPeriod) && flag == false && Mode == 1)
  {
    digitalWrite(motor1pin2, HIGH);
    flag = true;
    startMillis = currentMillis;

    if (Count == 0)
      Mode = 0;

    else
      Count = 0;
  }
  else if (Mode == 0)
  {
    digitalWrite(LEDR, HIGH);
    digitalWrite(motor1pin2, HIGH);
  }
}