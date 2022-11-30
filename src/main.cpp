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

// Use larger queue size if necessary for large data transfer. Default is 512 bytes if not defined here
//#define ASYNC_QUEUE_LENGTH     512

// Use larger priority if necessary. Default is 10 if not defined here. Must be > 4 or adjusted to 4
//#define CONFIG_ASYNC_TCP_PRIORITY   (12)

#define NUM_DIFFERENT_SITES     2

const char* addreses[][NUM_DIFFERENT_SITES] =
{
	{firebase_url_temp},
  {firebase_url_ph}
	
};

#define NUM_ENTRIES_SITE_0        1
#define NUM_ENTRIES_SITE_1        1

byte reqCount[NUM_DIFFERENT_SITES]  = { NUM_ENTRIES_SITE_0, NUM_ENTRIES_SITE_1 };
bool readySend[NUM_DIFFERENT_SITES] = { true, true };

AsyncHTTPSRequest request[NUM_DIFFERENT_SITES];
int status; // the Wifi radio's status

void requestCB0(void* optParm, AsyncHTTPSRequest* thisRequest, int readyState);
void requestCB1(void* optParm, AsyncHTTPSRequest* thisRequest, int readyState);

void sendRequest0();
void sendRequest1();

typedef void (*requestCallback)(void* optParm, AsyncHTTPSRequest* thisRequest, int readyState);
typedef void (*sendCallback)();

requestCallback requestCB     [NUM_DIFFERENT_SITES] = { requestCB0,   requestCB1   };
sendCallback    sendRequestCB [NUM_DIFFERENT_SITES] = { sendRequest0, sendRequest1 };

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
unsigned long tempPeriod = 13000; // the value is a number of milliseconds
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

void sendRequest(uint16_t index)
{
	static bool requestOpenResult;

	reqCount[index]--;
	readySend[index] = false;

	requestOpenResult = request[index].open("POST", addreses[index][reqCount[index]]);

	if (requestOpenResult)
	{
		// Only send() if open() returns true, or crash
		Serial.print("\nSending request: ");
		request[index].send(sendJson);
	}
	else
	{
		Serial.print("\nCan't send bad request : ");
	}

	Serial.println(addreses[index][reqCount[index]]);
}

void sendRequest0()
{
	sendRequest(0);
}

void sendRequest1()
{
	sendRequest(1);
}

void sendRequests()
{
	for (int index = 0; index < NUM_DIFFERENT_SITES; index++)
	{
		reqCount[index] = 2;
	}

	reqCount[0] = NUM_ENTRIES_SITE_0;
	reqCount[1] = NUM_ENTRIES_SITE_1;
}

void requestCB0(void *optParm, AsyncHTTPSRequest *thisRequest, int readyState)
{
	(void) optParm;

	if (readyState == readyStateDone)
	{
		AHTTPS_LOGDEBUG0(F("\n**************************************\n"));
		AHTTPS_LOGDEBUG1(F("Response Code = "), thisRequest->responseHTTPString());

		if (thisRequest->responseHTTPcode() == 200)
		{
			Serial.println(F("\n**************************************"));
			Serial.println(thisRequest->responseText());
			Serial.println(F("**************************************"));
		}

		thisRequest->setDebug(false);
		readySend[0] = true;
	}
}

void requestCB1(void *optParm, AsyncHTTPSRequest *thisRequest, int readyState)
{
	(void) optParm;

	if (readyState == readyStateDone)
	{
		AHTTPS_LOGDEBUG0(F("\n**************************************\n"));
		AHTTPS_LOGDEBUG1(F("Response Code = "), thisRequest->responseHTTPString());

		if (thisRequest->responseHTTPcode() == 200)
		{
			Serial.println(F("\n**************************************"));
			Serial.println(thisRequest->responseText());
			Serial.println(F("**************************************"));
		}

		thisRequest->setDebug(false);
		readySend[1] = true;
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

  
	Serial.print("\nStarting AsyncHTTPSRequest_ESP_Multi on ");
	Serial.println(ARDUINO_BOARD);

#if defined(ESP32)
	Serial.println(ASYNC_TCP_SSL_VERSION);
#else
	//Serial.println(ESPASYNC_TCP_SSL_VERSION);
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

	Serial.println("Connecting to WiFi SSID: " + String(ssid));

	while (WiFi.status() != WL_CONNECTED)
	{
		delay(500);
		Serial.print(".");
	}

	Serial.print(F("\nAsyncHTTPSRequest @ IP : "));
	Serial.println(WiFi.localIP());

	for (int index = 0; index < NUM_DIFFERENT_SITES; index++)
	{
		request[index].setDebug(false);

		request[index].onReadyStateChange(requestCB[index]);
	}

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
    if (readySend[0])
		{
      reqCount[0] = NUM_ENTRIES_SITE_0;
			sendRequestCB[0]();
		}
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
    sendJson = ""; // clear string
    Doc.clear();   // release memory used by JsonObject
    Doc["time"] = getTime();
    Doc["value"] = phValue;
    serializeJson(Doc, sendJson);
    Serial.println(reqCount[1]);
    if (readySend[1])
		{
      reqCount[1] = NUM_ENTRIES_SITE_1;
			sendRequestCB[1]();
		}
    /*Serial.println(reqCount[1]);
    Serial.println(readySend[1]);
    if ((reqCount[1] > 0) && readySend[1])
		{
			sendRequestCB[1]();
		}*/
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