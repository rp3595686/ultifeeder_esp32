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
#include <Servo.h>

Servo myservo;

#if !( defined(ESP8266) ||  defined(ESP32) )
  #error This code is intended to run on the ESP8266 or ESP32 platform! Please check your Tools->Board setting.
#endif

#define ESP_ASYNC_WIFIMANAGER_VERSION_MIN_TARGET            "ESPAsync_WiFiManager v1.15.1"
#define ESP_ASYNC_WIFIMANAGER_VERSION_MIN                   1015001

#define ASYNC_HTTPS_REQUEST_GENERIC_VERSION_MIN_TARGET      "AsyncHTTPSRequest_Generic v2.2.1"
#define ASYNC_HTTPS_REQUEST_GENERIC_VERSION_MIN             2002001

// Get Unique ID
String chipID = String(ESP.getEfuseMac(), HEX);

/////////////////////////////////////////////////////////

// Uncomment for certain HTTP site to optimize
//#define NOT_SEND_HEADER_AFTER_CONNECTED        true

// Use larger queue size if necessary for large data transfer. Default is 512 bytes if not defined here
//#define ASYNC_QUEUE_LENGTH     512

// Use larger priority if necessary. Default is 10 if not defined here. Must be > 4 or adjusted to 4
//#define CONFIG_ASYNC_TCP_PRIORITY   (12)

/////////////////////////////////////////////////////////

// Level from 0-4
#define ASYNC_HTTPS_DEBUG_PORT     Serial

#define _ASYNC_TCP_SSL_LOGLEVEL_    1
#define _ASYNC_HTTPS_LOGLEVEL_      2
#define _WIFIMGR_LOGLEVEL_          1

// 300s = 5 minutes to not flooding, 60s in testing
#define HTTPS_REQUEST_INTERVAL      60  //300

//Ported to ESP32
#ifdef ESP32
  #include <esp_wifi.h>
  #include <WiFi.h>
  #include <WiFiClient.h>

  // From v1.1.1
  #include <WiFiMulti.h>
  WiFiMulti wifiMulti;

  // LittleFS has higher priority than SPIFFS
  #define USE_LITTLEFS    true
  #define USE_SPIFFS      false

  #if USE_LITTLEFS
    // Use LittleFS
    #include "FS.h"

    // The library will be depreciated after being merged to future major Arduino esp32 core release 2.x
    // At that time, just remove this library inclusion
    #if ( defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 2) )
      #warning Using ESP32 Core 1.0.6 or 2.0.0+ and core LittleFS library
      // The library has been merged into esp32 core from release 1.0.6
      #include <LittleFS.h>             // https://github.com/espressif/arduino-esp32/tree/master/libraries/LittleFS

      //#define FileFS        LittleFS
      //#define FS_Name       "LittleFS"
      FS* filesystem =      &LittleFS;
      #define FileFS        LittleFS
    #else
      #warning Using ESP32 Core 1.0.5-. You must install LITTLEFS library
      // The library has been merged into esp32 core from release 1.0.6
      #include <LITTLEFS.h>             // https://github.com/lorol/LITTLEFS

      //#define FileFS        LITTLEFS
      FS* filesystem =      &LITTLEFS;
      #define FileFS        LITTLEFS
    #endif

    #define FS_Name       "LittleFS"
  #elif USE_SPIFFS
    #include <SPIFFS.h>
    FS* filesystem =      &SPIFFS;
    #define FileFS        SPIFFS
    #define FS_Name       "SPIFFS"
  #else
    // +Use FFat
    #include <FFat.h>
    FS* filesystem =      &FFat;
    #define FileFS        FFat
    #define FS_Name       "FFat"
  #endif
  //////

  #if !defined(LED_BUILTIN)
    #define LED_BUILTIN       2
  #endif

  #define LED_ON            HIGH
  #define LED_OFF           LOW

#else

  #include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
  //needed for library
  #include <DNSServer.h>

  // From v1.1.1
  #include <ESP8266WiFiMulti.h>
  ESP8266WiFiMulti wifiMulti;

  #define USE_LITTLEFS      true

  #if USE_LITTLEFS
    #include <LittleFS.h>
    FS* filesystem =      &LittleFS;
    #define FileFS        LittleFS
    #define FS_Name       "LittleFS"
  #else
    FS* filesystem =      &SPIFFS;
    #define FileFS        SPIFFS
    #define FS_Name       "SPIFFS"
  #endif
  //////

  #define LED_ON      LOW
  #define LED_OFF     HIGH
#endif


// These defines must be put before #include <ESP_DoubleResetDetector.h>
// to select where to store DoubleResetDetector's variable.
// For ESP32, You must select one to be true (EEPROM or SPIFFS)
// For ESP8266, You must select one to be true (RTC, EEPROM, SPIFFS or LITTLEFS)
// Otherwise, library will use default EEPROM storage
#ifdef ESP32

  // These defines must be put before #include <ESP_DoubleResetDetector.h>
  // to select where to store DoubleResetDetector's variable.
  // For ESP32, You must select one to be true (EEPROM or SPIFFS)
  // Otherwise, library will use default EEPROM storage
  #if USE_LITTLEFS
    #define ESP_DRD_USE_LITTLEFS    true
    #define ESP_DRD_USE_SPIFFS      false
    #define ESP_DRD_USE_EEPROM      false
  #elif USE_SPIFFS
    #define ESP_DRD_USE_LITTLEFS    false
    #define ESP_DRD_USE_SPIFFS      true
    #define ESP_DRD_USE_EEPROM      false
  #else
    #define ESP_DRD_USE_LITTLEFS    false
    #define ESP_DRD_USE_SPIFFS      false
    #define ESP_DRD_USE_EEPROM      true
  #endif

#else //ESP8266

  // For DRD
  // These defines must be put before #include <ESP_DoubleResetDetector.h>
  // to select where to store DoubleResetDetector's variable.
  // For ESP8266, You must select one to be true (RTC, EEPROM, SPIFFS or LITTLEFS)
  // Otherwise, library will use default EEPROM storage
  #if USE_LITTLEFS
    #define ESP_DRD_USE_LITTLEFS    true
    #define ESP_DRD_USE_SPIFFS      false
  #else
    #define ESP_DRD_USE_LITTLEFS    false
    #define ESP_DRD_USE_SPIFFS      true
  #endif

  #define ESP_DRD_USE_EEPROM      false
  #define ESP8266_DRD_USE_RTC     false
#endif

#define DOUBLERESETDETECTOR_DEBUG       true  //false

#include <ESP_DoubleResetDetector.h>      //https://github.com/khoih-prog/ESP_DoubleResetDetector

// Number of seconds after reset during which a
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 10

// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 0

//DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);
DoubleResetDetector* drd;//////

// Config File for saving variable
char configFileName[] = "/config.json";

// SSID and PW for your Router
String Router_SSID;
String Router_Pass;

// From v1.1.1
// You only need to format the filesystem once
#define FORMAT_FILESYSTEM       true
//#define FORMAT_FILESYSTEM         false

#define MIN_AP_PASSWORD_SIZE    8

#define SSID_MAX_LEN            32
//From v1.0.10, WPA2 passwords can be up to 63 characters long.
#define PASS_MAX_LEN            64

typedef struct
{
  char wifi_ssid[SSID_MAX_LEN];
  char wifi_pw  [PASS_MAX_LEN];
}  WiFi_Credentials;

typedef struct
{
  String wifi_ssid;
  String wifi_pw;
}  WiFi_Credentials_String;

#define NUM_WIFI_CREDENTIALS      2

typedef struct
{
  WiFi_Credentials  WiFi_Creds [NUM_WIFI_CREDENTIALS];
  uint16_t checksum;
} WM_Config;

WM_Config         WM_config;

#define  CONFIG_FILENAME              F("/wifi_cred.dat")
//////

// Indicates whether ESP has WiFi credentials saved from previous session, or double reset detected
bool initialConfig = false;

// Use false if you don't like to display Available Pages in Information Page of Config Portal
// Comment out or use true to display Available Pages in Information Page of Config Portal
// Must be placed before #include <ESP_WiFiManager.h>
#define USE_AVAILABLE_PAGES     false

// From v1.0.10 to permit disable/enable StaticIP configuration in Config Portal from sketch. Valid only if DHCP is used.
// You'll loose the feature of dynamically changing from DHCP to static IP, or vice versa
// You have to explicitly specify false to disable the feature.
//#define USE_STATIC_IP_CONFIG_IN_CP          false

// Use false to disable NTP config. Advisable when using Cellphone, Tablet to access Config Portal.
// See Issue 23: On Android phone ConfigPortal is unresponsive (https://github.com/khoih-prog/ESP_WiFiManager/issues/23)
#define USE_ESP_WIFIMANAGER_NTP     false

// Use true to enable CloudFlare NTP service. System can hang if you don't have Internet access while accessing CloudFlare
// See Issue #21: CloudFlare link in the default portal (https://github.com/khoih-prog/ESP_WiFiManager/issues/21)
#define USE_CLOUDFLARE_NTP          false

// New in v1.0.11
#define USING_CORS_FEATURE          false
//////

// Use USE_DHCP_IP == true for dynamic DHCP IP, false to use static IP which you have to change accordingly to your network
#if (defined(USE_STATIC_IP_CONFIG_IN_CP) && !USE_STATIC_IP_CONFIG_IN_CP)
  // Force DHCP to be true
  #if defined(USE_DHCP_IP)
    #undef USE_DHCP_IP
  #endif
  #define USE_DHCP_IP     true
#else
  // You can select DHCP or Static IP here
  #define USE_DHCP_IP     true
  //#define USE_DHCP_IP     false
#endif

#if ( USE_DHCP_IP || ( defined(USE_STATIC_IP_CONFIG_IN_CP) && !USE_STATIC_IP_CONFIG_IN_CP ) )
  // Use DHCP
  #warning Using DHCP IP
  IPAddress stationIP   = IPAddress(0, 0, 0, 0);
  IPAddress gatewayIP   = IPAddress(192, 168, 2, 1);
  IPAddress netMask     = IPAddress(255, 255, 255, 0);
#else
  // Use static IP
  #warning Using static IP

  #ifdef ESP32
    IPAddress stationIP   = IPAddress(192, 168, 2, 232);
  #else
    IPAddress stationIP   = IPAddress(192, 168, 2, 186);
  #endif

  IPAddress gatewayIP   = IPAddress(192, 168, 2, 1);
  IPAddress netMask     = IPAddress(255, 255, 255, 0);
#endif

#define USE_CONFIGURABLE_DNS      true

IPAddress dns1IP      = gatewayIP;
IPAddress dns2IP      = IPAddress(8, 8, 8, 8);

#define USE_CUSTOM_AP_IP          false

IPAddress APStaticIP  = IPAddress(192, 168, 100, 1);
IPAddress APStaticGW  = IPAddress(192, 168, 100, 1);
IPAddress APStaticSN  = IPAddress(255, 255, 255, 0);

#include <ESPAsync_WiFiManager.h>              //https://github.com/khoih-prog/ESPAsync_WiFiManager

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
//#include <ESPAsync_WiFiManager-Impl.h>         //https://github.com/khoih-prog/ESPAsync_WiFiManager

#define HTTP_PORT           80

//define your default values here, if there are different values in configFileName (config.json), they are overwritten.
#define Read_PH_INTERVAL_LEN            6
#define Read_TEMP_INTERVAL_LEN          6
#define BLYNK_TOKEN_LEN           64

#define MQTT_SERVER_MAX_LEN             40
#define MQTT_SERVER_PORT_LEN            6
#define FEED_INTERVAL_LEN               6

char readPhInterval     [Read_PH_INTERVAL_LEN]        = "10000";
char readTempInterval   [Read_TEMP_INTERVAL_LEN]      = "11000";
char blynk_token  [BLYNK_TOKEN_LEN]         = "YOUR_BLYNK_TOKEN";

char mqtt_server  [MQTT_SERVER_MAX_LEN];
char mqtt_port    [MQTT_SERVER_PORT_LEN]    = "8080";
char feedInterval [FEED_INTERVAL_LEN]       = "5000";

#include <AsyncHTTPSRequest_Generic.h>             // https://github.com/khoih-prog/AsyncHTTPRequest_Generic

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
//#include <AsyncHTTPRequest_Impl_Generic.h>        // https://github.com/khoih-prog/AsyncHTTPRequest_Generic

// AsyncHTTPSRequest_Generic Settings (Must be declared after #include <AsyncHTTPSRequest_Generic.h>)
#define NUM_DIFFERENT_SITES     3

// Setup urls required
String strFirebase_url_ph = strFirebase_url + dataStoragePath + chipID + phDataPath +".json";
String strFirebase_url_temp = strFirebase_url + dataStoragePath + chipID + tempDataPath + ".json";
String strFirebase_url_config = strFirebase_url + configPath + chipID + ".json";

// Convert from String to const char*
const char*  firebase_url_ph = strFirebase_url_ph.c_str();
const char* firebase_url_temp = strFirebase_url_temp.c_str();
const char* firebase_url_config = strFirebase_url_config.c_str();

const char* addreses[][NUM_DIFFERENT_SITES] =
{
	{firebase_url_ph},
  {firebase_url_temp},
  {firebase_url_config}
};

#define NUM_ENTRIES_SITE_0        1
#define NUM_ENTRIES_SITE_1        1
#define NUM_ENTRIES_SITE_2        1

byte reqCount[NUM_DIFFERENT_SITES]  = { NUM_ENTRIES_SITE_0, NUM_ENTRIES_SITE_1, NUM_ENTRIES_SITE_2 };
bool readySend[NUM_DIFFERENT_SITES] = { true, true, true };

AsyncHTTPSRequest request[NUM_DIFFERENT_SITES];
int status; // the Wifi radio's status

void requestCB0(void* optParm, AsyncHTTPSRequest* thisRequest, int readyState);
void requestCB1(void* optParm, AsyncHTTPSRequest* thisRequest, int readyState);
void requestCB2(void* optParm, AsyncHTTPSRequest* thisRequest, int readyState);

void sendRequest0();
void sendRequest1();
void sendRequest2();

typedef void (*requestCallback)(void* optParm, AsyncHTTPSRequest* thisRequest, int readyState);
typedef void (*sendCallback)();

requestCallback requestCB     [NUM_DIFFERENT_SITES] = { requestCB0,   requestCB1,   requestCB2   };
sendCallback    sendRequestCB [NUM_DIFFERENT_SITES] = { sendRequest0, sendRequest1, sendRequest2 };

///////////////////////////////////////////
// New in v1.4.0
/******************************************
   // Defined in ESPAsync_WiFiManager.h
  typedef struct
  {
  IPAddress _ap_static_ip;
  IPAddress _ap_static_gw;
  IPAddress _ap_static_sn;
  }  WiFi_AP_IPConfig;
  typedef struct
  {
  IPAddress _sta_static_ip;
  IPAddress _sta_static_gw;
  IPAddress _sta_static_sn;
  #if USE_CONFIGURABLE_DNS
  IPAddress _sta_static_dns1;
  IPAddress _sta_static_dns2;
  #endif
  }  WiFi_STA_IPConfig;
******************************************/

WiFi_AP_IPConfig  WM_AP_IPconfig;
WiFi_STA_IPConfig WM_STA_IPconfig;

//NTP settings
const char *ntpServers[] = {"ntp.np.edu.sg", "pool.ntp.org","sg.pool.ntp.org","time.google.com","time.cloudflare.com"};
int ntpArraySize = sizeof(ntpServers) / sizeof(ntpServers[0]);
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

// Millis for non-blocking
unsigned long currentMillis;
unsigned long fetchConfig_startMillis;
unsigned long foodFeed_startMillis;
unsigned long readTemp_startMillis;
unsigned long readPh_startMillis;
unsigned long noFoodDetect_startMillis;
unsigned long foodStuckDetect_startMillis;

// Period Config
//unsigned long feedInterval = 10000;  // the value is a number of milliseconds
unsigned long fetchConfigInterval = 10000;  // the value is a number of milliseconds
unsigned long readTempPeriod = 13000; // the value is a number of milliseconds
unsigned long readPhPeriod = 10000;   // the value is a number of milliseconds
unsigned long noFoodDetectPeriod = 1000;   // the value is a number of milliseconds
unsigned long foodStuckDetectPeriod = 5000;   // the value is a number of milliseconds

//Time settings
struct tm timeinfo;
char strTime[51];
bool timeNotObtained = false;

//JSON settings
StaticJsonDocument<192> Doc;
String sendJson;

// Used variable
float voltage, phValue, temperature = 25;
int noFoodCount = 0;
bool isFoodLeft = false;
bool isFoodStuckDetectStart = false;
bool isFoodStuck = false;

// Pins
const int motor1pin1 = 33;
const int motor1pin2 = 25;
const int topIRpin = 13;
const int buttomIRpin = 12;

void initAPIPConfigStruct(WiFi_AP_IPConfig &in_WM_AP_IPconfig)
{
  in_WM_AP_IPconfig._ap_static_ip   = APStaticIP;
  in_WM_AP_IPconfig._ap_static_gw   = APStaticGW;
  in_WM_AP_IPconfig._ap_static_sn   = APStaticSN;
}

void initSTAIPConfigStruct(WiFi_STA_IPConfig &in_WM_STA_IPconfig)
{
  in_WM_STA_IPconfig._sta_static_ip   = stationIP;
  in_WM_STA_IPconfig._sta_static_gw   = gatewayIP;
  in_WM_STA_IPconfig._sta_static_sn   = netMask;
#if USE_CONFIGURABLE_DNS
  in_WM_STA_IPconfig._sta_static_dns1 = dns1IP;
  in_WM_STA_IPconfig._sta_static_dns2 = dns2IP;
#endif
}

void displayIPConfigStruct(WiFi_STA_IPConfig in_WM_STA_IPconfig)
{
  LOGERROR3(F("stationIP ="), in_WM_STA_IPconfig._sta_static_ip, F(", gatewayIP ="), in_WM_STA_IPconfig._sta_static_gw);
  LOGERROR1(F("netMask ="), in_WM_STA_IPconfig._sta_static_sn);
#if USE_CONFIGURABLE_DNS
  LOGERROR3(F("dns1IP ="), in_WM_STA_IPconfig._sta_static_dns1, F(", dns2IP ="), in_WM_STA_IPconfig._sta_static_dns2);
#endif
}

void configWiFi(WiFi_STA_IPConfig in_WM_STA_IPconfig)
{
#if USE_CONFIGURABLE_DNS
  // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5
  WiFi.config(in_WM_STA_IPconfig._sta_static_ip, in_WM_STA_IPconfig._sta_static_gw, in_WM_STA_IPconfig._sta_static_sn,
              in_WM_STA_IPconfig._sta_static_dns1, in_WM_STA_IPconfig._sta_static_dns2);
#else
  // Set static IP, Gateway, Subnetmask, Use auto DNS1 and DNS2.
  WiFi.config(in_WM_STA_IPconfig._sta_static_ip, in_WM_STA_IPconfig._sta_static_gw, in_WM_STA_IPconfig._sta_static_sn);
#endif
}

///////////////////////////////////////////

uint8_t connectMultiWiFi()
{
#if ESP32
  // For ESP32, this better be 0 to shorten the connect time.
  // For ESP32-S2, must be > 500
#if ( ARDUINO_ESP32S2_DEV || ARDUINO_FEATHERS2 || ARDUINO_PROS2 || ARDUINO_MICROS2 )
#define WIFI_MULTI_1ST_CONNECT_WAITING_MS           500L
#else
  // For ESP32 core v1.0.6, must be >= 500
#define WIFI_MULTI_1ST_CONNECT_WAITING_MS           800L
#endif
#else
  // For ESP8266, this better be 2200 to enable connect the 1st time
#define WIFI_MULTI_1ST_CONNECT_WAITING_MS             2200L
#endif

#define WIFI_MULTI_CONNECT_WAITING_MS                   100L

  uint8_t status;

  LOGERROR(F("ConnectMultiWiFi with :"));

  if ( (Router_SSID != "") && (Router_Pass != "") )
  {
    LOGERROR3(F("* Flash-stored Router_SSID = "), Router_SSID, F(", Router_Pass = "), Router_Pass );
  }

  for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
  {
    // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
    if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "")
         && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
    {
      LOGERROR3(F("* Additional SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
    }
  }

  LOGERROR(F("Connecting MultiWifi..."));

  WiFi.mode(WIFI_STA);

#if !USE_DHCP_IP
  // New in v1.4.0
  configWiFi(WM_STA_IPconfig);
  //////
#endif

  int i = 0;
  status = wifiMulti.run();

  delay(WIFI_MULTI_1ST_CONNECT_WAITING_MS);

  while ( ( i++ < 10 ) && ( status != WL_CONNECTED ) )
  {
    status = wifiMulti.run();

    if ( status == WL_CONNECTED )
      break;
    else
      delay(WIFI_MULTI_CONNECT_WAITING_MS);
  }

  if ( status == WL_CONNECTED )
  {
    LOGERROR1(F("WiFi connected after time: "), i);
    LOGERROR3(F("SSID:"), WiFi.SSID(), F(",RSSI="), WiFi.RSSI());
    LOGERROR3(F("Channel:"), WiFi.channel(), F(",IP address:"), WiFi.localIP() );
  }
  else
  {
    LOGERROR(F("WiFi not connected"));

    // To avoid unnecessary DRD
    drd->loop();
  }

  return status;
}


//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback()
{
  Serial.println(F("Should save config"));
  shouldSaveConfig = true;
}

bool loadFileFSConfigFile()
{
  //clean FS, for testing
  //FileFS.format();

  //read configuration from FS json
  Serial.println(F("Mounting FS..."));

  if (FileFS.begin())
  {
    Serial.println(F("Mounted file system"));

    if (FileFS.exists(configFileName))
    {
      //file exists, reading and loading
      Serial.println(F("Reading config file"));
      File configFile = FileFS.open(configFileName, "r");

      if (configFile)
      {
        Serial.print(F("Opened config file, size = "));
        size_t configFileSize = configFile.size();
        Serial.println(configFileSize);

        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[configFileSize + 1]);

        configFile.readBytes(buf.get(), configFileSize);

        Serial.print(F("\nJSON parseObject() result : "));

#if (ARDUINOJSON_VERSION_MAJOR >= 6)
        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get(), configFileSize);

        if ( deserializeError )
        {
          Serial.println(F("failed"));
          return false;
        }
        else
        {
          Serial.println(F("OK"));

          if (json["readPhInterval"])
            strncpy(readPhInterval, json["readPhInterval"], sizeof(readPhInterval));

          if (json["readTempInterval"])
            strncpy(readTempInterval, json["readTempInterval"], sizeof(readTempInterval));

          if (json["blynk_token"])
            strncpy(blynk_token,  json["blynk_token"], sizeof(blynk_token));

          if (json["mqtt_server"])
            strncpy(mqtt_server, json["mqtt_server"], sizeof(mqtt_server));

          if (json["mqtt_port"])
            strncpy(mqtt_port,   json["mqtt_port"], sizeof(mqtt_port));
          
          if (json["feedInterval"])
            strncpy(feedInterval,   json["feedInterval"], sizeof(feedInterval));
        }

        //serializeJson(json, Serial);
        serializeJsonPretty(json, Serial);
#else
        DynamicJsonBuffer jsonBuffer;
        // Parse JSON string
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        // Test if parsing succeeds.

        if (json.success())
        {
          Serial.println("OK");

          if (json["blynk_server"])
            strncpy(blynk_server, json["blynk_server"], sizeof(blynk_server));

          if (json["blynk_port"])
            strncpy(blynk_port, json["blynk_port"], sizeof(blynk_port));

          if (json["blynk_token"])
            strncpy(blynk_token,  json["blynk_token"], sizeof(blynk_token));

          if (json["mqtt_server"])
            strncpy(mqtt_server, json["mqtt_server"], sizeof(mqtt_server));

          if (json["mqtt_port"])
            strncpy(mqtt_port,   json["mqtt_port"], sizeof(mqtt_port));
        }
        else
        {
          Serial.println(F("failed"));
          return false;
        }

        //json.printTo(Serial);
        json.prettyPrintTo(Serial);
#endif

        configFile.close();
      }
    }
  }
  else
  {
    Serial.println(F("failed to mount FS"));
    return false;
  }

  return true;
}

bool saveFileFSConfigFile()
{
  Serial.println(F("Saving config"));

#if (ARDUINOJSON_VERSION_MAJOR >= 6)
  DynamicJsonDocument json(1024);
#else
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
#endif

  json["readPhInterval"] = readPhInterval;
  json["readTempInterval"]   = readTempInterval;
  json["blynk_token"]  = blynk_token;

  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"]   = mqtt_port;
  json["feedInterval"]   = feedInterval;

  File configFile = FileFS.open(configFileName, "w");

  if (!configFile)
  {
    Serial.println(F("Failed to open config file for writing"));

    return false;
  }

#if (ARDUINOJSON_VERSION_MAJOR >= 6)
  //serializeJson(json, Serial);
  serializeJsonPretty(json, Serial);
  // Write data to file and close it
  serializeJson(json, configFile);
#else
  //json.printTo(Serial);
  json.prettyPrintTo(Serial);
  // Write data to file and close it
  json.printTo(configFile);
#endif

  configFile.close();
  //end save

  return true;
}

void check_WiFi()
{
  if ( (WiFi.status() != WL_CONNECTED) )
  {
    Serial.println(F("\nWiFi lost. Call connectMultiWiFi in loop"));
    connectMultiWiFi();
  }
}

void check_status()
{
  static ulong checkwifi_timeout    = 0;

  static ulong current_millis;

#define WIFICHECK_INTERVAL    5000L

  current_millis = millis();

  // Check WiFi every WIFICHECK_INTERVAL (5) seconds.
  if ((current_millis > checkwifi_timeout) || (checkwifi_timeout == 0))
  {
    check_WiFi();
    checkwifi_timeout = current_millis + WIFICHECK_INTERVAL;
  }
}

int calcChecksum(uint8_t* address, uint16_t sizeToCalc)
{
  uint16_t checkSum = 0;

  for (uint16_t index = 0; index < sizeToCalc; index++)
  {
    checkSum += * ( ( (byte*) address ) + index);
  }

  return checkSum;
}

bool loadConfigData()
{
  File file = FileFS.open(CONFIG_FILENAME, "r");
  LOGERROR(F("LoadWiFiCfgFile "));

  memset((void *) &WM_config,       0, sizeof(WM_config));

  // New in v1.4.0
  memset((void *) &WM_STA_IPconfig, 0, sizeof(WM_STA_IPconfig));
  //////

  if (file)
  {
    file.readBytes((char *) &WM_config,   sizeof(WM_config));

    // New in v1.4.0
    file.readBytes((char *) &WM_STA_IPconfig, sizeof(WM_STA_IPconfig));
    //////

    file.close();
    LOGERROR(F("OK"));

    if ( WM_config.checksum != calcChecksum( (uint8_t*) &WM_config, sizeof(WM_config) - sizeof(WM_config.checksum) ) )
    {
      LOGERROR(F("WM_config checksum wrong"));

      return false;
    }

    // New in v1.4.0
    displayIPConfigStruct(WM_STA_IPconfig);
    //////

    return true;
  }
  else
  {
    LOGERROR(F("failed"));

    return false;
  }
}

void saveConfigData()
{
  File file = FileFS.open(CONFIG_FILENAME, "w");
  LOGERROR(F("SaveWiFiCfgFile "));

  if (file)
  {
    file.write((uint8_t*) &WM_config,   sizeof(WM_config));

    // New in v1.4.0
    file.write((uint8_t*) &WM_STA_IPconfig, sizeof(WM_STA_IPconfig));
    //////

    file.close();
    LOGERROR(F("OK"));
  }
  else
  {
    LOGERROR(F("failed"));
  }
}

float readTemperature()
{
  // add your code here to get the temperature from your temperature sensor
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
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

void sendGETRequest(uint16_t index)
{
	static bool requestOpenResult;

	reqCount[index]--;
	readySend[index] = false;

	requestOpenResult = request[index].open("GET", addreses[index][reqCount[index]]);

	if (requestOpenResult)
	{
		// Only send() if open() returns true, or crash
		Serial.print("\nSending request: ");
		request[index].send();
	}
	else
	{
		Serial.print("\nCan't send bad request : ");
	}

	Serial.println(addreses[index][reqCount[index]]);
}

void sendPOSTRequest(uint16_t index)
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
	sendPOSTRequest(0);
}

void sendRequest1()
{
	sendPOSTRequest(1);
}

void sendRequest2()
{
	sendGETRequest(2);
}

void sendRequests()
{
  // Setting all requests to have only 1 site
	/*for (int index = 0; index < NUM_DIFFERENT_SITES; index++)
	{
		reqCount[index] = 1;
	}*/

	reqCount[0] = NUM_ENTRIES_SITE_0;
	reqCount[1] = NUM_ENTRIES_SITE_1;
  reqCount[2] = NUM_ENTRIES_SITE_2;
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

void requestCB2(void *optParm, AsyncHTTPSRequest *thisRequest, int readyState)
{
	(void) optParm;

	if (readyState == readyStateDone)
	{
		AHTTPS_LOGDEBUG0(F("\n**************************************\n"));
		AHTTPS_LOGDEBUG1(F("Response Code = "), thisRequest->responseHTTPString());

		if (thisRequest->responseHTTPcode() == 200)
		{
      int pre_feedInterval,pre_readPhInterval,pre_readTempInterval; // save previous feedInterval
      String responseText = thisRequest->responseText();
			Serial.println(F("\n**************************************"));
      Serial.println(responseText);
      Serial.println(F("**************************************"));
      StaticJsonDocument<192> jsonResponse;
      auto deserializeError = deserializeJson(jsonResponse, responseText);

      if (deserializeError)
      {
        Serial.println(F("failed"));
      }
      else
      {
        Serial.println(F("OK"));

        if (jsonResponse["readPhInterval"])
        {
          pre_readPhInterval = atoi(readPhInterval); // save previous feedInterval
          int readPhInterval_buffer = jsonResponse["readPhInterval"];
          readPhInterval_buffer = readPhInterval_buffer * 1000; // Change from seconds to miliseconds
          sprintf(readPhInterval, "%d", readPhInterval_buffer);
          Serial.print("Set readPhInterval to ");
          Serial.println(readPhInterval);
        }
        if (jsonResponse["readTempInterval"])
        {
          pre_readTempInterval = atoi(readTempInterval); // save previous feedInterval
          int readTempInterval_buffer = jsonResponse["readTempInterval"];
          readTempInterval_buffer = readTempInterval_buffer * 1000; // Change from seconds to miliseconds
          sprintf(readTempInterval, "%d", readTempInterval_buffer);
          Serial.print("Set readTempInterval to ");
          Serial.println(readTempInterval);
        }
        if (jsonResponse["feedInterval"])
        {
          pre_feedInterval = atoi(feedInterval); // save previous feedInterval
          int feedInterval_buffer = jsonResponse["feedInterval"];
          feedInterval_buffer = feedInterval_buffer * 1000;                    // Change from seconds to miliseconds
          sprintf(feedInterval, "%d", feedInterval_buffer);
          Serial.print("Set feedInterval to ");
          Serial.println(feedInterval);
        }
        if (atoi(readPhInterval) != pre_readPhInterval || atoi(readTempInterval) != pre_readTempInterval || atoi(feedInterval) != pre_feedInterval) // feedInterval changed
          {
            saveFileFSConfigFile(); // save new data to Config File
          }
          jsonResponse.clear();   // release memory used by JsonObject
      }
    }

    thisRequest->setDebug(false);
		readySend[2] = true;
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

  foodFeed_startMillis = millis();      // initial start time
  readTemp_startMillis = millis(); // initial start time

  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  //myservo.attach(motor1pin1);
  pinMode(32, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(topIRpin, INPUT);
  pinMode(buttomIRpin, INPUT);

  // Controlling spin direction of motors:
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  analogWrite(32, 255); // ENA pin`

  EEPROM.begin(32); // needed EEPROM.begin to store calibration k in eeprom
  //ph.begin();
  //sensors.begin();
  //ads.setGain(GAIN_ONE);
  //ads.begin();

  // LCD Setup
  phost = &host;
  /* Init HW Hal */
  App_Common_Init(&host);

  
	// put your setup code here, to run once:
  Serial.begin(115200);

  while (!Serial && millis() < 5000);

  delay(200);

  Serial.print(F("\nStarting Ultifeeder using "));
  Serial.print(FS_Name);
  Serial.print(F(" on "));
  Serial.println(ARDUINO_BOARD);
  Serial.println(ESP_ASYNC_WIFIMANAGER_VERSION);
  Serial.println(ASYNC_HTTPS_REQUEST_GENERIC_VERSION);
  Serial.println(ESP_DOUBLE_RESET_DETECTOR_VERSION);

#if defined(ESP_ASYNC_WIFIMANAGER_VERSION_INT)

  if (ESP_ASYNC_WIFIMANAGER_VERSION_INT < ESP_ASYNC_WIFIMANAGER_VERSION_MIN)
  {
    Serial.print(F("Warning. Must use this example on Version later than : "));
    Serial.println(ESP_ASYNC_WIFIMANAGER_VERSION_MIN_TARGET);
  }

#endif

#if defined(ASYNC_HTTPS_REQUEST_GENERIC_VERSION_MIN)

  if (ASYNC_HTTPS_REQUEST_GENERIC_VERSION_INT < ASYNC_HTTPS_REQUEST_GENERIC_VERSION_MIN)
  {
    Serial.print(F("Warning. Must use this example on Version equal or later than : "));
    Serial.println(ASYNC_HTTPS_REQUEST_GENERIC_VERSION_MIN_TARGET);
  }

#endif

#ifdef ESP32

  if (!FileFS.begin(true))
#else
  if (!FileFS.begin())
#endif
  {
#ifdef ESP8266
    FileFS.format();
#endif

    Serial.println(F("SPIFFS/LittleFS failed! Already tried formatting."));

    if (!FileFS.begin())
    {
      // prevents debug info from the library to hide err message.
      delay(100);

#if USE_LITTLEFS
      Serial.println(F("LittleFS failed!. Please use SPIFFS or EEPROM. Stay forever"));
#else
      Serial.println(F("SPIFFS failed!. Please use LittleFS or EEPROM. Stay forever"));
#endif

      while (true)
      {
        delay(1);
      }
    }
  }

  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);

  loadFileFSConfigFile();

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  ESPAsync_WMParameter custom_readPhInterval("readPhInterval", "readPhInterval", readPhInterval, Read_PH_INTERVAL_LEN + 1);
  ESPAsync_WMParameter custom_readTempInterval  ("readTempInterval",   "readTempInterval",   readTempInterval,   Read_TEMP_INTERVAL_LEN + 1);
  ESPAsync_WMParameter custom_blynk_token ("blynk_token",  "blynk_token",  blynk_token,  BLYNK_TOKEN_LEN + 1 );

  ESPAsync_WMParameter custom_mqtt_server   ("mqtt_server", "mqtt_server", mqtt_server, MQTT_SERVER_MAX_LEN + 1);
  ESPAsync_WMParameter custom_mqtt_port     ("mqtt_port",   "mqtt_port",   mqtt_port,   MQTT_SERVER_PORT_LEN + 1);
  ESPAsync_WMParameter custom_feedInterval  ("feedInterval",   "feedInterval",   feedInterval,   FEED_INTERVAL_LEN + 1);

  unsigned long startedAt = millis();

  // New in v1.4.0
  initAPIPConfigStruct(WM_AP_IPconfig);
  initSTAIPConfigStruct(WM_STA_IPconfig);
  //////

  //Local intialization. Once its business is done, there is no need to keep it around
  // Use this to default DHCP hostname to ESP8266-XXXXXX or ESP32-XXXXXX
  //ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, &dnsServer);
  // Use this to personalize DHCP hostname (RFC952 conformed)
  AsyncWebServer webServer(HTTP_PORT);

  //ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, &dnsServer, "AutoConnectAP");
#if ( USING_ESP32_S2 || USING_ESP32_C3 )
  ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, NULL, "AutoConnectAP");
#else
  AsyncDNSServer dnsServer;

  ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, &dnsServer, "AutoConnectAP");
#endif

  //set config save notify callback
  ESPAsync_wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  ESPAsync_wifiManager.addParameter(&custom_readPhInterval);
  ESPAsync_wifiManager.addParameter(&custom_readTempInterval);
  ESPAsync_wifiManager.addParameter(&custom_blynk_token);

  ESPAsync_wifiManager.addParameter(&custom_mqtt_server);
  ESPAsync_wifiManager.addParameter(&custom_mqtt_port);
  ESPAsync_wifiManager.addParameter(&custom_feedInterval);

  //ESPAsync_wifiManager.setDebugOutput(true);

  //reset settings - for testing
  //ESPAsync_wifiManager.resetSettings();

#if USE_CUSTOM_AP_IP
  //set custom ip for portal
  // New in v1.4.0
  ESPAsync_wifiManager.setAPStaticIPConfig(WM_AP_IPconfig);
  //////
#endif

  ESPAsync_wifiManager.setMinimumSignalQuality(-1);

  // From v1.0.10 only
  // Set config portal channel, default = 1. Use 0 => random channel from 1-13
  ESPAsync_wifiManager.setConfigPortalChannel(0);
  //////

#if !USE_DHCP_IP
  // Set (static IP, Gateway, Subnetmask, DNS1 and DNS2) or (IP, Gateway, Subnetmask). New in v1.0.5
  // New in v1.4.0
  ESPAsync_wifiManager.setSTAStaticIPConfig(WM_STA_IPconfig);
  //////
#endif

  // New from v1.1.1
#if USING_CORS_FEATURE
  ESPAsync_wifiManager.setCORSHeader("Your Access-Control-Allow-Origin");
#endif

  // We can't use WiFi.SSID() in ESP32 as it's only valid after connected.
  // SSID and Password stored in ESP32 wifi_ap_record_t and wifi_config_t are also cleared in reboot
  // Have to create a new function to store in EEPROM/SPIFFS for this purpose
  Router_SSID = ESPAsync_wifiManager.WiFi_SSID();
  Router_Pass = ESPAsync_wifiManager.WiFi_Pass();

  //Remove this line if you do not want to see WiFi password printed
  Serial.print(F("Stored: SSID = "));
  Serial.print(Router_SSID);
  Serial.print(F(", Pass = "));
  Serial.println(Router_Pass);

  // SSID and PW for Config Portal
  String AP_SSID = "ESP_" + chipID;
  String AP_PASS = "ESP_" + chipID;
  //String AP_PASS = "your password";

  bool configDataLoaded = false;

  // From v1.1.0, Don't permit NULL password
  if ( (Router_SSID != "") && (Router_Pass != "") )
  {
    LOGERROR3(F("* Add SSID = "), Router_SSID, F(", PW = "), Router_Pass);
    wifiMulti.addAP(Router_SSID.c_str(), Router_Pass.c_str());

    //ESPAsync_wifiManager.setConfigPortalTimeout(0); //If no access point name has been previously entered disable timeout.
    Serial.println(F("Got ESP Self-Stored Credentials. Timeout 120s for Config Portal"));
  } else if  (loadConfigData()) {
    configDataLoaded = true;

    //ESPAsync_wifiManager.setConfigPortalTimeout(0); //If no access point name has been previously entered disable timeout.
    Serial.println(F("Got stored Credentials. Timeout 120s for Config Portal"));
  }
  else
  {
    // Enter CP only if no stored SSID on flash and file
    Serial.println(F("Open Config Portal without Timeout: No stored Credentials."));
    initialConfig = true;
  }

  if (drd->detectDoubleReset())
  {
    // DRD, disable timeout.
    ESPAsync_wifiManager.setConfigPortalTimeout(0);

    Serial.println(F("Open Config Portal without Timeout: Double Reset Detected"));
    initialConfig = true;
  }

  if (initialConfig)
  {
    Serial.print(F("Starting configuration portal @ "));

#if USE_CUSTOM_AP_IP
    Serial.print(APStaticIP);
#else
    Serial.print(F("192.168.4.1"));
#endif

    Serial.print(F(", SSID = "));
    Serial.print(AP_SSID);
    Serial.print(F(", PWD = "));
    Serial.println(AP_PASS);

    //sets timeout in seconds until configuration portal gets turned off.
    //If not specified device will remain in configuration mode until
    //switched off via webserver or device is restarted.
    //ESPAsync_wifiManager.setConfigPortalTimeout(600);

#if DISPLAY_STORED_CREDENTIALS_IN_CP
    // New. Update Credentials, got from loadConfigData(), to display on CP
    ESPAsync_wifiManager.setCredentials(WM_config.WiFi_Creds[0].wifi_ssid, WM_config.WiFi_Creds[0].wifi_pw,
                                        WM_config.WiFi_Creds[1].wifi_ssid, WM_config.WiFi_Creds[1].wifi_pw);
#endif

    // Starts an access point
    if (!ESPAsync_wifiManager.startConfigPortal((const char *) AP_SSID.c_str(), AP_PASS.c_str()))
      Serial.println(F("Not connected to WiFi but continuing anyway."));
    else
    {
      Serial.println(F("WiFi connected...yeey :)"));
    }

    // Stored  for later usage, from v1.1.0, but clear first
    memset(&WM_config, 0, sizeof(WM_config));

    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
    {
      String tempSSID = ESPAsync_wifiManager.getSSID(i);
      String tempPW   = ESPAsync_wifiManager.getPW(i);

      if (strlen(tempSSID.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1);

      if (strlen(tempPW.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1);

      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
      if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "")
           && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
      {
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
      }
    }

    // New in v1.4.0
    ESPAsync_wifiManager.getSTAStaticIPConfig(WM_STA_IPconfig);
    //////

    saveConfigData();
  }

  startedAt = millis();

  if (!initialConfig)
  {
    // Load stored data, the addAP ready for MultiWiFi reconnection
    if (!configDataLoaded)
      loadConfigData();

    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
    {
      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
      if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "")
           && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
      {
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
      }
    }

    if ( WiFi.status() != WL_CONNECTED )
    {
      Serial.println(F("ConnectMultiWiFi in setup"));

      connectMultiWiFi();
    }
  }

  Serial.print(F("After waiting "));
  Serial.print((float) (millis() - startedAt) / 1000);
  Serial.print(F(" secs more in setup(), connection result is "));

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print(F("connected. Local IP: "));
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println(ESPAsync_wifiManager.getStatus(WiFi.status()));
  }

  //read updated parameters
  strncpy(readPhInterval, custom_readPhInterval.getValue(), sizeof(readPhInterval));
  strncpy(readTempInterval,   custom_readTempInterval.getValue(),   sizeof(readTempInterval));
  strncpy(blynk_token,  custom_blynk_token.getValue(),  sizeof(blynk_token));

  strncpy(mqtt_server, custom_mqtt_server.getValue(), sizeof(mqtt_server));
  strncpy(mqtt_port, custom_mqtt_port.getValue(),     sizeof(mqtt_port));
  strncpy(feedInterval, custom_feedInterval.getValue(),     sizeof(feedInterval));

  //save the custom parameters to FS
  if (shouldSaveConfig)
  {
    saveFileFSConfigFile();
  }

	for (int index = 0; index < NUM_DIFFERENT_SITES; index++)
	{
		request[index].setDebug(false);

		request[index].onReadyStateChange(requestCB[index]);
	}

  // Setting up NTP
  int ntpArrayIndex = 0;
  String strTime = "Failed to obtain time"; // default string
  
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("No Wifi. Time will not be obtain.");
    Serial.println("Data will not be sent to database.");
    timeNotObtained = true;
  }

  while (strTime == "Failed to obtain time" && timeNotObtained == false) // Try to obtain time from the NTP sever configured
  {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServers[ntpArrayIndex]);
    strTime = getTime();
    if (strTime == "Failed to obtain time")
    {
      Serial.print("Failed to obtain time from ");
      Serial.println(ntpServers[ntpArrayIndex]);
      ntpArrayIndex++;
    }
    if (ntpArrayIndex > ntpArraySize - 1)
    {
      Serial.println("Failed to obtain time from all NTP Severs configured.");
      Serial.println("Data will not be sent to database.");
      timeNotObtained = true;
    }
  }
}

void loop()
{
  // Call the double reset detector loop method every so often,
  // so that it can recognise when the timeout expires.
  // You can also call drd.stop() when you wish to no longer
  // consider the next reset as a double reset.
  drd->loop();

  currentMillis = millis();

   if (currentMillis - readPh_startMillis >= atoi(readPhInterval))
  {
    // temperature = readTemperature();                     //needed for temperature compensation below
    // voltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE;  // read the voltage
    // Serial.println(voltage);
    // phValue = ph.readPH(voltage, temperature);  // convert voltage to pH with temperature compensation
    // phValue = random(10);
    //phValue = map(analogRead(PH_PIN), 0, 4096, 0, 14);
    phValue = random(6,8);
    Serial.print("pH:");
    Serial.println(phValue, 4);
    readPh_startMillis = currentMillis;
    showOnLcd();
    sendJson = ""; // clear string
    Doc.clear();   // release memory used by JsonObject
    if (!timeNotObtained) // Not able to configure NTP
    {
      Doc["time"] = getTime();
    }
    Doc["value"] = phValue;
    serializeJson(Doc, sendJson);
    if (readySend[0] && !timeNotObtained) // ready to send and time is configured correctly
		{
      reqCount[0] = NUM_ENTRIES_SITE_0;
			sendRequestCB[0]();
		}
  }

  if (currentMillis - readTemp_startMillis >= atoi(readTempInterval))
  {
    temperature = random(24,27);
    //temperature = readTemperature(); // read your temperature sensor to execute temperature compensation
    Serial.print("Temperature:");
    Serial.print(temperature, 1);
    Serial.println("^C");
    readTemp_startMillis = currentMillis;
    showOnLcd();
    sendJson = ""; // clear string
    Doc.clear();   // release memory used by JsonObject
    if (!timeNotObtained) // Not able to configure NTP
    {
      Doc["time"] = getTime();
    }
    Doc["value"] = temperature;
    serializeJson(Doc, sendJson);
    if (readySend[1] && !timeNotObtained) // ready to send and time is configured correctly
		{
      reqCount[1] = NUM_ENTRIES_SITE_1;
			sendRequestCB[1]();
		}
  }
  if (currentMillis - fetchConfig_startMillis >= fetchConfigInterval)
  {
    fetchConfig_startMillis = currentMillis;
    if (readySend[2] && WiFi.status() == WL_CONNECTED) // ready to send and wifi connected
    {
      reqCount[2] = NUM_ENTRIES_SITE_2;
      sendRequestCB[2]();
    }
  }
  /*
  if ((currentMillis - startMillis >= feedPeriod) && flag == true && Mode == 1)
  {
    //digitalWrite(motor1pin2, LOW);
    myservo.write(60); 
    digitalWrite(LEDR, LOW);
    flag = false;
    foodFeed_startMillis = currentMillis;
  }
  else if ((currentMillis - foodFeed_startMillis >= motorPeriod) && flag == false && Mode == 1)
  {
    //digitalWrite(motor1pin2, HIGH);
    myservo.write(0);      
    flag = true;
    foodFeed_startMillis = currentMillis;

    if (Count == 0)
      Mode = 0;

    else
      Count = 0;
  }
  else if (Mode == 0)
  {
    digitalWrite(LEDR, HIGH);
    //digitalWrite(motor1pin2, HIGH);
    myservo.write(0);   
  }*/
}