/*
 * esp8266.ino
 *
 * Copyright 2020 Victor Chew
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdint.h>
#include <FS.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ESP8266HTTPClient.h>
#include <Wire.h>
#include <Rtc_Pcf8563.h> // https://github.com/elpaso/Rtc_Pcf8563

// Application Javascript to be injected into WiFiManager's config page
#define QUOTE(...) #__VA_ARGS__
const char* jscript = QUOTE(
  <script>
    document.addEventListener('DOMContentLoaded', tzinit, false); 
    function tzinit() {
      document.getElementById("timezone").value = Intl.DateTimeFormat().resolvedOptions().timeZone;
    }
  </script>
);

/*
 * PHP script can be hosted on your own server:
 *
 * <?php
 * if (isset($_REQUEST['tz'])) {
 *   $time = new DateTime();
 *   $time->setTimezone(new DateTimeZone($_REQUEST['tz']));
 *   print $time->format('h:i:s');
 * }
 * ?>
 *
 * Or you can use the version hosted at http://espclock.randseq.org/now.php
 *
 * List of timezones can be found here: https://en.wikipedia.org/wiki/List_of_tz_database_time_zones
 */

#define I2C_SLAVE_ADDR 0x12
#define CMD_START_CLOCK 0x2
#define CMD_STOP_CLOCK 0x4
#define CMD_SET_CLOCK 0x6
#define CMD_SET_NETTIME 0x8
#define CONFIG_PIN 12
#define DEFAULT_SCRIPT_URL "http://espclock.randseq.org/now.php?tz=[tz]"

bool shouldSaveConfig = false;
byte clockHH = 0, clockMM = 0, clockSS = 0, netHH = 0, netMM = 0, netSS = 0;
char param_tz[64] = "UTC", param_url[256] = DEFAULT_SCRIPT_URL;
char buf_timezone[64] = "", buf_clockTime[10] = "", buf_scriptUrl[256] = DEFAULT_SCRIPT_URL;
WiFiManagerParameter form_timezone("timezone", "TZ database timezone code", buf_timezone, sizeof(buf_timezone)-1);
WiFiManagerParameter form_clockTime("clockTime", "Time on clock (12-hr HHMMSS)", buf_clockTime, sizeof(buf_clockTime)-1);
WiFiManagerParameter form_scriptUrl("scriptUrl", "URL to ESPCLOCK script", buf_scriptUrl, sizeof(buf_scriptUrl)-1);
WiFiManager wifiManager;
Rtc_Pcf8563 rtc;

#define BT_DEBUG

void debug(const char *format, ...) {
#ifdef BT_DEBUG
  char msg[256];
  va_list ap;
  va_start(ap, format);
  vsnprintf(msg, sizeof(msg), format, ap);
  va_end(ap);
  Serial.println(msg);
#endif
}

// Called by WiFiManager when there are config values to be saved
void saveConfigCallback () {
  shouldSaveConfig = true;
}

// Read config parameters from the SPIFFS filesystem
// Returns false if config file cannot be loaded
bool loadConfig() {
  if (SPIFFS.exists("/config.jsn")) {
    File configFile = SPIFFS.open("/config.jsn", "r");
    if (configFile) {
      char buf[1024];
      size_t len = configFile.readBytes(buf, sizeof(buf));
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, buf, len);
      if (!error) {      
        const char* tz = doc["tz"];
        const char* url = doc["url"];
        if (tz != NULL) strcpy(param_tz, tz);
        if (url != NULL) strcpy(param_url, url);
      }
    }
  }
}

// Write config parameters to the SPIFFS filesystem
void saveConfig() {
  DynamicJsonDocument doc(1024);
  doc["tz"] = param_tz;
  doc["url"] = param_url;
  File configFile = SPIFFS.open("/config.jsn", "w");
  if (configFile) {
    serializeJson(doc, configFile);
    configFile.close();
  }
}

bool getNetworkTime() {
  String url = param_url;
  String tzstr = param_tz;
  tzstr.replace("/", "%2F");
  url.replace("[tz]", tzstr);
  HTTPClient http;
  http.begin(url.c_str());
  int rc = http.GET();
  debug("http.get() rc=%d", rc);
  if (rc == HTTP_CODE_OK) {
    String payload = http.getString();
    if (payload.length() >= 8) {
      String hh = payload.substring(0, 2);
      String mm = payload.substring(3, 5);
      String ss = payload.substring(6, 8);
      netHH = hh.toInt();
      netMM = mm.toInt();
      netSS = ss.toInt();
      if (netHH >= 12) netHH -= 12;
      if (netHH < 0 || netHH > 23) netHH = 0;
      if (netMM < 0 || netMM > 59) netMM = 0;
      if (netSS < 0 || netSS > 59) netSS = 0;
      return true;
    }
  }
  return false;
}

// This function returns immediately if WiFi connection is made successfully using previous credentials.
// Otherwise it waits until user enters correct credentials for WiFi connection, or timeout after 5 minutes and go into deep-sleep.
// Onboard blue LED of ESP-7 (GPIO2) is used to indicate device has started in AP mode and waiting for configuration.
void initAP() {
  wifiManager.setCustomHeadElement(jscript);
  wifiManager.addParameter(&form_clockTime);
  wifiManager.addParameter(&form_timezone);
  wifiManager.addParameter(&form_scriptUrl);
  wifiManager.setConfigPortalTimeout(5*60);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  String clockName = String("ESPCLOCK3-") + ESP.getChipId();
  bool success = wifiManager.autoConnect(clockName.c_str());
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(LED_BUILTIN, INPUT_PULLUP);
  if (!success) ESP.deepSleep(2*60*60*1e6, WAKE_RF_DEFAULT);
}

// Parse config values entered in AP form
void parseConfig() {
  strncpy(param_tz, form_timezone.getValue(), sizeof(param_tz) - 1);
  strncpy(param_url, form_scriptUrl.getValue(), sizeof(param_url) - 1);
  int clockTime = atoi(form_clockTime.getValue());
  int clockMMSS = clockTime % 10000;
  clockHH = clockTime / 10000;
  clockMM = clockMMSS / 100;
  clockSS = clockMMSS % 100;
  while(clockHH >= 12) clockHH -= 12;
  if (clockMM > 59) clockMM = 0;    
  if (clockSS > 59) clockSS = 0;
  attinySetClock();
}

void attinyStartClock() {
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  byte start_clock[] = { CMD_START_CLOCK, 0xCC ^ CMD_START_CLOCK };
  debug("CMD_START_CLOCK (%d)", Wire.write(start_clock, sizeof(start_clock)));
  Wire.endTransmission();
  delay(500);
}

void attinyStopClock() {
  delay(200);
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  byte stop_clock[] = { CMD_STOP_CLOCK, 0xCC ^ CMD_STOP_CLOCK };
  Wire.write(stop_clock, sizeof(stop_clock));
  Wire.endTransmission();
}

void attinySetClock() {
  byte setclock[] = { CMD_SET_CLOCK, clockHH, clockMM, clockSS, 0xCC };
  for (int i=0; i<sizeof(setclock)-1; i++) setclock[sizeof(setclock)-1] ^= setclock[i];
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  int count = Wire.write(setclock, sizeof(setclock));
  debug("CMD_SET_CLOCK (%d) = %02d:%02d:%02d", count, clockHH, clockMM, clockSS);
  Wire.endTransmission();
  delay(500);
}

void attinySetNetTime() {
  byte nettime[] = { CMD_SET_NETTIME, netHH, netMM, netSS, 0xCC };
  for (int i=0; i<sizeof(nettime)-1; i++) nettime[sizeof(nettime)-1] ^= nettime[i];
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  int count = Wire.write(nettime, sizeof(nettime));
  debug("CMD_SET_NETTIME (%d) = %02d:%02d:%02d", count, netHH, netMM, netSS);
  Wire.endTransmission();
  delay(500);
}

void setup() {
#ifdef BT_DEBUG
  Serial.begin(9600);
#endif
  
  // Reduce power drain
  WiFi.setOutputPower(0.0);

  // Setup I2C - SDA (GPIO4), SCL (GPIO5)
  Wire.begin();
  Wire.setClockStretchLimit(1500); // to accommodate for ATTiny85's slower speed at 1MHz

  // Enable 1HZ clock output signal on RTC
  rtc.setSquareWave(SQW_1HZ);
  
  // Setup SPIFFS and load config values
  if (!SPIFFS.begin()) ESP.deepSleep(2*60*60*1e6, WAKE_RF_DEFAULT);
  loadConfig();

  // If reset detected, start in config mode
  delay(1000); // Wait a little for debounce
  pinMode(CONFIG_PIN, INPUT_PULLUP);
  bool reset = !digitalRead(CONFIG_PIN); 
  if (reset) attinyStopClock();
 
  // Start WiFiManager
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  if (reset) wifiManager.resetSettings();
  initAP();

  // At this point, shouldSaveConfig will tell us whether WiFiManager has connected
  // using previously stored SSID/password, or new ones has been into the config portal
  // and should be saved to flash memory.
  if (shouldSaveConfig) {
    parseConfig();
    saveConfig();
  } 
  
  // Tell ATtiny85 to start running the clock
  attinyStartClock();

  // Get network time
  bool rc = getNetworkTime();
  if (rc) attinySetNetTime();

  // 2 hour deep sleep
  ESP.deepSleep(2*60*60*1e6, WAKE_RF_DEFAULT);
}

void loop() {
  // When ESP8266 wakes up from deep sleep, setup() is called.
}
