/*
 * esp8266.cpp
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
 
#include <Arduino.h>
#include <stdint.h>
#include <LittleFS.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ESP8266HTTPClient.h>
#include <Rtc_Pcf8563.h> // https://github.com/elpaso/Rtc_Pcf8563

#define DEBUG
#ifdef DEBUG
#include <Syslog.h>
WiFiUDP udpClient;
Syslog syslog(udpClient, "192.168.1.167", 514, "ESPCLOCK3", "ESPCLOCK3", LOG_KERN);
#endif

void debug(const char *format, ...) {
  #ifdef DEBUG
    char buf[256];
    va_list ap;
    va_start(ap, format);
    vsnprintf(buf, sizeof(buf), format, ap);
    va_end(ap);
    syslog.log(buf);
  #endif
}

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
#define CONFIG_FILE "espclock.ini"
#define STOP_VOLTAGE (1.05 * 4)

// Variables
bool shouldSaveConfig = false;
byte clockHH = 0, clockMM = 0, clockSS = 0, netHH = 0, netMM = 0, netSS = 0;
char param_tz[48] = "UTC", param_url[128] = DEFAULT_SCRIPT_URL;
char buf_timezone[48] = "", buf_clockTime[10] = "", buf_scriptUrl[128] = DEFAULT_SCRIPT_URL;
WiFiManagerParameter form_timezone("timezone", "TZ database timezone code", buf_timezone, sizeof(buf_timezone)-1);
WiFiManagerParameter form_clockTime("clockTime", "Time on clock (12-hr HHMMSS)", buf_clockTime, sizeof(buf_clockTime)-1);
WiFiManagerParameter form_scriptUrl("scriptUrl", "URL to ESPCLOCK script", buf_scriptUrl, sizeof(buf_scriptUrl)-1);
WiFiManager wifiManager;
Rtc_Pcf8563 rtc;

// Called by WiFiManager when there are config values to be saved
void saveConfigCallback () {
  shouldSaveConfig = true;
}

void readLine(File& file, char* buf, unsigned int bufsize) {
  unsigned int i = 0;
  while(file.available() && i < bufsize-1) {
    buf[i] = file.read();
    if (buf[i++] == '\n') break;
  }
  buf[i] = 0;
}

bool startsWith(const char* str, const char* head) {
  unsigned int i, len = min(strlen(str), strlen(head));
  for (i=0; i<len; i++) 
    if (str[i] != head[i]) return false;
  return i == strlen(head);
}

void setParam(char* param, unsigned int paramSize, char* buf) {
  memset((void*)param, 0, paramSize);
  char* ptr = buf; while(*ptr++ != '=');
  strncpy(param, ptr, paramSize-1);
}

// Read config parameters to flash
void loadConfig() {
  char buf[256];
  memset((void*)buf, 0, sizeof(buf));
  File file = LittleFS.open(CONFIG_FILE, "r");
  if (!file) return; 
  while(true) {
    readLine(file, buf, sizeof(buf));
    if (strlen(buf) == 0) break;
    if (startsWith(buf, "tz=")) setParam(param_tz, sizeof(param_tz), buf);
    else if (startsWith(buf, "url=")) setParam(param_url, sizeof(param_url), buf);
  }
  file.close();
}

// Write config parameters to flash
void saveConfig() {
  File file = LittleFS.open(CONFIG_FILE, "w");
  if (!file) return;
  file.write("tz="); file.write(param_tz, sizeof(param_tz)); file.write('\n');
  file.write("url="); file.write(param_url, sizeof(param_url)); file.write('\n');
  file.close();
}

bool getNetworkTime() {
  String url = param_url;
  String tz = param_tz;
  tz.replace("/", "%2F");
  url.replace("[tz]", tz);
  WiFiClient wifi;
  HTTPClient http;
  http.begin(wifi, url.c_str());
  int rc = http.GET();
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

void attinyStartClock() {
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  byte start_clock[] = { CMD_START_CLOCK, 0xCC ^ CMD_START_CLOCK };
  Wire.write(start_clock, sizeof(start_clock));
  Wire.endTransmission();
  delay(100);
}

void attinyStopClock() {
  delay(200);
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  byte stop_clock[] = { CMD_STOP_CLOCK, 0xCC ^ CMD_STOP_CLOCK };
  Wire.write(stop_clock, sizeof(stop_clock));
  Wire.endTransmission();
  delay(100);
}

void attinySetClock() {
  byte setclock[] = { CMD_SET_CLOCK, clockHH, clockMM, clockSS, 0xCC };
  for (unsigned int i=0; i<sizeof(setclock)-1; i++) setclock[sizeof(setclock)-1] ^= setclock[i];
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(setclock, sizeof(setclock));
  Wire.endTransmission();
  delay(100);
}

void attinySetNetTime() {
  byte nettime[] = { CMD_SET_NETTIME, netHH, netMM, netSS, 0xCC };
  for (unsigned int i=0; i<sizeof(nettime)-1; i++) nettime[sizeof(nettime)-1] ^= nettime[i];
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(nettime, sizeof(nettime));
  Wire.endTransmission();
  delay(100);
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
  digitalWrite(LED_BUILTIN, LOW); // Note: built-in LED is active low
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
  if (clockTime < 10000) clockTime *= 100;
  clockSS = clockTime % 100; if (clockSS >= 60) clockSS = 0;
  clockMM = (clockTime / 100) % 100; if (clockMM >= 60) clockMM = 0;
  clockHH = clockTime / 10000; while(clockHH >= 12) clockHH -= 12;
  attinySetClock();
}

/**
 * I2C_ClearBus
 * (http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html)
 * (c)2014 Forward Computing and Control Pty. Ltd.
 * NSW Australia, www.forward.com.au
 * This code may be freely used for both private and commerical use
 */

/**
 * This routine turns off the I2C bus and clears it
 * on return SCA and SCL pins are tri-state inputs.
 * You need to call Wire.begin() after this to re-enable I2C
 * This routine does NOT use the Wire library at all.
 *
 * returns 0 if bus cleared
 *         1 if SCL held low.
 *         2 if SDA held low by slave clock stretch for > 2sec
 *         3 if SDA held low after 20 clocks.
 */
int I2C_ClearBus() {
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); // Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  delay(2500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master. 
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
  // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}

float getSupplyVoltage() {
  int sum = 0;
  analogRead(A0);
  for (int i=0; i<8; i++) sum += analogRead(A0);
  sum = sum >> 3; // divide by 8
  float ratio = sum / 1024.0;
  return ratio * 57.0 / 10.0;
}

byte I2C_Scan() {
  int num = 0;
  byte addr[5] = { 0, 0, 0, 0, 0 };

  Wire.beginTransmission(0x12);
  byte error = Wire.endTransmission();
  if (error == 0) addr[num++] = 0x12;
  delay(100);

  Wire.beginTransmission(0x51);
  error = Wire.endTransmission();
  if (error == 0) addr[num++] = 0x51;
  delay(100);

  #ifdef DEBUG
  char msg[256];
  sprintf(msg, "I2C devices (%d)", num);
  if (num > 0) {
    strcat(msg, ": ");
    for (int i=0; i<num; i++) {
      char addstr[8] = "";
      sprintf(addstr, "%02X ", addr[i]);
      strcat(msg, addstr);
    }
  }
  debug(msg);
  #endif
  
  return num;
}

void setup() {
  // Reduce power drain
  WiFi.setOutputPower(0.0);

  // Load config from flash
  if (!LittleFS.begin()) ESP.deepSleep(0);
  loadConfig();

  // Setup I2C - SDA (GPIO4), SCL (GPIO5)
  Wire.begin();
  Wire.setClockStretchLimit(1500); // to accommodate for ATTiny85's slower speed at 1MHz

  // If reset detected, start in config mode
  delay(500); // Wait a little for debounce
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

  // Test I2C line. If total number of slaves is not 2 (RTC and ATtiny85), 
  // try to reset I2C bus and try again after 1 minute deep sleep.
  if (I2C_Scan() != 2) {
    I2C_ClearBus();
    delay(1000);
    ESP.deepSleep(5*60*1e6, WAKE_RF_DEFAULT);
    while(true);
  }
  
  // Freeze when supply voltage goes below threshold
  float supply = getSupplyVoltage();
  debug("Supply = %.2f", supply);
  if (supply <= STOP_VOLTAGE) {
    rtc.setSquareWave(SQW_DISABLE);
    delay(1000);
    ESP.deepSleep(0);
  }

  // Read ATtiny85 heartbeat
  byte hb[3] = { 0 };
  Wire.requestFrom(I2C_SLAVE_ADDR, sizeof(hb));
  int numbytes = Wire.readBytes(hb, sizeof(hb));
  syslog.logf(LOG_INFO, "Heartbeat: [%d] %02d:%02d:%02d", numbytes, hb[0], hb[1], hb[2]);

  // Enable 1HZ clock output signal on RTC
  rtc.setSquareWave(SQW_1HZ);

  // Tell ATtiny85 to start ticking and send it the current network time
  attinyStartClock(); 
  if (getNetworkTime()) attinySetNetTime();

  // 2 hour deep sleep
  delay(1000);
  ESP.deepSleep(2*60*60*1e6, WAKE_RF_DEFAULT);
}

void loop() {
  // When ESP8266 wakes up from deep sleep, setup() is called.
}
