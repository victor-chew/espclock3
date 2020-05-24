/*
 * attiny85.ino
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

#include <limits.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <EEPROM.h>
#include <Wire.h>

#define HH 0
#define MM 1
#define SS 2

#define I2C_SLAVE_ADDR 0x12
#define CMD_START_CLOCK 0x2
#define CMD_STOP_CLOCK 0x4
#define CMD_SET_CLOCK 0x6
#define CMD_SET_NETTIME 0x8

#define SQW_PIN PB1
#define VCC_THRESHOLD 2800
#define PULSE_WIDTH 40

bool clock_running = false, hibernate = false;
byte tickpin = PB3, clocktime[3]; // physical clock time
uint16_t vcc = 3300;

// Variables used by ISRs must be marked as volatile
volatile bool check_vcc = false;
volatile byte nettime[3]; // network clock time
volatile byte msg[5];

//#define BT_DEBUG
#ifdef BT_DEBUG
#include <SendOnlySoftwareSerial.h>
SendOnlySoftwareSerial btserial(PB4);
void debug(const char *format, ...) {
  char msg[128];
  va_list ap;
  va_start(ap, format);
  vsnprintf(msg, sizeof(msg), format, ap);
  va_end(ap);
  btserial.print(msg);
  btserial.flush();
}
#endif

// Triggers every ~8s to check VCC level
ISR(WDT_vect) {
  check_vcc = true;
}

// Pin change interrupt; respond to falling edge of 1Hz clock signal connected to PB1
ISR(PCINT0_vect) {
  if (digitalRead(SQW_PIN) == LOW) incClockTime(nettime[HH], nettime[MM], nettime[SS]);
}

void receiveEvent(uint8_t numbytes) {
  if (numbytes > sizeof(msg)) return; // Sanity check
  int idx = 0;
  while(idx < numbytes) {
    if (Wire.available()) msg[idx++] = Wire.read();
  }
  byte checksum = 0xCC;
  for (int i=0; i<numbytes-1; i++) checksum ^= msg[i];
  if (checksum == msg[numbytes-1]) processMessage();
}  

void processMessage() {
  switch(msg[0]) {
    case CMD_START_CLOCK:
      clock_running = true;
      enablePinChangeInterrupt(true);
      break;
    case CMD_STOP_CLOCK:
      clock_running = false;
      enablePinChangeInterrupt(false);
      break;
    case CMD_SET_CLOCK:
      memcpy(clocktime, msg+1, sizeof(clocktime));
      memcpy(nettime, clocktime, sizeof(nettime));
      break;
    case CMD_SET_NETTIME:
      memcpy(nettime, msg+1, sizeof(nettime));
      break;
  }
}

void enablePinChangeInterrupt(bool enable) {
  if (enable) GIMSK |= bit(PCIE); 
  else GIMSK &= ~bit(PCIE);
}

void incClockTime(volatile byte& hh, volatile byte& mm, volatile byte& ss) {
  if (++ss >= 60) {
    ss = 0;
    if (++mm >= 60) {
      mm = 0;
      if (++hh >= 12) {
        hh = 0;
      }
    }
  }
}

// Move second hand one tick clockwise.
// See: http://www.cibomahto.com/2008/03/controlling-a-clock-with-an-arduino/
// PB3 and PB4 are connected to twin wires on clock via 100ohm resistors
void incSecondHand() {
  incClockTime(clocktime[HH], clocktime[MM], clocktime[SS]);
#ifdef BT_DEBUG
#else
  digitalWrite(tickpin, HIGH);
  delay(PULSE_WIDTH);
  digitalWrite(tickpin, LOW);
  tickpin = (tickpin == PB3) ? PB4 : PB3;
#endif
}

// Pulse the same pin, which will cause the second hand to vibrate but not advance.
// This lets the user know the clock is waiting for network time to catch up.
void pulseSecondHand() {
#ifdef BT_DEBUG
#else
  digitalWrite(tickpin, HIGH);
  delay(PULSE_WIDTH);
  digitalWrite(tickpin, LOW);
#endif
}

// If clock time != network time, adjust until they match (returns false)
// Otherwise put MCU to sleep (returns true).
bool synchronizeClock() {
  // Calculate time difference between physical clock and network time in seconds
  long diff = ((clocktime[HH] * 3600L) + (clocktime[MM] * 60L) + clocktime[SS]) - ((nettime[HH] * 3600L) + (nettime[MM] * 60L) + nettime[SS]);
  if (diff == 0) return true;
  if (diff < 0) diff = (12L*60*60) + diff;

  // If clock time is ahead about 5 mins, pulse second hand and wait for network time to catch up
  if (diff <= 5*60) {
    pulseSecondHand();
    return true;
  }
  
  // Otherwise, fast-forward clock and try to catch up
  incSecondHand();
  if (memcmp(clocktime, nettime, sizeof(clocktime)) == 0) return true;

  // Additional delay between clock ticks during fast-forwarding to prevent slippage
  delay(PULSE_WIDTH*2);
  return false;
}

// EEPROM: HH MM SS TICKPIN CHECKSUM
// CHECKSUM = 0xCC ^ HH ^ MM ^ SS ^ TICKPIN
void readEEPROM() {
  byte checksum = 0xCC, buf[4];
  for (int i=0; i<sizeof(buf); i++) {
    buf[i] = EEPROM.read(i);
    checksum ^= buf[i];
  }
  byte checksum2 = EEPROM.read(sizeof(buf));

  // Good checksum; use values in EEPROM
  if (checksum == checksum2) {
    memcpy(clocktime, buf, sizeof(clocktime));
    memcpy(nettime, buf, sizeof(nettime));
    tickpin = buf[3];
  } 
  // Bad checksum; use default values
  else {
    memset(clocktime, 0, sizeof(clocktime));
    memset(nettime, 0, sizeof(nettime));
    tickpin = PB3;
  }
}

void writeEEPROM() {
  byte checksum = 0xCC ^ clocktime[HH] ^ clocktime[MM] ^ clocktime[SS] ^ tickpin;
  byte buf[] = { clocktime[HH], clocktime[MM], clocktime[SS], tickpin, checksum }; 
  for (int i=0; i<sizeof(buf); i++) EEPROM.write(i, buf[i]);
}

void checkLowVoltage() {
  // Initialize ADC to measure VCC using interal 1.1V bandgap as reference
  check_vcc = false;
  ADMUX = bit(MUX3) | bit(MUX2); 
  delay(1); // Wait for bandgap voltage to settle
  ADCSRA = bit(ADEN) | bit(ADSC) | bit(ADPS2) | bit(ADPS1) | bit(ADPS0);
  while (bit_is_set(ADCSRA, ADSC));
  vcc = (1.1*1024*1000L) / ADC; // Compute VCC (in mV)
  ADCSRA = 0; // Turn off ADC to save power

  // Stop clock and save state to EEPROM when VCC is too low
  if (vcc < VCC_THRESHOLD) {
    if (!hibernate) {
      hibernate = true;
      writeEEPROM();
    }
  } else {
    hibernate = false;
  }
}

void setup() {
#ifdef BT_DEBUG
  btserial.begin(9600);
#else
  pinMode(PB3, OUTPUT); digitalWrite(PB3, LOW);
  pinMode(PB4, OUTPUT); digitalWrite(PB4, LOW);
#endif

  // Initialization
  pinMode(SQW_PIN, INPUT_PULLUP);
  memset(clocktime, 0, sizeof(clocktime));
  memset(nettime, 0, sizeof(nettime));
  memset(msg, 0, sizeof(msg));
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  // Read existing config values from EEPROM
  readEEPROM();

  cli();

  // Setup as I2C slave
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(receiveEvent);

  // Enable pin change interrupt for PCINT1 (PB6)
  // Global pin change interrupt will be enabled when clock is started
  PCMSK |= (1 << PCINT1);

  // Watchdog timer triggers every 8s to measure VCC
  MCUSR &= ~bit(WDRF);
  WDTCR |= bit(WDCE) | bit(WDE);
  WDTCR = bit(WDIE) | bit(WDP3) | bit(WDP0);

  sei();
}

void loop() {
#ifdef BT_DEBUG
  debug("VCC=%d, CVCC=%d, MSG=%d:%d:%d:%d:%d\n", 
    vcc, check_vcc, msg[0], msg[1], msg[2], msg[3], msg[4]);
  debug("HB=%d, CR=%d, CT=%d:%d:%d, NT=%d:%d:%d\n", 
    hibernate, clock_running, 
    clocktime[HH], clocktime[MM], clocktime[SS],
    nettime[HH], nettime[MM], nettime[SS]); 
#endif
  
  // Monitor VCC every 8s. If VCC is low, save clock state and put MCU to sleep.
  if (check_vcc) checkLowVoltage();

  // Power down if 1) clocking is not running, or 2) physical clock is already in sync with network clock.
  bool sleep = true;
  if (clock_running && !hibernate) sleep = synchronizeClock();
  if (sleep) {
    sleep_cpu();
    // There is something I don't understand here. If this small delay is not present, it seems
    // receiveEvent() will not be called when an i2c event arrives while the CPU is in power-down mode.
    delay(1);
  }
}
