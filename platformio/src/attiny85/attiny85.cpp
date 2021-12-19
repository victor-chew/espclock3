/*
 * attiny85.cpp
 *
 * Copyright 2020 Victor Chew
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unle_SS required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either expre_SS or implied.
 * See the License for the specific language governing permi_SSions and
 * limitations under the License.
 */

#include <Arduino.h>
#include <Wire.h>
#include <limits.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <EEPROM.h>

#define _HH 0
#define _MM 1
#define _SS 2

#define I2C_SLAVE_ADDR 0x12
#define CMD_START_CLOCK 0x2
#define CMD_STOP_CLOCK 0x4
#define CMD_SET_CLOCK 0x6
#define CMD_SET_NETTIME 0x8

#define SQW_PIN PB1
#define PULSE_WIDTH 40

bool clock_running = false;
byte tickpin = PB3, prev_net_ss = 255;

// Variables used by ISRs must be marked as volatile
volatile bool check_hibernate = false;
volatile byte numticks = 0;
volatile byte clocktime[3] = {0}; // physical clock time
volatile byte nettime[3] = {0};   // network clock time
volatile byte msg[5] = {0};

// Function prototypes
static void incTime(volatile byte& hh, volatile byte& mm, volatile byte& ss);
static void processMessage();
static void enablePinChangeInterrupt(bool enable);
static void writeEEPROM();

// Triggers every ~8s to check whether we should save status and hibernate
ISR(WDT_vect) {
  check_hibernate = true;
}

// Pin change interrupt; respond to falling edge of 1Hz clock signal connected to PB1
ISR(PCINT0_vect) {
  if (digitalRead(SQW_PIN) == LOW) numticks += 1;
}

// Process messages from master
void receiveEvent(int numbytes) {
  if (numbytes > (int)sizeof(msg)) return; // Sanity check
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
      prev_net_ss = 255;
      enablePinChangeInterrupt(true);
      break;
    case CMD_STOP_CLOCK:
      clock_running = false;
      enablePinChangeInterrupt(false);
      writeEEPROM();
      break;
    case CMD_SET_CLOCK:
      memcpy(clocktime, (void*)(msg+1), sizeof(clocktime));
      memcpy((void*)nettime, clocktime, sizeof(nettime));
      break;
    case CMD_SET_NETTIME:
      memcpy((void*)nettime, (void*)(msg+1), sizeof(nettime));
      break;
  }
}

void enablePinChangeInterrupt(bool enable) {
  if (enable) GIMSK |= bit(PCIE); 
  else GIMSK &= ~bit(PCIE);
}

void incTime(volatile byte& hh, volatile byte& mm, volatile byte& ss) {
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
void incSecondHand() {
  incTime(clocktime[_HH], clocktime[_MM], clocktime[_SS]);
  digitalWrite(tickpin, HIGH);
  delay(PULSE_WIDTH);
  digitalWrite(tickpin, LOW);
  tickpin = (tickpin == PB3) ? PB4 : PB3;
}

// Pulse the same pin, which will cause the second hand to vibrate but not advance.
// This lets the user know the clock is waiting for network time to catch up.
void pulseSecondHand() {
  digitalWrite(tickpin, HIGH);
  delay(PULSE_WIDTH);
  digitalWrite(tickpin, LOW);
}

// If clock time != network time, adjust until they match (returns false)
// Otherwise put MCU to sleep (returns true).
bool synchronizeClock() {
  // Calculate time difference between physical clock and network time in seconds
  long diff = ((clocktime[_HH] * 3600L) + (clocktime[_MM] * 60L) + clocktime[_SS]) -
              ((nettime[_HH] * 3600L) + (nettime[_MM] * 60L) + nettime[_SS]);
  if (diff == 0) return true;
  if (diff < 0) diff = (12L*60*60) + diff;

  // If clock time is ahead about 5 mins, pulse second hand and wait for network time to catch up
  if (diff <= 5*60) {
    pulseSecondHand();
    return true;
  }

  // Otherwise, fast-forward clock and try to catch up
  incSecondHand();
  if (memcmp(clocktime, (const void*)nettime, sizeof(clocktime)) == 0) return true;

  // Additional delay between clock ticks during fast-forwarding to prevent slippage
  delay(PULSE_WIDTH*2);
  return false;
}

// EEPROM: _HH MM _SS TICKPIN CHECKSUM
// CHECKSUM = 0xCC ^ _HH ^ MM ^ _SS ^ TICKPIN
void readEEPROM() {
  byte checksum = 0xCC, buf[4];
  for (unsigned int i=0; i<sizeof(buf); i++) {
    buf[i] = EEPROM.read(i);
    checksum ^= buf[i];
  }
  byte checksum2 = EEPROM.read(sizeof(buf));

  // Good checksum; use values in EEPROM
  if (checksum == checksum2) {
    memcpy(clocktime, buf, sizeof(clocktime));
    memcpy((void*)nettime, buf, sizeof(nettime));
    tickpin = buf[3];
  } 
  // Bad checksum; use default values
  else {
    memset(clocktime, 0, sizeof(clocktime));
    memset((void*)nettime, 0, sizeof(nettime));
    tickpin = PB3;
  }
}

void writeEEPROM() {
  byte checksum = 0xCC ^ clocktime[_HH] ^ clocktime[_MM] ^ clocktime[_SS] ^ tickpin;
  byte buf[] = { clocktime[_HH], clocktime[_MM], clocktime[_SS], tickpin, checksum }; 
  for (unsigned int i=0; i<sizeof(buf); i++) EEPROM.write(i, buf[i]);
}

void requestEvent() {
  for (int i=0; i<3; i++) Wire.write(clocktime[i]);
}

void setup() {
  // Initialization
  pinMode(SQW_PIN, INPUT_PULLUP);

  // Turn off ADC to save power during sleep
  ADCSRA &= ~bit(ADEN);
  power_adc_disable(); 
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  // Read existing config values from EEPROM
  readEEPROM();

  cli();

  // Setup as I2C slave
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  // Enable pin change interrupt for PCINT1 (PB6) to process clock signal from RTC
  // Global pin change interrupt will be enabled when clock is started
  PCMSK |= (1 << PCINT1);

  // Watchdog timer triggers every 8s to check if we should hibernate
  MCUSR &= ~bit(WDRF);
  WDTCR |= bit(WDCE) | bit(WDE);
  WDTCR = bit(WDIE) | bit(WDP3) | bit(WDP0);

  sei();
}

void loop() {
  // Increment net time by accumulated number of ticks
  for (int i=0; i<numticks; i++) incTime(nettime[_HH], nettime[_MM], nettime[_SS]);
  numticks = 0;

  // Go into hibernation if clock time (seconds) is not being incremented.
  // This indicates that the RTC has been powered down.
  if (check_hibernate) {
    if (clock_running && prev_net_ss == nettime[_SS]) {
      msg[0] = CMD_STOP_CLOCK;
      processMessage();
    }
    prev_net_ss = nettime[_SS];
    check_hibernate = false;
  }

  // Continue running without sleeping if we are trying to sync physical clock with network clock.
  bool sleep = true;
  if (clock_running) sleep = synchronizeClock();

  if (sleep) {
    sleep_cpu();
    // There is something I don't understand here. If this small delay is not present, it seems
    // receiveEvent() will not be called when an i2c event arrives while the CPU is in power-down mode.
    delay(1);
  }
}