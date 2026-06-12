/*
    ArdNativeAPI - Libraries that enable Arduino to be used as an API.
    Copyright (C) 2025- Denkousi
    This program is a derivative work of ArduinoCoreAPI-avr.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
// <Arduino.h>の代替。拡張子が.inoでないときに取り込まれる。
#if !defined(ARDCORE_H) && !defined(Arduino_h)
#define ARDCORE_H

#ifdef __cplusplus
<<<<<<< HEAD
=======
#include "HardwareSerial.h"
#include "WString.h"

>>>>>>> parent of f758f80 (Update)
extern "C" {
#endif

void pinMode(pin_size_t pinNumber, PinMode pinMode);

// Digital I/O
void digitalWrite(pin_size_t pinNumber, PinStatus status);
PinStatus digitalRead(pin_size_t pinNumber);

// Analog I/O
int analogRead(pin_size_t pinNumber);
void analogReadResolution(int res);
void analogReference(uint8_t mode);
void analogWrite(pin_size_t pinNumber, int value);
void analogWriteResolution(int res);

// times
unsigned long millis();
unsigned long micros();
void delay(unsigned long);
void delayMicroseconds(unsigned int us);

// Advanced I/O
void shiftOut(pin_size_t dataPin, pin_size_t clockPin, BitOrder bitOrder, uint8_t val);
uint8_t shiftIn(pin_size_t dataPin, pin_size_t clockPin, BitOrder bitOrder);

#if defined(__cplusplus)
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L);
unsigned long pulseInLong(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L);
void tone(uint8_t _pin, unsigned int frequency, unsigned long duration = 0);
#else
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout);
unsigned long pulseInLong(uint8_t pin, uint8_t state, unsigned long timeout);
void tone(uint8_t _pin, unsigned int frequency, unsigned long duration);
#endif
void noTone(uint8_t _pin);
// interrupt
typedef void (*voidFuncPtr)();
void attachInterrupt(pin_size_t interruptNumber, voidFuncPtr callback, PinStatus mode);
void detachInterrupt(pin_size_t interruptNumber);

// other utility
void yield();
long map(long, long, long, long, long);

#ifdef __cplusplus
} // extern "C"

uint16_t makeWord(uint16_t w);
uint16_t makeWord(uint8_t h, uint8_t l);

// Random Numbers
long random(long);
long random(long, long);
void randomSeed(unsigned long);
#endif // end __cplusplus
#endif // end ARDCORE_H