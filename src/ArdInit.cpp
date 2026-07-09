/*
    ArdNativeAPI - Libraries that enable Arduino to be used as an API.
    Copyright (c) 2005-2013 Arduino Team.  All right reserved.
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
/*
Original Source:
https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/main.cpp
https://github.com/arduino/ArduinoCore-samd/blob/master/cores/arduino/main.cpp
https://github.com/arduino/ArduinoCore-zephyr/blob/main/cores/arduino/main.cpp
*/
// 
// #define ARDNATIVE_SRC
#include "Ardinit.h"
#include "ArdCore.h"

// <Arduino.h> cannot be included. instead, manual decl.

class USBDeviceClass
{
public:
  // other menber function is unnecessary. That's why deleted it.
  void init();
  bool attach();

private:
  bool initialized; // ABI portability
};
extern USBDeviceClass USBDevice;

[[gnu::weak]] void serialEventRun();
// Weak empty variant initialization function.
// May be redefined by variant files.
[[gnu::weak]] void initVariant();
[[gnu::weak]] void initVariant() {}

[[gnu::weak]] void setupUSB();
[[gnu::weak]] void setupUSB() {}

#if defined(__ZEPHYR__)
[[gnu::weak]] void __loopHook();
[[gnu::weak]] void __loopHook() {}
#endif

extern "C"
{

extern void __libc_init_array(); // for ArduinoCore-samd
extern void init();

#if defined (__ZEPHYR__)
    void __loopHook();
    #include "zephyr/kernel.h"
    #ifdef CONFIG_LLEXT
        #include <zephyr/llext/symbol.h>
    #endif
#endif


void initCore()
{
    init();

#if defined(ARDUINO_ARCH_SAMD)
    __libc_init_array();
#endif
    initVariant();
#if defined(ARDUINO_ARCH_SAMD)
    delay(1);
#endif
#if defined(USBCON)
    #if defined(ARDUINO_ARCH_SAMD)
    USBDevice.init();
    #endif
    USBDevice.attach();
#endif
}

void serialUpdate()
{
    if(serialEventRun)
        serialEventRun();
}

} // end extern "C"