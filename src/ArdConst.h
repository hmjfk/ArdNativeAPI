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
/*
  Common.h - Common definitions for Arduino core
  Copyright (c) 2017 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/
#if !defined(ARDCONST_H)
#define ARDCONST_H

#include "ArdBorads.h"


#warning "The Arduino macro math constant has been deleted. Instead, please use <numbers>."


#undef LOW
#undef HIGH
#undef SERIAL
#undef DISPLAY
#undef CHANGE
#undef FALLING
#undef RISING

#undef INPUT
#undef OUTPUT
#undef INPUT_PULLUP
#undef INPUT_PULLDOWN
#undef OUTPUT_OPENDRAIN

#undef PI
#undef HALF_PI
#undef TWO_PI
#undef TWO_PI
#undef DEG_TO_RAD
#undef RAD_TO_DEG
#undef EULER

#undef DEC
#undef HEX
#undef OCT
#undef BIN

#undef LSBFIRST
#undef MSBFIRST

#if !__has_include(<api/Common.h>)
typedef enum : uint8_t
{
  LOW     = 0,
  HIGH    = 1,
  CHANGE  = 2,
  FALLING = 3,
  RISING  = 4,
} PinStatus;

typedef enum : uint8_t
{
  INPUT            = 0x0,
  OUTPUT           = 0x1,
  INPUT_PULLUP     = 0x2,
  INPUT_PULLDOWN   = 0x3,
  OUTPUT_OPENDRAIN = 0x4,
} PinMode;

typedef enum
{
  LSBFIRST = 0,
  MSBFIRST = 1,
} BitOrder;
#endif

enum : bool
{
  SERIAL = false,
  DISPLAY = true,
};

enum
{
  DEC = 10,
  HEX = 16,
  OCT = 8,
  BIN = 2
};

#undef LED_BUILTIN
#if defined (ARD_AVR_ROBOT_MOTOR) || defined(ARD_AVR_ROBOT_CONTROL)
// LED_BULIDIN is not defined
#elif defined(ARD_AVR_ETHERNET)
constexpr uint8_t LED_BUILTIN = 9;

#elif defined(ARD_AVR_GEMMA)
constexpr uint8_t LED_BUILTIN = 1;

#elif 0
constexpr uint8_t LED_BUILTIN = 32;

#elif 0
constexpr uint8_t LED_BUILTIN = 42;

#elif 0
constexpr uint8_t LED_BUILTIN = 6;

#elif 0
constexpr uint8_t LED_BUILTIN = 2;

#elif defined(ARD_ARCH_RENESAS_OPTA_MUXTO)
constexpr uint8_t LED_BUILTIN = 4;

#elif defined (ARD_ARCH_RENESAS_nanor4)
constexpr uint8_t LED_BUILTIN = 22;

#elif defined (ARD_ARCH_RENESAS_PORTENTA_C33)
constexpr uint8_t LED_BUILTIN = 35;

#elif 0
constexpr uint8_t LED_BUILTIN = 5;

#elif 0
constexpr uint8_t LED_BUILTIN = 16;

#elif 0
constexpr uint8_t LED_BUILTIN = 14;

#elif defined(ARD_ZEPHYR_ARDUINO_NANO_MATTER) || defined (ARD_AVR_UNO_WIFI_REV2)
constexpr uint8_t LED_BUILTIN = 25;

#else // standard
constexpr uint8_t LED_BUILTIN = 13;

#endif // end LED_BULIDIN defined
#endif // end ARDCONST_H
