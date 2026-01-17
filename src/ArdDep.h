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
  Original License:
  
  Arduino.h - Main include file for the Arduino SDK
  Copyright (c) 2005-2013 Arduino Team.  All right reserved.

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
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#if !defined(ARDDEP_H)
#define ARDDEP_H

// ここで定義解除しないと後のtypedefが書き換わってしまう。
#if defined(word)
#warning "Please don't use word macro to avoid name collisions. Use the makeword instead."
#undef word
#endif

#if defined(F)
#warning "Instead of Arduino's F macro, use [[gnu::progmem]] attribute."
#undef F
#endif

[[deprecated("Avoid using Arduino`s unique boolean type. Use the built-in bool type instead.")]]
typedef bool                    boolean;
[[deprecated]] typedef uint8_t  byte;
[[deprecated]] typedef uint16_t word;

#if __has_include(<WCharacter.h>)
[[deprecated("Instead, please use the <ctype.h> function.")]]
inline bool isAlphaNumeric(int c);
[[deprecated("Instead, please use the <ctype.h> function.")]]
inline bool isAlpha(int c);
[[deprecated("Instead, please use the <ctype.h> function.")]]
inline bool isAscii(int c);
[[deprecated("Instead, please use the <ctype.h> function.")]]
inline bool isWhitespace(int c);
[[deprecated("Instead, please use the <ctype.h> function.")]]
inline bool isControl(int c);
[[deprecated("Instead, please use the <ctype.h> function.")]]
inline bool isDigit(int c);
[[deprecated("Instead, please use the <ctype.h> function.")]]
inline bool isGraph(int c);
[[deprecated("Instead, please use the <ctype.h> function.")]]
inline bool isLowerCase(int c);
[[deprecated("Instead, please use the <ctype.h> function.")]]
inline bool isPrintable(int c);
[[deprecated("Instead, please use the <ctype.h> function.")]]
inline bool isPunct(int c);
[[deprecated("Instead, please use the <ctype.h> function.")]]
inline bool isSpace(int c);
[[deprecated("Instead, please use the <ctype.h> function.")]]
inline bool isUpperCase(int c);
[[deprecated("Instead, please use the <ctype.h> function.")]]
inline bool isHexadecimalDigit(int c);
[[deprecated("Instead, please use the <ctype.h> function.")]]
inline int toAscii(int c);
[[deprecated("Instead, please use the <ctype.h> function.")]]
inline int toLowerCase(int c);
[[deprecated("Instead, please use the <ctype.h> function.")]]
inline int toUpperCase(int c);
#endif
// C++標準で定義される関数などはマクロを外して非推奨。

#if defined(abs)
#undef abs
#warning "Instead of Arduino's abs, use std::abs from <cstdlib> or <stdlib.h>."
#endif

#if defined(max)
#undef max
#warning "Instead of Arduino's abs, use std::max from <algorithm>."
#endif

#if defined(min)
#undef min
#warning "Instead of Arduino's abs, use std::min from <algorithm>."
#endif
[[deprecated("It`s advisable to define the main function.")]] void setup();
[[deprecated("Please use an infinite loop statement.")]] void loop();

// Arduinoと同じ方法でC++やC言語のI/O libsが実装されていたので、非推奨としないことに決めた。
/*
[[deprecated("Please use the HardwareSerial class directly instead of an instance of the `Serial`.")]]
extern HardwareSerial Serial;

[[deprecated("Please use the HardwareSerial class directly instead of an instance of the `Serial1`.")]]
extern HardwareSerial Serial1;

[[deprecated("Please use the HardwareSerial class directly instead of an instance of the `Serial2`.")]]
extern HardwareSerial Serial2;

[[deprecated("Please use the HardwareSerial class directly instead of an instance of the `Serial3`.")]]
extern HardwareSerial Serial3;
*/
#endif // end ARDDEP_H header
