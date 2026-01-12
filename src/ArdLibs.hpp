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
  Keyboard.h

  Copyright (c) 2015, Arduino LLC
  Original code (pre-library): Copyright (c) 2011, Peter Barrett

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

#if !defined (ARDLIBS_H)
#define ARDLIBS_H

#if __has_include(<Mouse.h>)
[[deprecated]]
extern Mouse_ Mouse;

namespace ArdNative
{
  using Mouse = ::Mouse_;
}

#undef MOUSE_LEFT
#undef MOUSE_RIGHT
#undef MOUSE_MIDDLE
#undef MOUSE_ALL

enum : uint8_t
{
  MOUSE_LEFT = 1,
  MOUSE_RIGHT = 2,
  MOUSE_MIDDLE = 4,
  MOUSE_ALL = MOUSE_LEFT | MOUSE_RIGHT | MOUSE_MIDDLE
}
#endif // end __has_include

#if __has_include(<Keyboard.h>)
[[deprecated]]
extern Keyboard_ Keyboard;
namespace ArdNative
{
  using Keyboard = ::Keyboard_;
}

// Replace macro constants with enums.
#undef KEY_LEFT_CTRL
#undef KEY_LEFT_SHIFT
#undef KEY_LEFT_ALT
#undef KEY_LEFT_GUI
#undef KEY_RIGHT_CTRL
#undef KEY_RIGHT_SHIFT
#undef KEY_RIGHT_ALT
#undef KEY_RIGHT_GUI

#undef KEY_UP_ARROW
#undef KEY_DOWN_ARROW
#undef KEY_LEFT_ARROW
#undef KEY_RIGHT_ARROW
#undef KEY_BACKSPACE
#undef KEY_TAB
#undef KEY_RETURN
#undef KEY_MENU
#undef KEY_ESC
#undef KEY_INSERT
#undef KEY_DELETE
#undef KEY_PAGE_UP
#undef KEY_PAGE_DOWN
#undef KEY_HOME
#undef KEY_END
#undef KEY_CAPS_LOCK
#undef KEY_PRINT_SCREEN
#undef KEY_SCROLL_LOCK
#undef KEY_PAUSE

#undef KEY_NUM_LOCK
#undef KEY_KP_SLASH
#undef KEY_KP_ASTERISK
#undef KEY_KP_MINUS
#undef KEY_KP_PLUS
#undef KEY_KP_ENTER
#undef KEY_KP_1
#undef KEY_KP_2
#undef KEY_KP_3
#undef KEY_KP_4
#undef KEY_KP_5
#undef KEY_KP_6
#undef KEY_KP_7
#undef KEY_KP_8
#undef KEY_KP_9
#undef KEY_KP_0
#undef KEY_KP_DOT

#undef KEY_F1
#undef KEY_F2
#undef KEY_F3
#undef KEY_F4
#undef KEY_F5
#undef KEY_F6
#undef KEY_F7
#undef KEY_F8
#undef KEY_F9
#undef KEY_F10
#undef KEY_F11
#undef KEY_F12
#undef KEY_F13
#undef KEY_F14
#undef KEY_F15
#undef KEY_F16
#undef KEY_F17
#undef KEY_F18
#undef KEY_F19
#undef KEY_F20
#undef KEY_F21
#undef KEY_F22
#undef KEY_F23
#undef KEY_F24

//  Keyboard
enum : signed char
{
  // Modifiers
  KEY_LEFT_CTRL    = 0x80,
  KEY_LEFT_SHIFT   = 0x81,
  KEY_LEFT_ALT     = 0x82,
  KEY_LEFT_GUI     = 0x83,
  KEY_RIGHT_CTRL   = 0x84,
  KEY_RIGHT_SHIFT  = 0x85,
  KEY_RIGHT_ALT    = 0x86,
  KEY_RIGHT_GUI    = 0x87,

  // Misc keys
  KEY_UP_ARROW     = 0xDA,
  KEY_DOWN_ARROW   = 0xD9,
  KEY_LEFT_ARROW   = 0xD8,
  KEY_RIGHT_ARROW  = 0xD7,
  KEY_BACKSPACE    = 0xB2,
  KEY_TAB          = 0xB3,
  KEY_RETURN       = 0xB0,
  KEY_MENU         = 0xED, // "Keyboard Application" in USB standard
  KEY_ESC          = 0xB1,
  KEY_INSERT       = 0xD1,
  KEY_DELETE       = 0xD4,
  KEY_PAGE_UP      = 0xD3,
  KEY_PAGE_DOWN    = 0xD6,
  KEY_HOME         = 0xD2,
  KEY_END          = 0xD5,
  KEY_CAPS_LOCK    = 0xC1,
  KEY_PRINT_SCREEN = 0xCE, // Print Screen / SysRq
  KEY_SCROLL_LOCK  = 0xCF,
  KEY_PAUSE        = 0xD0, // Pause / Break

  // Numeric keypad
  KEY_NUM_LOCK     = 0xDB,
  KEY_KP_SLASH     = 0xDC,
  KEY_KP_ASTERISK  = 0xDD,
  KEY_KP_MINUS     = 0xDE,
  KEY_KP_PLUS      = 0xDF,
  KEY_KP_ENTER     = 0xE0,
  KEY_KP_1         = 0xE1,
  KEY_KP_2         = 0xE2,
  KEY_KP_3         = 0xE3,
  KEY_KP_4         = 0xE4,
  KEY_KP_5         = 0xE5,
  KEY_KP_6         = 0xE6,
  KEY_KP_7         = 0xE7,
  KEY_KP_8         = 0xE8,
  KEY_KP_9         = 0xE9,
  KEY_KP_0         = 0xEA,
  KEY_KP_DOT       = 0xEB,

  // Function keys
  KEY_F1           = 0xC2,
  KEY_F2           = 0xC3,
  KEY_F3           = 0xC4,
  KEY_F4           = 0xC5,
  KEY_F5           = 0xC6,
  KEY_F6           = 0xC7,
  KEY_F7           = 0xC8,
  KEY_F8           = 0xC9,
  KEY_F9           = 0xCA,
  KEY_F10          = 0xCB,
  KEY_F11          = 0xCC,
  KEY_F12          = 0xCD,
  KEY_F13          = 0xF0,
  KEY_F14          = 0xF1,
  KEY_F15          = 0xF2,
  KEY_F16          = 0xF3,
  KEY_F17          = 0xF4,
  KEY_F18          = 0xF5,
  KEY_F19          = 0xF6,
  KEY_F20          = 0xF7,
  KEY_F21          = 0xF8,
  KEY_F22          = 0xF9,
  KEY_F23          = 0xFA,
  KEY_F24          = 0xFB,
};
#endif // end __has_include

#if __has_include(<Servo.h>)
struct Servo;
namespace ArdNative
{
  using Servo = ::Servo;
}
#endif // end __has_include

#if __has_include(<WiFi.h>)
namespace ArdNative
{
  using WiFi = ::WiFiClass;
}
#endif // end __has_include

#if __has_include(<SD.h>)
namespace ArdNative
{
  using SD = ::SDClass;
}
#endif // end __has_include

#if __has_include(<firmata.h>)
namespace ArdNative
{
  using Firmata = firmata::FirmataClass;
}
#endif // end __has_include

#if __has_include(<EEPROM.h>)
namespace ArdNative
{
  using EEPROM = ::EEPROMClass;
}
#endif // end __has_include

#if __has_include(<PDM.h>)
namespace ArdNative
{
  using PDM = ::PDMClass;
}
#endif
#endif // end ARDLIBS_H