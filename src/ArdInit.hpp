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
  
  Common.h - Common definitions for Arduino core
  main.cpp - Main loop for Arduino sketches
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

#if !defined(ARDINIT_HPP)
#define ARDINIT_HPP
/*
Currently, since we’re using GNU C++17, the file extension is “.hpp,” but once Arduino moves to C23,
we plan to switch it to “.h.”
*/

// Weak empty variant initialization function.
// May be redefined by variant files.
[[gnu::weak]] void initVariant();
[[gnu::weak]] void initVariant()  { }

[[gnu::weak]] void setupUSB();
[[gnu::weak]] void setupUSB() { }

#if defined(__ZEPHYR__)
[[gnu::weak]] void __loopHook();
[[gnu::weak]] void __loopHook() { }
#endif

#if defined(__cplusplus)
extern "C" {
#endif

void initCore();
void serialUpdate();

#if defined(__cplusplus)
}
#endif


#endif // end ARDINIT_H
