/*
    ArdNativeAPI - Libraries that enable Arduino to be used as an API.
    Copyright (C) 2025- Denkousi

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
// For C header

#if !defined(ARDNATIVE_HPP) && !defined(ARDNATIVE_H)
#define ARDNATIVE_H

#if __STDC_VERSION__ < 202311L
    #error "stdc version old. requisite: C23 or later"
#endif

#if !defined(Arduino_h)
    #include <pins_arduino.h>
    #include <stdint.h>
#endif

#ifdef EXTENDED_PIN_MODE
    typedef uint32_t pin_size_t;
#else
    typedef uint8_t pin_size_t;
#endif

#include "ArdConst.h"
#include "ArdCore.h"
#include "ArdInit.hpp"
#include "ArdInterrupt.h"

#endif // end ARDNATIVE_H
