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

#if !defined(ARDINTERRUPUT_H)
#define ARDINTERRUPUT_H
// These functions are implemented in assembly for each architecture. Please refer to the interrupt directory.

#include "ArdBorads.h"

#if defined(__cplusplus)
extern "C" {
#endif

#if defined(NOT_AN_INTERRUPT)
#undef NOT_AN_INTERRUPT
constexpr uint8_t NOT_AN_INTERRUPT = -1;
#endif

#if defined(interrupts)
#undef interrupts
void interrupts();
#endif // end interrupts defined

#if defined(noInterrupts)
#undef noInterrupts
void noInterrupts();
#endif // end noInterrupts defined

#if defined(digitalPinToInterrupt)
#undef digitalPinToInterrupt
inline int8_t digitalPinToInterrupt(uint8_t p);
inline int8_t digitalPinToInterrupt(uint8_t p)
{
#if defined(ARD_AVR_LEONARDO) || defined(ARD_AVR_MICRO) || \
    defined(ARD_AVR_ROBOT_CONTROL) || \
    defined(ARD_AVR_ROBOT_MOTOR) || \
    defined(ARD_AVR_YUN)
    
    switch(p)
    {
        case 0:
            return 2;
        case 1:
            return 3;
        case 2:
            return 1;
        case 3:
            return 0;
        case 7:
            return 4;
        default:
            return NOT_AN_INTERRUPT;
    }

// ARD_AVR_STANDARD, (other)
#elif defined(__AVR__)

    switch(p)
    {
        case 2:
            return 0;

#if !defined(ARD_AVR_GEMMA)
        case 3:
            return 1;
#endif

        default:

#if defined(ARD_AVR_MEGA)
            if(p >= 18 && p <= 21)
                return 23 - p;
#endif

            return NOT_AN_INTERRUPT;
    }
#endif // end boards if
}
#endif // end digitalPinToInterrupt defined


#if defined(__cplusplus)
}
#endif

#endif // end ARDINTERRUPUT_H
