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
// Xtensa LX ABI: see https://www.cadence.com/content/dam/cadence-www/global/en_US/documents/tools/silicon-solutions/compute-ip/isa-summary.pdf

.text

.global interrupts
        .type   interrupts, @function
        .weak   interrupts
.global noInterrupts
        .type   noInterrupts, @function
        .weak   noInterrupts

interrupts:
#if defined(__AVR__)
    sei
#elif defined(__Z80__)
    ei
#elif defined(__arm__)
    cpsie i
#elif defined(__XTENSA__)
    rsil    a8, 0 // set 0
#endif

#if defined(__AVR__) && defined(__Z80__)
    ret
#elif defined(__arm__)
    bx lr
#elif defined(__XTENSA__)
    retw.n
#endif


noInterrupts:
#if defined(__AVR__)
    cli
#elif defined(__Z80__)
    di
#elif defined(__arm__)
    cpsie i
#elif defined(__XTENSA__)
    rsil    a8, 3 // set 3
#endif

#if defined(__AVR__) && defined(__Z80__)
    ret
#elif defined(__arm__)
    bx lr
#elif defined(__XTENSA__)
    retw.n
#endif
