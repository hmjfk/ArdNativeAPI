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
#if !defined(ARDMATH_HPP)
#define ARDMATH_HPP

#if defined(__cpp_lib_math_constants)
#include<numbers>
#else
namespace std::numbers
{
    template <typename T>
    inline constexpr T pi_v;
    template <>
    inline constexpr float pi_v<float> = 3.141592653589793238462643383279502;
    template <>
    inline constexpr double pi_v<double> = 3.141592653589793238462643383279502;
    template <>
    inline constexpr long double pi_v<long double> = 3.141592653589793238462643383279502;
}
#endif


#if defined(sq)
#undef sq
#endif

#if defined (round)
#undef round
#endif

#if defined(constrain)
#undef constrain
#endif

#if defined(radians)
#undef radians
#endif

#if defined(degrees)
#undef degrees
#endif

#if defined(lowByte)
#undef lowByte
#endif

#if defined(highByte)
#undef highByte
#endif

#if defined(bitRead)
#undef bitRead
#endif

#if defined(bitSet)
#undef bitSet
#endif

#if defined(bitClear)
#undef bitClear
#endif

#if defined(bitToggle)
#undef bitToggle
#endif

#if defined(bitWrite)
#undef bitWrite
#endif

#if defined (bit)
#undef bit
#endif

// C++標準で定義されていない関数は総称型で再実装。
template<typename T>
constexpr T sq(T a)
{
    return a * a;
}

template<typename T>
constexpr T round(T x)
{
    return x >= 0 ? (long)(x + 0.5) : (long)(x - 0.5);
}

template<typename T1, typename T2, typename T3>
constexpr void constrain(T1 amt, T2  low,T3  high)
{
    (amt < low) ? low : ( amt > high ? high : amt );
}

template<typename T>
constexpr T radians(T rad)
{
    return (std::numbers::pi_v<long double> / 180.0L) * rad;
}

template<typename T>
constexpr T degrees(T deg)
{
    return (180.0L / std::numbers::pi_v<long double> ) * deg;
}

template<typename T>
constexpr uint8_t lowByte(T w)
{
    return uint8_t(w & 0xff);
}

template<typename T>
constexpr uint8_t highByte(T w)
{
    return uint8_t(w >> 8);
}

template<typename T1, typename T2>
constexpr void bitRead(T1&& value, T2&& bit)
{
    return (value >> bit) & 0x01;
}

template<typename T1, typename T2>
constexpr void bitSet(T1&& value, T2&& bit)
{
    value |= (1UL << bit);
}

template<typename T1, typename T2>
constexpr void bitClear(T1&& value, T2&& bit)
{
    value &= ~(1UL << bit);
}

template<typename T1, typename T2>
constexpr void bitToggle(T1&& value, T2&& bit)
{
    value ^= (1UL << bit);
}

template<typename T1, typename T2, typename T3>
constexpr void bitWrite(T1&& value, T2&& bitvalue, T3&& bit)
{
    bitvalue ? bitSet(value, bit) : bitClear(value, bit);
}

template<typename T>
constexpr T bit(T b)
{
    return 1UL << b;
}

// template overload
// original is long type only
template<typename T>
constexpr T map(T x, T in_min, T in_max, T out_min, T out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#endif // end ARDMATH_HPP header
