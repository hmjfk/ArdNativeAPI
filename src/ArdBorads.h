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
#if !defined(ARDBORADS_H)
#define ARDBORADS_H


// Arduino boards type macro

/*
Board name macro is boards.txt and platform.txt defined.

boards.txt:
    *.build.variant=<pins_dir>
    *.build.board=<name>

repository tree:
    /variants/<pins_dir>

platform.txt:
  -DARDUINO_<name> => ARDUINO_<name>

#if defined(__<architecture>__)  // architecture level
    #if defined(...) // board level
            #define ...
            :
            :
## board macro nameing rule
ARD_<architecture>_<name>
*/

// syxtax support: ARDUINO_
#if defined(__ZEPHYR__)
    #if defined(ARDUINO_GIGA)
        #define ARD_ZEPHYR_GIGA

    #elif defined(ARDUINO_NANO33BLE)
        #define ARD_ZEPHYR_NANO33BLE

    #elif defined(ARDUINO_ARDUINO_NANO_MATTER)
        #define ARD_ZEPHYR_ARDUINO_NANO_MATTER
        
    #elif defined(ARDUINO_NICLA_SENSE_ME)
        #define ARD_ZEPHYR_NICLA_SENSE_ME

    #elif defined(ARDUINO_OPTA)
        #define ARD_ZEPHYR_OPTA
    
    #elif defined(ARDUINO_PORTENTA_C33)
        #define ARD_ZEPHYR_PORTENTA_C33

    #elif defined(ARDUINO_PORTENTA_H7_M7)
        #define ARD_ZEPHYR_PORTENTA_H7_M7
    
    #elif defined(ARDUINO_UNO_Q)
        #define ARD_ZEPHYR_UNO_Q

    #elif defined(ARDUINO_EK_RA8D1)
        #define ARD_ZEPHYR_EK_RA8D1

    #elif defined(ARDUINO_FRDM_MCXN947)
        #define ARD_ZEPHYR_FRDM_MCXN947

    #elif defined(ARDUINO_FRDM_RW612)
        #define ARD_ZEPHYR_FRDM_RW612
    #endif // end __ZEPHYR__

#elif defined(__AVR__)
    #if defined(ARDUINO_AVR_CIRCUITPLAY)
        #define ARD_AVR_CIRCUITPLAY

    #elif defined(ARDUINO_AVR_ETHERNET)
        #define ARD_AVR_ETHERNET

    #elif defined(ARDUINO_AVR_GEMMA)
        #define ARD_AVR_GEMMA
    #elif defined(ARDUINO_AVR_LEONARDO) || defined(ARDUINO_AVR_LEONARDO_ETH) || \
          defined(ARDUINO_AVR_ESPLORA) || defined(ARDUINO_AVR_LILYPAD_USB)
        #define ARD_AVR_LEONARDO

    #elif defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_ADK) || \
          defined(ARDUINO_AVR_MEGA2560)
        #define ARD_AVR_MEGA

    #elif defined(ARDUINO_AVR_MICRO)
        #define ARD_AVR_MICRO

    #elif defined(ARDUINO_AVR_ROBOT_MOTOR)
        #define ARD_AVR_ROBOT_MOTOR

    #elif defined(ARDUINO_AVR_YUN) || defined(ARDUINO_AVR_YUNMINI) || \
          defined(ARDUINO_AVR_INDUSTRIAL101) || defined(ARDUINO_AVR_LININO_ONE)
        #define ARD_AVR_YUN
    
    // Arduino-megaavr
    #elif defined(ARDUINO_AVR_UNO_WIFI_REV2)
        #define ARD_AVR_UNO_WIFI_REV2
    
    #elif defined(ARDUINO_AVR_NANO_EVERY)
        #define ARD_AVR_NANO_EVERY
    
    #else
        #define ARD_AVR_STANDARD
    #endif // end __AVR__

#elif defined (ESP8266)
    #if defined(ESP8266_GENERIC) || defined(ESP8266_ESP01) || \
        defined(ESP8266_ESP210) || defined(GEN4_IOD)
        #define ARD_ESP8266_GENERIC
    
    #elif defined(ESP8266_ESP13)
        #define ARD_ESP8266_ESP13
    
    #elif defined(ESP8266_ESP12)
        #define ARD_ESP8266_ESP12

    #elif defined(ESP8266_ESPRESSO_LITE_V1)
        #define ARD_ESP8266_ESPRESSO_LITE_V1
    
    #elif defined(ESP8266_ESPRESSO_LITE_V2)
        #define ARD_ESP8266_ESPRESSO_LITE_V2
    
    #elif defined(ESP8266_PHOENIX_V1)
        #define ARD_ESP8266_PHOENIX_V1
    
    #elif defined(ESP8266_PHOENIX_V2)
        #define ARD_ESP8266_PHOENIX_V2

    #elif defined(ESP8266_NODEMCU)
        #define ARD_ESP8266_NODEMCU
    
    #elif defined(MOD_WIFI_ESP8266)
        #define ARD_ESP8266_MOD_WIFI
    
    #elif defined(ESP8266_THING) || defined(ESP8266_THING_DEV)
        #define ARD_ESP8266_THING
    
    #elif defined(ESP8266_WEMOS_D1MINI) || defined(ESP8266_WEMOS_D1MINIPRO) \
          defined(ESP8266_WEMOS_D1MINILITE)
        #define ARD_ESP8266_WEMOS_D1MINI
    
    #elif defined(ESP8266_WEMOS_D1R1)
        #define ARD_ESP8266_WEMOS_D1R1
    
    #elif defined(WIFINFO)
        #define ARD_ESP8266_WIFINFO
    
    #elif defined(ESP8266_ARDUINO_STAR_OTTO) || defined(ESP8266_ARDUINO_UNOWIFI)
        #define ARD_ESP8266_STAR_OTTO
    
    #elif define(ESP8266_ARDUINO_PRIMO)
        #define ARD_ESP8266_PRIMO
    
    #elif defined(ESP8266_OAK)
        #define ARD_ESP8266_OAK
    
    #elif defined(WIFIDUINO_ESP8266)
        #define ARD_ESP8266_WIFIDUINO

    #elif defined(AMPERKA_WIFI_SLOT)
        #define ARD_ESP8266_AMPERKA_WIFI_SLOT

#elif defined(ARDUINO_ARCH_RENESAS)
    #if defined(ARDUINO_PORTENTA_C33)
        #define ARD_ARCH_RENESAS_PORTENTA_C33

    #elif defined(ARDUINO_MINIMA)
        #define ARD_ARCH_RENESAS_MINIMA
    
    #elif defined(ARDUINO_UNOWIFIR4)
        #define ARD_ARCH_RENESAS_UNOWIFIR4
    
    #elif defined(ARDUINO_nanor4)
        #define ARD_ARCH_RENESAS_nanor4
    
    #elif defined(ARDUINO_OPTA_DIGITAL)
        #define ARD_ARCH_RENESAS_OPTA_DIGITAL
    
    #elif defined(ARDUINO_OPTA_ANALOG)
        #define ARD_ARCH_RENESAS_OPTA_ANALOG
    
    #elif defined(ARDUINO_MUXTO)
        #define ARD_ARCH_RENESAS_OPTA_MUXTO
    #endif // end ARCH_RENESAS

#endif // MCU if
#endif // end ARDBORADS_H