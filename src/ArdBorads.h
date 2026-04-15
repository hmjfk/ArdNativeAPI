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

If you have the following comments:
    // <architecture>=***
For readability, indicate that the <architecture> name is changed.
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

#elif defined (__XTENSA__)
    #if defined(ESP8266_GENERIC) || defined(ESP8266_ESP01) || \
        defined(ESP8266_ESP210) || defined(GEN4_IOD)
        #define ARD_XTENSA_ESP8266_GENERIC
    
    #elif defined(ESP8266_ESP13)
        #define ARD_XTENSA_ESP8266_ESP13
    
    #elif defined(ESP8266_ESP12)
        #define ARD_XTENSA_ESP8266_ESP12

    #elif defined(ESP8266_ESPRESSO_LITE_V1)
        #define ARD_XTENSA_ESP8266_ESPRESSO_LITE_V1
    
    #elif defined(ESP8266_ESPRESSO_LITE_V2)
        #define ARD_XTENSA_ESP8266_ESPRESSO_LITE_V2
    
    #elif defined(ESP8266_PHOENIX_V1)
        #define ARD_XTENSA_ESP8266_PHOENIX_V1
    
    #elif defined(ESP8266_PHOENIX_V2)
        #define ARD_XTENSA_ESP8266_PHOENIX_V2

    #elif defined(ESP8266_NODEMCU)
        #define ARD_XTENSA_ESP8266_NODEMCU
    
    #elif defined(MOD_WIFI_ESP8266)
        #define ARD_XTENSA_MOD_WIFI_ESP8266
    
    #elif defined(ESP8266_THING) || defined(ESP8266_THING_DEV)
        #define ARD_XTENSA_ESP8266_THING
    
    #elif defined(ESP8266_WEMOS_D1MINI) || defined(ESP8266_WEMOS_D1MINIPRO) \
          defined(ESP8266_WEMOS_D1MINILITE)
        #define ARD_XTENSA_ESP8266_WEMOS_D1MINI
    
    #elif defined(ESP8266_WEMOS_D1R1)
        #define ARD_XTENSA_ESP8266_WEMOS_D1R1
    
    #elif defined(WIFINFO)
        #define ARD_XTENSA_WIFINFO
    
    #elif defined(ESP8266_ARDUINO_STAR_OTTO) || defined(ESP8266_ARDUINO_UNOWIFI)
        #define ARD_XTENSA_ESP8266_STAR_OTTO
    
    #elif define(ESP8266_ARDUINO_PRIMO)
        #define ARD_XTENSA_ESP8266_PRIMO
    
    #elif defined(ESP8266_OAK)
        #define ARD_XTENSA_ESP8266_OAK
    
    #elif defined(WIFIDUINO_ESP8266) || defined(WIFIDUINO)
        #define ARD_XTENSA_ESP8266_WIFIDUINO

    #elif defined(AMPERKA_WIFI_SLOT)
        #define ARD_XTENSA_AMPERKA_WIFI_SLOT
    
    #elif defined (ESP32S3_DEV)
        #define ARD_XTENSA_ESP32S3_DEV

    #endif // end __XTENSA__

#elif defined(__arm__)
    #if defined (SAMD_ZERO)
        #define ARD_arm_SAMD_ZERO

    #elif defined(SAMD_MKR1000)
        #define ARD_arm_SAMD_MKR1000

    #elif defined(SAMD_MKRZERO)
        #define ARD_arm_SAMD_MKRZERO

    #elif defined(SAMD_MKRWIFI1010)
        #define ARD_arm_SAMD_MKRWIFI1010

    #elif defined(SAMD_NANO_33_IOT)
        #define ARD_arm_SAMD_NANO_33_IOT

    #elif defined(SAMD_MKRFox1200)
        #define ARD_arm_SAMD_MKRFox1200

    #elif defined(SAMD_MKRWAN1300) || defined(SAMD_MKRWAN1310)
        #define ARD_arm_SAMD_MKRWAN

    #elif defined(SAMD_MKRGSM1400)
        #define ARD_arm_SAMD_MKRGSM1400

    #elif defined(SAMD_MKRNB1500)
        #define ARD_arm_SAMD_MKRNB1500

    #elif defined(SAMD_MKRVIDOR4000)
        #define ARD_arm_SAMD_MKRVIDOR4000

    #elif defined(SAMD_CIRCUITPLAYGROUND_EXPRESS)
        #define ARD_arm_SAMD_CIRCUITPLAYGROUND_EXPRESS

    #elif defined(SAMD_TIAN)
        #define ARD_arm_SAMD_TIAN

    #elif defined(SAM_ZERO)
        #define ARD_arm_SAMD_M0

    #elif defined(__SAM3X8E__)
        #define ARD_arm_SAM_DUE

    #if defined(ARDUINO_PORTENTA_C33)
        #define ARD_arm_PORTENTA_C33

    #elif defined(ARDUINO_MINIMA)
        #define ARD_arm_MINIMA
    
    #elif defined(ARDUINO_UNOWIFIR4)
        #define ARD_arm_UNOWIFIR4
    
    #elif defined(ARDUINO_nanor4)
        #define ARD_arm_nanor4
    
    #elif defined(ARDUINO_OPTA_DIGITAL)
        #define ARD_arm_OPTA_DIGITAL
    
    #elif defined(ARDUINO_OPTA_ANALOG)
        #define ARD_arm_OPTA_ANALOG
    
    #elif defined(ARDUINO_MUXTO)
        #define ARD_arm_OPTA_MUXTO
    #endif // end __arm__
#endif // MCU if
#endif // end ARDBORADS_H