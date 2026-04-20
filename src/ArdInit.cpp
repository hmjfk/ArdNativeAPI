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
#define ARDUINO_MAIN
#include<Arduino.h>
#include"Ardinit.hpp"

using namespace arduino;

extern "C" {

#if defined (__ZEPHYR__)
void __loopHook();
    #include "zephyr/kernel.h"
    #ifdef CONFIG_LLEXT
        #include <zephyr/llext/symbol.h>
    #endif
#endif


void initCore()
{
#if defined (__ZEPHYR__)
    #if(DT_NODE_HAS_PROP(DT_PATH(zephyr_user), cdc_acm) &&                                            \
	(CONFIG_USB_CDC_ACM || CONFIG_USBD_CDC_ACM_CLASS))
  	Serial.begin(115200);
    #endif
#else
  init();
#endif

#if defined(ARDUINO_ARCH_SAMD)
    __libc_init_array();
#endif // end runtime part

    initVariant();
    #if defined(ARDUINO_ARCH_SAMD)
    delay(1);
#endif
#if defined(USBCON)
    #if defined(ARDUINO_ARCH_SAMD)
    USBDevice.init();
    #endif
    USBDevice.attach();
#endif

#if defined(__ZEPHYR__) && defined (CONFIG_MULTITHREADING)
	start_static_threads();
#endif
}

void serialUpdate()
{
#if defined(__ZEPHYR__)
    __loopHook();
#else

  if (serialEventRun)
	  	serialEventRun();
#endif
}

} // end extern "C"