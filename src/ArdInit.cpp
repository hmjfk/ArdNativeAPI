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

#include<Arduino.h>
#include"Ardinit.hpp"

extern "C" {


void initCore()
{
#if defined(__AVR_LIBC_VERSION__)
  init();
#elif defined (__ZEPHYR__)
#if(DT_NODE_HAS_PROP(DT_PATH(zephyr_user), cdc_acm) &&                                            \
	 (CONFIG_USB_CDC_ACM || CONFIG_USBD_CDC_ACM_CLASS))
  	Serial.begin(115200);
#endif
#endif // end runtime part

  initVariant();

#if defined(USBCON)
  USBDevice.attach();
#endif

#if defined (__ZEPHYR__) && defined (CONFIG_MULTITHREADING)
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