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

  RingBuffer.h - Ring buffer implementation
  Copyright (c) 2014 Arduino.  All right reserved.

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
#ifndef RING_BUFFER
#define RING_BUFFER


#include <stdint.h>
#include <string.h>

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which head is the index of the location
// to which to write the next incoming character and tail is the index of the
// location from which to read.
template <int N>
class RingBuffer
{
  public:
    uint8_t aucBuffer[N];
    volatile int iHead;
    volatile int iTail;
    volatile int numElems;

  public:
    RingBuffer();
    void store_char(uint8_t c);
    void clear();
    int read_char();
    int available();
    int availableForStore();
    int peek();
    bool isFull();

  private:
    int nextIndex(int index);
    inline bool isEmpty() const { return (numElems == 0); }
};

template <int N>
RingBuffer<N>::RingBuffer()
{
    memset( aucBuffer, 0, N ) ;
    clear();
}

template <int N>
void RingBuffer<N>::store_char(uint8_t c)
{
  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if (!isFull())
  {
    aucBuffer[iHead] = c ;
    iHead = nextIndex(iHead);
    numElems = numElems + 1;
  }
}

template <int N>
void RingBuffer<N>::clear()
{
  iHead = 0;
  iTail = 0;
  numElems = 0;
}

template <int N>
int RingBuffer<N>::read_char()
{
  if (isEmpty())
    return -1;

  uint8_t value = aucBuffer[iTail];
  iTail = nextIndex(iTail);
  numElems = numElems - 1;

  return value;
}

template <int N>
int RingBuffer<N>::available()
{
  return numElems;
}

template <int N>
int RingBuffer<N>::availableForStore()
{
  return (N - numElems);
}

template <int N>
int RingBuffer<N>::peek()
{
  if (isEmpty())
    return -1;

  return aucBuffer[iTail];
}

template <int N>
int RingBuffer<N>::nextIndex(int index)
{
  return (uint32_t)(index + 1) % N;
}

template <int N>
bool RingBuffer<N>::isFull()
{
  return (numElems == N);
}

#endif /* RING_BUFFER */