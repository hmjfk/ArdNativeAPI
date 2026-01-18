> この著作物はCC BY-SA 4.0以降でその利用が許諾されている。
> ただし、宣言の部分については、著作権の対象とならないと解されている。
# symbol list

これが、Arduino 及び ArdNative で定義される関数群の一覧である。

## ArduinoCoreAPI

### <Arduino.h>

```C++
#include <Arduino.h>


#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "binary.h"
#include "WString.h"
#include "HardwareSerial.h"
#include "USBAPI.h"


void pinMode(pin_size_t pinNumber, PinMode pinMode);

// Digital I/O
void digitalWrite(pin_size_t pinNumber, PinStatus status);
PinStatus digitalRead(pin_size_t pinNumber);

// Analog I/O
int analogRead(pin_size_t pinNumber);
void analogReadResolution(int res);
void analogReference(uint8_t mode);
void analogWrite(pin_size_t pinNumber, int value);
void analogWriteResolution(int res)

// times
unsigned long millis();
unsigned long micros();
void delay(unsigned long);
void delayMicroseconds(unsigned int us);

// Advanced I/O
void shiftOut(pin_size_t dataPin, pin_size_t clockPin, BitOrder bitOrder, uint8_t val);
uint8_t shiftIn(pin_size_t dataPin, pin_size_t clockPin, BitOrder bitOrder);
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L);
unsigned long pulseInLong(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L);
void tone(uint8_t _pin, unsigned int frequency, unsigned long duration = 0);
void noTone(uint8_t _pin);

// interrupt
void attachInterrupt(pin_size_t interruptNumber, voidFuncPtr callback, PinStatus mode);
void attachInterruptParam(pin_size_t interruptNumber, voidFuncPtrParam callback, PinStatus mode, void* param);
void detachInterrupt(pin_size_t interruptNumber);

// other utility
void yield();

uint16_t makeWord(uint16_t w);
uint16_t makeWord(uint8_t h, uint8_t l);

long map(long, long, long, long, long);

// Random Numbers
long random(long);
long random(long, long);
void randomSeed(unsigned long);

// interrupts
// The original Arduino.h defines it here, but in ArdNativeAPI it is in the “ArdInterrupt.h”.
/*
void interrupts();
void noInterrupts();
int8_t digitalPinToInterrupt(uint8_t p)

*/
```

### <pins_arduino.h>

```C++
// N is the implementation-defined number
static const uint8_t A0 = /* unspecified */ ;
static const uint8_t A1 = /* unspecified */ ;
.
.
static const uint8_t A*N* = /* unspecified */ ;

// implementation-defined extension
static const uint8_t D0 = /* unspecified */ ;
static const uint8_t D1 = /* unspecified */ ;
.
.
static const uint8_t D*N* = /* unspecified */ ;

static const uint8_t TX =  /* unspecified */ ;
static const uint8_t RX =  /* unspecified */ ;

static const uint8_t SDA =  /* unspecified */ ;
static const uint8_t SCL =  /* unspecified */ ;

static const uint8_t SS    =  /* unspecified */ ;
static const uint8_t MOSI  =  /* unspecified */ ;
static const uint8_t MISO  =  /* unspecified */ ;
static const uint8_t SCK   =  /* unspecified */ ;
```
### <WString.h>

```C++
#include <WString.h>

#include <stdlib.h>
#include <string.h>
#include <ctype.h>

namespace arduino
{
  class __FlashStringHelper;
  class StringSumHelper;

  // The string class
  class String
  {
    friend class StringSumHelper;

    typedef void (String::*StringIfHelperType)() const;
    void StringIfHelper() const;
    static size_t const FLT_MAX_DECIMAL_PLACES = 10;
    static size_t const DBL_MAX_DECIMAL_PLACES = FLT_MAX_DECIMAL_PLACES;

  public:
    // constructors
    String(const char* cstr = "");
    String(const char* cstr, unsigned int length);
    String(const uint8_t* cstr, unsigned int length);
    String(const String& str);
    String(const __FlashStringHelper* str);
    String(String& &rval);
    explicit String(char c);
    explicit String(unsigned char, unsigned char base = 10);
    explicit String(int, unsigned char base = 10);
    explicit String(unsigned int, unsigned char base = 10);
    explicit String(long, unsigned char base = 10);
    explicit String(unsigned long, unsigned char base = 10);
    explicit String(float, unsigned char decimalPlaces = 2);
    explicit String(double, unsigned char decimalPlaces = 2);
    ~String();

    // memory management
    bool reserve(unsigned int size);
    inline unsigned int length() const;
    inline bool isEmpty() const;


    String& operator=(const String& rhs);
    String& operator=(const char* cstr);
    String& operator=(const __FlashStringHelper* str);
    String& operator=(String& &rval);

    // concatenate
    bool concat(const String& str);
    bool concat(const char* cstr);
    bool concat(const char* cstr, unsigned int length);
    bool concat(const uint8_t* cstr, unsigned int length);
    bool concat(char c);
    bool concat(unsigned char num);
    bool concat(int num);
    bool concat(unsigned int num);
    bool concat(long num);
    bool concat(unsigned long num);
    bool concat(float num);
    bool concat(double num);
    bool concat(const __FlashStringHelper* str);

    String& operator+=(const String& rhs);
    String& operator+=(const char* cstr);
    String& operator+=(char c);
    String& operator+=(unsigned char num);
    String& operator+=(int num);
    String& operator+=(unsigned int num);
    String& operator+=(long num);
    String& operator+=(unsigned long num);
    String& operator+=(float num);
    String& operator+=(double num);
    String& operator+=(const __FlashStringHelper* str);

    friend StringSumHelper& operator+(const StringSumHelper& lhs, const String& rhs);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, const char* cstr);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, char c);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, unsigned char num);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, int num);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, unsigned int num);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, long num);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, unsigned long num);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, float num);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, double num);
    friend StringSumHelper& operator+(const StringSumHelper& lhs, const __FlashStringHelper* rhs);

    // comparison
    operatorStringIfHelperType() const;
    int compareTo(const String& s) const;
    int compareTo(const char* cstr) const;
    bool equals(const String& s) const;
    bool equals(const char* cstr) const;

    friend bool operator==(const String& a, const String& b);
    friend bool operator==(const String& a, const char*   b);
    friend bool operator==(const char*   a, const String& b);
    friend bool operator<(const String& a, const String& b);
    friend bool operator<(const String& a, const char*   b);
    friend bool operator<(const char*   a, const String& b);

    friend bool operator!=(const String& a, const String& b);
    friend bool operator!=(const String& a, const char*   b);
    friend bool operator!=(const char*   a, const String& b);
    friend bool operator>(const String& a, const String& b);
    friend bool operator>(const String& a, const char*   b);
    friend bool operator>(const char*   a, const String& b);
    friend bool operator<=(const String& a, const String& b);
    friend bool operator<=(const String& a, const char*   b);
    friend bool operator<=(const char*   a, const String& b);
    friend bool operator>=(const String& a, const String& b);
    friend bool operator>=(const String& a, const char*   b);
    friend bool operator>=(const char*   a, const String& b);

    bool equalsIgnoreCase(const String& s) const;
    bool startsWith(const String& prefix) const;
    bool startsWith(const String& prefix, unsigned int offset) const;
    bool endsWith(const String& suffix) const;

    // character access
    char charAt(unsigned int index) const;
    void setCharAt(unsigned int index, char c);
    char operator[](unsigned int index) const;
    char& operator[](unsigned int index);
    void getBytes(unsigned char* buf, unsigned int bufsize, unsigned int index=0) const;
    void toCharArray(char* buf, unsigned int bufsize, unsigned int index=0) const;
    const char* c_str() const;
    char* begin();
    char* end();
    const char* begin() const;
    const char* end() const;

    // search
    int indexOf(char ch) const;
    int indexOf(char ch, unsigned int fromIndex) const;
    int indexOf(const String& str) const;
    int indexOf(const String& str, unsigned int fromIndex) const;
    int lastIndexOf(char ch) const;
    int lastIndexOf(char ch, unsigned int fromIndex) const;
    int lastIndexOf(const String& str) const;
    int lastIndexOf(const String& str, unsigned int fromIndex) const;
    String substring(unsigned int beginIndex) const;
    String substring(unsigned int beginIndex, unsigned int endIndex) const;

    // modification
    void replace(char find, char replace);
    void replace(const String& find, const String& replace);
    void remove(unsigned int index);
    void remove(unsigned int index, unsigned int count);
    void toLowerCase();
    void toUpperCase();
    void trim();

    // parsing/conversion
    long toInt() const;
    float toFloat() const;
    double toDouble() const;

  protected:
    char* buffer;
    unsigned int capacity;
    unsigned int len;
  protected:
    void init();
    void invalidate();
    bool changeBuffer(unsigned int maxStrLen);

    // copy and move
    String& copy(const char* cstr, unsigned int length);
    String& copy(const __FlashStringHelper* pstr, unsigned int length);
    void move(String& rhs);
  };

  class StringSumHelper : public String
  {
  public:
      StringSumHelper(const String& s);
      StringSumHelper(const char* p);
      StringSumHelper(char c);
      StringSumHelper(unsigned char num);
      StringSumHelper(int num);
      StringSumHelper(unsigned int num);
      StringSumHelper(long num);
      StringSumHelper(unsigned long num);
      StringSumHelper(float num);
      StringSumHelper(double num);
  };
}
using arduino::__FlashStringHelper;
using arduino::String;
```

### <Printable.h>

```C++
class Printable
{
  public:
    virtual size_t printTo(Print& p) const = 0;
};
```
### <Print.h>

```C++
#include <Print.h>

#include <inttypes.h>
#include <stdio.h>

#include "String.h"
#include "Printable.h"

// Print class
class Print
{
  private:
    int write_error;
    size_t printNumber(unsigned long, uint8_t);
    size_t printFloat(double, uint8_t);
  protected:
    void setWriteError(int err = 1);
  public:
    Print();

    int getWriteError();
    void clearWriteError();

    virtual size_t write(uint8_t) = 0;
    size_t write(const char* str);
    virtual size_t write(const uint8_t* buffer, size_t size);
    size_t write(const char* buffer, size_t size);

    virtual int availableForWrite();

    size_t print(const __FlashStringHelper*);
    size_t print(const String&);
    size_t print(const char[]);
    size_t print(char);
    size_t print(unsigned char, int = DEC);
    size_t print(int, int = DEC);
    size_t print(unsigned int, int = DEC);
    size_t print(long, int = DEC);
    size_t print(unsigned long, int = DEC);
    size_t print(double, int = 2);
    size_t print(const Printable&);

    size_t println(const __FlashStringHelper*);
    size_t println(const String& s);
    size_t println(const char[]);
    size_t println(char);
    size_t println(unsigned char, int = DEC);
    size_t println(int, int = DEC);
    size_t println(unsigned int, int = DEC);
    size_t println(long, int = DEC);
    size_t println(unsigned long, int = DEC);
    size_t println(double, int = 2);
    size_t println(const Printable&);
    size_t println();

    virtual void flush();
};
```

### <Stream.h>

```C++
#include <Stream.h>

#include <inttypes.h>
#include "Print.h"

enum LookaheadMode
{
    SKIP_ALL,
    SKIP_NONE,
    SKIP_WHITESPACE
};

constexpr auto NO_IGNORE_CHAR = '\x01';

class Stream : public Print
{
  protected:
    unsigned long _timeout;
    unsigned long _startMillis;
    int timedRead();
    int timedPeek();
    int peekNextDigit(LookaheadMode lookahead, bool detectDecimal);

  public:
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;

    Stream();

  // parsing methods
  void setTimeout(unsigned long timeout);
  unsigned long getTimeout();

  bool find(char* target);
  bool find(uint8_t* target);

  bool find(char* target, size_t length);
  bool find(uint8_t* target, size_t length);
  bool find(char target);

  bool findUntil(char* target, char* terminator);
  bool findUntil(uint8_t* target, char* terminator);
  bool findUntil(char* target, size_t targetLen, char* terminate, size_t termLen);
  bool findUntil(uint8_t* target, size_t targetLen, char* terminate, size_t termLen);

  long parseInt(LookaheadMode lookahead = SKIP_ALL, char ignore = NO_IGNORE_CHAR);

  float parseFloat(LookaheadMode lookahead = SKIP_ALL, char ignore = NO_IGNORE_CHAR);


  size_t readBytes(char* buffer, size_t length);
  size_t readBytes(uint8_t* buffer, size_t length);


  size_t readBytesUntil(char terminator, char* buffer, size_t length);
  size_t readBytesUntil(char terminator, uint8_t* buffer, size_t length);
  String readString();
  String readStringUntil(char terminator);

  protected:
  long parseInt(char ignore);
  float parseFloat(char ignore);

  struct MultiTarget
  {
    const char* str;
    size_t len;
    size_t index;
  };

  int findMulti(MultiTarget* targets, int tCount);
};
```

### <HardwareSerial.h>

```C++
#include <HardwareSerial.h>

#include <inttypes.h>
#include "Stream.h"

constexpr auto SERIAL_TX_BUFFER_SIZE =  /* see below */ ;
constexpr auto SERIAL_RX_BUFFER_SIZE =  /* see below */ ;

typedef /* see below */ tx_buffer_index_t;
typedef /* see below */ tx_buffer_index_t;

class HardwareSerial : public Stream
{
  protected:
    volatile uint8_t* const _ubrrh;
    volatile uint8_t* const _ubrrl;
    volatile uint8_t* const _ucsra;
    volatile uint8_t* const _ucsrb;
    volatile uint8_t* const _ucsrc;
    volatile uint8_t* const _udr;

    bool _written;

    volatile rx_buffer_index_t _rx_buffer_head;
    volatile rx_buffer_index_t _rx_buffer_tail;
    volatile tx_buffer_index_t _tx_buffer_head;
    volatile tx_buffer_index_t _tx_buffer_tail;

    unsigned char _rx_buffer[SERIAL_RX_BUFFER_SIZE];
    unsigned char _tx_buffer[SERIAL_TX_BUFFER_SIZE];

  public:
    inline HardwareSerial(
      volatile uint8_t* ubrrh, volatile uint8_t* ubrrl,
      volatile uint8_t* ucsra, volatile uint8_t* ucsrb,
      volatile uint8_t* ucsrc, volatile uint8_t* udr);
    void begin(unsigned long baud);
    void begin(unsigned long, uint8_t);
    void end();
    virtual int available();
    virtual int peek();
    virtual int read();
    virtual int availableForWrite();
    virtual void flush();
    virtual size_t write(uint8_t);
    inline size_t write(unsigned long n);
    inline size_t write(long n)
    inline size_t write(unsigned int n)
    inline size_t write(int n)
    using Print::write;
    operator bool();

    inline void _rx_complete_irq();
    void _tx_udr_empty_irq();
};

extern HardwareSerial Serial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;

[[gnu::weak]] void serialEventRun();
```
### <IPAddress.h>
```C++

enum IPType
{
    IPv4,
    IPv6
};

class IPAddress : public Printable
{
private:
    union
    {
        uint8_t bytes[16];
        uint32_t dword[4];
    } _address;
    IPType _type;

    uint8_t* raw_address();

public:
    // Constructors
    // Default IPv4
    IPAddress();
    IPAddress(IPType ip_type);
    IPAddress(uint8_t first_octet, uint8_t second_octet, uint8_t third_octet, uint8_t fourth_octet);
    IPAddress(uint8_t o1, uint8_t o2, uint8_t o3, uint8_t o4,
              uint8_t o5, uint8_t o6, uint8_t o7, uint8_t o8,
              uint8_t o9, uint8_t o10, uint8_t o11, uint8_t o12,
              uint8_t o13, uint8_t o14, uint8_t o15, uint8_t o16);
    IPAddress(uint32_t address);
    IPAddress(const uint8_t* address);
    IPAddress(IPType ip_type, const uint8_t* address);
    IPAddress(const char* address);

    bool fromString(const char* address);
    bool fromString(const String& address);
    operator uint32_t() const;
    bool operator==(const IPAddress& addr) const;
    bool operator!=(const IPAddress& addr) const;
    bool operator==(const uint8_t* addr) const;

    uint8_t operator[](int index) const;
    uint8_t& operator[](int index);

    IPAddress& operator=(const uint8_t* address);
    IPAddress& operator=(uint32_t address);
    IPAddress& operator=(const char* address);

    virtual size_t printTo(Print& p) const;
    String toString() const;

    IPType type() const;

    friend class UDP;
    friend class Client;
    friend class Server;
    
    friend ::EthernetClass;
    friend ::DhcpClass;
    friend ::DNSClient;

protected:
    bool fromString4(const char* address);
    bool fromString6(const char* address);
    String toString4() const;
    String toString6() const;
};

extern const IPAddress IN6ADDR_ANY;
extern const IPAddress INADDR_NONE;
```
### <Udp.h>

```C++
#include<Udp.h>
#include <Stream.h>
#include <IPAddress.h>

class UDP : public Stream
{
public:
  virtual uint8_t begin(uint16_t)  = 0;
  virtual uint8_t beginMulticast(IPAddress, uint16_t);
  virtual void stop()  = 0;

  // Sending UDP packets
  virtual int beginPacket(IPAddress ip, uint16_t port)  = 0;
  virtual int beginPacket(const char* host, uint16_t port)  = 0;
  virtual int endPacket()  = 0;
  virtual size_t write(uint8_t)  = 0;
  virtual size_t write(const uint8_t* buffer, size_t size)  = 0;

  virtual int parsePacket()  = 0;
  virtual int available()  = 0;
  virtual int read()  = 0;
  virtual int read(unsigned char* buffer, size_t len)  = 0;
  virtual int read(char* buffer, size_t len)  = 0;
  virtual int peek()  = 0;
  virtual void flush()  = 0;
  virtual IPAddress remoteIP()  = 0;
  virtual uint16_t remotePort()  = 0;
protected:
  uint8_t* rawIPAddress(IPAddress& addr);
};
```

### <Client.h>

```C++
#include <Client.h>

#include "Print.h"
#include "Stream.h"
#include "IPAddress.h"

namespace arduino 
{
  class Client : public Stream
  {
  public:
    virtual int connect(IPAddress ip, uint16_t port)  = 0;
    virtual int connect(const char* host, uint16_t port) = 0;
    virtual size_t write(uint8_t)  = 0;
    virtual size_t write(const uint8_t* buf, size_t size)  = 0;
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int read(uint8_t* buf, size_t size) = 0;
    virtual int peek() = 0;
    virtual void flush() = 0;
    virtual void stop() = 0;
    virtual uint8_t connected() = 0;
    virtual operator bool() = 0;
protected:
    uint8_t* rawIPAddress(IPAddress& addr);
  };
}
```

### <Server.h>

```C++
#include <Server.h>

#include "Print.h"

class Server : public Print
{
  public:
    virtual void begin() = 0;
};
```
### <CanMsg.h>
```C++
#include <CanMsg.h>


#include <inttypes.h>
#include <string.h>

#include "Print.h"
#include "Printable.h"
#include "Common.h"

class CanMsg : public Printable
{
public:
  static uint8_t  constexpr MAX_DATA_LENGTH = 8;

  static uint32_t constexpr CAN_EFF_FLAG    = 0x80000000U;
  static uint32_t constexpr CAN_SFF_MASK    = 0x000007FFU;
  static uint32_t constexpr CAN_EFF_MASK    = 0x1FFFFFFFU;

  CanMsg(uint32_t const can_id, uint8_t const can_data_len, uint8_t const*  can_data_ptr);
  CanMsg();
  CanMsg(CanMsg const& other);
  virtual ~CanMsg();
  CanMsg& operator=(CanMsg const& other);
  virtual size_t printTo(Print& p) const override;
  uint32_t getStandardId() const;
  uint32_t getExtendedId() const;
  bool isStandardId() const;
  bool isExtendedId() const;
uint32_t id;
  uint8_t  data_length;
  uint8_t  data[MAX_DATA_LENGTH];
};
inline uint32_t CanStandardId(uint32_t const id);
inline uint32_t CanExtendedId(uint32_t const id);

```
### <CanMsgRingbuffer.h>
```C++
#include <CanMsgRingbuffer.h>


#include <stdint.h>
#include "CanMsg.h"

class CanMsgRingbuffer
{
public:
  static size_t constexpr RING_BUFFER_SIZE = 32U;

  CanMsgRingbuffer();

  inline bool isFull() const;
  void enqueue(CanMsg const& msg);

  inline bool isEmpty() const;
  CanMsg dequeue();

  inline size_t available() const;

private:
  CanMsg _buf[RING_BUFFER_SIZE];
  volatile size_t _head;
  volatile size_t _tail;
  volatile size_t _num_elems;

  inline size_t next(size_t const idx) const;
};
```

### <USBAPI.h>

```C++
#include <inttypes.h>

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned long u32;

#if defined(USBCON)
#include "USBDesc.h"
#include "USBCore.h"

class USBDevice_
{
public:
	USBDevice_();
	bool configured();

	void attach();
	void detach();
	void poll();
	bool wakeupHost();

	bool isSuspended();
};

extern USBDevice_ USBDevice;

class Serial_ : public Stream
{
private:
	int peek_buffer;
public:
	Serial_();
	void begin(unsigned long);
	void begin(unsigned long, uint8_t);
	void end();

	virtual int available();
	virtual int peek();
	virtual int read();
	virtual int availableForWrite();
	virtual void flush();
	virtual size_t write(uint8_t);
	virtual size_t write(const uint8_t*, size_t);
	using Print::write;
	operator bool();
	volatile uint8_t _rx_buffer_head;
	volatile uint8_t _rx_buffer_tail;
	unsigned char _rx_buffer[SERIAL_BUFFER_SIZE];
	int32_t readBreak();
	uint32_t baud();
	uint8_t stopbits();
	uint8_t paritytype();
	uint8_t numbits();
	bool dtr();
	bool rts();

	enum
  {
		ONE_STOP_BIT = 0,
		ONE_AND_HALF_STOP_BIT = 1,
		TWO_STOP_BITS = 2,
	};
	enum
  {
		NO_PARITY = 0,
		ODD_PARITY = 1,
		EVEN_PARITY = 2,
		MARK_PARITY = 3,
		SPACE_PARITY = 4,
	};
};

extern Serial_ Serial;

//  Low level API
typedef struct
{
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint8_t wValueL;
	uint8_t wValueH;
	uint16_t wIndex;
	uint16_t wLength;
} USBSetup;

//	MSC 'Driver'
int		MSC_GetInterface(uint8_t* interfaceNum);
int		MSC_GetDescriptor(int i);
bool	MSC_Setup(USBSetup& setup);
bool	MSC_Data(uint8_t rx,uint8_t tx);

//	CSC 'Driver'
int		CDC_GetInterface(uint8_t* interfaceNum);
int		CDC_GetDescriptor(int i);
bool	CDC_Setup(USBSetup& setup);


int USB_SendControl(uint8_t flags, const void* d, int len);
int USB_RecvControl(void* d, int len);
int USB_RecvControlLong(void* d, int len);

uint8_t	USB_Available(uint8_t ep);
uint8_t USB_SendSpace(uint8_t ep);
int USB_Send(uint8_t ep, const void* data, int len);
int USB_Recv(uint8_t ep, void* data, int len);
int USB_Recv(uint8_t ep);
void USB_Flush(uint8_t ep);

#endif
```
### <PluggableUSB.h>
```C++
#include <PluggableUSB.h>

#if defined(USBCON)

#include "USBAPI.h"
#include <stdint.h>
#include <stddef.h>

class PluggableUSBModule
{
public:
  PluggableUSBModule(uint8_t numEps, uint8_t numIfs, unsigned int* epType);

protected:
  virtual bool setup(USBSetup& setup) = 0;
  virtual int getInterface(uint8_t* interfaceCount) = 0;
  virtual int getDescriptor(USBSetup& setup) = 0;
  virtual uint8_t getShortName(char* name);

  uint8_t pluggedInterface;
  uint8_t pluggedEndpoint;

  const uint8_t numEndpoints;
  const uint8_t numInterfaces;
  const unsigned int* endpointType;

  PluggableUSBModule* next = NULL;

  friend class PluggableUSB_;
};

class PluggableUSB_
{
public:
  PluggableUSB_();
  bool plug(PluggableUSBModule* node);
  int getInterface(uint8_t* interfaceCount);
  int getDescriptor(USBSetup& setup);
  bool setup(USBSetup& setup);
  void getShortName(char* iSerialNum);

private:
  uint8_t lastIf;
  uint8_t lastEp;
  PluggableUSBModule* rootNode;
  uint8_t totalEP;
};

void* epBuffer(unsigned int n);

PluggableUSB_& PluggableUSB();

#endif
```

### <USBCore.h>

```C++
typedef struct
{
  u8 len;
  u8 dtype;
  u16 usbVersion;
  u8 deviceClass;
  u8 deviceSubClass;
  u8 deviceProtocol;
  u8 packetSize0;
  u16 idVendor;
  u16 idProduct;
  u16 deviceVersion;
  u8 iManufacturer;
  u8 iProduct;
  u8 iSerialNumber;
  u8 bNumConfigurations;
} DeviceDescriptor;

typedef struct
{
  u8 len;
  u8 dtype;
  u16 clen;
  u8 numInterfaces;
  u8 config;
  u8 iconfig;
  u8 attributes;
  u8 maxPower;
} ConfigDescriptor;
  
typedef struct
{
  u8 len;
  u8 dtype;
  u8 number;
  u8 alternate;
  u8 numEndpoints;
  u8 interfaceClass;
  u8 interfaceSubClass;
  u8 protocol;
  u8 iInterface;
} InterfaceDescriptor;

typedef struct
{
  u8 len;
  u8 dtype;
  u8 addr;
  u8 attr;
  u16 packetSize;
  u8 interval;
} EndpointDescriptor;

typedef struct
{
  u8 len;
  u8 dtype;
  u8 firstInterface;
  u8 interfaceCount;
  u8 functionClass;
  u8 funtionSubClass;
  u8 functionProtocol;
  u8 iInterface;
} IADDescriptor;

typedef struct
{
  u8 len;
  u8 dtype;
  u8 subtype;
  u8 d0;
  u8 d1;
} CDCCSInterfaceDescriptor;

typedef struct
{
  u8 len;
  u8 dtype;
  u8 subtype;
  u8 d0;
} CDCCSInterfaceDescriptor4;

typedef struct
{
  u8 len;
  u8 dtype;
  u8 subtype;
  u8 bmCapabilities;
  u8 bDataInterface;
} CMFunctionalDescriptor;

typedef struct
{
  u8 len;
  u8 dtype;
  u8 subtype;
  u8 bmCapabilities;
} ACMFunctionalDescriptor;

typedef struct
{

  IADDescriptor iad;

  InterfaceDescriptor cif;
  CDCCSInterfaceDescriptor header;
  CMFunctionalDescriptor callManagement;
  ACMFunctionalDescriptor controlManagement;
  CDCCSInterfaceDescriptor functionalDescriptor;
  EndpointDescriptor cifin;

  InterfaceDescriptor dif;
  EndpointDescriptor in;
  EndpointDescriptor out;
} CDCDescriptor;

typedef struct
{
  InterfaceDescriptor msc;
  EndpointDescriptor in;
  EndpointDescriptor out;
} MSCDescriptor;
```
### <DMAPool.h>
```C++
#include<DMAPool.h>

#include <atomic>
#include <memory>

namespace arduino
{
  // Single-producer, single-consumer, lock-free bounded Queue.
  template <class T> class SPSCQueue
  {
      private:
          size_t capacity;
          std::atomic<size_t> head;
          std::atomic<size_t> tail;
          std::unique_ptr<T[]> buff;

      public:
          SPSCQueue(size_t size = 0);
          void reset();
          size_t empty();
          operator bool() const;
          bool push(T data);
          T pop(bool peek = false);
  };

  enum
  {
      DMA_BUFFER_READ     = (1 << 0),
      DMA_BUFFER_WRITE    = (1 << 1),
      DMA_BUFFER_DISCONT  = (1 << 2),
      DMA_BUFFER_INTRLVD  = (1 << 3),
  } DMABufferFlags;

  // Forward declaration of DMAPool class.
  template <class, size_t> class DMAPool;

  template <class T, size_t A = std::hardware_constructive_interference_size> class DMABuffer
  {
      private:
          DMAPool<T, A>* pool;
          size_t n_samples;
          size_t n_channels;
          T* ptr;
          uint32_t ts;
          uint32_t flags;

      public:
          DMABuffer(DMAPool<T, A>* pool = nullptr, size_t samples = 0, size_t channels = 0, T* mem = nullptr);
          T* data();
          size_t size();
          size_t bytes();
          void flush();
          void invalidate();
          uint32_t timestamp();
          void timestamp(uint32_t ts);
          uint32_t channels();
          void release();
          void set_flags(uint32_t f);
          bool get_flags(uint32_t f = 0xFFFFFFFFU);
          void clr_flags(uint32_t f = 0xFFFFFFFFU);
          T& operator[](size_t i);
          const T& operator[](size_t i) const;
          operator bool() const;
  };

  template <class T, size_t A = std::hardware_constructive_interference_size> class DMAPool
  {
      private:
          uint8_t* mem;
          bool managed;
          SPSCQueue<DMABuffer<T>*> wqueue;
          SPSCQueue<DMABuffer<T>*> rqueue;
          // Allocates dynamic aligned memory.
          static void* aligned_malloc(size_t size);
          // Frees dynamic aligned memory allocated with aligned_malloc.
          static void aligned_free(void* ptr);

      public:
          DMAPool(size_t n_samples, size_t n_channels, size_t n_buffers, void* mem_in = nullptr);
          ~DMAPool()
          bool writable():
          bool readable();
          void flush();
          DMABuffer<T>* alloc(uint32_t flags);
          void free(DMABuffer<T>* buf, uint32_t flags = 0);
  };
}

using arduino::DMAPool;
using arduino::DMABuffer;
using arduino::SPSCQueue;
```
## Arduino Libraries

### <Servo.h>

```C++
typedef struct
{
  uint8_t nbr        :6;
  uint8_t isActive   :1;
} ServoPin_t;

typedef struct
{
  ServoPin_t Pin;
  volatile unsigned int ticks;
} servo_t;

class Servo
{
public:
  Servo();
  uint8_t attach(int pin);
  uint8_t attach(int pin, int min, int max);
  void detach();
  void write(int value);
  void writeMicroseconds(int value);
  int read();
  int readMicroseconds();
  bool attached();

private:
  uint8_t servoIndex;
  int8_t min;
  int8_t max;
};
```

### <Keybord.h>

```C++
class Keyboard_ : public Print
{
private:
  KeyReport _keyReport;
  const uint8_t* _asciimap;
  void sendReport(KeyReport* keys);
public:
  Keyboard_();
  void begin(const uint8_t* layout = KeyboardLayout_en_US);
  void end();
  size_t write(uint8_t k);
  size_t write(const uint8_t* buffer, size_t size);
  size_t press(uint8_t k);
  size_t release(uint8_t k);
  void releaseAll();
};
```

### <Mouse.h>

```C++
class Mouse_
{
private:
  uint8_t _buttons;
  void buttons(uint8_t b);
public:
  Mouse_();
  void begin();
  void end();
  void click(uint8_t b = MOUSE_LEFT);
  void move(signed char x, signed char y, signed char wheel = 0);
  void press(uint8_t b = MOUSE_LEFT);
  void release(uint8_t b = MOUSE_LEFT);
  bool isPressed(uint8_t b = MOUSE_LEFT);
};
```
### <EEPROM.h>
~~~C++
struct EEPROMClass
{
    // Basic user access methods.
    uint8_t read( int idx );
    void write( int idx, uint8_t val );
    void update( int idx, uint8_t val );
    
    // STL and C++11 iteration capability.
    EEPtr begin();
    EEPtr end();
    uint16_t length();
    
    // Functionality to 'get' and 'put' objects to and from EEPROM.
    template<typename T> T &get( int idx, T &t );
    
    template<typename T> const T &put( int idx, const T &t );
};
~~~
### <HID.h>

```C++
class HIDSubDescriptor
{
public:
  HIDSubDescriptor* next = NULL;
  HIDSubDescriptor(const void* d, const uint16_t l);
  const void* data;
  const uint16_t length;
};

class HID_ : public PluggableUSBModule
{
public:
  HID_();
  int begin();
  int SendReport(uint8_t id, const void* data, int len);
  void AppendDescriptor(HIDSubDescriptor* node);

protected:
  // Implementation of the PluggableUSBModule
  int getInterface(uint8_t* interfaceCount);
  int getDescriptor(USBSetup& setup);
  bool setup(USBSetup& setup);
  uint8_t getShortName(char* name);

private:
  uint8_t epType[1];

  HIDSubDescriptor* rootNode;
  uint16_t descriptorSize;

  uint8_t protocol;
  uint8_t idle;
};
```

### <SoftwareSerial.h>

```C++
constexpr auto SS_MAX_RX_BUFF = 64;

class SoftwareSerial : public Stream
{
private:
  // per object data
  uint8_t _receivePin;
  uint8_t _receiveBitMask;
  volatile uint8_t* _receivePortRegister;
  uint8_t _transmitBitMask;
  volatile uint8_t* _transmitPortRegister;
  volatile uint8_t* _pcint_maskreg;
  uint8_t _pcint_maskvalue;

  // Expressed as 4-cycle delays (must never be 0!)
  uint16_t _rx_delay_centering;
  uint16_t _rx_delay_intrabit;
  uint16_t _rx_delay_stopbit;
  uint16_t _tx_delay;

  uint16_t _buffer_overflow:1;
  uint16_t _inverse_logic:1;

  // static data
  static uint8_t _receive_buffer[SS_MAX_RX_BUFF]; 
  static volatile uint8_t _receive_buffer_tail;
  static volatile uint8_t _receive_buffer_head;
  static SoftwareSerial* active_object;

  // private methods
  [[gnu::always_inline]] inline void recv();
  uint8_t rx_pin_read();
  void setTX(uint8_t transmitPin);
  void setRX(uint8_t receivePin);
  [[gnu::always_inline]] inline void setRxIntMsk(bool enable);

  // Return num - sub, or 1 if the result would be < 1
  static uint16_t subtract_cap(uint16_t num, uint16_t sub);

  // private static method for timing
  static inline void tunedDelay(uint16_t delay);

public:
  // public methods
  SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);
  ~SoftwareSerial();
  void begin(long speed);
  bool listen();
  void end();
  bool isListening();
  bool stopListening();
  bool overflow();
  int peek();

  virtual size_t write(uint8_t byte);
  virtual int read();
  virtual int available();
  virtual void flush();
  operator bool();
  
  using Print::write;

  // public only for easy access by interrupt handlers
  [[gnu::always_inline]] static inline void handle_interrupt();
};

```

### <SPI.h>

```C++
class /* unspecified */
{
  public:
	// Transfer functions
	uint8_t transfer(uint8_t data, /* unspecified */);
	uint16_t transfer16(uint8_t data, /* unspecified */);
	void transfer(void* buf, size_t count, /* unspecified */);

	// Transaction Functions
	void usingInterrupt(uint8_t interruptNumber);
	void beginTransaction(SPISettings settings);
	void endTransaction();

	// SPI Configuration methods
	void attachInterrupt();
	void detachInterrupt();

	void begin();
	void end();

	// Attach/Detach pin to/from SPI controller
	void begin(uint8_t _pin);
	void end(uint8_t _pin);

	// These methods sets a parameter on a single pin
	void setBitOrder(uint8_t _pin, BitOrder);
	void setDataMode(uint8_t _pin, uint8_t);
	void setClockDivider(uint8_t _pin, uint8_t);

	// These methods sets the same parameters but on default pin BOARD_SPI_DEFAULT_SS
	void setBitOrder(BitOrder _order);
	void setDataMode(uint8_t _mode);
	void setClockDivider(uint8_t _div);

  private:
	void init();

  /* unspecified */
};
```

### <Wire.h>

```C++
class TwoWire : /* unspecified */
{
  public:
    TwoWire();
    void begin();
    void begin(uint8_t);
    void begin(int);
    void end();
    void setClock(uint32_t);
    void setWireTimeout(uint32_t timeout = 25000, bool reset_with_timeout = false);
    bool getWireTimeoutFlag(void);
    void clearWireTimeoutFlag(void);
    void beginTransmission(uint8_t);
    void beginTransmission(int);
    uint8_t endTransmission(void);
    uint8_t endTransmission(uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t, uint32_t, uint8_t, uint8_t);
    uint8_t requestFrom(int, int);
    uint8_t requestFrom(int, int, int);
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t* , size_t);
    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    void onReceive(void (*)(int));
    void onRequest(void (*)(void));

    inline size_t write(unsigned long n);
    inline size_t write(long n);
    inline size_t write(unsigned int n);
    inline size_t write(int n);
    using Print::write;

  private:
    /* unspecified */
};
```
###  **WiFi**
### <WiFi.h>
~~~C++
#include "IPAddress.h"
#include "WiFiClient.h"
#include "WiFiServer.h"

#include <inttypes.h>

class WiFiClass
{
private:

    static void init();
public:
    static int16_t 	_state[MAX_SOCK_NUM];
    static uint16_t _server_port[MAX_SOCK_NUM];

    WiFiClass();

    static uint8_t getSocket();

    static char* firmwareVersion();

    int begin(const char* ssid);
    int begin(const char* ssid, uint8_t key_idx, const char* key);
    int begin(const char* ssid, const char* passphrase);

    void config(IPAddress local_ip);
    void config(IPAddress local_ip, IPAddress dns_server);
    void config(IPAddress local_ip, IPAddress dns_server, IPAddress gateway);
    void config(IPAddress local_ip, IPAddress dns_server, IPAddress gateway, IPAddress subnet);

    void setDNS(IPAddress dns_server1);
    void setDNS(IPAddress dns_server1, IPAddress dns_server2);

    int disconnect(void);

    uint8_t* macAddress(uint8_t* mac);

    IPAddress localIP();
    IPAddress subnetMask();

   IPAddress gatewayIP();

    char* SSID();

    uint8_t* BSSID(uint8_t* bssid);

    int32_t RSSI();

    uint8_t	encryptionType();
    int8_t scanNetworks();

    char*	SSID(uint8_t networkItem);

    uint8_t	encryptionType(uint8_t networkItem);

    int32_t RSSI(uint8_t networkItem);

    uint8_t status();

    int hostByName(const char* aHostname, IPAddress& aResult);

    friend class WiFiClient;
    friend class WiFiServer;
};
~~~
### <WiFiServer.h>
~~~C++
#include "Server.h"

class WiFiClient;

class WiFiServer : public Server
{
private:
  uint16_t _port;
  void*     pcb;
public:
  WiFiServer(uint16_t);
  WiFiClient available(uint8_t* status = nullptr);
  void begin();
  virtual size_t write(uint8_t);
  virtual size_t write(const uint8_t* buf, size_t size);
  uint8_t status();

  using Print::write;
};
~~~
~~~C++
#include "Print.h"
#include "Client.h"
#include "IPAddress.h"

class WiFiClient : public Client
{
public:
  WiFiClient();
  WiFiClient(uint8_t sock);

  uint8_t status();
  virtual int connect(IPAddress ip, uint16_t port);
  virtual int connect(const char* host, uint16_t port);
  virtual size_t write(uint8_t);
  virtual size_t write(const uint8_t* buf, size_t size);
  virtual int available();
  virtual int read();
  virtual int read(uint8_t* buf, size_t size);
  virtual int peek();
  virtual void flush();
  virtual void stop();
  virtual uint8_t connected();
  virtual operator bool();

  friend class WiFiServer;

  using Print::write;

private:
  static uint16_t _srcport;
  uint8_t _sock;   //not used
  uint16_t  _socket;

  uint8_t getFirstSocket();
};
~~~
### <WiFiUdp.h>
~~~C++
#define UDP_TX_PACKET_MAX_SIZE 24

class WiFiUDP : public UDP
{
private:
  uint8_t _sock;
  uint16_t _port;

public:
  WiFiUDP();
  virtual uint8_t begin(uint16_t);
  virtual void stop();

  // Sending UDP packets
  
  virtual int beginPacket(IPAddress ip, uint16_t port);
  virtual int beginPacket(const char* host, uint16_t port);
  virtual int endPacket();
  virtual size_t write(uint8_t);
  virtual size_t write(const uint8_t* buffer, size_t size);
  
  using Print::write;

  virtual int parsePacket();
  virtual int available();
  virtual int read();
  virtual int read(unsigned char* buffer, size_t len);
  virtual int read(char* buffer, size_t len);
  virtual int peek();
  virtual void flush();

  virtual IPAddress remoteIP();
  virtual uint16_t remotePort();

  friend class WiFiDrv;
};
~~~
## ArdNativeAPI

### <ArdNative.hpp>

```C++
#include <ArdNative.hpp>

#include <Arduino.h>
#include <ArdConst.h>
#include <ArdMath.hpp>
#include <ArdInit.hpp>
#include <ArdInterrupt.h>
#include <ArdString.hpp>
```

### <ArdConst.h>

```C++
#include <ArdConst.h>

// constants
enum
{
  DEC = 10,
  HEX = 16,
  OCT = 8,
  BIN = 2
};

typedef enum : uint8_t
{
  LOW     = 0,
  HIGH    = 1,
  CHANGE  = 2,
  FALLING = 3,
  RISING  = 4,
} PinStatus;

typedef enum : uint8_t
{
  INPUT            = 0x0,
  OUTPUT           = 0x1,
  INPUT_PULLUP     = 0x2,
  INPUT_PULLDOWN   = 0x3,
  OUTPUT_OPENDRAIN = 0x4,
} PinMode;

constexpr auto LED_BUILTIN = /* unspecified */ ;

typedef enum
{
  LSBFIRST = 0,
  MSBFIRST = 1,
} BitOrder;
```

### <ArdInit.h>

```C
void initCore();
void serialUpdate();
```

### <ArdInterrupt.h>

```C
void interrupts();
void noInterrupts();
uint8_t digitalPinToInterrupt(uint8_t i);
```

### <ArdString.hpp>

```C++
#include <ArdString.hpp>

  class __FlashStringHelper;

  template<const char T[]>
  const __FlashStringHelper* F(T string_literal);
```

### <ArdMath.hpp>

```C++
#include <ArdMath.hpp>

#if defined(__cpp_lib_math_constants)
#include <numbers>
#endif

// math
template<typename T>
constexpr T sq(T a);
template<typename T>
constexpr T round(T x);
template<typename T1, typename T2, typename T3>
constexpr void constrain(T1 amt, T2  low, T3  high);
template<typename T>
constexpr T radians(T rad);
template<typename T>
constexpr T degrees(T deg);

// template overload for map function
template<typename T>
constexpr T map(T x, T in_min, T in_max, T out_min, T out_max);

// bit utilities
template<typename T>
constexpr T bit(T b);
template<typename T>
constexpr uint8_t lowByte(T w);
template<typename T>
constexpr uint8_t highByte(T w);
template<typename T1, typename T2>
constexpr void bitRead(T1&& value, T2&& bit);
template<typename T1, typename T2>
constexpr void bitSet(T1&& value, T2&& bit);
template<typename T1, typename T2>
constexpr void bitClear(T1&& value, T2&& bit);
template<typename T1, typename T2>
constexpr void bitToggle(T1&& value, T2&& bit);
template<typename T1, typename T2, typename T3>
constexpr void bitWrite(T1&& value, T2&& bitvalue, T3&& bit);
```
### <ArdLibs.h>
```C++
#if __has_include(<Mouse.h>)
[[deprecated]]
extern Mouse_ Mouse;
namespace ArdNative
{
  using Mouse = ::Mouse_;
}
#endif // end __has_include

#if __has_include(<SPI.h>)
typedef enum
{
  LSBFIRST = 0,
  MSBFIRST = 1,
};
#endif

#if __has_include(<Keyboard.h>)
[[deprecated]]
extern Keyboard_ Keyboard;
namespace ArdNative
{
  using Keyboard = ::Keyboard_;
}

//  Keyboard
enum : signed char
{
  // Modifiers
  KEY_LEFT_CTRL    = 0x80,
  KEY_LEFT_SHIFT   = 0x81,
  KEY_LEFT_ALT     = 0x82,
  KEY_LEFT_GUI     = 0x83,
  KEY_RIGHT_CTRL   = 0x84,
  KEY_RIGHT_SHIFT  = 0x85,
  KEY_RIGHT_ALT    = 0x86,
  KEY_RIGHT_GUI    = 0x87,

  // Misc keys
  KEY_UP_ARROW     = 0xDA,
  KEY_DOWN_ARROW   = 0xD9,
  KEY_LEFT_ARROW   = 0xD8,
  KEY_RIGHT_ARROW  = 0xD7,
  KEY_BACKSPACE    = 0xB2,
  KEY_TAB          = 0xB3,
  KEY_RETURN       = 0xB0,
  KEY_MENU         = 0xED, // "Keyboard Application" in USB standard
  KEY_ESC          = 0xB1,
  KEY_INSERT       = 0xD1,
  KEY_DELETE       = 0xD4,
  KEY_PAGE_UP      = 0xD3,
  KEY_PAGE_DOWN    = 0xD6,
  KEY_HOME         = 0xD2,
  KEY_END          = 0xD5,
  KEY_CAPS_LOCK    = 0xC1,
  KEY_PRINT_SCREEN = 0xCE, // Print Screen / SysRq
  KEY_SCROLL_LOCK  = 0xCF,
  KEY_PAUSE        = 0xD0, // Pause / Break

  // Numeric keypad
  KEY_NUM_LOCK     = 0xDB,
  KEY_KP_SLASH     = 0xDC,
  KEY_KP_ASTERISK  = 0xDD,
  KEY_KP_MINUS     = 0xDE,
  KEY_KP_PLUS      = 0xDF,
  KEY_KP_ENTER     = 0xE0,
  KEY_KP_1         = 0xE1,
  KEY_KP_2         = 0xE2,
  KEY_KP_3         = 0xE3,
  KEY_KP_4         = 0xE4,
  KEY_KP_5         = 0xE5,
  KEY_KP_6         = 0xE6,
  KEY_KP_7         = 0xE7,
  KEY_KP_8         = 0xE8,
  KEY_KP_9         = 0xE9,
  KEY_KP_0         = 0xEA,
  KEY_KP_DOT       = 0xEB,

  // Function keys
  KEY_F1           = 0xC2,
  KEY_F2           = 0xC3,
  KEY_F3           = 0xC4,
  KEY_F4           = 0xC5,
  KEY_F5           = 0xC6,
  KEY_F6           = 0xC7,
  KEY_F7           = 0xC8,
  KEY_F8           = 0xC9,
  KEY_F9           = 0xCA,
  KEY_F10          = 0xCB,
  KEY_F11          = 0xCC,
  KEY_F12          = 0xCD,
  KEY_F13          = 0xF0,
  KEY_F14          = 0xF1,
  KEY_F15          = 0xF2,
  KEY_F16          = 0xF3,
  KEY_F17          = 0xF4,
  KEY_F18          = 0xF5,
  KEY_F19          = 0xF6,
  KEY_F20          = 0xF7,
  KEY_F21          = 0xF8,
  KEY_F22          = 0xF9,
  KEY_F23          = 0xFA,
  KEY_F24          = 0xFB,
};
#endif // end __has_include

#if __has_include(<Servo.h>)
namespace ArdNative
{
  using Servo = ::Servo;
}
#endif // end __has_include

#if __has_include(<WiFi.h>)
namespace ArdNative
{
  using WiFi = ::WiFiClass;
}
#endif // end __has_include

#if __has_include(<SD.h>)
namespace ArdNative
{
  using SD = ::SDClass;
}
#endif // end __has_include

#if __has_include(<firmata.h>)
namespace ArdNative
{
  using Firmata = firmata::FirmataClass;
}
#endif // end __has_include
#if __has_include(<EEPROM.h>)
namespace ArdNative
{
  using EEPROM = ::EEPROMClass;
}
#endif // end __has_include

#if __has_include(<PDM.h>)
namespace ArdNative
{
  using PDM = ::PDMClass;
}
#endif
```
### <RingBuffer.hpp>

```C++
template <int N>
class RingBuffer
{
  public:
    uint8_t _aucBuffer[N];
    volatile int _iHead;
    volatile int _iTail;
    volatile int _numElems;

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
    inline bool isEmpty() const;
};
```
