# Library Specification

## ArduinoCoreAPI

ここでは、Arduino の Borad 環境を構築した段階で利用可能な部品集について説明する。
Arduino では、これらを CoreAPI と呼んでいる。

### header <Arduino.h>

### Pin I/O Library

#### **pinMode function**

```C++
void pinMode(pin_size_t pinNumber, PinMode pinMode);
```

#### 機能

指定された pin の機能を設定する。

#### 引数

- pinNumber: 対象の pin
- pinMode: pin の機能を指定する。pinMode 型の定数で指定することもできる。

#### 戻り値

なし

---

#### **digitalWrite/Read function**

```C++
void digitalWrite(pin_size_t pinNumber, PinStatus status);
PinStatus digitalRead(pin_size_t pinNumber);
```

#### 機能

digital として読み書きする。

#### 引数

- pinNumber: 対象の端子番号
- status: 書き込む値

#### 戻り値

- digitalWrite 関数: なし
- digitalRead 関数: pinNumber から読み取った pin の値。状態は pinMode 型の定数で表すこともできる。

---

#### **analogReference function**

```C++
void analogReference(uint8_t mode);
```

#### 機能

A/D 変換の基準電圧源を設定する。

> [!Warning]
> 次のいずれかに該当する場合、内部の基準電圧発生源と AREF 端子が短絡して Arduino 内蔵の MCU を破壊する可能性がある。
>
> - AREF 端子を基準電圧源とするとき、analogRead 関数を呼び出すよりも先にこの関数で AREF 端子の使用を指定しなかったとき。
> - 0V~5V 以外の電圧を AREF 端子に加えた場合。

#### 引数

- mode: 基準電圧の種類を表す実装定義の定数

参考までに、基準電圧定数は各機種ごとで次のようになっている。


| 機種            | 定数             | 機能                                                                                                            |
|-----------------|------------------|-----------------------------------------------------------------------------------------------------------------|
| Arduino AVR     | DEFAULT          | 既定の内部基準電圧。 Arduino本体に供給される電源電圧になる。                                                    |
|                 | INTERNAL         | MCU内部の基準電圧。ATmega168 と ATmega238P では 1.1V。ATmega32U4 と ATmega8(Arduino Megaは利用不可)では 2.56V。 |
|                 | INTERNAL1V1      | 内部基準電圧(1.1V)。 　Arduino Mega限定                                                                         |
|                 | INTERNAL2V56     | 内部基準電圧(2.56V)。　Arduino Mega限定                                                                         |
|                 | EXTERNAL         | AREF 端子を使用                                                                                                 |
| Arduino megaAVR | DEFAULT          | 既定の内部基準電圧(0.55V)                                                                                       |
|                 | INTERNAL         | MCU内部の基準電圧(0.55V)                                                                                        |
|                 | VDD              | MCUのVdd電圧を使用。Uno WiFi Rev2とNano Everyでは5.0V                                                           |
|                 | INTERNAL0V55     | 内部基準電圧(0.5V)                                                                                              |
|                 | INTERNAL1V1      | 内部基準電圧(1.1V)                                                                                              |
|                 | INTERNAL1V5      | 内部基準電圧(1.5V)                                                                                              |
|                 | INTERNAL2V5      | 内部基準電圧(2.5V)                                                                                              |
|                 | INTERNAL4V3      | 内部基準電圧(4.3V)                                                                                              |
|                 | EXTERNAL         | AREF 端子を使用                                                                                                 |
| Arduino Renesas | AR_DEFAULT       | 既定の内部基準電圧(5.0V)                                                                                        |
|                 | AR_INTERNAL      | RA4M1の内部基準電圧(1.5V)  UNO R4限定                                                                           |
|                 | AR_INTERNAL_1_5V | A/D変換IC R7FA6M5 の内部基準電圧(1.5V)。　Portenta C33限定                                                      |
|                 | AR_INTERNAL_2_0V | A/D変換IC R7FA6M5 の内部基準電圧(2.0V)。　Portenta C33限定                                                      |
|                 | AR_INTERNAL_2_5V | A/D変換IC R7FA6M5 の内部基準電圧(2.5V)。　Portenta C33限定                                                      |
|                 | AR_EXTERNAL      | AREF 端子を使用                                                                                                 |
| Arduino SAMD    | AR_DEFAULT       | 既定の内部基準電圧(3.3V)                                                                                        |
|                 | AR_INTERNAL1V0   | 内部基準電圧(1.5V)                                                                                              |
|                 | AR_INTERNAL1V65  | 内部基準電圧(2.0V)                                                                                              |
|                 | AR_INTERNAL2V23  | 内部基準電圧(2.5V)                                                                                              |
|                 | AR_EXTERNAL      | AREF 端子を使用                                                                                                 |
| Arduino SAM     | AR_DEFAULT       | 既定の内部基準電圧(3.3V)                                                                                        |
| Arduino SAMD    | AR_VDD           | 既定の内部基準電圧(3.3V)                                                                                        |
|                 | AR_INTERNAL      | MCU内部の基準電圧(0.6V)                                                                                         |
|                 | AR_INTERNAL1V2   | 内部基準電圧(1.2V) AR_INTERNAL の2倍                                                                            |
|                 | AR_INTERNAL2V4   | 内部基準電圧(2.4V) AR_INTERNAL の4倍                                                                            |
|                 |
#### 戻り値

なし

---

#### **analogRead function**

```C++
void analogRead(pin_size_t pinNumber, int value);
```

#### 機能

analogReference 関数及びの設定に基づいてA/D変換を行い値を読み込む。基準電圧の設定がない場合は既定の設定が適用される。

基準電圧として AREF 端子を使用する際、analogReference 関数で AREF 端子の使用を設定せずにこの関数を呼び出したときは、未定義の動作となる。詳しくは、analogReference 関数の警告節を参照のこと。

#### 引数

- pinNumber: 対象の端子番号。通常は`<pins_arduino.h>`で定義される定数を使う。
- value: 書き込む電圧値。

#### 戻り値

pinNumber から読み取った pin の値。

#### **analogWrite function**

```C++
void analogWrite(pin_size_t pinNumber, int value);
```
#### 機能
PWM信号を出力する。
#### 引数

- pinNumber: 対象の端子番号
- arg1 : 衝撃係数

#### 戻り値

なし

---

#### **indfender name**

```C++

```

#### 機能

#### 引数

- arg0 :
- arg1 :

#### 戻り値

なし

---

#### **indfender name**

```C++

```

#### 機能

#### 引数

- arg0 :
- arg1 :

#### 戻り値

なし

---

#### **indfender name**

```C++

```

#### 機能

#### 引数

- arg0 :
- arg1 :

#### 戻り値

なし

---

### <pins_arduino.h>

### <WString.h>

### <Printable.h>

### <Print.h>

### <Stream.h>

### <HardwareSerial.h>

### <IPAddress.h>

### <Udp.h>

### <Client.h>

### <Server.h>

### <CanMsg.h>

### <CanMsgRingbuffer.h>

### <USBAPI.h>

### <PluggableUSB.h>

### <USBCore.h>

## Arduino Libraries

ここでは、Arduino ISE が公式に管理しているか、もしくは代表的な部品集であって、文章化が不十分なものについて説明する。

### <Servo.h>

### <Keybord.h>

### <Mouse.h>

### <EEPROM.h>

### <PDM.h>

### <Firmata.h>

### <HID.h>

### <SoftwareSerial.h>

### <SPI.h>

### <Wire.h>

### <WiFi.h>

### <WiFi101.h>

### <WiFiNINA.h>

### <NTPClient.h>

### <ArduinoGraphics.h>

### <AudioFrequencyMeter.h>

### <MadgwickAHRS.h>

### <LiquidCrystal.h>

### <Arduino_CRC32.h>

---

- ### MIDIUSB
  - ### <MIDIUSB.h>
  - ### <MIDIUSB_Defs.h>

---

- ### USBHost
  - ### <MouseController.h>
  - ### <KeyboardController.h>

---

- ### Ethernet

  - ### <Ethernet.h>
  - ### <EthernetUdp.h>

    この記録体は、次の宣言だけが存在する。

    ```C++
    #include <Ethernet.h>
    ```

---

- ### ArduinoMDNS
  - ### <ArduinoMDNS.h>

    この記録体は、次の宣言だけが存在する。

    ```C++
    #include "MDNS.h"
    ```

  - ### <MDNS.h>

---

- ### ArduinoHttpClient

  - ### <ArduinoHttpClient.h>

    この記録体は、次の宣言だけが存在する。

    ```C++
    #include "HttpClient.h"
    #include "WebSocketClient.h"
    #include "URLEncoder.h"
    ```

  - ### <HttpClient.h>
  - ### <WebSocketClient.h>
  - ### <URLEncoder.h>
  - ### <URLParser.h>

---

- ### ArduinoBLE
- ### Stepper
- ### SD
- ### TFT
- ### ArduinoSound
- ### Audio
- ### Scheduler
- ### ArduinoJson

## ArdNativeAPI

ここからは、本部品集が独自に提供するものについて説明する。

### <ArdNative.hpp>

### <ArdConst.h>

### <ArdInit.h>
#### **initCore function**
```C++
void initCore();
```
#### 機能
CoreAPIを初期化する。
CoreAPIを使用する前にこの関数を呼び出さなかった場合は、未定義動作となる。

---

#### **serialUpdate function**
```C++
void serialUpdate();
```
#### 機能
serial monitorを更新し、buffに書き込まれた内容を描画する。
Serial類集体を操作した後、main関数の無限繰り返し部にてこの関数を呼び出さない場合は、未定義の動作が発生する。
> [!Warning]
> この関数を呼ばなかった場合、ArduinoとArduinoIDEを接続するUSB COM Portが正しく認識しなくなる。
> そのような症状が発生したときは、この関数の使用法に従い、空のmain関数を定義した上で、書込対象の機器にResetをかけ続け、書き込み直前にResetを解除すると解決できる。

### <ArdInterrupt.h>

### <ArdString.hpp>
### <RingBuffer.hpp>
## 著作権について

原著作物の利用許諾に従い、CC BY-SA 4.0 以降でその利用を許諾する。この著作物は、以下に示す原著作物の記述を改善し、より正確な仕様書となるよう編集を行った。

- [Arduino documentation](https://docs.arduino.cc/programming) (C) Arduino 2025  
  The Arduino documentation is licensed under the Creative Commons [Attribution-Share Alike 4.0 license](https://creativecommons.org/licenses/by-sa/4.0/).
- [Arduino ドキュメント](https://garretlab.web.fc2.com/arduino.cc/docs/) by garretlab  
  licensed under the [Attribution-Share Alike 3.0 license](https://creativecommons.org/licenses/by-sa/3.0/).  
  日本語訳+α-β です。
- [Arduino 日本語リファレンス](http://www.musashinodenpa.com/arduino/ref/index.php)　 Creative Commons Attribution-ShareAlike 3.0 License.  
  このドキュメントは[Arduino Team](https://arduino.cc/)により執筆され、[Takumi Funada](http://www.nnar.org/)が翻訳し、一部加筆修正したものです。

# this docs template

## Library name

### <.h>

### XXX Library

#### **indfender name**

```C++

```

#### 機能

#### 引数

- arg0 :
- arg1 :

#### 戻り値

なし
