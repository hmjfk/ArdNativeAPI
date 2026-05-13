# ArdNativeAPI
ArduinoをAPIとして使えるようにする部品集

## 概要
> [!NOTE]
> この部品集はArduino上級者向けであり、使用する際はC++最新規格に精通している必要がある。

ArdNativeAPIは、通称Arduino言語と呼ばれている状態から切り離して完全なC++として開発できるように設計された部品集であり、Arduinoを純粋なAPIとして呼び出すことができる。
ArdNativeAPIを使った場合のhello worldは次のようになる。
~~~C++
#include <ArdNative.hpp>
int main()
{
    initCore();

    Serial.begin(9600);

    Serial.println("hello world");

    while(true)
    {
        serialUpdate();
    }
    return 0;
}
~~~
## API説明
tree内のdocsを参照のこと。
## 新規Code用雛形
ArdNativeAPIを使って新たなSketchを書くのに便利な雛形が用意されている。
init_templateを参照のこと。
## 使用方法
前提条件：環境がPOSIXに対応していること。Windowsの場合はGit Bash環境であること。

1. Arduino用部品集が格納されている場所へ移動する。
~~~
cd ~/Arduino/libraries/
~~~

2. Gitから複製する。
~~~
git clone https://github.com/hmjfk/ArdNativeAPI.git
~~~

3. repoにある`platform.local.txt`の設定を追加する。既定の場所はOSごとに次の通りである。 
Windows: `%APPDATA%\..\local\Arduino15\packages\`  
MacOS: `~/Library/arduino15/packages/arduino/hardware`  
GNU/Linux: `~/.arduino15/packages/arduino/hardware`  
## 仕組み
Arduinoでは、`Arduino.h`を明示的に取り込むと、ArduinoCLIが本来行っている処理が無効になるという性質がある。これは、\[\[gnu::weak\]\]属性の効果や結合器の性質によるものであると考えられる。本部品集はこの性質に着目して、`Arduino.h`を部品集内で明示的に取り込んだ上で、利用者がmain関数を定義することによって、事前定義されているmain関数を無効化している。当然、`void setup()`及び`void loop()`の呼出もmain関数に含まれているので、無効にできるのだ。
一部のboardではこれが機能しないため、付属の`platform.local.txt`を利用して編纂されたmain.cpp.oを削除する必要がある。

## 利用許諾
source codeは、GPL 3.0以降で利用を許諾する。また、このrepo内にある文書はCC BY-SA 4.0(LICENCE.DOCSを参照)以降で利用を許諾する。
