# ArdNativeAPI
ArduinoをAPIとして使えるようにする部品集
> [!WARNING]
> まだ開発中で安定していないため、 使用すると **機器本体の故障** を招く可能性があることに注意を喚起する。

## 概要
> [!NOTE]
> この部品集はArduino上級者向けであり、使用する際はC++最新規格に精通している必要がある。

ArdNativeAPIは、通称Arduino言語と呼ばれている状態から切り離して完全なC++として開発できるように設計された部品集であり、Arduinoを純粋なAPIとして呼び出すことができる。現在のところ、対応している標準規格は、C++17である。  
今後、Arduinoで使用されている処理系が更新され次第、随時最新のものに移行する予定である。
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
1. Gitから複製する。
~~~
git clone https://github.com/hmjfk/ArdNativeAPI.git
~~~

2. ArdNativeAPIという名称でzip形式に圧縮する。
3. Arduino IDEを開き、sketch menuから、
~~~
            ︙
-----------------------------
スケッチフォルダを表示      Alt + ctrl + K
ライブラリをインクルード                 ⏵ ライブラリを管理...       ctrl + shift + I
                                            ---------------------------------------------
                                            .ZIP形式のライブラリをインストール…         ←ここ
~~~
を選択して、先ほど保存した場所を選択する。  
4. repoにある`platform.local.txt`の設定を機種ごとに追加する。場所は次の通り。    
macOS: `/Applications/Arduino.app/Contents/Java/hardware/arduino/`  
Windows: `%APPDATA%\..\local\Arduino15\packages\`  
GNU/Linux: `/home/ <User-name> /.arduino15/packages/arduino/hardware`  
## 仕組み
Arduinoでは、`Arduino.h`を明示的に取り込むと、Arduinoが本来行っている翻訳前の事前処理が無効になるという性質がある。本部品集はこの性質に着目して、`Arduino.h`を部品集内で明示的に取り込んだ上で、利用者がmain関数を定義することによって、事前定義されているmain関数を無効化しているのである。
当然、`void setup()`及び`void loop()`の呼出もmain関数に含まれているので、無効にできるのだ。

## 利用許諾
source codeは、GPL 3.0以降で利用を許諾する。また、このrepo内にある文書はCC BY-SA 4.0(LICENCE.DOCSを参照)以降で利用を許諾する。
