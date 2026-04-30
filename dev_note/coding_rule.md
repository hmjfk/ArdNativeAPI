# 開発作法規約
ArdNativeAPIは、組み込み環境が前提である。そのため、実行記録体やRAMの容量をできるだけ節約し、軽量化することが必要となってくる。
ここでは、こうした要求を実現するためにやるべきことを規定する。

1. 適用可能な最新のC++及びC言語規格に準拠すること。現時点ではC++17とC11が適用できる。
2. 基本的な規約は、次による。
    1. [SEI CERT C Coding Standard](https://wiki.sei.cmu.edu/confluence/spaces/c/pages/87152044/SEI+CERT+C+Coding+Standard)及び[SEI CERT C++ Coding Standard](https://cmu-sei.github.io/secure-coding-standards/sei-cert-cpp-coding-standard)を適用する。C言語版の参考訳は[JPCERT/CC](https://www.jpcert.or.jp/sc-rules/)が公開している。この規約に違反しない限り「Recommendations」規範にも従うこと。
    2. 規定の一部を変更した上で、C++及びC言語にも共通する部分は[The D Style](https://dlang.org/dstyle.html)を適用する。これは、Phobosの追加要件も含まれる。
        1. 変更して適用する部分は次の通り。
            - Naming Conventions
                - Keywordsはそのまま適用し、ここに明示がない部分は全て削除。
        2. Phobosの追加要件で削除する部分は次の通り。
            - Whitespace  
                ~~Put a space after for, foreach, if, while, and version:~~
            - Declarations
                - ~~Invariants should use the expression-based syntax when the equivalent long-form syntax would have a single assert statement. Put a space between invariant and the opening parentheses~~:
    3.  英語による説明文書の書き方は[ASD-STE100](https://www.asd-ste100.org/assets/files/ASD-STE100_ISSUE9.pdf)に従う。
3. 命名規則は単語省略記法やcamel法でも構わないが、一つの機能群や処理のまとまりごとにできるだけ統一すること。たとえば、文字列を表示する関数群や集成体、headerを単位にするとよい。また、3つ以上の単語を連結したsnake法は使用しないこと。文字数上限は概ね10文字程度とする。例外としてMacro名には適用しない。
> [!NOTE]
> これらの命名規則は、宗教論争になりがちなので統一感を損ねない程度で実装者の好みに任せることにした。また、頭文字を単語として用いる場合は、文字の大小を変更しないほうが見やすいことがある。参考までに、C言語の標準部品集やPOSIXでは、古くから母音省略形、短縮形の単語省略記法が用いられている。

4. 識別子の使用にあたっては、処理系の最適化を考慮し、局所性を高めること。大域変数はABIに影響がない限り、禁止する。
5. できるだけ構造化プログラミングや関数型プログラミングにおける参照透過性の概念を中心とすること。もし、他の考え方で書く場合は、C言語に落とし込めるような最低限の使用にどどめる。どうしても依存せざるを得ない場合は、実行記録体やRAM消費量の大きさに対する影響度合いに注意する。inline関数の扱いも同様とする。
6. 移植性を保つため、**asm**文や名前修飾のベタ書き(hard coding)は禁止する。高級言語で表現不能な場合は、LLVM IRを使用すること。
7. 言語処理系と部品集が癒着した言語機能を使用してはならない。具体的には、RTTIや例外、coroutineなどである。
8. 前処理器が言語の意味に影響を与えてはならない。**#define**で定義された名前が、前処理指令以外の文脈で現れる場合はこれに該当する。型に依存しない記述にするには総称型を使用すること。
9. 可読性を高くするように心がけること。たとえば、一箇所でしか呼び出されない関数や、逆に一つの関数や処理、宣言をまとめて書けるものを繰り返し書いているなど、いちいち回りくどい実装になっていないこと。関数を定義するかどうかの基準は、ASM言語の従処理がつくられた原点に戻って、同じ処理をまとめるという目的に従うものとする。内部実装は、詳細な実装の構造や仕組みを省略せずに文章化して残しておくこと。
10. headerの宣言順は、規格又はlibsに記載された通りとする。そのようになっていない部分が見つかった場合は適時直しておくこと。

## 参考文献
- [The D Style](https://dlang.org/dstyle.html)