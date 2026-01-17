## 今後の実装計画について
現在はC++17を対象としているが、[CPPETL](https://www.etlcpp.com/home.html)との整合性を考慮しつつ将来のC++規格対応を見据える必要がある。それまでの間やるべきことは、次のとおりである。

- 部品集仕様書の作成
- AVR以外のBoard対応
- libc++及び[CPPETL](https://www.etlcpp.com/home.html)統合時に不足する機能の洗い出し。
