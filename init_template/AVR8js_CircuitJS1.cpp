// 変更が多すぎて手動前処理するのが面倒なので、個別で前処理器にやらせること。
#include <ArdNative.hpp>

int main()
{
  initCore(); // don't erase here. Arduino API System Init code.

  // your setup code.


  while(true)
  {
    // your main code.

    // Uncomment the following line when using the Serial class.
    // serialUpdate();
  }
  return 0;
}
