# main関数が起動するまで
ここでは、main関数を起動させるにあたって特別な処理が必要な機種につき、その手順を解説した資料を示す。
## ESP32
- [ESP-IDF Programing Guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/startup.html)
- [Source](https://github.com/espressif/esp-idf/)
    1. components/esp_system/port/cpu_start.c (EntryPoint: call_start_cpu0)
    2. components/esp_system/include/esp_private/startup_internal.h
    3. components/esp_system/startup.c
    4. components/freertos/app_startup.c
    - esp_cpu_get_core_id()
        - components/esp_hw_support/include/esp_cpu.h
        - components/xtensa/include/xt_utils.h
        - components/riscv/include/riscv/rv_utils.h
        - components/riscv/include/riscv/csr.h