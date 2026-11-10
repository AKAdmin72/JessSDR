# JessSDR — проверенные на железе прошивки

Восстановить версию только для Tang Nano 20K (без ESP32) одной командой:
`./flash_tang_only` (или `./flash_tang_only sram` — временно, до выключения питания).
Исходники этой версии — ветка `tang-only` и тег `working-2026-11-07`.

Восстановить любую прошивку без пересборки:

    openFPGALoader -b tangnano20k -f releases/<файл>.fs
    esptool --chip esp32s3 -p /dev/ttyACM0 write-flash 0x0 releases/<файл>.bin   # ESP32-S3 (после . ~/esp/esp-idf/export.sh)

У версий с ESP32 два файла (`_tang.fs` и `_esp32.bin`), прошивать оба.

Восстановить исходники: `git checkout <тег>`, затем `cd sdr && ./build` (версия только Tang) или `cd sdr_esp && ./build` и `cd esp32 && ./build.sh` (с ESP32).

| Файл | Git-тег | Состояние |
|---|---|---|
| `2026-11-07_tang-only_sdr-v4.fs` | `working-2026-11-07` | Работает только Tang, ESP32 не нужен. Пользователь подтвердил после переделки AD8138 (+25 дБ): «приём намного лучше», с ДПФ 40 м и без LNA. АЦП 64.8 MSPS → DDC → SSB (LSB/USB) → AGC → встроенный MAX98357A (динамик на Tang). Дисплей ILI9341 на Tang (CS 71, RST/DC/MOSI/SCK 18/19/20/17): частота, S-метр (GAIN_DB 25), спектр ±33 кГц + водопад с автопорогом шума (floor_eq), масштаб 1/1.5/2 px/дБ. Кнопки S1/S2: ±шаг, шаг, LSB/USB. UART 1 Мбод: `tools/sdr.py`. sha256 2ff1038f…f787 |
| `2026-11-10_esp-ui-v5_tang.fs` + `2026-11-10_esp-ui-v5_esp32.bin` | `working-2026-11-10` | Первая версия с ESP32-S3. Пользователь подтвердил: «кнопки работают». Самотест связи при старте ESP весь PASS (SPI, LO, режимы, I2S). Tang (`sdr_esp/`, VERSION 5): DDC → SSB/AM → AGC → NR/ANF → MAX98357A; spi_regs (SCK 53, MISO 71, MOSI 79, CS 80), широкий I/Q 105.54 кГц по I2S на ESP (BCK 41, WS 72, DATA 42); TFT/FFT/водопад с Tang убраны (BSRAM 18/46). Кнопки S1/S2 вместо энкодера (регистр BTN 0x16). ESP: 4" ST7796S на 80 МГц SPI, 35 кадров/с, спектр + водопад, меню; после каждого обмена SPI гасит WS2812 на пине 79. sha256 ebd15aa5… / ec8a57f6… |
