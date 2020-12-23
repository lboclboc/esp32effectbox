# ESP32EffectBox
This is a simple guitar box effect box built upon the ESP32 using its internal 12bit ADC and sending audio out through I2S.
The pins used for I2S is BCLK = GPIO26, WS = GPIO25 and DATA = GPIO27.

The analog input, if connecting from a passive instrument, will require some pre-amplifier with a gain about 20dB. The input also needs a rather sharp a low pass filter for frequencies above 20KHz - depending of oversampling used.

# Setup eclipse
This project is preferably build with eclipse CDT with the expressif plugin installed.

- Install expressif eclipse plugin: https://github.com/espressif/idf-eclipse-plugin/blob/master/README.md

# Serial Monitor
- Click terminal icon in eclipse

# References
- [Espressif I2S](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-reference/peripherals/i2s.html)
- [ESP8266Audio](https://github.com/earlephilhower/ESP8266Audio)
- [ESP32 Technical Reference](https://espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf#page=306)
- [Adafruit I2S Stereo Decoder - UDA1334A Breakout](https://www.adafruit.com/product/3678) ([datasheet](https://www.nxp.com/docs/en/data-sheet/UDA1334ATS.pdf))


### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

### Configure SDK
cd build; cmake --build . -t menuconfig

## TODO
- Possibly use DMA-buffers for filtering instead of the echo-buffer.

