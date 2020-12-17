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


### Analysis of duplex clock error:

- Working (ADC disabled):

```
I (327) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (337) I2S: DMA Malloc info, datalen=blocksize=256, dma_buf_count=2
I (347) I2S: DMA Malloc info, datalen=blocksize=256, dma_buf_count=2
I (347) I2S: PLL_D2: Req RATE: 44100, real rate: 44642.000, BITS: 16, CLKM: 14, BCK: 8, MCLK: 11289966.924, SCLK: 1428544.000000, diva: 64, divb: 11
I (367) app_main: I2S_CLKM_CONF_REG(0): 3FF4F0AC
I (367) app_main: I2S_CLKA_ENA(0) = 0
I (377) app_main: I2S_CLKM_DIV_NUM(0) = 14
I (377) app_main: I2S_CLKM_DIV_A(0) = 63
I (387) app_main: I2S_CLKM_DIV_B(0) = 11
I (387) app_main: I2S_RX_BCK_DIV_NUM(0) = 8
I (397) app_main: I2S_TX_BCK_DIV_NUM(0) = 8
I (397) app_main: I2S_RX_PCM_BYPASS(0) = 1
I (407) app_main: I2S_TX_PCM_BYPASS(0) = 1
I (407) app_main: I2S_LCD_EN(0) = 0
I (417) app_main: I2S_CAMERA_EN(0) = 0
I (417) app_main: I2S_LCD_TX_SDX2_EN(0) = 0
I (417) app_main: I2S_LCD_TX_WRX2_EN(0) = 0
I (427) app_main: I2S_RX_MSB_SHIFT(0) = 1
I (427) app_main: I2S_TX_MSB_SHIFT(0) = 1
I (437) app_main: I2S_RX_MSB_RIGHT(0) = 0
I (437) app_main: I2S_RX_RIGHT_FIRST(0) = 0
I (447) app_main: I2S_RX_SHORT_SYNC(0) = 0
I (447) app_main: I2S_TX_SHORT_SYNC(0) = 0
I (457) app_main: I2S_RX_SLAVE_MOD(0) = 1
I (457) app_main: I2S_TX_SLAVE_MOD(0) = 0
I (467) app_main: I2S_TX_START(0) = 1
I (467) app_main: I2S_RX_START(0) = 1
I (477) app_main: I2S_TX_PDM_HP_BYPASS(0) = 0
I (477) app_main: I2S_TX_SLAVE_MOD(0) = 0
I (487) app_main: I2S_RX_FIFO_MOD_FORCE_EN(0) = 1
I (487) app_main: I2S_TX_FIFO_MOD_FORCE_EN(0) = 1
I (497) app_main: I2S_TX_DATA_NUM(0) = 32
I (497) app_main: I2S_RX_DATA_NUM(0) = 32
I (507) app_main: I2S_TX_FIFO_MOD(0) = 1
I (507) app_main: I2S_TX_CHAN_MOD(0) = 0
I (507) app_main: I2S_RX_FIFO_MOD(0) = 1
I (517) app_main: I2S_RX_CHAN_MOD(0) = 0
I (517) app_main: I2S_TX_PDM_FP(0) = 960
I (527) app_main: I2S_TX_PDM_FS(0) = 441
I (527) app_main: I2S_TX_PDM_EN(0) = 0
I (537) app_main: I2S_PCM2PDM_CONV_EN(0) = 0
I (537) app_main: I2S_TX_I2S_TX_PDM_SIGMADELTA_IN_SHIFTPDM_EN(0) = 1
I (547) app_main: I2S_TX_PDM_SINC_IN_SHIFT(0) = 1
I (557) app_main: I2S_TX_PDM_LP_IN_SHIFT(0) = 1
I (557) app_main: I2S_TX_PDM_HP_IN_SHIFT(0) = 1
I (567) app_main: I2S_RX_PDM_SINC_DSR_16_EN(0) = 1
I (567) app_main: I2S_TX_CHAN_MOD(0) = 1
I (577) app_main: APB_CTRL_SARADC_DATA_TO_I2S = 0
```

- Not Working (ADC enabled):

```
I (327) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (337) I2S: DMA Malloc info, datalen=blocksize=256, dma_buf_count=2
I (347) I2S: DMA Malloc info, datalen=blocksize=256, dma_buf_count=2
I (347) I2S: PLL_D2: Req RATE: 44100, real rate: 2777.000, BITS: 16, CLKM: 30, BCK: 60, MCLK: 30.234, SCLK: 88864.000000, diva: 64, divb: 14
I (367) app_main: I2S_CLKM_CONF_REG(0): 3FF4F0AC
I (367) app_main: I2S_CLKA_ENA(0) = 0
I (377) app_main: I2S_CLKM_DIV_NUM(0) = 14
I (377) app_main: I2S_CLKM_DIV_A(0) = 63
I (387) app_main: I2S_CLKM_DIV_B(0) = 11
I (387) app_main: I2S_RX_BCK_DIV_NUM(0) = 8
I (397) app_main: I2S_TX_BCK_DIV_NUM(0) = 8
I (397) app_main: I2S_RX_PCM_BYPASS(0) = 1
I (407) app_main: I2S_TX_PCM_BYPASS(0) = 1
I (407) app_main: I2S_LCD_EN(0) = 1
I (407) app_main: I2S_CAMERA_EN(0) = 0
I (417) app_main: I2S_LCD_TX_SDX2_EN(0) = 0
I (417) app_main: I2S_LCD_TX_WRX2_EN(0) = 0
I (427) app_main: I2S_RX_MSB_SHIFT(0) = 1
I (427) app_main: I2S_TX_MSB_SHIFT(0) = 1
I (437) app_main: I2S_RX_MSB_RIGHT(0) = 0
I (437) app_main: I2S_RX_RIGHT_FIRST(0) = 0
I (447) app_main: I2S_RX_SHORT_SYNC(0) = 0
I (447) app_main: I2S_TX_SHORT_SYNC(0) = 0
I (457) app_main: I2S_RX_SLAVE_MOD(0) = 1
I (457) app_main: I2S_TX_SLAVE_MOD(0) = 0
I (467) app_main: I2S_TX_START(0) = 1
I (467) app_main: I2S_RX_START(0) = 1
I (477) app_main: I2S_TX_PDM_HP_BYPASS(0) = 0
I (477) app_main: I2S_TX_SLAVE_MOD(0) = 0
I (477) app_main: I2S_RX_FIFO_MOD_FORCE_EN(0) = 1
I (487) app_main: I2S_TX_FIFO_MOD_FORCE_EN(0) = 1
I (497) app_main: I2S_TX_DATA_NUM(0) = 32
I (497) app_main: I2S_RX_DATA_NUM(0) = 32
I (507) app_main: I2S_TX_FIFO_MOD(0) = 1
I (507) app_main: I2S_TX_CHAN_MOD(0) = 0
I (507) app_main: I2S_RX_FIFO_MOD(0) = 1
I (517) app_main: I2S_RX_CHAN_MOD(0) = 0
I (517) app_main: I2S_TX_PDM_FP(0) = 960
I (527) app_main: I2S_TX_PDM_FS(0) = 441
I (527) app_main: I2S_TX_PDM_EN(0) = 0
I (537) app_main: I2S_PCM2PDM_CONV_EN(0) = 0
I (537) app_main: I2S_TX_I2S_TX_PDM_SIGMADELTA_IN_SHIFTPDM_EN(0) = 1
I (547) app_main: I2S_TX_PDM_SINC_IN_SHIFT(0) = 1
I (547) app_main: I2S_TX_PDM_LP_IN_SHIFT(0) = 1
I (557) app_main: I2S_TX_PDM_HP_IN_SHIFT(0) = 1
I (557) app_main: I2S_RX_PDM_SINC_DSR_16_EN(0) = 1
I (567) app_main: I2S_TX_CHAN_MOD(0) = 1
I (567) app_main: APB_CTRL_SARADC_DATA_TO_I2S = 0
```