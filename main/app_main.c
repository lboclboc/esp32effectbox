#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_spi_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"
//#include "soc/dport_access.h"
#include "soc/dport_reg.h"
#include "soc/apb_ctrl_reg.h"
#include "soc/dac_periph.h"
#include "soc/syscon_periph.h"

typedef int16_t sample_t;
#include "audio_example_file.h"

static const char *TAG = "app_main";

//I2S read buffer length
#define BUFFER_SIZE      		  (1024)

#define SAMPLE_RATE 44100

static void dump_registers();
static void tweak_registers();

/**
 * @brief I2S ADC mode init.
 * Input is set to ADC channel 0, or GPIO32
 * Outputs are:
 * - 25: WS
 * - 26: BCK
 * - 27: Data_Out
 */
int i2s_init_0(void)
{
     esp_err_t rc;

#if 0 // ARDUINO CODE
     i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
      .sample_rate =  I2S_SAMPLE_RATE,              // The format of the signal using ADC_BUILT_IN
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S_MSB,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 4,
      .dma_buf_len = 8,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0
     };
#endif

     // Testlog:
     // - Disabling RX does not help.
     // - Increasing I2S_TX_BCK_DIV_NUM slows the clock, but same number of bits i sent.
     //
     // To test:
     // - Use two I2S controllers I2S0 for ADC and I2S1 for tx
     // - Check for underruns and overruns...

	 // I2S0 ADC Input
     i2s_config_t i2s0_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN,
        .sample_rate =  SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .intr_alloc_flags = 0,
        .dma_buf_count = 2,
        .dma_buf_len = BUFFER_SIZE,
        .use_apll = false,
		.fixed_mclk = 0,
     };

     //install and start i2s driver
     if ((rc = i2s_driver_install(I2S_NUM_0, &i2s0_config, 0, NULL)) != ESP_OK) {
         ESP_LOGE(TAG, "Failed calling i2s_driver_install");
         return rc;
     }

#if 0 // This causes the is2_1 pin outputs to disapear.
     if ((rc = i2s_set_pin(I2S_NUM_0, NULL)) != ESP_OK) {
         ESP_LOGE(TAG, "Failed calling i2s_set_pin");
         return rc;
     }
#endif


     if ((rc = i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0)) != ESP_OK) {
     	 ESP_LOGE(TAG, "Failed calling i2s_set_adc_mode");
         return rc;
     }
     // config adc atten and width
     if ((rc = adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_11db)) != ESP_OK) {
     	 ESP_LOGE(TAG, "Failed calling adc1_config_channel_atten");
         return rc;
     }
     if ((rc =  adc1_config_width(ADC_WIDTH_12Bit)) != ESP_OK) {
     	 ESP_LOGE(TAG, "Failed calling adc1_config_width");
         return rc;
     }

     i2s_adc_enable(I2S_NUM_0);

     return ESP_OK;
}

int i2s_init_1()
{
     esp_err_t rc;

	 // I2S1 I2S Output
     i2s_config_t i2s1_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate =  SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
		// Using I2S_CHANNEL_FMT_RIGHT_LEFT does not fix 1bit problem
        .intr_alloc_flags = 0,
        .dma_buf_count = 2,
        .dma_buf_len = BUFFER_SIZE,
        .use_apll = 1,
		.fixed_mclk = 0,
     };

     //install and start i2s driver
     if ((rc = i2s_driver_install(I2S_NUM_1, &i2s1_config, 0, NULL)) != ESP_OK) {
         ESP_LOGE(TAG, "Failed calling i2s_driver_install");
         return rc;
     }

     static const i2s_pin_config_t pin_config = {
         .bck_io_num = 26,
         .ws_io_num = 25,
         .data_out_num = 27,
         .data_in_num = I2S_PIN_NO_CHANGE
     };

     if ((rc = i2s_set_pin(I2S_NUM_1, &pin_config)) != ESP_OK) {
     	 ESP_LOGE(TAG, "Failed calling i2s_set_pin");
         return rc;
     }

     return ESP_OK;
}

/**
 * @brief I2S ADC/DAC example
 */
void i2s_dac_task(void*arg)
{
	size_t bytes_written = 0;
	size_t bytes_read = 0;

    uint8_t *i2s_buffer = malloc(BUFFER_SIZE);
    if (i2s_buffer == 0) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        return;
    }
    memset(i2s_buffer, 0, BUFFER_SIZE);

    unsigned int i;

    // Fill with square wave data. FIXME: Remove later
	for(i = 0; i < BUFFER_SIZE; i++) {
		i2s_buffer[i] = i; // (i & 4) ? 0x7F : 0x00;
	}
	bytes_read = BUFFER_SIZE;

	while (1)
    {
    	for(i = 0; i < 2000; i++) {
#if 1
    		if (i2s_read(I2S_NUM_0, i2s_buffer, BUFFER_SIZE, &bytes_read, portMAX_DELAY) != ESP_OK) {
				ESP_LOGE(TAG, "failed to read data");
			}
#endif
			if (i2s_write(I2S_NUM_1, i2s_buffer, bytes_read, &bytes_written, portMAX_DELAY) != ESP_OK) {
				ESP_LOGE(TAG, "failed to read data");
			}
    	}
#if 0
    	esp_task_wdt_reset();
    	// Every now and then, debug print sampled data.
		ESP_LOGI(TAG, "Non-zero data:");
    	for(i = 0; i < BUFFER_SIZE; i++) {
    		if (i2s_buffer[i] != 0) {
    	        ESP_LOGI(TAG, "i2s_buffer[%d] = %d, %d, %d", i, i2s_buffer[i], i2s_buffer[i+1], i2s_buffer[i+2]);
    	        break;
    		}
    	}
#endif
    }

    free(i2s_buffer);
    vTaskDelete(NULL);
}

esp_err_t app_main(void)
{
    esp_err_t rc;
    esp_log_level_set("I2S", ESP_LOG_INFO);
    if ((rc = i2s_init_0()) != ESP_OK) {
    	return rc;
    }
    if ((rc = i2s_init_1()) != ESP_OK) {
    	return rc;
    }

    xTaskCreate(i2s_dac_task, "i2s_adc_dac", 1024 * 2, NULL, 5, NULL);
// FIXME    xTaskCreate(adc_read_task, "ADC read task", 2048, NULL, 5, NULL);

    return ESP_OK;
}

void tweak_registers()
{
    // See page 310 in refmanual.
    // No change: REG_SET_FIELD(I2S_CONF_REG(0), I2S_RX_MSB_SHIFT, 1);
    // No change: REG_SET_FIELD(I2S_CONF_REG(0), I2S_TX_MSB_SHIFT, 1);
    // No change: REG_SET_FIELD(I2S_CONF_REG(0), I2S_RX_MSB_SHIFT, 0);
    // No change: REG_SET_FIELD(I2S_CONF_REG(0), I2S_RX_MSB_SHIFT, 0);

    // No change: REG_SET_FIELD(I2S_CLKM_CONF_REG(0), I2S_CLKA_ENA, 1);

    // No Change: REG_SET_FIELD(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_NUM, 14);
    // No Change: REG_SET_FIELD(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_A, 63);
    // No Change: REG_SET_FIELD(I2S_CLKM_CONF_REG(0), I2S_CLKM_DIV_B, 11);
    // No Change:  REG_SET_FIELD(I2S_SAMPLE_RATE_CONF_REG(0), I2S_RX_BCK_DIV_NUM, 8);
    // No Change: REG_SET_FIELD(I2S_SAMPLE_RATE_CONF_REG(0), I2S_TX_BCK_DIV_NUM, 8);
    // No Change: REG_SET_FIELD(I2S_CONF_REG(0), I2S_RX_MSB_SHIFT, 1);
    // No Change: REG_SET_FIELD(I2S_CONF_REG(0), I2S_TX_MSB_SHIFT, 1);

	REG_SET_FIELD(I2S_CONF2_REG(0), I2S_LCD_EN, 0); // <----- Setting this to 0t it become correct clock forms, but only 2.7 Khz sample rate. with mclk=0 its 22KHz
	// No change	REG_SET_FIELD(I2S_CONF_REG(0), I2S_RX_MSB_SHIFT, 1);
	// No change	REG_SET_FIELD(I2S_CONF_REG(0), I2S_TX_MSB_SHIFT, 1);
}

void dump_registers(int i2s_num)
{
    ESP_LOGI(TAG, "I2S_CLKM_CONF_REG: %08X", I2S_CLKM_CONF_REG(i2s_num));
    ESP_LOGI(TAG, "I2S_CLKA_ENA = %d",       REG_GET_FIELD(I2S_CLKM_CONF_REG(i2s_num), I2S_CLKA_ENA));
    ESP_LOGI(TAG, "I2S_CLKM_DIV_NUM = %d",   REG_GET_FIELD(I2S_CLKM_CONF_REG(i2s_num), I2S_CLKM_DIV_NUM));
    ESP_LOGI(TAG, "I2S_CLKM_DIV_A = %d",     REG_GET_FIELD(I2S_CLKM_CONF_REG(i2s_num), I2S_CLKM_DIV_A));
    ESP_LOGI(TAG, "I2S_CLKM_DIV_B = %d",     REG_GET_FIELD(I2S_CLKM_CONF_REG(i2s_num), I2S_CLKM_DIV_B));

    ESP_LOGI(TAG, "I2S_RX_BCK_DIV_NUM = %d", REG_GET_FIELD(I2S_SAMPLE_RATE_CONF_REG(i2s_num), I2S_RX_BCK_DIV_NUM));
    ESP_LOGI(TAG, "I2S_TX_BCK_DIV_NUM = %d", REG_GET_FIELD(I2S_SAMPLE_RATE_CONF_REG(i2s_num), I2S_TX_BCK_DIV_NUM));

    ESP_LOGI(TAG, "I2S_RX_PCM_BYPASS = %d", REG_GET_FIELD(I2S_CONF1_REG(i2s_num), I2S_RX_PCM_BYPASS));
    ESP_LOGI(TAG, "I2S_TX_PCM_BYPASS = %d", REG_GET_FIELD(I2S_CONF1_REG(i2s_num), I2S_TX_PCM_BYPASS));

    ESP_LOGI(TAG, "I2S_LCD_EN = %d", REG_GET_FIELD(I2S_CONF2_REG(i2s_num), I2S_LCD_EN));
	 ESP_LOGI(TAG, "I2S_CAMERA_EN = %d", REG_GET_FIELD(I2S_CONF2_REG(i2s_num), I2S_CAMERA_EN));
    ESP_LOGI(TAG, "I2S_LCD_TX_SDX2_EN = %d", REG_GET_FIELD(I2S_CONF2_REG(i2s_num), I2S_LCD_TX_SDX2_EN));
    ESP_LOGI(TAG, "I2S_LCD_TX_WRX2_EN = %d", REG_GET_FIELD(I2S_CONF2_REG(i2s_num), I2S_LCD_TX_WRX2_EN));


    ESP_LOGI(TAG, "I2S_RX_MSB_SHIFT = %d", REG_GET_FIELD(I2S_CONF_REG(i2s_num), I2S_RX_MSB_SHIFT));
    ESP_LOGI(TAG, "I2S_TX_MSB_SHIFT = %d", REG_GET_FIELD(I2S_CONF_REG(i2s_num), I2S_TX_MSB_SHIFT));
    ESP_LOGI(TAG, "I2S_RX_MSB_RIGHT = %d", REG_GET_FIELD(I2S_CONF_REG(i2s_num), I2S_RX_MSB_RIGHT));
    ESP_LOGI(TAG, "I2S_RX_RIGHT_FIRST = %d", REG_GET_FIELD(I2S_CONF_REG(i2s_num), I2S_RX_RIGHT_FIRST));
    ESP_LOGI(TAG, "I2S_RX_SHORT_SYNC = %d", REG_GET_FIELD(I2S_CONF_REG(i2s_num), I2S_RX_SHORT_SYNC));
    ESP_LOGI(TAG, "I2S_TX_SHORT_SYNC = %d", REG_GET_FIELD(I2S_CONF_REG(i2s_num), I2S_TX_SHORT_SYNC));
    ESP_LOGI(TAG, "I2S_RX_SLAVE_MOD = %d", REG_GET_FIELD(I2S_CONF_REG(i2s_num), I2S_RX_SLAVE_MOD));
    ESP_LOGI(TAG, "I2S_TX_SLAVE_MOD = %d", REG_GET_FIELD(I2S_CONF_REG(i2s_num), I2S_TX_SLAVE_MOD));
    ESP_LOGI(TAG, "I2S_TX_START = %d", REG_GET_FIELD(I2S_CONF_REG(i2s_num), I2S_TX_START));
    ESP_LOGI(TAG, "I2S_RX_START = %d", REG_GET_FIELD(I2S_CONF_REG(i2s_num), I2S_RX_START));
    ESP_LOGI(TAG, "I2S_TX_PDM_HP_BYPASS = %d", REG_GET_FIELD(I2S_CONF_REG(i2s_num), I2S_TX_PDM_HP_BYPASS));
    ESP_LOGI(TAG, "I2S_TX_SLAVE_MOD = %d", REG_GET_FIELD(I2S_CONF_REG(i2s_num), I2S_TX_SLAVE_MOD));

    ESP_LOGI(TAG, "I2S_RX_FIFO_MOD_FORCE_EN = %d", REG_GET_FIELD(I2S_FIFO_CONF_REG(i2s_num), I2S_RX_FIFO_MOD_FORCE_EN));
    ESP_LOGI(TAG, "I2S_TX_FIFO_MOD_FORCE_EN = %d", REG_GET_FIELD(I2S_FIFO_CONF_REG(i2s_num), I2S_TX_FIFO_MOD_FORCE_EN));
    ESP_LOGI(TAG, "I2S_TX_DATA_NUM = %d", REG_GET_FIELD(I2S_FIFO_CONF_REG(i2s_num), I2S_TX_DATA_NUM));
    ESP_LOGI(TAG, "I2S_RX_DATA_NUM = %d", REG_GET_FIELD(I2S_FIFO_CONF_REG(i2s_num), I2S_RX_DATA_NUM));
    ESP_LOGI(TAG, "I2S_TX_FIFO_MOD = %d", REG_GET_FIELD(I2S_FIFO_CONF_REG(i2s_num), I2S_TX_FIFO_MOD));
    ESP_LOGI(TAG, "I2S_TX_CHAN_MOD = %d", REG_GET_FIELD(I2S_FIFO_CONF_REG(i2s_num), I2S_TX_CHAN_MOD));
    ESP_LOGI(TAG, "I2S_RX_FIFO_MOD = %d", REG_GET_FIELD(I2S_FIFO_CONF_REG(i2s_num), I2S_RX_FIFO_MOD));
    ESP_LOGI(TAG, "I2S_RX_CHAN_MOD = %d", REG_GET_FIELD(I2S_FIFO_CONF_REG(i2s_num), I2S_RX_CHAN_MOD));

    ESP_LOGI(TAG, "I2S_TX_PDM_FP = %d", REG_GET_FIELD(I2S_PDM_FREQ_CONF_REG(i2s_num), I2S_TX_PDM_FP));
    ESP_LOGI(TAG, "I2S_TX_PDM_FS = %d", REG_GET_FIELD(I2S_PDM_FREQ_CONF_REG(i2s_num), I2S_TX_PDM_FS));

    	 // Page 317
    ESP_LOGI(TAG, "I2S_TX_PDM_EN = %d", REG_GET_FIELD(I2S_PDM_CONF_REG(i2s_num), I2S_TX_PDM_EN));
    ESP_LOGI(TAG, "I2S_PCM2PDM_CONV_EN = %d", REG_GET_FIELD(I2S_PDM_CONF_REG(i2s_num), I2S_PCM2PDM_CONV_EN));
    ESP_LOGI(TAG, "I2S_TX_I2S_TX_PDM_SIGMADELTA_IN_SHIFTPDM_EN = %d", REG_GET_FIELD(I2S_PDM_CONF_REG(i2s_num), I2S_TX_PDM_SIGMADELTA_IN_SHIFT));
    ESP_LOGI(TAG, "I2S_TX_PDM_SINC_IN_SHIFT = %d", REG_GET_FIELD(I2S_PDM_CONF_REG(i2s_num), I2S_TX_PDM_SINC_IN_SHIFT));
    ESP_LOGI(TAG, "I2S_TX_PDM_LP_IN_SHIFT = %d", REG_GET_FIELD(I2S_PDM_CONF_REG(i2s_num), I2S_TX_PDM_LP_IN_SHIFT));
    ESP_LOGI(TAG, "I2S_TX_PDM_HP_IN_SHIFT = %d", REG_GET_FIELD(I2S_PDM_CONF_REG(i2s_num), I2S_TX_PDM_HP_IN_SHIFT));
    ESP_LOGI(TAG, "I2S_RX_PDM_SINC_DSR_16_EN = %d", REG_GET_FIELD(I2S_PDM_CONF_REG(i2s_num), I2S_RX_PDM_SINC_DSR_16_EN));

    ESP_LOGI(TAG, "I2S_TX_CHAN_MOD = %d", REG_GET_FIELD(I2S_CONF_CHAN_REG(i2s_num), I2S_TX_CHAN_MOD));

    ESP_LOGI(TAG, "APB_CTRL_SARADC_DATA_TO_I2S = %d", REG_GET_FIELD(APB_CTRL_APB_SARADC_CTRL_REG, APB_CTRL_SARADC_DATA_TO_I2S));
}
