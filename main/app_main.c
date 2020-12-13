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
#include "soc/dac_periph.h"
#include "soc/syscon_periph.h"

typedef int16_t sample_t;
#include "audio_example_file.h"

static const char *TAG = "app_main";

//I2S read buffer length
#define BUFFER_SIZE      		  (512)

#define SAMPLE_RATE 44100


/**
 * @brief I2S ADC mode init.
 * Input is set to ADC channel 0, or GPIO32
 * Outputs are:
 * - 25: WS
 * - 26: BCK
 * - 27: Data_Out
 */
int i2s_init(void)
{
     esp_err_t rc;

#if 0 // 6086: Does not fix 1-bit size error.
     // powerdown APLL/PLLA
     SET_PERI_REG_MASK(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_PLLA_FORCE_PD_M);
     CLEAR_PERI_REG_MASK(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_PLLA_FORCE_PU_M);

     // config adc atten and width
     adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_11db);
     adc1_config_width(ADC_WIDTH_12Bit);

     // set pattern tbl and invert, might not need, double check
     SYSCON.saradc_ctrl.sar1_patt_len = 1;
     SYSCON.saradc_sar1_patt_tab[0] = 0b00001111000011110000111100001111;
     SYSCON.saradc_ctrl2.sar1_inv = 1;
#endif

	 // I2S DAC Output
     i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN,
        .sample_rate =  SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .intr_alloc_flags = 0,
        .dma_buf_count = 2,
        .dma_buf_len = BUFFER_SIZE,
        .use_apll = 1,
		.fixed_mclk = 0,
     };

     //install and start i2s driver
     if ((rc = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL)) != ESP_OK) {
         ESP_LOGE(TAG, "Failed calling i2s_driver_install");
         return rc;
     }

     static const i2s_pin_config_t pin_config = {
         .bck_io_num = 26,
         .ws_io_num = 25,
         .data_out_num = 27,
         .data_in_num = I2S_PIN_NO_CHANGE
     };

     if ((rc = i2s_set_pin(I2S_NUM_0, &pin_config)) != ESP_OK) {
     	 ESP_LOGE(TAG, "Failed calling i2s_set_pin");
         return rc;
     }

     if ((rc = i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0)) != ESP_OK) {
     	 ESP_LOGE(TAG, "Failed calling i2s_set_adc_mode");
         return rc;
     }

// Does not fix 1-bit size error:  vTaskDelay(5000/portTICK_RATE_MS); i2s_adc_enable(I2S_NUM_0);
// Does not fix 1-bit size error: i2s_set_sample_rates(I2S_NUM_0, SAMPLE_RATE);
// Does not fix 1-bit size error: i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
// Does not fix 1-bit size error: adc1_config_width(ADC_WIDTH_BIT_12);
// Does not fix 1-bit size error: i2s_adc_enable(I2S_NUM_0);

#if 0 // 6086: Does not fix 1-bit size error.
     // powerup APLL/PLLA
     SET_PERI_REG_MASK(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_PLLA_FORCE_PU_M);
     CLEAR_PERI_REG_MASK(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_PLLA_FORCE_PD_M);
#endif

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

    unsigned short i;
#if 0
    // Fill with square wave data.
	for(i = 0; i < BUFFER_SIZE; i++) {
		i2s_buffer[i] = (i & 8) ? 0x7FFF : 0x0000;
	}
#endif

	while (1)
    {
    	for(i = 0; i < 200; i++) {
// Does not fix 1-bit size error: i2s_adc_enable(I2S_NUM_0); - also creates enormous lagging
			i2s_read(I2S_NUM_0, i2s_buffer, BUFFER_SIZE, &bytes_read, portMAX_DELAY);
// Does not fix 1-bit size error: i2s_adc_disable(I2S_NUM_0);
// Does not fix 1-bit size error: Filling the buffer with data as loop above.
			i2s_write(I2S_NUM_0, i2s_buffer, bytes_read, &bytes_written, portMAX_DELAY);
    	}
    	// Every now and then, debug print sampled data.
		ESP_LOGI(TAG, "Non-zero data:");
    	for(i = 0; i < BUFFER_SIZE; i++) {
    		if (i2s_buffer[i] != 0) {
    	        ESP_LOGI(TAG, "i2s_buffer[%d] = %d, %d, %d", i, i2s_buffer[i], i2s_buffer[i+1], i2s_buffer[i+2]);
    	        break;
    		}
    	}
    }

    free(i2s_buffer);
    vTaskDelete(NULL);
}

esp_err_t app_main(void)
{
    esp_err_t rc;
    esp_log_level_set("I2S", ESP_LOG_INFO);
    if ((rc = i2s_init()) != ESP_OK) {
    	return rc;
    }

    xTaskCreate(i2s_dac_task, "i2s_adc_dac", 1024 * 2, NULL, 5, NULL);
// FIXME    xTaskCreate(adc_read_task, "ADC read task", 2048, NULL, 5, NULL);

    return ESP_OK;
}

