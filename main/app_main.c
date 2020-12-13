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
#define BUFFER_LENGTH      		  (512)


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
	 // I2S DAC Output
     i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN,
        .sample_rate =  44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .intr_alloc_flags = 0,
        .dma_buf_count = 2,
        .dma_buf_len = BUFFER_LENGTH,
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

     if ((rc = i2s_start(I2S_NUM_0)) != ESP_OK) {
    	 ESP_LOGE(TAG, "failed to start i2s");
    	 return rc;
     }

     if ((rc = i2s_adc_enable(I2S_NUM_0)) != ESP_OK) {
    	 ESP_LOGE(TAG, "failed enable adc");
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

    size_t buffer_size = BUFFER_LENGTH * sizeof(sample_t);

    sample_t *i2s_buffer = (sample_t*) calloc(BUFFER_LENGTH, sizeof(sample_t));

    if (i2s_buffer == 0) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        return;
    }

    unsigned short i;
	for(i = 0; i < BUFFER_LENGTH; i++) {
		i2s_buffer[i] = (i & 8) ? 0x7FFF : 0x0000;
	}
	bytes_read = buffer_size;

    while (1)
    {
    	for(i = 0; i < 200; i++) {
			i2s_read(I2S_NUM_0, i2s_buffer, buffer_size, &bytes_read, portMAX_DELAY);
			i2s_write(I2S_NUM_0, i2s_buffer, bytes_read, &bytes_written, portMAX_DELAY);
    	}
		ESP_LOGI(TAG, "Non-zero data:");
    	for(i = 0; i < BUFFER_LENGTH; i++) {
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

