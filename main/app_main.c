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

typedef int16_t sample_t;
#include "audio_example_file.h"

static const char *TAG = "app_main";

#define V_REF   1100
#define ADC1_TEST_CHANNEL (ADC1_CHANNEL_7)

//i2s number
#define I2S_NUM           		  (I2S_NUM_0)
//i2s sample rate
#define I2S_SAMPLE_RATE   		  (16000)
//i2s data bits
#define I2S_SAMPLE_BITS   		  (I2S_BITS_PER_SAMPLE_16BIT)
//enable display buffer for debug
#define EXAMPLE_I2S_BUF_DEBUG     (0)
//I2S read buffer length
#define BUFFER_LENGTH      		  (256)
//I2S built-in ADC unit
#define I2S_ADC_UNIT              ADC_UNIT_1
//I2S built-in ADC channel
#define I2S_ADC_CHANNEL           ADC1_CHANNEL_0


/**
 * @brief I2S ADC mode init.
 * Input is set to ADC channel 0, or GPIO32
 * Outputs are:
 * - 25: WS
 * - 26: BCK
 * - 27: Data_Out
 */
void i2s_init(void)
{
     int i2s_num = I2S_NUM;
     i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX, // I2S_MODE_RX |
        .sample_rate =  I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_SAMPLE_BITS,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .intr_alloc_flags = 0,
        .dma_buf_count = 2,
        .dma_buf_len = BUFFER_LENGTH,
        .use_apll = 1,
     };

     //install and start i2s driver
     if (i2s_driver_install(i2s_num, &i2s_config, 0, NULL) != ESP_OK) {
         ESP_LOGE(TAG, "Failed calling i2s_driver_install");
         return;
     }

     //init DAC pad
     if (i2s_set_dac_mode(I2S_DAC_CHANNEL_DISABLE) != ESP_OK) {
     	 ESP_LOGI(TAG, "Failed calling i2s_set_dac_mode");
         return;
     }

     static const i2s_pin_config_t pin_config = {
         .bck_io_num = 26,
         .ws_io_num = 25,
         .data_out_num = 27,
         .data_in_num = I2S_PIN_NO_CHANGE
     };
     if (i2s_set_pin(i2s_num, &pin_config) != ESP_OK) {
     	 ESP_LOGI(TAG, "Failed calling i2s_set_pin");
         return;
     }

     //init ADC pad
     if (i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CHANNEL) != ESP_OK) {
     	 ESP_LOGI(TAG, "Failed calling i2s_set_adc_mode");
         return;
     }
}

/**
 * @brief I2S ADC/DAC example
 */
void i2s_dac_task(void*arg)
{
	size_t bytes_written;

    sample_t *i2s_write_buff = (sample_t*) calloc(BUFFER_LENGTH, sizeof(sample_t));
    if (i2s_write_buff == 0) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        return;
    }

    while (1)
    {
        ESP_LOGI(TAG, "Start of loop");
#if 1 // Audiofile
        int offset = 0;
        const int n_samples =  sizeof audio_table / sizeof audio_table[0];
        while (offset < n_samples)
        {
            int play_len = (n_samples - offset) > (BUFFER_LENGTH) ? (BUFFER_LENGTH) : (n_samples - offset);
            sample_t* p = i2s_write_buff;
            for(int i = 0; i < play_len; i++) {
            	*p++ = audio_table[i + offset];
            	*p++ = audio_table[i + offset];
            }

//            ESP_LOGI(TAG, "Writing to i2s size %d", play_len);
            i2s_write(I2S_NUM, i2s_write_buff, play_len * 2 * sizeof(sample_t), &bytes_written, portMAX_DELAY);
//            ESP_LOGI(TAG, "..done, bytes %d", bytes_written);
            offset += play_len;
        }
#endif

#if 0 // Sawtooth
        for(int i = 0; i < BUFFER_LENGTH; i += 2) {
        	i2s_write_buff[i] = i*(65535 / BUFFER_LENGTH);
        	i2s_write_buff[i+1] = i*(65535 / BUFFER_LENGTH);
        }
        i2s_write(I2S_NUM, i2s_write_buff, BUFFER_LENGTH * sizeof(sample_t), &bytes_written, portMAX_DELAY);
#endif

#if 0
        if (esp_task_wdt_reset() != ESP_OK) {
            ESP_LOGE(TAG, "esp_task_wdt_reset failed");
        }
#endif
        taskYIELD();
    }

    free(i2s_write_buff);
    vTaskDelete(NULL);
}

void adc_read_task(void* arg)
{
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_TEST_CHANNEL, ADC_ATTEN_11db);
    esp_adc_cal_characteristics_t characteristics;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, &characteristics);
    while(1) {
        uint32_t voltage;
        esp_adc_cal_get_voltage(ADC1_TEST_CHANNEL, &characteristics, &voltage);
        ESP_LOGI(TAG, "%d mV", voltage);
        vTaskDelay(200 / portTICK_RATE_MS);
    }
}

esp_err_t app_main(void)
{
    i2s_init();
    esp_log_level_set("I2S", ESP_LOG_INFO);
    xTaskCreate(i2s_dac_task, "i2s_adc_dac", 1024 * 2, NULL, 5, NULL);
// FIXME    xTaskCreate(adc_read_task, "ADC read task", 2048, NULL, 5, NULL);
    return ESP_OK;
}

