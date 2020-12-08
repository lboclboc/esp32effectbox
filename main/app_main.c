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
#include "audio_example_file.h"
#include "esp_adc_cal.h"
#include "esp_rom_sys.h"

static const char *TAG = "app_main";

#define V_REF   1100
#define ADC1_TEST_CHANNEL (ADC1_CHANNEL_7)

//i2s number
#define I2S_NUM           		  (I2S_NUM_0)
//i2s sample rate
#define I2S_SAMPLE_RATE   		  (44100)
//i2s data bits
#define I2S_SAMPLE_BITS   		  (I2S_BITS_PER_SAMPLE_16BIT)
//enable display buffer for debug
#define EXAMPLE_I2S_BUF_DEBUG     (0)
//I2S read buffer length
#define BUFFER_SIZE      		  (256)
//I2S data format
#define I2S_FORMAT        		  (I2S_CHANNEL_FMT_ONLY_LEFT)
//I2S channel number
#define I2S_CHANNEL_NUM   		  ((I2S_FORMAT < I2S_CHANNEL_FMT_ONLY_RIGHT) ? (2) : (1))
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
        .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
        .intr_alloc_flags = 0,
        .dma_buf_count = 2,
        .dma_buf_len = BUFFER_SIZE,
        .use_apll = 1,
     };

     //install and start i2s driver
     i2s_driver_install(i2s_num, &i2s_config, 0, NULL);

     //init DAC pad
     i2s_set_dac_mode(I2S_DAC_CHANNEL_DISABLE);

     static const i2s_pin_config_t pin_config = {
         .bck_io_num = 26,
         .ws_io_num = 25,
         .data_out_num = 27,
         .data_in_num = I2S_PIN_NO_CHANGE
     };
     i2s_set_pin(i2s_num, &pin_config);

#if 0
     ESP_LOGI(TAG, "Setting sample rate");

     if (ESP_OK != i2s_set_sample_rates(i2s_num, I2S_SAMPLE_RATE)) {
         ESP_LOGI(TAG, "..failed");
     }
     else {
         ESP_LOGI(TAG, "..ok");
     }
#endif

     //init ADC pad
     i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CHANNEL);
}

/**
 * @brief I2S ADC/DAC example
 */
void i2s_dac_task(void*arg)
{
    size_t bytes_written;

    uint8_t* i2s_write_buff = (uint8_t*) calloc(BUFFER_SIZE * 8, sizeof(char));

    while (1)
    {
        ESP_LOGI(TAG, "Start of loop");
        int offset = 0;
        while (offset < sizeof(audio_table))
        {
            int play_len = ((sizeof(audio_table) - offset) > (BUFFER_SIZE)) ? (BUFFER_SIZE) : (sizeof(audio_table) - offset);
            uint8_t* p = i2s_write_buff;
            for(int i = 0; i < play_len; i++) {
            	*p++ = 0;
            	*p++ = ((signed char )audio_table[i + offset]) / 8;
            	*p++ = 0;
            	*p++ = ((signed char )audio_table[i + offset]) / 8;
            	*p++ = 0;
            	*p++ = ((signed char )audio_table[i + offset]) / 8;
            	*p++ = 0;
            	*p++ = ((signed char )audio_table[i + offset]) / 8;
            }

            ESP_LOGI(TAG, "Writing to i2s size %d", play_len);
            i2s_write(I2S_NUM, i2s_write_buff, play_len * 8, &bytes_written, portMAX_DELAY);
            ESP_LOGI(TAG, "..done");
            offset += play_len;
        }
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

