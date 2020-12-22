#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
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
#include "soc/dport_reg.h"
#include "soc/apb_ctrl_reg.h"
#include "soc/dac_periph.h"
#include "soc/syscon_periph.h"

typedef uint16_t sample_t;
#include "audio_example_file.h"

#define ASSERT(val, msg) if(!(val)) { ESP_LOGE(TAG, "assert failed: " #msg); }
static const char *TAG = "app_main";

#define BUFFER_SIZE      		  (256)

#define SAMPLE_RATE 44100
#define OVERSAMPLING 5

// Effects active
#undef EF_ECHO

// Filtering
#undef OS_SIGMA
#undef OS_LP
#define OS_MEDIAN

#define LP_BETA 0.8

#define DC_BETA 0.99
#define RMS_BETA 0.999

// TODO: Use DMA-buffers for filtering instead of the echo-buffer.

#if 0
static void dump_registers();
static void tweak_registers();
#endif

int i2s_init_0(void)
{
     esp_err_t rc;

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
        .sample_rate =  SAMPLE_RATE * OVERSAMPLING,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 3,
        .dma_buf_len = BUFFER_SIZE,
	    .tx_desc_auto_clear = false,
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
     if ((rc = adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0)) != ESP_OK) {
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
        .dma_buf_count = 3,
        .dma_buf_len = BUFFER_SIZE,
        .use_apll = true,
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

#ifdef OS_MEDIAN
static int cmp_samples(sample_t *p1, sample_t *p2)
{
	if (*p1 > *p2) return 1;
	else if (*p1 < *p2) return -1;
	else return 0;
}
#endif

/**
 * @brief Main loop
 */
void i2s_dac_task(void*arg)
{
	size_t bytes_written = 0;
	size_t bytes_read = 0;
    esp_err_t rc;

	sample_t *i2s_buffer = calloc(BUFFER_SIZE * OVERSAMPLING, sizeof(sample_t));
    if (i2s_buffer == 0) {
        ESP_LOGE(TAG, "Failed to allocate buffer memory");
        return;
    }

#ifdef EF_ECHO
    unsigned short echo_pos = 0;
    int echo_size = 1<<13;
    int echo_mask = echo_size - 1;
    sample_t *echo_buffer = calloc(echo_size, sizeof (int32_t));
    if (echo_buffer == 0) {
        ESP_LOGE(TAG, "Failed to allocate echo memory");
         return;
    }
#endif

#define GET_SAMPLE() (integral += ((*sample_in) << 4) - 32768, ((int16_t)(*sample_in++) << 4) - 32768 - dc_offset)

    sample_t *sample_in;
    sample_t *sample_out;
    int32_t value;
#if defined(OS_SIGMA) || defined(OS_LP)
    int32_t v;
#endif
#ifdef OS_SIGMA
    int32_t high, low;
#endif
#ifdef OS_MEDIAN
    int32_t oversample_buffer[OVERSAMPLING];
#endif
    unsigned short osample;
    uint64_t ms = 0; // Signal level measured over each cycle
    int32_t dc_offset = -3030;
    int64_t integral = 0;
    int intervall = 0;

	while (1)
    {
    	for(intervall = 0; intervall < 200; intervall++)
    	{
    		rc = i2s_read(I2S_NUM_0, i2s_buffer, BUFFER_SIZE * OVERSAMPLING * sizeof(sample_t), &bytes_read, portMAX_DELAY);
    		ASSERT(rc == ESP_OK, "failed to read data");
    		ASSERT(bytes_read == BUFFER_SIZE * OVERSAMPLING * 2, "wrong data size read");

    		integral = 0;
    		for(sample_out = sample_in = i2s_buffer; sample_in < (i2s_buffer + BUFFER_SIZE * OVERSAMPLING); /* No increment */)
    		{

#ifdef OS_SIGMA
    			value = 0;
    			high = 0;
    			low = 32768;
       			for(osample = 0; osample < OVERSAMPLING; osample++) {
        				v = GET_SAMPLE();
        				if (v > high) high = v;
        				if (v < low) low = v;
        				value += v;
       			}
    			if (OVERSAMPLING >= 3) {
    				value -= high + low;
    				value /= OVERSAMPLING - 2;
    			}
    			else {
    				value /= OVERSAMPLING;
    			}
#endif

#ifdef OS_LP
    			value = GET_SAMPLE();
    			for(osample = 1; osample < OVERSAMPLING; osample++) {
    				v = GET_SAMPLE();
//    				value = value * LP_BETA + v * (1.0-LP_BETA);
    				value = value / 2 + v / 2;
    			}
#endif

#ifdef OS_MEDIAN
    			for(osample = 0; osample < OVERSAMPLING; osample++) {
    				oversample_buffer[osample] = GET_SAMPLE();
    			}
    			qsort(oversample_buffer, OVERSAMPLING, sizeof(oversample_buffer[0]), cmp_samples);
    			value = (oversample_buffer[OVERSAMPLING / 2] + oversample_buffer[OVERSAMPLING / 2 + 1]) / 2;
#endif

#ifdef EF_ECHO
    			// Echo
    			value += echo_buffer[echo_pos] * 0.4;

    			// IIR-filtering.
     			value +=  -echo_buffer[(echo_pos - 10) & echo_mask] * 0.0
    					  +echo_buffer[(echo_pos - 25) & echo_mask] * 0.0
						  ;

    			echo_buffer[echo_pos] = value;
    			echo_pos = (echo_pos + 1) & echo_mask;
#endif

    			if (value > 32767) {
    				value = 32767;
    			}
    			else if (value < -32768) {
    				value = -32768;
    			}
//value = 0;
    			ms = ms * RMS_BETA + value * value * (1.0 - RMS_BETA);
    			*sample_out++ = value; // Note I2S uses two-complement signed data.
    		}

    		ASSERT((sample_out - i2s_buffer) >= (BUFFER_SIZE), "no enough data written");
    		ASSERT((sample_out - i2s_buffer) <= (BUFFER_SIZE), "too much data written");
			rc = i2s_write(I2S_NUM_1, i2s_buffer, (sample_out - i2s_buffer) * sizeof(sample_t), &bytes_written, portMAX_DELAY);
			ASSERT(rc == ESP_OK, "failed to read data");

			dc_offset = dc_offset * DC_BETA + (integral / (BUFFER_SIZE * OVERSAMPLING)) * (1.0 - DC_BETA);
    	}
    	ESP_LOGI(TAG, "Signal rms = %f (dc-offset %d)", sqrt(ms), dc_offset);
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

    return ESP_OK;
}

#if 0

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
#endif
