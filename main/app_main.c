#include <stdio.h>
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

typedef int16_t sample_t;
#include "audio_example_file.h"

// Scale up from 12->16 bit and make signed.
#define ADC_TO_SIGNED(x) (((x) << 4) - 32768)

static const char *TAG = "app_main";

#define BUFFER_SIZE      		  (512)

#define SAMPLE_RATE 44100
#define OVERSAMPLING 1

#undef EF_ECHO

#undef OS_SIGMA_KAPPA
#define OS_LP
#define LP_BETA 0.8


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

/**
 * @brief Main loop
 */
void i2s_dac_task(void*arg)
{
	size_t bytes_written = 0;
	size_t bytes_read = 0;

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

//    unsigned short i;
    unsigned short sample_in;
    unsigned short sample_out;
    int32_t v, value;
#ifdef OS_SIGMA_KAPPA
    int32_t high, low;
#endif
    unsigned short osample;
    uint64_t ms = 0; // Signal level measured over each buffer
    int intervall = 0;

	while (1)
    {
    	for(intervall = 0; intervall < 200; intervall++)
    	{
    		if (i2s_read(I2S_NUM_0, i2s_buffer, BUFFER_SIZE * OVERSAMPLING * sizeof(sample_t), &bytes_read, portMAX_DELAY) != ESP_OK) {
				ESP_LOGE(TAG, "failed to read data");
			}

    		sample_out = 0;

    		for(sample_in = 0; sample_in < BUFFER_SIZE * OVERSAMPLING; sample_in += OVERSAMPLING)
    		{
#ifdef OS_SIGMA_KAPPA
    			value = 0;
    			high = 0;
    			low = 32768;
       			for(osample = 0; osample < OVERSAMPLING; osample++) {
        				v = ADC_TO_SIGNED(i2s_buffer[sample_in + osample]);
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
    			value = ADC_TO_SIGNED(i2s_buffer[sample_in]);
    			for(osample = 1; osample < OVERSAMPLING; osample++) {
    				v = ADC_TO_SIGNED(i2s_buffer[sample_in + osample]);

    				value = value * LP_BETA + v * (1.0-LP_BETA);
    			}
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
    			ms = ms * 0.999 + value * value * 0.001;

    			i2s_buffer[sample_out++] = value; // Note I2S uses two-complement signed data.
    		}

			if (i2s_write(I2S_NUM_1, i2s_buffer, sample_out * sizeof(sample_t), &bytes_written, portMAX_DELAY) != ESP_OK) {
				ESP_LOGE(TAG, "failed to read data");
			}
    	}
    	ESP_LOGI(TAG, "Signal rms = %f", sqrt(ms));
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
