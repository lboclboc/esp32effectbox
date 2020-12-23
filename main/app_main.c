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

typedef int32_t value_t; // Value type used for functions and history buffer.
typedef uint16_t sample_t;
#include "audio_example_file.h"

#define ASSERT(val, msg) if(!(val)) { ESP_LOGE(TAG, "assert failed: " #msg); }
static const char *TAG = "app_main";

#define BUFFER_SIZE      		  (256)

#define SAMPLE_RATE 44100
#define OVERSAMPLING 5

#define FIXED_DECIMAL_ONE 1000  // Used for all betas and other fractions below
#define FIXED_DECIMAL(x) ( (int)((x) *  FIXED_DECIMAL_ONE) )

// Beta of FIXED_DECIMAL(1.0) returns only v1, rest is mix
#define MIX(v1, v2, beta) ( ((v1) * beta + (FIXED_DECIMAL_ONE - beta) * (v2)) / FIXED_DECIMAL_ONE)

// Filtering
#undef OS_LP
#define OS_MEDIAN

#define LP_BETA FIXED_DECIMAL(0.2)

#define DC_BETA FIXED_DECIMAL(0.99)
#define RMS_BETA FIXED_DECIMAL(0.99)
#define HISTORY_LENGTH (1<<14)
#define HISTORY_MASK (HISTORY_LENGTH - 1)

// Effects

// Echo
#define EF_ECHO
#define ECHO_LENGTH 10000
#define ECHO_FEEDBACK  FIXED_DECIMAL(0.5)
#define ECHO_DRY FIXED_DECIMAL(0.7)
#if ECHO_LENGTH >= HISTORY_LENGTH
#error "To large echo size"
#endif

// Flanger
#define EF_FLANGER // Requires EF_ECHO
#define FLANGER_LENGTH 1000
#if FLANGER_LENGTH >= HISTORY_LENGTH
#error "To large flanger length"
#endif
#define FLANGER_DRY FIXED_DECIMAL(0.9)

extern void tweak_registers();
extern void dump_registers(int i2s_num);

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
static int cmp_samples(const void *p1, const void *p2)
{
	if (*(sample_t *)p1 > *(sample_t *)p2) return 1;
	else if (*(sample_t *)p1 < *(sample_t *)p2) return -1;
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

    unsigned short history_pos = 0;
    value_t *history = calloc(HISTORY_LENGTH, sizeof (value_t));
    if (history == 0) {
        ESP_LOGE(TAG, "Failed to allocate echo memory");
         return;
    }

#define GET_SAMPLE() (integral += ((*sample_in) << 4) + INT16_MIN, ((int16_t)(*sample_in++) << 4) + INT16_MIN - dc_offset)

    sample_t *sample_in;
    sample_t *sample_out;
    value_t value = 0;

#if defined(OS_LP) || defined(EF_FLANGER)
    value_t v;
#endif
#if defined(OS_LP)
    value_t lp_value = 0;
#endif
#ifdef OS_MEDIAN
    value_t oversample_buffer[OVERSAMPLING];
#endif

    unsigned short osample;
    uint64_t ms = 0; // Signal level measured over each cycle
    value_t dc_offset = -3030;
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
    			// - Filtering/oversampling -> value -
#ifdef OS_LP
    			for(osample = 0; osample < OVERSAMPLING; osample++) {
    				v = GET_SAMPLE();
    				lp_value = MIX(lp_value, v, LP_BETA);
    			}
    			value = lp_value;
#endif

#ifdef OS_MEDIAN
    			for(osample = 0; osample < OVERSAMPLING; osample++) {
    				oversample_buffer[osample] = GET_SAMPLE();
    			}
    			qsort((void *)oversample_buffer, OVERSAMPLING, sizeof(oversample_buffer[0]), cmp_samples);
    			value = (oversample_buffer[OVERSAMPLING / 2] + oversample_buffer[OVERSAMPLING / 2 + 1]) / 2;
#endif

    			// - History and RMS meassurement -
    			ms = MIX(ms, value*value, RMS_BETA);

    			history_pos = (history_pos + 1) & HISTORY_MASK;
    			history[history_pos] = value;

    			// - Effects -
#ifdef EF_ECHO
    			v = history[(history_pos - ECHO_LENGTH) & HISTORY_MASK];
    			history[history_pos] += MIX(v, 0, ECHO_FEEDBACK);
    			value =  MIX(value, v, ECHO_DRY);
#endif

#ifdef EF_FLANGER
    			v = value * history[(history_pos - FLANGER_LENGTH) & HISTORY_MASK] / 65536;
    			value = MIX(value, v, FLANGER_DRY);
#endif

    			if (value > INT16_MAX) {
    				value = INT16_MAX;
    			}
    			else if (value < INT16_MIN) {
    				value = INT16_MIN;
    			}
//value = 0;

    			*sample_out++ = value; // Note I2S uses two-complement signed data.
    		}

    		ASSERT((sample_out - i2s_buffer) >= (BUFFER_SIZE), "no enough data written");
    		ASSERT((sample_out - i2s_buffer) <= (BUFFER_SIZE), "too much data written");
			rc = i2s_write(I2S_NUM_1, i2s_buffer, (sample_out - i2s_buffer) * sizeof(sample_t), &bytes_written, portMAX_DELAY);
			ASSERT(rc == ESP_OK, "failed to read data");

			dc_offset = (dc_offset * DC_BETA + (integral / (BUFFER_SIZE * OVERSAMPLING)) * (FIXED_DECIMAL_ONE - DC_BETA)) / FIXED_DECIMAL_ONE;
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

    ESP_LOGI(TAG, "I2S0 settings:");
    dump_registers(0);
    ESP_LOGI(TAG, "I2S1 settings:");
	dump_registers(1);

    xTaskCreate(i2s_dac_task, "i2s_adc_dac", 1024 * 2, NULL, 5, NULL);

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
